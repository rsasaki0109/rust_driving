#!/usr/bin/env python3
"""Paired unchanged-input regressions for conservative directional reserves."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import check_later_stops as parent
from directional_reserve_oracle import certify

ROOT=Path(__file__).resolve().parents[1]
BASELINE='390e3147c94b81bd34e0269ab4671b1729e574e8'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def streams(folder):
    return {str(p.relative_to(folder)):sha(p) for p in sorted(folder.rglob('*')) if p.is_file()}

def byte_controls(output, old, new, specs):
    folder=output/'controls';folder.mkdir()
    first=next(s for s in specs if s['plant']=='reference-2d' and s['world']=='mission' and s['seed']==7)
    _,bounded=parent.materialize(first)
    bounded['replan_checked_approaches']=False
    bounded['search_later_obstruction_stops']=False
    inputs=[('default-mission',json.loads((ROOT/'scenarios/mission.json').read_text())),
            ('envelope-without-checked-approaches',bounded)]
    prepared=[]
    for name,value in inputs:
        target=folder/(name+'.json')
        target.write_bytes((json.dumps(value,indent=2,allow_nan=False)+'\n').encode())
        prepared.append((name,target,sha(target)))
    rows=[]
    for name,source,digest in prepared:
        executions=[]
        for label,binary in [('old',old),('new',new)]:
            work=folder/'work'
            result=subprocess.run([str(binary),'run','--scenario',str(source),'--seed','7','--output',str(work/'run')],
                capture_output=True,text=True)
            if result.returncode not in (0,1):raise ValueError(result.stderr)
            replay=subprocess.run([str(binary),'replay','--log',str(work/'run/sensors.jsonl'),'--output',str(work/'replay')],
                capture_output=True,text=True)
            if replay.returncode!=0 or json.loads((work/'replay/replay.json').read_text())['verified'] is not True:
                raise ValueError('negative-control sensor replay failed')
            executions.append(dict(name=label,exit_code=result.returncode,streams=streams(work/'run')))
            work.rename(folder/(name+'-'+label))
        rows.append(dict(name=name,input_sha256=digest,executions=executions,
            identical=executions[0]['streams']==executions[1]['streams']
                and executions[0]['exit_code']==executions[1]['exit_code']))
    return dict(passed=all(r['identical'] for r in rows),cases=rows)

def contract(row):
    return (not row.get('audit_error') and row.get('replay_exit_code')==0
        and row.get('replay',{}).get('verified') is True
        and row['replay']['ticks']==row.get('summary',{}).get('steps')
        and row.get('run_exit_code')==(0 if row.get('summary',{}).get('passed') else 1))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--baseline-binary',type=Path,required=True)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--baseline-native',type=Path,required=True)
    parser.add_argument('--native-binary',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.absolute()
    if output.exists() or output.is_symlink() or any(p.is_symlink() for p in output.parents):
        raise ValueError('results need a new path with real parents')
    output.mkdir(parents=True)
    binaries={label:{'reference-2d':b.resolve(),'rne-dynamic':n.resolve()}
        for label,b,n in [('old',args.baseline_binary,args.baseline_native),('new',args.binary,args.native_binary)]}
    fingerprints={label:{plant:sha(path) for plant,path in values.items()} for label,values in binaries.items()}
    specs=parent.specifications(['reference-2d','rne-dynamic'])
    if len(specs)!=48:raise ValueError('the complete original 48-case cohort changed')
    manifests={label:parent.freeze(output/label,specs,factory=parent.materialize,gates=parent.GATES) for label in ['old','new']}
    if manifests['old']!=manifests['new']:raise ValueError('source-only comparison changed frozen inputs')
    frozen_inputs={s['scenario']:dict(sha256=s['sha256'],input=json.loads((output/'old'/s['scenario']).read_text()))
        for s in manifests['old']['episodes']}

    report=dict(schema_version=1,source_commit=os.environ['RUSTDRIVING_TESTED_SOURCE'],baseline_source=BASELINE,
        protocol_sha256=sha(ROOT/'integrations/planning/directional-reserve-v1.json'),
        binaries=fingerprints,frozen_inputs=frozen_inputs,manifest_sha256={label:sha(output/label/'frozen-manifest.json') for label in ['old','new']},preliminary=True,qualified=False,default_policy_changed=False,
        fresh_holdout=False,actor_truth_used_for_operational_decisions=False,
        complete=False,passed=False,executions={},comparisons=[])
    target=output/'report.json'
    def save():
        target.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    report['byte_controls']=byte_controls(output,binaries['old']['reference-2d'],binaries['new']['reference-2d'],specs)
    save()
    for label in ['old','new']:
        destination=output/label
        selected=manifests[label]['episodes']
        def one(item):
            index,spec=item
            folder=destination/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
            def audit(run,config,ticks,s,frozen):
                result=parent.audit(run,config,ticks,dict(s,_folder=str(folder)),frozen)
                flips=sum(a['expected']['emergency']!=b['expected']['emergency'] for a,b in zip(ticks,ticks[1:])
                    if not a['expected']['health'] and not b['expected']['health'])
                result['healthy_emergency_transitions']=flips
                if label=='new':result['directional_certificates']=certify(config,ticks)
                return result
            row=parent.run_case(destination,spec,binaries[label][spec['plant']],binaries[label]['reference-2d'],auditor=audit)
            receipts=destination/'episode-receipts';receipts.mkdir(exist_ok=True)
            (receipts/f'{index:03d}.json').write_text(json.dumps(row,indent=2,allow_nan=False)+'\n')
            return row
        rows=[]
        report['executions'][label]=rows
        with ThreadPoolExecutor(max_workers=2) as pool:
            for row in pool.map(one,enumerate(selected)):
                rows.append(row);save()
                print(label,row['plant'],row['world'],row['mode'],row['seed'],
                    'goal',row.get('summary',{}).get('reached_goal'),
                    'emergency',row.get('summary',{}).get('emergency_steps'),
                    'audit',row.get('audit_error'),'gates',row.get('failed_gates'),flush=True)
    before,after=report['executions']['old'],report['executions']['new']
    for old,new in zip(before,after):
        identity=['scenario','sha256','plant','world','mode','seed']
        if any(old[k]!=new[k] for k in identity):raise ValueError('comparison case identity changed')
        comparison=(parent.compare(new,old) if contract(old) and contract(new) else
            dict(regression_passed=False,new_failed_gates=['incomplete_or_invalid_case']))
        report['comparisons'].append(dict(scenario=new['scenario'],plant=new['plant'],world=new['world'],seed=new['seed'],
            **comparison,old_summary=old.get('summary'),new_summary=new.get('summary'),
            old_failed_gates=old.get('failed_gates'),new_failed_gates_all=new.get('failed_gates'),
            old_healthy_emergency_transitions=old.get('healthy_emergency_transitions'),
            new_healthy_emergency_transitions=new.get('healthy_emergency_transitions')))
    first=next(i for i,r in enumerate(after) if r['plant']=='reference-2d' and r['world']=='mission' and r['seed']==7)
    stable=(contract(before[first]) and contract(after[first])
        and after[first]['summary']['emergency_steps']<=before[first]['summary']['emergency_steps']
        and after[first]['healthy_emergency_transitions']<=before[first]['healthy_emergency_transitions'])
    report.update(complete=len(before)==len(after)==48,
        contract_passed=all(contract(r) for r in before+after),
        directional_certificates_passed=all(r.get('directional_certificates',{}).get('passed') is True for r in after),
        regression_passed=all(c['regression_passed'] for c in report['comparisons']),
        first_reference_mission_stability_passed=stable,
        strict_passes={label:sum(r.get('passed',False) for r in rows) for label,rows in report['executions'].items()},
        arrivals={label:sum(r.get('summary',{}).get('reached_goal',False) for r in rows) for label,rows in report['executions'].items()})
    report['passed']=(report['complete'] and report['byte_controls']['passed'] and report['contract_passed']
        and report['directional_certificates_passed'] and report['regression_passed'] and stable
        and report['arrivals']['new']>=report['arrivals']['old'])
    report['qualified']=report['passed']
    report['original_full_adoption_gate_passed']=report['passed'] and report['strict_passes']['new']==48
    for label,values in binaries.items():
        if fingerprints[label]!={plant:sha(path) for plant,path in values.items()}:raise ValueError('binary changed during comparison')
    save()
    print(json.dumps({k:report[k] for k in ['complete','passed','contract_passed','directional_certificates_passed',
        'regression_passed','first_reference_mission_stability_passed','strict_passes','arrivals','original_full_adoption_gate_passed']}),flush=True)
    raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':main()
