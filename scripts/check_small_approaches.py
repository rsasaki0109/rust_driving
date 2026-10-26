#!/usr/bin/env python3
"""Four-case closed-loop canary for explicitly bounded smaller checked approaches."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import copy
import check_later_stops as parent
from check_forecast_calibration import read_retained
from inspect_stall_conflicts import CANONICAL
from check_track_initialization import compare

KEY='checked_approach_min_progress_m'
MINIMUM=.5

def sha(raw):
    return hashlib.sha256(raw).hexdigest()

def motion_metrics(ticks):
    moving=False;departures=[];settles=[];side=0;switches=0;emergency=False;entries=0
    for tick in ticks:
        out=tick['expected'];speed=out['estimate']['speed']
        if not moving and speed>=.5:
            moving=True;departures.append(out['time'])
        elif moving and speed<=.2:
            moving=False;settles.append(out['time'])
        target=out['trajectory']['lateral_target']
        new_side=1 if target>.1 else -1 if target<-.1 else 0
        if new_side and side and new_side!=side:switches+=1
        if new_side:side=new_side
        if out['emergency'] and not emergency:entries+=1
        emergency=out['emergency']
    return dict(measured_departures_s=departures,measured_settles_s=settles,
        nonzero_lateral_side_changes=switches,emergency_entries=entries,
        definition='Measured-speed hysteresis: depart >=0.5 m/s, settle <=0.2 m/s; includes initial departure.')

def contract(row):
    return (not row.get('audit_error') and row.get('replay_exit_code')==0
        and row.get('replay',{}).get('verified') is True
        and row['replay']['ticks']==row['summary']['steps']
        and row.get('run_exit_code')==(0 if row['summary']['passed'] else 1)
        and all(row.get(k,{}).get('passed') is True for k in
            ['pose_occupancy','cold_motion','recent_velocity_hypotheses','checked_approach','stopping_certificates','directional_certificates']))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ['evidence','output','binary','native-binary']:parser.add_argument('--'+key,required=True,type=Path)
    args=parser.parse_args()
    raw=CANONICAL.read_bytes()
    if (args.evidence/'report.json').read_bytes()!=raw:raise ValueError('wrong original qualified evidence')
    original=json.loads(raw)
    if not original['passed'] or not original['full_qualification_complete']:raise ValueError('incomplete baseline qualification')
    output=args.output.absolute()
    if output.exists() or output.is_symlink() or any(p.is_symlink() for p in output.parents):raise ValueError('new real output directory required')
    output.mkdir(parents=True)
    specs=parent.specifications(['reference-2d','rne-dynamic'])
    baseline_manifest=parent.freeze(output/'baseline',specs,factory=parent.materialize,gates=parent.GATES)
    old_rows=original['cases']
    if len(old_rows)!=48 or len(baseline_manifest['episodes'])!=48:raise ValueError('original cohort changed')
    for spec,row in zip(baseline_manifest['episodes'],old_rows):
        if any(spec[k]!=row[k] for k in ['scenario','sha256','plant','world','mode','seed']):
            raise ValueError('baseline frozen input changed')
    selected=[(spec,row) for spec,row in zip(baseline_manifest['episodes'],old_rows) if spec['world']=='mission']
    if len(selected)!=4 or {(s['plant'],s['seed']) for s,_ in selected}!={(p,n) for p in ['reference-2d','rne-dynamic'] for n in [7,42]}:
        raise ValueError('wrong original mission set')
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    fingerprints={p:sha(b.read_bytes()) for p,b in binaries.items()}
    report=dict(source=os.environ['ANALYSIS_SOURCE'],minimum_progress_m=MINIMUM,
        original_report_sha256=sha(raw),closed_loop=True,default_policy_changed=False,
        full_cohort_qualified=False,adoption_passed=False,complete=False,contract_passed=False,cases=[])
    def save():
        (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    save()
    def one(pair):
        spec,old=pair
        folder=lambda root:root/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        def audit_baseline(run,config,ticks,s,frozen):
            if config.get(KEY) is not None:raise ValueError('baseline unexpectedly opted in')
            result=parent.audit(run,config,ticks,dict(s,_folder=str(folder(output/'baseline'))),frozen,verify_directions=True)
            result['motion_metrics']=motion_metrics(ticks)
            return result
        control=parent.run_case(output/'baseline',spec,binaries[spec['plant']],binaries['reference-2d'],auditor=audit_baseline)
        if not contract(control):raise ValueError('baseline contract failed: '+repr(control.get('audit_error')))
        identical=all(control['storage'][k]['sha256']==old['storage'][k]['sha256']
                      for k in ['run/run.json','run/sensors.jsonl','run/summary.json'])
        if not identical:raise ValueError('absent-option baseline bytes differ')
        raw_control=read_retained(output/'baseline',control,'run/sensors.jsonl')
        original_config=json.loads(raw_control.splitlines()[0])['header']['config']
        frozen=json.loads((output/'baseline'/spec['scenario']).read_bytes())
        if KEY in frozen:raise ValueError('original input already sets progress floor')
        changed=copy.deepcopy(frozen);changed[KEY]=MINIMUM
        path=output/'new'/spec['scenario'];path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('x') as stream:stream.write(json.dumps(changed,indent=2)+'\n')
        new_spec=dict(spec,sha256=sha(path.read_bytes()))
        def audit_new(run,config,ticks,s,candidate):
            expected=dict(original_config,**{KEY:MINIMUM})
            if config!=expected:raise ValueError('another driver setting changed')
            stripped=dict(candidate);value=stripped.pop(KEY)
            if value!=MINIMUM or stripped!=frozen:raise ValueError('physics, sensing or original mission changed')
            result=parent.audit(run,config,ticks,dict(s,_folder=str(folder(output/'new'))),frozen,verify_directions=True)
            result['motion_metrics']=motion_metrics(ticks)
            return result
        candidate=parent.run_case(output/'new',new_spec,binaries[spec['plant']],binaries['reference-2d'],auditor=audit_new)
        return dict(plant=spec['plant'],seed=spec['seed'],default_bytes_identical=identical,
            baseline=control,candidate=candidate,comparison=compare(candidate,old))
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,selected):
            report['cases'].append(row);save()
            print(json.dumps(dict(plant=row['plant'],seed=row['seed'],
                before=row['baseline']['summary'],after=row['candidate'].get('summary'),
                audit_error=row['candidate'].get('audit_error'),
                comparison=row['comparison'],motion=row['candidate'].get('motion_metrics'),
                checked_stops=len(row['candidate'].get('checked_approach',{}).get('adoptions',[]))),allow_nan=False),flush=True)
    if fingerprints!={p:sha(b.read_bytes()) for p,b in binaries.items()}:raise ValueError('binary changed during experiment')
    report.update(complete=len(report['cases'])==4,
        contract_passed=all(contract(r['candidate']) and r['default_bytes_identical'] for r in report['cases']),
        no_new_failed_gates=all(r['comparison']['regression_passed'] for r in report['cases']),
        arrivals=sum(r['candidate'].get('summary',{}).get('reached_goal',False) for r in report['cases']))
    save()
    print(json.dumps({k:report[k] for k in ['complete','contract_passed','no_new_failed_gates','arrivals','adoption_passed']}),flush=True)
    raise SystemExit(0 if report['complete'] and report['contract_passed'] else 1)

if __name__=='__main__':main()
