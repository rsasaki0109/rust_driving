#!/usr/bin/env python3
"""Unchanged-source controls and a viewed first reference contingency canary."""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_later_stops as parent
from contingency_oracle import reconstruct
from inspect_planning import audit as inspect

ROOT=Path(__file__).resolve().parents[1]
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
def hashes(folder):
    return {str(p.relative_to(folder)):sha(p) for p in sorted(folder.rglob('*')) if p.is_file()}

def main():
    folder=ROOT/'artifacts/contingency-canary'
    folder.mkdir(parents=True,exist_ok=False)
    target=ROOT/'assets/contingency-v1/canaries'/(os.environ['RUSTDRIVING_TESTED_SOURCE']+'.json')
    if target.exists():raise ValueError('canary source already has retained evidence')
    selected=next(s for s in parent.specifications(['reference-2d'])
                  if s['world']=='mission' and s['seed']==7)
    source,baseline=parent.materialize(selected)
    if 'contingency_approaches' in baseline:raise ValueError('parent policy already changed')
    enabled=copy.deepcopy(baseline);enabled['contingency_approaches']=True
    inputs={}
    for name,value in [('baseline',baseline),('enabled',enabled)]:
        p=folder/(name+'.json');p.write_bytes((json.dumps(value,indent=2,allow_nan=False)+'\n').encode())
        inputs[name]=dict(path=str(p),sha256=sha(p),input=value)
    (folder/'frozen.json').write_text(json.dumps(dict(selected=selected,inputs=inputs),indent=2)+'\n')
    binary=ROOT/'target/release/rustdriving'
    old=Path(os.environ['RUSTDRIVING_BASELINE_BINARY'])
    report=dict(preliminary=True,qualified=False,default_enabled=False,
        source_commit=os.environ['RUSTDRIVING_TESTED_SOURCE'],baseline_source=os.environ['RUSTDRIVING_BASE_SOURCE'],
        protocol_sha256=sha(ROOT/'integrations/planning/contingency-v1.json'),
        binary_sha256=sha(binary),baseline_binary_sha256=sha(old),selected=selected,
        frozen_inputs=inputs,executions=[],errors=[])
    for name,program,input_name in [('old',old,'baseline'),('disabled',binary,'baseline'),('enabled',binary,'enabled')]:
        work=folder/'work';work.mkdir()
        commands=[]
        for mode,args,expected in [('run',['--scenario',inputs[input_name]['path'],'--seed',str(selected['seed'])],None),
                                    ('replay',['--log',str(work/'run/sensors.jsonl')],0)]:
            result=subprocess.run([str(program),mode,*args,'--output',str(work/mode)],capture_output=True,text=True)
            (work/(mode+'-process.log')).write_text(result.stdout+result.stderr)
            commands.append(dict(mode=mode,exit_code=result.returncode))
            if mode=='run' and result.returncode not in [0,1]:raise ValueError(result.stderr)
            if expected is not None and result.returncode!=expected:raise ValueError(result.stderr)
        run=json.loads((work/'run/run.json').read_text());replay=json.loads((work/'replay/replay.json').read_text())
        if not replay['verified']:raise ValueError('sensor-only replay failed')
        row=dict(name=name,commands=commands,summary=run['summary'],replay=replay,stream_sha256=hashes(work/'run'))
        if name=='enabled':
            with (work/'run/sensors.jsonl').open() as stream:
                header=json.loads(next(stream))['header']
                ticks=[item['tick'] for line in stream if 'tick' in (item:=json.loads(line))]
            try:
                row['contingency'],projected=reconstruct(header['config'],ticks)
                row['planning']=inspect(header['config'],projected)
            except Exception as error:
                row['audit_error']=repr(error);report['errors'].append(row['audit_error'])
        report['executions'].append(row)
        work.rename(folder/name)
    report['default_streams_identical']=report['executions'][0]['stream_sha256']==report['executions'][1]['stream_sha256']
    report['original_mission_goal_improved']=not report['executions'][0]['summary']['reached_goal'] and report['executions'][2]['summary']['reached_goal']
    report['canary_contract_passed']=report['default_streams_identical'] and not report['errors']
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('xb') as stream:stream.write((json.dumps(report,indent=2,allow_nan=False)+'\n').encode())
    latest=ROOT/'assets/contingency-v1/latest-canary.json'
    latest.write_text(json.dumps(dict(source_commit=report['source_commit'],path=str(target.relative_to(ROOT)),sha256=sha(target),canary_contract_passed=report['canary_contract_passed'],original_mission_goal_improved=report['original_mission_goal_improved']),indent=2)+'\n')
    if sha(binary)!=report['binary_sha256'] or sha(old)!=report['baseline_binary_sha256']:raise ValueError('binary changed during canary')
    print(json.dumps(dict(source_commit=report['source_commit'],
        defaults_identical=report['default_streams_identical'],goal_improved=report['original_mission_goal_improved'],
        episodes=[dict(name=r['name'],summary=r['summary'],contingency=r.get('contingency'),audit_error=r.get('audit_error')) for r in report['executions']],
        contract=report['canary_contract_passed'])),flush=True)
    if not report['canary_contract_passed']:raise SystemExit(1)
if __name__=='__main__':main()
