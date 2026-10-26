#!/usr/bin/env python3
"""Paired recorded-observation experiment; commands never advance a simulator."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from check_forecast_calibration import read_retained
from inspect_stall_conflicts import CANONICAL, SOURCE, inspect

evidence, output, binary = map(Path, sys.argv[1:])
raw = (evidence/'report.json').read_bytes()
if raw != CANONICAL.read_bytes():
    raise ValueError('wrong original qualification')
qualified = json.loads(raw)
rows = [r for r in qualified['cases'] if r['world']=='mission']
if len(rows)!=4 or {(r['plant'],r['seed']) for r in rows}!={(p,s) for p in ['reference-2d','rne-dynamic'] for s in [7,42]}:
    raise ValueError('wrong original mission cohort')
output.mkdir(parents=True,exist_ok=False)
result=dict(recorded_source=SOURCE,closed_loop=False,goal_improvement_established=False,
    default_policy_changed=False,complete=False,cases=[])
for row in rows:
    key=row['plant']+'-'+str(row['seed'])
    sensors=read_retained(evidence/'new',row,'run/sensors.jsonl')
    source=output/(key+'.jsonl')
    source.write_bytes(sensors)
    paired=output/(key+'-paired.jsonl')
    subprocess.run([str(binary.resolve()),str(source),str(paired)],check=True)
    records=[json.loads(line) for line in paired.read_text().splitlines()]
    config=records[0]['counterfactual_config']
    original=dict(records[0]['original_config'])
    reduced=dict(config);reduced.pop('anchored_side_recovery')
    if original!=reduced or config['anchored_side_recovery'] is not True:
        raise ValueError('more than the experimental recovery flag changed')
    ticks=[r for r in records if r['kind']=='tick']
    if records[-1]!=dict(kind='end',ticks=len(ticks),legacy_verified=True,closed_loop=False):
        raise ValueError('incomplete paired replay')
    if len(ticks)!=row['summary']['steps']:
        raise ValueError('missing original ticks')
    selected=lambda out: next((c['disposition'] for c in out['planning_diagnostics']['candidates']
        if c['index']==out['planning_diagnostics']['selected_candidate']),None)
    stationary=[t for t in ticks if not t['original']['health'] and t['original']['estimate']['speed']<=.2]
    result['cases'].append(dict(plant=row['plant'],seed=row['seed'],
        sensor_sha256=hashlib.sha256(sensors).hexdigest(),verified_legacy_ticks=len(ticks),
        healthy_stationary_complete_paths=dict(
            original=sum(selected(t['original'])=='CompletePath' for t in stationary),
            counterfactual=sum(selected(t['counterfactual'])=='CompletePath' for t in stationary)),
        counterfactual=inspect(config,[dict(expected=t['counterfactual']) for t in ticks])))
    source.unlink()
result['complete']=True
(output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps(result,allow_nan=False))
