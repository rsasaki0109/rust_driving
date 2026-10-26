#!/usr/bin/env python3
"""Inspect the original four mission stop searches; no simulation is rerun."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from check_forecast_calibration import read_retained
from inspect_stall_conflicts import CANONICAL, SOURCE
from inspect_planning import audit
from directional_reserve_oracle import certify
from stop_candidate_evidence import summarize

evidence,output,binary=map(Path,sys.argv[1:])
raw=(evidence/'report.json').read_bytes()
if raw!=CANONICAL.read_bytes():raise ValueError('wrong retained qualification')
rows=[r for r in json.loads(raw)['cases'] if r['world']=='mission']
if len(rows)!=4 or {(r['plant'],r['seed']) for r in rows}!={(p,s) for p in ['reference-2d','rne-dynamic'] for s in [7,42]}:
    raise ValueError('original mission cohort changed')
output.mkdir(parents=True,exist_ok=False)
result=dict(analysis_source=os.environ['ANALYSIS_SOURCE'],recorded_source=SOURCE,simulation_rerun=False,operational_policy_changed=False,complete=False,cases=[])
for row in rows:
    name=row['plant']+'-'+str(row['seed'])
    sensors=read_retained(evidence/'new',row,'run/sensors.jsonl')
    records=[json.loads(line) for line in sensors.splitlines()]
    config=records[0]['header']['config']
    ticks=[r['tick'] for r in records if 'tick' in r]
    source=output/(name+'-sensors.jsonl');source.write_bytes(sensors)
    target=output/(name+'-probe.jsonl')
    subprocess.run([str(binary.resolve()),str(source),str(target),str(ticks[-1]['input']['time'])],check=True)
    probe_records=[json.loads(line) for line in target.read_text().splitlines()]
    if len(probe_records)!=3 or probe_records[0]['config']!=config or probe_records[-1]!=dict(kind='end',ticks=len(ticks),legacy_verified=True,closed_loop=False):
        raise ValueError('incomplete or changed probe replay')
    if len(ticks)!=row['summary']['steps'] or probe_records[1]['original']!=ticks[-1]['expected']:
        raise ValueError('original recorded outcome changed')
    probe=probe_records[1]['probe']
    candidate=dict(ticks[-1]['expected'],trajectory=probe['trajectory'],
        planning_diagnostics=probe['planning_diagnostics'],emergency=probe['trajectory']['mode']=='Emergency')
    altered=ticks[:-1]+[dict(ticks[-1],expected=candidate)]
    validation=audit(config,altered)
    certificate=certify(config,altered)
    result['cases'].append(dict(plant=row['plant'],seed=row['seed'],verified_legacy_ticks=len(ticks),
        sensor_sha256=hashlib.sha256(sensors).hexdigest(),probe=probe,
        search=summarize(probe['attempts']),planning_validation=validation,
        selected_path_certificate=certificate))
    source.unlink()
result['complete']=True
(output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(complete=True,cases=[dict(plant=r['plant'],seed=r['seed'],search=r['search'],
    selected=r['probe']['planning_diagnostics']['selected_candidate'],
    candidates=r['probe']['planning_diagnostics']['candidates']) for r in result['cases']]),allow_nan=False))
