#!/usr/bin/env python3
"""Inspect recorded nominal conflicts; never infer global path infeasibility."""
import argparse
import hashlib
import json
from pathlib import Path

from check_forecast_calibration import read_retained
from inspect_planning import audit, finite

SOURCE = '92c72b1c473f0b6209f09c4ce1ff9508026b2d40'
ROOT = Path(__file__).resolve().parents[1]
CANONICAL = ROOT/'assets/chassis-acquisition-v2/qualifications'/(SOURCE+'.json')

def witness(config, out, candidate):
    conflict = candidate['nominal_forecast_conflict']
    if conflict is None:
        return None
    identity = conflict['prediction_id']
    predictions = [p for p in out['predictions'] if p['id']==identity]
    tracks = [t for t in out['tracks'] if t['id']==identity]
    if len(predictions)!=1 or len(tracks)!=1:
        raise ValueError('conflict requires a unique acquired track and prediction')
    prediction, track = predictions[0], tracks[0]
    age = finite(out['time'])-finite(track['last_seen'])
    time = finite(conflict['relative_time_s'])
    horizon = finite(prediction['dt'])*(len(prediction['positions'])-1)
    if age < 0. or time < 0. or horizon <= 0.:
        raise ValueError('invalid acquisition or prediction clock')
    # The recorded radius is endpoint-held beyond the prediction horizon.
    u = age+min(time,horizon)
    budget = config['forecast_envelope']
    terms = dict(position=finite(budget['position_error_m']),
                 velocity=finite(budget['velocity_error_m_s'])*u,
                 acceleration=finite(budget['acceleration_error_m_s2'])*u*u/2.)
    if any(v<0. for v in terms.values()):
        raise ValueError('negative motion-error budget')
    return dict(candidate_index=candidate['index'],lateral_target_m=candidate['lateral_target_m'],
        disposition=candidate['disposition'],prediction_id=identity,relative_conflict_time_s=time,
        acquisition_age_s=age,forecast_horizon_s=horizon,endpoint_held=time>horizon,
        declared_cv_terms_m=terms,declared_cv_radius_m=sum(terms.values()),
        published_forecast_radius_m=conflict['forecast_radius_m'],
        nominal_object_radius_m=prediction['radius'],
        minimum_clearance_floor_m=conflict['minimum_clearance_floor_m'])

def inspect(config,ticks):
    validation = audit(config,ticks)
    if not ticks or not validation['capture_enabled']:
        raise ValueError('complete captured decisions are required')
    snapshots = []
    for tick in ticks:
        out = tick['expected']
        if out['health'] or out['estimate']['speed']>.2:
            continue
        report = out['planning_diagnostics']
        if not report['candidates']:
            continue
        snapshots.append(dict(time_s=out['time'],route_progress_m=report['route_progress_m'],
            selected_candidate=report['selected_candidate'],
            candidates=[dict(index=c['index'],disposition=c['disposition'],
                conflict=witness(config,out,c)) for c in report['candidates']]))
    if not snapshots:
        raise ValueError('no healthy stationary planner snapshot')
    return dict(verified_control_ticks=validation['verified_control_ticks'],
        healthy_stationary_ticks=len(snapshots),first=snapshots[0],last=snapshots[-1],
        interpretation='Recorded nominal candidate conflicts only. CV terms are a reference bound, not an additive decomposition of the enclosing forecast. No proof of global infeasibility or of safety after reducing a budget.')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    raw=CANONICAL.read_bytes()
    if (args.evidence/'report.json').read_bytes()!=raw:
        raise ValueError('wrong retained qualification')
    source=json.loads(raw)
    if source['source']!=SOURCE or not source['passed'] or not source['full_qualification_complete']:
        raise ValueError('incomplete qualification')
    rows=[r for r in source['cases'] if r['world']=='mission']
    if {(r['plant'],r['seed']) for r in rows}!={(p,s) for p in ['reference-2d','rne-dynamic'] for s in [7,42]} or len(rows)!=4:
        raise ValueError('original mission cohort changed')
    result=dict(source=SOURCE,report_sha256=hashlib.sha256(raw).hexdigest(),
        simulation_rerun=False,operational_policy_changed=False,global_infeasibility_proven=False,cases=[])
    for row in rows:
        sensors=read_retained(args.evidence/'new',row,'run/sensors.jsonl')
        records=[json.loads(line) for line in sensors.splitlines()]
        ticks=[r['tick'] for r in records if 'tick' in r]
        if records[-1]!=dict(kind='end',ticks=len(ticks)):
            raise ValueError('incomplete sensor log footer')
        if len(ticks)!=row['summary']['steps']:
            raise ValueError('tick count differs from qualified outcome')
        result['cases'].append(dict(plant=row['plant'],seed=row['seed'],summary=row['summary'],
            sensor_sha256=hashlib.sha256(sensors).hexdigest(),
            **inspect(records[0]['header']['config'],ticks)))
    if args.output.exists() or args.output.is_symlink():
        raise ValueError('output must be new')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps(result,allow_nan=False))

if __name__=='__main__':
    main()
