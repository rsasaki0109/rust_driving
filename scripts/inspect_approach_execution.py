#!/usr/bin/env python3
"""Read-only execution traces after checked-approach authorization."""
import argparse
import hashlib
import json
from pathlib import Path
from check_forecast_calibration import read_retained
from checked_approach_oracle import reconstruct
from inspect_planning import audit, finite

SOURCE = '75d6c3fe11b5194ba522708f23a3a5c3eb43de46'
ROOT = Path(__file__).resolve().parents[1]
CANONICAL = ROOT/'assets/small-approaches'/(SOURCE+'.json')

def trace(out):
    path = out['trajectory']
    points = path['points']
    report = out['planning_diagnostics']
    selected = report['selected_candidate']
    chosen = None if selected is None else report['candidates'][selected]
    first = points[0] if points else None
    second = points[1] if len(points)>1 else None
    feedforward = None
    if second is not None:
        duration = finite(second['time'])-finite(first['time'])
        if duration <= 0:
            raise ValueError('non-increasing path clock')
        feedforward = (finite(second['speed'])-finite(first['speed']))/duration
    return dict(time_s=finite(out['time']),progress_m=finite(report['route_progress_m']),
        estimated_speed_m_s=finite(out['estimate']['speed']),
        acceleration_m_s2=finite(out['command']['acceleration']),
        steering_rad=finite(out['command']['steering']),emergency=out['emergency'],
        health=out['health'],mode=path['mode'],lateral_target_m=path['lateral_target'],
        selected_disposition=None if chosen is None else chosen['disposition'],
        endpoint=None if not points else points[-1]['position'],
        initial_speed_m_s=None if first is None else first['speed'],
        initial_feedforward_m_s2=feedforward,
        peak_planned_speed_m_s=max((p['speed'] for p in points),default=None),
        adopted=out['checked_approach']['adopted'])

def events(ticks):
    rows = [trace(t['expected']) for t in ticks]
    if not rows or any(b['time_s']<=a['time_s'] for a,b in zip(rows,rows[1:])):
        raise ValueError('strictly increasing nonempty control clock required')
    indices = [i for i,r in enumerate(rows) if r['adopted']]
    result = []
    for n,start in enumerate(indices):
        stop = indices[n+1] if n+1<len(indices) else len(rows)
        segment = rows[start:stop]
        initial = segment[0]
        def first_where(predicate):
            return next((r for r in segment[1:] if predicate(r)),None)
        result.append(dict(adoption_time_s=initial['time_s'],
            end_reason='next_adoption' if stop<len(rows) else 'recording_end',
            next_adoption_time_s=rows[stop]['time_s'] if stop<len(rows) else None,
            last_observed_time_s=segment[-1]['time_s'],
            observed_progress_gain_m=segment[-1]['progress_m']-initial['progress_m'],
            maximum_estimated_speed_m_s=max(r['estimated_speed_m_s'] for r in segment),
            positive_acceleration_ticks=sum(r['acceleration_m_s2']>0 for r in segment),
            emergency_ticks=sum(bool(r['emergency']) for r in segment),
            first_nonpositive_acceleration=first_where(lambda r:r['acceleration_m_s2']<=0),
            first_endpoint_change=first_where(lambda r:r['endpoint']!=initial['endpoint']),
            first_stationary_profile=first_where(lambda r:r['peak_planned_speed_m_s'] is not None
                and r['peak_planned_speed_m_s']<=.2),
            trace=segment))
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    raw=CANONICAL.read_bytes()
    if (args.evidence/'report.json').read_bytes()!=raw:
        raise ValueError('wrong paired evidence')
    source=json.loads(raw)
    if source['source']!=SOURCE or not source['complete'] or not source['contract_passed']:
        raise ValueError('unqualified or incomplete canary contract')
    cases=source['cases']
    if len(cases)!=4 or {(c['plant'],c['seed']) for c in cases}!={(p,s) for p in ['reference-2d','rne-dynamic'] for s in [7,42]}:
        raise ValueError('wrong paired cohort')
    result=dict(recorded_driver_source=SOURCE,report_sha256=hashlib.sha256(raw).hexdigest(),
        simulation_rerun=False,operational_policy_changed=False,
        interpretation='Descriptive sensor-only traces. Endpoint inequality is not proof of harmful replacement; feedforward is only one controller term. Recording-end intervals are censored, and estimates are not physical ground truth.',
        cases=[])
    for case in cases:
        pair=dict(plant=case['plant'],seed=case['seed'])
        for variant,folder in [('baseline','baseline'),('candidate','new')]:
            row=case[variant]
            sensors=read_retained(args.evidence/folder,row,'run/sensors.jsonl')
            records=[json.loads(line) for line in sensors.splitlines()]
            ticks=[r['tick'] for r in records if 'tick' in r]
            if records[-1]!=dict(kind='end',ticks=len(ticks)) or len(ticks)!=row['summary']['steps']:
                raise ValueError('incomplete sensor recording')
            config=records[0]['header']['config']
            planning=audit(config,ticks)
            checked=reconstruct(config,ticks)
            if checked!=row['checked_approach']:
                raise ValueError('authorization audit differs from retained canary')
            inspected=events(ticks)
            if [e['adoption_time_s'] for e in inspected]!=[a['time_s'] for a in checked['adoptions']]:
                raise ValueError('lost or duplicated authorization')
            pair[variant]=dict(sensor_sha256=hashlib.sha256(sensors).hexdigest(),
                verified_ticks=planning['verified_control_ticks'],summary=row['summary'],events=inspected)
        result['cases'].append(pair)
    if args.output.exists() or args.output.is_symlink():
        raise ValueError('new output required')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')
    for case in result['cases']:
        print(json.dumps(dict(plant=case['plant'],seed=case['seed'],
            **{v:[{k:x for k,x in e.items() if k!='trace'} for e in case[v]['events']]
               for v in ['baseline','candidate']}),allow_nan=False),flush=True)

if __name__=='__main__':
    main()
