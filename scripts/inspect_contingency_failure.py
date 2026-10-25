#!/usr/bin/env python3
"""Read-only localization of an archived, rejected contingency canary."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import zipfile

SOURCE = '796cd0407988606b943651d554dc2f4031c2e195'
CASE_SHA = '9215d6f7fe756c4227adefb7aef560a69c3d043244132a90161004f90ce4eab2'
MAX_MEMBER = 1024 * 1024 * 1024

def member(archive, suffix):
    matches = [n for n in archive.namelist() if n == suffix or n.endswith('/'+suffix)]
    if len(matches) != 1:
        raise ValueError('missing/ambiguous archived member: '+suffix)
    info = archive.getinfo(matches[0])
    if info.file_size > MAX_MEMBER:
        raise ValueError('oversized archived member')
    return matches[0]

def digest_member(archive, name):
    with archive.open(name) as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def compact(tick, vehicle_radius):
    out = tick['expected']
    plan = out.get('planning_diagnostics') or {}
    trajectory = out['trajectory']
    points = trajectory['points']
    return dict(time=out['time'], emergency=out['emergency'],
        position=out['estimate']['pose']['position'], speed=out['estimate']['speed'],
        acceleration=out['command']['acceleration'], health=out['health'],
        mode=trajectory['mode'], target=trajectory['lateral_target'],
        selected=plan.get('selected_candidate'), failure=plan.get('failure'),
        candidates=[dict(index=c['index'], target=c['lateral_target_m'],
            disposition=c['disposition'], conflict=c['nominal_forecast_conflict'],
            profile=c['profile']) for c in plan.get('candidates', [])],
        route_progress=plan.get('route_progress_m'),
        prefix=out.get('motion_prefix'), checked_approach=out.get('checked_approach'),
        trajectory_end=points[-1] if points else None,
        trajectory_hold_start=points[-2] if len(points)>1 else None,
        stationary_guard=stationary_guard(out['estimate'],out.get('planning_predictions',out['predictions']),vehicle_radius),
        forecasts=[dict(id=p['id'], radius=p['radius'], first=p['positions'][0],
            last=p['positions'][-1], horizon=p['dt']*(len(p['positions'])-1),
            first_padding=p.get('additional_radius_m', [0.])[0],
            last_padding=p.get('additional_radius_m', [0.])[-1])
            for p in out.get('planning_predictions', out['predictions'])])

def segments(rows, field):
    result = []
    start = None
    for i, row in enumerate(rows+[{}]):
        if row.get(field) and start is None:
            start = i
        if not row.get(field) and start is not None:
            result.append(dict(first_tick=start, last_tick=i-1, steps=i-start,
                start_s=rows[start]['time'], end_s=rows[i-1]['time']))
            start = None
    return result

def stationary_guard(estimate, predictions, vehicle_radius):
    """Exact quadratic interval checks for the declared eight-second zero-speed hold.

    A negative witness uses the existing directional reserve. The separate
    no-direction certificate is an explanatory ablation, never an admissibility
    rule or a recommendation to remove that reserve.
    """
    if abs(estimate['speed']) > 1e-12 or not predictions:
        return None
    origin=estimate['pose']['position']
    heading=(math.cos(estimate['pose']['yaw']),math.sin(estimate['pose']['yaw']))
    minimum_without=math.inf
    worst=None
    for obj in predictions:
        points=obj['positions']; dt=obj['dt']
        extra=obj.get('additional_radius_m') or [0.]*len(points)
        def at(t):
            step=t/dt; i=min(int(step),len(points)-1); j=min(i+1,len(points)-1)
            u=max(0.,min(1.,step-i))
            return ((points[i]['x']+(points[j]['x']-points[i]['x'])*u,
                     points[i]['y']+(points[j]['y']-points[i]['y'])*u),
                    obj['radius']+extra[i]+(extra[j]-extra[i])*u)
        cuts=sorted({0.,5.,8.,*[i*dt for i in range(len(points)) if 0.<i*dt<8.]})
        for lo,hi in zip(cuts,cuts[1:]):
            a,ra=at(lo); b,rb=at(hi)
            mid=(lo+hi)/2.; i=min(int(mid/dt),len(points)-1); j=min(i+1,len(points)-1)
            velocity=((points[j]['x']-points[i]['x'])/dt,(points[j]['y']-points[i]['y'])/dt)
            speed=math.hypot(*velocity)
            transverse=abs(velocity[0]*heading[1]-velocity[1]*heading[0])
            reserve=max(.4*min(transverse,1.),.7*transverse/speed) if speed else 0.
            q=(origin['x']-a[0],origin['y']-a[1]); delta=(a[0]-b[0],a[1]-b[1])
            start=vehicle_radius+ra+.3+.06*min(lo,5.)
            end=vehicle_radius+rb+.3+.06*min(hi,5.); change=end-start
            def minimum(extra_reserve):
                r=start+extra_reserve
                aa=delta[0]**2+delta[1]**2-change**2
                bb=2.*(q[0]*delta[0]+q[1]*delta[1]-r*change)
                cc=q[0]**2+q[1]**2-r*r
                choices=[0.,1.]
                if aa>0.:
                    choices.append(max(0.,min(1.,-bb/(2.*aa))))
                return min(((aa*u+bb)*u+cc,u) for u in choices)
            minimum_without=min(minimum_without,minimum(0.)[0])
            squared,u=minimum(reserve)
            if u==1.:
                u=1.-1e-7
            distance=math.hypot(q[0]+delta[0]*u,q[1]+delta[1]*u)
            radius=start+change*u
            witness=dict(prediction_id=obj['id'],time_s=lo+(hi-lo)*u,
                actual_gap_m=distance-radius-reserve,
                gap_without_directional_reserve_m=distance-radius,
                directional_reserve_m=reserve,observed_speed_m_s=speed,
                squared_separation_lower_bound=squared)
            if worst is None or witness['actual_gap_m']<worst['actual_gap_m']:
                worst=witness
    return dict(zero_speed_hold=True,existing_reserve_contact_witness=worst['actual_gap_m']<-1e-8,
        without_directional_reserve_certified=minimum_without>1e-9,
        without_directional_minimum_squared_separation=minimum_without,witness=worst)


def inspect(archive_path, output):
    output = Path(output).absolute()
    if output.exists() or output.is_symlink() or any(p.is_symlink() for p in output.parents):
        raise ValueError('output must be a new path with real parents')
    output.mkdir(parents=True)
    with zipfile.ZipFile(archive_path) as archive:
        case_path = member(archive, 'assets/contingency-v1/canaries/'+SOURCE+'.json')
        if digest_member(archive, case_path) != CASE_SHA:
            raise ValueError('original canary case bytes changed')
        case = json.loads(archive.read(case_path))
        if case['source_commit'] != SOURCE or case['qualified'] is not False:
            raise ValueError('wrong source or misrepresented qualification')
        all_rows, checkpoints, configs, stats = {}, {}, {}, {}
        for row in case['executions']:
            name = row['name']
            base = 'artifacts/contingency-canary/'+name+'/run/'
            for relative, expected in row['stream_sha256'].items():
                if digest_member(archive, member(archive, base+relative)) != expected:
                    raise ValueError(name+' retained stream bytes changed: '+relative)
            rows, complete, positives = [], [], []
            with archive.open(member(archive, base+'sensors.jsonl')) as stream:
                header = json.loads(next(stream))['header']
                configs[name] = header['config']
                for raw in stream:
                    item = json.loads(raw)
                    if 'tick' not in item:
                        continue
                    tick = item['tick']
                    c = compact(tick,header['config']['vehicle']['radius'])
                    if rows and c['time'] <= rows[-1]['time']:
                        raise ValueError('non-increasing control clock')
                    c['tick'] = len(rows)
                    rows.append(c)
                    if c['selected'] is not None and c['candidates'][c['selected']]['disposition'] == 'CompletePath':
                        complete.append(c['tick'])
                    if (c['prefix'] or {}).get('prefix_duration_s', 0.) > 0.:
                        positives.append(c['tick'])
            if len(rows) != row['summary']['steps'] or sum(r['emergency'] for r in rows) != row['summary']['emergency_steps']:
                raise ValueError('archived trace and original summary differ')
            all_rows[name] = rows
            stats[name] = dict(ticks=len(rows), emergency_segments=segments(rows, 'emergency'),
                modes=dict(Counter(r['mode'] for r in rows)), complete_path_ticks=complete,
                positive_prefix_ticks=positives,
                first_no_candidate=next((r for r in rows if r['failure']=='NoSelectableCandidate'), None))
        if case['executions'][0]['stream_sha256'] != case['executions'][1]['stream_sha256']:
            raise ValueError('disabled implementation differs from original')
        baseline, enabled = all_rows['old'], all_rows['enabled']
        if [r['time'] for r in baseline] != [r['time'] for r in enabled]:
            raise ValueError('different mission clocks')
        first = next((i for i,(a,b) in enumerate(zip(baseline, enabled))
            if b['emergency'] and not a['emergency']), None)
        selected = set()
        centers = [first]+stats['enabled']['complete_path_ticks']
        centers += [s['first_tick'] for s in stats['enabled']['emergency_segments'] if s['steps']>=20]
        positive = stats['enabled']['positive_prefix_ticks']
        if positive:
            centers += [positive[0], positive[-1]]
        for center in centers:
            if center is not None:
                selected.update(range(max(0, center-3), min(len(enabled), center+4)))
        for name, rows in all_rows.items():
            checkpoints[name] = [rows[i] for i in sorted(selected)]
        report = dict(schema_version=1, read_only=True, replay_or_simulation_rerun=False,
            actor_truth_used=False, original_source=SOURCE, original_case_sha256=CASE_SHA,
            default_streams_identical=True, first_extra_emergency_tick=first,
            first_extra_emergency_time_s=enabled[first]['time'] if first is not None else None,
            qualified=False, stats=stats, checkpoints=checkpoints)
        (output/'diagnosis.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        focus = dict(first_extra_emergency_tick=first,
            stats={n:dict(ticks=v['ticks'], emergency_segments=v['emergency_segments'],
                modes=v['modes']) for n,v in stats.items()},
            first_window={n:rows[max(0,first-3):first+4] for n,rows in all_rows.items()},
            low_speed_no_candidate_examples=[r for r in enabled
                if r['speed']<.05 and r['failure']=='NoSelectableCandidate'][:5])
        (output/'focus.json').write_text(json.dumps(focus,indent=2,allow_nan=False)+'\n')
        witnesses=[r for r in enabled if r['failure']=='NoSelectableCandidate' and r['stationary_guard']]
        directional=[r for r in witnesses if r['stationary_guard']['existing_reserve_contact_witness']
            and r['stationary_guard']['without_directional_reserve_certified']]
        witness_summary=dict(schema_version=1,actor_truth_used=False,
            original_case_sha256=CASE_SHA,original_source=SOURCE,
            failed_zero_speed_holds=len(witnesses),
            directional_only_rejections=len(directional),
            unclassified_failed_zero_speed_holds=len(witnesses)-len(directional),
            examples=[dict(tick=r['tick'],time_s=r['time'],position=r['position'],
                proof=r['stationary_guard']) for r in directional[:5]],
            scope='Independent stationary-envelope witnesses on the viewed rejected canary; the ablation does not authorize motion or remove a reserve.',
            first_rolling_emergency_cause_fully_localized=False)
        (output/'guard-witnesses.json').write_text(json.dumps(witness_summary,indent=2,allow_nan=False)+'\n')
        print(json.dumps(witness_summary,allow_nan=False))

        for name, rows in all_rows.items():
            (output/(name+'-timeline.json')).write_text(json.dumps(rows, separators=(',',':'), allow_nan=False)+'\n')
        print(json.dumps(dict(first_extra_emergency_time_s=report['first_extra_emergency_time_s'],
            stats={n:{k:v for k,v in s.items() if k not in ['first_no_candidate','positive_prefix_ticks','complete_path_ticks','emergency_segments']} for n,s in stats.items()},
            enabled_complete_ticks=stats['enabled']['complete_path_ticks'],
            enabled_positive_range=([positive[0],positive[-1]] if positive else [])), allow_nan=False))
        if first is not None:
            for i in range(max(0,first-3), min(len(enabled),first+4)):
                c=enabled[i]
                print(json.dumps({k:c[k] for k in ['tick','time','emergency','speed','position','acceleration','mode','target','selected','failure','prefix','candidates','forecasts','trajectory_end']}, allow_nan=False))

if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    inspect(args.archive,args.output)
