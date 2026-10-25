#!/usr/bin/env python3
"""Read-only localization of an archived, rejected contingency canary."""
import argparse
from collections import Counter
import hashlib
import json
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

def compact(tick):
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
                    c = compact(tick)
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
