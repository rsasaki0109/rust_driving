"""Read-only boundaries of executed continuation selections, not causal certificates."""
from collections import Counter
from inspect_continuation_probes import number


def held_command_comparison(out, dt):
    """Scalar command-hold model versus planned speed, not native plant truth."""
    dt = number(dt)
    points = out['trajectory']['points']
    if dt <= 0 or len(points) < 2 or number(points[0]['time']) != 0:
        raise ValueError('positive hold interval and complete time-zero profile required')
    for point in points:
        if number(point['speed']) < 0 or number(point['time']) < 0:
            raise ValueError('nonnegative profile required')
    if any(b['time'] <= a['time'] for a, b in zip(points, points[1:])):
        raise ValueError('strict profile clock required')
    # Refuse to invent the plan beyond its recorded horizon.
    if dt > points[-1]['time']:
        raise ValueError('command interval exceeds recorded plan')
    accelerations = []
    planned = None
    for a, b in zip(points, points[1:]):
        if a['time'] >= dt:
            break
        acceleration = (b['speed']-a['speed'])/(b['time']-a['time'])
        accelerations.append(number(acceleration))
        if dt <= b['time']:
            planned = a['speed'] + acceleration*(dt-a['time'])
            break
    initial = number(out['estimate']['speed'])
    held = max(0., initial + number(out['command']['acceleration'])*dt)
    return dict(interval_s=dt, first_segment_s=points[1]['time'],
        crosses_profile_knot=points[1]['time'] < dt,
        minimum_planned_acceleration_m_s2=min(accelerations),
        planned_end_speed_m_s=planned, held_command_end_speed_m_s=held,
        excess_end_speed_m_s=held-planned)


def snapshot(out):
    diagnostic = out.get('planning_diagnostics')
    selected = None if diagnostic is None else diagnostic['selected_candidate']
    candidate = None if selected is None else diagnostic['candidates'][selected]
    points = out['trajectory']['points']
    return dict(time_s=number(out['time']),
        estimated_speed_m_s=number(out['estimate']['speed']),
        acceleration_m_s2=number(out['command']['acceleration']),
        emergency=out['emergency'], health=out['health'],
        mode=out['trajectory']['mode'],
        peak_planned_speed_m_s=max((number(p['speed']) for p in points), default=None),
        endpoint=None if not points else points[-1]['position'],
        continuation=out.get('continuation'),
        checked_approach=out.get('checked_approach'),
        planning_diagnostics_available=diagnostic is not None,
        selected_disposition=None if candidate is None else candidate['disposition'],
        route_progress_m=None if diagnostic is None else diagnostic['route_progress_m'],
        obstruction_stop_requested=None if diagnostic is None else diagnostic['obstruction_stop_requested'],
        obstruction_stop_active=None if diagnostic is None else diagnostic['obstruction_stop_active'])


def inspect(ticks):
    rows = [snapshot(t['expected']) for t in ticks]
    if not rows or any(b['time_s'] <= a['time_s'] for a, b in zip(rows, rows[1:])):
        raise ValueError('nonempty strictly ordered recording required')
    comparisons = []
    for i, (tick, row) in enumerate(zip(ticks, rows)):
        row['command_hold'] = None
        if row['continuation'] is not None and i > 0:
            row['command_hold'] = held_command_comparison(
                tick['expected'], row['time_s']-rows[i-1]['time_s'])
            comparisons.append(row['command_hold'])
    spans = []
    i = 0
    while i < len(rows):
        if rows[i]['continuation'] is None:
            i += 1
            continue
        start = i
        permit = rows[start]['continuation']
        # Acquisition time advances; immutable expiry/endpoint identify the selection span.
        identity = (permit['expires_s'], permit['original_endpoint'], permit['original_end_progress_m'])
        while i < len(rows):
            current = rows[i]['continuation']
            if current is None or (current['expires_s'], current['original_endpoint'],
                                   current['original_end_progress_m']) != identity:
                break
            if number(current['now_s']) != rows[i]['time_s']:
                raise ValueError('selection timestamp differs from output')
            if number(current['expires_s']) <= rows[i]['time_s']:
                raise ValueError('selected continuation at or after expiry')
            i += 1
        segment = rows[start:i]
        following = rows[i] if i < len(rows) else None
        spans.append(dict(first=segment[0], last=segment[-1], following=following,
            previous=None if start == 0 else rows[start-1],
            selected_ticks=len(segment),
            positive_acceleration_ticks=sum(r['acceleration_m_s2'] > 0 for r in segment),
            maximum_estimated_speed_m_s=max(r['estimated_speed_m_s'] for r in segment),
            expiry_remaining_at_last_s=permit['expires_s']-segment[-1]['time_s'],
            end_observation=('recording_end' if following is None else
                'selection_absent' if following['continuation'] is None else 'different_permit')))
    tail = [r for r in rows if r['time_s'] >= rows[-1]['time_s']-5.]
    return dict(selected_ticks=sum(s['selected_ticks'] for s in spans), spans=spans,
        command_hold=dict(compared_ticks=len(comparisons),
            unavailable_ticks=sum(r['continuation'] is not None and r['command_hold'] is None for r in rows),
            knot_crossing_ticks=sum(r['crosses_profile_knot'] for r in comparisons),
            speed_excess_ticks=sum(r['excess_end_speed_m_s'] > 1e-8 for r in comparisons),
            maximum_speed_excess_m_s=max((max(0.,r['excess_end_speed_m_s']) for r in comparisons),default=0.),
            scope='Scalar held-command estimate versus recorded plan; not simulated or measured next vehicle speed.'),
        terminal=rows[-1], terminal_window_s=5.,
        terminal_dispositions=dict(Counter(
            r['selected_disposition'] if r['planning_diagnostics_available'] else 'diagnostic_unavailable'
            for r in tail)),
        interpretation='Observed selection boundaries only. Absence does not identify revocation cause; recording end is censored.')


if __name__ == '__main__':
    import argparse
    import json
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.log.read_bytes().splitlines()]
    ticks = [r['tick'] for r in records if r['kind'] == 'tick']
    if not records or records[-1] != dict(kind='end', ticks=len(ticks)):
        raise ValueError('complete recording required')
    with args.output.open('x') as stream:
        json.dump(inspect(ticks), stream, indent=2, allow_nan=False)
        stream.write('\n')
