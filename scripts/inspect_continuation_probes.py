"""Structural and comparative audit of unused continuation probe records."""
import math

def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('finite numeric probe field required')
    return value

def inspect(records):
    if len(records) < 2 or records[0].get('kind') != 'configuration' or records[-1].get('kind') != 'end':
        raise ValueError('complete inspection header/footer required')
    summary = records[-1]
    if summary.get('legacy_verified') is not True or summary.get('closed_loop') is not False:
        raise ValueError('verified non-executed inspection required')
    if type(summary.get('ticks')) is not int or summary['ticks'] <= 0:
        raise ValueError('positive verified tick count required')
    active = False
    adoption_time = None
    original_bound = None
    counts = dict(adoptions=0, permits=0, attempted_ticks=0, candidate_ticks=0)
    later = faster = low_speed = 0
    last_tick = -1
    last_time = -1.
    for row in records[1:-1]:
        tick = row.get('tick')
        time = number(row['time'])
        if row.get('kind') != 'probe' or type(tick) is not int or tick <= last_tick or time <= last_time:
            raise ValueError('strictly ordered probe records required')
        if time < 0 or tick < 0 or tick >= summary['ticks'] or row.get('closed_loop') is not False:
            raise ValueError('probe outside verified recording')
        last_tick, last_time = tick, time
        for field in ['new_adoption', 'permit_alive', 'healthy_unrestricted_output']:
            if type(row[field]) is not bool:
                raise ValueError('boolean probe state required')
        adopted = row['new_adoption']
        bound = row['endpoint_bound']
        candidate = row['candidate']
        if adopted:
            adoption_time = time
            original_bound = bound
        elif not active:
            raise ValueError('probe resumed without a new adoption')
        if bound is not None and (not isinstance(bound, list) or len(bound) != 2):
            raise ValueError('endpoint/progress bound required')
        if not adopted and bound != original_bound:
            raise ValueError('continuation extended its original bound')
        active = row['permit_alive']
        counts['adoptions'] += adopted
        counts['permits'] += adopted and bound is not None
        counts['attempted_ticks'] += bound is not None
        if candidate is None:
            if row['permit_alive']:
                raise ValueError('failed candidate did not revoke permit')
            continue
        if bound is None or not row['permit_alive'] or not row['healthy_unrestricted_output']:
            raise ValueError('candidate without live healthy permit')
        if adoption_time is None or time >= adoption_time + 8.:
            raise ValueError('candidate exceeded immutable permit duration')
        counts['candidate_ticks'] += 1
        later += not adopted
        points = candidate['points']
        if len(points) < 2 or candidate['mode'] != 'Yield':
            raise ValueError('complete yielding candidate required')
        for p in points:
            if number(p['speed']) < 0 or number(p['time']) < 0:
                raise ValueError('negative profile state')
            number(p['position']['x'])
            number(p['position']['y'])
        if any(b['time'] <= a['time'] for a, b in zip(points, points[1:])):
            raise ValueError('nonmonotonic candidate profile')
        if (points[-1]['position'] != points[-2]['position']
            or points[-1]['speed'] != 0 or points[-2]['speed'] != 0
            or points[-1]['time'] - points[-2]['time'] < 8. - 1e-9):
            raise ValueError('candidate lost its full stop and hold')
        initial = number(row['ego']['speed'])
        if abs(points[0]['speed'] - initial) > 1e-8:
            raise ValueError('candidate changed initial speed')
        peak = max(p['speed'] for p in points)
        original_peak = max((number(p['speed']) for p in row['original']['points']), default=0.)
        improved = peak > original_peak + 0.1
        faster += improved
        low_speed += not adopted and initial <= 0.32 and improved
    if any(type(summary.get(k)) is not int or summary[k] != v for k, v in counts.items()):
        raise ValueError('probe counts differ from footer')
    return dict(probe_records=len(records)-2, **counts,
        later_candidate_ticks=later, higher_peak_than_original_ticks=faster,
        later_low_speed_higher_peak_ticks=low_speed,
        interpretation='Structural audit and descriptive comparisons; not an independent collision certificate or closed-loop improvement.')
