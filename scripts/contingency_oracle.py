"""Independent shared-registration and acquisition-owned contingency checks."""
import math
from pose_forecast_oracle import nominal_output
from check_obstruction_stops import certify_trajectory

def xy(p):
    return [p['x'], p['y']]

def close(a, b):
    if type(a) not in (int, float) or type(b) not in (int, float):
        raise ValueError('mistyped planning number')
    if not math.isfinite(a) or not math.isfinite(b) or abs(a-b) > 1e-9*max(1.,abs(a),abs(b)):
        raise ValueError('conditional planning occupancy differs from declared motion basis')

def planning_view(config, output):
    if not config.get('contingency_approaches', False):
        if output.get('planning_predictions') is not None or output.get('motion_prefix') is not None:
            raise ValueError('undeclared conditional planning evidence')
        return output
    predictions = output.get('planning_predictions')
    if not isinstance(predictions, list) or not isinstance(output.get('motion_prefix'), dict):
        raise ValueError('conditional planning evidence disappeared')
    return dict(output, predictions=predictions)

def reconstruct(config, ticks, *, certify=True):
    if config.get('contingency_approaches') is not True:
        raise ValueError('contingency policy not enabled')
    limits = config.get('motion_limits') or {}
    acceleration = limits.get('max_acceleration_m_s2', 2.)
    deceleration = limits.get('max_deceleration_m_s2', 2.5)
    seen = []
    latest = None
    previous = None
    selected = positive = bounds = knots = 0
    maximum_padding = 0.
    projected = []
    for tick in ticks:
        source, output = tick['input'], tick['expected']
        now = output['time']
        dt = config['nominal_dt'] if previous is None else now-previous
        previous = now
        scan = source.get('lidar')
        cold = output.get('cold_motion') or {}
        healthy_scene = not output['health'] and all(row.get('confirmed') is True for row in cold.get('tracks', []))
        if scan is not None and healthy_scene and not source.get('lidar_failed'):
            stamp = scan['stamp']
            if latest is None or stamp > latest:
                latest = stamp
                seen.append((stamp, now))
        view = planning_view(config, output)
        evidence = output['motion_prefix']
        active = evidence['shared_pose_cancellation']
        if type(active) is not bool:
            raise ValueError('mistyped common registration ownership')
        if active:
            if output['health'] or config.get('forecast_pose_sigma') is None:
                raise ValueError('common-pose cancellation lacks a healthy declared basis')
            expected = nominal_output(output, config['forecast_pose_sigma'])['predictions']
        else:
            expected = output['predictions']
        if len(expected) != len(view['predictions']):
            raise ValueError('conditional occupancy omitted identities')
        for actual, base, world in zip(view['predictions'], expected, output['predictions']):
            for key in ['id', 'positions', 'radius', 'dt']:
                if actual[key] != base[key]:
                    raise ValueError('planning changed forecast centers, shape or clocks')
            a, b = actual.get('additional_radius_m', []), base.get('additional_radius_m', [])
            if len(a) != len(b):
                raise ValueError('conditional radius inventory changed')
            for index, (x, y) in enumerate(zip(a,b)):
                close(x,y)
                maximum_padding=max(maximum_padding,world['additional_radius_m'][index]-x)
                knots += 1
        projected.append(dict(tick, expected=view))
        if evidence['selected'] is not True:
            continue
        selected += 1
        path = output['trajectory']['points']
        if len(path)<2 or len(path)>84 or path[0]['time']!=0.:
            raise ValueError('missing bounded contingency path')
        span = evidence['prefix_duration_s']
        if not math.isfinite(span) or not 0.<=span<=.1+1e-9:
            raise ValueError('unbounded motion prefix')
        if evidence['lateral_target_m'] != output['trajectory']['lateral_target']:
            raise ValueError('prefix target not bound to actual path')
        if span>0.:
            positive += 1
            first,last,until=(evidence[key] for key in
                ['first_acquisition_s','last_acquisition_s','authorized_until_s'])
            if (evidence['acquisition_count']!=3 or first is None or last is None
                    or last-first<.2-1e-9 or latest!=last
                    or abs(until-last-.1)>1e-8 or abs(until-now-span)>1e-8
                    or evidence['control_interval_s']+1e-8<max(dt,config['nominal_dt'])
                    or span+1e-8<evidence['control_interval_s']):
                raise ValueError('prefix renewed without its acquired time budget')
            history=[stamp for stamp,_ in seen if first-1e-9<=stamp<=last+1e-9]
            if len(history)<3 or abs(history[0]-first)>1e-8 or any(b-a>.35+1e-9 for a,b in zip(history,history[1:])):
                raise ValueError('prefix lacks continuous fresh acquisitions')
            if not any(abs(p['time']-span)<1e-8 for p in path):
                raise ValueError('prefix endpoint missing from actual trajectory')
        elif output['command']['acceleration']>1e-8:
            raise ValueError('a braking-only backup commands acceleration')
        for a,b in zip(path,path[1:]):
            duration=b['time']-a['time']
            if not duration>0. or min(a['speed'],b['speed'])<0.:
                raise ValueError('invalid contingency time or velocity')
            value=(b['speed']-a['speed'])/duration
            if not -deceleration-1e-6<=value<=acceleration+1e-6:
                raise ValueError('contingency exceeds supplied longitudinal authority')
            close(math.dist(xy(a['position']),xy(b['position'])),(a['speed']+b['speed'])*duration/2.)
        if path[-1]['speed']!=0. or path[-1]['position']!=path[-2]['position'] or abs(path[-1]['time']-path[-2]['time']-8.)>1e-8:
            raise ValueError('contingency lost its full stationary tail')
        if output['health'] and output['command']['acceleration']>-5.99:
            raise ValueError('independent sensor fault lost braking')
        if certify:
            proof=certify_trajectory(view,config['vehicle']['radius'])
            bounds+=proof['interval_bounds']
    return dict(passed=True,verified_ticks=len(ticks),verified_radius_knots=knots,
        selected_contingency_ticks=selected,positive_prefix_ticks=positive,
        interval_bounds=bounds,maximum_removed_shared_pose_padding_m=maximum_padding,
        actor_truth_used=False,confidence_calibrated=False),projected
