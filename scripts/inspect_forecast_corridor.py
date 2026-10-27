"""Sampled road cross-sections against published occupancy; not path certification."""
import math
from check_hazards import prediction_radius_at
from inspect_continuation_probes import number


def xy(point):
    return number(point['x']), number(point['y'])


def section(center, tangent, half_width, discs):
    """Exact line/disc intersections in real arithmetic; floating-point diagnostic."""
    width = number(half_width)
    length = math.hypot(*tangent)
    if width <= 0 or not math.isfinite(length) or length <= 0:
        raise ValueError('positive section width and finite tangent required')
    center = tuple(number(v) for v in center)
    tangent = tuple(number(v)/length for v in tangent)
    normal = (-tangent[1], tangent[0])
    intervals = []
    for identity, position, radius in discs:
        radius = number(radius)
        if radius < 0:
            raise ValueError('negative occupied radius')
        delta = tuple(number(v)-c for v,c in zip(position,center))
        along = sum(x*y for x,y in zip(delta,tangent))
        lateral = sum(x*y for x,y in zip(delta,normal))
        if abs(along) > radius:
            continue
        reach = math.sqrt(max(0., (radius-abs(along))*(radius+abs(along))))
        lower, upper = max(-width,lateral-reach), min(width,lateral+reach)
        if lower <= upper:
            intervals.append(dict(prediction_id=identity, lower_m=lower, upper_m=upper))
    merged = []
    for interval in sorted(intervals, key=lambda r:r['lower_m']):
        lo,hi = interval['lower_m'],interval['upper_m']
        if merged and lo <= merged[-1][1]:
            merged[-1][1] = max(hi,merged[-1][1])
        else:
            merged.append([lo,hi])
    gaps = []
    cursor = -width
    for lo,hi in merged:
        if lo > cursor: gaps.append([cursor,lo])
        cursor = max(cursor,hi)
    if cursor < width: gaps.append([cursor,width])
    return dict(occupied_intervals=intervals, occupied_union_m=merged,
        free_intervals_m=gaps, free_width_m=sum(b-a for a,b in gaps),
        fully_covered=bool(merged) and merged[0][0] <= -width and merged[0][1] >= width)


def inspect(route, predictions, vehicle_radius, progress):
    points = [xy(p) for p in route['points']]
    if len(points) < 2:
        raise ValueError('route requires two points')
    lengths = [0.]
    for a,b in zip(points,points[1:]):
        distance = math.dist(a,b)
        if not math.isfinite(distance) or distance <= 0:
            raise ValueError('invalid route segment')
        lengths.append(lengths[-1]+distance)
    radius = number(vehicle_radius)
    width = number(route['half_width'])-radius
    progress = number(progress)
    if radius <= 0 or width <= 0 or not 0 <= progress <= lengths[-1]:
        raise ValueError('invalid ego corridor or progress')
    ids = [p['id'] for p in predictions]
    if len(set(ids)) != len(ids):
        raise ValueError('duplicate prediction identity')
    def discs_at(time):
        discs = []
        for prediction in predictions:
            positions = [xy(p) for p in prediction['positions']]
            occupied = prediction_radius_at(prediction,time)
            if not positions or not math.isfinite(occupied):
                raise ValueError('invalid prediction occupancy')
            step = time/number(prediction['dt'])
            index = min(int(step),len(positions)-1)
            nxt = min(index+1,len(positions)-1)
            fraction = min(1.,step-index)
            center = tuple((1-fraction)*a+fraction*b
                           for a,b in zip(positions[index],positions[nxt]))
            discs.append((prediction['id'],center,occupied+radius))
        return discs
    rows = []
    for time in [0.,2.,4.,8.]:
        discs = discs_at(time)
        for ahead in [0.,.5,1.,2.,4.,8.,12.]:
            distance = progress+ahead
            if distance > lengths[-1]: continue
            index = next((i for i in range(len(points)-1) if distance <= lengths[i+1]),len(points)-2)
            a,b = points[index:index+2]
            fraction = (distance-lengths[index])/(lengths[index+1]-lengths[index])
            center = tuple((1-fraction)*x+fraction*y for x,y in zip(a,b))
            tangent = tuple(y-x for x,y in zip(a,b))
            rows.append(dict(relative_time_s=time, ahead_m=ahead, route_progress_m=distance,
                center=dict(x=center[0],y=center[1]), **section(center,tangent,width,discs)))
    return dict(ego_center_half_width_m=width, samples=rows,
        additional_directional_and_clearance_reserves_included=False,
        continuous_space_time_certified=False, operational_policy_changed=False,
        interpretation='Published obstacle discs plus ego radius on sampled nominal road cross-sections. Free intervals are optimistic: extra reserves, corner footprint containment and connecting trajectories are not certified. Fully covered is a sampled floating-point result, not global infeasibility.')
