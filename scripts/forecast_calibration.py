"""Offline, grouped forecast calibration; physical truth never enters the driver."""
from fractions import Fraction
import math

from pose_forecast_oracle import factor, projected_padding

HORIZONS = (.2, 1., 2., 4., 8.)
STEP = .05


def finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('expected a finite number')
    return value


def xy(value):
    return tuple(finite(value[k]) for k in ('x', 'y'))


def quantile(groups, coverage='0.99'):
    """Exact split-conformal order statistic; None denotes an infinite score.

    This calculation alone establishes neither exchangeability nor confidence.
    Each group must have been fixed before viewing its outcomes.
    """
    probability = Fraction(coverage)
    if not 0 < probability < 1:
        raise ValueError('coverage must be strictly between zero and one')
    values = {}
    for group, score in groups:
        if not isinstance(group, str) or not group:
            raise ValueError('missing dependence group')
        if score is not None and finite(score) < 0:
            raise ValueError('negative nonconformity score')
        previous = values.get(group, 0.)
        values[group] = None if previous is None or score is None else max(previous, score)
    n = len(values)
    rank = (probability.numerator * (n + 1) + probability.denominator - 1) // probability.denominator
    ordered = sorted(values.values(), key=lambda v: math.inf if v is None else v)
    threshold = ordered[rank-1] if 1 <= rank <= n else None
    minimum = (probability.denominator-1) // (probability.denominator-probability.numerator)
    return dict(group_count=n, order_statistic_rank=rank, scale=threshold,
                unbounded=threshold is None, minimum_groups_for_finite_rank=minimum,
                confidence_established=False)


def grouped_split(cases, calibration_seeds, evaluation_seeds, coverage='0.99'):
    calibration_seeds, evaluation_seeds = set(calibration_seeds), set(evaluation_seeds)
    if not calibration_seeds or not evaluation_seeds or calibration_seeds & evaluation_seeds:
        raise ValueError('calibration and evaluation groups must be nonempty and disjoint')
    if any(type(s) is not int for s in calibration_seeds | evaluation_seeds):
        raise ValueError('seed groups require integer identities')
    ids, cal, test, counts = set(), [], [], {}
    for case in cases:
        identity = case['case_id']
        if identity in ids:
            raise ValueError('duplicate case')
        ids.add(identity)
        seed = case['seed']
        if type(seed) is not int or seed not in calibration_seeds | evaluation_seeds:
            raise ValueError('case is outside the frozen seed split')
        score = case['joint_nonconformity']
        group = 'seed:' + str(seed)
        counts[group] = counts.get(group, 0) + 1
        (cal if seed in calibration_seeds else test).append((group, score))
    if {s for s in calibration_seeds if 'seed:' + str(s) in counts} != calibration_seeds or {
            s for s in evaluation_seeds if 'seed:' + str(s) in counts} != evaluation_seeds:
        raise ValueError('declared seed group is missing')
    result = quantile(cal, coverage)
    threshold = result['scale']
    evaluation = {}
    for group, score in test:
        old = evaluation.get(group, 0.)
        evaluation[group] = None if old is None or score is None else max(old, score)
    return dict(calibration=result, evaluation_group_count=len(evaluation),
                group_case_counts=counts, evaluation_group_scores=evaluation,
                evaluation_covered_groups=(None if threshold is None else sum(
                    s is not None and s <= threshold for s in evaluation.values())),
                evaluation_status='unbounded_calibration' if threshold is None else 'descriptive_only',
                fresh_holdout=False, confidence_established=False)


def descriptive(values):
    if not values:
        return dict(count=0, minimum=None, median=None, p95=None, maximum=None)
    ordered = sorted(finite(x) for x in values)
    n = len(ordered)
    return dict(count=n, minimum=ordered[0], median=ordered[(n-1)//2],
                p95=ordered[(95*n+99)//100-1], maximum=ordered[-1])


def forecast_at(prediction, horizon):
    dt = finite(prediction['dt'])
    points = prediction['positions']
    extra = prediction.get('additional_radius_m', [])
    if dt <= 0 or len(points) < 2 or len(extra) != len(points):
        raise ValueError('incomplete forecast')
    if any(finite(v) < 0 for v in [prediction['radius'], *extra]):
        raise ValueError('negative forecast radius')
    if horizon > (len(points)-1)*dt + 1e-9:
        raise ValueError('forecast ends before the declared scoring horizon')
    step = horizon/dt
    index = min(int(math.floor(step)), len(points)-2)
    fraction = min(1., max(0., step-index))
    a, b = xy(points[index]), xy(points[index+1])
    center = tuple((1-fraction)*x + fraction*y for x, y in zip(a, b))
    additional = (1-fraction)*extra[index] + fraction*extra[index+1]
    return center, additional, index, fraction


def case_scores(run, config, ticks):
    """One joint score across acquired identities and all five fixed horizons.

    The origin is the first recorded prediction for an acquisition. Associations
    use that acquisition's physical frame once, never a nearest future actor.
    Only origins with a complete eight-second observation window are eligible.
    Unmatched/ambiguous origins and absent future actors give an infinite score.
    Late origins are counted as right-censored, rather than successful forecasts.
    """
    frames = run['frames']
    if not frames or len(frames) != len(ticks) or run['summary']['steps'] != len(ticks):
        raise ValueError('incomplete physical/control recording')
    for i, (frame, tick) in enumerate(zip(frames, ticks)):
        out = tick['expected']
        if abs(finite(frame['time'])-i*STEP) > 1e-8 or out['time'] != frame['time']:
            raise ValueError('physical/control clock mismatch')
        if frame['estimate'] != out['estimate']:
            raise ValueError('physical/control estimates differ')
    rows = {str(h): dict(eligible=0, matched=0, covered=0, unmatched=0, actor_absent=0,
                        _required=[], _published=[], _slack=[], _cv=[], _pose=[],
                        worst_utilization=None) for h in HORIZONS}
    seen = set()
    origins = censored = unmatched = absent = 0
    joint = 0.
    first_stopped = None
    for index, tick in enumerate(ticks):
        out = tick['expected']
        tracks = {t['id']: t for t in out['tracks']}
        if len(tracks) != len(out['tracks']):
            raise ValueError('duplicate track identity')
        identities = [p['id'] for p in out['predictions']]
        if len(set(identities)) != len(identities):
            raise ValueError('duplicate prediction identity')
        sigma = config.get('forecast_pose_sigma')
        covariance = out.get('pose_covariance')
        decomposition = factor(covariance) if sigma is not None and identities else None
        for prediction in out['predictions']:
            if prediction['id'] not in tracks:
                raise ValueError('forecast lacks acquisition ownership')
            track = tracks[prediction['id']]
            stamp = finite(track['last_seen'])
            age = out['time']-stamp
            if stamp < 0 or age < 0:
                raise ValueError('invalid forecast acquisition age')
            key = (track['id'], stamp)
            if key in seen:
                continue
            seen.add(key)
            acquired = round(stamp/STEP)
            if acquired > index or abs(frames[acquired]['time']-stamp) > 1e-8:
                raise ValueError('acquisition is not a recorded physical tick')
            if index + round(max(HORIZONS)/STEP) >= len(frames):
                censored += 1
                continue
            origins += 1
            candidates = sorted((math.dist(xy(track['position']), xy(o['position'])), o['id'])
                                for o in frames[acquired]['objects'])
            actor = candidates[0][1] if (candidates and candidates[0][0] <= 2.
                    and (len(candidates) == 1 or candidates[1][0]-candidates[0][0] >= .5)) else None
            if actor is None:
                unmatched += 1
                joint = None
            for horizon in HORIZONS:
                row = rows[str(horizon)]
                row['eligible'] += 1
                center, additional, knot, fraction = forecast_at(prediction, horizon)
                bounds = config.get('forecast_envelope')
                if bounds is not None:
                    elapsed = age+horizon
                    row['_cv'].append(finite(bounds['position_error_m'])
                        + elapsed*finite(bounds['velocity_error_m_s'])
                        + elapsed**2*finite(bounds['acceleration_error_m_s2'])/2)
                pose = 0.
                if sigma is not None:
                    ps = prediction['positions']
                    padding = [projected_padding(ps[k], out['estimate']['pose'], covariance, sigma, decomposition)
                               for k in (knot, knot+1)]
                    pose = (1-fraction)*padding[0]+fraction*padding[1]
                    row['_pose'].append(pose)
                row['_published'].append(additional)
                if actor is None:
                    row['unmatched'] += 1
                    continue
                future = index+round(horizon/STEP)
                actual = next((o for o in frames[future]['objects'] if o['id'] == actor), None)
                if actual is None:
                    row['actor_absent'] += 1
                    absent += 1
                    joint = None
                    continue
                row['matched'] += 1
                required = max(0., math.dist(center, xy(actual['position']))
                               + finite(actual['radius'])-prediction['radius'])
                score = required/additional if additional > 0. else (0. if required == 0. else None)
                row['_required'].append(required)
                row['_slack'].append(additional-required)
                row['covered'] += required <= additional+1e-8
                witness = dict(origin_s=out['time'], acquisition_s=stamp, prediction_id=track['id'],
                               actor_id=actor, required_extra_radius_m=required,
                               published_extra_radius_m=additional, world_pose_padding_m=pose, score=score)
                old = row['worst_utilization']
                if old is None or (old['score'] is not None and (score is None or score > old['score'])):
                    row['worst_utilization'] = witness
                if joint is not None:
                    joint = None if score is None else max(joint, score)
                diagnostic = out.get('planning_diagnostics') or {}
                selected = diagnostic.get('selected_candidate')
                if first_stopped is None and horizon == 8. and selected is not None:
                    candidate = diagnostic['candidates'][selected]
                    path = out['trajectory']['points']
                    if (not out['health'] and abs(out['estimate']['speed']) <= .2
                            and candidate['disposition'] == 'CheckedStop' and len(path) >= 2
                            and math.dist(xy(path[0]['position']), xy(path[-1]['position'])) <= .2):
                        first_stopped = dict(time_s=out['time'], route_progress_m=diagnostic['route_progress_m'],
                                             declared_cv_error_radius_m=row['_cv'][-1] if bounds else None,
                                             **witness)
    if not origins:
        joint = None
    for row in rows.values():
        for source, target in [('_required', 'required_extra_radius_m'), ('_published', 'published_extra_radius_m'),
                               ('_slack', 'unused_radius_m'), ('_cv', 'declared_cv_error_radius_m'),
                               ('_pose', 'world_pose_padding_m')]:
            row[target] = descriptive(row.pop(source))
        row['coverage_fraction'] = row['covered']/row['eligible'] if row['eligible'] else None
    return dict(joint_nonconformity=joint, joint_observed_covered=joint is not None and joint <= 1.,
                eligible_origins=origins, right_censored_origins=censored,
                unmatched_origins=unmatched, absent_actor_horizon_samples=absent,
                acquired_identities=len(seen), horizons=rows, first_stationary_checked_stop=first_stopped,
                original_summary=run['summary'], evaluation_only=True,
                association_uses_future=False, operational_policy_changed=False,
                confidence_established=False)
