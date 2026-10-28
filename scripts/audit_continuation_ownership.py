"""Independent log audit of continuation ownership; not a collision certificate."""
from inspect_continuation_probes import number


def audit(config, ticks):
    active = None
    last_time = -1.
    selected_count = adoption_count = overridden_count = 0
    for tick in ticks:
        out = tick['expected']
        now = number(out['time'])
        if now < 0 or now <= last_time:
            raise ValueError('strictly increasing nonnegative output clock required')
        last_time = now
        chosen = out.get('continuation')
        adoption = out.get('checked_approach') or {}
        if chosen is not None:
            if config.get('continue_checked_approaches') is not True or active is None:
                raise ValueError('selection without enabled live adoption')
            origin, record, epoch, acquisition = active
            if adoption.get('adopted'):
                raise ValueError('selection cannot also issue a new adoption')
            if out.get('localization_epoch', 0) != epoch:
                raise ValueError('selection crossed localization generation')
            if number(chosen['now_s']) != now or number(chosen['expires_s']) != origin + 8.:
                raise ValueError('selection changed its clock or immutable expiry')
            stamp = number(chosen['acquisition_s'])
            if not acquisition <= stamp <= now or now-stamp > .35 + 1e-9 or now >= origin+8.:
                raise ValueError('selection has stale, regressed or expired authority')
            for actual, source in [('original_endpoint','proposal_endpoint'),
                                   ('original_end_progress_m','proposal_end_progress_m'),
                                   ('lateral_target_m','proposal_lateral_target_m'),
                                   ('blocking_ids','blocking_prediction_ids')]:
                if chosen[actual] != record[source]:
                    raise ValueError('selection changed adopted '+actual)
            if number(chosen['selected_end_progress_m']) > number(record['proposal_end_progress_m']) + 1e-8:
                raise ValueError('selected stopping endpoint exceeds adopted bound')
            selected_count += 1
            active = (origin, record, epoch, stamp)
        else:
            # Even a single intervening ordinary plan ends selection authority.
            active = None
            if adoption.get('adopted') is True:
                first = number(adoption['first_acquisition_s'])
                last = number(adoption['last_acquisition_s'])
                if (type(adoption['supporting_acquisitions']) is not int
                    or adoption['supporting_acquisitions'] != 3
                    or first < 0 or last-first < .2-1e-9 or last > now
                    or now-last > .35+1e-9):
                    raise ValueError('adoption lacks three fresh acquisition support')
                adoption_count += 1
                active = (now, adoption, out.get('localization_epoch', 0), last)
        if out['emergency'] or out['health']:
            overridden_count += chosen is not None
            active = None
    if not ticks:
        raise ValueError('nonempty recording required')
    return dict(passed=True, selected_ticks=selected_count, adoptions=adoption_count,
        selected_guarded_ticks=overridden_count,
        scope='Recorded adoption ownership, clocks, frame and immutable bounds only; no independent geometry/collision certificate.')
