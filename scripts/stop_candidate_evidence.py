#!/usr/bin/env python3
"""Classify complete stop sweeps without changing their admission rules."""
import math
from collections import Counter

def classify(attempt):
    start,end,gain=(attempt[k] for k in ['hold_start_s','hold_end_s','progress_gain_m'])
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in [start,end,gain]):
        raise ValueError('nonfinite stop evidence')
    if start<0 or end-start<8.-1e-9:
        raise ValueError('missing complete stationary tail')
    contact=attempt['first_contact_s']
    if contact is None:
        return 'clear_at_least_2m' if gain>=2. else 'clear_below_2m'
    if type(contact) not in (int,float) or not math.isfinite(contact) or not 0<=contact<=end:
        raise ValueError('invalid contact clock')
    return 'moving_conflict' if contact<start else 'stationary_tail_conflict'

def summarize(attempts):
    if not attempts or len(attempts)>1000:
        raise ValueError('missing or unbounded stop search')
    counts=Counter(classify(a) for a in attempts)
    clear=[a['progress_gain_m'] for a in attempts if a['first_contact_s'] is None]
    return dict(sweep_counts=dict(counts),maximum_clear_progress_gain_m=max(clear) if clear else None,
                global_infeasibility_proven=False,confirmation_authorized=False)
