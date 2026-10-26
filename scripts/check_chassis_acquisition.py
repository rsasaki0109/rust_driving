#!/usr/bin/env python3
"""Compare declared native body-reference calibration with original recorded gates."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

import check_later_stops as parent
from check_forecast_calibration import CANONICAL, read_retained
from check_gnss_antenna import archive
from forecast_calibration import case_scores, descriptive

ROOT = Path(__file__).resolve().parents[1]
BASELINE = '6fd7e8a1e8f87e8effce36ce27449bafed393aa3'
RECORDED = '8550e1a2b1d06703d05036a625609e21b9fa0cf6'

def sha(raw):
    return hashlib.sha256(raw).hexdigest()

def load_recording(root, row):
    raw_run = read_retained(root, row, 'run/run.json')
    raw_sensors = read_retained(root, row, 'run/sensors.jsonl')
    run = json.loads(raw_run)
    records = [json.loads(line) for line in raw_sensors.splitlines()]
    config = records[0]['header']['config']
    ticks = [r['tick'] for r in records if 'tick' in r]
    if run['summary'] != row['summary'] or len(ticks) != row['summary']['steps']:
        raise ValueError('recorded outcome or tick coverage changed')
    return run, config, ticks, raw_sensors

def fixed_inputs(evidence, rows, output, example):
    results = []
    with tempfile.TemporaryDirectory(prefix='rustdriving-chassis-inputs-') as temporary:
        temp = Path(temporary)
        for row in rows:
            if row['plant'] != 'rne-dynamic' or row['world'] != 'mission':
                continue
            run, config, ticks, raw = load_recording(evidence/'new', row)
            source = temp/('native-'+str(row['seed'])+'.jsonl')
            source.write_bytes(raw)
            destination = output/('native-'+str(row['seed'])+'.jsonl')
            completed = subprocess.run([str(example), str(source), str(destination), '1.5'],
                                       capture_output=True, text=True)
            if completed.returncode:
                raise ValueError('fixed-input pipeline inspection failed: '+completed.stderr)
            with destination.open() as stream:
                records = [json.loads(line) for line in stream]
            header, footer = records[0], records[-1]
            if footer != dict(kind='end', ticks=len(ticks), legacy_verified=True, closed_loop=False):
                raise ValueError('counterfactual lost original replay or full tick coverage')
            expected = dict(config, rear_axle_offset_m=1.5)
            if header['original_config'] != config or header['counterfactual_config'] != expected:
                raise ValueError('counterfactual changed more than body-reference calibration')
            actual = [r['counterfactual'] for r in records[1:-1]]
            if len(actual) != len(ticks) or source.read_bytes() != raw:
                raise ValueError('counterfactual changed the source observations')
            scores = {label: [] for label in ['old', 'calibrated']}
            focus = {}
            seen = {label: set() for label in scores}
            for index, (tick, corrected) in enumerate(zip(ticks, actual)):
                if corrected['time'] != tick['expected']['time']:
                    raise ValueError('counterfactual changed control chronology')
                for label, out in [('old', tick['expected']), ('calibrated', corrected)]:
                    for forecast in out['predictions']:
                        if forecast['id'] != 1:
                            raise ValueError('fixed original mission identity changed')
                        track = next(t for t in out['tracks'] if t['id'] == 1)
                        key = (track['id'], track['last_seen'])
                        if key in seen[label]:
                            continue
                        seen[label].add(key)
                        future = index+160
                        if future >= len(run['frames']):
                            continue
                        actor = next((o for o in run['frames'][future]['objects'] if o['id'] == 0), None)
                        if actor is None:
                            raise ValueError('fixed original physical actor disappeared')
                        if forecast['dt'] != .2 or len(forecast['positions']) != 41:
                            raise ValueError('fixed eight-second forecast grid changed')
                        endpoint = forecast['positions'][40]
                        required = max(0., math.dist([endpoint[k] for k in ['x', 'y']],
                                                    [actor['position'][k] for k in ['x', 'y']])
                                       +actor['radius']-forecast['radius'])
                        scores[label].append(required)
                        if abs(out['time']-5.) < 1e-9:
                            focus[label] = dict(track=track, forecast_endpoint=endpoint,
                                                required_extra_radius_m=required,
                                                published_extra_radius_m=forecast['additional_radius_m'][40])
            summary = {label: descriptive(values) for label, values in scores.items()}
            passed = (summary['calibrated']['count'] == summary['old']['count']
                      and summary['calibrated']['maximum'] <= summary['old']['maximum'] + 1e-8
                      and (row['seed'] != 7 or summary['calibrated']['maximum'] <= .5*summary['old']['maximum']))
            results.append(dict(seed=row['seed'], input_sha256=sha(raw), required_eight_second_radius_m=summary,
                                at_five_seconds=focus, passed=passed, closed_loop=False,
                                original_replay_verified=True, actor_truth_evaluation_only=True))
    return dict(passed=all(r['passed'] for r in results) and len(results)==2, cases=results)

def contract(row):
    return (not row.get('audit_error') and row.get('run_exit_code') == (
        0 if row.get('summary',{}).get('passed') else 1)
        and row.get('replay_exit_code') == 0 and row.get('replay',{}).get('verified') is True
        and row['replay']['ticks'] == row['summary']['steps'])

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--native-binary', required=True, type=Path)
    parser.add_argument('--example-binary', required=True, type=Path)
    parser.add_argument('--stage', choices=['canary', 'full'], default='canary')
    args = parser.parse_args()
    output = args.output.absolute()
    if output.exists() or output.is_symlink() or any(p.is_symlink() for p in output.parents):
        raise ValueError('new evidence needs a real create-only destination')
    output.mkdir(parents=True)
    evidence = args.evidence.absolute()
    if (evidence/'report.json').read_bytes() != CANONICAL.read_bytes():
        raise ValueError('baseline differs from the first canonical paired recording')
    baseline = json.loads(CANONICAL.read_bytes())
    if baseline['source_commit'] != RECORDED or not baseline['complete']:
        raise ValueError('wrong baseline recording source')
    old_rows = baseline['executions']['new']
    specs = parent.specifications(['reference-2d', 'rne-dynamic'])
    manifest = parent.freeze(output/'new', specs, factory=parent.materialize, gates=parent.GATES)
    if len(manifest['episodes']) != len(old_rows) or len(old_rows) != 48:
        raise ValueError('complete original cohort changed')
    for spec, row in zip(manifest['episodes'], old_rows):
        if any(spec[k] != row[k] for k in ['scenario', 'sha256', 'plant', 'world', 'mode', 'seed']):
            raise ValueError('frozen original inputs changed')
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    fingerprints = {key: sha(path.read_bytes()) for key, path in binaries.items()}
    report = dict(source=os.environ['RUSTDRIVING_TESTED_SOURCE'], baseline_source=BASELINE,
                  baseline_recorded_source=RECORDED, stage=args.stage, complete=False, passed=False,
                  binaries=fingerprints, baseline_report_sha256=sha(CANONICAL.read_bytes()),
                  physics_noise_goals_deadlines_budgets_changed=False, cases=[],
                  default_policy_changed=False, new_scenario_flag=False, truth_operational_input=False)
    def save():
        (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    save()
    try:
        report['fixed_inputs'] = fixed_inputs(evidence, old_rows, output, args.example_binary.resolve())
    except Exception as error:
        report['fixed_inputs'] = dict(passed=False, complete=False, error=repr(error))
        save()
        raise
    save()
    selected = [(index, spec) for index, spec in enumerate(manifest['episodes'])
                if args.stage == 'full' or (spec['plant']=='rne-dynamic' and spec['world']=='mission' and spec['seed']==7)]
    def one(item):
        index, spec = item
        folder = output/'new'/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        previous_raw = read_retained(evidence/'new', old_rows[index], 'run/sensors.jsonl')
        previous_config = json.loads(previous_raw.splitlines()[0])['header']['config']
        def audit(run, config, ticks, s, frozen):
            result = parent.audit(run, config, ticks, dict(s, _folder=str(folder)), frozen, verify_directions=True)
            if s['plant']=='rne-dynamic':
                if config.get('rear_axle_offset_m') != 1.5:
                    raise ValueError('native chassis calibration was not recorded')
                old_config, new_config = dict(previous_config), dict(config)
                old_config.pop('rear_axle_offset_m', None)
                new_config.pop('rear_axle_offset_m', None)
                if old_config != new_config:
                    raise ValueError('calibration changed road geometry, limits or another driver setting')
            result['forecast_radius_use'] = case_scores(run, config, ticks)
            result['healthy_emergency_transitions'] = sum(
                a['expected']['emergency'] != b['expected']['emergency'] for a, b in zip(ticks,ticks[1:])
                if not a['expected']['health'] and not b['expected']['health'])
            return result
        row = parent.run_case(output/'new', spec, binaries[spec['plant']], binaries['reference-2d'], auditor=audit)
        old = old_rows[index]
        row['comparison'] = (parent.compare(row, old) if contract(row) else
                             dict(regression_passed=False, new_failed_gates=['audit_error']))
        if spec['plant']=='reference-2d' and not row.get('audit_error'):
            row['reference_bytes_identical'] = all(row['storage'][k]['sha256'] == old['storage'][k]['sha256']
                                                   for k in ['run/run.json', 'run/sensors.jsonl', 'run/summary.json'])
        return row
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one, selected):
            report['cases'].append(row)
            save()
            print(row['plant'], row['world'], row['seed'], 'goal', row.get('summary',{}).get('reached_goal'),
                  'emergency', row.get('summary',{}).get('emergency_steps'),
                  'audit', row.get('audit_error'), 'gates', row.get('failed_gates'),
                  'new', row['comparison']['new_failed_gates'], flush=True)
    report['complete'] = len(report['cases']) == len(selected)
    report['passed'] = (report['complete'] and report['fixed_inputs']['passed']
        and all(contract(r) and r['comparison']['regression_passed']
                and r.get('replay',{}).get('verified') and r.get('directional_certificates',{}).get('passed')
                and (r['plant'] != 'reference-2d' or r.get('reference_bytes_identical'))
                for r in report['cases']))
    report['full_qualification_complete'] = False
    if args.stage == 'full':
        report['passed'] = False  # Historical replay and default controls must also qualify.
    report['strict_passes'] = sum(r.get('passed',False) for r in report['cases'])
    report['arrivals'] = sum(r.get('summary',{}).get('reached_goal',False) for r in report['cases'])
    report['original_full_adoption_gate_passed'] = (args.stage=='full' and report['passed']
                                                    and report['strict_passes']==48)
    if fingerprints != {key:sha(path.read_bytes()) for key,path in binaries.items()}:
        raise ValueError('driver changed during evaluation')
    save()
    print(json.dumps({k:report[k] for k in ['complete','passed','stage','strict_passes','arrivals',
          'original_full_adoption_gate_passed']}),flush=True)
    raise SystemExit(0 if report['passed'] else 1)

if __name__=='__main__':
    main()
