#!/usr/bin/env python3
"""Reinspect every retained directional-reserve case without rerunning a simulator."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path

from check_forecast_envelope import coverage
from forecast_calibration import case_scores, grouped_split

ROOT = Path(__file__).resolve().parents[1]
SOURCE = '8550e1a2b1d06703d05036a625609e21b9fa0cf6'
CANONICAL = ROOT/'assets/directional-reserve-v1/canaries'/(SOURCE+'.json')
CALIBRATION_SEEDS = [7]
EVALUATION_SEEDS = [42]


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def read_retained(root, row, key):
    entry = row['storage'][key]
    relative = Path(row['recordings'])/entry['path']
    if relative.is_absolute() or '..' in relative.parts or entry['path'] != key+'.xz' or entry['encoding'] != 'xz':
        raise ValueError('unexpected retained recording path')
    path = root/relative
    if path.is_symlink() or any(p.is_symlink() for p in path.parents):
        raise ValueError('retained recording aliases are not allowed')
    encoded = path.read_bytes()
    if digest(encoded) != entry['stored_sha256']:
        raise ValueError('stored recording hash changed')
    raw = lzma.decompress(encoded)
    if digest(raw) != entry['sha256']:
        raise ValueError('raw recording hash changed')
    return raw


def inspect(evidence):
    evidence = evidence.absolute()
    raw_report = (evidence/'report.json').read_bytes()
    if raw_report != CANONICAL.read_bytes():
        raise ValueError('evidence differs from the first canonical paired report')
    paired = json.loads(raw_report)
    rows = paired['executions']['new']
    if paired['source_commit'] != SOURCE or not paired['complete'] or len(rows) != 48:
        raise ValueError('incomplete or wrong-source cohort')
    report = dict(schema_version=1, recorded_driver_source=SOURCE,
                  paired_report_sha256=digest(raw_report), simulation_rerun=False,
                  operational_policy_changed=False, confidence_established=False,
                  original_full_adoption_gate_passed=paired['original_full_adoption_gate_passed'],
                  protocol=dict(horizons_s=[.2, 1., 2., 4., 8.], dependence_group='seed across worlds and plants',
                      calibration_seeds=CALIBRATION_SEEDS, evaluation_seeds=EVALUATION_SEEDS,
                      target_coverage='0.99', fresh_holdout=False, origins='first prediction per acquired identity with complete eight-second follow-up',
                      score='maximum required/published additional disc radius over every eligible origin and horizon',
                      unresolved_score='infinity, serialized as null; never omitted'),
                  complete=False, cases=[])
    original_coverage = True
    for row in rows:
        if row.get('audit_error') or not row['replay']['verified'] or row['replay']['ticks'] != row['summary']['steps']:
            raise ValueError('original case lacks complete replay evidence')
        root = evidence/'new'
        frozen = (root/row['scenario']).read_bytes()
        if digest(frozen) != row['sha256']:
            raise ValueError('frozen input hash changed')
        raw_run = read_retained(root, row, 'run/run.json')
        raw_sensors = read_retained(root, row, 'run/sensors.jsonl')
        run = json.loads(raw_run)
        records = [json.loads(line) for line in raw_sensors.splitlines()]
        if not records or 'header' not in records[0]:
            raise ValueError('missing sensor-only header')
        config = records[0]['header']['config']
        ticks = [record['tick'] for record in records if 'tick' in record]
        if run['summary'] != row['summary']:
            raise ValueError('original outcome changed')
        retained_coverage = coverage(run, ticks)
        if retained_coverage != row['coverage']:
            raise ValueError('original per-horizon coverage changed')
        original_coverage &= retained_coverage['passed']
        result = case_scores(run, config, ticks)
        case_id = row['plant']+'/'+row['world']+'/'+row['mode']+'/seed-'+str(row['seed'])
        result.update(case_id=case_id, seed=row['seed'], plant=row['plant'], world=row['world'],
                      original_failed_gates=row['failed_gates'], original_coverage=retained_coverage,
                      frozen_input_sha256=row['sha256'],
                      recording_sha256=dict(run=digest(raw_run), sensors=digest(raw_sensors)))
        report['cases'].append(result)
        print(row['plant'], row['world'], row['seed'], 'origins', result['eligible_origins'],
              'joint_score', result['joint_nonconformity'], 'goal', row['summary']['reached_goal'], flush=True)
    report.update(complete=True, cases_verified=len(report['cases']),
                  original_coverage_passed=original_coverage,
                  strict_passes=sum(r['passed'] for r in rows), arrivals=sum(r['summary']['reached_goal'] for r in rows),
                  grouped_calibration=grouped_split(report['cases'], CALIBRATION_SEEDS, EVALUATION_SEEDS))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists() or args.output.is_symlink() or any(p.is_symlink() for p in args.output.absolute().parents):
        raise ValueError('output must be a new file with real parents')
    result = inspect(args.evidence)
    encoded = (json.dumps(result, indent=2, allow_nan=False)+'\n').encode()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('xb') as stream:
        stream.write(encoded)
    print(json.dumps(dict(complete=result['complete'], cases=result['cases_verified'],
                         calibration=result['grouped_calibration'], strict_passes=result['strict_passes'],
                         arrivals=result['arrivals']), allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
