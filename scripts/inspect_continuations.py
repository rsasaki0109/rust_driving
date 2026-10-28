#!/usr/bin/env python3
"""Replay the frozen paired sensor cohort and inspect unused continuation candidates."""
import argparse
import hashlib
import json
from pathlib import Path
import lzma
import subprocess
import tempfile
from check_forecast_calibration import read_retained
from inspect_continuation_probes import inspect

ROOT = Path(__file__).resolve().parents[1]
SOURCE = '75d6c3fe11b5194ba522708f23a3a5c3eb43de46'
CANONICAL = ROOT/'assets/small-approaches'/(SOURCE+'.json')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    canonical = CANONICAL.read_bytes()
    if (args.evidence/'report.json').read_bytes() != canonical:
        raise ValueError('paired report differs from frozen source')
    cohort = json.loads(canonical)
    if cohort['source'] != SOURCE or not cohort['complete'] or not cohort['contract_passed']:
        raise ValueError('incomplete or unexpected paired report')
    cases = cohort['cases']
    if len(cases) != 4 or {(c['plant'], c['seed']) for c in cases} != {
        (p, s) for p in ['reference-2d', 'rne-dynamic'] for s in [7, 42]
    }:
        raise ValueError('unexpected paired cohort')
    args.output.mkdir(parents=True, exist_ok=False)
    subprocess.run(['cargo', 'build', '--release', '--locked', '-p',
        'rustdriving-pipeline', '--example', 'inspect_continuations'], cwd=ROOT, check=True)
    report = dict(
        source=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        recorded_driver_source=SOURCE,
        paired_report_sha256=hashlib.sha256(canonical).hexdigest(),
        closed_loop=False, operational_policy_changed=False,
        interpretation='Post-original-plan shadow candidates on unchanged sensor recordings. Candidate availability is not closed-loop progress or mission recovery.',
        cases=[])
    with tempfile.TemporaryDirectory(prefix='continuation-input-') as temp:
        for case in cases:
            for variant, folder in [('baseline', 'baseline'), ('candidate', 'new')]:
                row = case[variant]
                sensors = read_retained(args.evidence/folder, row, 'run/sensors.jsonl')
                name = f"{case['plant']}-{case['seed']}-{variant}"
                source = Path(temp)/(name+'.jsonl')
                source.write_bytes(sensors)
                output = args.output/(name+'.jsonl')
                completed = subprocess.run([str(ROOT/'target/release/examples/inspect_continuations'),
                    str(source), str(output)], cwd=ROOT, check=True, capture_output=True, text=True)
                summary = json.loads(completed.stdout)
                if (summary['kind'] != 'end' or not summary['legacy_verified']
                    or summary['closed_loop'] or summary['ticks'] != row['summary']['steps']):
                    raise ValueError('invalid or incomplete replay result')
                raw = output.read_bytes()
                if json.loads(raw.splitlines()[-1]) != summary:
                    raise ValueError('inspection footer differs from stdout')
                metrics = inspect([json.loads(line) for line in raw.splitlines()])
                encoded = lzma.compress(raw)
                packed = output.with_suffix(output.suffix+'.xz')
                packed.write_bytes(encoded)
                output.unlink()
                entry = dict(plant=case['plant'], seed=case['seed'], variant=variant,
                    sensor_sha256=hashlib.sha256(sensors).hexdigest(), summary=summary,
                    comparative_audit=metrics,
                    inspection=dict(path=packed.name, encoding='xz',
                        sha256=hashlib.sha256(raw).hexdigest(),
                        stored_sha256=hashlib.sha256(encoded).hexdigest()))
                report['cases'].append(entry)
                print(json.dumps(entry, allow_nan=False), flush=True)
    report['complete'] = len(report['cases']) == 8
    with (args.output/'report.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')

if __name__ == '__main__':
    main()
