#!/usr/bin/env python3
"""Exploratory reference-plant continuation comparison, not adoption qualification."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import check_later_stops as parent

ROOT = Path(__file__).resolve().parents[1]
SOURCE = '75d6c3fe11b5194ba522708f23a3a5c3eb43de46'
KEY = 'continue_checked_approaches'

def sha(raw):
    return hashlib.sha256(raw).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.absolute()
    output.mkdir(parents=True, exist_ok=False)
    binary = ROOT/'target/release/rustdriving'
    fingerprint = sha(binary.read_bytes())
    canonical = json.loads((ROOT/'assets/small-approaches'/(SOURCE+'.json')).read_bytes())
    if canonical['source'] != SOURCE or not canonical['complete'] or not canonical['contract_passed']:
        raise ValueError('qualified frozen control contract required')
    controls = {c['seed']:c['candidate'] for c in canonical['cases'] if c['plant']=='reference-2d'}
    specs = [s for s in parent.specifications(['reference-2d']) if s['world']=='mission']
    if len(specs) != 2 or {s['seed'] for s in specs} != {7,42}:
        raise ValueError('unexpected reference mission cohort')
    report = dict(source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),closed_loop=True,
        exploratory=True,adoption_passed=False,full_cohort_qualified=False,
        independent_continuation_certificates_verified=False,complete=False,cases=[])
    def save():
        (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    save()
    for spec in specs:
        _, baseline = parent.materialize(spec)
        baseline['checked_approach_min_progress_m'] = 0.5
        if KEY in baseline:
            raise ValueError('frozen control already enables continuation')
        pair = dict(seed=spec['seed'],plant='reference-2d')
        original_config = None
        for variant in ['baseline','continuation']:
            scenario = copy.deepcopy(baseline)
            if variant == 'continuation': scenario[KEY] = True
            folder = output/f"seed-{spec['seed']}"/variant
            folder.mkdir(parents=True)
            source = folder/'scenario.json'
            source.write_text(json.dumps(scenario,indent=2)+'\n')
            run = folder/'run'
            command = subprocess.run([str(binary),'run','--scenario',str(source),
                '--seed',str(spec['seed']),'--output',str(run)],capture_output=True,text=True)
            (folder/'run.stdout').write_text(command.stdout)
            (folder/'run.stderr').write_text(command.stderr)
            if command.returncode not in [0,1]:
                raise ValueError('simulation execution failed: '+command.stderr)
            summary = json.loads((run/'summary.json').read_bytes())
            if command.returncode != (0 if summary['passed'] else 1):
                raise ValueError('simulation exit disagrees with summary')
            raw = (run/'sensors.jsonl').read_bytes()
            records = [json.loads(line) for line in raw.splitlines()]
            ticks = [r['tick'] for r in records if r['kind']=='tick']
            if records[-1] != dict(kind='end',ticks=len(ticks)) or len(ticks)!=summary['steps']:
                raise ValueError('incomplete fresh sensor recording')
            config = records[0]['header']['config']
            if variant == 'baseline':
                if sha(raw) != controls[spec['seed']]['storage']['run/sensors.jsonl']['sha256']:
                    raise ValueError('disabled policy changed frozen control outputs')
                original_config = config
            elif config != dict(original_config,**{KEY:True}):
                raise ValueError('another driver setting changed')
            replay = subprocess.run([str(binary),'replay','--log',str(run/'sensors.jsonl'),
                '--output',str(folder/'replay')],capture_output=True,text=True)
            (folder/'replay.stdout').write_text(replay.stdout)
            (folder/'replay.stderr').write_text(replay.stderr)
            if replay.returncode != 0:
                raise ValueError('fresh sensor replay failed: '+replay.stderr)
            replay_report = json.loads((folder/'replay/replay.json').read_bytes())
            if replay_report.get('verified') is not True or replay_report.get('ticks') != len(ticks):
                raise ValueError('replay report did not verify every tick')
            chosen = [t['expected'] for t in ticks if t['expected'].get('continuation') is not None]
            pair[variant] = dict(summary=summary,sensor_sha256=sha(raw),verified_ticks=len(ticks),
                selected_ticks=len(chosen),
                selected_positive_acceleration_ticks=sum(o['command']['acceleration']>0 for o in chosen),
                selected_emergency_ticks=sum(o['emergency'] for o in chosen),
                run_exit_code=command.returncode,replay_exit_code=replay.returncode)
        report['cases'].append(pair)
        save()
        print(json.dumps(pair,allow_nan=False),flush=True)
    if sha(binary.read_bytes()) != fingerprint:
        raise ValueError('binary changed during comparison')
    report.update(complete=True,binary_sha256=fingerprint)
    save()

if __name__=='__main__':
    main()
