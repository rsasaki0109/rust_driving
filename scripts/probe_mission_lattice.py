#!/usr/bin/env python3
"""Retain an unchanged viewed mission and exact acquired planning checkpoints."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import check_later_stops as study
from inspect_planning import audit as inspect

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    output = ROOT / 'artifacts/mission-lattice-probe'
    output.mkdir(parents=True, exist_ok=False)
    cases = study.specifications(['reference-2d'])
    missions = [s for s in cases if 'mission' in s.get('world', '') and s['seed'] == 7]
    if not missions:
        raise ValueError('Missing original mission seed 7: ' + repr(cases))
    spec = missions[0]
    original_path, scenario = study.materialize(spec)
    if scenario.get('record_planning_diagnostics') is not True:
        raise ValueError('Original cohort does not already capture planning decisions')
    frozen = output / 'scenario.json'
    frozen.write_bytes((json.dumps(scenario, indent=2, allow_nan=False) + '\n').encode('utf-8'))
    binary = ROOT / 'target/release/rustdriving'
    binary_sha = digest(binary)
    commands = []
    for arguments, folder, expected in [
            (['run', '--scenario', str(frozen), '--seed', str(spec['seed'])], 'run', 1),
            (['replay', '--log', str(output / 'run/sensors.jsonl')], 'replay', 0)]:
        command = [str(binary), *arguments, '--output', str(output / folder)]
        result = subprocess.run(command, capture_output=True, text=True)
        (output / (folder + '-process.log')).write_text(result.stdout + result.stderr, encoding='utf-8')
        commands.append(dict(command=command, exit_code=result.returncode))
        if result.returncode != expected:
            raise ValueError('Unexpected baseline native exit: ' + repr(commands[-1]) + result.stderr)
    run = json.loads((output / 'run/run.json').read_text())
    replay = json.loads((output / 'replay/replay.json').read_text())
    if replay.get('verified') is not True or run['summary']['reached_goal'] is not False:
        raise ValueError('Original incomplete mission was not retained')
    config = None
    ticks = []
    with (output / 'run/sensors.jsonl').open() as stream:
        for line in stream:
            record = json.loads(line)
            if 'header' in record:
                config = record['header']['config']
            elif 'tick' in record:
                ticks.append(record['tick'])
    if not config or len(ticks) != run['summary']['steps'] or len(ticks) != len(run['frames']):
        raise ValueError('Incomplete control/physical records')
    inspection = inspect(config, ticks)
    checkpoints = []
    for requested in [4., 6., 8., 10., 12., 20., 40., 64.]:
        index = min(range(len(ticks)), key=lambda i: abs(ticks[i]['input']['time'] - requested))
        tick = ticks[index]
        if abs(tick['input']['time'] - requested) > .026:
            raise ValueError('Missing checkpoint')
        out = tick['expected']
        checkpoints.append(dict(
            time_s=tick['input']['time'], estimate=out['estimate'],
            tracks=out['tracks'], predictions=out['predictions'],
            trajectory=out['trajectory'], command=out['command'],
            health=out['health'], planning=out['planning_diagnostics'],
            physical=run['frames'][index]))
    if digest(binary) != binary_sha:
        raise ValueError('Native binary changed during baseline')
    result = dict(
        source_commit=os.environ['RUSTDRIVING_SOURCE_COMMIT'],
        probe_commit=os.environ['GITHUB_SHA'],
        source_diff=subprocess.check_output(['git', 'diff', '--name-only',
            os.environ['RUSTDRIVING_SOURCE_COMMIT'], 'HEAD'], text=True).splitlines(),
        selected_spec=spec, original_source=original_path,
        frozen_scenario=scenario, frozen_sha256=digest(frozen),
        operational_config=config, binary_sha256=binary_sha,
        commands=commands, summary=run['summary'], replay=replay,
        inspection=inspection, checkpoints=checkpoints,
        interpretation='An unchanged viewed authored reference simulation; no new policy, relaxed gate or mission completion.')
    target = ROOT / 'assets/mission-lattice-probe/baseline.json'
    target.parent.mkdir(parents=True, exist_ok=False)
    target.write_bytes((json.dumps(result, indent=2, allow_nan=False) + '\n').encode('utf-8'))
    print(json.dumps(dict(selected_spec=spec, summary=run['summary'],
        inspection=inspection, checkpoint_times=[p['time_s'] for p in checkpoints],
        output=str(target.relative_to(ROOT)), bytes=target.stat().st_size)), flush=True)

if __name__ == '__main__':
    main()
