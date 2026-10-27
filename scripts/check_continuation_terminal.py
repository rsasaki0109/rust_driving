#!/usr/bin/env python3
"""Inspect new terminal stop proposals in frozen continuation runs; no driving changes."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from stop_candidate_evidence import summarize
from inspect_forecast_corridor import inspect as inspect_corridor

SOURCE = 'd73f28a4cc0ad0d809a0d6a0a8029b3030aba153'
ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--native', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    binary = ROOT/'target/release/examples/inspect_stop_candidates'
    binary_hash = digest(binary)
    result = dict(analysis_source=subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        recorded_source=SOURCE, simulation_rerun=False, operational_policy_changed=False,
        global_infeasibility_proven=False, independent_collision_certification=False,
        complete=False, cases=[])
    def save():
        (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    save()
    for plant, root in [('reference-2d', args.reference), ('rne-dynamic', args.native)]:
        manifest = json.loads((root/'report.json').read_bytes())
        rows = manifest['cases']
        if (manifest['source'] != SOURCE or manifest['complete'] is not True
            or len(rows) != 2 or {(r['plant'], r['seed']) for r in rows} != {(plant,7),(plant,42)}):
            raise ValueError('wrong or incomplete retained paired cohort')
        for row in rows:
            retained = row['continuation']
            source = root/f"seed-{row['seed']}"/'continuation/run/sensors.jsonl'
            sensor_hash = digest(source)
            if sensor_hash != retained['sensor_sha256']:
                raise ValueError('retained sensor hash mismatch')
            records = [json.loads(line) for line in source.read_bytes().splitlines()]
            ticks = [r['tick'] for r in records if r['kind'] == 'tick']
            if (not ticks or records[-1] != dict(kind='end', ticks=len(ticks))
                or len(ticks) != retained['summary']['steps']
                or retained['verified_ticks'] != len(ticks)):
                raise ValueError('incomplete retained recording')
            config = records[0]['header']['config']
            if config.get('continue_checked_approaches') is not True or config.get('checked_approach_min_progress_m') != .5:
                raise ValueError('unexpected recorded continuation policy')
            target_time = ticks[-1]['input']['time']
            target = args.output/f"{plant}-{row['seed']}-terminal.jsonl"
            subprocess.run([str(binary), str(source.resolve()), str(target.resolve()), str(target_time)], check=True)
            probes = [json.loads(line) for line in target.read_bytes().splitlines()]
            if (len(probes) != 3 or probes[0] != dict(kind='configuration', config=config, closed_loop=False)
                or probes[-1] != dict(kind='end', ticks=len(ticks), legacy_verified=True, closed_loop=False)
                or probes[1]['kind'] != 'probe' or probes[1]['time'] != target_time
                or probes[1]['original'] != ticks[-1]['expected']):
                raise ValueError('probe did not reproduce complete original recording')
            probe = probes[1]['probe']
            attempts = probe['attempts']
            # Preserve the older classifier's explicitly 2 m categories; count
            # the actual experimental 0.5 m admission threshold separately.
            search = summarize(attempts) if attempts else dict(
                sweep_counts={}, maximum_clear_progress_gain_m=None,
                global_infeasibility_proven=False, confirmation_authorized=False)
            search['attempt_count'] = len(attempts)
            search['configured_minimum_progress_m'] = config['checked_approach_min_progress_m']
            search['clear_at_configured_minimum'] = sum(
                a['first_contact_s'] is None and a['progress_gain_m'] >= config['checked_approach_min_progress_m']
                for a in attempts)
            diagnostic = probe['planning_diagnostics']
            chosen = diagnostic['selected_candidate']
            selected = None if chosen is None else diagnostic['candidates'][chosen]
            receipt = dict(plant=plant, seed=row['seed'], time_s=target_time,
                verified_original_ticks=len(ticks), sensor_sha256=sensor_hash,
                retained_report_sha256=digest(root/'report.json'), probe_sha256=digest(target),
                probe_path=target.name, search=search, selected=selected,
                initial_progress_m=probe['initial_progress_m'],
                returned_endpoint=(probe['trajectory']['points'][-1]['position'] if probe['trajectory']['points'] else None),
                original_checked_approach=ticks[-1]['expected'].get('checked_approach'),
                candidates=diagnostic['candidates'])
            alternative = probe['point_reference_recovery_probe']
            alternate_attempts = alternative['attempts']
            alternate_search = summarize(alternate_attempts) if alternate_attempts else dict(
                sweep_counts={}, maximum_clear_progress_gain_m=None,
                global_infeasibility_proven=False, confirmation_authorized=False)
            alternate_search.update(attempt_count=len(alternate_attempts),
                configured_minimum_progress_m=config['checked_approach_min_progress_m'],
                clear_at_configured_minimum=sum(a['first_contact_s'] is None
                    and a['progress_gain_m'] >= config['checked_approach_min_progress_m']
                    for a in alternate_attempts))
            metadata = probe['geometry_comparison']
            if (metadata['original_point_reference_recovery'] != (config.get('rear_axle_offset_m') is None)
                or metadata['local_route_geometry'] != config.get('local_route_geometry', False)
                or metadata['policy_changed'] != (config.get('rear_axle_offset_m') is not None)
                or metadata['probe_point_reference_recovery'] is not True
                or metadata['closed_loop'] is not False):
                raise ValueError('geometry probe policy provenance mismatch')
            alternate_diagnostic = alternative['planning_diagnostics']
            alternate_index = alternate_diagnostic['selected_candidate']
            receipt['geometry_comparison'] = dict(
                policy_changed=metadata['policy_changed'],
                local_route_geometry=metadata['local_route_geometry'],
                operational_policy_changed=False,
                point_reference_recovery=dict(search=alternate_search,
                    selected=None if alternate_index is None else alternate_diagnostic['candidates'][alternate_index],
                    candidates=alternate_diagnostic['candidates']))
            scales = probe['join_scale_probes']
            if [r['scale'] for r in scales] != [.5, 1., 2.]:
                raise ValueError('join scale cohort changed')
            receipt['join_scale_comparison'] = []
            for variant in scales:
                changed = variant['probe']
                if variant['scale'] == 1. and any(changed[k] != probe[k] for k in
                    ['initial_progress_m', 'trajectory', 'planning_diagnostics', 'attempts']):
                    raise ValueError('unit join scale changed ordinary probe')
                attempts = changed['attempts']
                search = summarize(attempts) if attempts else dict(sweep_counts={},
                    maximum_clear_progress_gain_m=None, global_infeasibility_proven=False,
                    confirmation_authorized=False)
                search.update(attempt_count=len(attempts),
                    configured_minimum_progress_m=config['checked_approach_min_progress_m'],
                    clear_at_configured_minimum=sum(a['first_contact_s'] is None and
                        a['progress_gain_m'] >= config['checked_approach_min_progress_m'] for a in attempts))
                diagnostic = changed['planning_diagnostics']
                chosen = diagnostic['selected_candidate']
                receipt['join_scale_comparison'].append(dict(scale=variant['scale'],
                    search=search, selected=None if chosen is None else diagnostic['candidates'][chosen],
                    candidates=diagnostic['candidates'], operational_policy_changed=False))
            receipt['forecast_corridor'] = inspect_corridor(
                metadata['route'], metadata['predictions'], config['vehicle']['radius'],
                probe['initial_progress_m'])
            result['cases'].append(receipt)
            save()
            print(json.dumps(receipt, allow_nan=False), flush=True)
    if digest(binary) != binary_hash:
        raise ValueError('inspection binary changed')
    result.update(complete=True, binary_sha256=binary_hash)
    save()


if __name__ == '__main__':
    main()
