#!/usr/bin/env python3
"""A completed physical matrix must not publish success before remaining checks."""
from contextlib import ExitStack, redirect_stdout
import copy
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import check_chassis_acquisition as checker

class ResultPublicationTests(unittest.TestCase):
    def execute(self, failure=None):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            evidence = root/'evidence'; evidence.mkdir()
            output = root/'result'
            binaries = {}
            for name in ['reference','native','example','baseline']:
                path = root/name; path.write_bytes(name.encode()); binaries[name] = path
            rows = []
            for i in range(48):
                rows.append(dict(scenario=f'frozen/{i:03d}.json',sha256=str(i),plant='reference-2d' if i<24 else 'rne-dynamic',
                    world='mission',mode='fixture',seed=7 if i%2==0 else 42,
                    passed=True,summary=dict(passed=True,steps=1,reached_goal=True),
                    run_exit_code=0,replay_exit_code=0,replay=dict(verified=True,ticks=1),
                    directional_certificates=dict(passed=True),
                    storage={key:dict(sha256='same') for key in [
                        'run/run.json','run/sensors.jsonl','run/summary.json','replay/outputs.jsonl']}))
            raw = json.dumps(dict(source_commit=checker.RECORDED,complete=True,executions=dict(new=rows))).encode()
            canonical = root/'canonical.json'; canonical.write_bytes(raw)
            (evidence/'report.json').write_bytes(raw)
            def freeze(destination, specs, **_):
                for spec in specs:
                    path = destination/spec['scenario']; path.parent.mkdir(parents=True,exist_ok=True)
                    path.write_text('{}')
                return dict(episodes=copy.deepcopy(specs))
            def run_case(_, spec, *args, **kwargs):
                return copy.deepcopy(spec)
            def historical(*_):
                if failure=='historical': raise RuntimeError('injected historical inspection failure')
                return dict(passed=True)
            def controls(*_):
                if failure=='controls': raise RuntimeError('injected byte-control failure')
                if failure=='binary-identity': binaries['reference'].write_bytes(b'changed after physical runs')
                return dict(passed=True)
            def compare(row, _):
                failed = failure=='regression' and row['scenario']=='frozen/047.json'
                return dict(regression_passed=not failed,new_failed_gates=['injected'] if failed else [])
            args = ['check_chassis_acquisition.py','--stage','full','--evidence',str(evidence),'--output',str(output),
                '--binary',str(binaries['reference']),'--native-binary',str(binaries['native']),
                '--example-binary',str(binaries['example']),'--baseline-native',str(binaries['baseline'])]
            with ExitStack() as stack:
                stack.enter_context(patch.object(checker,'CANONICAL',canonical))
                stack.enter_context(patch.object(checker.parent,'specifications',return_value=rows))
                stack.enter_context(patch.object(checker.parent,'freeze',side_effect=freeze))
                stack.enter_context(patch.object(checker.parent,'run_case',side_effect=run_case))
                stack.enter_context(patch.object(checker.parent,'compare',side_effect=compare))
                stack.enter_context(patch.object(checker,'read_retained',return_value=b'{"header":{"config":{}}}\n'))
                stack.enter_context(patch.object(checker,'fixed_inputs',return_value=dict(passed=True)))
                stack.enter_context(patch.object(checker,'historical_replays',side_effect=historical))
                stack.enter_context(patch.object(checker,'byte_controls',side_effect=controls))
                stack.enter_context(patch.dict(os.environ,RUSTDRIVING_TESTED_SOURCE='synthetic-publication-control'))
                stack.enter_context(patch('sys.argv',args))
                stack.enter_context(redirect_stdout(io.StringIO()))
                error = None
                try: checker.main()
                except (RuntimeError,ValueError,SystemExit) as caught: error = caught
            report = json.loads((output/'report.json').read_text())
            return error,report

    def test_historical_exception_retains_an_unpassed_complete_matrix(self):
        error,report = self.execute('historical')
        self.assertIsInstance(error,RuntimeError)
        self.assertEqual(len(report['cases']),48)
        self.assertFalse(report['passed'])

    def test_byte_control_exception_cannot_leave_intermediate_success(self):
        error,report = self.execute('controls')
        self.assertIsInstance(error,RuntimeError)
        self.assertTrue(report['complete'])
        self.assertFalse(report['passed'])

    def test_final_binary_change_cannot_leave_intermediate_success(self):
        error,report = self.execute('binary-identity')
        self.assertIsInstance(error,ValueError)
        self.assertTrue(report['complete'])
        self.assertFalse(report['passed'])

    def test_finished_auxiliary_checks_cannot_hide_a_physical_regression(self):
        error,report = self.execute('regression')
        self.assertIsInstance(error,SystemExit)
        self.assertEqual(error.code,1)
        self.assertTrue(report['full_qualification_complete'])
        self.assertFalse(report['passed'])
        self.assertFalse(report['original_full_adoption_gate_passed'])

    def test_success_is_published_after_all_phases(self):
        error,report = self.execute()
        self.assertIsInstance(error,SystemExit)
        self.assertEqual(error.code,0)
        self.assertTrue(report['passed'])
        self.assertTrue(report['full_qualification_complete'])

if __name__=='__main__':
    unittest.main()
