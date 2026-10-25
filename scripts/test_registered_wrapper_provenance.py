#!/usr/bin/env python3
"""Check distinct first-evidence and active-reproducer byte ownership."""
import copy
import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('registered_suite', ROOT/'scripts/check-registered-grid-temporal-suite.py')
suite = importlib.util.module_from_spec(spec)
spec.loader.exec_module(suite)

class WrapperBindings(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.paths = [suite.ORIGINAL_WRAPPER_PATH,
            'scripts/check-multiscale-temporal-suite.py', 'scripts/recorded_numeric.py']
        for relative in self.paths:
            destination = self.root/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/relative, destination)
        self.packet = {'wrapper_sha256': suite.ORIGINAL_WRAPPER_SHA}

    def test_first_packet_and_new_reproducer_have_separate_exact_identities(self):
        proof = suite.multiscale_wrapper_bindings(self.packet, self.root)
        self.assertEqual(proof['original_wrapper']['sha256'], self.packet['wrapper_sha256'])
        self.assertNotEqual(proof['replay_wrapper']['sha256'], self.packet['wrapper_sha256'])
        self.assertEqual(set(proof), {'original_wrapper','replay_wrapper','replay_comparator'})

    def test_tampering_each_owned_source_is_rejected(self):
        for relative in self.paths:
            with self.subTest(relative=relative):
                path = self.root/relative
                original = path.read_bytes()
                path.write_bytes(original+b'\n# changed source\n')
                with self.assertRaisesRegex(ValueError, 'source changed'):
                    suite.multiscale_wrapper_bindings(self.packet, self.root)
                path.write_bytes(original)

    def test_changed_first_packet_binding_is_rejected(self):
        packet = copy.deepcopy(self.packet)
        packet['wrapper_sha256'] = suite.REPLAY_WRAPPER_SHA
        with self.assertRaisesRegex(ValueError, 'original.*binding changed'):
            suite.multiscale_wrapper_bindings(packet, self.root)

    def test_replacing_first_wrapper_with_current_wrapper_is_rejected(self):
        shutil.copyfile(self.root/'scripts/check-multiscale-temporal-suite.py',
                        self.root/suite.ORIGINAL_WRAPPER_PATH)
        with self.assertRaisesRegex(ValueError, 'source changed'):
            suite.multiscale_wrapper_bindings(self.packet, self.root)

    def test_missing_sources_are_rejected(self):
        for relative in self.paths:
            with self.subTest(relative=relative):
                path = self.root/relative
                original = path.read_bytes()
                path.unlink()
                with self.assertRaisesRegex(ValueError, 'unsafe/missing'):
                    suite.multiscale_wrapper_bindings(self.packet, self.root)
                path.write_bytes(original)

if __name__ == '__main__':
    unittest.main()
