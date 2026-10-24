#!/usr/bin/env python3
"""Refuse changed shared math and preserve existing staging destinations."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'rgbd_regression', Path(__file__).with_name('prepare-rgbd-regression.py'))
staging = importlib.util.module_from_spec(spec)
spec.loader.exec_module(staging)


class SourceIntegrity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        snapshots = [staging.ROOT / 'integrations/rgbd/baselines' / name / 'sources'
                     for name in ('reprojection-fr2-desk-first-v1',
                                  'submap-office-first-v8', 'public-keyframes-v6')]
        archive = io.BytesIO()
        with tarfile.open(fileobj=archive, mode='w') as source:
            for name in dict.fromkeys((*staging.KERNELS, *staging.ARCHIVE_TOP_LEVEL,
                                       *(f'crates/{library}/src/lib.rs'
                                         for library in staging.LIBRARIES))):
                snapshot = next(root / name for root in snapshots if (root / name).is_file())
                source.add(snapshot, arcname=name)
        cls.archive = archive.getvalue()

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='rgbd-staging-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'source'
        for name in (*staging.KERNELS, 'crates/core/src/lib.rs'):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((staging.ROOT / name).read_bytes())
        self.output = Path(self.temporary.name) / 'regression'

    def refuse(self):
        with patch.object(staging, 'ROOT', self.root), \
                patch.object(staging.subprocess, 'check_output', return_value=self.archive):
            with self.assertRaises(ValueError):
                staging.prepare(self.output)
        self.assertFalse(self.output.exists())

    def test_each_changed_active_kernel_is_rejected_before_staging(self):
        for name in staging.KERNELS:
            with self.subTest(kernel=name):
                path = self.root / name
                original = path.read_bytes()
                path.write_bytes(original + b'\n// changed source\n')
                self.refuse()
                path.write_bytes(original)

    def test_changed_vec3_numeric_validation_is_rejected(self):
        path = self.root / 'crates/core/src/lib.rs'
        path.write_bytes(path.read_bytes().replace(
            b'self.x.is_finite() && self.y.is_finite() && self.z.is_finite()', b'true'))
        self.refuse()

    def test_duplicate_vec3_source_is_rejected(self):
        path = self.root / 'crates/core/src/lib.rs'
        path.write_bytes(path.read_bytes() + staging.vec3(path.read_bytes()))
        self.refuse()

    def complete_active_fixture(self):
        for name in ('Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml'):
            (self.root / name).write_bytes((staging.ROOT / name).read_bytes())
        for library in staging.LIBRARIES:
            name = f'crates/{library}/src/lib.rs'
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((staging.ROOT / name).read_bytes())
        for name in ('scripts', 'integrations/rgbd', 'assets', 'data',
                     'artifacts', 'maps', 'scenarios'):
            (self.root / name).mkdir(parents=True, exist_ok=True)

    def test_new_active_workspace_retains_original_historical_root_lock(self):
        self.complete_active_fixture()
        active = b'# A later active workspace lock must not enter the frozen workspace.\n'
        (self.root / 'Cargo.lock').write_bytes(active)
        with tarfile.open(fileobj=io.BytesIO(self.archive)) as source:
            historical = source.extractfile('Cargo.lock').read()
        with patch.object(staging, 'ROOT', self.root), \
                patch.object(staging.subprocess, 'check_output', return_value=self.archive) as command, \
                patch.object(Path, 'symlink_to'):
            # Data-link privileges are outside the frozen-lock contract.
            staging.prepare(self.output)
        self.assertIn('Cargo.lock', command.call_args.args[0])
        self.assertEqual((self.output / 'Cargo.lock').read_bytes(), historical)
        self.assertEqual((self.root / 'Cargo.lock').read_bytes(), active)
        receipt = json.loads((self.output / 'regression-sources.json').read_text())
        self.assertEqual(receipt['historical_top_level_files']['Cargo.lock'],
                         {'active_sha256': hashlib.sha256(active).hexdigest(),
                          'historical_sha256': hashlib.sha256(historical).hexdigest()})
        self.assertEqual(receipt['historical_revision'], staging.REVISION)

    def test_missing_historical_root_lock_is_refused_before_output(self):
        self.complete_active_fixture()
        incomplete = io.BytesIO()
        with tarfile.open(fileobj=io.BytesIO(self.archive)) as original, \
                tarfile.open(fileobj=incomplete, mode='w') as output:
            for member in original.getmembers():
                if member.name != 'Cargo.lock':
                    output.addfile(member, original.extractfile(member))
        with patch.object(staging, 'ROOT', self.root), \
                patch.object(staging.subprocess, 'check_output', return_value=incomplete.getvalue()):
            with self.assertRaisesRegex(ValueError, 'missing required top-level file: Cargo.lock'):
                staging.prepare(self.output)
        self.assertFalse(self.output.exists())

    def test_existing_destination_keeps_foreign_bytes(self):
        self.output.mkdir()
        foreign = self.output / 'keep'
        foreign.write_bytes(b'foreign data')
        with self.assertRaises(ValueError):
            staging.prepare(self.output)
        self.assertEqual(foreign.read_bytes(), b'foreign data')
        self.assertEqual(list(self.output.iterdir()), [foreign])


if __name__ == '__main__':
    unittest.main()
