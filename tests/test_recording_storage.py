"""Data preservation and save-failure tests for raw recording staging."""
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from recording_storage import RawRecordingStorage, capture_folder


class RawStorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.dest = self.root/'archive'
        self.dest.mkdir()

    def tearDown(self):
        self.temp.cleanup()

    def stage(self):
        s = RawRecordingStorage(self.dest, 1024, self.root/'staging')
        (s.folder/'images').mkdir()
        (s.folder/'images/000000.pgm').write_bytes(b'P5\n2 2\n255\n\x00\x20\x80\xff')
        (s.folder/'vio').mkdir()
        (s.folder/'vio/valid-poses.csv').write_text('t_rel_s,reset_epoch,px,py,pz\n1,0,0,0,0\n')
        (s.folder/'vio/map-epoch-0.ply').write_text('ply\nformat ascii 1.0\nelement vertex 0\nend_header\n')
        (s.folder/'raw.jsonl').write_text('{"unaltered":true}\n')
        (s.folder/'session.json').write_text('{"status":"completed"}\n')
        return s

    def test_verified_archive_preserves_all_bytes_before_removing_stage(self):
        s = self.stage()
        expected = {str(p.relative_to(s.folder)):p.read_bytes() for p in s.folder.rglob('*') if p.is_file()}
        self.assertEqual(capture_folder(self.dest), s.folder)
        s.persist()
        self.assertFalse(s.folder.exists())
        self.assertEqual(capture_folder(self.dest), self.dest)
        manifest = json.loads((self.dest/'archive-manifest.json').read_text())['files']
        self.assertEqual(set(manifest), set(expected))
        for name, data in expected.items():
            self.assertEqual((self.dest/name).read_bytes(), data)
            self.assertEqual(manifest[name]['sha256'], hashlib.sha256(data).hexdigest())

    def test_failed_archive_retains_complete_stage_for_recovery(self):
        s = self.stage()
        with patch.object(s, 'copy_verified', side_effect=OSError('simulated storage failure')):
            with self.assertRaises(OSError):
                s.persist()
        self.assertTrue((s.folder/'images/000000.pgm').exists())
        self.assertTrue((s.folder/'raw.jsonl').exists())
        self.assertEqual(capture_folder(self.dest), s.folder)
        self.assertEqual(json.loads((self.dest/'recording-storage.json').read_text())['state'], 'save_failed')

    def test_existing_archive_file_is_never_overwritten(self):
        s = self.stage()
        (self.dest/'raw.jsonl').write_text('existing evidence')
        with self.assertRaises(FileExistsError):
            s.persist()
        self.assertEqual((self.dest/'raw.jsonl').read_text(), 'existing evidence')
        self.assertTrue((s.folder/'session.json').exists())


if __name__ == '__main__':
    unittest.main()
