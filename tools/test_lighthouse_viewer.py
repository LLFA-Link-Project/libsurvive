"""Parser / bounded-window tests for the offline viewer; no optional packages."""

import gzip
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import threading
import unittest

import lighthouse_viewer as viewer


class ViewerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def index(self, text, kind='native', truncate=False):
        source = self.root/'source.gz'
        data = gzip.compress(text.encode())
        source.write_bytes(data[:-8] if truncate else data)
        output = self.root/'index.sqlite'
        meta = viewer.index_source(dict(path=source, kind=kind), output)
        with sqlite3.connect(output) as db:
            rows = db.execute(f'SELECT {viewer.SQL_COLUMNS} FROM samples ORDER BY rowid').fetchall()
        self.assertEqual(meta['source_sha256'], hashlib.sha256(source.read_bytes()).hexdigest())
        return meta, rows

    @staticmethod
    def native():
        return ('0.0 WM0 CONFIG {"device_serial_number":"tracker"}\n'
                '1.000 WM0 POSE 1 2 3 1 0 0 0\n'
                '1.000 WM0 POSE_TIMING 4294967488 48000000 0 0\n'
                '1.000 WM0 POSE_STATE 4294967488 48000000 0 4800000 -0.1 1.0000000000123 2 2.9 1 0 0 0\n'
                '1.000 WM0 POSE_BIAS 4294967488 12 .01 .02 .03 0 0 0 1 1 1 1 0 0 0\n'
                '1.000 WM0 VELOCITY .1 .2 .3 0 0 0\n'
                '1.000 WM0 I 3 192 0 0 1 .04 .05 .06 0 0 0 0\n'
                '1.001 WM0 INPUT_TIME 4294967488 1000001\n'
                '1.004 WM0 POSE 1.001 2 3 -1 0 0 0\n'
                '1.004 WM0 POSE_TIMING 4295159488 48000000 0 0\n'
                '1.004 WM0 POSE_STATE 4295159488 48000000 0 4992000 -0.1 1.001 2 2.9 -1 0 0 0\n')

    def test_full_precision_raw_state_and_tick_matched_diagnostics(self):
        meta, rows = self.index(self.native())
        self.assertEqual(len(rows), 2)
        self.assertEqual(meta['tracks'][0]['stage'], 'POSE_STATE')
        self.assertEqual(meta['health']['damaged_rows'], 0)
        r = rows[0]
        self.assertEqual(r[1:3], (4294967488, 48000000))
        self.assertEqual(r[3], 1.0000000000123)
        self.assertEqual(r[5], 2.9)  # Floor subtraction never applied to STATE.
        self.assertAlmostEqual(r[10], .1)
        self.assertEqual(r[11:20], (.1, .2, .3, .01, .02, .03, .04, .05, .06))
        self.assertAlmostEqual((rows[1][1]-r[1])/r[2], .004)
        self.assertEqual(meta['native_epoch_realtime_ns'], -999000)

    def test_mismatched_imu_and_bias_are_not_joined(self):
        text = self.native().replace('I 3 192 ', 'I 3 193 ').replace('POSE_BIAS 4294967488 ', 'POSE_BIAS 4294967489 ')
        _, rows = self.index(text)
        self.assertEqual(rows[0][14:20], (None,)*6)

    def test_pose_fallback_does_not_claim_imu_frame(self):
        meta, rows = self.index('0 WM0 CONFIG {"device_serial_number":"one"}\n1 WM0 POSE 1 2 3 1 0 0 0\n1 WM0 POSE_TIMING 96 48 0 0\n')
        self.assertEqual(meta['tracks'][0]['stage'], 'POSE')
        self.assertIn('reported pose', meta['tracks'][0]['reference'])
        self.assertEqual(rows[0][1:3], (96, 48))

    def test_serial_reassignment_is_separate_track(self):
        meta, rows = self.index('0 WM0 CONFIG {"device_serial_number":"one"}\n1 WM0 POSE 1 2 3 1 0 0 0\n2 WM0 CONFIG {"device_serial_number":"two"}\n3 WM0 POSE 1 2 3 1 0 0 0\n')
        self.assertEqual({t['serial'] for t in meta['tracks']}, {'one', 'two'})
        self.assertEqual(len(rows), 2)

    def test_truncated_gzip_is_explicit_prefix(self):
        meta, rows = self.index(self.native(), truncate=True)
        self.assertEqual(meta['health']['stream_integrity'], 'readable-prefix')
        self.assertEqual(len(rows), 2)

    def test_bad_state_remains_invalid_sample(self):
        meta, rows = self.index(self.native().replace('1.0000000000123', 'nan'))
        self.assertGreater(meta['health']['damaged_rows'], 0)
        self.assertEqual(rows[0][-1], 0)

    def test_nul_row_reported(self):
        meta, rows = self.index(self.native()+'2 WM0 B \0 damaged\n')
        self.assertEqual(meta['health']['damaged_rows'], 1)
        self.assertEqual(len(rows), 2)

    def test_cancelled_index_does_not_publish_partial_metadata(self):
        source = self.root/'source.gz'
        source.write_bytes(gzip.compress(self.native().encode()))
        cancel = threading.Event()
        cancel.set()
        output = self.root/'cancelled.sqlite'
        with self.assertRaises(viewer.IndexCancelled):
            viewer.index_source(dict(path=source, kind='native'), output, cancel)
        with sqlite3.connect(output) as db:
            self.assertIsNone(db.execute('SELECT value FROM metadata').fetchone())

    @staticmethod
    def json_pose(**extra):
        return dict(capture_time_ns=1_000_000_000, device_id='one', pose_is_valid=True,
                    position_m=[1, 0, 0], orientation_xyzw=[0, 0, 0, 1], **extra)

    def test_json_stages_and_reference_points_stay_separate(self):
        a = self.json_pose(stage='backend_received', reference_point='imu', coordinate_frame='Z-up')
        b = dict(a, stage='app_output', reference_point='grip', coordinate_frame='Y-up', capture_time_ns=1_004_000_000)
        meta, rows = self.index('\n'.join(map(json.dumps, [a, b]))+'\n', 'jsonl')
        self.assertEqual(len(meta['tracks']), 2)
        self.assertEqual(rows[0][1:3], (None, None))
        self.assertAlmostEqual(rows[1][0], .004)

    def test_valve_composition_and_xyzw(self):
        r = self.json_pose(source='steamvr_lh', device_is_connected=True, tracking_result=200,
                           world_from_driver_translation_m=[3, 0, 0],
                           world_from_driver_orientation_xyzw=[0, 0, 1, 0],
                           driver_from_head_translation_m=[0, 2, 0],
                           driver_from_head_orientation_xyzw=[0, 0, 0, 1])
        meta, rows = self.index(json.dumps(r)+'\n', 'jsonl')
        self.assertEqual(rows[0][3:6], (2, -2, 0))
        self.assertEqual(rows[0][6:10], (0, 0, 0, 1))
        self.assertIn('Y-up', meta['tracks'][0]['frame'])

    def test_invalid_pose_and_zero_quaternion_retained(self):
        a = self.json_pose()
        b = dict(a, pose_is_valid=False, capture_time_ns=1_004_000_000)
        c = dict(a, orientation_xyzw=[0, 0, 0, 0], capture_time_ns=1_008_000_000)
        meta, rows = self.index('\n'.join(map(json.dumps, [a, b, c]))+'\n', 'jsonl')
        self.assertEqual([r[-1] for r in rows], [1, 0, 0])
        self.assertEqual(meta['health']['damaged_rows'], 1)

    def test_monado_partial_validity_is_not_full_pose(self):
        a = self.json_pose(validity={'position': False, 'orientation': True})
        _, rows = self.index(json.dumps(a)+'\n', 'jsonl')
        self.assertEqual(rows[0][-1], 0)

    def test_window_limits_exact_cadence_and_cache_invalidation(self):
        folder = self.root/'dumps'/'test'
        folder.mkdir(parents=True)
        (folder/'manifest.json').write_text('{"status":"complete"}')
        (folder/'native.rec.gz').write_bytes(gzip.compress(self.native().encode()))
        store = viewer.Store(folder.parent, self.root/'cache', repo=self.root)
        try:
            key = 'dump:test:native'
            store.ensure(key)
            store.pool.shutdown(wait=True)
            meta = store.ensure(key)
            self.assertEqual(meta['state'], 'ready')
            track = meta['tracks'][0]['id']
            result = store.window(key, track, .999, 1.005)
            self.assertFalse(result['decimation'])
            self.assertEqual(len(result['rows']), 2)
            self.assertEqual(result['rows'][0][1], 4294967488)
            self.assertEqual(store.window(key, track, 1.003, 1.005)['rows'][0][0], 1.004)
            for lo, hi in ((1, 1), (2, 1), (0, 61), (0, float('nan'))):
                with self.assertRaises(ValueError):
                    store.window(key, track, lo, hi)
            with self.assertRaises(ValueError):
                store.window(key, 'unknown', 0, 1)
            before = store.db_path(store.sources[key])
            with (folder/'native.rec.gz').open('ab') as f:
                f.write(gzip.compress(b'2 WM0 POSE 1 2 3 1 0 0 0\n'))
            self.assertNotEqual(before, store.db_path(store.sources[key]))
        finally:
            store.pool.shutdown(wait=True)


if __name__ == '__main__':
    unittest.main()
