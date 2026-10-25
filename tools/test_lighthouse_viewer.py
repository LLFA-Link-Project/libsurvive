"""Parser / bounded-window tests for the offline viewer; no optional packages."""

import gzip
from contextlib import closing
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
        with closing(sqlite3.connect(output)) as db:
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
        with closing(sqlite3.connect(output)) as db:
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
            self.assertEqual(store.window(key, track, 1.003, 1.005)['previous'][0], 1.0)
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

    def test_scene_keeps_relative_positions_timed_bases_and_raw_optical(self):
        folder = self.root/'dumps'/'scene'
        folder.mkdir(parents=True)
        (folder/'manifest.json').write_text('{"status":"complete"}')
        (folder/'calibration.initial.json').write_text('"floor-offset":"-0.1"\n')
        text = ('0 OPTION report-in-imu b 1\n'
                '0 WM0 CONFIG {"device_serial_number":"one","model_number":"Knuckles Left","device_class":"controller"}\n'
                '0 WM1 CONFIG {"device_serial_number":"two","model_number":"VIVE Tracker Pro MV","device_class":"generic_tracker"}\n'
                '0.1 14 LH_POSE 10 20 30 1 0 0 0 123\n'
                '1 WM0 POSE 1 2 3.1 1 0 0 0\n'
                '1 WM0 POSE_STATE 48 48 0 0 -0.1 1 2 3 1 0 0 0\n'
                '1 WM1 POSE 4 5 6.1 1 0 0 0\n'
                '1 WM1 POSE_STATE 48 48 0 0 -0.1 4 5 6 1 0 0 0\n'
                '1.001 WM0-raw-obs EXTERNAL_POSE 1.01 2 3 1 0 0 0\n'
                '3 14 LH_POSE 100 200 300 1 0 0 0 123\n'
                '4 WM0 POSE 7 8 9.1 1 0 0 0\n'
                '4 WM0 POSE_STATE 192 48 0 0 -0.1 7 8 9 1 0 0 0\n')
        (folder/'native.rec.gz').write_bytes(gzip.compress(text.encode()))
        store = viewer.Store(folder.parent, self.root/'cache', repo=self.root)
        try:
            key = 'dump:scene:native'
            store.ensure(key)
            store.pool.shutdown(wait=True)
            meta = store.ensure(key)
            track = next(t for t in meta['tracks'] if t['serial'] == 'one' and t['stage'] == 'POSE_STATE')
            self.assertEqual(track['model'], 'Knuckles Left')
            window = store.window(key, track['id'], 1, 2)
            other = next(x for x in window['context'] if x['track']['serial'] == 'two')
            self.assertEqual(other['rows'][0][3:6], (4, 5, 6))  # Never zero each device independently.
            base = next(x for x in window['context'] if x['track']['stage'] == 'LH_POSE')
            self.assertEqual(len(base['rows']), 1)  # Not the future 3 s update.
            self.assertEqual(base['rows'][0][3:6], (10, 20, 29.9))
            optical = window['optical']
            self.assertEqual(optical['state'], 'ready')
            self.assertEqual(optical['origin'], 'recorded')
            self.assertEqual(optical['rows'][0][3], 1.01)
            self.assertEqual(store.window(key, track['id'], 1.003, 1.01)['optical']['rows'][0][0], 1.001)
            # Whole playback can use a fixed anchor outside the loaded window.
            later = store.window(key, track['id'], 3, 4.1, anchor=1)
            self.assertEqual(later['anchor'][3:6], (1, 2, 3))
            self.assertEqual(later['rows'][0][3:6], (7, 8, 9))
            self.assertEqual(store.window(key, track['id'], 0, .5, anchor='first')['anchor'][0], 1)
            with self.assertRaises(ValueError):
                store.window(key, track['id'], 3, 4.1, anchor=1.5)
            # Calibration is a dependency of the floor conversion cache.
            old = store.db_path(store.sources[key])
            (folder/'calibration.initial.json').write_text('"floor-offset":"-0.25"\n')
            self.assertNotEqual(old, store.db_path(store.sources[key]))
        finally:
            store.pool.shutdown(wait=True)

    def test_optical_head_reference_does_not_overlay_imu_state(self):
        meta, rows = self.index(self.native()+'1.002 WM0-raw-obs EXTERNAL_POSE 10 20 30 1 0 0 0\n')
        optical = next(t for t in meta['tracks'] if t['stage'] == 'OPTICAL')
        self.assertEqual(optical['reference'], 'head before floor')
        self.assertEqual(len(rows), 3)

    def test_optical_timing_keeps_full_ticks_and_is_consumed_once(self):
        meta, rows = self.index('0 OPTION report-in-imu b 1\n'
            '0 WM0 CONFIG {"device_serial_number":"one"}\n'
            '1 WM0-raw-obs OPTICAL_TIMING 4294967488 48000000\n'
            '1 WM0-raw-obs EXTERNAL_POSE 1 2 3 1 0 0 0\n'
            '2 WM0-raw-obs EXTERNAL_POSE 4 5 6 1 0 0 0\n'
            '3 WM0-raw-obs OPTICAL_TIMING 4295159488 48000000\n'
            '3 WM0-raw-obs EXTERNAL_POSE nan 5 6 1 0 0 0\n'
            '4 WM0-raw-obs EXTERNAL_POSE 4 5 6 1 0 0 0\n')
        self.assertEqual(rows[0][1:3], (4294967488, 48000000))
        self.assertEqual(rows[1][1:3], (None, None))
        self.assertEqual(rows[2][20], 0)
        self.assertEqual(rows[2][1], 4295159488)
        self.assertEqual(rows[3][1:3], (None, None))
        self.assertEqual(meta['tracks'][0]['reference'], 'raw IMU before floor')

    def test_reconstructed_optical_is_bound_to_both_file_hashes(self):
        folder = self.root/'dumps'/'test'
        folder.mkdir(parents=True)
        (folder/'manifest.json').write_text('{"status":"complete"}')
        (folder/'native.rec.gz').write_bytes(gzip.compress(self.native().encode()))
        derived = self.root/'.lh-evaluation/viewer-optical/diagnostic'
        derived.mkdir(parents=True)
        (derived/'record.rec.gz').write_bytes(gzip.compress((self.native()+
            '0 OPTION report-in-imu b 1\n1.002 WM0-raw-obs EXTERNAL_POSE 1.01 2 2.9 1 0 0 0\n').encode()))
        proof = dict(target_id='dump:test:native', target_sha256='0'*64, returncode=0,
                     record_sha256=hashlib.sha256((derived/'record.rec.gz').read_bytes()).hexdigest())
        proof_path = derived/'viewer-source.json'
        proof_path.write_text(json.dumps(proof))
        store = viewer.Store(folder.parent, self.root/'cache', repo=self.root)
        try:
            for key in store.sources:
                store.ensure(key)
            store.pool.shutdown(wait=True)
            meta = store.ensure('dump:test:native')
            track = meta['tracks'][0]['id']
            self.assertEqual(store.window('dump:test:native', track, 1, 1.01)['optical']['state'], 'unavailable')
            proof['target_sha256'] = meta['source_sha256']
            proof_path.write_text(json.dumps(proof))
            store.refresh()
            optical = store.window('dump:test:native', track, 1, 1.01)['optical']
            self.assertEqual(optical['origin'], 'reconstructed')
            self.assertEqual(optical['state'], 'ready')
            self.assertEqual(optical['source_sha256'], proof['record_sha256'])
            self.assertEqual(store.window('dump:test:native', track, 1.003, 1.01)['optical']['rows'][0][0], 1.002)
            self.assertEqual(optical['rows'][0][3], 1.01)
            proof['record_sha256'] = '0'*64
            proof_path.write_text(json.dumps(proof))
            store.refresh()
            optical = store.window('dump:test:native', track, 1, 1.01)['optical']
            self.assertEqual(optical['state'], 'unavailable')
        finally:
            store.pool.shutdown(wait=True)


if __name__ == '__main__':
    unittest.main()
