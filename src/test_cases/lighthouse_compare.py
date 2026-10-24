import gzip
import json
import math
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]/'tools'))
import lighthouse_compare as compare
import lighthouse_compare_disk as disk


def sample(real, mono, uncertainty=100):
    return dict(realtime_ns=real, monotonic_before_ns=mono-uncertainty,
                monotonic_after_ns=mono+uncertainty, uncertainty_ns=uncertainty,
                offset_realtime_minus_monotonic_ns=real-mono)


class Comparison(unittest.TestCase):
    def test_clock_bracketing(self):
        mapping = compare.ClockMap([sample(10_000_000_000, 1_000_000_000),
                                    sample(11_000_000_100, 2_000_000_000)])
        self.assertEqual(mapping.convert(10_500_000_050), (1_500_000_000, 1200))
        self.assertIsNone(mapping.convert(9_000_000_000))
        self.assertIsNone(mapping.convert(12_000_000_000))

    def test_clock_steps_and_overlap(self):
        jump = compare.ClockMap([sample(10_000_000_000, 1_000_000_000),
                                 sample(12_000_000_000, 2_000_000_000)])
        self.assertIsNone(jump.convert(11_000_000_000))
        overlap = compare.ClockMap([sample(10_000_000_000, 1_000_000_000),
                                    sample(11_000_000_000, 2_000_000_000),
                                    sample(10_500_000_000, 3_000_000_000),
                                    sample(11_500_000_000, 4_000_000_000)])
        self.assertIsNone(overlap.convert(10_750_000_000))

    def test_rigid_fit(self):
        world = ((2, -3, .2), (.5, .5, .5, .5))
        poses = [((1, 0, 0), (1, 0, 0, 0)), ((0, 2, 0), (0, 1, 0, 0)),
                 ((0, 0, 3), (0, 0, 1, 0)), ((1, 2, 3), (0, 0, 0, 1))]
        fitted = compare.fit([('a', pose, compare.compose(world, pose), i) for i, pose in enumerate(poses)])
        self.assertLess(compare.distance(fitted[0], world[0]), 1e-10)
        self.assertLess(compare.angle(fitted[1], world[1]), 1e-7)

    def test_invalid_gap_and_quaternion_sign(self):
        pose = ((0, 0, 0), (1, 0, 0, 0))
        negative = ((1, 0, 0), (-1, 0, 0, 0))
        samples = [(0, pose), (10_000_000, negative)]
        p, q = compare.interpolate(samples, [0, 10_000_000], 5_000_000)
        self.assertEqual(p, (.5, 0, 0))
        self.assertEqual(q, (1, 0, 0, 0))
        self.assertIsNone(compare.interpolate([(0, pose), (10, None)], [0, 10], 5))
        self.assertIsNone(compare.interpolate([(0, pose), (30_000_000, negative)], [0, 30_000_000], 20_000_000))

    def test_sensor_ticks_are_not_truncated_or_anchored_to_replay_start(self):
        base = 1 << 40
        start = 1_700_000_000_000_000_000
        clock = compare.ClockMap([sample(start, 1_000_000_000), sample(start+1_000_000_000, 2_000_000_000)])
        with tempfile.TemporaryDirectory() as temp:
            path = pathlib.Path(temp)/'input.gz'
            with gzip.open(path, 'wt') as f:
                f.write('9999 a CONFIG '+json.dumps({'device_serial_number': 'object'})+'\n')
                f.write(f'9999 a INPUT_TIME {base} {start//1000+100000}\n')
                f.write('12345 a POSE 1 2 3 1 0 0 0\n')
                f.write(f'12345 a POSE_TIMING {base+480000} 48000000\n')
            replay, _ = compare.load_replay(path, clock)
            self.assertEqual(replay['object'][0][0], 1_110_000_000)

    def test_alignment_does_not_fit_evaluation_error(self):
        replay, refs = [], []
        for i in range(10):
            pose = ((i/100, 0, 0), (1, 0, 0, 0))
            replay.append((i*1_000_000_000, pose))
            refs.append((i*1_000_000_000, ((i/100+(i >= 3), 0, 0), pose[1])))
        result = compare.compare({'a': replay}, {'a': refs}, warmup_s=0)
        self.assertLess(compare.distance(result['world_from_survive']['position_m'], (0, 0, 0)), 1e-12)
        self.assertAlmostEqual(result['objects']['a']['position_difference_m']['rmse'], 1)

    def test_time_bins_do_not_refit_the_transform(self):
        rows = [(i*1_000_000_000, ((i/100, 0, 0), (1, 0, 0, 0))) for i in range(10)]
        refs = [(t, ((p[0]+(i>=3), p[1], p[2]), q)) for i, (t, (p,q)) in enumerate(rows)]
        result = compare.compare({'a': rows}, {'a': refs}, warmup_s=0, diagnostic_bin_s=1)
        bins = result['objects']['a']['heldout_time_bins']
        self.assertEqual([row['start_after_capture_s'] for row in bins], list(range(3,10)))
        for row in bins:
            self.assertAlmostEqual(row['position_difference_m']['rmse'], 1)

    def test_missing_device_is_not_hidden(self):
        pose = ((0, 0, 0), (1, 0, 0, 0))
        rows = [(i*10_000_000, pose) for i in range(100)]
        result = compare.compare({'a': rows}, {'a': rows, 'missing': rows}, warmup_s=0)
        self.assertEqual(result['objects']['missing']['association'], 'no_replay_poses')
        self.assertEqual(result['objects']['missing']['coverage']['fraction'], 0)

    def test_state_events_preserve_loss_and_reacquisition(self):
        with tempfile.TemporaryDirectory() as temp:
            path = pathlib.Path(temp)
            with (path/'events.jsonl').open('w') as f:
                for i, (connected, valid) in enumerate([(True, True), (True, False),
                                                        (False, False), (True, False), (True, True)]):
                    f.write(json.dumps(dict(event='driver_state_changed', device_id='a',
                                            capture_time_ns=i, device_is_connected=connected,
                                            pose_is_valid=valid, tracking_result=200 if valid else 101))+'\n')
            result = compare.load_reference_events(path)
            self.assertEqual(result['counts']['a'], dict(lost_tracking=1, disconnected=1,
                                                       reconnected=1, reacquired=1))
            self.assertEqual(result['reacquisition_capture_times_ns']['a'], [4])

    def test_sparse_evaluation_does_not_claim_recovery_failures(self):
        pose = ((0, 0, 0), (1, 0, 0, 0))
        rows = [(i*200_000_000, pose) for i in range(30)]
        events = {'reacquisition_capture_times_ns': {'a': [4_000_000_000]}}
        result = compare.compare({'a': rows}, {'a': rows}, warmup_s=0,
                                 evaluation_hz=5, events=events)
        self.assertIn('unavailable', result['objects']['a']['recovery_after_reference_reacquisition'])


class DiskComparison(unittest.TestCase):
    def test_capture_window_keeps_tail_after_replay_input_stops(self):
        clock=[sample(10_000_000_000,0),sample(19_000_000_000,9_000_000_000)]
        pose=((0,0,0),(1,0,0,0))
        replay=[(i*1_000_000_000,pose) for i in range(6)]
        refs=[(i*1_000_000_000,pose) for i in range(10)]
        result=compare.compare({'a':replay},{'a':refs},warmup_s=0,bounds=disk.capture_bounds(clock))
        self.assertAlmostEqual(result['objects']['a']['coverage']['fraction'],.6)
        self.assertEqual(result['evaluation_samples'],3)

    def test_no_alignment_still_reports_missing_output_coverage(self):
        pose=((0,0,0),(1,0,0,0))
        refs=[(i*1_000_000_000,pose) for i in range(10)]
        result=compare.compare({}, {'a':refs},warmup_s=0,bounds=(0,9_000_000_000))
        self.assertEqual(result['objects']['a']['coverage']['fraction'],0)
        self.assertEqual(result['objects']['a']['coverage']['valid_reference_bins'],10)
        self.assertIsNone(result['objects']['a']['position_difference_m'])
        self.assertIsNone(result['world_from_survive'])
        self.assertEqual(result['evaluation_samples'],0)

    def test_disk_clock_mapping_matches_memory_implementation(self):
        start = 1_700_000_000_000_000_000
        tick_base = 1 << 40
        clock = compare.ClockMap([sample(start, 1_000_000_000),
                                  sample(start+2_000_000_000, 3_000_000_000)])
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            path = root/'input.gz'
            with gzip.open(path, 'wt') as f:
                f.write('999 a CONFIG '+json.dumps({'device_serial_number': 'object'})+'\n')
                for i in range(1000):
                    tick = tick_base+i*48000
                    receive_us = start//1000+100000+i*1000+(i%3-1)*50
                    f.write(f'999 a INPUT_TIME {tick} {receive_us}\n')
                    if i%2 == 0:
                        f.write('999 a POSE 1 2 3 1 0 0 0\n')
                        f.write(f'999 a POSE_TIMING {tick+43000} 48000000\n')
            expected, timing = compare.load_replay(path, clock)
            db = disk.create_db(root/'index.sqlite')
            try:
                actual_timing = disk.replay_index(db, path, clock)
                actual = list(disk.Samples(db, 'replay', 'object'))
                self.assertEqual(actual, expected['object'])
                self.assertEqual(actual_timing['capture_start_monotonic_ns'], timing['capture_start_monotonic_ns'])
                self.assertEqual(actual_timing['max_local_input_arrival_spread_ns'], timing['max_local_input_arrival_spread_ns'])
            finally:
                db.close()

    def test_receiver_identity_changes_are_segmented(self):
        start = 1_700_000_000_000_000_000
        clock = compare.ClockMap([sample(start, 1_000_000_000),
                                  sample(start+1_000_000_000, 2_000_000_000)])
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            path = root/'input.gz'
            with gzip.open(path, 'wt') as f:
                for i, serial in enumerate(['first', 'second']):
                    tick = (1 << 40)+i*4800000
                    f.write('0 a CONFIG '+json.dumps({'device_serial_number':serial})+'\n')
                    f.write(f'0 a INPUT_TIME {tick} {start//1000+100000+i*100000}\n')
                    f.write('0 a POSE 1 2 3 1 0 0 0\n')
                    f.write(f'0 a POSE_TIMING {tick} 48000000\n')
            db = disk.create_db(root/'index.sqlite')
            try:
                timing = disk.replay_index(db, path, clock)
                self.assertEqual(timing['receiver_identity_changes'], 1)
                self.assertEqual(list(disk.Samples(db, 'replay', 'first'))[0][0], 1_100_000_000)
                self.assertEqual(list(disk.Samples(db, 'replay', 'second'))[0][0], 1_200_000_000)
            finally:
                db.close()

    def test_disk_interpolation_and_heldout_fit(self):
        replay, refs = [], []
        with tempfile.TemporaryDirectory() as temp:
            db = disk.create_db(pathlib.Path(temp)/'index.sqlite')
            try:
                for i in range(10):
                    t = i*1_000_000_000
                    p = (i/100, 0, 0)
                    r = (i/100+(i >= 3), 0, 0)
                    replay.append((t, (p, (1, 0, 0, 0))))
                    refs.append((t, (r, (1, 0, 0, 0))))
                    for table, position in [('replay', p), ('reference', r)]:
                        db.execute(f'INSERT INTO {table}(serial,t,x,y,z,w,qx,qy,qz) VALUES(?,?,?,?,?,?,?,?,?)',
                                   ('a',t,*position,1,0,0,0))
                actual = compare.compare({'a': disk.Samples(db, 'replay', 'a')},
                                         {'a': disk.Samples(db, 'reference', 'a')}, warmup_s=0, evaluation_hz=5)
                expected = compare.compare({'a': replay}, {'a': refs}, warmup_s=0, evaluation_hz=5)
                self.assertEqual(actual, expected)
                samples = disk.Samples(db, 'reference', 'a')
                self.assertIsNone(samples.interpolate(500_000_000, 20_000_000))
                db.execute('UPDATE reference SET x=NULL WHERE t=0')
                self.assertIsNone(samples.interpolate(0, 20_000_000))
            finally:
                db.close()

    def test_cached_reference_is_read_only(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            reference = disk.create_db(root/'reference.sqlite')
            reference.execute("INSERT INTO reference(serial,t,x,y,z,w,qx,qy,qz) VALUES('a',0,1,2,3,1,0,0,0)")
            reference.commit(); reference.close()
            derived = disk.create_db(root/'derived.sqlite')
            try:
                derived.execute('ATTACH DATABASE ? AS native', ('file:'+str(root/'reference.sqlite')+'?mode=ro',))
                sample = disk.Samples(derived, 'native.reference', 'a')
                self.assertEqual(sample[0], (0, ((1,2,3),(1,0,0,0))))
                with self.assertRaises(disk.sqlite3.OperationalError):
                    derived.execute('DELETE FROM native.reference')
            finally:
                derived.close()

    def test_truncation_is_explicit_and_keeps_valid_prefix(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            row = dict(device_class=2, device_id='a', pose_time_estimate_ns=123,
                       pose_is_valid=False, device_is_connected=True, tracking_result=101)
            path = root/'poses.jsonl.gz'
            with gzip.open(path, 'wt') as f:
                f.write(json.dumps(row)+'\n')
            path.write_bytes(path.read_bytes()[:-8])
            strict = disk.create_db(root/'strict.sqlite')
            partial = disk.create_db(root/'partial.sqlite')
            try:
                with self.assertRaises(EOFError):
                    disk.reference_index(strict, root)
                result = disk.reference_index(partial, root, allow_truncated=True)
                self.assertTrue(result['integrity'].startswith('partial:'))
                self.assertEqual(result['rows_read'], 1)
                self.assertEqual(list(disk.Samples(partial, 'reference', 'a')), [(123, None)])
            finally:
                strict.close(); partial.close()


if __name__ == '__main__':
    unittest.main()
