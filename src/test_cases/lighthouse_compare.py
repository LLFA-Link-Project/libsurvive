import gzip
import json
import math
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]/'tools'))
import lighthouse_compare as compare


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


if __name__ == '__main__':
    unittest.main()
