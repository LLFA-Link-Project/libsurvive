import gzip
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import lighthouse_motion_metrics as motion


class MotionMetricsTests(unittest.TestCase):
    def test_frequency_bands_separate_slow_motion_from_fast_variation(self):
        t = np.arange(0, 8, .005)
        values = np.column_stack([3 + .1 * t + .002 * np.sin(2*np.pi*t),
                                  .001 * np.sin(2*np.pi*10*t), np.zeros_like(t)])
        bands = motion.frequency_band_rms(t, values)
        self.assertAlmostEqual(bands['0.5-2_hz'], .002 / np.sqrt(2), delta=2e-6)
        self.assertAlmostEqual(bands['5-15_hz'], .001 / np.sqrt(2), delta=2e-8)
        self.assertLess(bands['15-50_hz'], 1e-8)

    def test_frequency_power_matches_windowed_time_domain_for_even_and_odd_lengths(self):
        rng = np.random.default_rng(3947)
        for count in (799, 800):
            t = np.arange(count) * .005
            values = rng.normal(size=(count, 3))
            basis = np.column_stack([np.ones(count), t-t.mean()])
            residual = values - basis @ np.linalg.lstsq(basis, values, rcond=None)[0]
            window = np.hanning(count)
            expected = np.sqrt(np.sum(residual**2 * window[:, None]**2) / np.sum(window**2))
            measured = motion.frequency_band_rms(t, values, ((0, 100),))['0-100_hz']
            self.assertAlmostEqual(measured, expected)
        with self.assertRaises(ValueError):
            motion.frequency_band_rms([0, .005, .011, .015], np.zeros((4, 3)))

    def test_prediction_uses_world_angular_rate_and_gravity_free_acceleration(self):
        q = np.array([[np.sqrt(.5), np.sqrt(.5), 0, 0]])
        p, orientation = motion.predict(np.array([[1., 2, 3]]), q, np.array([[.1, 0, 0]]),
                                       np.array([[0, 0, 2.]]), np.array([[1., 0, 0]]), .02)
        np.testing.assert_allclose(p, [[1.0022, 2, 3]])
        # Rz(.04) * Rx(pi/2); body-frame angular velocity has the opposite y sign.
        np.testing.assert_allclose(orientation, [[np.cos(.02)*np.sqrt(.5), np.cos(.02)*np.sqrt(.5),
                                                 np.sin(.02)*np.sqrt(.5), np.sin(.02)*np.sqrt(.5)]])

    def test_constant_velocity_and_angular_rate_have_no_high_frequency_variation(self):
        t = np.arange(0, 4, .005)
        v = np.tile([.01, -.02, .03], (len(t), 1))
        w = np.tile([0, 0, .02], (len(t), 1))
        q = np.column_stack([np.cos(.01 * t), np.zeros(len(t)), np.zeros(len(t)), np.sin(.01 * t)])
        result = motion.window_metrics(t, t[:, None] * v, q, v, w, np.zeros_like(v))
        for prediction in result['predictions'].values():
            self.assertLess(prediction['position_hp_mm']['rms'], 1e-8)
            self.assertLess(prediction['orientation_hp_deg']['rms'], 1e-8)
        self.assertLess(result['velocity_hp_mm_s']['rms'], 1e-8)

    def test_acceleration_noise_is_visible_in_predictions_even_with_constant_pose(self):
        t = np.arange(0, 4, .005)
        zero = np.zeros((len(t), 3))
        q = np.tile([1., 0, 0, 0], (len(t), 1))
        a = zero.copy()
        a[:, 0] = np.sin(2 * np.pi * 20 * t)
        result = motion.window_metrics(t, zero, q, zero, zero, a)
        self.assertEqual(result['position_hp_mm']['rms'], 0)
        noise = result['acceleration_hp_m_s2']['rms']
        for horizon in (.01, .02, .03):
            self.assertAlmostEqual(result['predictions'][str(horizon)]['position_hp_mm']['rms'],
                                   .5 * horizon * horizon * noise * 1000)
            self.assertAlmostEqual(result['predictions'][str(horizon)]['frequency_bands']['position_mm']['15-50_hz'],
                .5 * horizon * horizon * result['frequency_bands']['acceleration_m_s2']['15-50_hz'] * 1000)

    def test_large_integer_sensor_epochs_and_missing_motion_are_not_silently_mixed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'motion.rec.gz'
            with gzip.open(path, 'wt') as f:
                f.write('0 WM0 CONFIG '+json.dumps(dict(device_serial_number='tracker'))+'\n')
                for i in range(501):
                    tick = (1 << 58) + i * 192000
                    t = i * .004
                    f.write(f'{t} WM0 POSE_STATE {tick} 48000000 0 0 0 {t*.01} 0 0 1 0 0 0\n')
                    f.write(f'{t} WM0 POSE_MOTION {tick} 48000000 .01 0 0 0 0 0 0 0 0\n')
            result = motion.evaluate(path, dict(held=(0, 2)))['objects']['tracker']['held']
            self.assertGreater(result['samples'], 300)
            self.assertLess(result['position_hp_mm']['rms'], 1e-8)
            missing = motion.evaluate(path, dict(held=(0, 3)))['objects']['tracker']['held']
            self.assertIn('unavailable', missing)

    def test_reused_receiver_and_invalid_frequency_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'invalid.rec.gz'
            for rows in [
                ['0 WM0 CONFIG {"device_serial_number":"first"}',
                 '1 WM0 CONFIG {"device_serial_number":"second"}'],
                ['0 WM0 POSE_MOTION 1 0 0 0 0 0 0 0 0 0 0'],
            ]:
                with gzip.open(path, 'wt') as f:
                    f.write('\n'.join(rows)+'\n')
                with self.assertRaises(ValueError):
                    motion.read(path)


if __name__ == '__main__':
    unittest.main()
