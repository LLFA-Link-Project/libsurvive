import unittest

import numpy as np

import lighthouse_optical_metrics as m


class OpticalMetricsTests(unittest.TestCase):
    @staticmethod
    def rows(times, omega=1):
        times = np.asarray(times)
        q = np.column_stack([np.cos(times*omega/2), times*0, times*0, np.sin(times*omega/2)])
        p = np.column_stack([times*.1, times*.2, times*-.3])
        return np.column_stack([times, times, p, q])

    def test_uneven_constant_velocity_has_zero_midpoint_residual(self):
        rows = self.rows([0, .004, .013, .020, .030])
        result = m.summarize(rows, 0, 1)
        self.assertLess(result['position_midpoint_residual_mm']['maximum'], 1e-10)
        self.assertEqual(result['samples'], 5)

    def test_clock_gap_does_not_become_a_large_step(self):
        rows = self.rows([0, .01, 1, 1.01])
        rows[2:, 2] += 10
        result = m.summarize(rows, 0, 2)
        self.assertEqual(result['gaps_over_20ms'], 1)
        self.assertLess(result['position_step_mm']['maximum'], 4)
        self.assertIsNone(result['position_midpoint_residual_mm'])

    def test_gyro_comparison_is_invariant_to_quaternion_sign(self):
        rows = self.rows(np.arange(.01, .99, .01), omega=3)
        rows[::2, 5:] *= -1
        times = np.arange(0, 1.001, .001)
        gyro = np.column_stack([times, times*0, times*0, times*0+3])
        result = m.gyro_residual(rows, m.gyro_curve(gyro))
        self.assertLess(result['maximum'], 1e-8)
        rows[20, 5:] = [1, 0, 0, 0]
        self.assertGreater(m.gyro_residual(rows, m.gyro_curve(gyro))['maximum'], 30)

    def test_gyro_gaps_and_missing_optical_ticks_are_not_interpolated(self):
        rows = self.rows([.01, .02, .03, .04, .05, .06])
        gyro = np.array([[.0, 0, 0, 1], [.01, 0, 0, 1], [.02, 0, 0, 1],
                         [.05, 0, 0, 1], [.06, 0, 0, 1], [.07, 0, 0, 1]])
        result = m.gyro_residual(rows, m.gyro_curve(gyro))
        self.assertEqual(result['count'], 2)
        rows[:, 1] = np.nan
        self.assertIsNone(m.gyro_residual(rows, m.gyro_curve(gyro)))
        gyro[2, 0] = gyro[1, 0]
        self.assertIsNone(m.gyro_curve(gyro))


if __name__ == '__main__':
    unittest.main()
