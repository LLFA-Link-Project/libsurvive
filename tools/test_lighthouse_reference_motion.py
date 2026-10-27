import math
import unittest

import lighthouse_compare as c
from lighthouse_reference_motion import tracking_reference_state


def exponential(w, dt):
    rate = math.sqrt(sum(x*x for x in w))
    return (math.cos(rate*dt/2), *(x*math.sin(rate*dt/2)/rate for x in w))


def xyzw(q):
    return (*q[1:], q[0])


class ReferenceMotion(unittest.TestCase):
    def test_future_rotation_matches_physical_body_motion_in_both_frames(self):
        # Noncommuting world/body rotations distinguish the two conventions.
        world = exponential((.7, -.2, .4), 1)
        body = exponential((-.4, 1.1, .3), 1)
        driver_omega = (.12, -.07, .19)
        body_omega = c.rotate(c.qinv(body), driver_omega)
        for frame, omega in [('body', body_omega), ('driver-world', driver_omega)]:
            for sign in (1, -1):
                row = dict(world_from_driver_translation_m=(3, -2, 1),
                    world_from_driver_orientation_xyzw=xyzw(world), position_m=(1, 2, 3),
                    orientation_xyzw=xyzw(tuple(sign*x for x in body)),
                    linear_velocity_m_s=(.03, .02, -.01), angular_velocity_rad_s=omega,
                    # Head offset must not enter the IMU-origin conversion.
                    driver_from_head_translation_m=(9, 8, 7),
                    driver_from_head_orientation_xyzw=(0, 0, 1, 0))
                position, q, velocity, w = tracking_reference_state(row, angular_velocity_frame=frame)
                self.assertLess(c.distance(position, c.add((3, -2, 1), c.rotate(world, (1, 2, 3)))), 1e-12)
                self.assertLess(c.distance(velocity, c.rotate(world, (.03, .02, -.01))), 1e-12)
                for horizon in (.01, .02, .03):
                    predicted = c.qmul(exponential(w, horizon), q)
                    physical = c.qmul(world, c.qmul(exponential(driver_omega, horizon), body))
                    self.assertLess(min(c.distance(predicted, physical),
                        c.distance(tuple(-x for x in predicted), physical)), 1e-12)

    def test_frame_is_required_and_unknown_frame_rejected(self):
        with self.assertRaises(TypeError):
            tracking_reference_state({})
        row = dict(world_from_driver_translation_m=(0, 0, 0),
            world_from_driver_orientation_xyzw=(0, 0, 0, 1),position_m=(0, 0, 0),
            orientation_xyzw=(0, 0, 0, 1),linear_velocity_m_s=(0, 0, 0))
        with self.assertRaises(ValueError):
            tracking_reference_state(row, angular_velocity_frame='unknown')


if __name__ == '__main__':
    unittest.main()
