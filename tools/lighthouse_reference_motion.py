"""Convert a raw DriverPose recording at its tracking reference point.

The angular-velocity convention must be explicit. OpenVR's documentation says
driver world, but recorded steamvr_lh streams use body angular velocity, as
confirmed against their orientation increments and Monado's conversion. Do not
apply that stream-specific convention to other drivers without checking it.
"""

import lighthouse_compare as c


def tracking_reference_state(row, *, angular_velocity_frame):
    """Return world position, wxyz orientation, linear and angular velocity.

    No head offset: the original position and linear velocity describe the
    tracking reference. World translation affects position only. The angular
    vector is rotated by either world_from_driver or the full body orientation.
    Validity and timestamps remain the caller's responsibility.
    """
    world = (row['world_from_driver_translation_m'],
             c.xyzw(row['world_from_driver_orientation_xyzw']))
    position, orientation = c.compose(
        world, (row['position_m'], c.xyzw(row['orientation_xyzw'])))
    linear_velocity = c.rotate(world[1], row['linear_velocity_m_s'])
    if angular_velocity_frame == 'body':
        angular_rotation = orientation
    elif angular_velocity_frame == 'driver-world':
        angular_rotation = world[1]
    else:
        raise ValueError('Angular velocity frame must be body or driver-world')
    angular_velocity = c.rotate(angular_rotation, row['angular_velocity_rad_s'])
    return position, orientation, linear_velocity, angular_velocity
