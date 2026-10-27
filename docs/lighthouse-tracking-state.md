# Optional native tracking-state diagnostics

`record-tracking-state=1` adds diagnostic records to an ordinary native recording
(`record=/path/native.rec.gz`). The default is OFF. This does not change filtering,
calibration, prediction or the existing POSE/POSE_TIMING/VELOCITY layouts.
`record-pose-timing=1` and `record-cal-imu=1` are useful companion settings.

Each line retains the usual leading library receive-clock seconds and object
codename. Integer sensor clocks are device-local ticks; their frequency is not
assumed to be a wall clock. Quaternions below use **wxyz**.

```
receive_s object POSE_STATE tick hz last_movement_tick stationary_ticks floor_offset_m raw_imu_x raw_imu_y raw_imu_z raw_qw raw_qx raw_qy raw_qz
receive_s object POSE_MOTION tick hz vx vy vz wx wy wz ax ay az
receive_s object POSE_BIAS tick bias_error_state_size gyro_bias_x gyro_bias_y gyro_bias_z accel_bias_x accel_bias_y accel_bias_z accel_scale_x accel_scale_y accel_scale_z correction_qw correction_qx correction_qy correction_qz
receive_s object POSE_BIAS_VAR tick gyro_var_x gyro_var_y gyro_var_z accel_var_x accel_var_y accel_var_z
```

POSE_STATE stores the object's raw `OutPoseIMU` before floor subtraction and head
or interaction-profile offsets, together with the current floor offset. It is
recorded while handling a POSE callback; the ordinary POSE can use another
reference point. Compare them using the recorded configuration/transforms.
`stationary_ticks` is the current activation detector's duration since movement,
not an independently measured assertion that the device is physically still.
Its thresholds are in the recorded OPTION settings.

POSE_MOTION joins POSE_STATE by the full integer tick and object identity. It
stores the tracker's world-frame linear velocity (m/s), world-frame angular
velocity (rad/s), and the exported gravity-free world-frame linear acceleration
(m/s²) at that pose update. The acceleration is the value provided to consumers
for prediction. A device without a tracker has no POSE_MOTION record. Use this
tag when evaluating prediction; the separate VELOCITY callback can follow POSE
and does not identify its own measurement epoch.

`tools/lighthouse_motion_metrics.py` measures pose, velocity, acceleration and
10/20/30 ms predicted-pose variation in explicitly declared receive-time windows.
It joins integer sensor ticks before interpolation and rejects missing motion
states, incomplete windows and gaps over 20 ms. Its centered trend subtraction
is for analysis only. These statistics contain real motion as well as noise;
lower variation alone does not establish better tracking or lower latency.

The analysis also reports vector RMS in 0.5–2, 2–5, 5–15 and 15–50 Hz bands
for each state and prediction horizon. It removes a fitted straight line, applies
a Hann window and normalizes spectral power by window energy. This separates
frequency ranges without treating high frequencies as noise: hand tremor and
other real motion can occupy the same bands. The input grid must be uniform.

POSE_BIAS stores the filter's residual IMU bias/scale/correction state. Gyro bias
uses rad/s; accelerometer bias uses the same g units as the calibrated IMU
measurements. These residual values are applied in addition to factory calibration.
The state-size field identifies whether all bias error-state coordinates are
active. POSE_BIAS_VAR is emitted when the covariance includes all three gyro-bias
coordinates; the first three entries use (rad/s)^2, the final three use g^2.
A device without a tracker has no POSE_BIAS or POSE_BIAS_VAR record.

These tags are emitted on pose updates, rather than every IMU update. Keep
calibrated `I` records and pose/velocity timestamps when comparing bias with raw
angular rate. Velocity is a separate native record and can be delivered after
the corresponding POSE line. Native playback ignores these optional POSE_ tags;
it does not feed diagnostic bias/pose values back into the solver.

This recorder is synchronous. Its extra work is confined to explicitly enabled
recording; it has no asynchronous writer/drop accounting. Retain the library and
plugin revisions, configuration, source data and write/integrity results, and
measure logging overhead when drawing conclusions about a live application.
Since `612629c`, output uses a recorder-owned mutex: timestamps and payloads,
CONFIG, matrices and covariance records are serialized across worker/API writers.
Gzip EOF/CRC validity alone does not establish native record integrity: concurrent
writes in an older recording can leave NUL-bearing/interleaved records inside an
otherwise valid gzip stream. The calibration audit reports these lines explicitly.
See [the WiVRn measurement handoff](libsurvive-micro-motion-handoff.md).

With `record-tracking-state=1`, global scene optimization also emits `INFO LOG`
diagnostics for failed and successful candidates:

```
GSS_QUALITY status=... scenes=... optical_rms_rad=... optical_count=... object_gravity_rms_deg=... object_gravity_count=...
GSS_LIGHTHOUSE_GRAVITY index=... channel=... angle_deg=...
```

The optical RMS is the solver's existing fitted-light residual statistic. Object
gravity angles are recomputed from the returned scene poses and their measured
acceleration; lighthouse angles compare the returned lighthouse pose with its
OOTX acceleration. These compare local measured gravity with world up, regardless
of mounting angle. Unavailable gravity produces `nan`. Candidate diagnostics do
not indicate acceptance and do not change acceptance rules or filter settings.
They measure internal agreement, not ground-truth accuracy. See
[the calibration investigation](lighthouse-calibration-failure-results.md).

Since `4def8c2`, the legacy `up err` statistic sums squared vector residuals,
rather than squaring their signed sum. It includes object and lighthouse gravity
constraints. Its numeric values are not directly comparable with older logs.

The optimizer represents a camera as world-to-lighthouse, while lighthouse
observations use lighthouse-to-world. The MPFIT delivery paths now transport
the joint camera covariance through pose inversion, including rotation-to-position
lever arms. Scene coordinate changes also transport the inter-lighthouse blocks,
rather than just the individual marginal covariances. The helpers in `poser.h`
allow overlapping input/output storage and preserve inactive pose sentinels.

`survive_covariance_apply_pose_to_poses` treats its left transform as a fixed
coordinate change. It does not model the statistical uncertainty of choosing an
initial world frame from calibration data. Persistent map/object correlations
also remain outside the default compressed optical-pose update. Correcting this
representation does not by itself establish lower pose or prediction jitter.
The physical perturbation, quaternion derivative and aliasing checks are in
`pose_covariance.c`; current replay evidence and limitations are recorded in
[the stability investigation](lighthouse-stability-20261026.md).

When fusing a lighthouse observation into its cached prior, the quaternion is
aligned to the hemisphere of the prior actually restored for that update.
Changing its sign also changes the position/quaternion covariance cross terms;
the caller's pose and covariance stay untouched. Without this coordinate change,
equivalent quaternion signs could produce different physical lighthouse updates.
`lighthouse_observation.c` covers both prior signs and a stale opposite state.
The replay controls checked so far do not use this cached-prior branch, so its
physical regression test does not establish a reduction in recorded jitter.
