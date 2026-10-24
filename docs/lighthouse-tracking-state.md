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
