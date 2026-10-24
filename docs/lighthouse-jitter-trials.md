# Handheld micro-motion, jitter and IMU bias trials

The user reports stable rigid rest but visible jitter/sliding while holding a
controller with small movements, including with additional prediction disabled.
This is the target; reducing rigid-static RMS alone does not establish a fix.
No SteamVR lighthouse extrinsic calibration is imported in these experiments.

Commit `d50e4e0` fixes a concrete IMU-bias filter defect. The configured bias
random walk was assigned to the kinematic pose model, whose initialization then
erased it. The separate bias covariance received no process noise. The fix adds
configured accel/gyro bias noise to that filter using its error-state indices.
Two native tests fail before and pass after, including runtime configuration
changes, both process-noise models and an unchanged predicted bias mean.
This restores the intended uncertainty model, not a proven cure for the symptom.

The supplied ~48-second static recording was replayed from fresh configuration.
Recorded LH_POSE lines were removed; factory sensor geometry/IMU intrinsics and
raw optical/IMU input were kept. Automatic floor-height changes were disabled
on both sides to isolate raw estimator movement. This is a diagnostic choice,
not a proposed product default. Metrics below use 20–45 s, uniform 200 Hz
resampling, and zero additional prediction. They are replay stability, not true
pose accuracy or a new live WiVRn measurement.

| Device | Position RMS, old → fix (mm) | Orientation RMS (degrees) |
| --- | --- | --- |
| WM1 / right Index | 0.04368 → 0.04367 | 0.03682 → 0.03697 |
| WM2 / left Index | 0.03820 → 0.03763 | 0.10935 → 0.10846 |
| WM0 / tracker | 0.06156 → 0.06166 | 0.14922 → 0.14965 |

Changes are small and mixed. In the old replay, gyro-bias covariance falls from
about 1.9e-7 to 6.3e-9 (rad/s)^2 between 20 and 45 s. The fix retains about 4.0e-8
at 45 s. This demonstrates the changed adaptation uncertainty; it does not prove
that IMU bias causes the visible sliding. A false stationary decision could
learn real micro-motion as bias; the live A/B procedure explicitly checks this.
Two full BS 1.0/2.0 motion captures
were also compared at 50 Hz. Changes are small and mixed; BS 1.0 F39 orientation
RMSE increases from 2.45 to 2.61 degrees. Whole-corpus post-fix validation and
annotated handheld micro-motion/live latency measurements remain outstanding.

Exploratory parameter changes were **not adopted**:

- Stronger stationary ZVU greatly reduces rigid-static position variation, but
  can suppress the very micro-motion the user wants to preserve.
- Lower angular process noise reduces the fast part of static angular jitter.
  At 0.6 instead of 60, the left Index's 0.5 s high-pass RMS drops from about
  0.0280 to 0.0058 degrees, but total RMS only changes from 0.1093 to 0.1064.
  Slower variation remains. Motion error changes are mixed. At 0.006, BS 1.0 DF
  angular RMSE worsens from 2.73 to 5.59 degrees. Added live lag is unmeasured.
- Tightening the shared motion detector loses all output in the BS 1.0 trial.
  The detector also gates self-calibration, so changing it globally is unsafe.
- Disabling the stationary observation window or stationary ZVU has mixed
  motion results and does not establish a micro-motion fix.

Commit `1fc8b3d` adds optional [tracking-state records](lighthouse-tracking-state.md)
for raw IMU pose, floor offset, stationarity, residual IMU bias and covariance.
The logger is OFF by default. With it enabled, all 21,437 static replay poses have
matching state/bias/variance records, and the old pose-stream hash is unchanged.
The 15 relevant CTest tests, ASan/UBSan bias/reporting tests and the complete
native diagnostic replay pass. Native recording remains synchronous; logging
load must be considered in live measurements.

The [WiVRn Codex handoff](libsurvive-micro-motion-handoff.md) specifies actual
handheld micro-motion, slow translation/rotation, movement onset and reversal,
with additional prediction disabled. It uses the same updated binary for A/B:
full state in both, bias process noise zero for the old effective behavior and
1e-8 for the fixed behavior. The A setting exactly reproduces the old static and
two motion pose streams; B with full state exactly reproduces the normal fixed
static stream. This avoids mixing different library builds in a live comparison.

[Machine-readable results](lighthouse-jitter-trials.json) retain settings, support
counts, coverage, calibration fits, source/build hashes and rejected trials.
Early copied-CLI exploratory trials retained a build-directory RUNPATH, so
adjacent-file hashes alone did not attest their loaded libraries. The bias A/B
controls use a rebuilt $ORIGIN CLI and verified library resolution; their control
streams exactly reproduce both exploratory motion baselines and the static
baseline. The earlier four cadence streams were also reproduced after explicitly
binding the frozen library and verifying its resolved path. The frozen
`/tmp/lh-build` whole-corpus baseline is unaffected by this provenance caveat.
