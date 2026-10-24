# Native Lighthouse capture replay

`tools/lighthouse_replay.py` runs native WiVRn Lighthouse recorder sessions without
accessing USB hardware. It requires Python 3 and a libsurvive build containing
`driver_usbmon`, `driver_vive`, the posers, and `survive-cli`.

```sh
python3 tools/lighthouse_replay.py /path/to/survive-cli \
  ~/LH-Dump/2026-10-18_220701_114777916_3198653 \
  --compare --output /tmp/lighthouse-results
```

The output directory must be new. Each run uses a fresh configuration, disables
threaded posers and the threaded scene solver, and replays without pacing. Logs,
pose recordings, and `results.json` are retained there. The original sessions are
read only. Files contain device identities/configuration and are private replay
artifacts, not anonymized exports.

Specify the dataset parent to process its immediate session directories. Only
sessions whose manifest says `complete` and reports no errors are replayed;
others are listed as skipped. This qualification does not replace the recorder's
full-content/hash validation. Original USB/pose loss counters are included in the
report. Missing `.usbdevs` files are reported, and filesystem device-config
fallback is never enabled. A timeout, process failure, corrupt output, nonfinite
pose, or zero output poses makes the command fail. A zero-pose result can also
mean a capture lacks the configuration/optical information needed to solve.

`--seconds 600` limits playback to its first ten minutes. `--repeat 2` is the
default; each run reports pose counts, quaternion normalization residuals,
receive-clock output gaps and a pose-stream SHA-256 for reproducibility.
Output gaps are not classified as tracking loss: the recording's state events
are needed to distinguish loss, disconnection and missing updates.

## Fixes found using LH-Dump

Passive device registration left `nextCfgSubmitTime` at zero. Watchman v2
interpretation treats a nonnegative value as a pending configuration transfer
and discards the entire report. Passive replay does not submit a transfer that
could clear this state, even after successfully loading captured device config.
Initializing this field to its inactive value restores decoding and solving.
The live USB configuration state machine continues to set its own schedule.

Packet handling now checks captured lengths before reading USB headers or
payloads, copying reports, assembling configuration chunks, or decoding RF/IMU
and feature reports. IMU samples require all thirteen bytes and failure stops
interpretation of that packet. These checks prevent malformed data from becoming
tracking updates. Playback waits are bounded so closing across a large capture
gap does not wait for the entire gap; initial thread synchronization also covers
captures containing only invalid headers. Integer field access supports both
older libpcap headers and libpcap 1.11 ABI wrappers.

## Solver and clock corrections

The optimizer now scales covariance using residual degrees of freedom (number
of residuals minus free parameters), rather than number of free parameters.
The axis-angle to quaternion covariance Jacobian now reads the full parameter
index when preceding poses are fixed. Both errors have numerical regression
cases that fail with the previous implementation. The raw-observation diagnostic
flag is also respected rather than overwritten during initialization.

Optical pair validity now reconstructs the 64-bit device-clock epoch before
comparing timestamps; previously it failed after a 32-bit wrap (about 89.5 seconds
at 48 MHz). Runtime conversion now subtracts in floating point, avoiding unsigned
underflow when the receive clock precedes the sensor clock.

Passive USB replay used to reset its optical quarantine on ordinary feature
reads and haptic commands. Only actual USB/RF mode switches now start quarantine.
This changes which optical data reach the solver and its initial calibration.
It is a replay decoder correction, not evidence of improved live USB handling.

## Reference comparison

The format was checked against `~/WiVRn/docs/lighthouse-recorder.md`. Use
`--compare` to include held-out disagreement, reference coverage and explicit
reference state changes in `results.json`. Valve poses are reference estimates,
not ground truth. The comparator uses captured object configuration for identity,
converts xyzw to wxyz, and composes world/driver/head transforms to compare the
same head reference point.

`--record-pose-timing` adds optional `POSE_TIMING` records containing full 64-bit
sensor ticks, frequency and last optical/IMU ticks, and `INPUT_TIME` records
containing sensor ticks and original USB receive microseconds. Normal recordings
are unchanged. Clock alignment uses nearby input anchors and the recorded
realtime/monotonic brackets. Clock steps, overlapping ambiguous clock intervals,
extrapolation beyond clock brackets, and excessive input timing spread are
rejected. The report retains clock mapping estimates and input arrival spread.
These do not bound unknown device-to-USB latency, Valve prediction or driver
latency. The report's capture start/end fields mean first/last recorded input.

After ten seconds of warmup from the first input, the first third of the remaining
capture fits one common rigid transform, with equal total weight per object and
orientation axes of length 0.25 m. Only the remaining two thirds are evaluated.
There is no fitted time shift. Reference interpolation rejects invalid poses and
gaps above 20 ms; it does not infer loss from missing callbacks. Coverage uses
valid reference bins and counts objects with no replay output. It indicates
output availability, not positional correctness. Source loss counters and state
events must be considered alongside it. Recovery after explicit reference
reacquisition uses a declared diagnostic threshold (three samples within 50 mm
and 5 degrees), not a steamvr_lh equivalence criterion.

The comparator currently loads recordings and reference poses in memory. Use
bounded sessions for comparison; full multi-hour comparisons need a streaming
implementation. `--seconds` bounds replay but does not bound reference loading.

## Offline results, 2026-10-23

Four complete captures were replayed twice with fresh configuration. Each pair
produced identical pose hashes and no nonfinite components. Counts with the
current decoder were 82,071 (255 s capture), 7,295 (38 s), 54,561 (337 s), and
60,098 (165 s). The original repository decoder produced zero poses for these
objects because of the passive configuration state bug.

The table compares the working passive decoder before the solver/quarantine
corrections with the current implementation. Both use the same held-out protocol
above and independent fresh calibrations. Values are position RMSE in mm /
orientation RMSE in degrees. These are disagreements with the reference, not
absolute tracking errors. Device suffixes identify objects consistently across
captures.

| Capture | Object | Before | Current |
| --- | --- | ---: | ---: |
| `2026-10-18_220050` (255 s) | DF24384D | 904.1 / 76.36 | 405.8 / 28.45 |
| same | F39A0C97 | 847.8 / 128.10 | 42.4 / 2.50 |
| `2026-10-18_220701` (38 s) | DF24384D | 17.8 / 1.69 | 60.8 / 0.68 |
| same | F39A0C97 | 19.0 / 1.89 | 60.0 / 0.54 |
| `2026-10-19_185631` (337 s) | 5D3D7A83 | 18.3 / 0.42 | 18.3 / 0.41 |
| `2026-10-20_125408` (165 s) | 5D3D7A83 | 76.8 / 1.71 | 62.1 / 1.87 |
| same | DF24384D | 124.5 / 2.45 | 116.0 / 2.76 |
| same | F39A0C97 | 81.8 / 1.98 | 69.7 / 2.22 |

The large remaining error after an optical gap in the 255 s capture and the
short-capture position regression are unresolved. The changes do not establish
steamvr_lh equivalence. Parameter sweeps, periodic reseeding, and IMU calibration
unit changes did not improve results consistently and were not adopted. Live
tracking, independent ground truth, full multi-hour stability and controlled
occlusion/reacquisition trials remain necessary.

## Regression checks

Synthetic tests cover malformed USB/RF/IMU packets, passive registration,
quarantine transitions, shutdown across a one-hour gap, covariance mathematics,
clock wraps and signed runtime offsets. Comparator tests cover clock ambiguity,
rigid transforms, invalid interpolation, full-width timing, held-out evaluation,
missing devices and explicit loss/reacquisition events.

```sh
ctest --test-dir /path/to/build -R '^(watchman|usbmon_packets|optimizer|sensor_activations|lighthouse_compare)$' --output-on-failure
```

These pass with AddressSanitizer/UndefinedBehaviorSanitizer (leak detection
disabled). Existing reproject, generated-math, barycentric SVD, angular-velocity
and config-export checks also pass. The upstream downloaded replay corpus was
unavailable and was not evaluated.

## Follow-up investigation

Commit `4df6172` is the baseline for the following experiments. Always starting
from the IMU pose instead of the previous optical solution changed the long
capture's DF24384D disagreement only from 405.8 mm / 28.45 degrees to 404.7 mm /
28.40 degrees, with mixed effects elsewhere; this change was not retained.
Disabling the optical velocity model gave 405.2 mm / 28.04 degrees, while numeric
Jacobians gave 405.8 mm / 28.45 degrees. Neither explains the large remaining
error. These are diagnostic experiments, not additional improvements.

A separate optimizer error was reproduced: the paired-measurement shortcut could
combine adjacent axes belonging to different objects or lighthouses when sensor
indices happened to match. It now requires both identities to match. The new
synthetic test compares the shortcut with independent projection at known exact
poses, and fails before the fix. Optimizer tests pass under ASan/UBSan. All four
capture pose hashes are unchanged by this correction; it is not the explanation
for the observed long-capture divergence.

The exported activation-count helper also truncated 64-bit measurement ages and
its 64-bit tolerance to 32 bits. An unobserved sensor could therefore look fresh
again after roughly 89.5 seconds. Both quantities now retain their full width.
A synthetic test fails on the old implementation (two stale readings counted as
fresh), and also checks a deliberately large 64-bit tolerance. This helper is
not called by the current default MPFIT path, so this fix does not account for
the remaining reference disagreement.

Huber loss experiments at 1 and 5 mrad did not remove the long-capture divergence:
DF24384D remained near 405 mm / 28 degrees. The experimental implementation was
removed; no loss-function or threshold tuning was adopted.

The BaryCentricSVD comparison exposed a crash in the common pose dispatcher:
solvers may omit covariance, but the dispatcher unconditionally constructed a
matrix view of the null pointer. It now passes the missing covariance through to
the tracker, which supplies its configured observation variance. A synthetic
dispatch test reproduces the previous segmentation fault and passes under
ASan/UBSan after the fix. A separate early-initialization problem in the SVD poser
was then exposed by the real-capture Debug run and is investigated separately.

The SVD poser now defers construction until sensor geometry is available. USB
replay may deliver IMU data before captured configuration; constructing the SVD
model then permanently captured zero sensors, leading to an out-of-range
correspondence assertion (and invalid access without assertions). A dedicated
regression covers IMU-before-config, subsequent initialization and cleanup. The
previously failing 20-second native BS 1.0 prefix now completes under ASan/UBSan.
