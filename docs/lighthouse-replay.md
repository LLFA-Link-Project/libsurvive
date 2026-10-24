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

## BS 2.0 captures, 2026-10-24

Four newly supplied complete captures were evaluated at commit `59eed6c`. Each
was replayed twice with fresh configuration; all four pairs have identical pose
hashes and no nonfinite components. The three-station captures include radio
channels 0, 1 and 14; all three OOTX identities are decoded. One capture reports
only a single observed base station. This is the reference benchmark after the
stability fixes, not a demonstrated BS 2.0 accuracy improvement.

| Capture | Duration | Object | Position RMSE, mm | Orientation RMSE, degrees |
| --- | ---: | --- | ---: | ---: |
| `2026-10-23_222834` | 71 s | DF24384D | 16.72 | 3.405 |
| same | | F39A0C97 | 24.33 | 1.579 |
| `2026-10-23_223404` | 114 s | DF24384D | 8.97 | 0.581 |
| same | | F39A0C97 | 18.89 | 1.472 |
| `2026-10-23_223623` | 84 s | DF24384D | 70.88 | 5.216 |
| same | | F39A0C97 | 53.18 | 2.347 |
| `2026-10-23_223843` | 347 s | DF24384D | 8.86 | 2.347 |
| same | | F39A0C97 | 8.45 | 1.997 |

Full measurements, source loss counters, reference state events and hashes are in
`lighthouse-bs2-results.json`. Coverage after the ten-second warmup ranges from
about 89.6% to 96.6%; first output occurs later than the reference while captured
calibration is acquired. Coverage is output availability, not correctness.
Results retain the earlier timing, calibration and ground-truth limitations.
The short 10/26/31-second recordings were not used as accuracy benchmarks.

Additional experiments did not establish a consistently better default:

* Raw optical observations and lower observation covariance did not remove the
  orientation discrepancy. Disabling global calibration worsened the 71-second
  recording substantially despite lower position differences in the 347-second
  recording.
* Accumulating small optical angle changes detects slow motion missed by the
  consecutive-sample detector. However, the candidate altered static scene
  selection and worsened the 347-second recording's F39A0C97 disagreement from
  8.45 mm / 1.997 degrees to 11.77 mm / 3.291 degrees. The candidate and its
  public-structure change were removed.
* Relaxing the calibration gravity prior improved the 71-second recording but
  slightly worsened the 347-second recording. No default was changed on that
  evidence alone.

The optical pairing regression now covers both lighthouse generations with
unequal, nonzero per-plane calibration and points in front of the stations.
Eleven targeted tests pass; six decoder/optimizer/timing/poser/comparator tests
also pass under ASan/UBSan (leak detection disabled). The interrupted 30-minute
replay from the previous temporary environment did not produce a completed
result and is not reported as a pass.

The gravity-prior experiment was also checked on four BS 1.0 captures. It
worsened the 165-second multi-device capture's tracker position disagreement from
62.08 mm to 76.77 mm and F39A0C97 from 69.66 mm to 82.55 mm, confirming that the
short BS 2.0 improvement does not justify a new default. Compact experiment
measurements are retained in `lighthouse-gravity-experiments.json`.

The entire 71-second three-station BS 2.0 capture also completed in a Debug build
under ASan/UBSan with no reported sanitizer errors, using fresh configuration and
no physical USB devices. This is one complete capture validation, not proof of
multi-hour stability or accuracy equivalence.

## Entire-corpus evaluation

The 2026-10-24 snapshot contains 73 sessions and 46,400,284,364 stored bytes:
25 complete, 31 still marked recording, and 17 abnormal. It includes data added
after the earlier approximately 25 GB description. `lighthouse-full-inventory.json`
records every session, recorded duration where available, source sizes, and
manifest errors. Every file was read for SHA-256 verification; all 378 declared
hashes match, no declared file is missing, and no file changed during reading.
Hash verification is separate from tracking evaluation.

The frozen original-CLI baseline completed every capture without prefix limits,
with fresh configuration. It emitted 48,573,129 finite poses across 48 captures;
25 captures emitted no poses. No nonfinite poses were recorded. All 73 reference
streams were assessed, parsing 123,522,379 callbacks. The native reader observed
318,291,080 recognized-device USB packets; all 42 available manifest packet
counters match the native counters. Initial and final SHA-256 audits agree,
with no source/build changes or added/missing sessions. One selected run per
capture does not establish whole-corpus reproducibility. Incomplete/error-marked sessions are attempted explicitly with
`--include-incomplete`; their results remain unqualified even when they emit
finite poses. Empty captures, missing configuration, truncated streams, solver
failures and unavailable comparisons must remain visible in the final ledger.
The inventory and the completed baseline ledger are separate artifacts. See
`lighthouse-full-results.md`, `lighthouse-full-results.csv` and
`lighthouse-full-results.json`; `lighthouse-full-source-hashes.json` retains the
source hash audit. All gzip streams were checked: 30 recording-marked sessions
have truncated streams, while complete/abnormal sessions passed their gzip checks.

For the 25 complete, integrity-checked captures, pooled matched held-out
reference disagreement is **239.51 mm / 8.837 degrees RMSE** across 704,693
samples. Qualified reference coverage is **91.212%** (10,846,565 / 11,891,626
reference bins). This is an original-CLI baseline: 22 captures contain MENU
reset events, including the late reset described below. It is not a clean
tracking-engine equivalence result. The 35 captures without held-out error
samples retain their output/coverage/qualification states; empty references,
short captures and insufficient alignment are not silently removed. In
particular, two complete captures emit no native poses, and four complete
captures have no held-out error samples.

Long replay runs can now checkpoint atomically after each run and resume with
`--resume`. Resume checks the binary/plugin content hashes, replay input hashes and settings
before reusing results. A changed capture is rejected without overwriting its
previous checkpoint. `--record-timing` records timing without loading reference
streams, allowing replay and reference evaluation to run independently.

```sh
python3 tools/lighthouse_replay.py /path/to/survive-cli ~/LH-Dump \
  --include-incomplete --record-timing --repeat 1 --timeout 14400 \
  --output /path/to/full-replay --resume
python3 tools/lighthouse_compare_disk.py /path/to/session /path/to/session-0.rec.gz \
  --database /path/to/new-index.sqlite --output /path/to/new-comparison.json
```

The disk comparator indexes all input timing anchors and all reference validity
states in SQLite, with a 32 MiB SQLite page cache. Raw samples are streamed into
the index; the full reference/input streams are not held in Python lists. It
maps a 50 Hz replay stream for coverage, then samples pose error at 5 Hz to bound
the alignment/error working set for multi-hour captures. Training remains the
first third after warmup, evaluation the last two thirds, with no fitted time
shift. The disk comparator uses the capture clock-sync sample span independently
of native output/input, so a trailing input or pose failure does not shorten
the coverage denominator. Unsampled capture edges outside the first/last clock
sample are explicitly excluded. Too few alignment pairs leaves error unavailable
while retaining the independent coverage measurement. It sorts by full sensor ticks and segments receiver identity changes.
Sampling rate and timing assumptions are included in each result. Five-hertz
errors cannot resolve the existing <=50 ms reacquisition criterion; that recovery
metric is explicitly unavailable at this rate. Native reference state events
remain reported. Per-minute held-out error bins use the same fitted transform;
they do not refit each minute to conceal drift or outliers. These results are not numerically interchangeable with earlier
50 Hz error tables.

Before switching to the independent capture-clock window, a 50 Hz disk-backed
comparison of the 71-second BS 2.0 recording exactly matched the earlier
in-memory position RMSE and coverage under the same input-span bounds. Synthetic tests also check
clock-anchor equivalence, receiver identity changes, invalid interpolation,
held-out fitting, explicit truncation handling, checkpoint/resume, and changed
build rejection. `--allow-truncated` is only for explicitly unqualified source
captures and records partial-stream status; it does not certify their integrity.
The alignment/error working set and coverage timestamps still scale with the
downsampled duration. SQLite files and replay recordings need disk space.

### Refreshing global-calibration scenes safely

The original collector wrote candidates directly into the next retained slot,
even when fewer than 11 measurements made the candidate unusable. Once the
32-scene ring filled, a deferred optimization could consume that sparse data.
Candidates are now validated separately before replacing retained data.

An initial implementation also preserved the old slot when a complete candidate
added no new coverage. The entire 9839-second three-station BS 2.0 replay showed
that this discarded a useful freshness update: DF24384D reference disagreement
rose from 487.6 to 821.5 mm, and F39A0C97 from 391.3 to 617.8 mm. Both streams
contain no MENU events, and the baseline/frozen candidate differ in the scene
collector plugin, so this regression cannot be dismissed as a CLI reset effect.
The first collector candidate is not a demonstrated tracking improvement.

The revised collector protects sparse candidates while allowing a complete
candidate with a valid current pose to refresh a full ring. A redundant refresh
does not claim new coverage or advance the ring; it supplies current complete
data to a pending solve. This retains the previous freshness behavior without
letting insufficient data erase an accepted scene. Synthetic rejection,
refresh, replacement and ownership checks pass normally and under ASan/UBSan.
A separate full BS 2.0 replay evaluates this revision; the original entire-corpus
baseline retains its frozen pre-change hashes.

The disk comparator can reuse an independently indexed reference database through
its Python API. It attaches that database read-only, so candidate comparisons
share exactly the same reference validity samples. Standard-library JSON is the
default parser; an optional installed `orjson` accelerates indexing and is named
in the metadata. The 71-second BS 2.0 comparison produces identical position and
angle RMSE with both parsers. No optional dependency is required by the tests.

`tools/lighthouse_corpus_report.py` assembles inventory, integrity, replay and
comparison checkpoints into a session ledger, object CSV, Markdown table and
standalone HTML report with session/device filters and per-minute error plots:

```sh
python3 tools/lighthouse_corpus_report.py /path/to/corpus-work \
  --output /path/to/corpus-work/report
```

It keeps pending work and unavailable comparisons visible, excludes incomplete
captures and integrity failures from pooled metrics, and labels the report in
progress until all sessions and reference indexes have been assessed. Pooled
RMSE is weighted by matched held-out samples, so long or mostly static recordings
can dominate it. Per-session/per-object results must accompany that aggregate.
Qualified reference coverage is pooled from matched/valid reference bins,
including zero-output comparisons, independently of the error sample count.
A corpus can therefore have low matched error and poor coverage.
`evaluation-settings.json` can pin the ledger to a specific replay binary,
keeping controlled variants out of the frozen baseline.

### Disconnects before deferred global calibration

Scenes were pruned when adding a new scene, but a pending solve could run after
an object had already been removed and freed. It could pass that stale object to
the optimizer, or read `objs[0]` after the last object disappeared. The solver now
prunes again under its scene lock immediately before solving, and skips an empty
scene or object list. A regression captures three scenes, removes one object,
checks that only the surviving object's scene reaches the poser, then removes
the last object and checks that no solve occurs. The previous code fails this
regression; the fix passes normally and with ASan/UBSan.

This lifecycle fix was made after freezing both the full-corpus baseline and the
separate long BS 2.0 scene-retention candidate. Those artifacts retain their
original hashes and do not validate this later fix.

### CLI button side effects during replay

The original `survive-cli` resets every lighthouse and tracked object on any MENU
button event, including release/touch events. A full replay of the 4h50m capture
showed a simultaneous reset at 11189.525 seconds, followed by no further libsurvive
poses despite later valid reference poses. The reset path is in the CLI callback;
it must not be attributed to the tracking library alone.

The CLI now requires `--cli-menu-reset` to enable this action, and only a button
press triggers it. Normal MENU input no longer discards calibration. A regression
checks the default behavior, explicit enablement, release/touch events and other
buttons, normally and under ASan/UBSan. Evaluation artifacts from the original CLI
remain identifiable by their frozen binary hashes. MENU events in the recorded
replays are audited, and affected captures require a separate controlled replay
with the new CLI and the same frozen core/plugins before drawing tracking-quality
conclusions. The later global-scene fixes are evaluated separately.

The complete current build passes 13 targeted CTest cases. Full 256-second BS 1.0
and 71-second BS 2.0 captures also replay twice with identical pose stream hashes
in that build. These short repeats include both global-scene fixes and the CLI
change; they do not extend the frozen whole-corpus build's validation scope.

The corpus callback total counts every parsed reference callback, including
rejected/missing-time rows and base-station callbacks. The pose index keeps
tracked-object validity samples; base-station poses are excluded from the
reference fit. This count is labelled callbacks read, not matched error samples.
