# IMU reporting cadence and static stability (2026-10-24)

Commit `b13c3b3` fixes the reporting-rate issue described in
`~/WiVRn/libsurvive-jitter-handoff-20261024.txt`. A 250 Hz IMU with a measured
3.989333 ms period previously lost every other output to the strict nominal
4 ms minimum. Subtracting floating-point seconds also rejected exact-period
samples at some sensor-clock epochs.

The default negative `min-report-time` now permits 1% period variation around
the nominal IMU frequency. The attached configuration value stays negative, so
it remains distinguishable from an explicit limit. Nonnegative values retain
the requested limit, quantized to the nearest sensor tick, without this 1%
tolerance. Integer tick differences control reporting and its variance statistics.
Non-increasing timestamps are suppressed, and intervening optical updates remain
rate-limited. This does not add smoothing or change the zero-velocity update.

The supplied 47.929-second raw recording was replayed through the native library
with a fresh configuration and serialized posers/GSS. Output in its 20–45 s window:

| Device | Previous poses / Hz | Corrected poses / Hz |
| --- | --- | --- |
| Right Index, WM1 | 5,918 / 236.72 | 6,237 / 249.48 |
| Left Index, WM2 | 3,130 / 125.20 | 6,260 / 250.40 |
| Tracker, WM0 | 4,503 / 180.12 | 6,226 / 249.04 |

The old replay counts match the original live recording. The corrected counts
follow the observed IMU cadence. These rates do not measure motion-to-photon
latency or prove improved static jitter.

The supplied live record has right/left position RMS 0.0308/0.1201 mm and
orientation RMS 0.0297/0.1359 degrees. Its fixed 100 ms diagnostic extrapolation
barely increases the static variation. Recomputing that analysis reproduces the
handoff's values. However, fresh serial replay has a different filter/calibration
trajectory, with approximately 2.98 mm position RMS. Its baseline/candidate
position RMS differs by less than 0.000002 mm; the left orientation RMS changes
from 0.10928 to 0.10935 degrees. Neither is a live before/after jitter measurement.
A new live static and moving recording is needed to assess that effect.

Four complete BS 1.0/2.0 captures were also replayed. Each produced the same
pose-stream hash on three independent executions, including the final build.
All output poses are finite. Their held-out 50 Hz coverage is unchanged; 5 Hz
pose-disagreement RMSE changes are small and mixed (up to about 1.3 mm and
0.13 degrees in this selection). The previous 73-capture corpus assessment does
not yet include the cadence change.

The old native library fails the 3.989333 ms regression (500 outputs from 1,000
samples); the corrected library emits all 1,000. Five reporting cases also cover
4.000/4.005333 ms periods, crossing the 32-bit epoch, day/large epochs, explicit
limits, optical interleaving, faster events and sub-ULP tick differences. The
14 relevant CTest tests and ASan/UBSan reporting checks pass.

[Detailed metrics and source/build fingerprints](lighthouse-cadence-results.json)
retain original live analysis, replay RMS and fixed-prediction diagnostics,
per-device coverage/error distributions, commands and repeat hashes. Source
recordings remain untouched. This is a reporting-frequency fix; large tracking
outliers and slow static variation remain separate investigation targets.

A subsequent provenance check found a retained build-directory RUNPATH in the
copied CLI. All four published streams have now been reproduced with an explicit
frozen-library binding verified by `ldd`; proof is included in JSON. Adjacent-file
hashes alone should not be used to infer the loaded shared library.
