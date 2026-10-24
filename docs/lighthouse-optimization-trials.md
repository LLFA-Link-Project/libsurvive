# Full-duration optimization trials (2026-10-24)

The [machine-readable trial ledger](lighthouse-optimization-trials.json) retains
per-device support counts, coverage, error distributions, replay fingerprints,
and explicit rejected/invalid experiments. Valve outputs are reference estimates,
not ground truth. The completed [73-capture assessment](lighthouse-controlled-results.md)
is the corpus baseline; these trials do not replace it.

Hard-locking the existing reference lighthouse pose after 32 scenes improved
all five position/orientation RMSE pairs in the first 600 seconds of the longest
BS 2.0 capture. Full 9,838.988-second replay rejected the change:

| Device suffix | Position RMSE, baseline → candidate (mm) | Orientation RMSE (degrees) | Coverage (%) |
| --- | --- | --- | --- |
| 9338C3A6 | 28.55 → 27.64 | 3.21 → 2.96 | 99.818 → 99.818 |
| 9BB5412D | 658.45 → 792.11 | 25.92 → 27.30 | 99.721 → 99.525 |
| D8CCD0C5 | 311.83 → 313.04 | 4.92 → 4.90 | 99.863 → 99.863 |
| DF24384D | 487.58 → 503.65 | 15.84 → 17.65 | 99.747 → 99.767 |
| F39A0C97 | 391.32 → 610.33 | 13.61 → 15.17 | 99.797 → 98.763 |

Position medians improved, but tails and availability regressed. Sample counts
change with availability and are retained in JSON; these are not a paired-sample
causal estimate. Four short BS 1.0/2.0 captures also tested locking from startup,
which had mixed results. The 32-scene threshold did not activate in those four.
No reference-lock default or configuration switch was added.

Earlier `fixed-gauge`/`mature-gauge` experiments incorrectly called
`survive_optimizer_fix_camera`, which zeros camera parameters. They do not test
preserving an existing reference pose and are excluded from quality conclusions.
The corrected trials directly fixed parameter flags without changing the pose.

Rejecting all redundant stored scenes also regressed long BS 2.0 tracking.
Commit `2cdfabb` instead rejects sparse candidates while retaining complete
full-ring refreshes. Its full 7,035,157-pose stream is byte-identical to the frozen
baseline (pose-stream SHA-256 in JSON).

Explicit reset probes used the same BS 2.0 capture through 600 seconds and
injected `survive_reset_lighthouse_positions` near runtime 250 seconds. Original
output resumed after 18.57–18.64 seconds. Re-solving cached scenes reduced this
to 1.13–1.15 seconds, but can restore pre-movement calibration observations.
Discarding all old scenes took 18.54–21.50 seconds and changed the world frame;
the fixed pre-reset alignment therefore records a discontinuity, not pure tracking
accuracy. Neither prototype was adopted. Recovery figures mean output availability,
not verified accurate reacquisition or live processing latency. The exact reset
poll times differed slightly and are retained in JSON.

Independent repeated full replays of 15 finite-output captures produced identical
pose-stream hashes in all 15 cases. This is not whole-corpus repeatability proof.
The five BS 2.0 device head/IMU offsets also match the recorded reference transforms
within 1.71e-8 m and 5.4e-6 degrees after quaternion normalization. The large pose
tails are not explained by those configured reference-point offsets.
