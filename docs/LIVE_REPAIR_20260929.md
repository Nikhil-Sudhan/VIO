# Live OpenVINS repair — 29 September 2026

Hardware: Raspberry Pi 5, IMX219 CSI camera at 820×616 / 20 Hz, LSM6DSOX I2C IMU at approximately 204 Hz. User confirmed +X forward, +Y left, +Z up and a rigid common mounting. Current AprilGrid print scale remains assumed from the A4 PDF, not measured.

## Confirmed faults and changes

- Default feature tracking provided very few successful visual corrections during the printed-grid motion recordings. Added an opt-in AprilTag 36h11, two-border-bit corner tracker alongside KLT. It supplies only measured 2D image corners to the actual OpenVINS camera/IMU filter. It does not inject a target pose, known 3D grid, known scale, or synthetic trajectory. Only one copy of this grid may be visible because the printed sheets repeat tag IDs.
- Long stationary feature histories made the first movement block the filter for 3.089 seconds, after which the existing one-second input freshness check stopped the live run. Replaced repeated vector erases with stable compaction. All 144 comparison cases pass; a 6000-observation benchmark improved from 348.671 ms to 0.728399 ms. The 781-pose movement replay is byte-identical before and after the cleanup patch.
- Historical-track rendering also accumulated delay while stationary. The native display now draws current measured feature points and exports actual estimated AprilGrid landmarks separately. Complete processing-cycle timing is saved in cycles.csv. Original sensor samples, timestamps and stale-input checks remain unchanged.
- The adapters previously forced OpenCV to one worker regardless of configuration. They now honor the reviewed setting. On the same 20 frames, four workers reduced mean tag detection from 21.51 ms to 10.46 ms. The complete four-worker movement replay has a byte-identical pose CSV to the single-worker result.
- RViz now uses 10 cm grid cells and a closer default view. Status text is sized for that view. Live and stopped images remain explicitly labelled.

## Evidence and limits

The latest provisional calibration plus tag corners reduced maximum estimated position norm in the straight-slide replay from 7.72 m (KLT only with the same calibration) to 0.375 m, with up to 67 landmarks. This is improvement, not proof of metric accuracy. A same-image target diagnostic (assumed print scale, not independent ground truth) still found a maximum 0.343 m disagreement. An earlier independent translation replay also terminated on a negative covariance diagonal. Walking/navigation accuracy is not accepted.

Lowering physical image rate to 10 Hz was not promoted: a sampled-input replay diverged to 20.96 m. The experimental visual veto for zero-velocity updates was also rejected: the tag-assisted slide replay diverged to 367.53 m. Both remain disabled in the live configuration.

Configuration currently under live test: calibration/lights-return-20260929/manual-grid-vio-fast/estimator_config.yaml. All original raw recordings and numerical results are preserved. Regenerable ROS bag exports were removed to make recording space; exact source paths and removed hardlinks are listed in calibration/lights-return-20260929/derived-export-cleanup.json.

## Current live validation

Pending completion of user motion test in data/native-live/20260929_230224_qcnkez0c. Do not claim the moving-drift problem fixed from stationary stability or replay improvements alone.


## Saved at end of session, 30 September 2026

User turned off room lighting and explicitly deferred further physical testing until tomorrow. No sensor acquisition is active. User requested a detailed 3D enclosure prompt; deliverable is `output/enclosure/VIO_Enclosure_CAD_Prompt.md`. Any installation in the new case invalidates the previous camera/IMU mounting transform; refit after final assembly.

### Last live run

`data/native-live/20260930_000816_jrecsjcu` completed its 120 seconds normally: 2401 camera frames, estimator exit 0, maximum input event-queue depth 7, maximum event wait 76.991 ms. Camera-to-output latency median 86.254 ms, p95 102.740 ms, max 154.981 ms. This run remained in stationary initialization/ZUPT and ended with the room dark; it is NOT a successful moving VIO validation. Maximum estimated position norm 0.00560 m does not prove moving accuracy.

For this urgent run, the full capture used `/dev/shm/pi-vio-quick-test/20260930_000816_jrecsjcu`, mirrored to the SD card by `tools/persist_live_ram.py`. The disk backup marker confirms completion, and current-session.txt points to the persistent disk directory. Original live_transport text said raw sensors stayed on disk; that statement did not apply to this temporary RAM capture. Raw frames and all final outputs are now saved on disk. The source description has been corrected to identify the recorder destination explicitly.

### Runtime changes to validate with movement

Native demo now uses the grid-assisted fast manual bundle by default. Added opt-in RAM storage for derived estimator files (`--ram-estimator-output`); it preserves the output in the session after normal or failed estimator exit. Four live-bridge tests pass, including preservation of output and failure reason after a simulated estimator failure. Added queue-wait/write-duration metrics and C++ `intake.csv` to separate transport, IMU and camera processing delays. The last earlier moving run `20260929_232136_0zyrndg4` stopped on >1 s stale input despite passing raw integrity and causal clock checks; SD writing/backpressure remains a hypothesis. The new static RAM run does not establish that this moving failure is fixed.

Clock fitter uses four recent training anchors after the 1.5 s warmup and 50 ms requested anchor cadence. Original prediction/stale/monotonicity checks remain enforced.

### New calibration measurements and candidates

Both raw captures completed without integrity faults:
- `data/current-mount-calibration/20260929_234048__wzje0kl`: 4782 images, broad translation and yaw, weak tilt/roll.
- `data/current-mount-calibration/20260929_235643_xt2pev7y`: 3521 images, substantial tilt and yaw, limited roll.

Both offline Kalibr jobs completed and saved reports, transformations and poses:
- `calibration/lights-return-20260929/all-axis-2340-fit`: fitted time offset -0.000316740 s; mean reprojection error 0.270171 px, gyro residual 0.00194712 rad/s, acceleration residual 0.0251650 m/s^2.
- `calibration/lights-return-20260929/tilt-roll-fit`: fitted time offset -0.000363351 s; mean reprojection error 0.218894 px, gyro residual 0.00245564 rad/s, acceleration residual 0.0267593 m/s^2.

These differ from the previous +0.0232079 s time-offset initialization. The two new spatial transforms also differ; neither has been promoted into the live calibration or validated on held-out moving recordings. Noise weights remain 10x exploratory weights, not accepted sensor-noise identification. Printed target size remains nominal A4/20 mm, not measured.

### Rejected offline variants

A one-corner-per-tag configuration with a tag-based stationary veto looked better on the most recent selected movement but failed the older straight-slide recording (179 m estimated excursion). Four corners with that veto failed the straight slide (~299 m); a separate bounded feature history did not improve it. Removing the 10x noise inflation alone failed the same slide (~19.3 m). These variants remain offline experiments and are disabled in the native default. Two `*-tag-history-20260929` runs started before the revised binary build finished and were explicitly interrupted and marked INVALID_RUN.json; use only the v2 output when discussing that experiment.

### Resume work

1. Preserve current mounting until a baseline test, or recalibrate after installation in the new rigid case.
2. Review new candidate transform repeatability and test each against preserved independent recordings before selecting one. User deferred physical tests; coordinate fresh light/readiness before opening sensors again.
3. Verify actual native RViz is foreground/visible before requesting motion. In the urgent run a terminal covered RViz in the screenshot.
4. Test moving input latency with the new storage/transport diagnostics. Do not relax stale-input checks or call static stability a moving accuracy result.
5. Only after valid moving output, evaluate a longer run. The controller remains a bounded manual-development demo and is not navigation-validated.

Regenerable completed Kalibr bag exports and some compiled object caches were removed to free space, preserving all raw captures, source files, installed libraries, calibration results and export provenance. Cleanup manifests are saved in the same calibration directory.

## Resumed physical and offline review — 30 September 2026 morning

User confirmed unchanged mounting, lights on and readiness, then granted full access. Fresh three-second check `data/resume-camera-20260930/20260930_085855_amjvproo` passed raw checks; all 36 printed tags were visible. No remount or accepted calibration was inferred. An initial invocation omitted the project OpenCV environment and exited before opening hardware; corrected invocation used `tools/env.sh`.

Completed 12 offline replays: yesterday's default, each of the two new fits, and three controlled all-axis variants, each on the same two earlier recording selections. Input hashes match within each test; final target comparisons use identical timestamps (176 slide pairs, 33 later-motion pairs), one initial rigid pose alignment and no fitted scale. Baseline target maximum disagreement reproduces yesterday's saved value to within 1e-12 m. Source/configuration/binary hashes, commands, raw outputs and failures are retained under `evidence/resume-20260930`. The interrupted fixed-calibration replay is separately marked `INVALID_RUN.json` and excluded; its replacement completed.

The new fits differ by 3.074 degrees in rotation, 28.264 mm in translation and 0.04661 ms in fitted offset. On the later movement, maximum estimated position norm improves from 2.94666 m baseline to 0.12594 m with all-axis fitting; tilt/roll fitting diverges to 21.09846 m. This improvement does not generalize: the all-axis older-slide result reaches 0.58084 m, with 0.55055 m maximum target disagreement over the shared comparison interval. Fixed extrinsic/time calibration does not resolve both tests. One corner per tag fails at 14.81 m / 3.18 m; retaining all tag corners while removing ordinary persistent SLAM landmarks fails the later test at 29.13 m. These variants remain offline and are not promoted. Default native configuration is unchanged.

A separate explicitly provisional live bundle, `calibration/resume-20260930/manual-all-axis`, was used for one bounded live test. Its guard passed with navigation acceptance false. The 120-second attempt was rejected before sensor access by disk-space preflight; a 90-second bound fit available storage. RViz foreground and current image were verified before asking for the 5–10 cm out-and-back. User replied Done. No other estimator or capture was running concurrently.

Live session `data/native-live/20260930_090103_o5nzvzwt` **FAILED** after roughly 58 seconds of recorded camera data: 1166 saved images, 12308 derived IMU groups, 1145 processed camera frames, 1121 poses including stationary estimates. Actual visual updates began at estimator time 37.0007 s; 57 pose frames have MSCKF features. Maximum estimated position norm was 0.13591 m. Same-image target diagnostic indicates 0.15854 m maximum excursion, with position disagreement median/p95/max 0.00486/0.04273/0.05516 m across 167 paired observations; after visual initialization these are 0.02336/0.04956/0.05516 m. These are NOT independent accuracy measurements: they reuse the camera images, assume nominal 20 mm printed tags and use initial extrinsics for the lever arm while the filter refines them. The captured target endpoint remains about 0.14986 m from its initial position; the full requested return cannot be assumed captured just because the user later reported completion.

The original one-second freshness guard stopped the run. Camera-to-output median/p95/p99/max latency was 53.625/84.794/787.531/1058.012 ms. Across the 414 frames after visual initialization, latency p95 was 606.368 ms. Maximum transport event wait was 955.879 ms; queue high-water 89. Filter processing peaked at 155.999 ms, with sustained >50 ms work near the stop. Derived estimator output and console were already in RAM and were preserved to disk afterward. Image/raw writer queue highs were both one, no missing images or FIFO overflow were found, and the only raw-analysis fault was abnormal session completion. Thus the full acquisition gate remains failed; recorded samples are retained for diagnostic export with the explicit estimator-failure exception. Causal clock maximum prediction residual was 489.383 us. A later idle health snapshot showed no throttling and 52.1 C; this is not continuous thermal evidence for the run.

Plots: `evidence/resume-20260930/live-result.png` and `.pdf`, plus `candidate-comparison.png` and `.pdf`. Numerical results: `live-review.json`, `review.json`, `comparison-table.json`, and `verification.json`. New analysis scripts compile; common observation timestamps and provenance checks pass. No algorithm binary, freshness limit, noise model or physical camera configuration changed this morning. RViz remains available with the stopped run and explicit failure review; no sensor acquisition is active.

Next technical work: profile the cost of moving filter updates while retaining the visual information needed for consistent motion estimates; resolve the backlog and the older-slide inconsistency before claiming accurate VIO or requesting a longer validation run. No calibration has been accepted for navigation and no Pixhawk changes were made.
