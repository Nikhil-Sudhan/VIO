# Estimator repair experiments — rejected for live use

The native adapter was rebuilt with read-only triangulation rejection diagnostics. The diagnostic reconstructs the upstream linear triangulation system on copied feature tracks; it does not modify tracking, state, covariance, acceptance thresholds, or vendor sources. Rejection counts can overlap. `VIO_DIAGNOSTICS_EVERY` controls diagnostic output cadence and must be positive.

At the start of the daylight motion, all 80 eligible tracks at one frame and all 96 at the next reconstructed behind the camera. After the short movement, the remaining observations could not determine depth reliably. This is evidence of inconsistent predicted motion and image geometry; it does not identify a unique cause.

Two concrete alternatives were evaluated on the preserved original-mount recording:

- Upstream dynamic visual/inertial initialization, with gyro bias measured from the resting prefix, initialized but subsequently diverged to kilometre scale. Its configuration was not promoted.
- Gyro-based rolling-shutter image correction preserved the original timestamps and used an explicitly assumed positive 12.013 ms row-readout interval. The image-level target residual improved only slightly, while VIO drift worsened to about 166 m. It was rejected. The Python prototype also costs about 307 ms per frame, unsuitable for 20 Hz live operation.

The readout duration comes from the earlier driver-control timing calculation in [TIMING.md](TIMING.md). Its sign and optical exposure phase are not established. The prototype uses the old fitted effective camera time as its reference; it does not claim SensorTimestamp is exposure midpoint. It corrects rotation only, not translation or exposure blur. A union mask excludes remap borders. Kalibr separately provides [rolling-shutter calibration](https://github.com/ethz-asl/kalibr); this prototype is not an implementation of that calibration and was not accepted as measured shutter calibration.

Three analytic software checks passed: zero rotation preserves coordinates, constant yaw gives the expected opposite top/bottom displacement, and missing bracketing IMU data is rejected. These tests do not establish sensor accuracy or physical VIO.

## Reproduction from preserved inputs

```bash
source tools/env.sh
cmake --build build/adapter --target vio_replay -j1
VIO_DIAGNOSTICS=1 VIO_DIAGNOSTICS_EVERY=1 build/adapter/vio_replay \
  calibration/rig-manual-demo-v2/estimator_config.yaml \
  data/daylight-replay-input/motion-window.csv data/daylight-replay-input/imu.csv \
  data/NEW_REJECTION_AUDIT 400

python3 tools/deskew_replay.py \
  data/hardware-demo-poses4-window/images.csv data/hardware-replay-poses4/imu.csv \
  calibration/rig-exploratory-online-refinement-v4/imucam.yaml \
  data/NEW_DESKEW --readout-ms 12.013 \
  --gyro-bias .013199480244984607 -.009611245858059649 .0018836508729826644

python3 tools/compare_deskew_target.py \
  data/hardware-demo-poses4-window/images.csv data/NEW_DESKEW/images.csv \
  calibration/rig-exploratory-online-refinement-v4/imucam.yaml \
  data/NEW_DESKEW/target-comparison.json --stride 10
```

Every output directory must be new. The replay adapter is an offline diagnostic and does not approve the supplied calibration for live use. To reproduce the rejected corrected replay, copy the v4 estimator/IMU/camera YAML files into a new experiment directory, change only `use_mask` to true, and add `mask0` pointing **relative to that directory** to `NEW_DESKEW/invalid-mask.png`. Replay `NEW_DESKEW/images.csv` with the unchanged original IMU CSV. The original executed experiment is preserved at `data/deskew-poses4-config/`; the dynamic-start changes and their bias provenance are recorded in `calibration/estimator-repair-20260927/dynamic-start-experiment.json`.

Complete result summaries, hashes, timing, and paths are in [the repair report](../calibration/estimator-repair-20260927/report.json). The latest fixed-assembly capture contains mostly stationary data and does not establish a new joint calibration. No correction was enabled in the live path; no trustworthy physical trajectory or Pixhawk integration result exists.
