# Independent camera-only Kalibr check

This run used the same 16 independently selected still images as the OpenCV calibration, with Kalibr’s own tag detector and optimizer. All views are available to its fit, so its training residual is not a new held-out score.

**The target size 1.0 displayed by Kalibr is one normalized tag unit, NOT a measured metre.** This run checks scale-invariant camera intrinsics only. Do not use the unit-scale target for camera/IMU calibration, metric target poses, or vehicle offsets. The subsequent ruler measurements agree at 133 x 133 mm. A separate measured-scale rerun is preserved in ../kalibr-camera-static16-measured/.

The reproducible command and run log are in `evidence/kalibr-camera-static16-run2.log` and project tool history; input is `data/calibration-static16.bag`, selection is `data/calibration-static16.selection.json`. Kalibr source/patch/dependency pins are recorded in the project. The first fit succeeded but report plotting failed; outputs from that attempt are preserved in `evidence/kalibr-static16-first-fit/`.
