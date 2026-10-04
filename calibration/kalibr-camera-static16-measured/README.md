# Camera-only Kalibr cross-check with measured target

The user measured the displayed six-tag extent as 133 x 133 mm. Tag width is 17.733333 mm with a 0.3 gap/tag ratio. Ruler and edge placement uncertainty is not quantified. The fixed display zoom and square aspect ratio must be preserved.

This run uses the same 16 selected still images and all are fitted. It corroborates the accepted OpenCV intrinsics, but is not a new held-out test and supplies no IMU extrinsics or time offset. Estimated target poses shown in the PDF are camera calibration observations, not a VIO trajectory. Sparse extreme-corner coverage and occasional ~1.9 px residuals remain.

Reproduce from the project directory:

```bash
source tools/kalibr_env.sh
python3 tools/static_camera_bag.py calibration/intrinsics-static-apriltag4/observations.json data/NEW_STATIC16.bag
build/kalibr-ws/devel/lib/kalibr/kalibr_calibrate_cameras --bag data/NEW_STATIC16.bag --topics /cam0/image_raw --models pinhole-radtan --target calibration/target/target-screen-MEASURED.yaml --dont-show-report --no-shuffle --mi-tol -1
```

Run evidence: `evidence/kalibr-camera-static16-measured.log`. The earlier normalized-unit run is retained separately.
