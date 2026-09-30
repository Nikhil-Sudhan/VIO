# Native OpenVINS display and physical results

The requested reference is [Patrick Geneva's tutorial at 48:00](https://www.youtube.com/watch?v=rBT5O5TEOV4&t=2880s).
The actual frame was downloaded and inspected in `evidence/tutorial-48min/48m00.png`.
It shows RViz with tracked stereo images, a green trajectory, orange MSCKF points
and red SLAM points. This Pi has one IMX219 camera, so its Image Tracks panel is monocular.

The native display now receives actual OpenVINS tracker images, IMU pose and
accepted landmark coordinates. It does not synthesize geometry or animate a
reference dataset. Capture is bounded to five minutes and uses a new folder.
Stale output and rejected trajectories are labeled. The default desktop entry
opens native controls; the older web viewer remains an explicit legacy option.

## Start and setup

On the existing Pi, open **OpenVINS Live (RViz)** or **VIO Demo (RViz)**.
Click **Start live (5 min)**, rest the complete assembly until **READY**, then
move slowly. **Stop and save** preserves raw measurements. Closing the controls
stops its recorder and display processes. No service is installed.

```bash
# Existing installation
bash tools/start_native_demo.sh
# Rebuild the local native viewer dependencies if needed
bash tools/setup_rviz.sh
python3 tools/install_desktop.py
```

ROS uses localhost port 11319. Dependencies are verified Debian packages extracted
into `deps/`, recorded in `config/rviz-deps.lock.json`. The small scoped preload
in `adapter/rviz_local_ogre.cpp` relocates RViz's compiled OGRE plugin directory;
it is used only by `run_rviz.sh`. Local ROS plugin links and fonts are prepared by
`setup_rviz.sh`. No sudo or system package installation was used.

## Provisional mounting and camera configuration

User confirmed IMU +X points through the camera lens and IMU +Z points upward
with the image upright. Thus, for camera coordinates right/down/forward:

```text
R_cam_imu = [ 0 -1  0
              0  0 -1
              1  0  0 ]
```

This is an axis-derived initial rotation, not measured joint calibration.
Translation is unknown and initialized to zero; the prior approximately 8 ms
timing fit is only an initial guess. Previously accepted intrinsics are reused
at unchanged geometry, with provisional empirical IMU noise weights. All
navigation-acceptance flags remain false. The original invalidated mounting
bundle was preserved.

`rig-axis-demo-20260928-room` increases exposure from 8 to 16 ms and changes global
histogram equalization to CLAHE. Camera mode, crop, resolution and gain are
unchanged. Increased exposure changes timing and motion blur; joint timing is
still unverified. Exact assumptions and hashes accompany each bundle and capture.

## Verification and limitations

- Native output transport check: 250 reference frames, 161 poses; snapshot pose
  agreed with the estimator CSV. This was a software check, not the displayed demo.
- Fixed a startup-status bug: upstream `initialized()` remains false until a
  normal visual update, even after successful stationary initialization and
  repeated zero-velocity updates. The adapter now separately exports
  `stationary_ready` from the initialization timestamp and current state time.
  It leaves the estimator and its gates unchanged.
- Regression on the saved 25-second physical stationary sequence: 500 frames,
  440 exported stationary poses, approximately 1.90 mm final position norm,
  maximum estimated speed 0.00875 m/s. This verifies stationary state export,
  not moving VIO accuracy. The original capture's adapter error is preserved.
- First physical moving run `20260928_205257_kvkxz8a7`: 3900 frames, raw acquisition
  checks passed, but its trajectory diverged to tens of metres with few visual
  updates. Rejected in `vio/display-review.json`.
- Second moving run `20260928_210213_bupx9z0y`: reached stationary-ready status,
  then again diverged during the requested sideways-and-back movement. 1921
  frames passed raw acquisition checks, but estimated displacement reached
  463.7 m, with MSCKF updates on only 9 of 1881 exported pose frames. Rejected
  and stopped; exposure/contrast changes did not resolve moving estimation.
- The camera/gyro comparison found insufficient reliable essential-matrix pairs
  for a defensible replacement rotation. It did not promote a calibration.

There is no successful physical VIO trajectory to claim from these attempts.
The user-requested native display is implemented; trustworthy joint calibration
and successful moving validation remain outstanding. No flight-controller work
was performed.
