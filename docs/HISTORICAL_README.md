# Raspberry Pi visual-inertial odometry

**Work in progress.** OpenVINS runs on a reference dataset. The IMX219/LSM6DSOX assembly has buffered raw acquisition, and accepted camera-only intrinsics, and fitted provisional joint calibration. Physical live attempts have run but diverged; there is no validated physical VIO result yet. Pixhawk integration has not been performed. See [STATUS.md](STATUS.md) and the separate [acceptance gates](docs/ACCEPTANCE.md).

## Desktop start

**2026-09-28: the default desktop shortcut now opens native RViz controls.**
`OpenVINS Live (RViz)` and `VIO Demo (RViz)` show the tracked camera image,
green trajectory, orange MSCKF points and red SLAM points from the real estimator,
matching the requested tutorial's 48:00 layout (one camera on this rig).
Run `bash tools/start_native_demo.sh`; click **Start live (5 min)**, rest until
**READY**, then move slowly. **Stop and save** ends acquisition normally.
The two physical moving attempts on September 28 diverged and were rejected;
this display is working, but usable physical VIO is still not demonstrated.
See [native demo results and setup](docs/NATIVE_RVIZ_20260928.md).

The older browser controller described below remains available explicitly.

Open **VIO Demo** on the Pi Connect desktop. The manual app opens idle; **Check camera** records for 30 seconds and **Start VIO** starts a three-minute development run. Use **Stop & save** before repositioning or restarting. Follow the app's stationary-start instruction before moving. [Morning instructions](docs/MORNING.md) cover display, saved logs, recovery and limitations. The desktop workflow has been compiled but not exercised through a physical run or a new test suite, at the user's request.

```bash
python3 tools/install_desktop.py
bash tools/start_demo.sh
# To close the manually launched app:
python3 tools/stop_demo.py
```

The app uses localhost with a per-process control token, opens no controller connection, and creates a new data folder for each capture. It does not install an automatic boot service. Its sources are `tools/demo_app.py`, `tools/session_view.py` and `ui/demo.html`.

Bench identification/parameter-backup software is prepared in an isolated pinned environment:

```bash
bash tools/setup_bench.sh
```

See [Pixhawk preparation](docs/PIXHAWK_BENCH.md) before using it. Identification and backup remain unperformed on hardware. The VIO transmitter and firmware-specific settings require identification of the actual controller and remain unfinished.

## Environment and reproducible builds

Developed directly on Raspberry Pi 5, Debian 13 aarch64, Python 3.13.5. The existing Picamera2/libcamera installation is retained. Native Debian dependencies are extracted into `deps/`; there are no Ubuntu packages or system package replacement scripts. Exact package versions, archive SHA256 values and source commits are in `config/*lock.json`. Installed system runtime library links are listed in `evidence/system-library-links.json`; the machine baseline is in `evidence/baseline/`.

From `/home/pluto/vio-project`:

```bash
/usr/bin/python3 tools/fetch_debian_deps.py --profile openvins
/usr/bin/python3 tools/fetch_debian_deps.py --profile kalibr
/usr/bin/python3 tools/fetch_debian_deps.py --profile kalibr-gui
/usr/bin/python3 tools/link_system_libraries.py
git clone https://github.com/rpng/open_vins.git vendor/open_vins
git -C vendor/open_vins checkout --detach 69488123ed9362dd44b6f28e7f4680abbff1442b
VIO_BUILD_JOBS=1 bash tools/build_openvins.sh
VIO_BUILD_JOBS=1 bash tools/build_kalibr.sh
```

Clone only on a fresh checkout; existing sources are preserved. Dependency download scripts reuse verified cached archives. Their locks rely on Debian retaining those exact packages; retain `data/debs/` for offline rebuilds. Kalibr source is checked against its pinned archive SHA256, then the recorded compatibility patches are applied. Kalibr's Debian 13/Python 3.13 port now compiles and passes binding/numerical smoke checks; actual calibration runs require their own residual review. See [port notes](docs/KALIBR_PORT.md). Build success alone does not validate calibration math or bindings. Do not run concurrent builds in the same build directory.

Use `source tools/env.sh` in Bash for the local C++/ROS Python dependencies. Use `/home/pluto/vio-env/bin/python` for physical camera/IMU recording. No managed service is installed.

## Acquisition and safe stopping

The camera and IMU must remain rigidly attached. Candidate image geometry is 820x616 from the wide 1640x1232 sensor mode, full sensor crop, 20 Hz, 8 ms requested exposure, gain 4. The recorder additionally turns denoising off and saves actual camera configuration and per-frame metadata. The exact configuration is now preserved with accepted camera-only intrinsics in `calibration/camera-820x616-v1/`; the complete VIO calibration remains unfinished. Camera Module 2 has fixed focus, not autofocus.

```bash
/home/pluto/vio-env/bin/python tools/camera_probe.py --mode wide --exposure-us 8000 --gain 4
/home/pluto/vio-env/bin/python tools/record.py --seconds 30
# Guided capture: hard maximum ten minutes; normal early finish by the command below.
/home/pluto/vio-env/bin/python tools/record.py --seconds 600 --finish-on-usr1 --output data/calibration-views
/usr/bin/python3 tools/finish_capture.py data/calibration-views/SESSION
# Analyze only after session.json reaches completed.
source tools/env.sh
/usr/bin/python3 tools/analyse_recording.py data/calibration-views/SESSION
# Stationary noise capture: rest the assembly first. No camera images are saved.
/home/pluto/vio-env/bin/python tools/record.py --imu-only --temperature --seconds 1800 --output data/noise-recordings
```

Each capture creates a new directory, containing raw FIFO words/status/receipt brackets, PGM images, camera metadata and session information. Original recordings are never overwritten. Normal duration expiry and guided finish preserve an IMU tail. Ctrl-C/SIGTERM stops and preserves available data but marks the run interrupted. The finish tool checks boot identity and process command before signalling. Inspect final `session.json` and `analysis/report.json`, not just console output.

Acceleration retains gravity in m/s², angular velocity is rad/s, both use sensor native XYZ. Measurement time and host receipt time are separate. Camera `SensorTimestamp` is unchanged; it is **not assumed to be exposure midpoint**. IMU hardware ticks are mapped to the host clock from recorded anchors; this does not establish camera/IMU time offset. See [TIMING.md](docs/TIMING.md). The camera's approximately 12 ms row readout is not modeled by OpenVINS.

## Calibration workflow

`calibration/target/aprilgrid-a4.pdf` contains the 6x6 AprilGrid. For a physical print, use actual size and measure the resulting tag width. The current flat-screen display was measured by the user as 133 x 133 mm across six tags, giving 17.733333 mm tags; see `target-screen-MEASURED.yaml` and the saved measurement provenance. Measurement uncertainty is not independently quantified. `target-NOMINAL.yaml` is not a measured target definition. Keep screen zoom/aspect ratio unchanged while collecting views. Camera-only intrinsics can use abstract grid units; metric camera-to-IMU calibration cannot use those units as metres.

```bash
source tools/env.sh
/usr/bin/python3 tools/calibrate_intrinsics.py data/calibration-views/SESSION --output calibration/NEW_ATTEMPT
/usr/bin/python3 tools/to_kalibr_bag.py data/calibration-views/SESSION data/NEW_CAPTURE.bag
/usr/bin/python3 tools/verify_calibration_bag.py data/NEW_CAPTURE.bag --output evidence/NEW_BAG_CHECK.json
```

For intrinsics, capture at least 20 different viewpoints with about five seconds still at each; distribute the grid through the image and vary tilt/distance. The tool rejects moving images before fitting, separates held-out viewpoints and reports coverage, residuals and uncertainty. Candidate YAML files are explicitly unaccepted until reviewed. The accepted camera-only result is `calibration/camera-820x616-v1/camera.yaml`, with `review.json` and the original configuration bytes. Its four held-out view RMS values are 0.20–0.59 px; this does not validate moving-image rolling-shutter behavior. Changing sensor mode, crop, output resolution, orientation or focus invalidates the corresponding intrinsics. Moving either sensor on the base invalidates the spatial calibration. New exposure or driver changes require timing reassessment.

Kalibr builds and its measured-scale camera-only cross-check agrees with the accepted focal lengths within 0.4%; report: `calibration/kalibr-camera-static16-measured/`. Joint calibration and hardware replay/live commands will be added after the noise, excitation and residual gates pass. No identity camera/IMU transform, borrowed noise values or guessed offset is supplied to simulate a completed system.

A provisional IMU noise report can be generated with `python3 tools/imu_noise.py SESSION --output calibration/NEW_NOISE_REPORT` after sourcing the environment. Review time-series and Allan plots for stationarity, colored noise and bias behavior. This tool does not create estimator noise YAML or invent random-walk parameters from a short recording. The 30-minute run passed raw continuity but contains disturbances; its complete-run noise fit is rejected. `tools/propose_noise_weights.py` separately creates explicitly experimental solver weights from upper Allan envelopes of contiguous quiet windows. Those surrogates are not identified sensor coefficients or accepted hardware calibration. The current exploratory joint fit requires residual/covariance and weight-sensitivity review. [Kalibr’s noise documentation](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) recommends a much longer stationary recording for full identification. `bash tools/check_kalibr.sh` runs binding and numerical smoke tests after compilation; these checks pass. Source `tools/kalibr_env.sh` before invoking Kalibr itself; it supplies package discovery and headless Matplotlib configuration.

## Reference replay and viewing

Reference data and configurations are exclusively for TUM VI; never apply them to this Pi's sensors. The official room4 archive has published MD5 `8e2ec2c35ee40a54c9aaa5bc2b3c9d8c` and recorded SHA256 in `data/tumvi-room4/source.json`.

```bash
source tools/env.sh
/usr/bin/python3 tools/prepare_reference.py data/dataset-room4_512_16.tar.partial data/dataset-room4_512_16.tar.md5 data/NEW_REFERENCE
build/adapter/vio_replay config/reference/tumvi-mono/estimator_config.yaml data/tumvi-room4/images-mono.csv data/tumvi-room4/imu.csv data/NEW_RUN 0
/usr/bin/python3 tools/evaluate_reference.py data/NEW_RUN data/tumvi-room4/dataset-room4_512_16/mav0/mocap0/data.csv
/usr/bin/python3 tools/plot_trajectory.py data/NEW_RUN --title 'OpenVINS TUM VI room4 monocular reference'
/usr/bin/python3 -m unittest discover -s tests -v
```

The `.partial` archive name is historical: the retained file's complete checksum was verified. Prepared CSV image paths are absolute; prepare again after moving the dataset. Replay output includes pose/velocity, Hamilton xyzw IMU-to-local quaternion, full error-state covariance and processing timings. The interactive `viewer.html` is standalone and includes time/playback and projection controls. PNG/PDF plots are also saved. `data/reference-room4-mono-run2/` contains the final-install mono result: 3.53 cm position RMS with one rigid alignment and no scale fitting. This is not a measurement of the Pi assembly's accuracy or live latency.


## Experimental hardware replay and stream connection

`tools/prepare_hardware_replay.py SESSION NEW_DIRECTORY` exports strictly checked Pi measurements to `images.csv` and `imu.csv` without inventing calibration. Two existing sessions have been exported to `data/hardware-replay-motion1/` and `data/hardware-replay-poses4/`. They are not yet accepted hardware VIO runs.

The ROS-free `vio_stream` and its protocol are documented in [STREAM.md](docs/STREAM.md). An official room4 paced reference run passes through this transport with 3.60 cm position RMS; this validates the reference/software path only. The causal IMU clock mapper matches the offline ten-minute mapping within 23.1 µs at p95, after an explicitly excluded startup period. This comparison does not determine physical camera/IMU offset.

The recorder now has an experimental `--estimator-config` option. It requires `review.json` beside the estimator configuration, with `accepted_for_hardware_estimation: true`, matching camera-configuration SHA256, exact IMU configured registers, a mounting identity, supporting-report hashes, and hashes for the estimator/IMU/camera files. Camera-to-IMU rotation, translation, time offset and positive noise parameters must be present. Camera-only and upstream reference configurations are rejected. No accepted navigation bundle exists yet. Separate provisional bundles require explicit `--experimental-calibration`, a matching manual-development review and a run of at most 300 seconds. Physical runs with that override have failed usable VIO validation. After a reviewed bundle exists, source `tools/env.sh`, then invoke the normal camera/IMU recorder with that configuration path. Its `vio/` subdirectory will contain estimator logs while raw measurements remain in the recording folder.

The stream stops on stale/missing input, invalid timestamps, FIFO faults and bounded-queue overflow. A downstream navigation consumer must also reject old output independently. Normal recording stop includes an IMU tail. New processes create new output directories; no hidden estimator reset or flight-controller transmission is implemented.

## Recovery and troubleshooting

- Camera busy: inspect running recorder/probe processes; use normal finish for an active guided capture. Never start two clients against the IMU. Its advisory lock prevents project tools from doing so.
- After interruption/reboot: preserve the session, check saved boot ID and process identity, run analysis for diagnosis and label the interruption. A stale `recording` status is not a completed run. Do not reuse previous PIDs or fill gaps with invented samples.
- Dropped image/FIFO overflow/queue error: reject that acquisition for calibration, preserve all raw evidence and fix the cause. Storage stalls previously caused failures; separate bounded writers and camera callbacks have passed shorter tests, and a ten-minute run now passing raw continuity checks.
- Gray/blurry image: inspect a fresh saved image under good light with a textured scene. Check scene, obstruction, fixed-focus adjustment and selected crop. Do not assume autofocus or that old gray previews prove a driver fault.
- Power: undervoltage/throttling occurred on earlier boots; a current clear flag is insufficient evidence for a full run. Retest performance after the planned supply replacement. Coordinate any reboot rather than initiating one unexpectedly.
- Build interruption: verify pinned source/archive and saved patches, then resume the same build with its script. Do not delete original captures or silently treat an interrupted build as installed.

Pixhawk 4 firmware, TELEM2 signal wiring and Pi 5 UART routing remain unverified. No parameters have been changed; `/dev/serial0` currently refers to the Pi 5 debug UART, not an assumed header UART. Bench integration will require actual firmware identification, verified wiring, parameter backup, frame/timestamp/covariance checks and stale-data tests. Arming, motors and autonomous flight are outside this task.

## Saved unattended work (2026-09-26)

The bounded IMU job at `data/noise-job-unattended-20260926/job.json` records for two hours and then runs raw checks and provisional noise analysis automatically. It survives a chat disconnect while the Pi stays powered, but cannot survive a reboot. It never transmits to the Pixhawk or accepts calibration automatically. Its single 15-second end-to-end smoke check passed. After confirming PID and boot identity from `job.json`, SIGTERM to that job PID stops recording through normal cleanup and marks it interrupted. Original measurements remain saved.

To start a new bounded job from an active shell (new output directory required):

```bash
/usr/bin/python3 tools/run_noise_job.py --seconds 7200 --warmup-s 300 --output data/NEW_NOISE_JOB
```

`calibration/joint-sensitivity-first-review/` archives three fitting-weight variants. `calibration/joint-independent-first-review/` compares two nonoverlapping motion intervals. Neither is an accepted live calibration. `calibration/rig-exploratory-offline-v1/` is restricted to exploratory offline replay: fitted rig geometry/timing, empirical provisional noise weights, and factory IMU scale conversion. Its review is explicitly false for live hardware use. The recorded first replay is `data/rig-exploratory-motion1-run1/`; consult its saved result before claiming initialization or useful VIO.

## Physical development display

For a saved session, open its actual images and estimated path on the Pi desktop:

```bash
source tools/env.sh
python3 tools/demo_viewer.py data/live-demo/SESSION --open
```

The displayed address is localhost on the **Pi**, accessible in its Pi Connect desktop browser. The viewer labels ended captures and stale output. It does not make the estimated trajectory accurate. Keep a single viewer running to conserve CPU; Ctrl-C stops a foreground viewer.

A bounded experimental live run (unvalidated calibration; no autopilot output):

```bash
source tools/env.sh
/home/pluto/vio-env/bin/python tools/record.py --seconds 180 --finish-on-usr1 \
  --estimator-config calibration/rig-manual-demo-v2/estimator_config.yaml \
  --experimental-calibration --output data/live-demo
# In a second terminal, use demo_viewer.py with the NEW folder printed by record.py.
# Normal early stop:
python3 tools/finish_capture.py data/live-demo/SESSION
```

Use bright room lighting, a textured view, and a completely resting rig for startup. Do not move until stationary initialization is observed in that session's estimator.log. v2 tightens the visual/accelerometer stationary gates after v1 accepted a moving startup. Its one targeted 40-second failed-recording replay correctly withheld initialization, but this does not demonstrate successful initialization or tracking. Do not loosen the gate to obtain a moving plot. The most recent dark view gave thousands of extra FAST detections after histogram equalization; count alone is not evidence of stable scene features.
