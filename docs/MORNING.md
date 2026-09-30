# Morning start

Open Pi Connect's desktop and double-click **VIO Demo**. If the console was left open overnight, use that window. Nothing records merely because the app is open.

1. Rest the rigid camera/IMU base facing several detailed objects in daylight, about 1–3 metres away. Keep bright windows outside the camera view. Keep the mounting unchanged.
2. Click **Check camera**. It records for at most 30 seconds. Look for sharp, distinct objects across the image. A bright image or a corner count alone is not a tracking pass. Click **Stop & save** when you have seen it.
3. Click **Start VIO** and keep the rig completely still. This is a bounded three-minute development run using provisional calibration.
4. Wait for the console to say **Stationary initialization complete** and give a movement instruction. Then move the whole base sideways about 20 cm over three seconds, keep it pointed the same way, and set it down. If it never initializes, save that failure rather than moving on a timer.
5. Observe the estimated path and velocity. If the console says visual support is missing, output is stale, or motion is unreliable, stop. A moving plot is not proof of successful VIO. Use **Stop & save** before repositioning or restarting.

The current measured joint calibration and noise model remain provisional. Previous physical runs diverged. Tomorrow's lighting may improve image quality but does not resolve every possible timing/calibration/initialization issue. A usable physical result still has to be established.

## What is available

- Camera view plus XY/XZ/YZ trajectory projections.
- IMU position, local velocity, quaternion orientation and orientation axes.
- Initialization, missing visual support, stale-camera/pose and failure messages.
- Saved-run selection, full pose/velocity CSV download and current summary download.
- Temperature and available disk; resource logs include CPU counters, memory, process stats and throttling observations during a run.
- Raw IMU, images, timing, available pose/covariance logs, exact settings and acquisition-source snapshot preserved per recording.

All positions refer to the IMU origin. Orientation is a Hamilton quaternion in xyzw order, IMU-to-local active rotation. This is a local gravity-aligned frame with arbitrary yaw and position, not GPS or a compass heading. The on-screen MSCKF count is not a complete SLAM-health metric. No output is sent to the Pixhawk.

## Files and stopping

App recordings are under `/home/pluto/vio-project/data/demo-sessions/`. Each launch gets a new folder with `launch.json`, `capture.log`, `resources.jsonl`, `run-summary.json` after exit, and the recorder's timestamped subfolder. Raw data is never replaced on restart. Summaries do not automatically declare an accuracy pass.

The app uses the frozen 820×616 / full-sensor crop, 20 Hz, requested 8 ms / gain4 configuration. The night gain probe did not change it. If geometry, focus or mounting changes, review calibration before using the estimator. Exposure changes also require timing review.

Stop a capture with **Stop & save**. Closing the browser tab does not stop capture: its fixed duration still applies. Reopen **VIO Demo** to reconnect to the same app. The app is manually launched; it is not a boot service. After reboot, reopen the shortcut. Do not reuse old process IDs.

Terminal fallback:

```bash
cd /home/pluto/vio-project
bash tools/start_demo.sh
```

To finish a guided recording without the app:

```bash
python3 tools/finish_capture.py data/demo-sessions/RUN/SESSION
```

To close the idle app, use `python3 tools/stop_demo.py`. It verifies process and boot identity; an active capture is finished or marked interrupted if clean shutdown cannot complete. The UI address is saved in `data/demo-app.json`. A launcher failure is recorded in `data/demo-app-console.log`.

## Saved-data tools

These commands are prepared for later use; no new tests were run during the overnight packaging work.

```bash
source tools/env.sh
python3 tools/analyse_recording.py PATH_TO_SESSION
python3 tools/prepare_hardware_replay.py PATH_TO_SESSION data/NEW_REPLAY_INPUT
build/adapter/vio_replay calibration/rig-manual-demo-v2/estimator_config.yaml \
  data/NEW_REPLAY_INPUT/images.csv data/NEW_REPLAY_INPUT/imu.csv data/NEW_REPLAY_OUTPUT 0
python3 tools/plot_trajectory.py data/NEW_REPLAY_OUTPUT --title 'Physical rig — accuracy unvalidated'
```

Normal replay export deliberately rejects failed recordings. Preserve failed runs for diagnosis; do not relabel them completed. `README.md` contains the calibration/build commands and the historical reference-dataset results. `docs/ACCEPTANCE.md` separates software, acquisition, physical VIO and controller requirements.

## Current limits

No reliable physical trajectory, accepted complete noise model, complete motion/return-to-start checks, or verified Pixhawk integration exists yet. The new desktop workflow was compiled but not exercised through a sensor run or new test suite, as requested. The original accepted camera-only calibration and all earlier evidence remain available.
