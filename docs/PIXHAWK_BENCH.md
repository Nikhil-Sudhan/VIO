# Pixhawk bench preparation

No firmware or TELEM2 signal wiring has been verified. No navigation message or controller parameter profile is selected yet. Physical VIO remains unvalidated. This document prepares the next stage; it is not an integration pass.

`tools/pixhawk_inspect.py` is prepared for a bounded identification/backup session after the actual connection is verified. It never sends VIO, sets parameters, changes modes, arms, or operates motors. Without flags it listens only. `--request-info` sends one standard request for AUTOPILOT_VERSION after an autopilot heartbeat; `--backup-parameters` requests the parameter list. These information requests are transmissions, not passive reads. No serial connection or information request was performed during overnight packaging.

Prefer identifying the controller over its verified USB connection before preparing the TELEM2 cable. Use separate Pi power. Opening a serial device can affect control lines; connect only the identified controller on the bench. The script rejects the current Pi 5 debug UART `ttyAMA10`. `/dev/serial0` currently points there, so it must not be assumed to be the 40-pin header UART.

After connection and device identity are verified:

```bash
cd /home/pluto/vio-project
bench-env/bin/python tools/pixhawk_inspect.py \
  --device /dev/serial/by-id/VERIFIED_CONTROLLER --baud 115200 \
  --seconds 45 --request-info --backup-parameters --output data/NEW_PIXHAWK_INSPECTION
```

The tool preserves incoming binary MAVLink messages in `received.tlog` and identity/parameter metadata in `inspection.json`. A missing firmware response or incomplete parameter list stays incomplete. An unknown autopilot or version is not replaced by a default. No automatic retry, parameter restore, or protocol fallback is implemented. Parameters retain raw float values plus MAV_PARAM_TYPE; integer encodings must be interpreted using the detected firmware's protocol, not guessed.

## Work requiring that evidence

1. Record exact firmware/version, hardware ID and USB/TELEM device identity. Select the matching version of its documentation.
2. Verify Pixhawk 4 connector orientation/pinout against manufacturer information and continuity of the actual cable. Verify Pi 5 header UART routing before configuring it. Back up relevant Pi configuration before any change; coordinate any required reboot.
3. Review the preserved complete parameter backup before changes. Select the firmware-supported external-navigation interface, rate, frames, covariance interpretation, estimator/reset/quality semantics and clock/time synchronization.
4. Measure the IMU/camera/body mounting transform and offset. Transform the **full** relevant covariance using the correct Jacobian; do not merely reorder its diagonal. OpenVINS output uses the IMU origin, local gravity frame and Hamilton xyzw IMU-to-local quaternion. It is not automatically a MAVLink body/local frame or the controller's required quaternion order.
5. Require fresh, supported, validated physical estimates before transmission. Preserve restart/reset identity, stop on stale/missing data, and verify controller receipt and estimator use through telemetry and saved logs. No arming or motors.

The actual VIO transmitter and firmware-specific parameter profile are deliberately not declared built: choosing those before identifying the installed firmware would contradict the integration sequence. The inspection dependency environment and tool are prepared; they remain untested on a controller.

## Official references

- [MAVLink AUTOPILOT_VERSION](https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION), [MAV_CMD_REQUEST_MESSAGE](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE), [parameter protocol](https://mavlink.io/en/services/parameter.html).
- [PX4 external position estimation](https://docs.px4.io/main/en/ros/external_position_estimation): select the matching installed firmware version before deriving settings.
- [ArduPilot external navigation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html): use only if ArduPilot and its actual version are identified.
- [Raspberry Pi UART configuration](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts): Pi 5 debug UART/default alias differs from older Pi assumptions.

Dependency wheel filenames, hashes and versions are recorded in `config/bench-deps.lock.json` and `config/bench-requirements.txt`. This separate environment does not replace camera-stack packages.
