# ROS-free streaming transport

`vio_stream` accepts one camera and SI IMU measurements over stdin. It saves the same pose, velocity and 15x15 covariance format as `vio_replay`. A new output directory identifies each process/reset epoch. Reference playback tests transport and estimator integration; it does not validate live Pi acquisition or camera/IMU calibration.

All fields are little endian. The stream starts with eight bytes `PIVIO001`. Every record has the 29-byte header `<BQQQI`: kind, measurement timestamp ns, host receipt timestamp ns, sequence, payload length. Timestamps use a common clock; they are not receipt times substituted for measurement time. Sequence numbers are consecutive within each sensor and can start at any value.

| Kind | Payload |
|---|---|
| 1, IMU | Six float64 values: gyro XYZ in rad/s, acceleration XYZ in m/s² including gravity. |
| 2, camera | Width and height as uint32, then exactly width×height bytes of grayscale pixels. |
| 3, normal end | Empty payload; all header fields except kind must be zero. Disconnect without END is an error. |

The adapter checks message size, finite IMU values, sensor sequence/time order, and calibrated image geometry. It retains at most eight pending camera frames and waits for an IMU sample at least 10 ms past the current camera-to-IMU corrected time. Excess queue growth, incomplete IMU tail, input disconnection and nonfinite state cause failure. In live mode, one second of input inactivity or data age also causes failure; downstream navigation consumers must independently reject stale outputs. Pose rows are written only for initialized states updated to the current camera time. These conditions alone do not prove tracking quality or accuracy.

Live `sensor_to_output_ms` measures output time minus the original camera timestamp, which is the driver's frame-start event rather than exposure midpoint. Receipt latency is logged separately. Reference mode emits NaN for both latency fields because old dataset times are not the Pi's current boot clock. A 30 ms simulated image delivery delay in `stream_reference.py` is a transport exercise, not a measured sensor delay.

```bash
source tools/env.sh
set -o pipefail
python3 tools/stream_reference.py data/tumvi-room4/images-mono.csv data/tumvi-room4/imu.csv |
  build/adapter/vio_stream config/reference/tumvi-mono/estimator_config.yaml data/NEW_STREAM_REFERENCE reference
python3 tools/evaluate_reference.py data/NEW_STREAM_REFERENCE data/tumvi-room4/dataset-room4_512_16/mav0/mocap0/data.csv
```

The physical live producer, causal sensor clock mapping validation and reviewed rig calibration are required before live hardware use. No physical VIO result is asserted by this transport implementation.
