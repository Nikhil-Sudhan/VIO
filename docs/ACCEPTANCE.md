# Acceptance gates

These are initial engineering targets, not achieved results. Revise explicitly if measured hardware limits require it.

| Gate | Required evidence before pass |
|---|---|
| Software installation | Pinned sources/dependency versions and repeatable build; upstream reference dataset processed with finite pose/velocity output and ground-truth evaluation. A simulator or successful build alone is insufficient. |
| Image acquisition | Visually clear textured scene in stock and recorder captures; repeatable geometry/orientation/crop; at least 100 distributed trackable corners in a suitable static test scene; moving-test exposure initially <=10 ms with sufficient lighting. |
| Timing/acquisition | 10-minute simultaneous 20 Hz image / ~208 Hz IMU run with raw metadata retained, no unreported drops, overflow or reversal; bounded queues; distinguish sample time from receipt time. Quantify sensor clock mapping residuals and systematic uncertainty, targeting <1 ms residual without claiming this bounds internal sensor/filter delay. |
| Calibration | Intrinsics target coverage and held-out reprojection RMS initially <1 pixel; sufficient multiaxis excitation for spatial/time calibration; residual plots and estimated uncertainty reviewed; gravity and sensor units retained; noise estimate backed by stationary data and its duration. Files identify exact hardware, mode, mounting and provenance. |
| Hardware replay | Initialization succeeds on five deliberately acquired sequences; finite orientation/position/velocity; no hidden resets, lost samples or unbounded backlog; outputs tied to measured calibration. |
| Live performance | 10-minute run at configured rates; camera-to-pose latency p95 <150 ms, no sustained queue growth, no thermal throttling, memory <6 GiB and CPU headroom recorded. Stationary 60 s drift target <0.2 m and velocity magnitude p95 <0.05 m/s after initialization. |
| Handheld behavior | Document stationary, multiaxis rotation, measured translation, tracking occlusion/restart, and three measured >=10 m return-to-start routes. Endpoint error target <=0.3 m; path measurement is not independent 6-DoF ground truth and cannot establish full accuracy. |
| Pixhawk bench | Actual firmware/version and message interface confirmed; wiring verified; parameter backup; validated frames/offsets/time/covariance/reset/quality semantics; telemetry/log evidence of receipt, estimator use and stale-data rejection. No arming/motors/flight. |

Software tests, reference dataset results, physical sensor tests, and vehicle integration each retain separate pass/fail records. Later stages do not retroactively validate earlier assumptions.
