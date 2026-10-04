# Handheld tracking health and local map recovery

`tracking_supervisor: true` enables the same supervisor in `vio_replay` and
`vio_stream`. This is a slow handheld application envelope, not independent
accuracy validation, calibration acceptance, or navigation certification.

The upstream initialized flag means that initialization and normal propagation
have occurred. It does not imply accepted visual constraints. Stored SLAM
landmarks can also remain present without successful corrections. The recorded
OpenVINS patch counts features only after completed MSCKF/SLAM EKF corrections.
It does not change rejection gates, covariance arithmetic, or feature selection.

A pose becomes publishable after two camera calls, at most 0.3 seconds apart,
each completing corrections from at least three features. Subsequent camera
corrections or accepted zero-velocity updates keep it constrained. A stationary
initialization alone is `ready`, not a usable 3D map. Two accumulated seconds of
unconstrained movement before depth lock, more than one second without either
constraint after lock, speed above 2 m/s, position standard deviation above 1 m,
or invalid state/covariance causes loss. These thresholds are application
limits; passing them cannot establish accuracy.

On loss, the current pose is invalid. On the next camera call the manager is
recreated with the same calibration and the last three seconds of original IMU
samples. Camera and IMU timestamps are unchanged. A fresh static initialization
and visual depth lock are required. Every recreation increments `reset_epoch`.
There is no relocalization, frame registration, or loop closure between epochs.
RViz clears old paths/points and stops publishing poses while invalid or stale.

Output evidence:

- `poses.csv` / `covariance.csv`: raw filter diagnostics, including invalid poses.
  Join by time to `tracking.csv` for validity and coordinate frame.
- `tracking.csv`: every processed image, health state, actual correction count,
  speed/uncertainty, epoch and loss reason.
- `epochs.csv`: reset and loss events.
- `valid-poses.csv`: only health-qualified poses with explicit epoch IDs.
- `landmarks.csv`: real filter landmarks with stable feature IDs and epochs;
  retains their last estimate from a health-qualified visual update after they
  leave the filter window. These are sparse local landmarks, not a dense map.
- `map-epoch-N.ply`: latest archived landmark estimate per ID in that epoch.
  Check the epoch's loss events before using it. Never concatenate reset maps
  as though they shared an origin. Files are finalized on clean shutdown/reset.

With the supervisor disabled, estimator state arithmetic and reset behavior
remain unchanged for regression testing. Health diagnostics still run, latch
loss and withhold pose publication. Legacy config files do not gain live approval.

`tools/review_tracking.py RUN --output report.json` summarizes coverage, losses
and each separate map. `tools/plot_local_map.py RUN map.png --epoch 0` plots an
actual exported map without inventing geometry.

Native live runs use bounded `--ram-recording`, including raw measurements and
derived output. Capture finishes before verified archive copying starts. Nested
`--ram-estimator-output` is rejected. Local Stop also protects the saving phase;
closing the window defers closure until saving completes. Desktop video recording
is disabled during live acquisition to remove encoding and SD-write contention.
