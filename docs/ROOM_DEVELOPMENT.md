# Room VIO development, September 30

The bounded single-grid demo is the rollback baseline. Full-room VIO is not ready.
The user confirmed the camera and IMU are rigidly fixed and moved together.

The first isolated trial disabled `use_aprilgrid_features` and
`aprilgrid_zupt_veto`, retaining ordinary KLT observations and the original
physical calibration. An offline replay of the earlier successful desk recording
completed: 1,052 poses, peak position norm 0.160 m, final norm 0.013 m. This was
feasibility evidence, not independent accuracy validation.

Live session `20260930_204002_h42ebrd4` initially tracked a small translation and
turn. During wider room motion it lost visual corrections and diverged. Power
remained clean (`get_throttled=0x0`). A repeated room take also diverged. The trial
review is withdrawn and must not become the default configuration.

The original single-grid tracker uses fixed tag IDs 0–35. Identical sheets in
different room locations are ambiguous: matching a repeated ID does not identify
a unique physical landmark. Natural-feature mode ignores tag identities but still
needs consistent image tracks, valid camera/IMU calibration and sufficient texture.

Next work on `develop/room-vio`:

1. Preserve a full-rate raw diagnostic capture with stationary periods and slow
   separate yaw, pitch and roll while one grid remains visible. Current demo-only
   captures lack a complete camera archive and must not be presented as complete
   calibration inputs.
2. Check gyro/image rotation and timing consistency with translation-aware
   diagnostics; the existing short-pair pure-rotation approximation cannot prove
   the extrinsics are wrong or right.
3. Measure image track survival, inlier rejection, blur and exposure through the
   transition from grid to room texture. Compare isolated candidates on preserved
   recordings before another live movement.
4. Verify stationary → translation → slow turn → grid leaving view → return, with
   fresh visual corrections and bounded latency. Save failed runs as failures.

Do not hide drift, freeze the last good pose, send target-PnP poses into VIO, or
claim room tracking based only on a stationary or single-grid result.
