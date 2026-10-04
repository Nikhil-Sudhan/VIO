# Raspberry Pi camera + IMU VIO

## Current hardware status — October 4, 2026

The new screw-fixed three-pin enclosure has a fresh **provisional** camera/IMU
mounting calibration. Both sensors deliver continuous data. The larger image buffer passed a 3.5-second stall but a later ~8-second SD
write delay still overflowed it. **Record sensors** now stages each bounded take
in RAM and copies/verifies every file after Stop; live storage writes cannot
block its image writer. RAM staging passed a delayed-save hardware test.

**Experimental live local mapping is available with `supervised-natural-v3`.**
The new supervisor requires actual visual corrections before displaying poses.
Loss clears the live path, reinitializes, and starts a separate coordinate frame;
old and new maps are never silently joined. This is a bounded handheld development
mode, not validated navigation or full room reconstruction. V1/V2 remain withdrawn.

Use **Start live tracking** on the desktop or LAN page. Keep still until ready,
move sideways slowly until tracking starts, then explore slowly. If tracking is
lost, hold still for reinitialization and establish depth again. No AprilGrid is
needed for this mode. **Stop and save** preserves raw camera/IMU measurements,
health diagnostics, poses and actual sparse landmarks. Both live and raw capture
stage in RAM and verify the archive after capture stops; screen video recording
is disabled during acquisition. Runs remain bounded to 120 seconds by default.

Replays of the short natural-only take and the earlier usable room take retain
their exact previous poses/covariances and save 95 and 186 distinct natural
landmarks respectively. The untrackable room take is rejected with zero usable
poses, rather than hundreds of metres of drift. A black-image fault injection
triggers loss; a separately labelled synthetic recovery test establishes another
map after reinitialization. These tests establish software behavior, not accuracy.

A paced streaming check completes all561 images with identical poses/map, maximum
22.6ms processing against a50ms image period. A fresh15s stationary hardware check
captures all300 images, passes strict raw checks, finishes the live estimator and
verifies every archived file. It correctly reports ready without claiming depth.
The first moving live run completed2401 frames with812 health-qualified poses and186 landmarks; it lost visual corrections at46.755s near a plain wall and restarted. Whole-room accuracy and flight operation remain unverified.
See [tracking health and map files](docs/TRACKING_HEALTH.md) and
`evidence/new-case-20261004/tracking-recovery/`.

Fresh calibration acquisition passed: 5,743 frames and 58,999 IMU samples. Two
independent joint-fit sections agree within 0.67 degrees, 8.8 mm and 0.96 ms.
V2 earlier-recording consistency against shared-calibration target observations
is 5.8 cm RMS / 12.4 cm maximum, **not independent accuracy validation**.

Latest short user-controlled take saved all **561 frames / 6,157 IMU samples**.
Offline replay excluding the visible grid region forms 20 natural-feature
landmarks, reaches 19.4 cm displacement and ends 0.9 cm from its starting estimate.
This is a bounded replay result, **not an independently measured accuracy figure**.
The supervisor rejects the untrackable room take; its missing depth remains unresolved.
See `evidence/new-case-20261004/local-sideways-review.json` and its result plot.

Existing lens intrinsics remain provisional because new still views cover too
little of the image for a full lens recalibration. Printed A4 scale is nominal,
IMU noise weights are provisional, and rolling shutter is unmodeled. Reliable
room mapping and navigation are not yet validated. Current details:
`evidence/new-case-20261004/status.json`.

Historical checkpoint (old mounting only): **grid-demo-2026-09-30** — bounded 120-second OpenVINS demos,
real RViz display, a fullscreen LAN webpage, automatic screen video saving, and
repeatable manual Start/Stop. Raspberry Pi 5, IMX219, LSM6DSOX.

```bash
cd /home/pluto/vio-project
bash tools/start_remote_demo.sh
```

Open the printed URL (currently http://192.168.137.196:8766/) to inspect the
last saved view, start **Record sensors**, or start the experimental live mapping mode. Keep the rig still for initialization, then move slowly with the whole
case fixed together and a textured room scene visible. No AprilGrid is required
for the current natural-feature trial. Click the picture to reveal Stop and Download controls.
Historical screen videos are under `/home/pluto/Videos/`; current raw data and maps are in
`data/native-live/`. Current live mode archives every camera frame. Legacy demo mode did not archive
raw camera images, preventing repeated takes from rapidly filling the disk.

**Full-room tracking is not yet reliable.** This is provisional camera-and-IMU
VIO, not independently validated navigation. The historical grid demo needed the
sheet visible. The current default uses the new mounting and natural features;
old-mounting calibrations remain invalidated. Repeated identical sheets do not
form a uniquely identified room map. See `evidence/new-case-20261004/room-test2-review.json`
for the failed live trial and diagnostic replay comparison.

## Recovery and versions

- [Recover after a reboot or replace the SD card](recovery/README.md)
- [Working binary/calibration manifest](recovery/working-demo-manifest.json)
- [Room development findings and next checks](docs/ROOM_DEVELOPMENT.md)
- [Full investigation history](STATUS.md)
- [Earlier setup and calibration documentation](docs/HISTORICAL_README.md)

`stable/grid-demo-2026-09-30` preserves the working baseline. `develop/room-vio`
is for isolated experiments; `main` holds reviewed application changes.

GitHub backs up the historical runnable source, calibration bundle, dependency
locks and rebuild instructions. On October 4, old physical recordings, derived
calibration bags and VIO screen videos were deleted at the user's request,
recovering about 85.4 GB. Small session/analysis reports were retained under
`evidence/new-case-20261004/old-recording-metadata/`; the deletion manifest is
`evidence/new-case-20261004/cleanup.json`. New recordings use `data/new-case-20261004/`.

```bash
bash tools/restore_demo.sh --check  # no hardware opened
# On a replacement Pi OS installation with prerequisites installed:
bash tools/restore_demo.sh --build
```

The existing-machine recovery check is exercised. A clean replacement-SD rebuild
still needs testing. Reliable power is required; earlier low-voltage events caused
throttling and coincided with capture failures.
