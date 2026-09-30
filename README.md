# Raspberry Pi camera + IMU VIO

Working checkpoint: **grid-demo-2026-09-30** — bounded 120-second OpenVINS demos,
real RViz display, a fullscreen LAN webpage, automatic screen video saving, and
repeatable manual Start/Stop. Raspberry Pi 5, IMX219, LSM6DSOX.

```bash
cd /home/pluto/vio-project
bash tools/start_remote_demo.sh
```

Open the printed URL (currently http://192.168.137.196:8766/). Click **Start** for
each take. Keep the rig still for initialization, then move slowly with the full
single AprilGrid visible. Click the picture to reveal Stop and Download controls.
Screen videos save under `/home/pluto/Videos/`; logs and IMU data are in
`data/native-live/`. Every camera frame feeds VIO, but demo mode does not archive
raw camera images, preventing repeated takes from rapidly filling the disk.

**Full-room tracking is not yet reliable.** This is provisional camera-and-IMU
VIO, not independently validated navigation. The original grid demo needs the
sheet visible. Repeated identical sheets do not form a uniquely identified room
map. The separate natural-feature room trial failed during wider turns and was
withdrawn; the original calibration remains the default.

## Recovery and versions

- [Recover after a reboot or replace the SD card](recovery/README.md)
- [Working binary/calibration manifest](recovery/working-demo-manifest.json)
- [Room development findings and next checks](docs/ROOM_DEVELOPMENT.md)
- [Full investigation history](STATUS.md)
- [Earlier setup and calibration documentation](docs/HISTORICAL_README.md)

`stable/grid-demo-2026-09-30` preserves the working baseline. `develop/room-vio`
is for isolated experiments; `main` holds reviewed application changes.

GitHub backs up the runnable source, calibration bundle, dependency locks and
rebuild instructions. The approximately 82 GB of recordings, generated build
outputs and screen videos remain on the Pi for now at the user's request. They
are **not protected against SD-card failure** by a source-code push.

```bash
bash tools/restore_demo.sh --check  # no hardware opened
# On a replacement Pi OS installation with prerequisites installed:
bash tools/restore_demo.sh --build
```

The existing-machine recovery check is exercised. A clean replacement-SD rebuild
still needs testing. Reliable power is required; earlier low-voltage events caused
throttling and coincided with capture failures.
