# Recover the working demo

Checkpoint: **grid-demo-2026-09-30**. The GitHub source backup is not a full SD-card image.
Raw recordings and videos remain only on the Pi by the user's choice on September 30.

## After a normal reboot

```bash
cd /home/pluto/vio-project
bash tools/restore_demo.sh --check
bash tools/start_remote_demo.sh
```

Open the printed LAN URL, currently `http://192.168.137.196:8766/`.
The launcher starts idle. Each click starts a new 120-second capture and screen recording.
Keep the entire grid visible, rest for initialization, then move slowly.
Click the image to reveal controls; Stop saves the take. Refresh after a page update.

## After SD-card replacement

Use the same Raspberry Pi 5, camera mode, rigid sensor mounting, account `pluto`,
and Raspberry Pi OS / Debian 13 aarch64 environment. Preserve the physical mounting:
moving the camera relative to the IMU requires new extrinsic calibration.

Restore the OS's working IMX219/Picamera2/libcamera support and I2C bus 1 access first.
The demo uses LSM6DSOX at I2C address 0x6a. Do not copy reference-dataset calibration.
`system-packages.tsv` and `python-local-inventory.txt` record the present environment;
they are inventories, not commands to blindly replace system packages.

Prerequisites include Git, Python venv/apt/PyQt5/Pillow/Picamera2, build-essential,
Ninja, CMake, curl, grim, ffmpeg, a working Wayland desktop and the Pi hardware drivers.
Use the matching Pi OS repositories for these packages.

```bash
git clone --branch stable/grid-demo-2026-09-30 https://github.com/Nikhil-Sudhan/VIO.git /home/pluto/vio-project
cd /home/pluto/vio-project
bash tools/restore_demo.sh --build
bash tools/start_remote_demo.sh
```

The build uses locked OpenVINS sources and repository patches, plus verified Debian
dependency archives. Upstream archive availability is required: this is a source
recovery procedure, not an offline binary backup. The current-machine `--check`
is tested; a clean replacement-SD rebuild has not yet been exercised.

`working-demo-manifest.json` records hashes of this Pi's working binaries and
calibration. Different compiler/system versions can change rebuilt binary hashes.
The review file independently verifies the physical calibration configuration.

The optional headless display was reduced to 1266x555 to limit screen-capture load:

```bash
XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 wlr-randr --output NOOP-1 --custom-mode 1266x555
```

Use this only when `wlr-randr` lists `NOOP-1`; real monitor names differ.
The LAN launcher currently discovers its address using the laptop hotspot at
192.168.137.1. Update that network assumption if the network changes.

## What is and is not protected

Git contains the runnable application sources, current calibration, dependency
locks, patches, tests and recovery instructions. Build outputs and large recordings
are intentionally excluded. Source restoration does not recover deleted videos,
raw measurements, extracted dependency packages or the whole operating system.

The latest demo saves full-rate IMU, derived VIO and a screen video; it sends every
camera frame to VIO but does not archive raw camera images. These demo sessions
cannot be used as complete raw camera/IMU calibration datasets.

For a complete disaster backup, copy `data/`, `evidence/`, `build/`, `install/`,
`deps/`, `vendor/`, `/home/pluto/Videos/`, the Python environment and a system image
to another device. A backup kept only on this SD card will not survive its failure.

## Version discipline

Keep `stable/grid-demo-2026-09-30` at the known working checkpoint. Use `main` for
reviewed changes and `develop/room-vio` for room experiments. Do not change the
default live calibration to an experiment until it passes a physical movement check.
The first natural-feature room experiment failed and is explicitly withdrawn.
