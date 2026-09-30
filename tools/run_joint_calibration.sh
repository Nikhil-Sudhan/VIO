#!/bin/bash
# Fit a candidate calibration from the measured rig data; never auto-accept it.
set -euo pipefail
if [ "$#" -ne 2 ] && [ "$#" -ne 4 ] && [ "$#" -ne 7 ]; then
  echo 'Usage: bash tools/run_joint_calibration.sh REVIEWED_NOISE.yaml NEW_ATTEMPT_DIRECTORY [BAG_START_S BAG_END_S [SOURCE_BAG CAMERA.yaml MEASURED_TARGET.yaml]]' >&2
  exit 2
fi
source "$(dirname -- "$0")/kalibr_env.sh"
cd "$VIO_ROOT"
VIO_NOISE_INPUT="$(realpath -- "$1")"
VIO_ATTEMPT="$(realpath -m -- "$2")"
VIO_START_S="${3:-102.054662994}"
VIO_END_S="${4:-262.054662994}"
VIO_SOURCE_BAG="$(realpath -- "${5:-data/calibration-poses4.bag}")"
VIO_CAMERA_INPUT="$(realpath -- "${6:-calibration/joint-preparation/camera-intrinsics.yaml}")"
VIO_TARGET_INPUT="$(realpath -- "${7:-calibration/target/target-screen-MEASURED.yaml}")"
VIO_KALIBR_MAX_IMAGE_HZ="${VIO_KALIBR_MAX_IMAGE_HZ:-10}"
test -f "$VIO_NOISE_INPUT"
test -f "$VIO_SOURCE_BAG"
test -f "$VIO_CAMERA_INPUT"
test -f "$VIO_TARGET_INPUT"
test ! -e "$VIO_ATTEMPT"
mkdir -- "$VIO_ATTEMPT"
cp -- "$VIO_NOISE_INPUT" "$VIO_ATTEMPT/noise-input.yaml"
cp -- "$VIO_CAMERA_INPUT" "$VIO_ATTEMPT/camera-input.yaml"
cp -- "$VIO_TARGET_INPUT" "$VIO_ATTEMPT/target-input.yaml"
if [ "$#" -ne 7 ]; then
  cp -- calibration/joint-preparation/plan.json "$VIO_ATTEMPT/selection-plan.json"
fi
printf 'Actual bag interval: %s to %s seconds\nMaximum feature extraction rate: %s Hz\n' "$VIO_START_S" "$VIO_END_S" "$VIO_KALIBR_MAX_IMAGE_HZ" > "$VIO_ATTEMPT/run-arguments.txt"
printf 'Source bag: %s\nCamera input: %s\nTarget input: %s\nNoise input: %s\n' "$VIO_SOURCE_BAG" "$VIO_CAMERA_INPUT" "$VIO_TARGET_INPUT" "$VIO_NOISE_INPUT" >> "$VIO_ATTEMPT/run-arguments.txt"
# A hard link avoids duplicating 2.9 GB. Kalibr opens the bag read-only and writes
# its results beside it with distinct suffixes. Original data stays preserved.
ln -- "$VIO_SOURCE_BAG" "$VIO_ATTEMPT/input.bag"
printf '%s\n' 'CANDIDATE ONLY. Review inertial/visual residuals, covariance, motion observability and repeatability before use.' > "$VIO_ATTEMPT/UNREVIEWED.txt"
sha256sum "$VIO_ATTEMPT/noise-input.yaml" "$VIO_ATTEMPT/camera-input.yaml" "$VIO_ATTEMPT/target-input.yaml" > "$VIO_ATTEMPT/input-config-sha256.txt"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
export PYTHONUNBUFFERED=1
printf '%s\n' 'Analytic covariance unavailable: the pinned Kalibr --recover-covariance path calls an unsupported BSplineMotionError Jacobian and aborts. This run omits that optional step. Residual, weight-sensitivity and independent-repeat checks remain required; no covariance is invented.' > "$VIO_ATTEMPT/UNCERTAINTY_LIMITATION.txt"
"$VIO_ROOT/build/kalibr-ws/devel/lib/kalibr/kalibr_calibrate_imu_camera" \
  --bag "$VIO_ATTEMPT/input.bag" --bag-from-to "$VIO_START_S" "$VIO_END_S" --bag-freq "$VIO_KALIBR_MAX_IMAGE_HZ" \
  --cams "$VIO_ATTEMPT/camera-input.yaml" --imu "$VIO_ATTEMPT/noise-input.yaml" \
  --target "$VIO_ATTEMPT/target-input.yaml" --dont-show-report --export-poses \
  > "$VIO_ATTEMPT/kalibr.log" 2>&1
printf '%s\n' 'Fit process completed; outputs remain unreviewed candidates.'
