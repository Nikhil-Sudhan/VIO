#!/bin/bash
set -euo pipefail
source "$(dirname -- "$0")/env.sh"
cd "$VIO_ROOT"
expected=$(/usr/bin/python3 -c 'import json; print(json.load(open("config/sources.lock.json"))["openvins"]["commit"])')
test "$(git -C vendor/open_vins rev-parse HEAD)" = "$expected"
# Only the pinned source plus these exact recorded patches may be built.
patch_files=("$VIO_ROOT/patches/openvins-feature-cleanup.patch" "$VIO_ROOT/patches/openvins-zupt-visual-veto.patch" "$VIO_ROOT/patches/openvins-ekf-cross-covariance.patch")
for patch_file in "${patch_files[@]}"; do
  if git -C vendor/open_vins apply --check "$patch_file" 2>/dev/null; then
    git -C vendor/open_vins apply "$patch_file"
  else
    git -C vendor/open_vins apply --reverse --check "$patch_file"
  fi
done
# Disjoint recorded patches may list paths in a different order than git diff.
# Compare the complete per-file diff bytes, not just the set of changed paths.
canonical_patch() {
  /usr/bin/python3 -c 'import sys; d=sys.stdin.buffer.read(); parts=d.split(b"diff --git "); assert not parts[0]; sys.stdout.buffer.write(b"".join(b"diff --git "+p for p in sorted(parts[1:])))'
}
cmp <(git -C vendor/open_vins diff --binary | canonical_patch) <(cat "${patch_files[@]}" | canonical_patch)
test -z "$(git -C vendor/open_vins ls-files --others --exclude-standard)"
git -C vendor/open_vins diff --cached --exit-code
cmake -S vendor/open_vins/ov_msckf -B build/openvins -G Ninja \
  -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_DISABLE_FIND_PACKAGE_catkin=TRUE -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=TRUE \
  -DCMAKE_INSTALL_PREFIX="$VIO_ROOT/install" \
  -DCMAKE_LIBRARY_PATH="$VIO_ROOT/deps/usr/lib/aarch64-linux-gnu/blas;$VIO_ROOT/deps/usr/lib/aarch64-linux-gnu/lapack"
cmake --build build/openvins -j"${VIO_BUILD_JOBS:-2}"
cmake --install build/openvins
cmake -S adapter -B build/adapter -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/adapter -j"${VIO_BUILD_JOBS:-2}"
