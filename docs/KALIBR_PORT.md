# Native Debian 13 Kalibr build

Upstream source is pinned to `1f60227442d25e36365ef5f72cd80b9666d73467`. The project uses Debian's native ROS 1 libraries only for Kalibr and bag conversion; OpenVINS itself is built without ROS. Sources and dependencies remain local to this project, preserving the working Pi camera stack.

The [upstream installation instructions](https://github.com/ethz-asl/kalibr/wiki/installation) target older Ubuntu environments. This is a compatibility port, whose successful compilation alone does not establish correct calibration. `tools/prepare_kalibr.py` verifies the source archive and applies the following saved patches; `tools/build_kalibr.sh` builds it with the local dependency prefix.

| Patch | Reason |
|---|---|
| `kalibr-python-version.patch` | Derive Boost.Python's suffix from the selected Python version instead of assuming Python 3.8. |
| `kalibr-boost-endian.patch` | Use Boost's available endian definitions instead of the removed private header. |
| `kalibr-bsplines-suitesparse-link.patch` | Explicitly link bsplines to the SuiteSparse libraries it calls; fixes an unresolved CHOLMOD symbol at Python import. |
| `kalibr-matplotlib-colorbar.patch` | Attach colorbars to the existing axes as required by modern Matplotlib. |
| `kalibr-boost-placeholders.patch` | Qualify bind placeholders in the spline wrapper. |
| `kalibr-cholmod-long.patch` | Use the long-index CHOLMOD representation for matrices whose row and column index arrays are both long. |
| `kalibr-suitesparse7-templates.patch` | Match the added SuiteSparse integer template parameter and include its real public declaration rather than obsolete forward declarations. |
| `kalibr-public-column-norm.patch` | Replace a dependency on an uninstalled private SPQR header with the maximum column 2-norm computed from validated public CHOLMOD storage using Eigen's stable norm. |
| `kalibr-numpy-headers.patch` | Discover NumPy's C include path through the selected interpreter and suppress test-only generated bindings when testing is disabled. |
| `kalibr-numpy2-enum.patch` | Use the available legacy enum name in NumPy 2 diagnostic code. |
| `catkin-local-prefix.patch` | Preserve absolute imported library paths; Debian catkin's `/usr` rewrite otherwise corrupts a relocated prefix. Applied by `tools/patch_catkin_prefix.py` only to extracted dependencies. |

SuiteSparse interface changes were checked against its [public v7.10.1 header](https://github.com/DrTimothyAldenDavis/SuiteSparse/blob/v7.10.1/SPQR/Include/SuiteSparseQR.hpp). The replaced private helper computes a maximum column 2-norm, as shown in the [upstream implementation](https://github.com/DrTimothyAldenDavis/SuiteSparse/blob/v7.10.1/SPQR/Source/spqr_maxcolnorm.cpp). The replacement rejects unsupported matrix layouts rather than reinterpreting them.

`tools/check_kalibr.sh` checks module imports, NumPy/Eigen quaternion conversions, spline values/derivatives against SciPy, and the changed QR tolerance calculation against known norms (including large/small values and invalid layout). These checks are separate from an actual calibration dataset run and its residual/observability review. Consult STATUS.md for checks actually completed.

Camera-only OpenCV calibration is also separate: AprilTag edge fitting replaced unstable default subpixel refinement on small displayed tags. The [OpenCV parameter documentation](https://docs.opencv.org/4.10.0/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html) describes its size-dependent refinement window. Saved static-image comparisons establish that corner detection contributed to the earlier rejected fit; camera motion was not its sole demonstrated cause.
