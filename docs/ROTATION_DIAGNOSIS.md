# Saved-data rotation diagnosis

Scope: the original moving AprilGrid capture, not independent ground truth or accepted calibration. The target poses use the measured grid and fitted camera intrinsics.

The initial stationary interval extends to approximately 117.7 seconds in adapter-relative time. The previously suspected seven-second zero-velocity release delay is not supported by these target poses; no ZUPT change was applied.

Gyro-only integration and VIO each disagree with target-derived orientation by approximately three degrees during early motion. The disagreement therefore precedes significant visual corrections. A small rotation/time fit and an approximate scalar gyro-sensitivity fit do not materially remove the discrepancy on the separate review interval. Neither fitted correction has been applied to acquisition or calibration. Rolling shutter, target-pose error and inertial calibration remain possible contributors; this analysis does not identify one proven cause.

Reproduce the relative-rotation fit with `source tools/env.sh` then `python3 tools/audit_rotation_alignment.py`. The script reads preserved target observations and IMU samples and writes diagnostic JSON under evidence/. Fit and review intervals are explicit in the script and reports. The optional scale fit scales relative rotation vectors and is only an approximate diagnostic for changing-axis motion.

Original report snapshots follow.

```json
{
  "rotation_time_fit": {
    "scope": "Diagnostic rotation/time fit only; not accepted calibration or independent ground truth",
    "training_interval_s": [
      182.06,
      242.05
    ],
    "review_interval_s": [
      112.06,
      172.05
    ],
    "measured_static_gyro_bias_rad_s": [
      0.013199480244984607,
      -0.009611245858059649,
      0.0018836508729826644
    ],
    "original_offset_s": 0.007970614446713634,
    "fit_converged": true,
    "fitted_offset_s": 0.009723367133206939,
    "rotation_change_deg": 0.6571377209247762,
    "fitted_R_cam_imu": [
      [
        -0.2207029172340081,
        -0.9547799074334342,
        0.1992123256372484
      ],
      [
        0.39244451862671664,
        -0.2739168822144363,
        -0.8780414804767296
      ],
      [
        0.8929039825895295,
        -0.11560653095446294,
        0.4351523961515455
      ]
    ],
    "train_before": {
      "pairs": 414,
      "rms_deg": 0.38412269978276237,
      "p50_p95_max_deg": [
        0.21419538174755917,
        0.8250674293773383,
        1.410090250453808
      ]
    },
    "train_after": {
      "pairs": 414,
      "rms_deg": 0.3759996981866325,
      "p50_p95_max_deg": [
        0.22431245608966052,
        0.7704762681563194,
        1.3841594691375816
      ]
    },
    "review_before": {
      "pairs": 341,
      "rms_deg": 0.41760840177067593,
      "p50_p95_max_deg": [
        0.17279378660806155,
        0.7870490959272306,
        3.643789546680631
      ]
    },
    "review_after": {
      "pairs": 341,
      "rms_deg": 0.4160457314782897,
      "p50_p95_max_deg": [
        0.16899011632107633,
        0.8010787386648516,
        3.5831104880407487
      ]
    }
  },
  "approximate_scalar_scale_fit": {
    "scope": "Approximate gyro scale diagnostic; must not be applied as measured calibration",
    "scale_factor": 1.0071670871893803,
    "offset_s": 0.009641573688697499,
    "rotation_change_deg": 0.7719440858809036,
    "train_before": {
      "pairs": 414,
      "rms_deg": 0.38412269978276253,
      "p50_p95_max_deg": [
        0.2141953817475602,
        0.8250674293773369,
        1.4100902504538118
      ]
    },
    "train_after": {
      "pairs": 414,
      "rms_deg": 0.371047999208184,
      "p50_p95_max_deg": [
        0.21087425232362875,
        0.7594206279017446,
        1.386904971130012
      ]
    },
    "review_before": {
      "pairs": 341,
      "rms_deg": 0.417608401770676,
      "p50_p95_max_deg": [
        0.17279378660806524,
        0.7870490959272322,
        3.643789546680631
      ]
    },
    "review_after": {
      "pairs": 341,
      "rms_deg": 0.41380771043094916,
      "p50_p95_max_deg": [
        0.1696326540980255,
        0.792117150461173,
        3.5871608949750047
      ]
    }
  }
}
```
