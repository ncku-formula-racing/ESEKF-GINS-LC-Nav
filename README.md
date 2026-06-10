# ESEKF-GINS-LC-Nav

A C implementation of ESEKF-Based GNSS/INS Loosely-Coupled Navigation using CMSIS-DSP Library.

## Usage

Working example: [`tests/nav_example.c`](./tests/nav_example.c).

```
init       ──> NAV_Init(vel0, quat0, gravity, cfg)
IMU sample ──> NAV_FeedIMU(accel, gyro, dt)
GNSS fix   ──> NAV_FeedGNSS_Vel(vN, vE)
(optional) ──> NAV_FeedAttitude(quat)
anytime    ──> NAV_GetOutput(gyro, &out)
```

- Frames: body = **FRD** (x forward, y right, z down, fixed to the car);
  navigation = **NED** (North, East, Down).  All body-frame inputs
  (`accel`, `gyro`, `quat`, `lever_arm`) must share the same FRD axes.
- `accel` (m/s²), `gyro` (rad/s): raw calibrated MTi-630 output, body frame
- `vN, vE` (m/s): GNSS North/East velocity (PX1120S).  Returns
  `NAV_GNSS_OK` / `NAV_GNSS_GATED` (outlier rejected) / `NAV_GNSS_FAILED`.
- `quat` (`[w, x, y, z]`, body→NED): the MTi's **fused** attitude, not a raw
  measurement.  Optional (exists because the MTi-630 has an onboard AHRS);
  consumed only by `NAV_Init` (seed) and `NAV_FeedAttitude` (roll/pitch
  aid).
- Output: `vx, vy` body-frame horizontal velocity (m/s); `wx, wy, wz`
  bias-corrected angular rate (rad/s)
- `NAV_Context` is ~3.5 KB with every buffer inline, so make it `static`
  on the MCU, not a stack local.

Everything tunable enters once, through `NAV_Config`:

| Field | Meaning |
|---|---|
| `P0_diag[12]` | initial state uncertainty (variances) |
| `Q_psd_diag[12]` | process noise PSD (variance per second) |
| `R_diag[2]` | GNSS velocity measurement variance |
| `gnss_chi2_gate` | GNSS outlier gate (χ², 2 DOF) |
| `lever_arm[3]` | GNSS antenna position rel. IMU (body, m) |
| `att_tilt_var` | roll/pitch aid variance (only with `NAV_FeedAttitude`) |

How each number is chosen: next section.  Where each one enters the filter:
[Derivation](#derivation).

## Tuning (MTi-630 + PX1120S)
## Benchmark

`tests/i2nav_test.c`, velocity RMSE (m/s):

| Dataset | EKF vN / vE | GNSS-only vN / vE |
|---|---|---|
| ICM20602 | 0.016 / 0.015 | 0.247 / 0.257 |
| ADIS16460 | 0.010 / 0.009 | 0.267 / 0.276 |
| ADIS16465 | 0.013 / 0.013 | 0.265 / 0.275 |
| i300 | 0.007 / 0.007 | 0.265 / 0.275 |

