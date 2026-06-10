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

### `Q_psd_diag` from the IMU datasheet

| Error block | Driven by | PSD formula |
|---|---|---|
| $\delta v$, $\delta\theta$ | sensor white noise | $Q_{psd} = N^2$, $N$ = noise density |
| $\delta b_a$, $\delta b_g$ | bias random walk | $Q_{psd} = \sigma_{BI}^2/\tau_c$, $\tau_c \approx 1\,\mathrm{hr}$ |

($\sigma_{BI}$ = in-run bias stability, $\tau_c$ = its Allan correlation
time.)  Unit conversions:

$$
N_g\,[\mathrm{rad/\sqrt{s}}] = N_g\,[°/s/\sqrt{Hz}]\cdot\tfrac{\pi}{180},
\qquad
N_a\,[\mathrm{m/s/\sqrt{s}}] = N_a\,[g/\sqrt{Hz}]\cdot 9.81
$$

MTi-630 ([datasheet](https://www.xsens.com/hubfs/Downloads/Leaflets/MTi-630.pdf)):

| Spec | Value | Converted | Q_psd entry |
|---|---|---|---|
| Gyro noise density | 0.007 °/s/√Hz | 1.22e-4 rad/√s | $\delta\theta$: **1.5e-8** rad²/s |
| Gyro bias stability | 8 °/h | 3.88e-5 rad/s | $\delta b_g$: **4.2e-13** rad²/s³ |
| Accel noise density | 60 µg/√Hz | 5.89e-4 m/s²/√Hz | $\delta v$: **3.5e-7** m²/s³ |
| Accel bias stability | 10–15 µg | ~1.5e-4 m/s² | $\delta b_a$: **~6e-12** m²/s⁵ |

Datasheet PSDs are a **lower bound**: the filter must also absorb
linearization error, Euler integration, float32 round-off, vibration
aliasing (usually dominant on the car) and temperature-driven bias drift.
Inflate from there; it is the safe direction to err.

### `R_diag`

[PX1120S datasheet](https://www.skytraq.com.tw/homesite/PX1120S_DS.pdf):
velocity accuracy 0.1 m/s → `R_diag = {0.01, 0.01}` $(\mathrm{m/s})^2$.
That is an open-sky spec; in consistently degraded environments raise
`R_diag` rather than relying on the gate.

### `P0_diag`

Variances of how well each state is known at `NAV_Init`:

| Block | Value | σ | Rationale |
|---|---|---|---|
| $\delta v$ | 0.1 | 0.32 m/s | near rest / first GNSS fix |
| $\delta\theta$ | 1e-3 | 1.8° | MTi fused attitude as seed |
| $\delta b_a$ | 1e-4 | 1 mg | turn-on bias class |
| $\delta b_g$ | 1e-5 | 0.18 °/s | turn-on bias class |

### Remaining scalars

- `gnss_chi2_gate`: 9.21 (χ², 2 DOF, 99%).  Reject threshold vs confidence
  (higher = looser gate):

  | DOF | 90% | 95% | 99% | 99.9% |
  |---|---|---|---|---|
  | 2 | 4.61 | 5.99 | **9.21** | 13.82 |
- `lever_arm`: antenna position relative to the IMU, body axes (m).
  `{0,0,0}` disables.  At ~2 rad/s yaw rate a 1 m offset is ~2 m/s of
  apparent velocity, so measure it.
- `att_tilt_var`: innovation components ≈ sin(tilt error); MTi-630 dynamic
  roll/pitch ~0.25° → $(0.25 \cdot \pi/180)^2 \approx$ **2e-5** rad².

## Benchmark

`tests/i2nav_test.c`, velocity RMSE (m/s):

| Dataset | EKF vN / vE | GNSS-only vN / vE |
|---|---|---|
| ICM20602 | 0.016 / 0.015 | 0.247 / 0.257 |
| ADIS16460 | 0.010 / 0.009 | 0.267 / 0.276 |
| ADIS16465 | 0.013 / 0.013 | 0.265 / 0.275 |
| i300 | 0.007 / 0.007 | 0.265 / 0.275 |

