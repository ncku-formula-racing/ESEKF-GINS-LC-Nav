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

## Derivation

ESKF: the INS integrates the IMU into a *nominal* state (velocity +
attitude); the EKF estimates a 12-dim *error* state

$$
x = [\delta v,\ \delta\theta,\ \delta b_a,\ \delta b_g]
$$

(NED velocity error, attitude error, accel / gyro bias error).  After each
update the error is injected into the nominal state and reset to zero, so
the filter always linearizes around a small error.

The EKF needs two objects, both derived below.  The transition matrix
$\Phi$ propagates the error covariance $P$ between fixes; it comes from the
error-state differential equations, whose coefficient matrix is $F$.  The
observation matrices $H$ map the error state onto each measurement.

### Conventions

- Body frame **FRD**, navigation frame **NED**.  Earth rotation / Coriolis
  ignored (far below MEMS noise at track scale).
- Quaternions are **Hamilton** convention, stored `[w, x, y, z]`, body→NED.
- $\hat\cdot$ = nominal (INS) quantity; $\delta$ = error = **truth −
  estimate**.

### Nominal propagation (`NAV_FeedIMU` → `INS_Propagate`)

The IMU measures specific force $a_{meas}$ and angular rate $\omega_{meas}$
(body frame).  Bias-compensate first:

$$
f = a_{meas} - \hat b_a,\qquad \omega = \omega_{meas} - \hat b_g
$$

$R(\hat q)$ is the rotation matrix of the nominal quaternion ($v_{ned} =
R(\hat q)\,v_{body}$); below it is abbreviated $R$, and it is always
evaluated at the current nominal attitude.  With gravity
$g_n = (0, 0, +g)$ in NED:

$$
\dot{\hat v} = R\,f + g_n,\qquad
\dot{\hat q} = \tfrac{1}{2}\,\hat q \otimes (0, \omega)
$$

integrated with first-order Euler + quaternion renormalization per sample.
Here $\otimes$ is the quaternion product and $(0, \omega)$ the pure
quaternion with vector part $\omega$.

The quaternion equation comes from composing rotations: over $\Delta t$ the
body rotates by $|\omega|\Delta t$ about $\omega/|\omega|$, whose quaternion
(half-angle form) is

$$
\delta q(\Delta t) = \left(\cos\tfrac{|\omega|\Delta t}{2},\
\tfrac{\omega}{|\omega|}\sin\tfrac{|\omega|\Delta t}{2}\right)
$$

A body-frame rotation composes on the right,
$\hat q(t+\Delta t) = \hat q(t) \otimes \delta q(\Delta t)$, so

$$
\dot{\hat q} = \hat q \otimes \left.\tfrac{d\,\delta q}{d\Delta t}\right|_{\Delta t = 0}
             = \hat q \otimes (0,\ \omega/2)
$$

(A NED-frame rotation composes on the left, which is exactly how the error
injection applies $\delta\theta$.)

### Error-state dynamics

Velocity and bias errors are plain differences ($\delta v = v_{true} - \hat
v$, etc.).  The attitude error is a rotation vector $\delta\theta$ (**NED
frame**) entering through a left-multiplied error quaternion, using the
same half-angle form as above with $\delta\theta$ in place of
$\omega\Delta t$:

$$
q_{true} = \delta q(\delta\theta) \otimes \hat q
$$

The EKF keeps only first order in the error state; for the rotation this
means

$$
\delta q = \begin{pmatrix}1\\ \delta\theta/2\end{pmatrix} + O(\delta\theta^2),
\qquad
R_{true} = \left(I + [\delta\theta]_\times\right) R + O(\delta\theta^2)
$$

with $[v]_\times$ the skew-symmetric matrix of $v$.  The derivations below
use these forms and drop $O(\delta\theta^2)$ terms throughout.

**Velocity.**  Truth and nominal see the same $a_{meas}$, so the true
specific force is $f - \delta b_a - n_a$ ($n_a$ = accel white noise):

$$
\dot v_{true} = (I + [\delta\theta]_\times)\,R\,(f - \delta b_a - n_a) + g_n
$$

Expand, drop second-order products, and subtract $\dot{\hat v} = Rf + g_n$:

$$
\dot{\delta v} = -[R f]_\times\, \delta\theta - R\,\delta b_a - R\,n_a
$$

i.e. a tilt error mis-projects the specific force (gravity included) into
NED; an accel bias error pushes velocity directly.

**Attitude.**  Differentiate the definition $R_{true}R^T = I +
[\delta\theta]_\times$, using $\dot R = R[\omega]_\times$ and
$(R[\omega]_\times)^T = -[\omega]_\times R^T$:

$$
\frac{d}{dt}(R_{true}R^T) = R_{true}\,[\omega_{true} - \omega]_\times\,R^T
$$

The measured rate cancels inside the bracket: $\omega_{true} - \omega =
-\delta b_g - n_g$.  The bracket is already first order, so replacing
$R_{true}$ by $R$ only discards $O(\delta\theta^2)$ terms; with
$R[v]_\times R^T = [Rv]_\times$:

$$
\dot{\delta\theta} = -R\,\delta b_g - R\,n_g
$$

Because $\omega_{meas}$ cancelled exactly, the NED-frame convention has
**no** $-[\omega]_\times\delta\theta$ term (it only appears with a
body-frame $\delta\theta$).  Quaternion version of the same derivation: top
of `src/nav/nav.c`.

**Biases.**  Random walk: $\dot{\delta b_a} = n_{ba}$, $\dot{\delta b_g} =
n_{bg}$.

### F and Φ (`build_Phi`)

Collecting the equations above into $\dot x = Fx + \text{noise}$, in 3×3
blocks over $x$:

$$
F = \begin{bmatrix}
0 & -[R f]_\times & -R & 0\\
0 & 0 & 0 & -R\\
0 & 0 & 0 & 0\\
0 & 0 & 0 & 0
\end{bmatrix},
\qquad
\Phi = \exp(F\Delta t) = I_{12} + F\,\Delta t + O(\Delta t^2)
$$

`build_Phi` writes $\Phi$ into `A_buf` every IMU step (identity diagonal,
six elements of $-[Rf]_\times\Delta t$, two $-R\Delta t$ blocks).  It is
rebuilt each step because $R$ and $f$ change.

### Process noise

All process noises ($n_a$, $n_g$, $n_{ba}$, $n_{bg}$) are modeled zero-mean
white Gaussian and mutually independent.  Their PSDs are `Q_psd_diag`, a
continuous-time PSD (variance per second); each step uses
$Q_d = \mathrm{diag}(Q_{psd})\cdot\Delta t$, making the tuning independent
of IMU rate.  The noise enters $\delta v$/$\delta\theta$ through $R\,n$,
and $R\,\mathrm{diag}(\sigma^2)R^T = \sigma^2 I$ only if the three axes
share one density, so keep each block's three entries equal.

### GNSS observation H (`NAV_FeedGNSS_Vel`)

The antenna sits at $r$ = `lever_arm` (body, m) from the IMU, so GNSS
measures $v_{ant} = v_{imu} + R(\omega \times r)$.  Remove the rotational
term, then form the innovation against the nominal velocity:

$$
y = \begin{pmatrix} v_N^{gnss} \\ v_E^{gnss} \end{pmatrix}
  - [R(\omega\times r)]_{N,E}
  - \begin{pmatrix} \hat v_N \\ \hat v_E \end{pmatrix},
\qquad
H = [\,I_2 \ \ 0_{2\times10}\,]
$$

The corrected measurement equals $\hat v + \delta v + \nu$, i.e.
$y = Hx + \nu$ with $H$ selecting $\delta v_{N,E}$ and the observation
noise modeled zero-mean white Gaussian,
$\nu \sim \mathcal{N}(0,\ \mathrm{diag}(\texttt{R\_diag}))$.  Outliers are
rejected by a chi-square gate on $d^2 = y^T S^{-1} y$ with $S = HPH^T +
R_{gnss}$ (the innovation covariance).  Under the Gaussian model
$d^2 \sim \chi^2(2)$ when the filter is consistent, so
`gnss_chi2_gate = 9.21` rejects at 99%.  The gate is loose after init while
$P$ is large and tightens as the filter converges.

### Attitude observation H (`NAV_FeedAttitude`, optional)

Observes the gravity direction in body frame, $g_b = R^T e_3$ (the third
row of $R$), which is independent of heading.  Innovation
$z = g_b^{meas} - g_b^{ins}$,
with observation noise modeled zero-mean white Gaussian,
$\nu \sim \mathcal{N}(0,\ \texttt{att\_tilt\_var}\cdot I_3)$; perturbing $R$
with $\delta\theta$ gives $H$ rows $(R_{1i},\ -R_{0i},\ 0)$ in the
$\delta\theta$ columns.  The heading column is zero by construction, so the
MTi's magnetometer yaw is never injected.
