#ifndef NAV_H
#define NAV_H

#include "ekf/ekf.h"
#include "ins/ins.h"

#define NAV_STATE_DIM 12    // dv(3) + dpsi(3) + b_a(3) + b_g(3)
#define NAV_GNSS_VEL_DIM 2  // GNSS velocity measurement: vN, vE
#define NAV_ATT_DIM 3       // attitude measurement: gravity direction in body
#define NAV_MAX_MEAS_DIM 3  // max(GNSS, attitude) -- sizes EKF workspace and R

// State vector offsets
enum {
    SV_DV = 0,    // dv   [0:3)
    SV_DPSI = 3,  // dpsi [3:6)
    SV_BA = 6,    // b_a [6:9)
    SV_BG = 9,    // b_g [9:12)
};

// ~3.5 KB -- should be static or global on embedded targets (not on stack).
//
// NED-frame phi-angle ESKF.  All four error blocks in x_buf are reset to
// zero after every GNSS update; the persistent estimates live in:
//   ins.vel  / ins.quat            -- kinematic state
//   bias_a   / bias_g              -- accel / gyro bias estimates (body)
//   Q_psd                          -- continuous-time process noise PSD
//                                     (Q_buf is rebuilt as Q_psd * dt each
//                                     step)
//
typedef struct {
    EKF_Context ekf;
    INS_State ins;

    float32_t bias_a[3];
    float32_t bias_g[3];
    float32_t
        Q_psd[NAV_STATE_DIM];  // continuous-time PSD (variance per second)
    float32_t chi2_gate;       // GNSS chi-square reject threshold (2 DOF)

    float32_t gnss_R[NAV_GNSS_VEL_DIM];  // GNSS vel measurement variance (diag)
    float32_t att_R;                     // tilt measurement variance (diag)
    float32_t lever_arm[3];              // GNSS antenna pos rel. IMU (body, m)
    float32_t last_w[3];  // last bias-corrected gyro (body, rad/s)

    float32_t x_buf[NAV_STATE_DIM];
    float32_t P_buf[NAV_STATE_DIM * NAV_STATE_DIM];
    float32_t A_buf[NAV_STATE_DIM * NAV_STATE_DIM];
    float32_t Q_buf[NAV_STATE_DIM * NAV_STATE_DIM];
    // R_buf is filled per update (GNSS is 2x2, attitude is 3x3), so it is sized
    // for the larger one.
    float32_t R_buf[NAV_MAX_MEAS_DIM * NAV_MAX_MEAS_DIM];
    float32_t work_buf[EKF_WORK_SIZE(NAV_STATE_DIM, NAV_MAX_MEAS_DIM)];
} NAV_Context;

typedef struct {
    float32_t vx, vy;      // body frame (m/s)
    float32_t wx, wy, wz;  // (rad/s)
} NAV_Output;

typedef struct {
    float32_t P0_diag[NAV_STATE_DIM];  // initial state uncertainty (variance)
    // Continuous-time process noise PSD (variance / second).  Discrete-time
    // Q at each predict step is Q_psd * dt, so tuning is independent of IMU
    // rate.  See tests/i2nav_test.c for the datasheet -> Q_psd derivation.
    float32_t Q_psd_diag[NAV_STATE_DIM];
    float32_t R_diag[NAV_GNSS_VEL_DIM];  // GNSS velocity measurement noise
    // GNSS outlier gate: chi-square threshold on d2 = y' S^-1 y (2 DOF).
    // Reject if d2 > gate.  e.g. 9.21 = 99%, 13.82 = 99.9% (looser).
    float32_t gnss_chi2_gate;

    // GNSS antenna position relative to the IMU, in body frame (m).  Used to
    // remove the omega x r rotational velocity so the GNSS measurement is
    // referenced to the IMU.  Leave {0,0,0} to disable lever-arm compensation.
    float32_t lever_arm[3];
    // Measurement variance for the hybrid roll/pitch update (unit gravity-
    // direction components, ~ sin(tilt_error)^2).  Small => trust MTi leveling.
    // Only used when NAV_FeedAttitude is called.
    float32_t att_tilt_var;
} NAV_Config;

// NAV_FeedGNSS_Vel return codes
#define NAV_GNSS_OK 0
#define NAV_GNSS_GATED -1   // innovation failed the chi-square gate
#define NAV_GNSS_FAILED -2  // EKF update returned error (e.g. singular S)

// vel0: NED velocity (m/s), quat0: body-to-NED [w,x,y,z], gravity: (m/s^2)
void NAV_Init(NAV_Context *ctx, float32_t *vel0, float32_t *quat0,
              float32_t gravity, const NAV_Config *cfg);

void NAV_FeedIMU(NAV_Context *ctx, const float32_t *accel,
                 const float32_t *gyro, float32_t dt);

int NAV_FeedGNSS_Vel(NAV_Context *ctx, float32_t vN, float32_t vE);

// Hybrid attitude aid: use the MTi fused quaternion to anchor roll/pitch only.
// The update observes the gravity direction in body frame, which is heading-
// independent, so the MTi's (magnetometer-based) heading is NOT injected --
// heading stays governed by the gyro + GNSS.  Call after NAV_FeedIMU.
// quat_meas: body->NED [w,x,y,z].
void NAV_FeedAttitude(NAV_Context *ctx, const float32_t *quat_meas);

void NAV_GetOutput(NAV_Context *ctx, const float32_t *gyro, NAV_Output *out);

#endif
