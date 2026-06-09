#include "nav.h"

#include <string.h>

#include "utils/utils.h"

// NED-frame phi-angle ESKF
//
// Convention everywhere: NED-frame ("global") angular error.
//   q_truth = dq * q_estimate,  with dq ~ (1, dtheta_NED/2)   (LEFT-multiply)
//
// State vector (12-dim error, fully reset after every GNSS update):
//   x = [dv(NED), dtheta(NED), db_a(body), db_g(body)]
//
//   d/dt(dv)     = -skew(R * f_compensated) * dtheta  -  R * db_a
//   d/dt(dtheta) =                                    -  R * db_g
//   db_a, db_g  -- random walk (Q-driven only)
//
//   where f_compensated = a_meas - bias_a,  w_compensated = w_meas - bias_g.
//
// NED-frame dtheta does NOT pick up a -skew(w)*dtheta kinematic term: when
// nominal and truth share the same w, the rotation discrepancy R_truth*R^-1
// stays constant in NED.  The kinematic term only appears in body-local
// convention.

static void build_Phi(NAV_Context *ctx, const float32_t *accel,
                      const float32_t *gyro, float32_t dt) {
    float32_t R[3][3];
    quat_to_R(ctx->ins.quat, R);

    float32_t *A = ctx->A_buf;
    int n = NAV_STATE_DIM;
    memset(A, 0, n * n * sizeof(float32_t));
    for (int i = 0; i < n; i++) A[i * n + i] = 1.0f;

    // F[dv, dtheta] = -skew(R * f_body)   (NED-frame phi-angle form)
    float32_t Rf[3] = {0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) Rf[i] += R[i][j] * accel[j];
    A[(SV_DV + 0) * n + (SV_DPSI + 1)] = Rf[2] * dt;
    A[(SV_DV + 0) * n + (SV_DPSI + 2)] = -Rf[1] * dt;
    A[(SV_DV + 1) * n + (SV_DPSI + 0)] = -Rf[2] * dt;
    A[(SV_DV + 1) * n + (SV_DPSI + 2)] = Rf[0] * dt;
    A[(SV_DV + 2) * n + (SV_DPSI + 0)] = Rf[1] * dt;
    A[(SV_DV + 2) * n + (SV_DPSI + 1)] = -Rf[0] * dt;

    // F[dv, db_a] = -R
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            A[(SV_DV + i) * n + (SV_BA + j)] = -R[i][j] * dt;

    // F[dtheta_NED, dtheta_NED] = 0  (no kinematic coupling in NED convention)
    (void)gyro;  // gyro only enters via db_g coupling below

    // F[dtheta_NED, db_g_body] = -R   (rotates body-frame bias error to NED)
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            A[(SV_DPSI + i) * n + (SV_BG + j)] = -R[i][j] * dt;
}

// Inject error state into nominal, then full reset of x_buf.
// All four error blocks (dv, dtheta, db_a, db_g) are reset; their *covariances*
// stay in P (they represent uncertainty, not the corrected estimate itself).
static void inject_and_reset(NAV_Context *ctx) {
    float32_t *x = ctx->x_buf;

    // dv -> vel
    for (int i = 0; i < 3; i++) ctx->ins.vel[i] += x[SV_DV + i];

    // dtheta_NED -> quat  (left-multiply: q_new = (1, dtheta_NED/2) * q_old)
    float32_t *q = ctx->ins.quat;
    float32_t hx = x[SV_DPSI + 0] * 0.5f;
    float32_t hy = x[SV_DPSI + 1] * 0.5f;
    float32_t hz = x[SV_DPSI + 2] * 0.5f;
    float32_t qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    // Hamilton product (1, hx, hy, hz) (X) (qw, qx, qy, qz)
    float32_t nw = qw - hx * qx - hy * qy - hz * qz;
    float32_t nx = qx + hx * qw + hy * qz - hz * qy;
    float32_t ny = qy + hy * qw + hz * qx - hx * qz;
    float32_t nz = qz + hz * qw + hx * qy - hy * qx;
    float32_t norm;
    arm_sqrt_f32(nw * nw + nx * nx + ny * ny + nz * nz, &norm);
    q[0] = nw / norm;
    q[1] = nx / norm;
    q[2] = ny / norm;
    q[3] = nz / norm;

    // db -> nominal bias estimates
    for (int i = 0; i < 3; i++) {
        ctx->bias_a[i] += x[SV_BA + i];
        ctx->bias_g[i] += x[SV_BG + i];
    }

    // Full reset
    memset(x, 0, NAV_STATE_DIM * sizeof(float32_t));
}

static void fill_diag(float32_t *mat, int n, const float32_t *diag) {
    memset(mat, 0, n * n * sizeof(float32_t));
    for (int i = 0; i < n; i++) mat[i * n + i] = diag[i];
}

void NAV_Init(NAV_Context *ctx, float32_t *vel0, float32_t *quat0,
              float32_t gravity, const NAV_Config *cfg) {
    INS_Init(&ctx->ins, vel0, quat0, gravity);
    memset(ctx->x_buf, 0, sizeof(ctx->x_buf));
    memset(ctx->bias_a, 0, sizeof(ctx->bias_a));
    memset(ctx->bias_g, 0, sizeof(ctx->bias_g));
    memset(ctx->last_w, 0, sizeof(ctx->last_w));

    // Stash the continuous-time PSD; Q_buf is rebuilt as Q_psd * dt each step.
    memcpy(ctx->Q_psd, cfg->Q_psd_diag, sizeof(ctx->Q_psd));
    ctx->chi2_gate = cfg->gnss_chi2_gate;

    // Measurement noises (R_buf is filled per update -- GNSS and attitude use
    // different dimensions) and lever arm.
    memcpy(ctx->gnss_R, cfg->R_diag, sizeof(ctx->gnss_R));
    ctx->att_R = cfg->att_tilt_var;
    memcpy(ctx->lever_arm, cfg->lever_arm, sizeof(ctx->lever_arm));

    fill_diag(ctx->P_buf, NAV_STATE_DIM, cfg->P0_diag);
    memset(ctx->Q_buf, 0, sizeof(ctx->Q_buf));  // populated per-step
    memset(ctx->R_buf, 0, sizeof(ctx->R_buf));  // populated per update

    // A starts as identity; rebuilt by build_Phi each IMU step
    memset(ctx->A_buf, 0, sizeof(ctx->A_buf));
    for (int i = 0; i < NAV_STATE_DIM; i++)
        ctx->A_buf[i * NAV_STATE_DIM + i] = 1.0f;

    EKF_Init(&ctx->ekf, NAV_STATE_DIM, NAV_MAX_MEAS_DIM, ctx->work_buf,
             ctx->x_buf, ctx->P_buf, ctx->A_buf, ctx->Q_buf, ctx->R_buf);
}

void NAV_FeedIMU(NAV_Context *ctx, const float32_t *accel,
                 const float32_t *gyro, float32_t dt) {
    // Compensate raw IMU with the running bias estimate, then propagate.
    float32_t a_corr[3], w_corr[3];
    for (int i = 0; i < 3; ++i) {
        a_corr[i] = accel[i] - ctx->bias_a[i];
        w_corr[i] = gyro[i] - ctx->bias_g[i];
    }
    memcpy(ctx->last_w, w_corr, sizeof(w_corr));  // cached for GNSS lever-arm
    INS_Propagate(&ctx->ins, a_corr, w_corr, dt);
    build_Phi(ctx, a_corr, w_corr, dt);

    // Discrete-time Q for this step:  Q_d = Q_psd * dt.  Rebuilding here keeps
    // covariance growth rate (per-second) constant regardless of IMU rate.
    for (int i = 0; i < NAV_STATE_DIM; i++)
        ctx->Q_buf[i * NAV_STATE_DIM + i] = ctx->Q_psd[i] * dt;

    EKF_Predict(&ctx->ekf, NULL);
}

// H (2x12): observe NED horizontal velocity (vN, vE) -> dv[0], dv[1].
static float32_t H_data[2 * NAV_STATE_DIM] = {
    [0 * NAV_STATE_DIM + SV_DV + 0] = 1.0f,
    [1 * NAV_STATE_DIM + SV_DV + 1] = 1.0f,
};

// Reject GNSS outliers with a chi-square (Mahalanobis) test on the innovation:
// d2 = y' S^-1 y, S = H P H^T + R.  d2 ~ chi-square(2 DOF) when the filter is
// consistent, so the gate scales with the current uncertainty (loose right
// after init when P is large, tight once converged).  Catches multipath /
// re-lock glitches.
int NAV_FeedGNSS_Vel(NAV_Context *ctx, float32_t vN, float32_t vE) {
    // Lever-arm: GNSS measures the antenna velocity = v_imu + R * (w x r).
    // Remove the rotational term so the measurement references the IMU.
    float32_t R[3][3];
    quat_to_R(ctx->ins.quat, R);
    const float32_t *w = ctx->last_w, *r = ctx->lever_arm;
    float32_t wr[3] = {w[1] * r[2] - w[2] * r[1], w[2] * r[0] - w[0] * r[2],
                       w[0] * r[1] - w[1] * r[0]};
    float32_t lv_n = R[0][0] * wr[0] + R[0][1] * wr[1] + R[0][2] * wr[2];
    float32_t lv_e = R[1][0] * wr[0] + R[1][1] * wr[1] + R[1][2] * wr[2];

    float32_t innov_n = (vN - lv_n) - ctx->ins.vel[0];
    float32_t innov_e = (vE - lv_e) - ctx->ins.vel[1];

    // Chi-square gate.  H selects dv_N, dv_E, so H P H^T is the top-left 2x2
    // block of P; S = that + R.  d2 = y' S^-1 y with the closed-form 2x2
    // inverse.
    const float32_t *P = ctx->P_buf;
    int n = NAV_STATE_DIM;
    float32_t s00 = P[0 * n + 0] + ctx->gnss_R[0];
    float32_t s01 = P[0 * n + 1];
    float32_t s11 = P[1 * n + 1] + ctx->gnss_R[1];
    float32_t det = s00 * s11 - s01 * s01;
    float32_t d2 = (s11 * innov_n * innov_n - 2.0f * s01 * innov_n * innov_e +
                    s00 * innov_e * innov_e) /
                   det;
    if (det <= 0.0f || d2 > ctx->chi2_gate) {
        return NAV_GNSS_GATED;
    }

    float32_t z_data[NAV_GNSS_VEL_DIM] = {innov_n, innov_e};
    arm_matrix_instance_f32 z;
    arm_mat_init_f32(&z, NAV_GNSS_VEL_DIM, 1, z_data);

    arm_matrix_instance_f32 H;
    arm_mat_init_f32(&H, NAV_GNSS_VEL_DIM, NAV_STATE_DIM, H_data);

    fill_diag(ctx->R_buf, NAV_GNSS_VEL_DIM, ctx->gnss_R);
    if (EKF_Update(&ctx->ekf, &H, &z) != 0) return NAV_GNSS_FAILED;
    inject_and_reset(ctx);
    return NAV_GNSS_OK;
}

// Hybrid roll/pitch aid.  Observe the gravity direction in body frame,
// g_body = R^T * (0,0,1) = the 3rd row of R.  This is independent of heading,
// so the MTi's magnetometer heading is never injected (the dtheta_Down column
// of H is zero by construction); heading stays from gyro + GNSS.
void NAV_FeedAttitude(NAV_Context *ctx, const float32_t *quat_meas) {
    float32_t Rm[3][3], Ri[3][3];
    quat_to_R(quat_meas, Rm);
    quat_to_R(ctx->ins.quat, Ri);

    // Innovation = g_body(meas) - g_body(nominal).
    float32_t z_data[NAV_ATT_DIM] = {Rm[2][0] - Ri[2][0], Rm[2][1] - Ri[2][1],
                                     Rm[2][2] - Ri[2][2]};

    // H (3x12): d(g_body)/d(dtheta_NED) row i = (Ri[1][i], -Ri[0][i], 0).
    float32_t H_buf[NAV_ATT_DIM * NAV_STATE_DIM];
    memset(H_buf, 0, sizeof(H_buf));
    for (int i = 0; i < NAV_ATT_DIM; i++) {
        H_buf[i * NAV_STATE_DIM + SV_DPSI + 0] = Ri[1][i];
        H_buf[i * NAV_STATE_DIM + SV_DPSI + 1] = -Ri[0][i];
        // SV_DPSI + 2 (heading) stays 0 -> heading is not observed here.
    }

    arm_matrix_instance_f32 z, H;
    arm_mat_init_f32(&z, NAV_ATT_DIM, 1, z_data);
    arm_mat_init_f32(&H, NAV_ATT_DIM, NAV_STATE_DIM, H_buf);

    float32_t att_diag[NAV_ATT_DIM] = {ctx->att_R, ctx->att_R, ctx->att_R};
    fill_diag(ctx->R_buf, NAV_ATT_DIM, att_diag);

    if (EKF_Update(&ctx->ekf, &H, &z) == 0) inject_and_reset(ctx);
}

void NAV_GetOutput(NAV_Context *ctx, const float32_t *gyro, NAV_Output *out) {
    float32_t R[3][3];
    quat_to_R(ctx->ins.quat, R);

    // v_body = R^T * v_NED  (R is body->NED; transpose flips direction)
    float32_t *v = ctx->ins.vel;
    out->vx = R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2];
    out->vy = R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2];

    // Angular rate with gyro bias removed -- uses the persistent estimate.
    out->wx = gyro[0] - ctx->bias_g[0];
    out->wy = gyro[1] - ctx->bias_g[1];
    out->wz = gyro[2] - ctx->bias_g[2];
}
