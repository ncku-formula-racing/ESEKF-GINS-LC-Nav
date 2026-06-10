/**
 * nav_example.c -- minimal NAV API walkthrough on a synthetic scenario
 *
 * Build & run (from the repo root):
 *   make
 *   ./build/tests/nav_example
 *
 * Scenario: car facing North, at rest for 2 s, accelerates North at
 * 2.5 m/s^2 for 2 s, then cruises at 5 m/s.  IMU 500 Hz, GNSS 10 Hz, with
 * MTi-630 / PX1120S-grade noise injected.
 *
 * Prints CSV to stdout at 10 Hz: time, true vs estimated body velocity,
 * estimated yaw rate, and the persistent z-gyro bias estimate.  Pipe it to
 * a file and plot, e.g.:
 *   ./build/tests/nav_example > out.csv
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "nav/nav.h"

// Q_psd values match the i2nav_test ICM20602-grade tuning so synthetic
// MTi-630 (which is similar grade) is well-served.  Per-second variance
// growth; library scales by dt internally.
static const NAV_Config kNavCfg = {
    .P0_diag = {1.0f, 1.0f, 1.0f,        // dv      (m/s)^2
                0.01f, 0.01f, 0.01f,     // dtheta  (0.1 rad)^2
                1e-4f, 1e-4f, 1e-4f,     // db_a
                1e-6f, 1e-6f, 1e-6f},    // db_g
    .Q_psd_diag = {6e-4f, 6e-4f, 6e-4f,  // dv
                   1.8e-5f, 1.8e-5f, 1.8e-5f, 2e-6f, 2e-6f, 2e-6f, 2e-8f, 2e-8f,
                   2e-6f},
    .R_diag = {0.01f, 0.01f},
    .gnss_chi2_gate = 9.21f,  // chi-square 2 DOF, 99%
};

// Box-Muller Gaussian noise
static float gauss(float std) {
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    return std * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2);
}

int main(void) {
    srand(42);

    NAV_Context ctx;

    // Scenario: car facing North, at rest.  At t=2s it accelerates North at
    // 2.5 m/s^2 for 2 s, then cruises at 5 m/s.
    // Body frame: x=forward(N), y=right(E), z=down -> identity quaternion
    float32_t vel0[3] = {0.0f, 0.0f, 0.0f};
    float32_t quat0[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    NAV_Init(&ctx, vel0, quat0, 9.80665f, &kNavCfg);

    const float dt = 0.002f;    // 500 Hz IMU
    const int gnss_every = 50;  // 10 Hz GNSS
    const int steps = 5000;     // 10 seconds

    // MTi-630 / PX1120S noise specs
    const float accel_noise = 0.013f;  // m/s^2
    const float gyro_noise = 0.0016f;  // rad/s
    const float gnss_noise = 0.1f;     // m/s

    printf("t,true_vx,est_vx,true_vy,est_vy,est_wz,b_gz\n");

    for (int i = 0; i < steps; i++) {
        float t = i * dt;
        float a_true = (t >= 2.0f && t < 4.0f) ? 2.5f : 0.0f;  // m/s^2, North
        float true_vN = (t < 2.0f)   ? 0.0f
                        : (t < 4.0f) ? 2.5f * (t - 2.0f)
                                     : 5.0f;
        const float true_vE = 0.0f;

        // Accelerometer measures specific force: the kinematic acceleration
        // (body x = North while facing North) minus gravity, which for a
        // level sensor is -g on the Down(z) axis.  gyro = 0 (no rotation).
        float32_t accel[3] = {a_true + gauss(accel_noise), gauss(accel_noise),
                              -9.80665f + gauss(accel_noise)};
        float32_t gyro[3] = {gauss(gyro_noise), gauss(gyro_noise),
                             gauss(gyro_noise)};

        NAV_FeedIMU(&ctx, accel, gyro, dt);

        if (i % gnss_every == gnss_every - 1) {
            float32_t vN = true_vN + gauss(gnss_noise);
            float32_t vE = true_vE + gauss(gnss_noise);
            NAV_FeedGNSS_Vel(&ctx, vN, vE);
        }

        // Log at 10 Hz
        if (i % gnss_every == 0) {
            NAV_Output out;
            NAV_GetOutput(&ctx, gyro, &out);

            // Facing North: body vx = vN, body vy = vE
            // bias_g[2] is the persistent b_gz estimate (x_buf[SV_BG+2] is
            // db_gz which gets zeroed each GNSS update).
            printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.6f\n", t, true_vN, out.vx,
                   true_vE, out.vy, out.wz, ctx.bias_g[2]);
        }
    }

    return 0;
}
