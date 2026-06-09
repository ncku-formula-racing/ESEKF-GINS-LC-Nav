#include "ins.h"

#include "utils/utils.h"

void INS_Init(INS_State *s, float32_t *vel0, float32_t *quat0,
              float32_t gravity) {
    for (int i = 0; i < 3; ++i) s->vel[i] = vel0[i];
    for (int i = 0; i < 4; ++i) s->quat[i] = quat0[i];
    s->gravity = gravity;
}

void INS_Propagate(INS_State *s, const float32_t *accel, const float32_t *gyro,
                   float32_t dt) {
    float32_t w = s->quat[0], x = s->quat[1], y = s->quat[2], z = s->quat[3];
    float32_t gx = gyro[0], gy = gyro[1], gz = gyro[2];

    float32_t R[3][3];
    quat_to_R(s->quat, R);

    // Kinematic acceleration: a_ned = R * f_body + g_ned, with the gravity
    // vector pointing down, g_ned = (0, 0, +gravity).  `accel` is specific
    // force (e.g. a stationary level sensor reads ~ -g on the Down axis, so
    // (R*f)[2] ~ -g and a_ned[2] ~ 0).
    float32_t a_ned[3] = {
        R[0][0] * accel[0] + R[0][1] * accel[1] + R[0][2] * accel[2],
        R[1][0] * accel[0] + R[1][1] * accel[1] + R[1][2] * accel[2],
        R[2][0] * accel[0] + R[2][1] * accel[1] + R[2][2] * accel[2]};
    a_ned[2] += s->gravity;

    for (int i = 0; i < 3; ++i) s->vel[i] += a_ned[i] * dt;

    // Quaternion kinematics: dq = 0.5 * Omega(gyro) * q
    float32_t dq[4] = {
        0.5f * (-x * gx - y * gy - z * gz), 0.5f * (w * gx + y * gz - z * gy),
        0.5f * (w * gy - x * gz + z * gx), 0.5f * (w * gz + x * gy - y * gx)};

    float32_t norm = 0;
    for (int i = 0; i < 4; ++i)
        s->quat[i] += dq[i] * dt, norm += s->quat[i] * s->quat[i];
    arm_sqrt_f32(norm, &norm);
    for (int i = 0; i < 4; ++i) s->quat[i] /= norm;
}
