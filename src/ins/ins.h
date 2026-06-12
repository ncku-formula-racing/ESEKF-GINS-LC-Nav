#ifndef INS_H
#define INS_H

#include "arm_math.h"

typedef struct {
    float32_t vel[3];   // NED velocity (m/s)
    float32_t quat[4];  // body to NED [w,x,y,z]
    float32_t gravity;  // local gravity magnitude (m/s^2, positive down)
} INS_State;

/// vel0:    initial NED velocity (m/s), 3 elements
/// quat0:   initial body-to-NED quaternion [w,x,y,z], 4 elements, unit norm
/// gravity: local gravity magnitude (m/s^2), e.g. 9.81
void INS_Init(INS_State *s, const float32_t *vel0, const float32_t *quat0,
              float32_t gravity);

/// Integrate one IMU sample into the nominal state (first-order Euler).
///   accel: specific force in body frame (m/s^2), 3 elements, already
///          bias-compensated (a stationary level sensor reads ~ -g on Down)
///   gyro:  angular rate in body frame (rad/s), 3 elements, already
///          bias-compensated
///   dt:    time step (s)
void INS_Propagate(INS_State *s, const float32_t *accel, const float32_t *gyro,
                   float32_t dt);

#endif
