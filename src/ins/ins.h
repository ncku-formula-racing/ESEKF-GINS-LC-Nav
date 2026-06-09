#ifndef INS_H
#define INS_H

#include "arm_math.h"

typedef struct {
    float32_t vel[3];   // NED velocity (m/s)
    float32_t quat[4];  // body to NED [w,x,y,z]
    float32_t gravity;
} INS_State;

void INS_Init(INS_State *s, float32_t *vel0, float32_t *quat0,
              float32_t gravity);
void INS_Propagate(INS_State *s, const float32_t *accel, const float32_t *gyro,
                   float32_t dt);

#endif
