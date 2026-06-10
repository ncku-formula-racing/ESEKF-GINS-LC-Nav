#ifndef UTILS_H
#define UTILS_H

#include "arm_math.h"

/// Unit quaternion -> rotation matrix.
///   q: [w,x,y,z], unit norm.  For a body-to-NED quaternion the result R
///      satisfies v_ned = R * v_body (and v_body = R' * v_ned).
void quat_to_R(const float32_t *q, float32_t R[3][3]);

#ifdef KF_GINS_DEBUG
void print_matrix(const char *matName, arm_matrix_instance_f32 *mat);
#endif

#endif
