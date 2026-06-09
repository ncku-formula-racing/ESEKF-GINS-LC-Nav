#ifndef UTILS_H
#define UTILS_H

#include "arm_math.h"

void quat_to_R(const float32_t *q, float32_t R[3][3]);

#ifdef KF_GINS_DEBUG
void print_matrix(const char *matName, arm_matrix_instance_f32 *mat);
#endif

#endif
