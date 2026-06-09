#ifndef EKF_H
#define EKF_H

#include "arm_math.h"

#define EKF_WORK_SIZE(n, m) \
    (2 * (n) * (n) + 3 * (n) * (m) + 2 * (m) * (m) + (m))

/// f(x_in, u) = x_out
typedef void (*StateTransitionFunc)(arm_matrix_instance_f32 *x_in,
                                    arm_matrix_instance_f32 *u,
                                    arm_matrix_instance_f32 *x_out);

/// h(x) = h_out
typedef void (*ObservationFunc)(arm_matrix_instance_f32 *x,
                                arm_matrix_instance_f32 *h_out);

typedef struct {
    uint16_t n;  // State dimension
    uint16_t m;  // Max measurement dimension

    arm_matrix_instance_f32 x, P, A, Q, R;

    // Workspace (allocated from work_mem)
    arm_matrix_instance_f32 T1, T2, K, S, invS, HT, HP, dy;

    StateTransitionFunc f;
    ObservationFunc h;
} EKF_Context;

/// n: State dimension
/// m: Max measurement dimension
/// work_mem size should be EKF_WORK_SIZE(n, m)
void EKF_Init(EKF_Context *ctx, uint16_t n, uint16_t m, float32_t *work_mem,
              float32_t *x_ptr, float32_t *P_ptr, float32_t *A_ptr,
              float32_t *Q_ptr, float32_t *R_ptr);

void EKF_Predict(EKF_Context *ctx, arm_matrix_instance_f32 *u);

/// Returns 0 on success,
///        -1 if S is singular (update skipped),
///        -2 if H or z dimensions are invalid for this context.
int EKF_Update(EKF_Context *ctx, arm_matrix_instance_f32 *H,
               arm_matrix_instance_f32 *z);

#endif
