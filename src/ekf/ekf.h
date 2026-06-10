#ifndef EKF_H
#define EKF_H

#include "arm_math.h"

/// Workspace size in float32 elements for an n-state filter with max
/// measurement dimension m.  Pass a buffer of this size as work_mem to
/// EKF_Init.
#define EKF_WORK_SIZE(n, m) \
    (2 * (n) * (n) + 3 * (n) * (m) + 2 * (m) * (m) + (m))

/// Nonlinear state transition x_out = f(x_in, u).
///   x_in:  current state (n x 1)
///   u:     control input, forwarded verbatim from EKF_Predict (may be NULL)
///   x_out: next state (n x 1); must not alias x_in
typedef void (*StateTransitionFunc)(arm_matrix_instance_f32 *x_in,
                                    arm_matrix_instance_f32 *u,
                                    arm_matrix_instance_f32 *x_out);

/// Nonlinear observation h_out = h(x).
///   x:     current state (n x 1)
///   h_out: predicted measurement (m x 1, current measurement dim)
typedef void (*ObservationFunc)(arm_matrix_instance_f32 *x,
                                arm_matrix_instance_f32 *h_out);

/// Generic EKF over caller-owned buffers (no heap).  All matrix instances
/// below are *views* of the pointers handed to EKF_Init -- the context never
/// owns memory.
typedef struct {
    uint16_t n;  // State dimension
    uint16_t m;  // Max measurement dimension (must satisfy m <= n; the
                 // workspace reuse in EKF_Update depends on it)

    // x (n x 1): state estimate
    // P (n x n): state covariance
    // A (n x n): state transition matrix / Jacobian.  Used as x = A*x when
    //            f == NULL, and always as P = A*P*A' + Q.  Refresh it before
    //            each EKF_Predict if the model is time-varying.
    // Q (n x n): process noise covariance, discrete (per predict step)
    // R (m x m): measurement noise covariance.  Stored row-major for the
    //            CURRENT measurement dimension -- when updating with a
    //            smaller m than the previous call, refill R entirely (the
    //            same buffer is reinterpreted as m x m, so stale max-m
    //            layouts read garbage).
    arm_matrix_instance_f32 x, P, A, Q, R;

    // Workspace (allocated from work_mem; internal, contents are scratch)
    arm_matrix_instance_f32 T1, T2, K, S, invS, HT, HP, dy;

    StateTransitionFunc f;  // NULL => linear predict x = A*x
    ObservationFunc h;      // NULL => linear innovation dy = z - H*x
} EKF_Context;

/// Initialize the context as views over caller-owned buffers.
///   n:        state dimension
///   m:        max measurement dimension (m <= n)
///   work_mem: scratch buffer, EKF_WORK_SIZE(n, m) float32 elements
///   x_ptr:    state estimate, n elements (initialize before first predict)
///   P_ptr:    state covariance, n*n elements, row-major
///   A_ptr:    state transition matrix, n*n elements, row-major
///   Q_ptr:    process noise covariance, n*n elements, row-major
///   R_ptr:    measurement noise covariance, m*m elements, row-major
/// f and h start NULL (linear model); assign ctx->f / ctx->h afterwards for
/// nonlinear models.
void EKF_Init(EKF_Context *ctx, uint16_t n, uint16_t m, float32_t *work_mem,
              float32_t *x_ptr, float32_t *P_ptr, float32_t *A_ptr,
              float32_t *Q_ptr, float32_t *R_ptr);

/// Predict step: x = f(x, u) (or A*x when f == NULL), P = A*P*A' + Q.
///   u: control input forwarded to f; ignored when f == NULL (pass NULL)
void EKF_Predict(EKF_Context *ctx, arm_matrix_instance_f32 *u);

/// Update step with a measurement of any dimension up to the max m given at
/// init (internal temporaries are resized in place each call).
///   H: measurement Jacobian (m x n), where m = z->numRows
///   z: measurement vector (m x 1)
/// Returns 0 on success,
///        -1 if S is singular (update skipped),
///        -2 if H or z dimensions are invalid for this context.
int EKF_Update(EKF_Context *ctx, arm_matrix_instance_f32 *H,
               arm_matrix_instance_f32 *z);

#endif
