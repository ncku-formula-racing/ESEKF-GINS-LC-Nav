#include "ekf.h"

#include <string.h>

#include "dsp/matrix_functions.h"

void EKF_Init(EKF_Context *ctx, uint16_t n, uint16_t m, float32_t *work_mem,
              float32_t *x_ptr, float32_t *P_ptr, float32_t *A_ptr,
              float32_t *Q_ptr, float32_t *R_ptr) {
    ctx->n = n;
    ctx->m = m;

    arm_mat_init_f32(&ctx->x, n, 1, x_ptr);
    arm_mat_init_f32(&ctx->P, n, n, P_ptr);
    arm_mat_init_f32(&ctx->A, n, n, A_ptr);
    arm_mat_init_f32(&ctx->Q, n, n, Q_ptr);
    arm_mat_init_f32(&ctx->R, m, m, R_ptr);

    float32_t *p = work_mem;
    arm_mat_init_f32(&ctx->T1, n, n, p);
    p += n * n;
    arm_mat_init_f32(&ctx->T2, n, n, p);
    p += n * n;
    arm_mat_init_f32(&ctx->K, n, m, p);
    p += n * m;
    arm_mat_init_f32(&ctx->S, m, m, p);
    p += m * m;
    arm_mat_init_f32(&ctx->invS, m, m, p);
    p += m * m;
    arm_mat_init_f32(&ctx->HT, n, m, p);
    p += n * m;
    arm_mat_init_f32(&ctx->HP, m, n, p);
    p += m * n;
    arm_mat_init_f32(&ctx->dy, m, 1, p);

    ctx->f = NULL;
    ctx->h = NULL;
}

void EKF_Predict(EKF_Context *ctx, arm_matrix_instance_f32 *u) {
    // x = f(x, u) or x = A * x
    if (ctx->f) {
        arm_mat_init_f32(&ctx->T1, ctx->n, 1, ctx->T1.pData);
        ctx->f(&ctx->x, u, &ctx->T1);
        memcpy(ctx->x.pData, ctx->T1.pData, ctx->n * sizeof(float32_t));
        arm_mat_init_f32(&ctx->T1, ctx->n, ctx->n, ctx->T1.pData);
    } else {
        arm_mat_init_f32(&ctx->T1, ctx->n, 1, ctx->T1.pData);
        arm_mat_mult_f32(&ctx->A, &ctx->x, &ctx->T1);
        memcpy(ctx->x.pData, ctx->T1.pData, ctx->n * sizeof(float32_t));
        arm_mat_init_f32(&ctx->T1, ctx->n, ctx->n, ctx->T1.pData);
    }

    // P = A * P * A' + Q
    arm_mat_trans_f32(&ctx->A, &ctx->T1);          // T1 = A'
    arm_mat_mult_f32(&ctx->A, &ctx->P, &ctx->T2);  // T2 = A * P
    arm_mat_mult_f32(&ctx->T2, &ctx->T1, &ctx->P);
    arm_mat_add_f32(&ctx->P, &ctx->Q, &ctx->P);
}

int EKF_Update(EKF_Context *ctx, arm_matrix_instance_f32 *H,
               arm_matrix_instance_f32 *z) {
    uint16_t m = z->numRows;
    uint16_t n = ctx->n;

    // Bounds: caller must respect the max measurement dim baked into work_mem.
    if (m == 0 || m > ctx->m || z->numCols != 1 || H->numRows != m ||
        H->numCols != n) {
        return -2;
    }

    // Resize variable-dimension matrices for current measurement
    ctx->R.numRows = ctx->R.numCols = m;
    ctx->S.numRows = ctx->S.numCols = m;
    ctx->invS.numRows = ctx->invS.numCols = m;
    ctx->K.numCols = ctx->HT.numCols = ctx->HP.numRows = m;
    ctx->dy.numRows = m;

    // dy = z - h(x) or z - H*x
    if (ctx->h) {
        ctx->h(&ctx->x, &ctx->dy);
    } else {
        arm_mat_mult_f32(H, &ctx->x, &ctx->dy);
    }
    arm_mat_sub_f32(z, &ctx->dy, &ctx->dy);

    // S = H*P*H' + R
    arm_mat_trans_f32(H, &ctx->HT);
    arm_mat_mult_f32(H, &ctx->P, &ctx->HP);
    arm_mat_mult_f32(&ctx->HP, &ctx->HT, &ctx->S);
    arm_mat_add_f32(&ctx->S, &ctx->R, &ctx->S);

    if (arm_mat_inverse_f32(&ctx->S, &ctx->invS) != ARM_MATH_SUCCESS) return -1;

    // K = P*H' * S^-1  (T1 as temp to avoid in-place write to K)
    arm_mat_init_f32(&ctx->T1, n, m, ctx->T1.pData);
    arm_mat_mult_f32(&ctx->P, &ctx->HT, &ctx->T1);
    arm_mat_mult_f32(&ctx->T1, &ctx->invS, &ctx->K);

    // x = x + K * dy
    arm_mat_init_f32(&ctx->T1, n, 1, ctx->T1.pData);
    arm_mat_mult_f32(&ctx->K, &ctx->dy, &ctx->T1);
    arm_mat_add_f32(&ctx->x, &ctx->T1, &ctx->x);

    // P = (I-KH)*P*(I-KH)' + K*R*K' (Joseph form)
    // Instead of using P = (I-KH)P to avoid flaot32 rounding causing
    // covariances to lose symmetry and positive definiteness
    arm_mat_init_f32(&ctx->T1, n, n, ctx->T1.pData);
    arm_mat_mult_f32(&ctx->K, H, &ctx->T1);  // T1 = KH
    arm_mat_scale_f32(&ctx->T1, -1.0f, &ctx->T1);
    for (int i = 0; i < n; i++)
        ctx->T1.pData[i * n + i] += 1.0f;  // T1 = I - KH

    arm_mat_mult_f32(&ctx->T1, &ctx->P, &ctx->T2);  // T2 = (I-KH)*P
    arm_mat_trans_f32(&ctx->T1, &ctx->P);           // P  = (I-KH)' (temp)
    arm_mat_mult_f32(&ctx->T2, &ctx->P, &ctx->T1);  // T1 = (I-KH)*P*(I-KH)'

    arm_mat_mult_f32(&ctx->K, &ctx->R, &ctx->HT);    // HT = K*R   (n*m)
    arm_mat_trans_f32(&ctx->K, &ctx->HP);            // HP = K'    (m*n)
    arm_mat_mult_f32(&ctx->HT, &ctx->HP, &ctx->T2);  // T2 = K*R*K'(n*n)

    arm_mat_add_f32(&ctx->T1, &ctx->T2, &ctx->P);  // P  = result

    return 0;
}
