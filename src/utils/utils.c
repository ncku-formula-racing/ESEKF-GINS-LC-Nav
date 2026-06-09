#include "utils.h"

void quat_to_R(const float32_t *q, float32_t R[3][3]) {
    float32_t w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y * y + z * z);
    R[0][1] = 2 * (x * y - w * z);
    R[0][2] = 2 * (x * z + w * y);
    R[1][0] = 2 * (x * y + w * z);
    R[1][1] = 1 - 2 * (x * x + z * z);
    R[1][2] = 2 * (y * z - w * x);
    R[2][0] = 2 * (x * z - w * y);
    R[2][1] = 2 * (y * z + w * x);
    R[2][2] = 1 - 2 * (x * x + y * y);
}

#ifdef KF_GINS_DEBUG
#include <stdio.h>

void print_matrix(const char *matName, arm_matrix_instance_f32 *mat) {
    printf("Matrix: %s (%dx%d)\n", matName, mat->numRows, mat->numCols);
    for (int i = 0; i < mat->numRows; i++) {
        for (int j = 0; j < mat->numCols; j++) {
            printf("%10.4f ", mat->pData[i * mat->numCols + j]);
        }
        printf("\n");
    }
    printf("\n");
}
#endif
