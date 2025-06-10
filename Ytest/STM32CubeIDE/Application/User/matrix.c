/*
 * matrix.c
 *
 *  Created on: Jun 4, 2025
 *      Author: 79489
 */

#include "matrix.h"
#include<string.h>

#include<math.h>

void matrix_multiply_2x2(float A[2][2], float B[2][2], float C[2][2]);
void matrix_multiply_2x1(float A[2][2], float B[2], float C[2]);
void matrix_multiply_1x2(float A[1][2], float B[2][2], float C[1][2]);
//void matrix_transpose_2x1(float A[2], float At[1][2]);
void matrix_inverse_1x1(float A[1][1], float A_inv[1][1]);
void matrix_add_2x2(float A[2][2], float B[2][2], float C[2][2]);
void matrix_sub_2x2(float A[2][2], float B[2][2], float C[2][2]);
void matrix_copy_2x2(float src[2][2], float dst[2][2]);


// 矩阵乘法：2x2 * 2x2 = 2x2
void matrix_multiply_2x2(float A[2][2], float B[2][2], float C[2][2]){
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
}

// 矩阵乘法：2x2 * 2x1 = 2x1
void matrix_multiply_2x1(float A[2][2], float B[2], float C[2]){
    C[0] = A[0][0] * B[0] + A[0][1] * B[1];
    C[1] = A[1][0] * B[0] + A[1][1] * B[1];
}

// 矩阵乘法：1x2 * 2x2 = 1x2
void matrix_multiply_1x2(float A[1][2], float B[2][2], float C[1][2]){
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
}

// 矩阵乘法：1x2 * 2x1 = 1x1（标量）
void matrix_multiply_1x1(float A[1][2], float B[2][1], float C[1][1]){
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
}

// 1x1矩阵求逆
void matrix_inverse_1x1(float A[1][1], float A_inv[1][1]){
    if (fabsf(A[0][0]) < 1e-10f) {
        A_inv[0][0] = 1e10f;  // 避免除以零
    } else {
        A_inv[0][0] = 1.0f / A[0][0];
    }
}

// 矩阵加法：2x2 + 2x2 = 2x2
void matrix_add_2x2(float A[2][2], float B[2][2], float C[2][2]){
    C[0][0] = A[0][0] + B[0][0];
    C[0][1] = A[0][1] + B[0][1];
    C[1][0] = A[1][0] + B[1][0];
    C[1][1] = A[1][1] + B[1][1];
}

// 矩阵减法：2x2 - 2x2 = 2x2
void matrix_sub_2x2(float A[2][2], float B[2][2], float C[2][2]){
    C[0][0] = A[0][0] - B[0][0];
    C[0][1] = A[0][1] - B[0][1];
    C[1][0] = A[1][0] - B[1][0];
    C[1][1] = A[1][1] - B[1][1];
}

// 矩阵复制：2x2
void matrix_copy_2x2(float src[2][2], float dst[2][2]){
    memcpy(dst, src, 4 * sizeof(float));
}
