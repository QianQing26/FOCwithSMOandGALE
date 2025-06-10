/*
 * matrix.h
 *
 *  Created on: Jun 4, 2025
 *      Author: 79489
 */

#ifndef APPLICATION_USER_MATRIX_H_
#define APPLICATION_USER_MATRIX_H_

void matrix_multiply_2x2(float A[2][2], float B[2][2], float C[2][2]);
void matrix_multiply_2x1(float A[2][2], float B[2], float C[2]);
void matrix_multiply_1x2(float A[1][2], float B[2][2], float C[1][2]);
void matrix_multiply_1x1(float A[1][2], float B[2][1], float C[1][1]);
//void matrix_transpose_2x1(float A[2], float At[1][2]);
void matrix_inverse_1x1(float A[1][1], float A_inv[1][1]);
void matrix_add_2x2(float A[2][2], float B[2][2], float C[2][2]);
void matrix_sub_2x2(float A[2][2], float B[2][2], float C[2][2]);
void matrix_copy_2x2(float src[2][2], float dst[2][2]);


#endif /* APPLICATION_USER_MATRIX_H_ */
