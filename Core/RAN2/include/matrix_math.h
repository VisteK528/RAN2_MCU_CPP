#ifndef LEARNINGC_MATRIX_MATH_H
#define LEARNINGC_MATRIX_MATH_H

#include <stdint.h>
#include <math.h>

typedef enum{
    MATRIX_SUCCESS,
    MATRIX_ERROR,
    MATRIX_WRONG_MATRIX_DIMENSIONS
} status;

typedef struct{
    uint16_t num_cols;
    uint16_t num_rows;
    float* p_data;
} matrix_f32;

// Initialization functions
void matrix_init_f32(matrix_f32* mat, uint16_t num_cols, uint16_t num_rows, float* p_data);

// Math functions
status lu_decomposition(matrix_f32* src, matrix_f32* lt, matrix_f32* ut, matrix_f32* p);
status inverse_matrix(matrix_f32* src, matrix_f32* dst);
status matrix_mult(matrix_f32* a, matrix_f32* b, matrix_f32* dst);

#endif //LEARNINGC_MATRIX_MATH_H