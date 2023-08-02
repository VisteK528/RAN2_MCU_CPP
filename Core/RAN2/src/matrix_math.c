#include "../include/matrix_math.h"

static int MAX_COL(matrix_f32* matrix, int start_row, int column){
    int max_index = start_row;

    float max_value = fabsf(matrix->p_data[start_row*matrix->num_cols+column]);

    for(int i = start_row; i < matrix->num_rows; i++){
        if(fabsf(matrix->p_data[i*matrix->num_cols+column]) > max_value){
            max_index = i;
        }
    }
    return max_index;
}

/// Zamiana A z B, A idzie na miejsce B
static void SWAP_ROWS(matrix_f32* matrix, int row_a, int row_b){
    float buffer;

    for(int i = 0; i < matrix->num_cols; i++){
        buffer = matrix->p_data[row_b*matrix->num_cols+i];
        matrix->p_data[row_b*matrix->num_cols+i] = matrix->p_data[row_a*matrix->num_cols+i];
        matrix->p_data[row_a*matrix->num_cols+i] = buffer;
    }
}


static void INVERSE_TRIANGULAR(matrix_f32* src, matrix_f32* inv, uint8_t upper){
    /// Dodać jeszcze obejście algorytmu odwracania, jeżeli mamy na diagonali same 1
    /// Wtedy odwrotna macierz to po prostu macierz ze zmienionymi znakami

    float coeff;

    uint16_t rows = src->num_rows;
    uint16_t columns = src->num_cols;

    for(int i = 0; i < rows; i++){
        for(int j = 0; j < columns; j++){
            if(i == j){
                inv->p_data[i*rows+j] = 1;
            }
            else{
                inv->p_data[i*rows+j] = 0;
            }
        }
    }

    if(upper == 0){
        for(int i = 0; i < columns; i++){
            for(int j = i; j < rows; j++){
                if(i == j){
                    coeff = 1/src->p_data[i*columns+j];
                }
                else{
                    coeff = -src->p_data[j*columns+i];
                }

                for(int z = 0; z < columns; z++){
                    if(i == j){
                        src->p_data[j*columns+z] = coeff*src->p_data[i*columns+z];
                        inv->p_data[j*columns+z] = coeff*inv->p_data[i*columns+z];
                    }
                    else{
                        src->p_data[j*columns+z] = src->p_data[j*columns+z] + coeff*src->p_data[i*columns+z];
                        inv->p_data[j*columns+z] = inv->p_data[j*columns+z] + coeff*inv->p_data[i*columns+z];
                    }
                }
            }

        }
    }
    else{
        for(int i = rows-1; i >= 0; i--){
            for(int j = i; j >= 0; j--){
                if(i == j){
                    coeff = 1/src->p_data[i*columns+j];
                }
                else{
                    coeff = -src->p_data[j*columns+i];
                }

                for(int z = 0; z < columns; z++){
                    if(i == j){
                        src->p_data[j*columns+z] = coeff*src->p_data[i*columns+z];
                        inv->p_data[j*columns+z] = coeff*inv->p_data[i*columns+z];
                    }
                    else{
                        src->p_data[j*columns+z] = src->p_data[j*columns+z] + coeff*src->p_data[i*columns+z];
                        inv->p_data[j*columns+z] = inv->p_data[j*columns+z] + coeff*inv->p_data[i*columns+z];
                    }
                }
            }

        }
    }
}

void matrix_init_f32(matrix_f32* mat, uint16_t num_cols, uint16_t num_rows, float* p_data){
    mat->num_cols = num_cols;
    mat->num_rows = num_rows;
    mat->p_data = p_data;
}

status lu_decomposition(matrix_f32* src, matrix_f32* lt, matrix_f32* ut, matrix_f32* p){
    uint16_t rows = src->num_rows;
    uint16_t columns = src->num_cols;
    float l;
    int max;

    // Wypełnienie macierzy lt i ut zeramy oraz macierzy lt 1 na przekątnej
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < rows; j++){
            ut->p_data[i*rows+j] = 0;
            lt->p_data[i*rows+j] = 0;
            if(i == j){
                p->p_data[i*rows+j] = 1;
            }
            else{
                p->p_data[i*rows+j] = 0;
            }
        }
    }

    for(int i = 0; i < columns; i++){
        // Wybór elementu głównego
        max = MAX_COL(src, i, i);

        if(max != i){
            SWAP_ROWS(src, i, max);
            SWAP_ROWS(p, i, max);
            SWAP_ROWS(lt, i, max);
        }

        // Zamiana wierszy
        for(int j = i; j < (rows-1); j++){
            // Obliczenie l
            l = src->p_data[(j+1)*columns+i]/src->p_data[i*columns+i];

            for(int z = 0; z < columns; z++){
                src->p_data[(j+1)*columns+z] = src->p_data[(j+1)*columns+z] - l*src->p_data[i*columns+z];
            }

            // Przypisanie wartości l do dolnej macierzy trójkątnej
            lt->p_data[(j+1)*columns+i] = l;
        }
    }

    // Wypełnienie macierzy lt i ut zeramy oraz macierzy lt 1 na przekątnej
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < rows; j++){
            ut->p_data[i*rows+j] = 0;
            if(i == j){
                lt->p_data[i*rows+j] = 1;
            }
        }
    }

    return MATRIX_SUCCESS;
}

status inverse_matrix(matrix_f32* src, matrix_f32* dst){
    uint16_t rows = src->num_rows;
    uint16_t columns = src->num_cols;

    if(rows != columns){
        // Aby macierz była odwracalna musi być macierzą kwadratową
        return MATRIX_WRONG_MATRIX_DIMENSIONS;
    }
    else if(rows != dst->num_rows || columns != dst->num_cols){
        // Macierz src i dst muszą mieć  takie same wymiary
        return MATRIX_WRONG_MATRIX_DIMENSIONS;
    }

    // Wypełnienie macierzy DST jako jednostkowej
    for(uint16_t i = 0; i < rows; i++){
        for(uint16_t j = 0; j < rows; j++){
            if(i == j){
                dst->p_data[i*columns+j] = 1;
            }
            else{
                dst->p_data[i*columns+j] = 0;
            }
        }
    }

    float lt_data[rows*columns];
    float ut_data[rows*columns];
    float buffer_data[rows*columns];
    float p_data[rows*columns];
    float inv_lt_data[rows*columns];
    float inv_ut_data[rows*columns];

    matrix_f32 LT, UT, P, buffer, INV_LT, INV_UT;

    matrix_init_f32(&LT, rows, columns, lt_data);
    matrix_init_f32(&UT, rows, columns, ut_data);
    matrix_init_f32(&P, rows, columns, p_data);
    matrix_init_f32(&buffer, rows, columns, buffer_data);
    matrix_init_f32(&INV_LT, rows, columns, inv_lt_data);
    matrix_init_f32(&INV_UT, rows, columns, inv_ut_data);

    lu_decomposition(src, &LT, &UT, &P);

    INVERSE_TRIANGULAR(&LT, &INV_LT, 0);
    INVERSE_TRIANGULAR(src, &INV_UT, 1);

    matrix_mult(&INV_UT, &INV_LT, &buffer);
    matrix_mult(&buffer, &P, dst);
    return MATRIX_SUCCESS;
}

status matrix_mult(matrix_f32* a, matrix_f32* b, matrix_f32* dst){
    /// Możliwe wtedy i tylko wtedy gdy liczba kolumn a jest równa liczbie wierszy b
    if(a->num_cols != b->num_rows){
        return MATRIX_WRONG_MATRIX_DIMENSIONS;
    }

    if(a->num_rows != dst->num_rows || b->num_cols != dst->num_cols){
        return MATRIX_WRONG_MATRIX_DIMENSIONS;
    }

    uint16_t rows = b->num_cols;
    uint16_t columns = a->num_rows;

    uint16_t common = a->num_cols;

    float sum;
    for(uint16_t k = 0; k < rows; k++){
        for(uint16_t i = 0; i < rows; i++){
            sum = 0;
            for(uint16_t j = 0; j < common; j++){
                sum += a->p_data[i*columns+j] * b->p_data[j*columns+k];
            }
            dst->p_data[i*rows+k] = sum;
        }
    }

    return MATRIX_SUCCESS;
}

