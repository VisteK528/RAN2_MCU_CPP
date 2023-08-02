#ifndef STM32_I2C_LCD_MATRIX_MATH_H
#define STM32_I2C_LCD_MATRIX_MATH_H

#include "stm32f4xx.h"
#include "arm_math.h"


arm_status mat_lu_decomposition(arm_matrix_instance_f32* src, arm_matrix_instance_f32* lt, arm_matrix_instance_f32* ut);
arm_status modified_arm_mat_inverse_f32(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst);

#endif //STM32_I2C_LCD_MATRIX_MATH_H
