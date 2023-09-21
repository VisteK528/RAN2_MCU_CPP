/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define SAFEGUARD_PIN_Pin GPIO_PIN_9
#define SAFEGUARD_PIN_GPIO_Port GPIOF
#define SAFEGUARD_PIN_EXTI_IRQn EXTI9_5_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define J5_STEP_Pin GPIO_PIN_0
#define J5_STEP_GPIO_Port GPIOA
#define GRIPPER_FEEDBACK_Pin GPIO_PIN_1
#define GRIPPER_FEEDBACK_GPIO_Port GPIOA
#define J1_EN_Pin GPIO_PIN_5
#define J1_EN_GPIO_Port GPIOA
#define J1_DIR_Pin GPIO_PIN_6
#define J1_DIR_GPIO_Port GPIOA
#define J2_EN_Pin GPIO_PIN_7
#define J2_EN_GPIO_Port GPIOA
#define J2_DIR_Pin GPIO_PIN_4
#define J2_DIR_GPIO_Port GPIOC
#define J3_EN_Pin GPIO_PIN_5
#define J3_EN_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define J3_DIR_Pin GPIO_PIN_1
#define J3_DIR_GPIO_Port GPIOB
#define J4_EN_Pin GPIO_PIN_2
#define J4_EN_GPIO_Port GPIOB
#define J4_DIR_Pin GPIO_PIN_11
#define J4_DIR_GPIO_Port GPIOF
#define J5_EN_Pin GPIO_PIN_13
#define J5_EN_GPIO_Port GPIOF
#define J5_DIR_Pin GPIO_PIN_14
#define J5_DIR_GPIO_Port GPIOF
#define J6_EN_Pin GPIO_PIN_0
#define J6_EN_GPIO_Port GPIOG
#define J6_DIR_Pin GPIO_PIN_1
#define J6_DIR_GPIO_Port GPIOG
#define J1_STEP_Pin GPIO_PIN_9
#define J1_STEP_GPIO_Port GPIOE
#define J2_STEP_Pin GPIO_PIN_11
#define J2_STEP_GPIO_Port GPIOE
#define J3_STEP_Pin GPIO_PIN_13
#define J3_STEP_GPIO_Port GPIOE
#define J4_STEP_Pin GPIO_PIN_14
#define J4_STEP_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define GRIPPER_PWM_Pin GPIO_PIN_6
#define GRIPPER_PWM_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define J1_ENDSTOP_Pin GPIO_PIN_15
#define J1_ENDSTOP_GPIO_Port GPIOA
#define J2_ENDSTOP_Pin GPIO_PIN_11
#define J2_ENDSTOP_GPIO_Port GPIOC
#define J3_ENDSTOP_Pin GPIO_PIN_0
#define J3_ENDSTOP_GPIO_Port GPIOD
#define J4_ENDSTOP_Pin GPIO_PIN_1
#define J4_ENDSTOP_GPIO_Port GPIOD
#define J5_ENDSTOP_Pin GPIO_PIN_4
#define J5_ENDSTOP_GPIO_Port GPIOD
#define LCD_RST_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOD
#define LCD_CS_Pin GPIO_PIN_9
#define LCD_CS_GPIO_Port GPIOG
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_DC_GPIO_Port GPIOG
#define FLASH_CS_Pin GPIO_PIN_11
#define FLASH_CS_GPIO_Port GPIOG
#define J6_STEP_Pin GPIO_PIN_3
#define J6_STEP_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
