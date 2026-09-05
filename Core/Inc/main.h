/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define IMU_INT_Pin GPIO_PIN_8
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn
#define Referee_tx_Pin GPIO_PIN_14
#define Referee_tx_GPIO_Port GPIOG
#define PUSH_INIT_L_Pin GPIO_PIN_4
#define PUSH_INIT_L_GPIO_Port GPIOE
#define PUSH_L_Pin GPIO_PIN_5
#define PUSH_L_GPIO_Port GPIOE
#define PUSH_R_Pin GPIO_PIN_6
#define PUSH_R_GPIO_Port GPIOE
#define DBUS_Pin GPIO_PIN_7
#define DBUS_GPIO_Port GPIOB
#define Referee_rx_Pin GPIO_PIN_9
#define Referee_rx_GPIO_Port GPIOG
#define YAW_Init_Pin GPIO_PIN_0
#define YAW_Init_GPIO_Port GPIOF
#define YWA_Pin GPIO_PIN_1
#define YWA_GPIO_Port GPIOF
#define MPU_CS_Pin GPIO_PIN_6
#define MPU_CS_GPIO_Port GPIOF
#define TURN_R_DISH_Pin GPIO_PIN_0
#define TURN_R_DISH_GPIO_Port GPIOC
#define TRIGGER_Pin GPIO_PIN_2
#define TRIGGER_GPIO_Port GPIOC
#define TURN_L_Pin GPIO_PIN_3
#define TURN_L_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_6
#define Buzzer_GPIO_Port GPIOH
#define TURN_R_Pin GPIO_PIN_4
#define TURN_R_GPIO_Port GPIOC
#define Triger_Pin GPIO_PIN_12
#define Triger_GPIO_Port GPIOD
#define TURN_L_DISH_Pin GPIO_PIN_1
#define TURN_L_DISH_GPIO_Port GPIOB
#define TRIGGER_INIT_Pin GPIO_PIN_0
#define TRIGGER_INIT_GPIO_Port GPIOB
#define PUSH_INIT_R_Pin GPIO_PIN_12
#define PUSH_INIT_R_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
