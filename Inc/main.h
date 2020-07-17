/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define IMU_ON() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET)
#define IMU_OFF() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET)
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
#define ADC7_Pin GPIO_PIN_0
#define ADC7_GPIO_Port GPIOC
#define ADC8_Pin GPIO_PIN_1
#define ADC8_GPIO_Port GPIOC
#define MODULE5_Pin GPIO_PIN_0
#define MODULE5_GPIO_Port GPIOA
#define MODULE6_Pin GPIO_PIN_1
#define MODULE6_GPIO_Port GPIOA
#define MOUDLE7_Pin GPIO_PIN_2
#define MOUDLE7_GPIO_Port GPIOA
#define MODULE8_Pin GPIO_PIN_3
#define MODULE8_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_4
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_5
#define ADC2_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_6
#define ADC3_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_7
#define ADC4_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_0
#define ADC5_GPIO_Port GPIOB
#define ADC6_Pin GPIO_PIN_1
#define ADC6_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_11
#define IMU_SDA_GPIO_Port GPIOB
#define M8_EN_Pin GPIO_PIN_14
#define M8_EN_GPIO_Port GPIOB
#define M7_EN_Pin GPIO_PIN_15
#define M7_EN_GPIO_Port GPIOB
#define M6_EN_Pin GPIO_PIN_6
#define M6_EN_GPIO_Port GPIOC
#define M5_EN_Pin GPIO_PIN_7
#define M5_EN_GPIO_Port GPIOC
#define M4_EN_Pin GPIO_PIN_8
#define M4_EN_GPIO_Port GPIOC
#define M3_EN_Pin GPIO_PIN_9
#define M3_EN_GPIO_Port GPIOC
#define M2_EN_Pin GPIO_PIN_8
#define M2_EN_GPIO_Port GPIOA
#define M1_EN_Pin GPIO_PIN_9
#define M1_EN_GPIO_Port GPIOA
#define CAN_STBY_Pin GPIO_PIN_10
#define CAN_STBY_GPIO_Port GPIOA
#define IMU_POWER_Pin GPIO_PIN_10
#define IMU_POWER_GPIO_Port GPIOC
#define MODULE1_Pin GPIO_PIN_6
#define MODULE1_GPIO_Port GPIOB
#define MODULE2_Pin GPIO_PIN_7
#define MODULE2_GPIO_Port GPIOB
#define MODULE3_Pin GPIO_PIN_8
#define MODULE3_GPIO_Port GPIOB
#define MODULE4_Pin GPIO_PIN_9
#define MODULE4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
