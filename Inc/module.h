/*
 * ENGINE_control.h
 *
 *  Created on: 03.07.2019
 *      Author: Krystian
 */
#ifndef MODULES_H
#define MODULES_H


#include "main.h"
#include "adc.h"
#include "gpio.h"

#define MODULE1											0
#define MODULE2											1
#define MODULE3											2
#define MODULE4											3
#define MODULE5											4
#define MODULE6											5
#define MODULE7											6
#define MODULE8											7

#define MODULE_ENGINE_FL								MODULE1
#define MODULE_ENGINE_FR								MODULE2
#define MODULE_ENGINE_RL								MODULE3
#define MODULE_ENGINE_RR								MODULE4
#define MODULE_ENGINE_TL								MODULE5
#define MODULE_ENGINE_TR								MODULE6
#define MODULE_ARM_PIN									MODULE7
#define MODULE_MAGNETOMETER_PIN							MODULE8


#define M1_EN_PORT							    		GPIOA
#define M2_EN_PORT										GPIOA
#define M3_EN_PORT	 							    	GPIOC
#define M4_EN_PORT	 									GPIOC
#define M5_EN_PORT	 									GPIOC
#define M6_EN_PORT	 									GPIOC
#define M7_EN_PORT										GPIOB
#define M8_EN_PORT										GPIOB

#define M1_EN_PIN 										GPIO_PIN_9
#define M2_EN_PIN 										GPIO_PIN_8
#define M3_EN_PIN 										GPIO_PIN_9
#define M4_EN_PIN  										GPIO_PIN_8
#define M5_EN_PIN  										GPIO_PIN_7
#define M6_EN_PIN 										GPIO_PIN_6
#define M7_EN_PIN 										GPIO_PIN_15
#define M8_EN_PIN 										GPIO_PIN_14


#define MODULES_AMOUNT									8

#define MODULE_ADC_HANDLE								hadc1

#define MODULE_ENGINE_FL_PIN							M1_EN_PIN
#define MODULE_ENGINE_FR_PIN							M2_EN_PIN
#define MODULE_ENGINE_RL_PIN							M3_EN_PIN
#define MODULE_ENGINE_RR_PIN							M4_EN_PIN
#define MODULE_ENGINE_TL_PIN							M5_EN_PIN
#define MODULE_ENGINE_TR_PIN							M6_EN_PIN
#define MODULE_ARM_PIN									M7_EN_PIN
#define MODULE_MAGNETOMETER_PIN							M8_EN_PIN

#define MODULE_ENGINE_FL_GPIO_PORT						M1_EN_PORT
#define MODULE_ENGINE_FR_GPIO_PORT						M2_EN_PORT
#define MODULE_ENGINE_RL_GPIO_PORT						M3_EN_PORT
#define MODULE_ENGINE_RR_GPIO_PORT						M4_EN_PORT
#define MODULE_ENGINE_TL_GPIO_PORT						M5_EN_PORT
#define MODULE_ENGINE_TR_GPIO_PORT						M6_EN_PORT
#define MODULE_ARM_GPIO_PORT							M7_EN_PORT
#define MODULE_MAGNETOMETER_GPIO_PORT					M8_EN_PORT

#define MODULE_ENGINE_FL_INIT_STATE					    ON
#define MODULE_ENGINE_FR_INIT_STATE					    ON
#define MODULE_ENGINE_RL_INIT_STATE					    ON
#define MODULE_ENGINE_RR_INIT_STATE					    ON
#define MODULE_ENGINE_TL_INIT_STATE					    ON
#define MODULE_ENGINE_TR_INIT_STATE					    ON
#define MODULE_ARM_INIT_STATE					    	ON
#define MODULE_MAGNETOMETER_INIT_STATE					ON

#define MODULE_ENGINE_FL_CURRENT_LIMIT					1.0
#define MODULE_ENGINE_FR_CURRENT_LIMIT					1.0
#define MODULE_ENGINE_RL_CURRENT_LIMIT					1.0
#define MODULE_ENGINE_RR_CURRENT_LIMIT					1.0
#define MODULE_ENGINE_TL_CURRENT_LIMIT					1.0
#define MODULE_ENGINE_TR_CURRENT_LIMIT					1.0
#define MODULE_ARM_CURRENT_LIMIT						1.0
#define MODULE_MAGNETOMETER_CURRENT_LIMIT							1.0

#define MODULE_ENGINE_FL_ADC_TO_CURRENT_FACTOR			0.001
#define MODULE_ENGINE_FR_ADC_TO_CURRENT_FACTOR			0.001
#define MODULE_ENGINE_RL_ADC_TO_CURRENT_FACTOR			0.001
#define MODULE_ENGINE_RR_ADC_TO_CURRENT_FACTOR			0.001
#define MODULE_ENGINE_TL_ADC_TO_CURRENT_FACTOR			0.001
#define MODULE_ENGINE_TR_ADC_TO_CURRENT_FACTOR			0.001
#define MODULE_ARM_ADC_TO_CURRENT_FACTOR				0.001
#define MODULE_MAGNETOMETER_ADC_TO_CURRENT_FACTOR		0.001

#define MODULE_ENGINE_FL									0
#define MODULE_ENGINE_FR									1
#define MODULE_ENGINE_RL									2
#define MODULE_ENGINE_RR									3
#define MODULE_ENGINE_TL									4
#define MODULE_ENGINE_TR									5
#define MODULE_ARM											6
#define MODULE_MAGNETOMETER									7

#define ON													1
#define OFF													0



extern bool				MODULE_RequiredState[MODULES_AMOUNT];
extern bool	 			MODULE_ActualState[MODULES_AMOUNT];
extern uint16_t 		MODULE_AdcValue[MODULES_AMOUNT];
extern float    		MODULE_Current[MODULES_AMOUNT];
extern float			MODULE_AdcToCurrentFactor[MODULES_AMOUNT];
extern float			MODULE_CurrentLimit[MODULES_AMOUNT];



void 			MODULE_Init();
void 			MODULE_SetRequiredState(uint8_t moduleNumber, bool state);
void 			MODULE_SetActualState(uint8_t moduleNumber, bool state);
void			MODULE_CalcCurrent();
void 			MODULE_SetCurrentLimit(uint8_t moduleNumber, float limit);
void			MODULE_CheckCurrentProtection();
bool			MODULE_GetActualState(uint8_t moduleNumber);


#endif
