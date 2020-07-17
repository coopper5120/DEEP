/*
 * ENGINE_control.h
 *
 *  Created on: 03.07.2019
 *      Author: Krystian
 */
#ifndef ENGINES_H// is myheader.h already included?
#define ENGINES_H // define this so we know it's included

// ustawienie zegara


#include "main.h"
#include "tim.h"
#include "module.h"

#define ENGINE_AMOUNT 					6

#define FORWARD							1
#define BACKWARD					   -1

#define ENGINE_FL_DIRECTION				FORWARD
#define ENGINE_FR_DIRECTION				FORWARD
#define ENGINE_RL_DIRECTION				FORWARD
#define ENGINE_RR_DIRECTION				FORWARD
#define ENGINE_TL_DIRECTION				FORWARD
#define ENGINE_TR_DIRECTION				FORWARD
#define ENGINE_7_DIRECTION				FORWARD
#define ENGINE_8_DIRECTION				FORWARD

#define ENGINE_FL_TIM_HANDLE			htim4
#define ENGINE_FR_TIM_HANDLE			htim4
#define ENGINE_RL_TIM_HANDLE			htim4
#define ENGINE_RR_TIM_HANDLE			htim4
#define ENGINE_TL_TIM_HANDLE			htim5
#define ENGINE_TR_TIM_HANDLE			htim5
#define ENGINE_7_TIM_HANDLE				htim5
#define ENGINE_8_TIM_HANDLE				htim5

#define ENGINE_FL_TIM_CHANNEL			TIM_CHANNEL_1
#define ENGINE_FR_TIM_CHANNEL			TIM_CHANNEL_2
#define ENGINE_RL_TIM_CHANNEL			TIM_CHANNEL_3
#define ENGINE_RR_TIM_CHANNEL			TIM_CHANNEL_4
#define ENGINE_TL_TIM_CHANNEL			TIM_CHANNEL_1
#define ENGINE_TR_TIM_CHANNEL			TIM_CHANNEL_2
#define ENGINE_7_TIM_CHANNEL			TIM_CHANNEL_3
#define ENGINE_8_TIM_CHANNEL			TIM_CHANNEL_4


#define ENGINE_FL_TIM					TIM4
#define ENGINE_FR_TIM					TIM4
#define ENGINE_RL_TIM					TIM4
#define ENGINE_RR_TIM					TIM4
#define ENGINE_TL_TIM					TIM5
#define ENGINE_TR_TIM					TIM5
#define ENGINE_7_TIM					TIM5
#define ENGINE_8_TIM					TIM5

#define ENGINE_FL_CCR					CCR1
#define ENGINE_FR_CCR					CCR2
#define ENGINE_RL_CCR					CCR3
#define ENGINE_RR_CCR					CCR4
#define ENGINE_TL_CCR					CCR1
#define ENGINE_TR_CCR					CCR2
#define ENGINE_7_CCR					CCR3
#define ENGINE_8_CCR					CCR4

#define ENGINE_FL						0
#define ENGINE_FR						1
#define ENGINE_RL						2
#define ENGINE_RR						3
#define ENGINE_TL						4
#define ENGINE_TR						5
#define ENGINE_7				   	    6
#define ENGINE_8				        7

// druga definicja
#define ENGINE_PULSE_NEUTRUM 			1500
#define ENGINE_PULSE_MAX				2000
#define ENGINE_PULSE_MIN				1000
//
#define ENGINE_MAX_SPEED				100
#define ENGINE_MIN_SPEED	   		   -100


extern int 		ENGINE_Speed[ENGINE_AMOUNT];

extern void		ENGINE_SetSpeed(uint8_t ENGINEName, int speed);
extern void		ENGINE_Init();


#endif
