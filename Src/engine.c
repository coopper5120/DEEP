/*
 * ENGINE_control.c
 *
 *  Created on: 03.07.2019
 *      Author: Krystian
 */

#include "engine.h"



int 		ENGINE_Speed[ENGINE_AMOUNT];



void	ENGINE_SetSpeed(uint8_t ENGINEName, int speed)
{
	uint16_t pulse;

	switch(ENGINEName)
	{
		case ENGINE_FL:	speed = ENGINE_FL_DIRECTION *speed;
						pulse = ENGINE_PULSE_NEUTRUM + speed * (ENGINE_PULSE_MAX - ENGINE_PULSE_MIN)/(ENGINE_MAX_SPEED-ENGINE_MIN_SPEED);
						ENGINE_Speed[ENGINE_FL] = speed;
						ENGINE_FL_TIM->ENGINE_FL_CCR = pulse;
						break;
		case ENGINE_FR:	speed = ENGINE_FR_DIRECTION *speed;
						pulse = ENGINE_PULSE_NEUTRUM + speed * (ENGINE_PULSE_MAX - ENGINE_PULSE_MIN)/(ENGINE_MAX_SPEED-ENGINE_MIN_SPEED);
						ENGINE_Speed[ENGINE_FR] = speed;
						ENGINE_FR_TIM->ENGINE_FR_CCR = pulse;
						break;
		case ENGINE_RL:	speed = ENGINE_RL_DIRECTION *speed;
						pulse = ENGINE_PULSE_NEUTRUM + speed * (ENGINE_PULSE_MAX - ENGINE_PULSE_MIN)/(ENGINE_MAX_SPEED-ENGINE_MIN_SPEED);
						ENGINE_Speed[ENGINE_RL] = speed;
						ENGINE_RL_TIM->ENGINE_RL_CCR = pulse;
						break;
		case ENGINE_RR:	speed = ENGINE_RR_DIRECTION *speed;
						pulse = ENGINE_PULSE_NEUTRUM + speed * (ENGINE_PULSE_MAX - ENGINE_PULSE_MIN)/(ENGINE_MAX_SPEED-ENGINE_MIN_SPEED);
						ENGINE_Speed[ENGINE_RR] = speed;
						ENGINE_RR_TIM->ENGINE_RR_CCR = pulse;
						break;
		case ENGINE_TL:	speed = ENGINE_TL_DIRECTION *speed;
						pulse = ENGINE_PULSE_NEUTRUM + speed * (ENGINE_PULSE_MAX - ENGINE_PULSE_MIN)/(ENGINE_MAX_SPEED-ENGINE_MIN_SPEED);
						ENGINE_Speed[ENGINE_TL] = speed;
						ENGINE_TL_TIM->ENGINE_TL_CCR = pulse;
						break;
		case ENGINE_TR:	speed = ENGINE_TR_DIRECTION *speed;
						pulse = ENGINE_PULSE_NEUTRUM + speed * (ENGINE_PULSE_MAX - ENGINE_PULSE_MIN)/(ENGINE_MAX_SPEED-ENGINE_MIN_SPEED);
						ENGINE_Speed[ENGINE_TR] = speed;
						ENGINE_TR_TIM->ENGINE_TR_CCR = pulse;
						break;
		case ENGINE_7:	speed = ENGINE_7_DIRECTION *speed;
						pulse = ENGINE_PULSE_NEUTRUM + speed * (ENGINE_PULSE_MAX - ENGINE_PULSE_MIN)/(ENGINE_MAX_SPEED-ENGINE_MIN_SPEED);
					    ENGINE_Speed[ENGINE_7] = speed;
					    ENGINE_7_TIM->ENGINE_7_CCR = pulse;
						break;
		case ENGINE_8:	speed = ENGINE_8_DIRECTION *speed;
						pulse = ENGINE_PULSE_NEUTRUM + speed * (ENGINE_PULSE_MAX - ENGINE_PULSE_MIN)/(ENGINE_MAX_SPEED-ENGINE_MIN_SPEED);
						ENGINE_Speed[ENGINE_8] = speed;
						ENGINE_8_TIM->ENGINE_8_CCR = pulse;
						break;
	}

}
void	ENGINE_Init()
{
	  HAL_TIM_PWM_Start(&ENGINE_FL_TIM_HANDLE, ENGINE_FL_TIM_CHANNEL);
	  HAL_TIM_PWM_Start(&ENGINE_FR_TIM_HANDLE, ENGINE_FR_TIM_CHANNEL);
	  HAL_TIM_PWM_Start(&ENGINE_RL_TIM_HANDLE, ENGINE_RL_TIM_CHANNEL);
	  HAL_TIM_PWM_Start(&ENGINE_RR_TIM_HANDLE, ENGINE_RR_TIM_CHANNEL);
	  HAL_TIM_PWM_Start(&ENGINE_TL_TIM_HANDLE, ENGINE_TL_TIM_CHANNEL);
	  HAL_TIM_PWM_Start(&ENGINE_TR_TIM_HANDLE, ENGINE_TR_TIM_CHANNEL);
	  HAL_TIM_PWM_Start(&ENGINE_7_TIM_HANDLE, ENGINE_7_TIM_CHANNEL);
	  HAL_TIM_PWM_Start(&ENGINE_8_TIM_HANDLE, ENGINE_8_TIM_CHANNEL);

	  ENGINE_SetSpeed(ENGINE_FL, 0);
	  ENGINE_SetSpeed(ENGINE_FR, 0);
	  ENGINE_SetSpeed(ENGINE_RL, 0);
	  ENGINE_SetSpeed(ENGINE_RR, 0);
	  ENGINE_SetSpeed(ENGINE_TL, 0);
	  ENGINE_SetSpeed(ENGINE_TR, 0);
	  ENGINE_SetSpeed(ENGINE_7, 0);
	  ENGINE_SetSpeed(ENGINE_8, 0);
	  HAL_Delay(5000);
}
