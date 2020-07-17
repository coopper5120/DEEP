
#include "module.h"




float		MODULE_AdcToCurrentFactor[MODULES_AMOUNT] =
																{MODULE_ENGINE_FL_ADC_TO_CURRENT_FACTOR,
																MODULE_ENGINE_FR_ADC_TO_CURRENT_FACTOR,
																MODULE_ENGINE_RL_ADC_TO_CURRENT_FACTOR,
																MODULE_ENGINE_RR_ADC_TO_CURRENT_FACTOR,
																MODULE_ENGINE_TL_ADC_TO_CURRENT_FACTOR,
																MODULE_ENGINE_TR_ADC_TO_CURRENT_FACTOR,
																MODULE_ARM_ADC_TO_CURRENT_FACTOR,
																MODULE_MAGNETOMETER_ADC_TO_CURRENT_FACTOR};

float		MODULE_CurrentLimit[MODULES_AMOUNT] =
																{MODULE_ENGINE_FL_CURRENT_LIMIT,
																MODULE_ENGINE_FR_CURRENT_LIMIT,
																MODULE_ENGINE_RL_CURRENT_LIMIT,
																MODULE_ENGINE_RR_CURRENT_LIMIT,
																MODULE_ENGINE_TL_CURRENT_LIMIT,
																MODULE_ENGINE_TR_CURRENT_LIMIT,
																MODULE_ARM_CURRENT_LIMIT,
																MODULE_MAGNETOMETER_CURRENT_LIMIT};

bool	 	MODULE_RequiredState[MODULES_AMOUNT];
bool	 	MODULE_ActualState[MODULES_AMOUNT];
uint16_t 	MODULE_AdcValue[MODULES_AMOUNT];
float    	MODULE_Current[MODULES_AMOUNT];



void 	MODULE_SetRequiredState(uint8_t moduleName, bool state)
{

	switch(moduleName)
	{
		case  MODULE_ENGINE_FL: HAL_GPIO_WritePin(MODULE_ENGINE_FL_GPIO_PORT, MODULE_ENGINE_FL_PIN, state);
								break;

		case MODULE_ENGINE_FR: 	HAL_GPIO_WritePin(MODULE_ENGINE_FR_GPIO_PORT, MODULE_ENGINE_FR_PIN, state);
								break;

		case MODULE_ENGINE_RL:	HAL_GPIO_WritePin(MODULE_ENGINE_RL_GPIO_PORT, MODULE_ENGINE_RL_PIN, state);
								break;

		case MODULE_ENGINE_RR: 	HAL_GPIO_WritePin(MODULE_ENGINE_RR_GPIO_PORT, MODULE_ENGINE_RR_PIN, state);
								break;

		case MODULE_ENGINE_TL: 	HAL_GPIO_WritePin(MODULE_ENGINE_TL_GPIO_PORT, MODULE_ENGINE_TL_PIN, state);
								break;

		case  MODULE_ENGINE_TR: HAL_GPIO_WritePin(MODULE_ENGINE_TR_GPIO_PORT, MODULE_ENGINE_TR_PIN, state);
								break;

		case MODULE_ARM: 		HAL_GPIO_WritePin(MODULE_ARM_GPIO_PORT, MODULE_ARM_PIN, state);
								break;

		case MODULE_MAGNETOMETER:HAL_GPIO_WritePin(MODULE_MAGNETOMETER_GPIO_PORT, MODULE_MAGNETOMETER_PIN, state);
								break;
		//MODULE_ActualState[moduleName] = true;
	}
	MODULE_ActualState[moduleName] = state;
}

void 	MODULE_SetActualState(uint8_t moduleName, bool state)
{
		MODULE_ActualState[moduleName] = state;
}

void	MODULE_CalcCurrent()
{
	for(uint8_t moduleNumber = 0; moduleNumber<MODULES_AMOUNT; moduleNumber++)
	{
		MODULE_Current[moduleNumber] = ((float)MODULE_AdcValue[moduleNumber]) * MODULE_AdcToCurrentFactor[moduleNumber];
//
		int size = 6;
	    char buf[size+1];

		//HAL_UART_Transmit(&huart3, itoa(MODULE_AdcValue[MODULE_AdcNumber[moduleNumber]],buf,10), size+1,10000);
		//HAL_UART_Transmit(&huart3, " ", 1,10000);
	}
	//HAL_UART_Transmit(&huart3, MODULE_AdcValue[MODULE_AdcNumber[0]], 2,10000);
}

void	MODULE_CheckCurrentProtection()
{
	for(uint8_t moduleNumber = 0; moduleNumber<MODULES_AMOUNT;moduleNumber++)
		{
			if(MODULE_Current[moduleNumber] >= MODULE_CurrentLimit[moduleNumber])
			{
					MODULE_SetActualState(moduleNumber, OFF);
					// tutaj wysylanie komunikatu o przekroczonym pradzie
			}
			else 	MODULE_SetActualState(moduleNumber, MODULE_RequiredState[moduleNumber]);
		}
}

void 	MODULE_Init()
{
	HAL_ADC_Start_DMA(&MODULE_ADC_HANDLE, (uint16_t*)MODULE_AdcValue, MODULES_AMOUNT);

	MODULE_SetRequiredState(MODULE_ENGINE_FL,MODULE_ENGINE_FL_INIT_STATE);
	MODULE_SetRequiredState(MODULE_ENGINE_FR,MODULE_ENGINE_FR_INIT_STATE);
	MODULE_SetRequiredState(MODULE_ENGINE_RL,MODULE_ENGINE_RL_INIT_STATE);
	MODULE_SetRequiredState(MODULE_ENGINE_RR,MODULE_ENGINE_RR_INIT_STATE);
	MODULE_SetRequiredState(MODULE_ENGINE_TL,MODULE_ENGINE_TL_INIT_STATE);
	MODULE_SetRequiredState(MODULE_ENGINE_TR,MODULE_ENGINE_TR_INIT_STATE);
	MODULE_SetRequiredState(MODULE_ARM,MODULE_ARM_INIT_STATE);
	MODULE_SetRequiredState(MODULE_MAGNETOMETER,MODULE_MAGNETOMETER_INIT_STATE);
}

void 	MODULE_SetCurrentLimit(uint8_t moduleNumber, float limit)
{
	MODULE_CurrentLimit[moduleNumber] = limit;
}
