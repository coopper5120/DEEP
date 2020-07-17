/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LSM6DS33.h"
#include "LIS3MDL.h"
#include "IMU.h"
#include "PID.h"
#include "can_parser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t VirtAddVarTab[NB_OF_VAR] = {0x0000, 0x0001, 0x0002,0x0003,0x0004, 0x0005, 0x0006,0x0007,
									 0x0008, 0x0009, 0x000A,0x000B,0x000C, 0x000D, 0x000E,0x000F,
									 0x0010, 0x0011, 0x0012,0x0013,0x0014, 0x0015, 0x0016};

#define VERSION_STR_LENG 35

float MFX_DELTATIME =  0.010;

char lib_version[VERSION_STR_LENG];
char acc_orientation[3];

//MFX_knobs_t iKnobs;
//MFX_input_t data_in;W
//MFX_output_t data_out;


float ypr[3];

volatile uint16_t Timer1, Timer2, Timer3, Timer4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  IMU_OFF();
  HAL_Delay(100);
  IMU_ON();
  HAL_TIM_Base_Start(&htim2);
  MODULE_Init();
  HAL_Delay(5000);
  ENGINE_Init();
  HAL_TIM_Base_Start_IT(&htim3);
  CAN_PARSER_Init();


  PID_Init(&PID_RegRoll, PID_ROLL_INIT_P, PID_ROLL_INIT_I, PID_ROLL_INIT_D);
  PID_Init(&PID_RegPitch, PID_PITCH_INIT_P, PID_PITCH_INIT_I, PID_PITCH_INIT_D);
  PID_Init(&PID_RegYaw, PID_YAW_INIT_P, PID_YAW_INIT_I, PID_YAW_INIT_D);

//  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,SET);

  HAL_SYSTICK_Config(SystemCoreClock/1000);
  LSM6DS33_Init(ODR_104HZ,ODR_104HZ);
  HAL_Delay(100);
  LIS3MDL_Init(ODR_80HZ);
  HAL_Delay(100);
  Timer1 = 100;
  Timer2 = 10;
  Timer3 = 5;
  Timer4 = 12;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

      //	MotionFX_initialize();
      //	MotionFX_GetLibVersion(lib_version);
      	//MotionFX_getKnobs(&iKnobs);
      	/* Modify knobs settings */
      	//MotionFX_setKnobs(&iKnobs);
      	/* Enable 9-axis sensor fusion */
      	//MotionFX_enable_9X(MFX_ENGINE_ENABLE);

    IMU_init();
    Initialize_Q();
    while (1)
    {
  	  if(!Timer2)
  	 	 	  {
  		 IMU_getYawPitchRoll(ypr);


  	 	 		  Timer2 = 10;

  	 	 	  }
  	  if(!Timer4)
  	  {
  		  //MotionFX_propagate(&data_out, &data_in, &MFX_DELTATIME);
  		//  MotionFX_update(&data_out, &data_in, &MFX_DELTATIME, NULL);
  		  Timer4 = 10;
  	  }
  	  if(!Timer3)
  	  	  {
  //	  				data_in.acc[0] = accGyroData.accX /ACC_SCALE;
  //	  				data_in.acc[1] = accGyroData.accY  /ACC_SCALE;
  //	  				data_in.acc[2] = accGyroData.accZ / ACC_SCALE;
  //	  				data_in.gyro[0] = ((accGyroData.gyroX ) / GYRO_SCALE);// * M_PI/180.0;
  //	  				data_in.gyro[1] = ((accGyroData.gyroY ) / GYRO_SCALE);// * M_PI/180.0;
  //	  				data_in.gyro[2] = ((accGyroData.gyroZ ) / GYRO_SCALE);// * M_PI/180.0;
  //	  				data_in.mag[0] = ((magData.magX));///(float)(INT16_MAX/16)*2;
  //	  				data_in.mag[1] = ((magData.magY));///(float)(INT16_MAX/16)*2;
  //	  				data_in.mag[2] = ((magData.magZ));///(float)(INT16_MAX/16)*2;

  	  		  	  	Timer3 = 25;
  	  	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	//CAN_PARSER_AddMessage(CAN_ROLL_CURRENT , ypr, 4, &CAN_TXMailbox1);
  	//HAL_Delay(1);
	//CAN_PARSER_AddMessage(CAN_PITCH_CURRENT , ypr+1, 4, &CAN_TXMailbox2);
	//HAL_Delay(1);
	//CAN_PARSER_AddMessage(CAN_YAW_CURRENT  , ypr+2, 4, &CAN_TXMailbox3);
	//HAL_Delay(1);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
