
/******************** (C) COPYRIGHT 2015 DUT *********************** *********
 * Author : Hu Wenbo
 * File name : IMU.h
 * Description: attitude solving algorithm header file
 * Date : 2015/11/30 12:43:38
 * Contact: 1461318172 (qq)
************************************************** ********************************/
/********************
  * @attention
  *
  * Occupy STM32 resources:
  *1. Using the Tim7 timer to generate the system time of the us level
  ************************************************** ****************************
 */

#ifndef __IMU_H
#define __IMU_H



#include <math.h>
#include "tim.h"
#define M_PI (float)3.1415926535
#define micros() __HAL_TIM_GET_COUNTER(&htim2)

// After the solution of the gesture output for external program calls
extern volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll; //Unit
//The output of the gyroscope,
extern volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz; //Units per second
extern volatile float acc_vector; //The force detected by the current acceleration is M/S^2.
//Mini IMU AHRS solution API
void IMU_init(void); //Initialize
void IMU_getYawPitchRoll(float *ypr); //Update gestures need to be called periodically
//uint32_t micros(void); //Read the time after the system is powered on. unit us
void Initialize_Q(void);

#endif

//------------------End of File--------------------------- -
