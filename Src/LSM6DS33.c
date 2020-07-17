/*

 * LSM6DS33.c
 *
 *  Created on: 02.12.2017
 *      Author: Bartosz Cichocki SP2FET
 */

#include "LSM6DS33.h"
//#include "motion_fx.h"
#include "main.h"
////#include "cmsis_os.h"

#define SAMPLES_TO_CAL 1000
#define CAL_SIGNATURE 0xAA

int16_t gyroBias[3];

uint8_t LSM6DS33_WriteCmd(uint8_t reg, uint8_t value) {
//	xSemaphoreTake(I2CMutex, 10);

	if (HAL_I2C_Mem_Write_DMA(&hi2c2, LSM6DS33_ADDR_8BIT, reg,
			I2C_MEMADD_SIZE_8BIT, &value, sizeof(reg)) != HAL_OK) {

		//xSemaphoreGive(I2CMutex);

		return 1;
	}

	else {
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
		//	xTimerStart(I2CTimeoutTimerHandle,10);
		//	vTaskDelay(1);
		}

		//xSemaphoreGive(I2CMutex);

		return 0;
	}
}
uint8_t LSM6DS33_UpdateCmd(uint8_t reg, uint8_t value) {
	uint8_t tempData = value;
	//xSemaphoreTake(I2CMutex, 10);
	if (HAL_I2C_Mem_Read_DMA(&hi2c2, LSM6DS33_ADDR_8BIT, reg,
			I2C_MEMADD_SIZE_8BIT, &tempData, sizeof(reg)) == HAL_OK) {
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
			//vTaskDelay(1);

		}
		tempData |= value;
	}
	else
	{
		//xSemaphoreGive(I2CMutex);

		return 1;
	}

	if (HAL_I2C_Mem_Write_DMA(&hi2c2, LSM6DS33_ADDR_8BIT, reg,
			I2C_MEMADD_SIZE_8BIT, &tempData, sizeof(reg)) != HAL_OK)
	{
	//	xSemaphoreGive(I2CMutex);

		return 1;
	}
	else {
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
		//	xTimerStart(I2CTimeoutTimerHandle,10);
		//	vTaskDelay(1);

		}
	//	xSemaphoreGive(I2CMutex);

		return 0;
	}

}

void LSM6DS33_Reset(void) {
	//LSM6DS33_WriteCmd(CTRL3_C,(uint8_t)(1 << SW_RESET));
	HAL_I2C_Mem_Write_DMA(&hi2c2, LSM6DS33_ADDR_8BIT, CTRL3_C,
			I2C_MEMADD_SIZE_8BIT, (uint8_t) (1 << SW_RESET), sizeof(uint8_t));
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
	//	vTaskDelay(1);

	}
}

uint8_t LSM6DS33_ReadCmd(uint8_t reg) {

	uint8_t readData;
	//xSemaphoreTake(I2CMutex, 10);

	if (HAL_I2C_Mem_Read_DMA(&hi2c2, LSM6DS33_ADDR_8BIT, reg,
			I2C_MEMADD_SIZE_8BIT, &readData, sizeof(readData)) != HAL_OK) {
	///	xSemaphoreGive(I2CMutex);

		return 1;
	} else {
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {

		//	xTimerStart(I2CTimeoutTimerHandle,10);
		//	vTaskDelay(1);

			}
	//	xSemaphoreGive(I2CMutex);

		return readData;
	}

}

uint8_t LSM6DS33_SetSpeedXL(uint8_t ODRSpeed) {
	return LSM6DS33_WriteCmd(CTRL1_XL, (ODRSpeed << 4));
}

uint8_t LSM6DS33_SetSpeedG(uint8_t ODRSpeed) {
	return LSM6DS33_WriteCmd(CTRL2_G, (ODRSpeed << 4));
}

uint8_t LSM6DS33_Init(uint8_t ODRSpeedXL, uint8_t ODRSpeedG) {
	uint8_t settingsToSend;
	if (LSM6DS33_ReadCmd(WHO_AM_I) != 0x69)
	{
		IMU_OFF();
		HAL_Delay(100);
		IMU_ON();
		HAL_Delay(100);
		if (LSM6DS33_ReadCmd(WHO_AM_I) != 0x69)
			return 1; // nie odczytuje sygnatury czujnika -> nie ma po³¹czenia na i2c

	}


	settingsToSend = (1 << XEN_XL) | (1 << YEN_XL) | (1 << ZEN_XL);
	if (LSM6DS33_WriteCmd(CTRL9_XL, settingsToSend)) // All axis enabled
		return 1;
	if (LSM6DS33_SetSpeedXL(ODRSpeedXL)) // Acc speed
		return 1;

	settingsToSend = (1 << XEN_G) | (1 << YEN_G) | (1 << ZEN_G);
	if (LSM6DS33_WriteCmd(CTRL10_C, settingsToSend)) // All axis enabled + ACC LPF
		return 1;
	if (LSM6DS33_SetSpeedG(ODRSpeedG)) // Gyro speed
		return 1;

	settingsToSend = (1 << BDU);
	if (LSM6DS33_WriteCmd(CTRL3_C, settingsToSend)) // Block data update enabled
		return 1;

	settingsToSend = (1 << LPF2_XL_EN);
	if (LSM6DS33_WriteCmd(CTRL8_XL, settingsToSend)) // Acc LPF enabled
		return 1;

	settingsToSend = (1 << FUNC_EN);
	if (LSM6DS33_UpdateCmd(CTRL10_C, settingsToSend)) //  ACC LPF
		return 1;
//		settingsToSend =(1 << FS_XL0);
//			if (LSM6DS33_UpdateCmd(CTRL1_XL,settingsToSend)) //  +- 16g
//					return 1;

	return 0;
}

LSM6DS33_DATA LSM6DS33_Read() {
	LSM6DS33_DATA readData;

		while (!(LSM6DS33_ReadCmd(STATUS_REG) && (1 << GDA))) {

		//	vTaskDelay(1);
		};

			readData.accX = LSM6DS33_ReadCmd(OUTX_L_XL) | (LSM6DS33_ReadCmd(OUTX_H_XL) << 8);
			readData.accY = LSM6DS33_ReadCmd(OUTY_L_XL) | (LSM6DS33_ReadCmd(OUTY_H_XL) << 8);
			readData.accZ = LSM6DS33_ReadCmd(OUTZ_L_XL) | (LSM6DS33_ReadCmd(OUTZ_H_XL) << 8);
			readData.gyroX = (LSM6DS33_ReadCmd(OUTX_L_G) | (LSM6DS33_ReadCmd(OUTX_H_G) << 8));
			readData.gyroY = (LSM6DS33_ReadCmd(OUTY_L_G) | (LSM6DS33_ReadCmd(OUTY_H_G) << 8));
			readData.gyroZ = (LSM6DS33_ReadCmd(OUTZ_L_G) | (LSM6DS33_ReadCmd(OUTZ_H_G) << 8));
			readData.temp = LSM6DS33_ReadCmd(OUT_TEMP_L)|(LSM6DS33_ReadCmd(OUT_TEMP_H)  << 8);


	return readData;
}


void Gyro_Calibrate(void)
{

		int16_t gyroMax[3] = {-32767,-32767,-32767};
		int16_t gyroMin[3] = {32767,32767,32767};

		LSM6DS33_DATA data;

		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET); //red
		for(uint16_t i = 0; i < SAMPLES_TO_CAL ;i++)
		{
			data = LSM6DS33_Read();
			if(data.gyroX > gyroMax[0]) gyroMax[0] = data.gyroX;
			if(data.gyroX < gyroMin[0]) gyroMin[0] = data.gyroX;
			if(data.gyroY > gyroMax[1]) gyroMax[1] = data.gyroY;
			if(data.gyroY < gyroMin[1]) gyroMin[1] = data.gyroY;
			if(data.gyroZ > gyroMax[2]) gyroMax[2] = data.gyroZ;
			if(data.gyroZ < gyroMin[2]) gyroMin[2] = data.gyroZ;
		}

		gyroBias[0] = (gyroMax[0] + gyroMin[0])/2;
		gyroBias[1] = (gyroMax[1] + gyroMin[1])/2;
		gyroBias[2] = (gyroMax[2] + gyroMin[2])/2;

		EE_WriteVariable(VirtAddVarTab[11],gyroBias[0]);
		EE_WriteVariable(VirtAddVarTab[12],gyroBias[1]);
		EE_WriteVariable(VirtAddVarTab[13],gyroBias[2]);
		EE_WriteVariable(VirtAddVarTab[10],CAL_SIGNATURE);
		// MotionFX_setGbias(gyroBias)
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);

}
