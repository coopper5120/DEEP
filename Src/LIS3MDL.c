/*
 * LIS3MDL.c
 *
 *  Created on: 24.12.2017
 *      Author: Jas
 */


#include "LIS3MDL.h"
//#include "errors.h"
int16_t magBias[3];


uint8_t LIS3MDL_UpdateCmd(uint8_t reg, uint8_t value)
{
	uint8_t tempData = value;
	if(HAL_I2C_Mem_Read_DMA(&hi2c2,LIS3MDL_ADDR_8BIT,reg,I2C_MEMADD_SIZE_8BIT,&tempData,sizeof(reg)) == HAL_OK)
	{
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}
		tempData |= value;
	}
	if (HAL_I2C_Mem_Write_DMA(&hi2c2,LIS3MDL_ADDR_8BIT,reg,I2C_MEMADD_SIZE_8BIT,&tempData,sizeof(reg)) != HAL_OK)
		return 1;
	else
	{
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}
		return 0;
	}

}

uint8_t LIS3MDL_WriteCmd(uint8_t reg, uint8_t value)
{
	//if(errorReg && (1 << LIS_ERR)) return 1;
	if (HAL_I2C_Mem_Write_DMA(&hi2c2,LIS3MDL_ADDR_8BIT,reg,I2C_MEMADD_SIZE_8BIT,&value,sizeof(reg)) != HAL_OK)
		return 1;
		else
				{
					while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}
					return 0;
				}
}

uint8_t LIS3MDL_ReadCmd(uint8_t reg)
{
	uint8_t readData;
	//if(errorReg && (1 << LIS_ERR)) return 1;
	if (HAL_I2C_Mem_Read_DMA(&hi2c2,LIS3MDL_ADDR_8BIT,reg,I2C_MEMADD_SIZE_8BIT,&readData,sizeof(reg)) != HAL_OK)
			return 1;
		else
		{
			 while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}
			 return readData;
		}

}

uint8_t LIS3MDL_SetSpeedMG(uint8_t ODRSpeed)
{
	return LIS3MDL_UpdateCmd(CTRL_REG1, (uint8_t)(ODRSpeed << 3) );  //Nie jestem pewien czy o 3 czy o 2
}

uint8_t LIS3MDL_Init(uint8_t ODRSpeed)
{
	uint8_t settingsToSend;
	uint16_t isCalibrated;
	extern int16_t calData[3];
	if(LIS3MDL_ReadCmd(WHO_AM_I) != 0x3D)
	{
		IMU_OFF();
		HAL_Delay(100);
		IMU_ON();
		HAL_Delay(100);
		if(LIS3MDL_ReadCmd(WHO_AM_I) != 0x3D)
			return 1;
	}


	if (LIS3MDL_SetSpeedMG(ODRSpeed)) // Mg speed
			return 1;

	settingsToSend = (1 << OM0) | (1 << OM1);		// UHP mode
	if(LIS3MDL_UpdateCmd(CTRL_REG1,settingsToSend))
			return 1;

	settingsToSend = (1 << FS0); 					// -+8 Gauss
	if(LIS3MDL_WriteCmd(CTRL_REG2,settingsToSend))
			return 1;

	settingsToSend = (1 << OMZ0) | (1 << OMZ1);		// UHP mode (Z axis)
	if(LIS3MDL_WriteCmd(CTRL_REG4,settingsToSend))
			return 1;

	if(LIS3MDL_WriteCmd(CTRL_REG3,0))				//Continous conversion
				return 1;


	settingsToSend = (1 << BDU);
	if (LIS3MDL_WriteCmd(CTRL_REG5,settingsToSend)) // Block data update enabled
		return 1;

	EE_ReadVariable(VirtAddVarTab[3],&isCalibrated);
	if(isCalibrated == CAL_SIGNATURE)
	{
		EE_ReadVariable(VirtAddVarTab[0],&magBias[0]);
		EE_ReadVariable(VirtAddVarTab[1],&magBias[1]);
		EE_ReadVariable(VirtAddVarTab[2],&magBias[2]);
	}
//	else
//		LIS3MDL_Calibrate();

//	for(uint8_t i = 0; i < 3; i++)
//			calData[i] = magBias[i];
	return 0;
}

void LIS3MDL_Reset(void)
{
	HAL_I2C_Mem_Write_DMA(&hi2c2,LIS3MDL_ADDR_8BIT,CTRL_REG2,I2C_MEMADD_SIZE_8BIT,(uint8_t)(1 << SOFT_RST),sizeof(uint8_t));
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}
}
LIS3MDL_DATA LIS3MDL_Read(void)
{
	LIS3MDL_DATA readData;
	while(!(LIS3MDL_ReadCmd(LIS_STATUS_REG) && (1 << ZYXDA)))
	{
		//if(errorReg && (1 << LIS_ERR)) break;
	};

		readData.magX = (LIS3MDL_ReadCmd(OUT_X_L)|(LIS3MDL_ReadCmd(OUT_X_H) << 8))- magBias[0];
		readData.magY = (LIS3MDL_ReadCmd(OUT_Y_L)|(LIS3MDL_ReadCmd(OUT_Y_H) << 8))- magBias[1];
		readData.magZ = (LIS3MDL_ReadCmd(OUT_Z_L)|(LIS3MDL_ReadCmd(OUT_Z_H) << 8))- magBias[2];
		readData.temp = LIS3MDL_ReadCmd(TEMP_OUT_L)|(LIS3MDL_ReadCmd(TEMP_OUT_H) << 8);

	return readData;
}

void LIS3MDL_Calibrate(void)
{
	int16_t magMax[3] = {-32767,-32767,-32767};
	int16_t magMin[3] = {32767,32767,32767};
	int16_t magBias[3];
	LIS3MDL_DATA data;


	for(uint16_t i = 0; i < SAMPLES_TO_CAL ;i++)
	{
		data = LIS3MDL_Read();
		if(data.magX > magMax[0]) magMax[0] = data.magX;
		if(data.magX < magMin[0]) magMin[0] = data.magX;
		if(data.magY > magMax[1]) magMax[1] = data.magY;
		if(data.magY < magMin[1]) magMin[1] = data.magY;
		if(data.magZ > magMax[2]) magMax[2] = data.magZ;
		if(data.magZ < magMin[2]) magMin[2] = data.magZ;
	}
	//Hard iron correction
	magBias[0] = (magMax[0] + magMin[0])/2;
	magBias[1] = (magMax[1] + magMin[1])/2;
	magBias[2] = (magMax[2] + magMin[2])/2;

	EE_WriteVariable(VirtAddVarTab[0],magBias[0]);
	EE_WriteVariable(VirtAddVarTab[1],magBias[1]);
	EE_WriteVariable(VirtAddVarTab[2],magBias[2]);
	EE_WriteVariable(VirtAddVarTab[3],CAL_SIGNATURE);

}

