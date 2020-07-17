/*
 * LIS3MDL.h
 *
 *  Created on: 15.12.2017
 *      Author: Jas
 */

#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "i2c.h"
#include "eeprom.h"
#include <stdbool.h>

#define LIS3MDL_ADDR		0x1E
#define LIS3MDL_ADDR_8BIT	(LIS3MDL_ADDR << 1)
#define SAMPLES_TO_CAL		1000  //amount of samples to take during calibration
#define CAL_SIGNATURE		(uint16_t)0xA5

/*REGISTERSS*/
#define WHO_AM_I 	0x0F
#define CTRL_REG1	0x20
#define CTRL_REG2	0x21
#define CTRL_REG3	0x22
#define CTRL_REG4	0x23
#define CTRL_REG5	0x24
#define LIS_STATUS_REG	0x27
#define OUT_X_L		0x28
#define OUT_X_H		0x29
#define OUT_Y_L		0x2A
#define OUT_Y_H		0x2B
#define OUT_Z_L		0x2C
#define OUT_Z_H		0x2D
#define TEMP_OUT_L	0x2E
#define TEMP_OUT_H	0x2F
#define	INT_CFG		0x30
#define	INT_SRC		0x31
#define	INT_THS_L	0x32
#define INT_THS_H	0x33

/*BITS*/
#define TEMP_EN 	7
#define	OM1			6
#define	OM0			5
#define DO2			4
#define	DO1			3
#define	DO0			2
#define	FAST_ODR	1
#define	ST			0
#define	FS1			6
#define	FS0			5
#define	REBOOT		3
#define	SOFT_RST	2
#define	LP			5
#define	LIS_SIM			2
#define MD1			1
#define MD0			0
#define OMZ1		3
#define OMZ0		2
#define BLE			1
#define FAST_READ 	7
#define BDU			6
#define ZYXOR		7
#define ZOR			6
#define YOR			5
#define XOR			4
#define ZYXDA		3
#define ZDA			2
#define YDA			1
#define XDA			0
#define XIEN		7
#define YIEN		6
#define ZIEN		5
#define IEA			2
#define LIS_LIR			1
#define IEN			0
#define PTH_X		7
#define PTH_Y		6
#define PTH_Z		5
#define NTH_X		4
#define NTH_Y		3
#define NTH_Z		2
#define MROI		1
#define INT			0
#define THS7		7
#define THS6		6
#define THS5		5
#define THS4		4
#define THS3		3
#define THS2		2
#define THS1		1
#define THS0		0
#define THS14		6
#define THS13		5
#define THS12		4
#define THS11		3
#define THS10		2
#define THS9		1
#define THS8		0

/*MACROS*/
#define ODR_0_625HZ		0
#define ODR_1_25HZ		1
#define ODR_2_5HZ		2
#define ODR_5HZ			3
#define ODR_10HZ		4
#define ODR_20HZ		5
#define ODR_40HZ		6
#define ODR_80HZ		7

#define MAG_SCALE 5461.25
/*DEFS*/
typedef struct
{
	int16_t magX;
	int16_t magY;
	int16_t magZ;
	int16_t temp;
}LIS3MDL_DATA;

uint8_t LIS3MDL_Init(uint8_t ODRSpeed);
uint8_t LIS3MDL_WriteCmd(uint8_t reg, uint8_t value);
uint8_t LIS3MDL_ReadCmd(uint8_t reg);
uint8_t LIS3MDL_SetSpeedMG(uint8_t ODRSpeed);
LIS3MDL_DATA LIS3MDL_Read();
void LIS3MDL_Calibrate(void);
void LIS3MDL_Reset(void);


#endif /* LIS3MDL_H_ */
