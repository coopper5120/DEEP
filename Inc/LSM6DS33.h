/*
 * LSM6DS33.h
 *
 *  Created on: 29.11.2017
 *      Author: Bartosz Cichocki SP2FET
 */

#ifndef LSM6DS33_H_
#define LSM6DS33_H_

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "i2c.h"
#include "eeprom.h"

#include <math.h>
#include <stdbool.h>

#define LSM6DS33_ADDR		0x6B
#define LSM6DS33_ADDR_8BIT	(LSM6DS33_ADDR << 1)


/* REGISTERS */
#define FUNC_CFG_ADDRESS 	0x01
#define FIFO_CTRL1			0x06
#define FIFO_CTRL2			0x07
#define FIFO_CTRL3			0x08
#define FIFO_CTRL4			0x09
#define FIFO_CTRL5			0x0A
#define ORIENT_CFG_G		0x0B
#define INT1_CTRL			0x0D
#define INT2_CTRL			0x0E
#define WHO_AM_I			0x0F
#define CTRL1_XL			0x10
#define CTRL2_G				0x11
#define CTRL3_C				0x12
#define CTRL4_C				0x13
#define CTRL5_C				0x14
#define CTRL6_C				0x15
#define CTRL7_C				0x16
#define CTRL8_XL			0x17
#define CTRL9_XL			0x18
#define CTRL10_C			0x19
#define WAKE_UP_SRC			0x1B
#define TAP_SRC				0x1C
#define D6D_SRC				0x1D
#define STATUS_REG			0x1E
#define OUT_TEMP_L			0x20
#define OUT_TEMP_H			0x21
#define OUTX_L_G			0x22
#define OUTX_H_G			0x23
#define OUTY_L_G			0x24
#define OUTY_H_G			0x25
#define OUTZ_L_G			0x26
#define OUTZ_H_G			0x27
#define OUTX_L_XL			0x28
#define OUTX_H_XL			0x29
#define OUTY_L_XL			0x2A
#define OUTY_H_XL			0x2B
#define OUTZ_L_XL			0x2C
#define OUTZ_H_XL			0x2D
#define FIFO_STATUS1		0x3A
#define FIFO_STATUS2		0x3B
#define FIFO_STATUS3		0x3C
#define FIFO_STATUS4		0x3D
#define FIFO_DATA_OUT_L		0x3E
#define FIFO_DATA_OUT_H		0x3F
#define TIMESTAMP0_REG		0x40
#define TIMESTAMP1_REG		0x41
#define TIMESTAMP2_REG		0x42
#define STEP_TIMESTAMP_L	0x49
#define STEP_TIMESTAMP_H	0x4A
#define STEP_COUNTER_L		0x4B
#define STEP_COUNTER_H		0x4C
#define FUNC_SRC			0x53
#define TAP_CFG				0x58
#define TAP_THS_6D			0x59
#define INT_DUR2			0x5A
#define WAKE_UP_THS			0x5B
#define WAKE_UP_DUR			0x5C
#define FREE_FALL			0x5D
#define MD1_CFG				0x5E
#define MD2_CFG				0x5F

/* BITS */
#define FUNC_CFG_EN				7
#define FTH_11					3
#define FTH_10					2
#define FTH_9					1
#define FTH_8					0
#define FTH_7					7
#define FTH_6					6
#define FTH_5					5
#define FTH_4					4
#define FTH_3					3
#define FTH_2					2
#define FTH_1					1
#define FTH_0					0
#define TIMER_PEDO_FIFO_EN		7
#define TIMER_PEDO_FIFO_DRDY	6
#define DEC_FIFO_GYRO2			5
#define DEC_FIFO_GYRO1			4
#define DEC_FIFO_GYRO0			3
#define DEC_FIFO_XL2			2
#define DEC_FIFO_XL1			1
#define DEC_FIFO_XL0			0
#define ONLY_HIGH_DATA			6
#define TIMER_PEDO_DEC_FIFO2	5
#define TIMER_PEDO_DEC_FIFO1	4
#define TIMER_PEDO_DEC_FIFO0	3
#define ODR_FIFO_3				6
#define ODR_FIFO_2				5
#define ODR_FIFO_1				4
#define ODR_FIFO_0				3
#define FIFO_MODE_2				2
#define FIFO_MODE_1				1
#define FIFO_MODE_0				0
#define SIGNX_G					5
#define SIGNY_G					4
#define SIGNZ_G					3
#define ORIENT_2				2
#define ORIENT_1				1
#define ORIENT_0				0
#define INT1_STEP_DETECTOR		7
#define INT1_SIG_MOT			6
#define INT1_FULL_FLAG			5
#define INT1_FIFO_OVR			4
#define INT1_FTH				3
#define INT1_BOOT				2
#define INT1_DRDY_G				1
#define INT1_DRDY_XL			0
#define INT2_STEP_DELTA			7
#define INT2_STEP_COUNT_OV		6
#define INT2_FULL_FLAG			5
#define INT2_FIFO_OVR			4
#define INT2_FTH				3
#define INT2_DRDY_TEMP			2
#define INT2_DRDY_G				1
#define INT2_DRDY_XL			0
#define ODR_XL3					7
#define ODR_XL2					6
#define ODR_XL1					5
#define ODR_XL0					4
#define FS_XL1					3
#define FS_XL0					2
#define BW_XL1					1
#define BW_XL0					0
#define ODR_G3					7
#define ODR_G2					6
#define ODR_G1					5
#define ODR_G0					4
#define FS_G1					3
#define FS_G0					2
#define FS_125					1
#define BOOT					7
#define BDU						6
#define H_LACTIVE				5
#define PP_OD					4
#define SIM						3
#define IF_INC					2
#define BLE						1
#define SW_RESET				0
#define XL_BW_SCAL_ODR			7
#define SLEEP_G					6
#define INT2_ON_INT1			5
#define FIFO_TEMP_EN			4
#define DRDY_MASK				3
#define I2C_DISABLE				2
#define STOP_ON_FTH				0
#define ROUNDING2				7
#define ROUNDING1				6
#define ROUNDING0				5
#define ST1_G					3
#define ST0_G					2
#define ST1_XL					1
#define ST0_XL					0
#define TRIG_EN					7
#define LVLEN					6
#define LVL2_EN					5
#define XL_HM_MODE				4
#define G_HM_MODE				7
#define HP_G_EN					6
#define HPCF_G1					5
#define HPCF_G0					4
#define HP_G_RST				3
#define ROUNDING_STATUS			2
#define LPF2_XL_EN				7
#define HPCF_XL1				6
#define HPCF_XL0				5
#define HP_SLOPE_XL_EN			2
#define LOW_PASS_ON_6D			0
#define ZEN_XL					5
#define YEN_XL					4
#define XEN_XL					3
#define ZEN_G					5
#define YEN_G					4
#define XEN_G					3
#define FUNC_EN					2
#define PEDO_RST_STEP			1
#define SIGN_MOTI_ON_EN			0
#define FF_IA					5
#define SLEEP_STATE_IA			4
#define WU_IA					3
#define X_WU					2
#define Y_WU					1
#define Z_WU					0
#define TAP_IA					6
#define SINGLE_TAP				5
#define DOUBLE_TAP				4
#define TAP_SIGN				3
#define X_TAP					2
#define Y_TAP					1
#define Z_TAP					0
#define D6D_IA					6
#define ZH						5
#define ZL						4
#define YH						3
#define YL						2
#define XH						1
#define	XL						0
#define EV_BOOT					3
#define TDA						2
#define GDA						1
#define XLDA					0
#define DIFF_FIFO_11			3
#define DIFF_FIFO_10			2
#define DIFF_FIFO_9				1
#define DIFF_FIFO_8				0
#define DIFF_FIFO_7				7
#define DIFF_FIFO_6				6
#define DIFF_FIFO_5				5
#define DIFF_FIFO_4				4
#define DIFF_FIFO_3				3
#define DIFF_FIFO_2				2
#define DIFF_FIFO_1				1
#define DIFF_FIFO_0				0
#define FTH						7
#define FIFO_OVER_RUN			6
#define FIFO_FULL				5
#define FIFO_EMPTY				4
#define FIFO_PATTERN_9			1
#define FIFO_PATTERN_8			0
#define FIFO_PATTERN_7			7
#define FIFO_PATTERN_6			6
#define FIFO_PATTERN_5			5
#define FIFO_PATTERN_4			4
#define FIFO_PATTERN_3			3
#define FIFO_PATTERN_2			2
#define FIFO_PATTERN_1			1
#define FIFO_PATTERN_0			0
#define STEP_COUNT_DELTA_IA		7
#define SIGN_MOTION_IA			6
#define TILT_IA					5
#define STEP_DETECTED			4
#define STEP_OVERFLOW			3
#define TIMER_EN				7
#define PEDO_EN					6
#define TILT_EN					5
#define SLOPE_FDS				4
#define TAP_X_EN				3
#define TAP_Y_EN				2
#define TAP_Z_EN				1
#define LIR						0
#define D4D_EN					7
#define SIXD_THS1				6
#define SIXD_THS0				5
#define TAP_THS4				4
#define TAP_THS3				3
#define TAP_THS2				2
#define TAP_THS1				1
#define TAP_THS0				0

/* MACROS */
#define ODR_PWR_DWN				0
#define ODR_13HZ				1
#define ODR_26HZ				2
#define ODR_52HZ				3
#define ODR_104HZ				4
#define ODR_208HZ				5
#define ODR_416HZ				6
#define ODR_833HZ				7
#define ODR_1066HZ				8
#define ODR_3330HZ				9
#define ODR_6660HZ				10


#ifndef M_PI
#define M_PI 3.14159265359
#endif

#define dt 0.002403846

#define ACC_SCALE 16393.0
#define GYRO_SCALE 114.280

#define SAMPLE_FREQ                     ((uint8_t)100)  /* [Hz] */
#define SAMPLE_PERIOD                   ((uint8_t)10)   /* [ms] */
#define MOTIONFX_ENGINE_DELTATIME       0.01f

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f


/* DEFS */
typedef struct
{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
	int16_t temp;

}LSM6DS33_DATA;

uint8_t LSM6DS33_WriteCmd(uint8_t reg, uint8_t value);
uint8_t LSM6DS33_UpdateCmd(uint8_t reg, uint8_t value);
uint8_t LSM6DS33_ReadCmd(uint8_t reg);
uint8_t LSM6DS33_SetSpeedXL(uint8_t ODRSpeed);
uint8_t LSM6DS33_SetSpeedG(uint8_t ODRSpeed);
uint8_t LSM6DS33_Init(uint8_t ODRSpeedXL, uint8_t ODRSpeedG);
LSM6DS33_DATA LSM6DS33_Read();
void Calculate(LSM6DS33_DATA readData);
void LSM6DS33_Reset(void);
void Gyro_Calibrate(void);

float pitch, yaw, roll;
#endif /* LSM6DS33_H_ */
