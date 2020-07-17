/******************** (C) COPYRIGHT 2015 DUT *********************** *********
 * Author : Hu Wenbo
 * File name : IMU.c
 * Description: Attitude solution algorithm
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

#include "IMU.h"
#include "LSM6DS33.h"
#include "LIS3MDL.h"

#define twoKpDef (1.0f ) // 2 * proportional gain
#define twoKiDef (0.2f) // 2 * integral gain

#define DATA_SIZE 100
volatile float exInt, eyInt, ezInt; // error integral
volatile float integralFBx, integralFBy, integralFBz;
volatile float q0, q1, q2, q3; // global quaternion
volatile float qa0, qa1, qa2, qa3;
volatile float integralFBhand, handdiff;
volatile double halftime ;
volatile uint32_t lastUpdate, now; // sampling period count unit us
volatile uint16_t sysytem_time_ms = 0;
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll;// attitude angle unit
volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz; // attitude angular rate unit
volatile unsigned char IMU_inited = 0;
volatile uint16_t imu_clce = 0;
volatile float acc_vector = 0; //The force detected by the current acceleration M/S^2
volatile float acc_X, acc_Y, acc_Z, acc_MX, acc_MY, acc_MZ; //acceleration initializes quaternion
LIS3MDL_DATA magData;
LSM6DS33_DATA accGyroData;
// Fast inverse square-root
/**************************Implement function ************************* ***********************
*Function prototype: float invSqrt(float x)
*Function: Quick calculation 1/Sqrt(x)
Input parameters: the value to be calculated
Output parameters: result
************************************************** *****************************/
float invSqrt(float x)
{
	volatile float halfx = 0.5f * x;
	volatile float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************Implement function ************************* ***********************
*Function prototype: void IMU_init(void)
*Features: Initialize IMU related
Initialize each sensor
Initialize quaternion
Clear the points
Update system time
Input parameters: none
Output parameters: no
************************************************** *****************************/
void IMU_init(void)
{
	//MPU6050_initialize();
	//HMC5883L_SetUp();
	//Delay_ms(50);
	//MPU6050_initialize();
	//HMC5883L_SetUp();

	// initialize quaternion
	q0 = 1.0f; // initialize quaternion
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	qa0 = 1.0f; // initialize quaternion
	qa1 = 0.0f;
	qa2 = 0.0f;
	qa3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	integralFBx = 0.0;
	integralFBy = 0.0;
	integralFBz = 0.0;
	lastUpdate = micros();//update time
	now = micros();
}

/**************************Implement function ************************* ***********************
*Function prototype: void IMU_getValues(volatile float * values)
*Function: Read acceleration Gyro Current value of magnetometer
Input parameters: the first address of the array where the result will be stored
Output parameters: no
************************************************** *****************************/
#define new_weight 0.4f//New weight
#define old_weight 0.6f//old weight

void IMU_getValues(float *values)
{
	int16_t accgyroval[6];
	static volatile float lastacc[3] = {0, 0, 0};
	int i;
	// Read the current ADC of the acceleration and gyroscope
	//MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);

	 accGyroData = LSM6DS33_Read();
	 magData = LIS3MDL_Read();
	 values[0] = (float) (accGyroData.accX/ACC_SCALE) * new_weight + lastacc[0] * old_weight ;
	 values[1] = (float) (accGyroData.accY/ACC_SCALE) * new_weight + lastacc[1] * old_weight ;
	 values[2] = (float) (accGyroData.accZ/ACC_SCALE) * new_weight + lastacc[2] * old_weight ;
	 for(i = 0; i < 3; i++)
		 lastacc[i] = values[i];

	 values[3] = (float) accGyroData.gyroX/ GYRO_SCALE;
	 values[4] = (float) accGyroData.gyroY/ GYRO_SCALE;
	 values[5] = (float) accGyroData.gyroZ/ GYRO_SCALE;

	 values[6] = (float) magData.magX;
	 values[7] = (float) magData.magY;
	 values[8] = (float) magData.magZ;


//	for(i = 0; i < 3; i++)
//	{
//
//			values[i] = (float) accgyroval[i] * new_weight + lastacc[i] * old_weight ;
//			lastacc[i] = values[i];
//		}
//		else
//		{
//			values[i] = ((float) accgyroval[i]);// / 16.4f; // convert to degrees per second
//		//This has changed the range to 2000 degrees per second. 16.4 corresponds to 1 degree per second.
//		}
//	}
//	HMC58X3_mgetValues(&values[6]); //Read the ADC value of the magnetometer
	IMU_GYROx = values[3];
	IMU_GYROy = values[4];
	IMU_GYROz = values[5];

}


/**************************Implement function ************************* ***********************
*Function prototype: void IMU_AHRSupdate
*Features: Update AHRS Update Quaternion
Input parameters: Current measured value.
Output parameters: no
************************************************** *****************************/
#define Kp 2.0f // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.03f // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az, volatile float mx, volatile float my, volatile float mz)
{

	volatile float norm;
	volatile float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz, wx, wy, wz;
	volatile float ex, ey, ez, halfT;
	volatile float temp0, temp1, temp2, temp3;
	volatile float temp;
	// Calculate the valuesobtained by using these first.
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	now = micros(); //read time
	if(now < lastUpdate) //The timer overflows.
	{
		halfT = ((float)(now + (0xffffffff - lastUpdate)) / 2000000.0f);
		lastUpdate = now;
	//return;
	}
	else
	{
		halfT = ((float)(now - lastUpdate) / 2000000.0f);
	}
	halftime = halfT;
	lastUpdate = now; //update time

	temp = sqrt(ax * ax + ay * ay + az * az);
	temp = (temp / 16384.0f) * 9.8f; // into M / S ^ 2 as the unit
	acc_vector = acc_vector + // low pass filtering. Cutoff frequency 20hz
	(halfT * 2.0f / (7.9577e-3f + halfT * 2.0f)) * (temp - acc_vector);

	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	// Calculate roll, pitch with acceleration
	// temp = ax * invSqrt((ay * ay + az * az));
	// ACC_Pitch = atan(temp)* 57.3;
	//
	// temp = ay * invSqrt((ax * ax + az * az));
	// ACC_Roll = atan(temp)* 57.3;

	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;

	//comput reference direction of flux
	//The vector measured from the electronic compass of the body coordinate system is converted into the magnetic field vector hxyz (measured value) in the geographic coordinate system.
	hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
	hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
	hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
	/*
	Calculate the magnetic field vector bxyz (reference value) in the geographic coordinate system.
	Because of the geographical geomagnetic horizontal angle, we know that it is 0 degrees (the factor of throwing off the magnetic declination, fixed to the north), so by=0, bx=a certain value
	However, the georeferenced geomagnetic vector also has a component bz on the vertical plane, which is different everywhere on the earth.
	We can't know, it can't be used for fusion (the accelerometer is more suitable for vertical direction correction fusion), so it is copied directly from the measured value hz, bz=hz.
	The horizontal component of the magnetic field, the reference value and the measured value should be the same size (bx*bx) + (by*by)) = ((hx*hx) + (hy*hy)).
	Because by=0, it is reduced to (bx*bx) = ((hx*hx) + (hy*hy)). Can calculate bx.
	*/
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	// estimated direction of gravity and flux (v and w)
	/*
	This is the three elements that convert the quaternion into the third column in the Direction Cosine Matrix.
	According to the definition of cosine matrix and Euler angle, the gravity vector of the geographic coordinate system is transferred to the body coordinate system, which is exactly these three elements.
	So here vx\y\z is actually the gravity unit vector converted from the current coordinate angle of the Euler angle (ie quaternion).
	*/
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	/*
	We transfer the magnetic field vector bxyz on the geographic coordinate system to the body to wxyz.
	Because by=0, all the parts related to by are omitted.
	Similar to the above calculation of gravity vxyz, because gravity g gz=1, gx=gy=0, so the part related to gxgy is also omitted.
	*/
	wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
	wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
	wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	// Now the acceleration measurement vector and the reference vector cross product, the magnetic field measurement vector and the reference vector also cross product. They are all used to fix the top.
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	/*
	Axyz is the gravity vector measured by the accelerometer on the body coordinate reference system, which is the actual measured gravity vector.
	Axyz is the measured gravity vector, and vxyz is the gravity vector derived from the gyro-integrated attitude. They are the gravity vectors on the body coordinate reference system.
	The error vector between them is the error between the attitude after the gyro integration and the added attitude.
	The error between vectors can be represented by vector cross product (also called vector outer product, cross product multiplication), and exyz is the cross product of two gravity vectors.
	This cross product vector is still located in the body coordinate system, and the gyro integral error is also in the body coordinate system, and the cross product is proportional to the gyro integral error, which is just used to correct the gyro. (You can imagine things yourself) Since the gyro is directly integrated into the body, the correction of the gyro is directly reflected in the correction of the body coordinate system.
	*/
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		// adjusted gyroscope measurements
		// Use the cross product error to do the PI correction gyro bias
		gx = gx + (Kp * ex + exInt);
		gy = gy + (Kp * ey + eyInt);
		gz = gz + (Kp * ez + ezInt);

	}

	// integrate quaternion rate and normalise
	// quaternion differential equation
	temp0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	temp1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	temp2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	temp3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	q0 = temp0 * norm;
	q1 = temp1 * norm;
	q2 = temp2 * norm;
	q3 = temp3 * norm;
}




void FreeIMU_AHRSupdate(volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az)
{
	volatile float norm;
	// float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;

	// Calculate the values obtained by using these first.
	volatile float q0q0 = qa0 * qa0;
	volatile float q0q1 = qa0 * qa1;
	volatile float q0q2 = qa0 * qa2;
	volatile float q0q3 = qa0 * qa3;
	volatile float q1q1 = qa1 * qa1;
	volatile float q1q2 = qa1 * qa2;
	volatile float q1q3 = qa1 * qa3;
	volatile float q2q2 = qa2 * qa2;
	volatile float q2q3 = qa2 * qa3;
	volatile float q3q3 = qa3 * qa3;

	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	// estimated direction of gravity and flux (v and w)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay * vz - az * vy) ;
	ey = (az * vx - ax * vz) ;
	ez = (ax * vy - ay * vx) ;

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{

		integralFBx += ex * twoKiDef * halftime;
		integralFBy += ey * twoKiDef * halftime;
		integralFBz += ez * twoKiDef * halftime;

		gx = gx + twoKpDef * ex + integralFBx;
		gy = gy + twoKpDef * ey + integralFBy;
		gz = gz + twoKpDef * ez + integralFBz;

	}
	// integrate quaternion rate and normalise
	temp0 = qa0 + (double)(-qa1 * gx - qa2 * gy - qa3 * gz) * halftime;
	temp1 = qa1 + (double)(qa0 * gx + qa2 * gz - qa3 * gy) * halftime;
	temp2 = qa2 + (double)(qa0 * gy - qa1 * gz + qa3 * gx) * halftime;
	temp3 = qa3 + (double)(qa0 * gz + qa1 * gy - qa2 * gx) * halftime;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	qa0 = temp0 * norm;
	qa1 = temp1 * norm;
	qa2 = temp2 * norm;
	qa3 = temp3 * norm;
}

/**************************Implement function ************************* ***********************
*Function prototype: void IMU_getQ(float * q)
*Features: Update quaternion Returns the current quaternion array value
Input parameters: The first address of the array to store the quaternion
Output parameters: no
************************************************** *****************************/
float mygetqval[9]; //Array for storing sensor conversion results
void IMU_getQ(float *q)
{

	IMU_getValues(mygetqval);
	// Turn the measured value of the gyroscope into radians per second
	//Acceleration and magnetometer hold ADC value No conversion required
	IMU_AHRSupdate(mygetqval[3] * M_PI / 180, mygetqval[4] * M_PI / 180, mygetqval[5] * M_PI / 180,
	mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

	FreeIMU_AHRSupdate(mygetqval[3] * M_PI / 180, mygetqval[4] * M_PI / 180, mygetqval[5] * M_PI / 180,
	mygetqval[0], mygetqval[1], mygetqval[2]);

	q[0] = qa0; //return the current value FreeIMU_AHRSupdate The calculated quaternion is used
	q[1] = qa1;
	q[2] = qa2;
	q[3] = qa3;
}

// a varient of asin() that checks the input ranges and guarantee a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
	if (isnan(v))
	{
	return 0.0f;
	}
	if (v >= 1.0f)
	{
	return M_PI / 2;
	}
	if (v <= -1.0f)
	{
	return -M_PI / 2;
	}
	return asin(v);
}


/**************************Implement function ************************* ***********************
*Function prototype: void IMU_getYawPitchRoll(float * angles)
*Function: Update quaternion returns the posture data after the current solution
Input parameters: the first address of the array where the attitude angle will be stored
Output parameters: no
************************************************** *****************************/
void IMU_getYawPitchRoll(float *angles)
{
	static float q[4]; // quaternion
	IMU_getQ(q); //Update global quaternion
	// angles[2] = IMU_Roll = ACC_Roll;
	// angles[1] = IMU_Pitch = ACC_Pitch;
	// angles[0] = IMU_Yaw;
	IMU_Roll = angles[2] = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
	1 - 2.0f * (q[1] * q[1] + q[2] * q[2]))) * 180 / M_PI;
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	IMU_Pitch = angles[1] = -safe_asin(2.0f * (q[0] * q[2] - q[3] * q[1])) * 180 / M_PI;

	IMU_Yaw = angles[0] = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw

	// if(IMU_Yaw <0)IMU_Yaw +=360.0f; // convert -+180 degrees to 0-360 degrees
}

/**************************Implement function ************************* ***********************
* Function prototype: void Initialize_Q(void)
*Function: Initialize quaternion with acceleration and compass
Input parameters: no
Output parameters: no
************************************************** *****************************/
static float acc[9];
void Initialize_Q()
{
	int i;
	volatile float temp, roll, pitch, yaw, yh, xh;
	for(i = 0; i < DATA_SIZE; i++)
	{
		IMU_getValues(acc);
		acc_X += acc[0];
		acc_Y += acc[1];
		acc_Z += acc[2];
		acc_MX += acc[6];
		acc_MY += acc[7];
		acc_MZ += acc[8];
	}
	acc_X /= DATA_SIZE;
	acc_Y /= DATA_SIZE;
	acc_Z /= DATA_SIZE;
	acc_MX /= DATA_SIZE;
	acc_MY /= DATA_SIZE;
	acc_MZ /= DATA_SIZE;

	temp = acc_X * invSqrt((acc_Y * acc_Y + acc_Z * acc_Z));
	pitch = atan(temp) * 57.3;

	temp = acc_Y * invSqrt((acc_X * acc_X + acc_Z * acc_Z));
	roll = atan(temp) * 57.3;

	yh = acc_MY * cos(roll) + acc_MZ * sin(roll);
	xh = acc_MX * cos(pitch) + acc_MY * sin(roll) * sin(pitch) - acc_MZ * cos(roll) * sin(pitch);
	yaw = atan2(yh, xh);
	// Initialize the quaternion, Euler angle to quaternion
	q0 = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q1 = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q2 = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
	q3 = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
}




//------------------End of File--------------------------- -
