#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include "PID.h"

PID			PID_RegRoll;
PID			PID_RegPitch;
PID			PID_RegYaw;

float		PID_RollRequested;
float		PID_PitchRequested;
float		PID_YawRequested;
float		PID_XRequested;
float		PID_ZRequested;


int sgn(float v) {
	return (v < 0) ? -1 : ((v > 0) ? 1 : 0);
}

void PID_Init(PID *pid, float Kp, float Ki, float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->e = 0.0;
	pid->integral = 0;
	pid->derivative = 0.0;
	pid->proportional = 0.0;
	pid->y_pid = 0.0;
}

float PID_CalcOut(PID *pid, float angle_in, float current_angle)
{
	float e_before = pid->e;
	pid->e = PID_CalcError(angle_in,current_angle);

	//integral
	float integral_before = pid->integral;

	pid->integral = pid->integral + pid->Ki * pid->e * PID_DT;

	//proportional
	pid->proportional = pid->Kp * pid->e;

	//derivative
	pid->derivative = pid->Kd * (pid->e - e_before) / PID_DT;

	//sum it all together
	pid->y_pid = pid->integral + pid->proportional + pid->derivative;

	//wind-up
	pid->integral = pid->y_pid >= PID_MAX_Y ? integral_before : pid->integral;
	pid->integral = pid->y_pid <= PID_MIN_Y ? integral_before : pid->integral;

	//output saturation
	pid->y_pid = pid->y_pid >= PID_MAX_Y ? PID_MAX_Y : pid->y_pid;
	pid->y_pid = pid->y_pid <= PID_MIN_Y ? PID_MIN_Y : pid->y_pid;

	//assign previous input value
	//pid->e = e;

	return pid->y_pid;
}

//inputs in percent
//pid roll output, pid pitch output, pid yaw output, x, z
//outputs values directly to engines
void PID_SetEnginePowers(float pid_r, float pid_p, float pid_y, float x, float z)
{
		//calculate engine powers
		int fl = (int)(pid_r + pid_p - pid_y + z);
		int fr = (int)(-pid_r + pid_p + pid_y + z);
		int rl = (int)(pid_r - pid_p + pid_y + z);
		int rr = (int)(-pid_r - pid_p - pid_y + z);
		int tl = (int)(pid_y + x);
		int tr = (int)(-pid_y + x);

		//handle saturations and output values
		fl = fl > PID_MAX_Y ? PID_MAX_Y : ((fl < PID_MIN_Y) ? PID_MIN_Y : fl);
		fr = fr > PID_MAX_Y ? PID_MAX_Y : ((fr < PID_MIN_Y) ? PID_MIN_Y : fr);
		rl = rl > PID_MAX_Y ? PID_MAX_Y : ((rl < PID_MIN_Y) ? PID_MIN_Y : rl);
		rr = rr > PID_MAX_Y ? PID_MAX_Y : ((rr < PID_MIN_Y) ? PID_MIN_Y : rr);
		tl=  tl > PID_MAX_Y ? PID_MAX_Y : ((tl < PID_MIN_Y) ? PID_MIN_Y : tl);
		tr = tr > PID_MAX_Y ? PID_MAX_Y : ((tr < PID_MIN_Y) ? PID_MIN_Y : tr);

		ENGINE_SetSpeed(ENGINE_FL,fl);
		ENGINE_SetSpeed(ENGINE_FR,fr);
		ENGINE_SetSpeed(ENGINE_RL,rl);
		ENGINE_SetSpeed(ENGINE_RR,rr);
		ENGINE_SetSpeed(ENGINE_TL,tl);
		ENGINE_SetSpeed(ENGINE_TR,tr);

}

//prevent PID from going e.g 345 degrees instead of -15
float PID_CalcError(float angle_in, float current_angle)
{
	return sgn(current_angle - angle_in)*(fmodf(current_angle - fmodf(angle_in + 360.0, 360.0), 360.0));
}

//
//int main()
//{
//	srand(time(0));
//
//	//simulated engine powers
//	int FL = 0, FR = 0, BL = 0, BR = 0, L = 0, R = 0;
//
//
//	//create PID controllers
//	struct PID pid_roll;
//	struct PID pid_pitch;
//	struct PID pid_yaw;
//
//	//initialize controllers with proper gains and other parameters
//	init_pid(&pid_roll, 1, 0.40, 1.000);
//	init_pid(&pid_pitch, 1, 0.40, 1.000);
//	init_pid(&pid_yaw, 1, 0.40, 1.000);
//
//	//simulate inputs
//	float u = 0.0;
//	float x = 50.0, z = 30.0;
//	float imu_angle = 0.0;
//	float y_roll = 0.0, y_pitch = 0.0, y_yaw = 0.0;
//
//
//	while(1)
//	{
//		//calculate pid outputs
//		y_roll = calc_pid_y(&pid_roll, calc_error(u, imu_angle));
//		y_pitch = calc_pid_y(&pid_pitch, calc_error(u, imu_angle));
//		y_yaw = calc_pid_y(&pid_yaw, calc_error(u, imu_angle));
//
////		printf("%.6f ", y_roll);
//		//combine pid calculated powers for each engine
//		calc_engine_powers(y_roll, y_pitch, y_yaw, x, z, &FL, &FR, &BL, &BR, &L, &R);
////		printf("%d ", FL);
////		printf("%d ", FR);
////		printf("%d ", BL);
////		printf("%d ", BR);
////		printf("%d ", L);
////		printf("%d\n", R);
//		usleep(1000000);
//
//		//simulate object stabilization
//		imu_angle = imu_angle >= u ? u : imu_angle + 0.1;
//	}
//
//	return 0;
//}
