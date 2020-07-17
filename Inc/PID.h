#ifndef PID_H
#define PID_H

#include "engine.h"



 #define PID_ROLL_INIT_P			1
 #define PID_ROLL_INIT_I			0
 #define PID_ROLL_INIT_D			0

 #define PID_PITCH_INIT_P			1
 #define PID_PITCH_INIT_I			0
 #define PID_PITCH_INIT_D			0

 #define PID_YAW_INIT_P				1
 #define PID_YAW_INIT_I				0
 #define PID_YAW_INIT_D				0

#define PID_MAX_Y		ENGINE_MAX_SPEED
#define PID_MIN_Y	    ENGINE_MIN_SPEED
#define PID_DT			0.01

//r -> + -> e -> PID ->_/- -> y ->OOBJECT----
//     - <- y <- SENSOR <-----------------/

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float integral;
	float derivative;
	float proportional;
	float e;
	float y_pid;
} PID;

extern PID			PID_RegRoll;
extern PID			PID_RegPitch;
extern PID			PID_RegYaw;

extern float		PID_RollRequested;
extern float		PID_PitchRequested;
extern float		PID_YawRequested;
extern float		PID_XRequested;
extern float		PID_ZRequested;




void 		PID_Init(PID *pid, float Kp, float Ki, float Kd);
float       PID_CalcOut(PID *pid, float angle_in, float current_angle);
void 		PID_SetEnginePowers(float pid_r, float pid_p, float pid_y, float x, float z);
float 		PID_CalcError(float angle_in, float current_angle);






#endif
