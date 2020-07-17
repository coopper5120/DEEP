/*
 * RPI TO STM commands have even identifiers whereas STM TO RPI have odd identifiers
 */

//CAN BUS config
#define CAN_FRAME_LENGTH							8
#define CAN_BITRATE									1000000
#define CAN_STANDARD_FRAME							11
#define CAN_EXTENDED_FRAME							29

#define CAN_ADDR_OFFSET								80


//RPI TO STM_MAIN CONFIG
#define CAN_ENGINE_FL_POWER_REQUESTED 				80	//bool
#define CAN_ENGINE_FR_POWER_REQUESTED 				81	//bool
#define CAN_ENGINE_RL_POWER_REQUESTED 				82	//bool
#define CAN_ENGINE_RR_POWER_REQUESTED 				83	//bool
#define CAN_ENGINE_TL_POWER_REQUESTED 				84	//bool
#define CAN_ENGINE_TR_POWER_REQUESTED		 		85	//bool
#define CAN_ARM_POWER_REQUESTED 					86	//bool
#define CAN_MAGNETOMETER_POWER_REQUESTED 			87	//bool

//RPI TO ROBOT_ARM
#define CAN_ARM_ENABLE_REQUESTED					90	//bool

//STM_MAIN TO RPI
#define CAN_ENGINE_FL_POWER_CURRENT					100	//float
#define CAN_ENGINE_FR_POWER_CURRENT 				101	//float
#define CAN_ENGINE_RL_POWER_CURRENT 				102	//float
#define CAN_ENGINE_RR_POWER_CURRENT 				103 //float
#define CAN_ENGINE_TL_POWER_CURRENT 				104	//float
#define CAN_ENGINE_TR_POWER_CURRENT		 			105	//float
#define CAN_ARM_POWER_CURRENT 						106	//float
#define CAN_MAGNETOMETER_POWER_CURRENT 				107	//float

//STM_MAIN TO RPI
#define CAN_ENGINE_FL_CURRENT_CURRENT		 		110	//float
#define CAN_ENGINE_FR_CURRENT_CURRENT 				111	//float
#define CAN_ENGINE_RL_CURRENT_CURRENT 				112	//float
#define CAN_ENGINE_RR_CURRENT_CURRENT 				113	//float
#define CAN_ENGINE_TL_CURRENT_CURRENT 				114	//float
#define CAN_ENGINE_TR_CURRENT_CURRENT 				115	//float
#define CAN_ARM_CURRENT_CURRENT 					116	//float
#define CAN_MAGNETOMETER_CURRENT_CURRENT 			117	//float

//ROBOT_ARM TO RPI
#define CAN_ARM_GRIP_CURRENT_CURRENT				120	//float

//RPI TO STM_MAIN DATA
#define CAN_X_REQUESTED 							130	//float
#define CAN_Y_REQUESTED 							131	//float
#define CAN_Z_REQUESTED 							132	//float

//RPI TO ROBOT_ARM
#define CAN_ARM_OPEN_REQUESTED						140	//float
#define CAN_ARM_CLOSE_REQUESTED						141 //float
#define CAN_ARM_AXE1_REQUESTED						142 //float
#define CAN_ARM_AXE2_REQUESTED						143	//float
#define CAN_ARM_AXE3_REQUESTED						144	//float
#define CAN_ARM_AXE4_REQUESTED						145	//float

//RPI TO STM_MAIN DATA
#define CAN_ROLL_REQUESTED 							150	//float
#define CAN_PITCH_REQUESTED 						151	//float
#define CAN_YAW_REQUESTED	 						152	//float

//STM_MAIN TO RPI
#define CAN_ROLL_CURRENT 							200	//float
#define CAN_PITCH_CURRENT 							201	//float
#define CAN_YAW_CURRENT								202 //float

//RPI TO STM_MAIN PID CONFIG
#define CAN_ENGINE_FL_CURRENT_LIMIT_REQUESTED 		500	//float
#define CAN_ENGINE_FR_CURRENT_LIMIT_REQUESTED 		501 //float
#define CAN_ENGINE_RL_CURRENT_LIMIT_REQUESTED 		502	//float
#define CAN_ENGINE_RR_CURRENT_LIMIT_REQUESTED 		503	//float
#define CAN_ENGINE_TL_CURRENT_LIMIT_REQUESTED 		504	//float
#define CAN_ENGINE_TR_CURRENT_LIMIT_REQUESTED 		505	//float
#define CAN_ARM_CURRENT_LIMIT_REQUESTED 			506	//float
#define CAN_MAGNETOMETER_CURRENT_LIMIT_REQUESTED 	507	//float

//RPI TO STM_MAIN PID CONFIG
#define CAN_PID_ROLL_P_REQUESTED					521	//float
#define CAN_PID_ROLL_I_REQUESTED					522	//float
#define CAN_PID_ROLL_D_REQUESTED					523	//float
#define CAN_PID_PITCH_P_REQUESTED					524	//float
#define CAN_PID_PITCH_I_REQUESTED					525	//float
#define CAN_PID_PITCH_D_REQUESTED					526	//float
#define CAN_PID_YAW_P_REQUESTED						527	//float
#define CAN_PID_YAW_I_REQUESTED						528	//float
#define CAN_PID_YAW_D_REQUESTED						529	//float
#define CAN_PID_X_P_REQUESTED						530	//float
#define CAN_PID_X_I_REQUESTED						531	//float
#define CAN_PID_X_D_REQUESTED						532	//float
#define CAN_PID_Y_P_REQUESTED						533	//float
#define CAN_PID_Y_I_REQUESTED						534	//float
#define CAN_PID_Y_D_REQUESTED						535	//float
#define CAN_PID_Z_P_REQUESTED						536	//float
#define CAN_PID_Z_I_REQUESTED						537	//float
#define CAN_PID_Z_D_REQUESTED						538	//float

#define CAN_CALIBRATE_IMU_REQUESTED         550 //bool

//STM_MAIN TO RPI
#define CAN_ENGINE_FL_SPEED_CURRENT					580	//int
#define CAN_ENGINE_FR_SPEED_CURRENT					581	//int
#define CAN_ENGINE_RL_SPEED_CURRENT					582	//int
#define CAN_ENGINE_RR_SPEED_CURRENT					583	//int
#define CAN_ENGINE_TL_SPEED_CURRENT					584	//int
#define CAN_ENGINE_TR_SPEED_CURRENT					585	//int

//MAG_MODULE TO RPI
#define CAN_MAG_FIELD_1_CURRENT						600	//float
#define CAN_MAG_FIELD_2_CURRENT						601	//float
#define CAN_MAG_FIELD_3_CURRENT						602	//float
#define CAN_MAG_FIELD_4_CURRENT						603	//float

//RPI TO TORPEDOS
#define CAN_TORPEDO1_REQUESTED 						650 //bool
#define CAN_TORPEDO2_REQUESTED 						651 //bool
