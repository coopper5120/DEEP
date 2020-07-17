#include "can_parser.h"
#include "can_config.h"
#include "module.h"
#include "engine.h"
#include "PID.h"
#include "can.h"

extern float* ypr;

uint8_t  CAN_FILTER_BankNumber = 0 ;
TXFifo 	 CAN_PARSER_TXFifo[CAN_PARSER_TX_FIFO_LENGTH];
uint16_t CAN_PARSER_TXFifoCounter = 0;
bool 	 CAN_PARSER_TXFifoBusy = false;
uint32_t CAN_PARSER_TXFifoDelayCounter = 1;

float MAGNETOMETER[11];


void CAN_PARSER_SetFilterId(uint16_t id, uint16_t mask)
{
	 CAN_FilterTypeDef	FilterConfig;


	  FilterConfig.FilterIdHigh = id<<5;          /*!< Specifies the filter identification number (MSBs for a 32-bit
	                                       configuration, first one for a 16-bit configuration).
	                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

	  FilterConfig.FilterIdLow = 0;           /*!< Specifies the filter identification number (LSBs for a 32-bit
	                                       configuration, second one for a 16-bit configuration).
	                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

	  FilterConfig.FilterMaskIdHigh = 0;      /*!< Specifies the filter mask number or identification number,
	                                       according to the mode (MSBs for a 32-bit configuration,
	                                       first one for a 16-bit configuration).
	                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

	  FilterConfig.FilterMaskIdLow = 0;       /*!< Specifies the filter mask number or identification number,
	                                       according to the mode (LSBs for a 32-bit configuration,
	                                       second one for a 16-bit configuration).
	                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

	  FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  /*!< Specifies the FIFO (0 or 1U) which will be assigned to the filter.
	                                       This parameter can be a value of @ref CAN_filter_FIFO */

	  FilterConfig.FilterBank=CAN_FILTER_BankNumber;            /*!< Specifies the filter bank which will be initialized.
	                                       For single CAN instance(14 dedicated filter banks),
	                                       this parameter must be a number between Min_Data = 0 and Max_Data = 13.
	                                       For dual CAN instances(28 filter banks shared),
	                                       this parameter must be a number between Min_Data = 0 and Max_Data = 27. */

	  FilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;            /*!< Specifies the filter mode to be initialized.
	                                       This parameter can be a value of @ref CAN_filter_mode */

	  FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;           /*!< Specifies the filter scale.
	                                       This parameter can be a value of @ref CAN_filter_scale */
	  FilterConfig.FilterActivation = ENABLE;

	  HAL_CAN_ConfigFilter(&CAN_PARSER_HANDLE, &FilterConfig);
	  CAN_FILTER_BankNumber++;
}

void 	CAN_PARSER_SetFilterMask(uint16_t id, uint16_t mask)
{
	 CAN_FilterTypeDef	FilterConfig;


		  FilterConfig.FilterIdHigh = id;          /*!< Specifies the filter identification number (MSBs for a 32-bit
		                                       configuration, first one for a 16-bit configuration).
		                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

		  FilterConfig.FilterIdLow = 0;           /*!< Specifies the filter identification number (LSBs for a 32-bit
		                                       configuration, second one for a 16-bit configuration).
		                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

		  FilterConfig.FilterMaskIdHigh = mask<<5;      /*!< Specifies the filter mask number or identification number,
		                                       according to the mode (MSBs for a 32-bit configuration,
		                                       first one for a 16-bit configuration).
		                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

		  FilterConfig.FilterMaskIdLow = 0;       /*!< Specifies the filter mask number or identification number,
		                                       according to the mode (LSBs for a 32-bit configuration,
		                                       second one for a 16-bit configuration).
		                                       This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

		  FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  /*!< Specifies the FIFO (0 or 1U) which will be assigned to the filter.
		                                       This parameter can be a value of @ref CAN_filter_FIFO */

		  FilterConfig.FilterBank=CAN_FILTER_BankNumber;            /*!< Specifies the filter bank which will be initialized.
		                                       For single CAN instance(14 dedicated filter banks),
		                                       this parameter must be a number between Min_Data = 0 and Max_Data = 13.
		                                       For dual CAN instances(28 filter banks shared),
		                                       this parameter must be a number between Min_Data = 0 and Max_Data = 27. */

		  FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;            /*!< Specifies the filter mode to be initialized.
		                                       This parameter can be a value of @ref CAN_filter_mode */

		  FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;           /*!< Specifies the filter scale.
		                                       This parameter can be a value of @ref CAN_filter_scale */
		  FilterConfig.FilterActivation = ENABLE;

		  HAL_CAN_ConfigFilter(&CAN_PARSER_HANDLE, &FilterConfig);
		  CAN_FILTER_BankNumber++;
}

void 	CAN_PARSER_AddMessageTX(uint16_t id, uint8_t* message, uint8_t bytesAmount)
{
	CAN_TxHeaderTypeDef		header;
	uint32_t mailbox;

	  header.DLC = bytesAmount;
	  header.IDE = CAN_ID_STD;
	  header.RTR = CAN_RTR_DATA;
	  header.StdId = id;
	  header.TransmitGlobalTime = DISABLE;

	HAL_CAN_AddTxMessage(&CAN_PARSER_HANDLE, &header, message, &mailbox);
}


bool 	CAN_PARSER_AddMessageTXFifo(uint16_t id, uint8_t* message, uint8_t bytesAmount)
{
	CAN_TxHeaderTypeDef		header;
	//uint32_t mailbox;

	  header.DLC = bytesAmount;
	  header.IDE = CAN_ID_STD;
	  header.RTR = CAN_RTR_DATA;
	  header.StdId = id;
	  header.TransmitGlobalTime = DISABLE;

	  if(CAN_PARSER_TXFifoCounter < CAN_PARSER_TX_FIFO_LENGTH)
	{
		  CAN_PARSER_TXFifo[CAN_PARSER_TXFifoCounter].header = header;
		  CAN_PARSER_TXFifo[CAN_PARSER_TXFifoCounter].message = message;
		  CAN_PARSER_TXFifoCounter++;
		  return true;
	}else return false;

	// HAL_CAN_AddTxMessage(&CAN_PARSER_HANDLE, &header, message, &mailbox);
}

bool 	CAN_PARSER_SendMessageFromFifo()
{
	if(CAN_PARSER_TXFifoBusy)
	{
		if(CAN_PARSER_TXFifoCounter>0)
		{
			CAN_PARSER_TXFifoCounter--;
			HAL_CAN_AddTxMessage(&CAN_PARSER_HANDLE, &CAN_PARSER_TXFifo[CAN_PARSER_TXFifoCounter].header, CAN_PARSER_TXFifo[CAN_PARSER_TXFifoCounter].message, &CAN_PARSER_TXFifo[CAN_PARSER_TXFifoCounter].mailbox);
		}
		else CAN_PARSER_TXFifoBusy = false;
	}
}

void 	CAN_PARSER_Init()
{
	//RESZTA DO DODANIA
	/*
	CAN_PARSER_SetFilterId(CAN_ENGINE_FL_POWER_REQUESTED);
	CAN_PARSER_SetFilterId(CAN_ENGINE_FR_POWER_REQUESTED);
	CAN_PARSER_SetFilterId(CAN_ENGINE_RL_POWER_REQUESTED);
	CAN_PARSER_SetFilterId(CAN_ENGINE_RR_POWER_REQUESTED);
	CAN_PARSER_SetFilterId(CAN_ENGINE_TL_POWER_REQUESTED);
    CAN_PARSER_SetFilterId(CAN_ENGINE_TR_POWER_REQUESTED);
    CAN_PARSER_SetFilterId(CAN_ARM_POWER_REQUESTED);
    CAN_PARSER_SetFilterId(CAN_MAGNETOMETER_POWER_REQUESTED);

    CAN_PARSER_SetFilterId(CAN_ROLL_REQUESTED);
    CAN_PARSER_SetFilterId(CAN_PITCH_REQUESTED);
    CAN_PARSER_SetFilterId(CAN_YAW_REQUESTED);
*/
	CAN_PARSER_SetFilterMask(0x0000,0x0000);
	CAN_PARSER_SetFilterId(0x0000,0x0000);
	CAN1->IER |= (1<<CAN_IER_TMEIE_Pos);  //enable TX interrupt
	HAL_GPIO_WritePin(CAN_ENABLE_PORT, CAN_ENABLE_PIN, 0);
	HAL_CAN_ActivateNotification(&CAN_PARSER_HANDLE, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&CAN_PARSER_HANDLE);
}

void CAN_PARSER_InterpretMessage()
{
	CAN_RxHeaderTypeDef header;
	uint8_t  message[CAN_PARSER_MESSAGE_MAX_LENGTH];
	HAL_CAN_GetRxMessage(&CAN_PARSER_HANDLE,CAN_RX_FIFO0,&header, message);
	uint32_t id = header.StdId;
	float  message_f = *((float*)message);
	int8_t message_i8= *((int8_t*)message);
	bool   message_b = *((bool*)message);


	switch	(id)
	{
		case  CAN_ENGINE_FL_POWER_REQUESTED: 		  MODULE_SetRequiredState(MODULE_ENGINE_FL,message_b);
											 		  break;
		case  CAN_ENGINE_FR_POWER_REQUESTED: 		  MODULE_SetRequiredState(MODULE_ENGINE_FR,message_b);
											 		  break;
		case  CAN_ENGINE_RL_POWER_REQUESTED: 		  MODULE_SetRequiredState(MODULE_ENGINE_RL,message_b);
											 		  break;
		case  CAN_ENGINE_RR_POWER_REQUESTED: 	      MODULE_SetRequiredState(MODULE_ENGINE_RR,message_b);
											 	      break;
		case  CAN_ENGINE_TL_POWER_REQUESTED: 		  MODULE_SetRequiredState(MODULE_ENGINE_TL,message_b);
											 		  break;
		case  CAN_ENGINE_TR_POWER_REQUESTED: 		  MODULE_SetRequiredState(MODULE_ENGINE_TR,message_b);
											 	      break;
		case  CAN_ARM_POWER_REQUESTED:		 	      MODULE_SetRequiredState(MODULE_ARM,message_b);
											 		  break;
		case  CAN_MAGNETOMETER_POWER_REQUESTED:		  MODULE_SetRequiredState(MODULE_MAGNETOMETER,message_b);
											 	 	  break;

		case CAN_ROLL_REQUESTED: 					  PID_RollRequested = message_f ;
	 												  break;
		case CAN_PITCH_REQUESTED: 				      PID_PitchRequested = message_f ;
	 												  break;
		case CAN_YAW_REQUESTED:	 					  PID_YawRequested = message_f ;
	 												  break;
		case CAN_X_REQUESTED:						  PID_XRequested = message_f ;
													  break;
		case CAN_Z_REQUESTED:						  PID_ZRequested = message_f ;
												      break;

		case CAN_PID_ROLL_P_REQUESTED:				  PID_RegRoll.Kp = message_f ;
													  break;
		case CAN_PID_ROLL_I_REQUESTED:				  PID_RegRoll.Ki = message_f ;
													  break;
		case CAN_PID_ROLL_D_REQUESTED:				  PID_RegRoll.Kd = message_f ;
													  break;
		case CAN_PID_PITCH_P_REQUESTED: 			  PID_RegPitch.Kp = message_f ;
		 											  break;
		case CAN_PID_PITCH_I_REQUESTED: 			  PID_RegPitch.Ki = message_f ;
		 											  break;
		case CAN_PID_PITCH_D_REQUESTED:				  PID_RegPitch.Kd = message_f ;
		 											  break;
		case CAN_PID_YAW_P_REQUESTED: 				  PID_RegYaw.Kp = message_f ;
		 								 			  break;
		case CAN_PID_YAW_I_REQUESTED: 				  PID_RegYaw.Ki = message_f ;
		 								    		  break;
		case CAN_PID_YAW_D_REQUESTED: 				  PID_RegYaw.Kd = message_f ;
		 								    		  break;

		case CAN_ENGINE_FL_CURRENT_LIMIT_REQUESTED:    MODULE_SetCurrentLimit(MODULE_ENGINE_FL,message_f);
	 												   break;
		case CAN_ENGINE_FR_CURRENT_LIMIT_REQUESTED:    MODULE_SetCurrentLimit(MODULE_ENGINE_FR,message_f);
													   break;
		case CAN_ENGINE_RL_CURRENT_LIMIT_REQUESTED:    MODULE_SetCurrentLimit(MODULE_ENGINE_RL,message_f);
	 												   break;
		case CAN_ENGINE_RR_CURRENT_LIMIT_REQUESTED:    MODULE_SetCurrentLimit(MODULE_ENGINE_RR,message_f);
				 									   break;
		case CAN_ENGINE_TL_CURRENT_LIMIT_REQUESTED:    MODULE_SetCurrentLimit(MODULE_ENGINE_TL,message_f);
				 									   break;
		case CAN_ENGINE_TR_CURRENT_LIMIT_REQUESTED:	   MODULE_SetCurrentLimit(MODULE_ENGINE_TR,message_f);
				 									   break;
		case CAN_ARM_CURRENT_LIMIT_REQUESTED:		   MODULE_SetCurrentLimit(MODULE_ARM,message_f);
				 									   break;
		case CAN_MAGNETOMETER_CURRENT_LIMIT_REQUESTED: MODULE_SetCurrentLimit(MODULE_MAGNETOMETER,message_f);
				 									   break;

		case 600: 									   MAGNETOMETER[0] = message_f;
						 							   break;
		case 601: 									   MAGNETOMETER[1] = message_f;
								 					   break;
		case 602: 									   MAGNETOMETER[2] = message_f;
								 					   break;
		case 603: 									   MAGNETOMETER[3] = message_f;
								 							   break;
		case 604: 									   MAGNETOMETER[4] = message_f;
								 							   break;
		case 605: 									   MAGNETOMETER[5] = message_f;
								 							   break;
		case 606: 									   MAGNETOMETER[6] = message_f;
								 							   break;
		case 607: 									   MAGNETOMETER[7] = message_f;
								 							   break;
		case 608: 									   MAGNETOMETER[8] = message_f;
								 							   break;
		case 609: 									   MAGNETOMETER[9] = message_f;
								 							   break;
		case 610: 									   MAGNETOMETER[10] = message_f;
								 							   break;
		case 611: 									   MAGNETOMETER[11] = message_f;
										 							   break;

	}
}

void CAN_PARSER_PrepareWholeFifo()
{
	//POSITION
	CAN_PARSER_AddMessageTXFifo(CAN_ROLL_CURRENT , &ypr[0], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_PITCH_CURRENT , &ypr[1], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_YAW_CURRENT, &ypr[2], CAN_PARSER_FLOAT_BYTES_AMOUNT);

	//MODULES ON/OFF
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_FL_POWER_CURRENT,     &MODULE_ActualState[MODULE_ENGINE_FL], CAN_PARSER_BOOL_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_FR_POWER_CURRENT,     &MODULE_ActualState[MODULE_ENGINE_FR], CAN_PARSER_BOOL_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_RL_POWER_CURRENT,     &MODULE_ActualState[MODULE_ENGINE_RL], CAN_PARSER_BOOL_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_RR_POWER_CURRENT,     &MODULE_ActualState[MODULE_ENGINE_RR], CAN_PARSER_BOOL_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_TL_POWER_CURRENT,     &MODULE_ActualState[MODULE_ENGINE_TL], CAN_PARSER_BOOL_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_TR_POWER_CURRENT,     &MODULE_ActualState[MODULE_ENGINE_TR], CAN_PARSER_BOOL_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ARM_POWER_CURRENT,           &MODULE_ActualState[MODULE_ARM], CAN_PARSER_BOOL_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_MAGNETOMETER_POWER_CURRENT,  &MODULE_ActualState[MODULE_MAGNETOMETER], CAN_PARSER_BOOL_BYTES_AMOUNT);

	//CURRENT
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_FL_CURRENT_CURRENT,    &MODULE_Current[MODULE_ENGINE_FL], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_FR_CURRENT_CURRENT,    &MODULE_Current[MODULE_ENGINE_FR], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_RL_CURRENT_CURRENT,    &MODULE_Current[MODULE_ENGINE_RL], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_RR_CURRENT_CURRENT,    &MODULE_Current[MODULE_ENGINE_RR], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_TL_CURRENT_CURRENT,    &MODULE_Current[MODULE_ENGINE_TL], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_TR_CURRENT_CURRENT,    &MODULE_Current[MODULE_ENGINE_TR], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ARM_CURRENT_CURRENT,          &MODULE_Current[MODULE_ARM], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_MAGNETOMETER_CURRENT_CURRENT, &MODULE_Current[MODULE_MAGNETOMETER], CAN_PARSER_FLOAT_BYTES_AMOUNT);

	// MOTORS SPEED
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_FL_SPEED_CURRENT, &ENGINE_Speed[ENGINE_FL], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_FR_SPEED_CURRENT, &ENGINE_Speed[ENGINE_FR], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_RL_SPEED_CURRENT, &ENGINE_Speed[ENGINE_RL], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_RR_SPEED_CURRENT, &ENGINE_Speed[ENGINE_RR], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_TL_SPEED_CURRENT, &ENGINE_Speed[ENGINE_TL], CAN_PARSER_FLOAT_BYTES_AMOUNT);
	CAN_PARSER_AddMessageTXFifo(CAN_ENGINE_TR_SPEED_CURRENT, &ENGINE_Speed[ENGINE_TR], CAN_PARSER_FLOAT_BYTES_AMOUNT);
}

