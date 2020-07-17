#ifndef CAN_PARSER_H
#define CAN_PARSER_H


#include "can.h"
#include "can_config.h"


#define CAN_PARSER_HANDLE 					hcan1
#define CAN_ENABLE_PORT						GPIOA
#define CAN_ENABLE_PIN						GPIO_PIN_10
#define CAN_PARSER_MESSAGE_MAX_LENGTH		8
#define CAN_PARSER_BANK_START				14

#define CAN_PARSER_FLOAT_BYTES_AMOUNT		4
#define CAN_PARSER_BOOL_BYTES_AMOUNT		10
#define CAN_PARSER_TX_FIFO_LENGTH			500

// * 1/10 ms
#define CAN_PARSER_TX_FIFO_DELAY			3
#define CAN_PARSER_TX_PERIOD				300


typedef struct
{
	CAN_TxHeaderTypeDef		header;
	uint8_t*			    message;
	uint32_t				mailbox;
}TXFifo;



extern TXFifo 		CAN_PARSER_TXFifo[CAN_PARSER_TX_FIFO_LENGTH];
extern uint16_t		CAN_PARSER_TXFifoCounter;
extern bool 	    CAN_PARSER_TXFifoBusy;
extern uint32_t		CAN_PARSER_TXFifoDelayCounter;



void 	CAN_PARSER_Init();
void 	CAN_PARSER_SetFilterId(uint16_t id, uint16_t mask);
void 	CAN_PARSER_SetFilterMask(uint16_t id, uint16_t mask);
void    CAN_PARSER_AddMessageTX(uint16_t id, uint8_t* message, uint8_t bytesAmount);
bool 	CAN_PARSER_AddMessageTXFifo(uint16_t id, uint8_t* message, uint8_t bytesAmount);
bool 	CAN_PARSER_SendMessageFromFifo();
void    CAN_PARSER_InterpretMessage();
void 	CAN_PARSER_PrepareWholeFifo();
//void	CAN_PARSER_





#endif
