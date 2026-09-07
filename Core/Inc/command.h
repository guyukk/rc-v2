#ifndef __COMMAND_H__
#define __COMMAND_H__

#include "usart.h"

#define COMMAND_RECV_BUFFER_SIZE	128
#define COMMAND_OFFSET_CMD				0
#define COMMAND_OFFSET_DIRECTION	1
#define COMMAND_OFFSET_SPEED			2
#define COMMAND_OFFSET_ANGLE			2

enum{
	COMMAND_DIRECTION_FRONT = 0,
	COMMAND_DIRECTION_BACK = 1,
	COMMAND_DIRECTION_LEFT = 2,
	COMMAND_DIRECTION_RIGHT = 3,
};

enum{
	COMMAND_CMD_STOP = 0,
	COMMAND_CMD_SET_SPEED = 1,
	COMMAND_CMD_SET_ANGLE = 2,
};


void COMMAND_Init(void);
void COMMAND_Response(uint8_t code,uint8_t cmd);
void COMMAND_ReceiveProcess(void);

#endif


