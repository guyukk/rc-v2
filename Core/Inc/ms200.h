#ifndef __MS200_H__
#define __MS200_H__

#include "usart.h"

#define TOF_DATA_COUNT_N 12
#define TOF_PACKAGE_LENGTH (1 + 1 + 2 + 2 + 3 * TOF_DATA_COUNT_N + 2 + 2 + 1)
#define TOF_UART_RECV_BUFFER_SIZE (TOF_PACKAGE_LENGTH*2)

typedef struct{
	float angle;
	float distance;
}MS200_Point;

void MS200_SetRotationSpeed(uint8_t speed_hz);
void MS200_Init(void);
void MS200_ReceiveProcess(void);
MS200_Point* MS200_GetPointsData(void);
#endif

