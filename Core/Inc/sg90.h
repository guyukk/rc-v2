#ifndef __S90_H__
#define __S90_H__

#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"

#define SG90_TIMER htim8
#define SG90_TIMER_CHANNEL TIM_CHANNEL_3

void SG90_Init(void);
void SG90_SetAngle(int angle);

#endif

