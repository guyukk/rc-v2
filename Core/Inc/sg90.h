#ifndef __S90_H__
#define __S90_H__

#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"

#define SG90_TIMER htim8
#define SG90_TIMER_CHANNEL TIM_CHANNEL_3

/* 舵机最大转向限幅（°）：顶到机械限位就往下调。
 * 放在头文件里是为了让 sg90.c（校准层）和 sg90_test.c（扫动测试）共用，
 * 改限幅只需改这一处，测试自动跟随。 */
#define SG90_MAX_ANGLE 17

/* 舵机中心点偏移校准（µs）：调试方法：
 * - SG90_CENTER_OFFSET 只移动回正点：回中脉宽 = 基准脉宽 + offset，单位µs。
 *   1° ≈ 22µs，每次微调 ±10/±20，一次只调一个变量。
 *   方向（脉宽大偏左）
 * - 初始值为 0，调试时根据实际情况修改。 */
#define SG90_CENTER_OFFSET 440

void SG90_Init(void);
void SG90_SetAngle(int angle);

#endif

