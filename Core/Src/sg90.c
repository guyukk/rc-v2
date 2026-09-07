//#include "sg90.h"

//#include "tim.h"

//void SG90_Init(void){
//	HAL_TIMEx_PWMN_Start(&SG90_TIMER,SG90_TIMER_CHANNEL);
//}
//void SG90_SetAngle(int angle){
//	// center = 1700;
//	// 1300 ~ 2100
//	if(angle < -30) angle = -30;
//	if(angle > 30) angle = 30;
//	
//	int pulse = 1400 + (( angle + 30) * 10);
//	
//	__HAL_TIM_SET_COMPARE(&SG90_TIMER,SG90_TIMER_CHANNEL,pulse);
//}

#include "sg90.h"

#include "tim.h"

void SG90_Init(void){
	HAL_TIMEx_PWMN_Start(&SG90_TIMER,SG90_TIMER_CHANNEL);
}

#if 1
/* 舵机摆正校准层（调试方法同 rc-car-v3 工程）：
 * - SG90_CENTER_OFFSET 只移动回正点：回中脉宽 = 基准脉宽 + offset，单位µs。
 *   1° ≈ 22µs，每次微调 ±10/±20，一次只调一个变量。
 *   方向（脉宽大偏左/偏右）因接线而异，需实测一次确定后写在注释里。
 * - SG90_MAX_ANGLE 只限制最大转向（°，顶机械限位就往下调），已移到 sg90.h，
 *   舵机扫动测试也共用它，改限幅只改头文件那一处。
 */
void SG90_SetAngle(int angle){
	// center = 1150;
	// 1100 ~ 2300
	angle = -angle;
	if(angle < -SG90_MAX_ANGLE) angle = -SG90_MAX_ANGLE;
	if(angle > SG90_MAX_ANGLE) angle = SG90_MAX_ANGLE;
	angle *= 3;
	angle /= 2;

	int pulse = 750 + ((angle + 30) * 15) + SG90_CENTER_OFFSET;

	__HAL_TIM_SET_COMPARE(&SG90_TIMER,SG90_TIMER_CHANNEL,pulse);
}

#else
void SG90_SetAngle(int angle){
	// center = 1700;
	// 1300 ~ 2100
	if(angle < -30) angle = -30;
	if(angle > 30) angle = 30;
	//angle *= 3;
	//angle /= 2;+
	
	int pulse = 1400 + ((angle + 30) * 10);
	__HAL_TIM_SET_COMPARE(&SG90_TIMER,SG90_TIMER_CHANNEL,pulse);
}

#endif

