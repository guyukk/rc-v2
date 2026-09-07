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
void SG90_SetAngle(int angle){
	// center = 1150;
	// 1100 ~ 2300
	angle = -angle;
	if(angle < -30) angle = -30;
	if(angle > 30) angle = 30;
	angle *= 3;
	angle /= 2;
	
	int pulse = 750 + ((angle + 30) * 15);
	
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

