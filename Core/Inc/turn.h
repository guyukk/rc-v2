#ifndef _TURN_H
#define _TURN_H

#define PI 3.1415926
#include "math.h"

typedef struct{
	float T_F_param ;
	float T_S_param; 
}params,*params_t;

void params_init(params_t p, float T_F_p, float T_S_p);
float tern_angle(params_t p,float T_F_deviation, float T_S_deviation);
float change_angle(float left_side, float right_side , float side_angle);
float distance_Map(float T_S_deviation, float inMin, float inMax, float outMin, float	outMax);

#endif
