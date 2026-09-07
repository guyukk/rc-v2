#include "turn.h"


//void params_init(params_t p, float T_F_p, float T_S_p){
//	
//	p->T_F_param = T_F_p;
//	p->T_S_param = T_S_p;
//	
//}


/**
  *@Brief 参数结构体，角度误差，左右距离误差
	*@Param p : params_t 权重结构体  
	*				T_F_deviation : float 通过左右前方获取的待转角度
	*				T_S_deviation : float 通过左右两侧获取的待转角度
  *@Return 返回小车转的角度
  *@exception none
  */
//float tern_angle(params_t p,float T_F_deviation, float T_S_deviation)
//{
//	return p->T_F_param * T_F_deviation + p->T_S_param * T_S_deviation;
//}	

/**
  *@Brief 规定左转返回正，右转负
	*@Param left_side : float side_angle方向上的左侧的距离
	*				right_side : float side_angle方向上的右侧的距离
	*				side_angle ：float 角度
  *@Return 通过side_angle方向的左右两侧距离，算出的待转角度
  *@exception none
  */ 
float change_angle(float left_side, float right_side , float side_angle)
{
	if(left_side == 0 && right_side == 0 )
		return 0;
	
	
	//第三条side
	float _side;
	//算出平行四边形两个相同角中的另外一个的弧度数值
	float radian_angle = ((180 - side_angle * 2) * PI) / 180.0;
	//c*c = a*a + b*b - 2*a*b*cos(C);
	_side = sqrt(right_side * right_side + left_side * left_side - 2 * right_side * left_side * cos(radian_angle));
	
	double cos_goal_angle;
	
	if(right_side > left_side){
		//右转
		cos_goal_angle = (right_side * right_side + _side * _side - left_side * left_side) * 1.0/ (2 * right_side * _side);
		return -(side_angle - (acos(cos_goal_angle) * 180) / PI);
	}
	else{
		//左转
		cos_goal_angle = (left_side * left_side + _side * _side - right_side * right_side) / (2 * left_side * _side);
		return side_angle - (acos(cos_goal_angle) * 180) / PI;
	}
		
}


/**
  *@Brief 距离映射5-30cm 对应 0-15度
  *@Param T_S_deviation ；float 计算的小车通过左右两侧距离差获取的待转角度
	*					inMin ： float 待转换的左端参数
	*					inMax ： float 待转换的右端参数
	*					outmin ： float 转换后的左端参数
	*					outmax ： float 转换后的右端参数
	*@Return 
  *@exception
  */

float distance_Map(float T_S_distance, float inMin, float inMax, float outMin, float	outMax){
	
	float value;
	//如果距离差大于 25 ，返回15 或者 -15
	if( fabs(T_S_distance) > 25){
		value = 15;
	}else{
		float normalizedValue = (fabs(T_S_distance) - inMin) / (inMax - inMin);
		value = normalizedValue * (outMax - outMin);
	}
	
	return T_S_distance > 0 ? value : - value; 
}

