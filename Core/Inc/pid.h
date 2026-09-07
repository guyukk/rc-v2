#ifndef __PID_H__
#define __PID_H__

typedef struct
{
	float target_val;   //目标值
	float Error;          /*第 k 次偏差 */
	float LastError;     /* Error[-1],第 k-1 次偏差 */
	float PrevError;    /* Error[-2],第 k-2 次偏差 */
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
	float output_val;   //输出值
}pid_handle,*pid_handle_t;


void pid_init(pid_handle_t ph);
float pi_realize(pid_handle_t ph, float actual_val);
float pid_realize(pid_handle_t ph, float actual_val);

#endif
