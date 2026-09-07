/**
 ******************************************************************************
 * @file           : pid.c
 * @brief          : PID控制器实现
 * @description    : 提供PI控制器和PID控制器的实现
 ******************************************************************************
 */

#include "pid.h"

/**
 * @brief  初始化PID控制器
 * @param  ph: PID控制器句柄
 * @retval None
 * @note   将所有状态变量清零，为控制循环做准备
 */
void pid_init(pid_handle_t ph)
{		
	ph->output_val = 0.0f;    // 输出值清零
	ph->Error = 0.0f;          // 当前误差清零
	ph->LastError = 0.0f;      // 上次误差清零
	ph->integral = 0.0f;       // 积分项清零
}

/**
 * @brief  PI控制器实现（比例+积分控制）
 * @param  ph: PID控制器句柄
 * @param  actual_val: 当前实际值（反馈值）
 * @retval 控制器输出值
 * @note   
 *   - 使用比例项(P)快速响应误差
 *   - 使用积分项(I)消除稳态误差
 *   - 积分项使用当前误差，而非累加误差（避免积分饱和）
 *   - 适用于不需要微分控制的场景
 */
float pi_realize(pid_handle_t ph, float actual_val)
{
	// 1. 计算当前误差（目标值 - 实际值）
	ph->Error = ph->target_val - actual_val;
	
	// 2. 积分项计算
	// 注意：这里使用当前误差直接赋值，而非累加
	// 这种方式可以避免积分饱和，但会损失一些积分特性
	ph->integral = ph->Error;
	
	// 3. PI算法计算输出
	// 输出 = Kp * 误差 + Ki * 积分项
	ph->output_val = ph->Kp * ph->Error + 
	                 ph->Ki * ph->integral;
	
	// 4. 保存当前误差，供下次使用
	ph->LastError = ph->Error;
	
	// 5. 返回控制器输出
	return ph->output_val;
}

/**
 * @brief  完整PID控制器实现（比例+积分+微分控制）
 * @param  ph: PID控制器句柄
 * @param  actual_val: 当前实际值（反馈值）
 * @retval 控制器输出值
 * @note   
 *   - 比例项(P): 根据当前误差快速响应
 *   - 积分项(I): 消除稳态误差
 *   - 微分项(D): 预测误差变化趋势，提高系统稳定性
 *   - 积分项使用当前误差，避免积分饱和
 *   - 微分项计算误差变化率
 */
float pid_realize(pid_handle_t ph, float actual_val)
{
	// 1. 计算当前误差（目标值 - 实际值）
	ph->Error = ph->target_val - actual_val;
	
	// 2. 积分项计算
	// 使用当前误差直接赋值（非累加方式）
	ph->integral = ph->Error;

	// 3. 完整PID算法计算输出
	// 输出 = Kp * 误差 + Ki * 积分项 + Kd * 误差变化率
	// 其中：误差变化率 = 当前误差 - 上次误差
	ph->output_val = ph->Kp * ph->Error +                    // 比例项
	                 ph->Ki * ph->integral +                  // 积分项
	                 ph->Kd * (ph->Error - ph->LastError);    // 微分项
	
	// 4. 保存当前误差，供下次微分计算使用
	ph->LastError = ph->Error;
	
	// 5. 返回控制器输出
	return ph->output_val;
}

/**
 * @note 使用说明：
 * 
 * 1. 初始化：
 *    pid_handle pid;
 *    pid_init(&pid);
 *    pid.target_val = 目标值;
 *    pid.Kp = 比例系数;
 *    pid.Ki = 积分系数;
 *    pid.Kd = 微分系数;
 * 
 * 2. 控制循环中调用：
 *    float output = pid_realize(&pid, 当前实际值);
 *    // 将output应用到执行器
 * 
 * 3. 参数调试建议：
 *    - 先设置Kp，使系统有基本响应
 *    - 再加入Kd，提高稳定性，抑制振荡
 *    - 最后加入Ki，消除稳态误差
 */