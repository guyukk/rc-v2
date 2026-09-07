/**
 ******************************************************************************
 * @file           : motor.c
 * @brief          : 电机控制模块 - 包含编码器反馈和PWM速度控制
 * @details        : 使用定时器编码器模式读取电机转速，通过PWM控制电机正反转
 ******************************************************************************
 */

#include "motor.h"

/* ======================== 全局变量 ======================== */
Motor motor1;  // 电机状态结构体

/* ======================== 定时器回调函数 ======================== */

/**
 * @brief  定时器周期回调函数
 * @details 处理两种定时器中断：
 *          1. 编码器定时器溢出中断 - 防止计数溢出
 *          2. 间隔定时器中断 - 周期性计算电机速度
 * @param  htim: 定时器句柄指针
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* -------- 编码器定时器溢出处理 -------- */
    if (htim == &ENCODER_TIM)
    {
        // 判断溢出方向：向上溢出或向下溢出
        if (COUNTERNUM < 100000) 
        {
            motor1.overflowNum++;  // 向上溢出（正转时计数增大）
        }
        else if (COUNTERNUM >= 10000) 
        {
            motor1.overflowNum--;  // 向下溢出（反转时计数减小）
        }
        
        // 重置计数器到中间值，留出溢出缓冲空间
        __HAL_TIM_SetCounter(&ENCODER_TIM, 100000);
    }
    /* -------- 速度计算定时器中断 -------- */
    else if (htim == &GAP_TIM)
    {
        // 读取电机转向：0=正转，1=反转
        motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_TIM);
        
        // 计算总脉冲数 = 当前计数值 + 溢出次数 × 重装载值
        motor1.totalCount = COUNTERNUM + motor1.overflowNum * RELOADVALUE;
        
        // 计算线速度（单位：mm/s）
        // 公式：速度 = (脉冲增量 / (4 × 减速比 × 每圈脉冲数)) × 采样频率 × 轮周长系数
        // 说明：
        //   - 脉冲增量：本次计数 - 上次计数
        //   - 4：编码器四倍频
        //   - 10：采样频率(GAP_TIM是100ms中断，所以×10得到每秒)
        //   - LINE_SPEED_C：轮周长系数，将转速转换为线速度
        motor1.speed = -(float)(motor1.totalCount - motor1.lastCount) 
                       / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) 
                       * 10 
                       * LINE_SPEED_C;
        
        // 保存本次计数值，供下次计算使用
        motor1.lastCount = motor1.totalCount;
    }
}

/* ======================== 电机控制函数 ======================== */

/**
 * @brief  电机模块初始化
 * @details 初始化编码器定时器、PWM定时器和速度计算定时器
 *          设置编码器初始值为中间值10000，防止初始溢出
 * @retval None
 */
void MOTOR_Init(void)
{
    /* -------- 启动编码器定时器 -------- */
    HAL_TIM_Encoder_Start(&ENCODER_TIM, ENCODER_TIM_CHANNEL1);
    HAL_TIM_Encoder_Start(&ENCODER_TIM, ENCODER_TIM_CHANNEL2);
    
    // 使能编码器定时器更新中断，用于溢出处理
    __HAL_TIM_ENABLE_IT(&ENCODER_TIM, TIM_IT_UPDATE);
    
    // 设置编码器计数器初始值为10000（中间值）
    __HAL_TIM_SET_COUNTER(&ENCODER_TIM, 10000);
    
    /* -------- 启动PWM输出 -------- */
    HAL_TIM_PWM_Start(&PWM_TIM, PWM_TIM_CHANNEL1);  // 反转通道
    HAL_TIM_PWM_Start(&PWM_TIM, PWM_TIM_CHANNEL2);  // 正转通道
    
    /* -------- 启动速度计算定时器 -------- */
    HAL_TIM_Base_Start_IT(&GAP_TIM);  // 100ms周期中断
    
    /* -------- 初始化电机状态结构体 -------- */
    motor1.lastCount = 0;      // 上次计数值
    motor1.totalCount = 0;     // 总计数值
    motor1.overflowNum = 0;    // 溢出次数
    motor1.speed = 0;          // 当前速度
    motor1.direct = 0;         // 转向
}

/**
 * @brief  获取电机状态
 * @details 返回电机状态结构体指针，包含速度、方向等信息
 * @retval Motor* 电机状态结构体指针
 */
Motor* MOTOR_GetStatus(void)
{
    return &motor1;
}

/**
 * @brief  设置电机速度
 * @details 通过PWM占空比控制电机速度和方向
 *          正值=正转，负值=反转，0=停止
 * @param  present: 速度百分比 [-100, 100]
 *                  正值表示正转，负值表示反转
 *                  绝对值表示速度大小（0-100%）
 * @retval None
 * @note   速度范围会被限制在[-100, 100]之间
 */
void MOTOR_SetSpeed(int present)
{
    // 取反电机方向（根据硬件接线调整）
    present = -present;
    
    /* -------- 正转控制 -------- */
    if (present >= 0)
    {
        // 限制最大速度为100%
        if (present > 100)
        {
            present = 100;
        }
        
        // 设置正转PWM占空比，停止反转
        __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_TIM_CHANNEL2, 200 * present);  // 正转通道
        __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_TIM_CHANNEL1, 0);              // 反转通道关闭
    }
    /* -------- 反转控制 -------- */
    else
    {
        // 限制最大反转速度为-100%
        if (present < -100)
        {
            present = -100;
        }
        
        // 设置反转PWM占空比，停止正转
        __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_TIM_CHANNEL2, 0);                // 正转通道关闭
        __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_TIM_CHANNEL1, -200 * present);  // 反转通道（取绝对值）
    }
}