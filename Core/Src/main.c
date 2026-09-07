/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sg90.h"
#include "ms200.h"
#include "motor.h"
#include "oled.h"
#include "Remote_OLED_display.h"
#include "pid.h"
#include "turn.h"
#include "linearRegression.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* 卡尔曼滤波器结构?? */
typedef struct {
    float Q;    // 过程噪声协方??
    float R;    // 测量噪声协方??
    float P;    // 估计误差协方??
    float K;    // 卡尔曼增??
    float X;    // 状态估计??
} KalmanFilter;

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef PI
#define PI 3.14159265358979323846
#endif

// ========== 基于速度的卡住检测参数 ==========
#define SPEED_STUCK_THRESHOLD 5.0f        // 速度阈值(mm/s)，低于此值认为可能被卡
#define SPEED_STUCK_CHECK_CYCLES 20       // 连续检测周期数(20*50ms=1秒)
#define SPEED_STUCK_MIN_COMMAND 20        // 最小命令速度，低于此值不检测卡住
#define SPEED_STUCK_BACK_TIME 1000         // 被卡后倒退时间(ms)
#define SPEED_STUCK_RECOVERY_ANGLE 35     // 被卡后恢复转向角度

// ========== 侧边蹭边保护（多角度检测）==========

#define SIDE_10_WARNING_DISTANCE 500   // 10°侧边警告距离(mm)
#define SIDE_10_AVOIDANCE_ANGLE 60     // 10°避让角度增量(度)

#define SIDE_15_WARNING_DISTANCE 400   // 15°侧边警告距离(mm)
#define SIDE_15_AVOIDANCE_ANGLE 60      // 15°避让角度增量(度)

#define SIDE_20_WARNING_DISTANCE 350   // 20°侧边警告距离(mm)
#define SIDE_20_AVOIDANCE_ANGLE 60      // 20°避让角度增量(度)

#define SIDE_60_WARNING_DISTANCE 200   // 60°侧边警告距离(mm)
#define SIDE_60_AVOIDANCE_ANGLE 10     // 60°避让角度增量(度)

#define SIDE_150_WARNING_DISTANCE 450  // 150°侧边警告距离(mm)
#define SIDE_150_AVOIDANCE_ANGLE 5    // 150°避让角度增量(度)，更大因为更侧面

// 分段预警距离定义
#define HOUTUI_WARNING 42
#define DEFAULT_WARRING_DISTANCE_CENTER 420   // 中心10°预警距离(0°-10°, 350°-360°)
#define DEFAULT_WARRING_DISTANCE_INNER  220   // 内侧预警距离(10°-20°, 340°-350°)
#define DEFAULT_WARRING_DISTANCE_OUTER  180   // 外侧预警距离(20°-45°, 315°-340°)
#define REVERSE_COEFFICIENT_CENTER 1.00f   // 中心区域后退系数
#define REVERSE_COEFFICIENT_INNER  1.29f  // 内侧区域后退系数
#define REVERSE_COEFFICIENT_OUTER  1.4f   // 外侧区域后退系数
#define ATTENUATION_COEFFICIENT_CENTER 0.5f   // 中心区域衰减系数
#define ATTENUATION_COEFFICIENT_INNER  0.85f   // 内侧区域衰减系数
#define ATTENUATION_COEFFICIENT_OUTER  0.86f   // 外侧区域衰减系数
#define NUM_ATTENUATION_CYCLES 15             // 衰减周期??

// 预警减速功??
#define JIANSU_DISTANCE 400         // 触发减速的距离阈??(mm)
#define SAFE_SPEED 35               // 减速后的安全速度
#define JIANSU_RECOVERY_DISTANCE 500 // 恢复正常速度的距离阈??(mm)

// 速度控制
#define DEFAULT_SPEED 32  //转弯时的速度
#define DEFAULT_HIGH_SPEED 35 //直道时的速度
#define DEFAULT_AFTER_BACK_ANGLE 30//默认倒车后偏离的角度
#define DEFAULT_BACK_SPEED 70
#define DEFAULT_BACK_HIGH_SPEED 70
#define DEFAULT_STOP_SPEED -100 //紧急止动速度

//曲率判断
#define CURVATURE_THRESHOLD_SHARP 600.0f   // 改小，提前识别急弯
#define CURVATURE_SHARP_EXIT  CURVATURE_THRESHOLD_SHARP * 0.90     // 退出急弯阈值（15%滞后??
#define CURVATURE_THRESHOLD_NORMAL 100.0f  // 改小，提前识别普通弯
#define CURVATURE_NORMAL_EXIT  CURVATURE_THRESHOLD_NORMAL * 0.85    // 退出普通弯阈值（15%滞后??
#define SHARP_DECELERATION_FACTER 0.7f
#define NORMAL_DECELERATION_FACTER 1.0f

#define STRAIGHT_ANGLE_MULTIPLIER 1.2f   // 直道状况下的转角放大系数（可调整，如1.2f表示放大20%）

//U弯检测变??
#define ENTER_U 800.0f         // U弯判定阈值（转角绝对值）
#define NUM_MORE_ANGLE 10.0f   // U弯转角放大系??
static int u_turn_state = 0;          // U弯状态标志：0=正常??1=U弯中

//曲率弯道检测变??
#define CURVATURE_ENTER_THRESHOLD 100.0f    // 曲率弯道判定阈??
#define CURVATURE_ANGLE_MULTIPLIER 1.6f    // 曲率弯道转角放大系数
static int curvature_turn_state = 0;       // 曲率弯道状态标志：0=正常??1=曲率弯道??

static int current_pid_mode = 1;  // 当前PID模式??1/2/3
static int pid_lock_timer = 0;    // PID切换锁定计时??
#define PID_LOCK_CYCLES 5          // PID切换后的锁定周期

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static KalmanFilter curvature_kf;
static float filtered_max_curvature = 0.0f;

// 减速功能状态变??
static int deceleration_state = 0;      // 减速状??: 0=正常, 1=减速中
static int normal_speed_backup = 0;     // 备份正常速度

/* USER CODE BEGIN PV */

// 基于速度的卡住检测变量
typedef struct {
    int stuck_counter;           // 卡住计数器
    int is_stuck;               // 卡住状态标志
    int last_command_speed;     // 上次命令速度
} SpeedStuckDetection;

static SpeedStuckDetection speed_stuck_detect = {
    .stuck_counter = 0,
    .is_stuck = 0,
    .last_command_speed = 0
};

//新增：拟合结果变??
FitResult left_fit_result = {FIT_LINEAR, {0,0,0,0}, 0, 0};
FitResult right_fit_result = {FIT_LINEAR, {0,0,0,0}, 0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float P, float initial_value);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void handle_missing_data(int i, MS200_Point* points, float *currentData, float *lastData);
int check_speed_stuck(Motor* motor, int command_speed);
void execute_speed_stuck_recovery(MS200_Point* points);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  基于电机速度检测小车是否被卡住
  * @param  motor: 电机状态结构体指针
  * @param  command_speed: 当前命令速度
  * @retval 1=被卡住, 0=正常
  */
int check_speed_stuck(Motor* motor, int command_speed)
{
    // 记录命令速度
    speed_stuck_detect.last_command_speed = command_speed;
    
    // 如果命令速度太小，不检测（可能是正常停车）
    if(abs(command_speed) < SPEED_STUCK_MIN_COMMAND){
        speed_stuck_detect.stuck_counter = 0;
        return 0;
    }
    
    // 获取当前实际速度（绝对值）
    float actual_speed = fabs(motor->speed);
    
    // 判断：命令速度足够大，但实际速度很小
    if(actual_speed < SPEED_STUCK_THRESHOLD){
        speed_stuck_detect.stuck_counter++;
        
        // 连续多次检测到速度过低，判定为卡住
        if(speed_stuck_detect.stuck_counter >= SPEED_STUCK_CHECK_CYCLES){
            return 1;  // 被卡住
        }
    }
    else{
        // 速度正常，重置计数器（使用衰减而不是直接清零，避免抖动）
        if(speed_stuck_detect.stuck_counter > 0){
            speed_stuck_detect.stuck_counter--;
        }
    }
    
    return 0;  // 正常
}

/**
  * @brief  执行速度卡住后的脱困动作
  * @param  points: 激光雷达数据点
  * @retval None
  */
void execute_speed_stuck_recovery(MS200_Point* points)
{
    // 停车
    MOTOR_SetSpeed(0);
    HAL_Delay(200);
    
    // ========== 读取左右35°后方距离 ==========
    float left_back_35, right_back_35;
    float left_back_35_last = 50, right_back_35_last = 50;
    
    // 左后35°（325°位置）
    handle_missing_data(406, points, &left_back_35, &left_back_35_last);
    
    // 右后35°（215°位置）
    handle_missing_data(269, points, &right_back_35, &right_back_35_last);
    
    // 检查后方是否安全
    if(left_back_35 < HOUTUI_WARNING || right_back_35 < HOUTUI_WARNING){
        // 后方也有障碍，尝试原地转向
        SG90_SetAngle(SPEED_STUCK_RECOVERY_ANGLE);
        MOTOR_SetSpeed(25);
        HAL_Delay(400);
        SG90_SetAngle(-SPEED_STUCK_RECOVERY_ANGLE);
        HAL_Delay(400);
        SG90_SetAngle(0);
        MOTOR_SetSpeed(0);
        HAL_Delay(100);
    }
    else{
        // 🔑 后方安全，保持当前角度后退（不改变舵机角度）
        // 注意：不调用 SG90_SetAngle()，保持小车卡住时的转向角度
        MOTOR_SetSpeed(-DEFAULT_BACK_SPEED);
        HAL_Delay(SPEED_STUCK_BACK_TIME);
        
        // 停止后退
        MOTOR_SetSpeed(0);
        HAL_Delay(100);
        
        // 前进调整位置（也保持当前角度）
        MOTOR_SetSpeed(30);
        HAL_Delay(300);
    }
    
    // 重置卡住检测状态
    speed_stuck_detect.stuck_counter = 0;
    speed_stuck_detect.is_stuck = 0;
}

/**
  * @brief  初始化卡尔曼滤波??
  * @param  kf: 卡尔曼滤波器结构体指??
  * @param  Q: 过程噪声协方?? (建议??: 0.01)
  * @param  R: 测量噪声协方?? (建议??: 0.1)
  * @param  P: 初始估计误差协方?? (建议??: 1.0)
  * @param  initial_value: 初始状态??
  * @retval None
  */
void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float P, float initial_value)
{
    kf->Q = Q;
    kf->R = R;
    kf->P = P;
    kf->K = 0;
    kf->X = initial_value;
}

/**
  * @brief  更新卡尔曼滤波器
  * @param  kf: 卡尔曼滤波器结构体指??
  * @param  measurement: 测量??
  * @retval 滤波后的??
  */
float KalmanFilter_Update(KalmanFilter *kf, float measurement)
{
    // 预测步骤
    // 状态预??: X(k|k-1) = X(k-1|k-1) (假设没有控制输入)
    // 协方差预??: P(k|k-1) = P(k-1|k-1) + Q
    kf->P = kf->P + kf->Q;
    
    // 更新步骤
    // 计算卡尔曼增??: K(k) = P(k|k-1) / (P(k|k-1) + R)
    kf->K = kf->P / (kf->P + kf->R);
    
    // 更新状态估??: X(k|k) = X(k|k-1) + K(k) * (Z(k) - X(k|k-1))
    kf->X = kf->X + kf->K * (measurement - kf->X);
    
    // 更新误差协方??: P(k|k) = (1 - K(k)) * P(k|k-1)
    kf->P = (1 - kf->K) * kf->P;
    
    return kf->X;
}

#define UART2_RECV_BUFFER_SIZE 96

static uint8_t uart2_recv_buffer[UART2_RECV_BUFFER_SIZE];
Remote_OLED_display_param_handle_t remote_oled_display_handle;

static pid_handle  center_pid;

//左右前方转换角度的pid
static pid_handle  angle_pid ;

//左右前方转换角度的pid参数
static float angle_pid_1[3] = { 0.35 , 0.02 , 0.40};
static float angle_pid_2[3] = { 0.55 , 0.02 , 0.40};
static float angle_pid_3[3] = { 0.60 , 0.02 , 0.40};

/**
  * @brief  应用指定的PID参数模式
  */
void apply_pid_mode(pid_handle *pid, int mode)
{
    switch(mode){
        case 1:
            pid->Kp = angle_pid_1[0];
            pid->Ki = angle_pid_1[1];
            pid->Kd = angle_pid_1[2];
            break;
        case 2:
            pid->Kp = angle_pid_2[0];
            pid->Ki = angle_pid_2[1];
            pid->Kd = angle_pid_2[2];
            break;
        case 3:
            pid->Kp = angle_pid_3[0];
            pid->Ki = angle_pid_3[1];
            pid->Kd = angle_pid_3[2];
            break;
        default:
            pid->Kp = angle_pid_1[0];
            pid->Ki = angle_pid_1[1];
            pid->Kd = angle_pid_1[2];
            break;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t response[32] = {0};
	size_t response_len = 32;
	if (huart->Instance == USART2)
	{
		remote_oled_display_hexdump("UART-RX",uart2_recv_buffer,Size);
		if(!remote_oled_display_process(remote_oled_display_handle,uart2_recv_buffer,Size,response,&response_len)){
			remote_oled_display_hexdump("UART-TX",response,response_len);
			HAL_UART_Transmit(&huart2, response,response_len,0xffff);
		}
		memset(uart2_recv_buffer,0,UART2_RECV_BUFFER_SIZE);
	}
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART5)
	{
		MS200_ReceiveProcess();
	}
}


//计时模块******************************************

//计时模块******************************************



// 定义处理雷达返回数据点的数量
#define SIZE 9

DataPoint LidarRightpoints[SIZE + 1];
DataPoint LidarLeftpoints[SIZE + 1];

// 定义存储??5degree开始逐次增加5，至60degree的正余弦??
static double sin_values[9] = {  
    0.3420201433, 0.4226182617, 0.5000000000,  
    0.5735764364, 0.6427876097, 0.7071067812, 
	  0.7660444431, 0.8191520443, 0.8660254038
};
static double cos_values[9] = {  
    0.9396926208, 0.9063077870, 0.8660254038,  
    0.8191520443, 0.7660444431, 0.7071067812,
		0.6427876097, 0.5735764364, 0.5000000000
};

// 滤波和去噪的简化函??
void filterAndDenoise2D(DataPoint* points, int size) {
    // 这里只是一个示例，实际滤波去噪会更复杂
    for (int i = 0; i < size; ++i) {
        // 根据具体规则去除噪声??
        // 例如，假设我们移除超出某个范围的??
        if (points[i].xDistance < 0 || points[i].yDistance < 0) {
            points[i].xDistance = 0;
            points[i].yDistance = 0;
        }
    }
}

void handle_missing_data(int i , MS200_Point* points , float *currentData , float *lastData){
	*currentData = points[i].distance / 10;
	if(*currentData != 0){
		*lastData = *currentData;
	}else{
		if(*lastData != 0)
			*currentData = *lastData;
		else
			*currentData = 50;
	}
}
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_UART5_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    KalmanFilter_Init(&curvature_kf, 0.01f, 0.1f, 1.0f, 0.0f);
	SG90_Init();
	MS200_Init();
	MOTOR_Init();
	OLED_Init();

    MS200_SetRotationSpeed(12); 
	SG90_SetAngle(0);
	//*******************css
	
	//初始化pid
	pid_init(&angle_pid);
	pid_init(&center_pid);
	//pid_init(&speed_pid);
	//左右前方测得的对角线??0度线的角度为0
	
	
	//angle初始??
	angle_pid.target_val = 0;
	angle_pid.Kp = angle_pid_1[0];
	angle_pid.Ki = angle_pid_1[1];
	angle_pid.Kd = angle_pid_1[2];
	//center初始??
	center_pid.target_val = 0.5;
	center_pid.Kp = 5;
	center_pid.Ki = 5;
	center_pid.Kd = 5;
	
	remote_oled_display_init(&remote_oled_display_handle);
    remote_oled_display_set_key(remote_oled_display_handle, NULL, 0); 
    remote_oled_display_oled_display_init(remote_oled_display_handle); 
    
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float left_reverse_coefficient = REVERSE_COEFFICIENT_OUTER;   // 左侧后退系数
	float right_reverse_coefficient = REVERSE_COEFFICIENT_CENTER; // 右侧后退系数

	float distance_left_90,distance_right_90; //mm
	float distance_left_90_last,distance_right_90_last; //mm
	float center_percent;

	// 判断当转角时，一侧与赛道近乎平行
	float distance_L90,distance_L110,distance_L80;
	float distance_L90_last,distance_L110_last,distance_L80_last;

	float distance_R90,distance_R110,distance_R80;
	float distance_R90_last,distance_R110_last,distance_R80_last;

	// 通过去除丢失点，采取临近次数的有效值作为??
	float left_30_distance , left_40_distance ,left_50_distance;
	float left_30_distance_last , left_40_distance_last , left_50_distance_last;

	float right_30_distance , right_40_distance , right_50_distance;
	float right_30_distance_last , right_40_distance_last , right_50_distance_last;

	// 左后??
	float distance_200 , distance_197 , distance_203;
	float distance_200_last, distance_197_last , distance_203_last;
	float distance_left_back_200;

	// 右后??
	float distance_160 , distance_157 , distance_163;
	float distance_160_last, distance_157_last , distance_163_last;
	float distance_right_back_160;

	DataPoint Last_LidarRightpoints[SIZE + 1];
	DataPoint Last_LidarLeftpoints[SIZE + 1];

	// 计时
	int timer = 0;
	int flag = 0;//-1 : ??    1 : ?? 

	// === 修改：使用三段式预警距离 ===
	float distance_center = DEFAULT_WARRING_DISTANCE_CENTER;  // 中心预警距离
	float distance_inner = DEFAULT_WARRING_DISTANCE_INNER;    // 内侧预警距离
	float distance_outer = DEFAULT_WARRING_DISTANCE_OUTER;    // 外侧预警距离
	int distance_timer = 0;

	int speed = DEFAULT_HIGH_SPEED;	
	HAL_Delay(1000);

while (1){
    // ========== 变量声明（在循环开始处） ==========
    float left_min_distance = 1000.0f;
    float right_min_distance = 1000.0f;
    float left_detected_warning = DEFAULT_WARRING_DISTANCE_OUTER;
    float right_detected_warning = DEFAULT_WARRING_DISTANCE_CENTER;
    int min_left_point = 0;
    int min_right_point = 0;
    
    if(distance_timer != 0){
        distance_timer++;
        // 在衰减周期内，各区域使用各自的衰减系数
        distance_center = DEFAULT_WARRING_DISTANCE_CENTER * ATTENUATION_COEFFICIENT_CENTER;
        distance_inner = DEFAULT_WARRING_DISTANCE_INNER * ATTENUATION_COEFFICIENT_INNER;
        distance_outer = DEFAULT_WARRING_DISTANCE_OUTER * ATTENUATION_COEFFICIENT_OUTER;
    }

    if(distance_timer > NUM_ATTENUATION_CYCLES){
        // 超过衰减周期后，恢复正常预警距离
        distance_timer = 0;
        distance_center = DEFAULT_WARRING_DISTANCE_CENTER;
        distance_inner = DEFAULT_WARRING_DISTANCE_INNER;
        distance_outer = DEFAULT_WARRING_DISTANCE_OUTER;
    }
    
    Motor* motor1 = MOTOR_GetStatus();
    MS200_Point* points = MS200_GetPointsData();
    HAL_Delay(50);

	    // ========== 【在这里插入】基于速度的卡住检测 ==========
    if(check_speed_stuck(motor1, speed)){
        // 检测到被卡住，执行脱困
        speed_stuck_detect.is_stuck = 1;
        
        // 刷新雷达数据
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
        points = MS200_GetPointsData();
        
        // 执行脱困动作
        execute_speed_stuck_recovery(points);
        
        // 重新读取雷达数据
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
        points = MS200_GetPointsData();
        HAL_Delay(50);
        
        continue;  // 跳过本次循环，重新开始
    }
    // ========== 速度卡住检测结束 ==========
		
		//speed = DEFAULT_HIGH_SPEED;
		
		//计算左后轮辐射方向的距离
		handle_missing_data(250 , points , &distance_200 , &distance_200_last);	// 200/0.8=250
		handle_missing_data(246 , points , &distance_197 , &distance_197_last);	// 197/0.8=246.25≈246
		handle_missing_data(254 , points , &distance_203 , &distance_203_last);	// 203/0.8=253.75≈254
		
		distance_left_back_200 = (distance_200 + distance_197 + distance_203) / 3;
		

		//计算右后轮辐射方向的距离
		handle_missing_data(200 , points , &distance_160 , &distance_160_last);	// 160/0.8=200
		handle_missing_data(196 , points , &distance_157 , &distance_157_last);	// 157/0.8=196.25≈196
		handle_missing_data(204 , points , &distance_163 , &distance_163_last);	// 163/0.8=203.75≈204
		
		distance_right_back_160 = (distance_160 + distance_157 + distance_163) / 3;
		
		for(int i=0; i<SIZE; i++){
			// 右侧角度：20°+i*5° ，转换为索引 (20+i*5)/0.8
		int right_angle_index = (int)floor((20 + i*5) / 0.8);  // 右侧使用向下取整
			double right_distance_cm = points[right_angle_index].distance / 10.0;
			LidarRightpoints[i].xDistance = right_distance_cm * cos_values[i];
			LidarRightpoints[i].yDistance = right_distance_cm * sin_values[i];
			
			// 非零记录逻辑保持不变
			if(LidarRightpoints[i].xDistance != 0){
				Last_LidarRightpoints[i].xDistance = LidarRightpoints[i].xDistance;
			}else{
				LidarRightpoints[i].xDistance = Last_LidarRightpoints[i].xDistance;
			}
			
			if(LidarRightpoints[i].yDistance != 0){
				Last_LidarRightpoints[i].yDistance = LidarRightpoints[i].yDistance;
			}else{
				LidarRightpoints[i].yDistance = Last_LidarRightpoints[i].yDistance;
			}
			
			// 左侧角度：340°-i*5° ，转换为索引 (340-i*5)/0.8
			int left_angle_index = (int)ceil((340 - i*5) / 0.8);
			double left_distance_cm = points[left_angle_index].distance / 10.0;
			LidarLeftpoints[i].xDistance = left_distance_cm * cos_values[i];
			LidarLeftpoints[i].yDistance = left_distance_cm * sin_values[i];
			
			if(LidarLeftpoints[i].xDistance != 0){
				Last_LidarLeftpoints[i].xDistance = LidarLeftpoints[i].xDistance;
			}else{
				LidarLeftpoints[i].xDistance = Last_LidarLeftpoints[i].xDistance;
			}
			
			if(LidarLeftpoints[i].yDistance != 0){
				Last_LidarLeftpoints[i].yDistance = LidarLeftpoints[i].yDistance;
			}else{
				LidarLeftpoints[i].yDistance = Last_LidarLeftpoints[i].yDistance;
			}
		}
		
	    // 数据预处理（保持不变）
        filterAndDenoise2D(LidarRightpoints, SIZE);
        filterAndDenoise2D(LidarLeftpoints, SIZE);

        // ========== 新版：自适应拟合 ==========
        adaptiveFitting(LidarRightpoints, SIZE, &right_fit_result);
        adaptiveFitting(LidarLeftpoints, SIZE, &left_fit_result);

        // 提取曲率值（用于后续逻辑）
        float curvatureRight = right_fit_result.curvature / 100;
        float curvatureLeft = left_fit_result.curvature / 100;

		// 计算原始最大曲率
		float raw_max_curvature = (curvatureLeft > curvatureRight) ? curvatureLeft : curvatureRight;

		// 应用卡尔曼滤波
		filtered_max_curvature = KalmanFilter_Update(&curvature_kf, raw_max_curvature);
		

		// ========== 减速功能:检测左右10°范围的距离 ==========
		// 检测左侧350°-360°(10°范围)和右侧0°-10°(10°范围)
		float left_10deg_min = 1000.0f;
		float right_10deg_min = 1000.0f;

		// 左侧350°-360°范围检测
		for(int angle = 350; angle <= 360; angle++){
			int left_index = (int)ceil(angle / 0.8);
			if(points[left_index].distance > 10.0f && 
			points[left_index].distance < left_10deg_min){
				left_10deg_min = points[left_index].distance;
			}
		}

		// 右侧0°-10°范围检测
		for(int angle = 0; angle <= 10; angle++){
			int right_index = (int)floor(angle / 0.8);
			if(points[right_index].distance > 10.0f && 
			points[right_index].distance < right_10deg_min){
				right_10deg_min = points[right_index].distance;
			}
		}

		// 计算中心最小距离
		float min_center_distance = (left_10deg_min < right_10deg_min) ? 
									left_10deg_min : right_10deg_min;

		// 减速状态机
		if(deceleration_state == 0){
			// 正常状态：检测是否需要减速
			if(min_center_distance < JIANSU_DISTANCE){
				// 触发减速：进入减速状态
				deceleration_state = 1;
				normal_speed_backup = speed;  // 备份当前速度
				speed = SAFE_SPEED;           // 直接设置为安全速度
				MOTOR_SetSpeed(speed);		
			}
		}
		else if(deceleration_state == 1){
			// 减速状态：持续使用安全速度，并检测是否可以恢复
			speed = SAFE_SPEED;
			MOTOR_SetSpeed(speed);	
			
			// 检测是否可以恢复正常速度
			if(min_center_distance > JIANSU_RECOVERY_DISTANCE){
				// 距离恢复安全，退出减速状态
				deceleration_state = 0;
				speed = normal_speed_backup;  // 恢复正常速度
				MOTOR_SetSpeed(speed);	
			}	
		}
	
	
    // ========== 前方距离检测 - 三段预警 ==========
    float left_warning_distance = distance_outer;   // 左侧当前预警距离
    float right_warning_distance = distance_center; // 右侧当前预警距离
    
	// 距离判断 - 三段处理（修改后的版本）
	for(int i=0; i<56; i++){  // 45°范围：45/0.8=56.25≈56个点
		
		// === 左前方处理：315°-360° ===
		int left_angle = 315 + i;  // 当前角度
		int left_front_index = (int)ceil((315 + i) / 0.8);
		
		// 确定左侧预警距离 (三段式)
		if(left_angle < 340){
			left_warning_distance = distance_outer;
		} else if(left_angle < 350){
			left_warning_distance = distance_inner;
		} else {
			left_warning_distance = distance_center;
		}
		
		// 检测左侧最小距离（考虑对应的预警距离）
		if(points[left_front_index].distance <= left_min_distance && 
		points[left_front_index].distance >= 10.0f &&
		points[left_front_index].distance < left_warning_distance){
			left_min_distance = points[left_front_index].distance;
			min_left_point = i;
			left_detected_warning = left_warning_distance;  // 记录此时的预警距离
			
			// *** 新增：根据角度设置对应的后退系数 ***
			if(left_angle < 340){
				left_reverse_coefficient = REVERSE_COEFFICIENT_OUTER;
			} else if(left_angle < 350){
				left_reverse_coefficient = REVERSE_COEFFICIENT_INNER;
			} else {
				left_reverse_coefficient = REVERSE_COEFFICIENT_CENTER;
			}
		}
		
		// === 右前方处理：0°-45° ===
		int right_angle = i;  // 当前角度
		int right_front_index = (int)floor(i / 0.8);
		
		// 确定右侧预警距离 (三段式)
		if(right_angle < 10){
			right_warning_distance = distance_center;
		} else if(right_angle < 20){
			right_warning_distance = distance_inner;
		} else {
			right_warning_distance = distance_outer;
		}
		
		// 检测右侧最小距离（考虑对应的预警距离）
		if(points[right_front_index].distance <= right_min_distance && 
		points[right_front_index].distance >= 10.0f &&
		points[right_front_index].distance < right_warning_distance){
			right_min_distance = points[right_front_index].distance;
			min_right_point = i;
			right_detected_warning = right_warning_distance;  // 记录此时的预警距离
			
			// *** 新增：根据角度设置对应的后退系数 ***
			if(right_angle < 10){
				right_reverse_coefficient = REVERSE_COEFFICIENT_CENTER;
			} else if(right_angle < 20){
				right_reverse_coefficient = REVERSE_COEFFICIENT_INNER;
			} else {
				right_reverse_coefficient = REVERSE_COEFFICIENT_OUTER;
			}
		}
	}


    // ========== PID参数自适应管理（移到这里！）==========
    
    // 1. 更新锁定计时器
    if(pid_lock_timer > 0){
        pid_lock_timer--;
    }
    
    // 2. 根据曲率和距离共同决定目标PID模式
    int target_pid_mode = current_pid_mode;  // 默认保持当前模式
    
    // ===== 新增：紧急距离升级优先级最高 =====
    // 当距离小于1.5倍预警距离时，强制升级到急弯模式
    int emergency_distance_upgrade = 0;
    if((left_min_distance > 10.0f && left_min_distance < left_detected_warning * 1.5f) ||
       (right_min_distance > 10.0f && right_min_distance < right_detected_warning * 1.5f)){
        emergency_distance_upgrade = 1;
        target_pid_mode = 3;  // 紧急情况，直接升级到最高级
    }
    
    // ===== 如果不是紧急距离情况，按曲率正常判断 =====
    if(!emergency_distance_upgrade){
        if(current_pid_mode == 3){
            // 当前在急弯模式，使用退出阈值（避免抖动）
            if(filtered_max_curvature < CURVATURE_SHARP_EXIT){
                // 判断是否降到普通弯
                if(filtered_max_curvature >= CURVATURE_NORMAL_EXIT){
                    target_pid_mode = 2;
                } else {
                    target_pid_mode = 1;
                }
            }
        }
        else if(current_pid_mode == 2){
            // 当前在普通弯模式
            if(filtered_max_curvature >= CURVATURE_THRESHOLD_SHARP){
                target_pid_mode = 3;  // 升级到急弯
            }
            else if(filtered_max_curvature < CURVATURE_NORMAL_EXIT){
                target_pid_mode = 1;  // 降级到直道
            }
        }
        else {  // current_pid_mode == 1
            // 当前在直道模式
            if(filtered_max_curvature >= CURVATURE_THRESHOLD_SHARP){
                target_pid_mode = 3;
            }
            else if(filtered_max_curvature >= CURVATURE_THRESHOLD_NORMAL){
                target_pid_mode = 2;
            }
        }
    }
    
    // 3. 只在解锁状态下切换，且模式确实改变时才切换
    if(pid_lock_timer == 0 && target_pid_mode != current_pid_mode){
        current_pid_mode = target_pid_mode;
        apply_pid_mode(&angle_pid, current_pid_mode);
        
        // 紧急距离升级时使用正常锁定周期
        if(emergency_distance_upgrade){
            pid_lock_timer = PID_LOCK_CYCLES;
        } else {
            pid_lock_timer = PID_LOCK_CYCLES;
        }
    }


	// ========== 左前方障碍物后退逻辑修改 ==========

	if(left_min_distance < left_detected_warning && 
	left_min_distance < right_min_distance && 
	left_min_distance > 10.0f){
		
		distance_timer++;
        // 修改：记录避障前的PID模式（用于调试/日志）
        int pid_before_obstacle = current_pid_mode;
        
        // 修改：强制切换到急弯模式
        current_pid_mode = 3;
        apply_pid_mode(&angle_pid, 3);
        
		// 紧急制动
		MOTOR_SetSpeed(DEFAULT_STOP_SPEED);
		HAL_Delay(150);
		MOTOR_SetSpeed(0);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
		points = MS200_GetPointsData();
		
		// 计算激光雷达在90度附近的距离
		handle_missing_data(338 , points , &distance_L90 , &distance_L90_last);
		handle_missing_data(350 , points , &distance_L80 , &distance_L80_last);
		handle_missing_data(325 , points , &distance_L110 , &distance_L110_last);
		
		if(distance_left_back_200 < HOUTUI_WARNING || distance_right_back_160 < HOUTUI_WARNING){
			// 后方过近，不后退
		}else{
			MOTOR_SetSpeed(-DEFAULT_BACK_SPEED);
			HAL_Delay(100);
			SG90_SetAngle(-DEFAULT_AFTER_BACK_ANGLE);
						
			int left_check_index = (int)ceil((315 + min_left_point) / 0.8);
			
			do{
				HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
				points = MS200_GetPointsData();
				
				// 实时更新后方距离
				handle_missing_data(250 , points , &distance_200 , &distance_200_last);
				handle_missing_data(246 , points , &distance_197 , &distance_197_last);
				handle_missing_data(254 , points , &distance_203 , &distance_203_last);
				distance_left_back_200 = (distance_200 + distance_197 + distance_203) / 3;
				
				handle_missing_data(200 , points , &distance_160 , &distance_160_last);
				handle_missing_data(196 , points , &distance_157 , &distance_157_last);
				handle_missing_data(204 , points , &distance_163 , &distance_163_last);
				distance_right_back_160 = (distance_160 + distance_157 + distance_163) / 3;
				
				// 检查退出条件：前方安全 OR 后方过近
				if(distance_left_back_200 < HOUTUI_WARNING || distance_right_back_160 < HOUTUI_WARNING){
					break;
				}
				
				// *** 修改：使用左侧对应的后退系数 ***
			}while(points[left_check_index].distance < left_detected_warning * left_reverse_coefficient);
			
		}			
		
		// 规整位置
		SG90_SetAngle(DEFAULT_AFTER_BACK_ANGLE);
		MOTOR_SetSpeed(DEFAULT_BACK_HIGH_SPEED);
		HAL_Delay(150);
		SG90_SetAngle(-30);
		MOTOR_SetSpeed(30);

        // 重新读取雷达数据并计算曲率（确保使用最新数据）
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
        points = MS200_GetPointsData();
        
        // 根据当前曲率重新选择PID模式（不使用滞后，直接判断）
        if(filtered_max_curvature >= CURVATURE_THRESHOLD_SHARP){
            current_pid_mode = 3;
        }
        else if(filtered_max_curvature >= CURVATURE_THRESHOLD_NORMAL){
            current_pid_mode = 2;
        }
        else{
            current_pid_mode = 1;  // ⚠ 关键：回到直道使用直道PID
        }
        
        apply_pid_mode(&angle_pid, current_pid_mode);
        pid_lock_timer = PID_LOCK_CYCLES * 2;  // 避障后延长锁定时间，稳定控制

        distance_timer = 1;
	}


	// ========== 右前方障碍物后退逻辑修改 ==========

	else if(right_min_distance < right_detected_warning && 
			right_min_distance < left_min_distance && 
			right_min_distance > 10.0f){
		
		distance_timer++;
 
        int pid_before_obstacle = current_pid_mode;
        current_pid_mode = 3;
        apply_pid_mode(&angle_pid, 3);
		
		MOTOR_SetSpeed(DEFAULT_STOP_SPEED);
		HAL_Delay(150);
		MOTOR_SetSpeed(0);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
		points = MS200_GetPointsData();
		
		// 计算激光雷达在90度附近的距离
		handle_missing_data(112 , points , &distance_R90 , &distance_R90_last);
		handle_missing_data(100 , points , &distance_R80 , &distance_R80_last);
		handle_missing_data(137 , points , &distance_R110 , &distance_R110_last);

		if(distance_left_back_200 < HOUTUI_WARNING || distance_right_back_160 < HOUTUI_WARNING){
			// 后方过近，不后退
		}else{
			MOTOR_SetSpeed(-DEFAULT_BACK_SPEED);
			HAL_Delay(100);
			SG90_SetAngle(DEFAULT_AFTER_BACK_ANGLE);
			
			int right_check_index = (int)floor(min_right_point / 0.8);
			
			do{
				HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
				points = MS200_GetPointsData();
				
				// 实时更新后方距离
				handle_missing_data(250 , points , &distance_200 , &distance_200_last);
				handle_missing_data(246 , points , &distance_197 , &distance_197_last);
				handle_missing_data(254 , points , &distance_203 , &distance_203_last);
				distance_left_back_200 = (distance_200 + distance_197 + distance_203) / 3;
				
				handle_missing_data(200 , points , &distance_160 , &distance_160_last);
				handle_missing_data(196 , points , &distance_157 , &distance_157_last);
				handle_missing_data(204 , points , &distance_163 , &distance_163_last);
				distance_right_back_160 = (distance_160 + distance_157 + distance_163) / 3;
				
				// 检查退出条件：前方安全 OR 后方过近
				if(distance_left_back_200 < HOUTUI_WARNING || distance_right_back_160 < HOUTUI_WARNING){
					break;
				}
				
				// *** 修改：使用右侧对应的后退系数 ***
			}while(points[right_check_index].distance < right_detected_warning * right_reverse_coefficient);
			
		}
		
		// 规整位置
		SG90_SetAngle(-DEFAULT_AFTER_BACK_ANGLE);
		MOTOR_SetSpeed(DEFAULT_BACK_HIGH_SPEED);
		HAL_Delay(150);
		SG90_SetAngle(30);
		MOTOR_SetSpeed(30);
		
        // ⚠ 新增：避障结束后重新判断PID
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
        points = MS200_GetPointsData();
        
        if(filtered_max_curvature >= CURVATURE_THRESHOLD_SHARP){
            current_pid_mode = 3;
        }
        else if(filtered_max_curvature >= CURVATURE_THRESHOLD_NORMAL){
            current_pid_mode = 2;
        }
        else{
            current_pid_mode = 1;
        }
        
        apply_pid_mode(&angle_pid, current_pid_mode);
        pid_lock_timer = PID_LOCK_CYCLES * 2;

        distance_timer = 1;
	}
		
		
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
		points = MS200_GetPointsData();
		
		//转向角度
		float param_angle_float = 0;
		
		float angle = 0; 
			handle_missing_data(413 , points , &left_30_distance , &left_30_distance_last);	// 330/0.8=412.5≈413
			handle_missing_data(388 , points , &left_50_distance , &left_50_distance_last);	// 310/0.8=387.5≈388
			handle_missing_data(400 , points , &left_40_distance , &left_40_distance_last);	// 320/0.8=400
			
			handle_missing_data(50 , points , &right_40_distance , &right_40_distance_last);	// 40/0.8=50
			handle_missing_data(37 , points , &right_30_distance , &right_30_distance_last);	// 30/0.8=37.5≈37（向下取整）
			handle_missing_data(62 , points , &right_50_distance , &right_50_distance_last);	// 50/0.8=62.5≈62（向下取整）
		
		
			if(fabs(change_angle( left_50_distance , right_50_distance , 50 )) > 30){
			
				if(fabs(change_angle( left_40_distance , right_40_distance , 40 )) > 30){
						angle += ( change_angle( left_30_distance , right_30_distance , 30 ) + (change_angle( left_50_distance , right_50_distance , 50 ) > 0 ? 30 : -30 ) + (change_angle( left_40_distance , right_40_distance , 40 ) > 0 ? 30 : -30 ) ) / 3 ;
				}else{
						angle += ( change_angle( left_30_distance , right_30_distance , 30 ) + (change_angle( left_50_distance , right_50_distance , 50 ) > 0 ? 30 : -30 ) + change_angle( left_40_distance , right_40_distance , 40 )) / 3 ;
				}
				
			}else{
			
				if(fabs(change_angle( left_40_distance , right_40_distance , 40 )) > 30){
					angle += ( change_angle( left_30_distance , right_30_distance , 30 ) + change_angle( left_50_distance , right_50_distance , 50) + (change_angle( left_40_distance , right_40_distance , 40 ) > 0 ? 30 : -30 ) ) / 3 ;
				}else{
					angle += ( change_angle( left_30_distance , right_30_distance , 30 ) + change_angle( left_50_distance , right_50_distance , 50 ) + change_angle( left_40_distance , right_40_distance , 40 )) / 3 ;
				}
			
			}
			
			param_angle_float = pid_realize(&angle_pid , angle);
		
		handle_missing_data(338 , points , &distance_left_90 , &distance_left_90_last);	// 270/0.8=337.5≈338
		handle_missing_data(112 , points , &distance_right_90 , &distance_right_90_last);	// 90/0.8=112.5≈112（向下取整）
		if(distance_left_90 !=0 && distance_right_90 != 0){
				//获取左右侧距离的权重
			center_percent = distance_left_90 / (distance_left_90 + distance_right_90);
				//根据权重获取待转的角度
			param_angle_float += pid_realize(&center_pid,center_percent);
		}

		
		
		int param_angle_int = param_angle_float;

		// ========== 直道转角倍数处理 ==========
		// 判断是否为直道状况：PID模式1 且 不在U弯 且 不在曲率弯道
		if(current_pid_mode == 1 && u_turn_state == 0 && curvature_turn_state == 0){
			param_angle_int = param_angle_int * STRAIGHT_ANGLE_MULTIPLIER;
		}

		// ========== 【新增】舵机物理限幅（±20°）==========
		#define SERVO_MAX_ANGLE 20
		if(param_angle_int > SERVO_MAX_ANGLE){
			param_angle_int = SERVO_MAX_ANGLE;
		}
		if(param_angle_int < -SERVO_MAX_ANGLE){
			param_angle_int = -SERVO_MAX_ANGLE;
		}
        
		// 10°位置
		int left_10_index = (int)ceil(350.0 / 0.8);   // 350°/0.8=437.5≈438
		int right_10_index = (int)floor(10.0 / 0.8);  // 10°/0.8=12.5≈12
		
		// 15°位置
		int left_15_index = (int)ceil(345.0 / 0.8);   // 345°/0.8=431.25≈432
		int right_15_index = (int)floor(15.0 / 0.8);  // 15°/0.8=18.75≈18
		
		// 20°位置
		int left_20_index = (int)ceil(340.0 / 0.8);   // 340°/0.8=425≈425
		int right_20_index = (int)floor(20.0 / 0.8);  // 20°/0.8=25≈25
		
		// 60°位置（原有代码已定义）
		int left_60_index = (int)ceil(300.0 / 0.8);   // 300°/0.8=375≈375
		int right_60_index = (int)floor(60.0 / 0.8);  // 60°/0.8=75≈75
		
		// 150°位置（原有代码已定义）
		int left_150_index = (int)ceil(210.0 / 0.8);  // 210°/0.8=262.5≈263
		int right_150_index = (int)floor(150.0 / 0.8); // 150°/0.8=187.5≈187

		// 读取距离数据
		float left_10_distance = points[left_10_index].distance;
		float right_10_distance = points[right_10_index].distance;
		
		float left_15_distance = points[left_15_index].distance;
		float right_15_distance = points[right_15_index].distance;
		
		float left_20_distance = points[left_20_index].distance;
		float right_20_distance = points[right_20_index].distance;
		
		float left_60_distance = points[left_60_index].distance;
		float right_60_distance = points[right_60_index].distance;
		
		float left_150_distance = points[left_150_index].distance;
		float right_150_distance = points[right_150_index].distance;

		// ========== 侧边蹭边保护（按优先级从高到低）==========
		
		// 左侧10°危险（最接近前方，最优先）
		if(left_10_distance > 10.0f && left_10_distance < SIDE_10_WARNING_DISTANCE){
			param_angle_int += SIDE_10_AVOIDANCE_ANGLE;
			if(param_angle_int > SERVO_MAX_ANGLE) param_angle_int = SERVO_MAX_ANGLE;
		}

		// 右侧10°危险
		if(right_10_distance > 10.0f && right_10_distance < SIDE_10_WARNING_DISTANCE){
			param_angle_int -= SIDE_10_AVOIDANCE_ANGLE;
			if(param_angle_int < -SERVO_MAX_ANGLE) param_angle_int = -SERVO_MAX_ANGLE;
		}

		// 左侧15°危险
		if(left_15_distance > 10.0f && left_15_distance < SIDE_15_WARNING_DISTANCE){
			param_angle_int += SIDE_15_AVOIDANCE_ANGLE;
			if(param_angle_int > SERVO_MAX_ANGLE) param_angle_int = SERVO_MAX_ANGLE;
		}

		// 右侧15°危险
		if(right_15_distance > 10.0f && right_15_distance < SIDE_15_WARNING_DISTANCE){
			param_angle_int -= SIDE_15_AVOIDANCE_ANGLE;
			if(param_angle_int < -SERVO_MAX_ANGLE) param_angle_int = -SERVO_MAX_ANGLE;
		}

		// 左侧20°危险
		if(left_20_distance > 10.0f && left_20_distance < SIDE_20_WARNING_DISTANCE){
			param_angle_int += SIDE_20_AVOIDANCE_ANGLE;
			if(param_angle_int > SERVO_MAX_ANGLE) param_angle_int = SERVO_MAX_ANGLE;
		}

		// 右侧20°危险
		if(right_20_distance > 10.0f && right_20_distance < SIDE_20_WARNING_DISTANCE){
			param_angle_int -= SIDE_20_AVOIDANCE_ANGLE;
			if(param_angle_int < -SERVO_MAX_ANGLE) param_angle_int = -SERVO_MAX_ANGLE;
		}

		// 左侧60°危险
		if(left_60_distance > 10.0f && left_60_distance < SIDE_60_WARNING_DISTANCE){
			param_angle_int += SIDE_60_AVOIDANCE_ANGLE;
			if(param_angle_int > SERVO_MAX_ANGLE) param_angle_int = SERVO_MAX_ANGLE;
		}

		// 右侧60°危险
		if(right_60_distance > 10.0f && right_60_distance < SIDE_60_WARNING_DISTANCE){
			param_angle_int -= SIDE_60_AVOIDANCE_ANGLE;
			if(param_angle_int < -SERVO_MAX_ANGLE) param_angle_int = -SERVO_MAX_ANGLE;
		}

		// 左侧150°危险（最侧面）
		if(left_150_distance > 10.0f && left_150_distance < SIDE_150_WARNING_DISTANCE){
			param_angle_int += SIDE_150_AVOIDANCE_ANGLE;
			if(param_angle_int > SERVO_MAX_ANGLE) param_angle_int = SERVO_MAX_ANGLE;
		}

		// 右侧150°危险
		if(right_150_distance > 10.0f && right_150_distance < SIDE_150_WARNING_DISTANCE){
			param_angle_int -= SIDE_150_AVOIDANCE_ANGLE;
			if(param_angle_int < -SERVO_MAX_ANGLE) param_angle_int = -SERVO_MAX_ANGLE;
		}

		// ========== U弯检测 ==========
		if(filtered_max_curvature >= ENTER_U && u_turn_state == 0){
			u_turn_state = 1;
			curvature_turn_state = 0;
		}

		if(u_turn_state == 1){
			param_angle_int = param_angle_int * NUM_MORE_ANGLE;
			
			// 【重要】U弯时也要限幅
			if(param_angle_int > SERVO_MAX_ANGLE) param_angle_int = SERVO_MAX_ANGLE;
			if(param_angle_int < -SERVO_MAX_ANGLE) param_angle_int = -SERVO_MAX_ANGLE;
			
			if(filtered_max_curvature < ENTER_U * 0.75f){
				u_turn_state = 0;
				
				// ⚠ 修复：退出U弯时，根据当前曲率决定下一个状态
				if(filtered_max_curvature >= CURVATURE_ENTER_THRESHOLD){
					// 仍在弯道中，激活曲率弯道状态
					curvature_turn_state = 1;
				}
				
				// 根据当前曲率恢复速度
				if(filtered_max_curvature > CURVATURE_THRESHOLD_SHARP){
					speed = DEFAULT_SPEED * SHARP_DECELERATION_FACTER;
				}
				else if(filtered_max_curvature > CURVATURE_THRESHOLD_NORMAL){
					speed = DEFAULT_SPEED * NORMAL_DECELERATION_FACTER;
				}
				else{
					speed = DEFAULT_HIGH_SPEED;
				}
			}
		}

		// ========== 曲率弯道检测 ==========
		else if(filtered_max_curvature >= CURVATURE_ENTER_THRESHOLD && curvature_turn_state == 0){
			curvature_turn_state = 1;
		}

		if(curvature_turn_state == 1 && u_turn_state == 0){
			param_angle_int = param_angle_int * CURVATURE_ANGLE_MULTIPLIER;
			
			// 【重要】曲率弯道时也要限幅
			if(param_angle_int > SERVO_MAX_ANGLE) param_angle_int = SERVO_MAX_ANGLE;
			if(param_angle_int < -SERVO_MAX_ANGLE) param_angle_int = -SERVO_MAX_ANGLE;
			
			if(filtered_max_curvature < CURVATURE_ENTER_THRESHOLD * 0.75f){
				curvature_turn_state = 0;
			}
		}
				
		SG90_SetAngle(param_angle_int);
		MOTOR_SetSpeed(speed);
	
        remote_oled_display_oled_data_t oled_data = {
            .angle = param_angle_int,
            .center_percent = center_percent,
            .left_curvature = left_fit_result.curvature / 100,
            .right_curvature = right_fit_result.curvature / 100,
            .left_fit_type = left_fit_result.type,
            .left_r_squared = left_fit_result.r_squared * 100,
            .distance_30 = points[37].distance / 10,
            .distance_330 = points[413].distance / 10,
            .speed = speed,
            .u_turn_state = u_turn_state,
            .curvature_turn_state = curvature_turn_state,
            .deceleration_state = deceleration_state
        };
        
		remote_oled_display_oled_display_update(remote_oled_display_handle, &oled_data);
		OLED_Clear();

        // ??0：转?? + 中心偏移
        OLED_ShowFloat(0, 0, param_angle_int);
        OLED_ShowFloat(60, 0, center_percent);

        // ??1：左右曲率（新增??
        OLED_ShowFloat(0, 16, left_fit_result.curvature / 100);   // 除以100显示
        OLED_ShowFloat(60, 16, right_fit_result.curvature / 100);

        // ??2：拟合类?? + R?值（新增??
        OLED_ShowFloat(0, 32, motor1->speed);                      // 实际速度
		OLED_ShowFloat(40, 32, speed_stuck_detect.stuck_counter);  // 卡住计数器

        // ??3：原有距离显??
        OLED_ShowFloat(0, 48, points[37].distance / 10);	// 30/0.8=37.5??37（向下取整）
        OLED_ShowFloat(60, 48, points[413].distance / 10);	// 330/0.8=412.5??413

        OLED_Display();
		HAL_Delay(50);

		
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_recv_buffer, UART2_RECV_BUFFER_SIZE);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
