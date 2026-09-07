/**
 ******************************************************************************
 * @file           : ms200.c
 * @brief          : MS200激光雷达驱动模块
 * @details        : 通过UART5接收雷达数据，解析距离和角度信息
 *                   支持360度扫描，数据格式符合MS200协议
 ******************************************************************************
 */

#include "ms200.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "usart.h"

/* ======================== 宏定义 ======================== */
#define ANGLE_CROSSOVER_THRESHOLD_START  345  // 跨越0度的起始角度阈值
#define ANGLE_CROSSOVER_THRESHOLD_END    10   // 跨越0度的结束角度阈值
#define ANGLE_MAX_VALUE                  36000.0f  // 最大角度值(360.00度)
#define ANGLE_TO_INDEX_DIVISOR          80    // 角度转索引的除数(0.8度分辨率)

/* ======================== 全局变量 ======================== */

// 数据帧头标识
const uint8_t _tof_data_header[2] = { 0x54, 0x2C };

// UART接收缓冲区相关变量
static int tof_uart_recv_buffer_end = 0;                       // 接收缓冲区当前位置
static uint8_t tof_uart_recv_buffer[TOF_UART_RECV_BUFFER_SIZE] = {0};  // 接收缓冲区
static uint8_t tof_recv_byte = 0;                              // 单字节接收变量

// 雷达数据点数组（450个点，覆盖360度，每0.8度一个点）
static MS200_Point points[450];

// CRC-8查找表（用于快速校验）
const uint8_t crc8_table[256] = { 
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43, 
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb, 
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20, 
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96, 
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8 
};

/* ======================== 私有函数 ======================== */

/**
 * @brief  计算CRC-8校验值
 * @details 使用查找表方式快速计算CRC-8校验
 * @param  data: 待校验数据指针
 * @param  len: 数据长度
 * @retval uint8_t 计算得到的CRC-8值
 */
static uint8_t ComputeCRC8(uint8_t* data, int len) 
{
    uint8_t crc = 0x00;
    
    for (int i = 0; i < len; i++) 
    {
        crc = crc8_table[(crc ^ *data++) & 0xff];
    }
    
    return crc;
}

/**
 * @brief  处理角度跨越0度的情况
 * @details 当扫描坐标从345度到10度时，需要特殊处理
 * @param  start_angle: 起始角度（单位：0.01度）
 * @param  stop_angle: 结束角度（单位：0.01度）
 * @param  tof_data: 测距数据数组
 * @retval None
 */
static void ProcessCrossoverAngle(uint16_t start_angle, uint16_t stop_angle, uint16_t* tof_data)
{
    // 计算角度步进值
    float step = (stop_angle + 360 - start_angle) / (TOF_DATA_COUNT_N - 1);
    
    for (int i = 0; i < TOF_DATA_COUNT_N; i++) 
    {
        float current_angle = (start_angle + i * step);
        
        // 角度超过360度时回绕
        if (current_angle > ANGLE_MAX_VALUE) 
        {
            current_angle -= ANGLE_MAX_VALUE;
        }
        
        // 将角度转换为数组索引
        int current_angle_int = (int)current_angle;
        int index = current_angle_int / ANGLE_TO_INDEX_DIVISOR;
        
        // 存储角度和距离数据
        points[index].angle = current_angle / ANGLE_TO_INDEX_DIVISOR;
        points[index].distance = tof_data[i];
    }
}

/**
 * @brief  处理正常角度范围的数据
 * @details 当扫描角度不跨越0度时的处理
 * @param  start_angle: 起始角度（单位：0.01度）
 * @param  stop_angle: 结束角度（单位：0.01度）
 * @param  tof_data: 测距数据数组
 * @retval None
 */
static void ProcessNormalAngle(uint16_t start_angle, uint16_t stop_angle, uint16_t* tof_data)
{
    // 计算角度步进值
    float step = (stop_angle - start_angle) / (TOF_DATA_COUNT_N - 1);
    
    for (int i = 0; i < TOF_DATA_COUNT_N; i++) 
    {
        float current_angle = (start_angle + i * step);
        
        // 角度超过360度时回绕
        if (current_angle > ANGLE_MAX_VALUE) 
        {
            current_angle -= ANGLE_MAX_VALUE;
        }
        
        // 将角度转换为数组索引
        int current_angle_int = (int)current_angle;
        int index = current_angle_int / ANGLE_TO_INDEX_DIVISOR;
        
        // 存储角度和距离数据
        points[index].angle = current_angle / ANGLE_TO_INDEX_DIVISOR;
        points[index].distance = tof_data[i];
    }
}

/* ======================== 公共函数 ======================== */

/**
 * @brief  MS200雷达模块初始化
 * @details 启动UART5接收中断，准备接收雷达数据
 * @retval None
 */
void MS200_Init(void)
{
    HAL_UART_Receive_IT(&huart5, &tof_recv_byte, 1);
}

/**
 * @brief  雷达数据接收处理函数
 * @details 在UART接收中断中调用，逐字节接收并解析雷达数据包
 *          数据包格式：
 *          [帧头2B][预留2B][起始角度2B][测距数据36B][结束角度2B][时间戳2B][CRC1B]
 * @retval None
 */
void MS200_ReceiveProcess(void)
{
    /* -------- 检测帧头 -------- */
    if (tof_recv_byte == 0x54) 
    {
        // 收到帧头，重置缓冲区指针
        tof_uart_recv_buffer_end = 0;
    }
    
    /* -------- 存储接收字节 -------- */
    if (tof_uart_recv_buffer_end < TOF_PACKAGE_LENGTH)
    {
        tof_uart_recv_buffer[tof_uart_recv_buffer_end++] = tof_recv_byte;
    }
    
    /* -------- 完整数据包解析 -------- */
    if (tof_uart_recv_buffer_end >= TOF_PACKAGE_LENGTH)
    {
        int offset = 0;
        uint8_t checksum = 0;
        uint16_t start_angle;
        uint16_t stop_angle;
        uint16_t tof_data[TOF_DATA_COUNT_N] = { 0 };
        
        // 验证帧头
        if (!memcmp(tof_uart_recv_buffer, _tof_data_header, 2)) 
        {
            /* -------- 解析数据包字段 -------- */
            offset += 2;  // 跳过帧头
            offset += 2;  // 跳过预留字段
            
            // 解析起始角度（小端格式）
            start_angle = tof_uart_recv_buffer[offset] | 
                         (tof_uart_recv_buffer[offset + 1] << 8);
            offset += 2;
            
            // 解析12个测距数据点
            for (int i = 0; i < TOF_DATA_COUNT_N; i++) 
            {
                tof_data[i] = tof_uart_recv_buffer[offset + i * 3] | 
                             (tof_uart_recv_buffer[offset + i * 3 + 1] << 8);
            }
            offset += 3 * TOF_DATA_COUNT_N;
            
            // 解析结束角度
            stop_angle = tof_uart_recv_buffer[offset] | 
                        (tof_uart_recv_buffer[offset + 1] << 8);
            offset += 2;
            
            offset += 2;  // 跳过时间戳
            
            /* -------- CRC校验 -------- */
            checksum = ComputeCRC8(tof_uart_recv_buffer, offset);
            
            if (checksum == tof_uart_recv_buffer[offset]) 
            {
                /* -------- 根据角度范围选择处理方式 -------- */
                if (start_angle > ANGLE_CROSSOVER_THRESHOLD_START && 
                    stop_angle < ANGLE_CROSSOVER_THRESHOLD_END)
                {
                    // 扫描跨越0度点的情况
                    ProcessCrossoverAngle(start_angle, stop_angle, tof_data);
                }
                else
                {
                    // 正常角度范围
                    ProcessNormalAngle(start_angle, stop_angle, tof_data);
                }
                
                // 对0度位置进行平滑处理（取相邻点平均值）
                points[0].distance = (points[449].distance + points[1].distance) / 2;
            }
        }
    }
    
    /* -------- 继续接收下一个字节 -------- */
    HAL_UART_Receive_IT(&huart5, &tof_recv_byte, 1);
}

/**
 * @brief  获取雷达数据点数组
 * @details 返回包含所有角度测距数据的数组指针
 * @retval MS200_Point* 数据点数组指针（450个点）
 */
MS200_Point* MS200_GetPointsData(void)
{
    return points;
}

/**
 * @brief  设置雷达旋转速度
 * @details 通过UART发送指令设置雷达扫描频率
 * @param  speed_hz: 旋转速度（单位：Hz，范围1-15）
 * @retval None
 * @note   指令格式：
 *         [帧头2B][指令码1B][操作码1B][长度1B][速度1B][预留1B][CRC1B][帧尾2B]
 */
void MS200_SetRotationSpeed(uint8_t speed_hz) 
{
    uint8_t cmd[10] = {0};
    uint8_t crc;
    
    /* -------- 构建指令包 -------- */
    cmd[0] = 0xA5;  // 帧头高字节
    cmd[1] = 0xF5;  // 帧头低字节
    cmd[2] = 0xA1;  // 指令码：设置转速
    cmd[3] = 0xC1;  // 操作码：设置命令
    cmd[4] = 0x02;  // 指令长度：2字节
    cmd[5] = speed_hz;  // 转速值参数
    cmd[6] = 0x00;  // 预留字节
    
    /* -------- 计算校验值（BCC异或校验）-------- */
    crc = cmd[0];
    for (int i = 1; i < 7; i++) 
    {
        crc ^= cmd[i];
    }
    cmd[7] = crc;
    
    /* -------- 帧尾 -------- */
    cmd[8] = 0x31;  // 帧尾高字节
    cmd[9] = 0xF2;  // 帧尾低字节
    
    /* -------- 通过UART5发送指令 -------- */
    HAL_UART_Transmit(&huart5, cmd, 10, 1000);
}

/*
 * ============================================================================
 * 数据包格式示例（用于参考）
 * ============================================================================
 * 完整数据包结构：
 * 
 * 54 2C          - 帧头
 * B2 14          - 预留
 * FD 54          - 起始角度
 * 8C 00 1C       - 测距数据点1
 * 90 00 24       - 测距数据点2
 * 99 00 23       - 测距数据点3
 * E5 00 2D       - 测距数据点4
 * DE 00 20       - 测距数据点5
 * D8 00 1E       - 测距数据点6
 * ED 00 28       - 测距数据点7
 * F0 00 1D       - 测距数据点8
 * D7 00 32       - 测距数据点9
 * D8 00 6B       - 测距数据点10
 * D8 00 34       - 测距数据点11
 * D6 00 21       - 测距数据点12
 * 3B 5A          - 结束角度
 * ED 42          - 时间戳
 * 64             - CRC校验
 * ============================================================================
 */