/**
 * @file sg90_test.c
 * @brief 黔行者2.0 舵机调试测试模块实现
 *
 * 包含两种测试模式：
 *   1. 扫动测试：连续左右扫动，检查限位和回中
 *   2. 单步测试：逐个角度测试，精确校准中心点
 *
 * 目的：验证 SG90_CENTER_OFFSET 零点偏移与 SG90_MAX_ANGLE 限幅调整后，
 *       前轮能否顺畅打满左右限位、回中是否居中、有无堵转异响。
 */

#include "sg90_test.h"
#include "sg90.h"
#include "motor.h"
#include "main.h"
#include "tim.h"

/**
 * @brief 舵机扫动测试
 */
void SG90_SteeringSweepTest(void)
{
    const int dwell_center_1 = 2000;   /* 满打前回中停留 */
    const int dwell_extreme  = 2000;   /* 满左/满右停留（观察+听声） */
    const int dwell_center_2 = 2000;   /* 满打后回中停留 */

    printf("\r\n===== SG90 Steering Sweep Test Start =====\r\n");
    printf("Max Angle: +-%d deg\r\n", SG90_MAX_ANGLE);
    printf("Center Offset: %d us\r\n", SG90_CENTER_OFFSET);

    /* 测试前先清电机，防止驱动轮带动车身 */
    MOTOR_SetSpeed(0);

    /* 先回中，确认上电/接线正常 */
    SG90_SetAngle(0);
    printf("[init] servo back to 0 deg\r\n");
    HAL_Delay(3000);

    while (1) {
        /* 满左 */
        printf("[sweep] +%d deg (full left), hold %d ms\r\n",
               SG90_MAX_ANGLE, dwell_extreme);
        SG90_SetAngle(0);
        HAL_Delay(dwell_center_1);
        SG90_SetAngle(SG90_MAX_ANGLE);
        HAL_Delay(dwell_extreme);
        SG90_SetAngle(0);
        HAL_Delay(dwell_center_2);

        /* 满右 */
        printf("[sweep] -%d deg (full right), hold %d ms\r\n",
               SG90_MAX_ANGLE, dwell_extreme);
        SG90_SetAngle(-SG90_MAX_ANGLE);
        HAL_Delay(dwell_extreme);
        SG90_SetAngle(0);
        HAL_Delay(dwell_center_2);
    }
}

/**
 * @brief 舵机单步测试
 */
void SG90_SingleStepTest(void)
{
    const int dwell_time = 2000;   /* 每个角度停留时间 */
    const int test_angles[] = {0, 5, 10, 15, 0, -5, -10, -15, 0};
    const int num_angles = sizeof(test_angles) / sizeof(test_angles[0]);

    printf("\r\n===== SG90 Single Step Test Start =====\r\n");
    printf("Max Angle: +-%d deg\r\n", SG90_MAX_ANGLE);
    printf("Center Offset: %d us\r\n", SG90_CENTER_OFFSET);
    printf("Test sequence: ");
    for (int i = 0; i < num_angles; i++) {
        printf("%+d%s ", test_angles[i], (i < num_angles - 1) ? "," : "");
    }
    printf("deg\r\n\r\n");

    /* 测试前先清电机，防止驱动轮带动车身 */
    MOTOR_SetSpeed(0);

    /* 遍历测试角度 */
    for (int i = 0; i < num_angles; i++) {
        int angle = test_angles[i];
        printf("[step %d/%d] Set angle to %+d deg, hold %d ms\r\n",
               i + 1, num_angles, angle, dwell_time);
        SG90_SetAngle(angle);
        HAL_Delay(dwell_time);
    }

    printf("\r\n===== SG90 Single Step Test Complete =====\r\n");
    printf("Observe: Is center (0 deg) aligned? Any abnormal sound?\r\n");
    printf("Adjust SG90_CENTER_OFFSET in sg90.h if needed.\r\n\r\n");
}
