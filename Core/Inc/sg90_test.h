/**
 * @file sg90_test.h
 * @brief 黔行者2.0 舵机调试测试模块
 */

#ifndef SG90_TEST_H__
#define SG90_TEST_H__

/**
 * @brief 舵机扫动测试
 * @details 测试流程：
 *   1. 回中 0°，停 3 秒 —— 确认上电/接线正常
 *   2. 无限循环往复：满左 +SG90_MAX_ANGLE° → 回中 → 满右 -SG90_MAX_ANGLE° → 回中
 *      （满打停留 2 秒观察+听声）
 *   3. 需手动复位退出（无限循环不返回）
 *
 * @note 全程通过 SG90_SetAngle() 走转向标定层（含反号/限幅/中心偏移），
 *       与正常循迹模式完全一致。测试前清零电机，确保车身不动。
 */
void SG90_SteeringSweepTest(void);

/**
 * @brief 舵机单步测试
 * @details 测试流程：
 *   按顺序测试各个角度：0°, +5°, +10°, +15°, -5°, -10°, -15°
 *   每个角度停留 2 秒，便于观察舵机位置和听异响
 *   循环一次后结束（可重复调用）
 *
 * @note 用于精确校准中心点和检查各角度是否顺畅
 */
void SG90_SingleStepTest(void);

#endif /* SG90_TEST_H__ */
