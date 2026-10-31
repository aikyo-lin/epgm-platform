/**
 * @file hal_motor.h
 * @brief [ZH] ESP32-S3 双轮差速电机驱动硬件抽象层 (PWM 控制)。
 *        [EN] ESP32-S3 Dual-wheel Differential Drive Motor HAL (PWM Control).
 */

#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 电机控制引脚定义 (适用于典型 ESP32-S3 电机驱动板，如 TB6612 / L298N)
#define PIN_MOTOR_L_PWM  4
#define PIN_MOTOR_L_DIR  5
#define PIN_MOTOR_R_PWM  6
#define PIN_MOTOR_R_DIR  7

void hal_motor_init(void);

/**
 * @brief [ZH] 根据动作指令驱动电机
 *        [EN] Drive motors according to action index
 * @param action 0:停, 1:前, 2:后, 3:左, 4:右
 */
void hal_motor_set_action(uint8_t action);

void hal_motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_MOTOR_H
