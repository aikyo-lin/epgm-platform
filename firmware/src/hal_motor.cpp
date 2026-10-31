/**
 * @file hal_motor.cpp
 * @brief [ZH] ESP32-S3 双轮差速电机控制实现。
 *        [EN] ESP32-S3 Dual-wheel Differential Drive Motor Implementation.
 */

#include "hal_motor.h"
#include <Arduino.h>

void hal_motor_init(void) {
    pinMode(PIN_MOTOR_L_PWM, OUTPUT);
    pinMode(PIN_MOTOR_L_DIR, OUTPUT);
    pinMode(PIN_MOTOR_R_PWM, OUTPUT);
    pinMode(PIN_MOTOR_R_DIR, OUTPUT);
    hal_motor_stop();
}

void hal_motor_set_action(uint8_t action) {
    switch (action) {
        case 1: // 前进 (Forward)
            digitalWrite(PIN_MOTOR_L_DIR, HIGH);
            analogWrite(PIN_MOTOR_L_PWM, 200);
            digitalWrite(PIN_MOTOR_R_DIR, HIGH);
            analogWrite(PIN_MOTOR_R_PWM, 200);
            break;
        case 2: // 后退 (Backward)
            digitalWrite(PIN_MOTOR_L_DIR, LOW);
            analogWrite(PIN_MOTOR_L_PWM, 150);
            digitalWrite(PIN_MOTOR_R_DIR, LOW);
            analogWrite(PIN_MOTOR_R_PWM, 150);
            break;
        case 3: // 原地左转 (Turn Left)
            digitalWrite(PIN_MOTOR_L_DIR, LOW);
            analogWrite(PIN_MOTOR_L_PWM, 160);
            digitalWrite(PIN_MOTOR_R_DIR, HIGH);
            analogWrite(PIN_MOTOR_R_PWM, 160);
            break;
        case 4: // 原地右转 (Turn Right)
            digitalWrite(PIN_MOTOR_L_DIR, HIGH);
            analogWrite(PIN_MOTOR_L_PWM, 160);
            digitalWrite(PIN_MOTOR_R_DIR, LOW);
            analogWrite(PIN_MOTOR_R_PWM, 160);
            break;
        case 0: // 停止 (Stop)
        default:
            hal_motor_stop();
            break;
    }
}

void hal_motor_stop(void) {
    analogWrite(PIN_MOTOR_L_PWM, 0);
    analogWrite(PIN_MOTOR_R_PWM, 0);
}
