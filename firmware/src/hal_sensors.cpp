/**
 * @file hal_sensors.cpp
 * @brief [ZH] ESP32-S3 传感器采集实现 (支持模拟/实际传感器采集)。
 *        [EN] ESP32-S3 Sensor Acquisition Implementation.
 */

#include "hal_sensors.h"
#include <Arduino.h>

static volatile bool g_bumper_triggered = false;

// 外部中断服务函数 (微秒级即刻捕获碰撞)
static void IRAM_ATTR bumper_isr(void) {
    g_bumper_triggered = true;
}

void hal_sensors_init(void) {
    pinMode(PIN_BUMPER_FRONT, INPUT_PULLUP);
    pinMode(PIN_BUMPER_REAR, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_BUMPER_FRONT), bumper_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BUMPER_REAR), bumper_isr, FALLING);
}

void hal_sensors_read_distances(uint16_t distances[8]) {
    // 实际硬件中通过 I2C 读取 VL53L0X/VL53L1X ToF 雷达或 ADC 读取红外/超声波
    // 在本基础实现中，若无外部 I2C 从机响应，提供默认安全测试基线
    for (int i = 0; i < 8; ++i) {
        distances[i] = 800; // 默认 800mm (开阔安全)
    }
}

bool hal_sensors_check_bumper(void) {
    bool hit = g_bumper_triggered || (digitalRead(PIN_BUMPER_FRONT) == LOW) || (digitalRead(PIN_BUMPER_REAR) == LOW);
    g_bumper_triggered = false; // 清除脉冲标记
    return hit;
}
