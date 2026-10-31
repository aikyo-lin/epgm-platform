/**
 * @file hal_hcsr04.h
 * @brief [ZH] HC-SR04 超声波测距传感器 HAL 驱动头文件。
 *        [EN] HC-SR04 Ultrasonic Distance Sensor HAL Driver Header.
 *
 * 特性与设计意图 / Features:
 *  - 10µs 触发脉冲与高精度微秒级回响脉宽捕获；
 *  - 距离公式: D_mm = (Delta_t_us * 343) / 2000；
 *  - 25ms 严格抗阻塞硬超时门狗 (Timeout Guard)，防止吸收材料或失步挂死 50Hz 控制闭环。
 */

#ifndef HAL_HCSR04_H
#define HAL_HCSR04_H

#include "hal_device.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HCSR04_TIMEOUT_MS        25     ///< 25ms 保护超时门狗
#define HCSR04_MAX_RANGE_MM      4000   ///< 4000mm 最大测量距离

typedef struct {
    uint32_t pin_trigger;       ///< Trigger GPIO 引脚
    uint32_t pin_echo;          ///< Echo GPIO 引脚
    uint32_t timeout_ms;        ///< 超时看门狗阈值 (默认 25ms)
    uint16_t max_range_mm;      ///< 超量程上限 (默认 4000mm)
} hcsr04_config_t;

typedef struct {
    uint16_t distance_mm;       ///< 测算距离 (mm)
    bool     is_timeout;        ///< 是否发生超时
    uint32_t echo_duration_us;  ///< 回波持续微秒数
} hcsr04_reading_t;

/**
 * @brief 创建并实例化 HC-SR04 超声波 HAL 设备
 * @param dev 设备指针
 * @param name 设备名称 (如 "us_front")
 * @return HAL_OK 成功
 */
int hal_hcsr04_create(hal_device_t *dev, const char *name);

#ifdef __cplusplus
}
#endif

#endif // HAL_HCSR04_H
