/**
 * @file hal_sensors.h
 * @brief [ZH] ESP32-S3 传感器阵列硬件抽象层 (8向测距 + 微动碰撞检测)。
 *        [EN] ESP32-S3 Sensor Array HAL (8-direction range + microswitch bumper).
 */

#ifndef HAL_SENSORS_H
#define HAL_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 物理微动碰撞开关引脚 (支持下降沿外部中断)
#define PIN_BUMPER_FRONT 14
#define PIN_BUMPER_REAR  15

void hal_sensors_init(void);

/**
 * @brief [ZH] 读取 8 个方向测距传感器读数 (单位: 毫米 mm)
 *        [EN] Read distance measurements for 8 directions (unit: mm)
 * @param distances 长度为 8 的数组缓冲 / Array buffer of length 8
 */
void hal_sensors_read_distances(uint16_t distances[8]);

/**
 * @brief [ZH] 检查是否触发碰撞微动开关
 *        [EN] Check if bumper microswitch is triggered
 * @return true 发生碰撞 / collision occurred
 */
bool hal_sensors_check_bumper(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_SENSORS_H
