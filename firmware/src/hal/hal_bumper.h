/**
 * @file hal_bumper.h
 * @brief [ZH] 机械防碰撞微动触须开关 HAL 驱动头文件。
 *        [EN] Mechanical Collision Bumper / Whisker Microswitch HAL Driver Header.
 *
 * 特性与设计意图 / Features:
 *  - 支持多路常开微动碰撞触须 (前左、前右、后方)；
 *  - 具备 5ms 软件消抖滤波 (Debounce Filter)，防止机械弹跳产生假阳性碰撞；
 *  - 支持 GPIO 下降沿中断与轮询双模式；
 *  - 触发后瞬时置位 emergency_stop 并向 EPGM 提供绝对高优先级的碰撞事件 0。
 */

#ifndef HAL_BUMPER_H
#define HAL_BUMPER_H

#include "hal_device.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUMPER_MASK_FRONT_LEFT  (1 << 0)
#define BUMPER_MASK_FRONT_RIGHT (1 << 1)
#define BUMPER_MASK_REAR        (1 << 2)

typedef struct {
    uint32_t pin_front_left;    ///< 前左微动 GPIO
    uint32_t pin_front_right;   ///< 前右微动 GPIO
    uint32_t pin_rear;          ///< 后方微动 GPIO
    uint32_t debounce_ms;       ///< 软件消抖延时 (默认 5ms)
    bool     active_low;        ///< 低电平有效 (常开上拉默认 true)
} bumper_config_t;

typedef struct {
    uint8_t bumper_mask;        ///< 碰撞掩码 (bit0:前左, bit1:前右, bit2:后方)
    bool    is_triggered;       ///< 整体碰撞标志 (任一触须触发即为 true)
    bool    emergency_stop;     ///< 物理急停状态
    uint32_t trigger_count;     ///< 累计碰撞触发次数
} bumper_status_t;

/**
 * @brief 创建并实例化微动触须 HAL 设备
 * @param dev 设备指针
 * @param name 设备名称 (如 "bumper_array")
 * @return HAL_OK 成功
 */
int hal_bumper_create(hal_device_t *dev, const char *name);

#ifdef __cplusplus
}
#endif

#endif // HAL_BUMPER_H
