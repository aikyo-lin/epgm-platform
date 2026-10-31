/**
 * @file hal_tca9548a.h
 * @brief [ZH] TCA9548A 8通道 I2C 多路复用器 HAL 驱动头文件。
 *        [EN] TCA9548A 8-Channel I2C Multiplexer HAL Driver Header.
 *
 * 特性与设计意图 / Features:
 *  - 突破同 I2C 地址外设冲突 (如 8 枚默认地址均为 0x29 的 VL53L0X)；
 *  - 支持单通道动态选通 (1 << ch) 与多通道并发使能；
 *  - 具备总线原子互斥锁 (Bus Lock)，防止多任务并发切换通道导致的数据混淆。
 */

#ifndef HAL_TCA9548A_H
#define HAL_TCA9548A_H

#include "hal_device.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TCA9548A_DEFAULT_I2C_ADDR 0x70

typedef struct {
    uint8_t  i2c_addr;          ///< I2C 地址 (默认 0x70)
    uint8_t  default_channel;   ///< 上电默认通道 (0~7)
    bool     enable_mutex;      ///< 是否使能总线互斥锁
} tca9548a_config_t;

/**
 * @brief 创建并实例化 TCA9548A HAL 设备
 * @param dev 设备指针
 * @param name 设备名称 (如 "i2c_mux_0")
 * @return HAL_OK 成功
 */
int hal_tca9548a_create(hal_device_t *dev, const char *name);

/**
 * @brief 便捷函数: 切换激活指定 I2C 通道 (0~7)
 * @param dev 设备指针
 * @param channel_id 通道编号 (0~7)
 * @return HAL_OK 成功
 */
int hal_tca9548a_select_channel(hal_device_t *dev, uint8_t channel_id);

/**
 * @brief 便捷函数: 获取当前选通的通道掩码 (0x01~0x80)
 * @param dev 设备指针
 * @param out_mask 输出通道掩码
 * @return HAL_OK 成功
 */
int hal_tca9548a_get_channel_mask(hal_device_t *dev, uint8_t *out_mask);

#ifdef __cplusplus
}
#endif

#endif // HAL_TCA9548A_H
