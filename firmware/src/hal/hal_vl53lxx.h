/**
 * @file hal_vl53lxx.h
 * @brief [ZH] VL53L0X / VL53L1X ToF 激光测距传感器 HAL 驱动头文件。
 *        [EN] VL53L0X / VL53L1X ToF Laser Ranging Sensor HAL Driver Header.
 *
 * 特性与设计意图 / Features:
 *  - 统一适配 VL53L0X (量程 2m) 与 VL53L1X (量程 4m)；
 *  - 支持测量时序预算配置 (Timing Budget: 20ms~50ms)；
 *  - 包含光学串扰 (Crosstalk) 补偿与硬件偏差 (Offset) 标定；
 *  - 严格检测测距状态码 (Range Status)，遇无效测量时输出安全极大值，防止量化器紊乱。
 */

#ifndef HAL_VL53LXX_H
#define HAL_VL53LXX_H

#include "hal_device.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VL53LXX_MODEL_VL53L0X = 0,
    VL53LXX_MODEL_VL53L1X = 1
} vl53lxx_model_t;

typedef enum {
    VL53LXX_RANGE_VALID         = 0,
    VL53LXX_RANGE_SIGMA_FAIL    = 1,
    VL53LXX_RANGE_SIGNAL_FAIL   = 2,
    VL53LXX_RANGE_OUT_OF_BOUNDS = 4
} vl53lxx_range_status_t;

// 配置参数结构体
typedef struct {
    vl53lxx_model_t model;             ///< 芯片型号
    uint8_t         i2c_addr;          ///< I2C 地址 (默认 0x29)
    uint8_t         mux_channel;       ///< TCA9548A 对应通道编号 (0~7，若不经过复用器设为 0xFF)
    uint32_t        timing_budget_ms;  ///< 时序预算 (ms, 推荐 20 或 33)
    int16_t         offset_mm;         ///< 偏差标定值 (mm)
    uint16_t        crosstalk_kcps;    ///< 串扰补偿标定值 (kcps)
    uint16_t        max_range_mm;      ///< 设定的最大量程 (VL53L0X: 2000, VL53L1X: 4000)
} vl53lxx_config_t;

// 测距输出结构体
typedef struct {
    uint16_t               distance_mm;   ///< 最终有效距离 (mm)
    vl53lxx_range_status_t status;        ///< 测量状态
    uint32_t               timestamp_us;  ///< 测量微秒时间戳
} vl53lxx_reading_t;

/**
 * @brief 创建并实例化 VL53Lxx HAL 设备
 * @param dev 设备结构体指针
 * @param name 设备名称 (如 "tof_front")
 * @return HAL_OK 成功
 */
int hal_vl53lxx_create(hal_device_t *dev, const char *name);

#ifdef __cplusplus
}
#endif

#endif // HAL_VL53LXX_H
