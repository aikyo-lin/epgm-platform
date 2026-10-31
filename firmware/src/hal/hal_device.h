/**
 * @file hal_device.h
 * @brief [ZH] 通用机器人外设硬件抽象层 (HAL) 设备基类与驱动注册表头文件。
 *        [EN] Universal Robot Peripheral HAL Device Base & Registry Header.
 *
 * 架构模型 / Architectural Model:
 *  - 纯 C 面向对象接口: init, read, write, ioctl, deinit 统一多态生命周期；
 *  - 设备注册表 (Device Registry): 集中管理 ToF, 超声波, IMU, I2C 多路复用器, 编码器 PID 与触须微动；
 *  - 统一错误码与设备类型定义。
 */

#ifndef HAL_DEVICE_H
#define HAL_DEVICE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MAX_DEVICES 32

// 设备类型枚举
typedef enum {
    HAL_DEV_TYPE_UNKNOWN     = 0,
    HAL_DEV_TYPE_TOF         = 1, ///< 激光测距传感器 (VL53L0X / VL53L1X)
    HAL_DEV_TYPE_ULTRASONIC  = 2, ///< 超声波测距传感器 (HC-SR04)
    HAL_DEV_TYPE_IMU         = 3, ///< 6轴惯导姿态与打滑检测 (MPU6050 / ICM20948)
    HAL_DEV_TYPE_I2C_MUX     = 4, ///< I2C 选通多路复用器 (TCA9548A)
    HAL_DEV_TYPE_ENCODER_PID = 5, ///< 差速双轮编码器与闭环速度 PID 控制器
    HAL_DEV_TYPE_BUMPER      = 6  ///< 机械防碰撞微动触须开关
} hal_device_type_t;

// 标准 HAL 返回状态码
typedef enum {
    HAL_OK           = 0,
    HAL_ERR_PARAM    = -1,
    HAL_ERR_NOT_INIT = -2,
    HAL_ERR_IO       = -3,
    HAL_ERR_TIMEOUT  = -4,
    HAL_ERR_BUSY     = -5,
    HAL_ERR_FULL     = -6,
    HAL_ERR_NOT_FOUND= -7
} hal_status_t;

// 标准 ioctl 命令基准
#define HAL_IOCTL_GET_STATUS     0x0001
#define HAL_IOCTL_SET_CONFIG     0x0002
#define HAL_IOCTL_RESET          0x0003
#define HAL_IOCTL_CALIBRATE      0x0004
#define HAL_IOCTL_SET_CHANNEL    0x0010
#define HAL_IOCTL_SET_TARGET     0x0020
#define HAL_IOCTL_CLEAR_FLAG     0x0030

// 统一设备驱动结构体 (与 PROJECT.md 接口契约严格一致)
typedef struct hal_device {
    const char       *name;
    hal_device_type_t type;
    bool              is_initialized;
    int (*init)(struct hal_device *dev, const void *config);
    int (*read)(struct hal_device *dev, void *buf, size_t len);
    int (*write)(struct hal_device *dev, const void *buf, size_t len);
    int (*ioctl)(struct hal_device *dev, uint32_t cmd, void *arg);
    int (*deinit)(struct hal_device *dev);
    void             *priv_data;
} hal_device_t;

// ==============================================================================
// 设备注册表 API
// ==============================================================================

/**
 * @brief 注册设备至全局 HAL 设备表
 * @param dev 设备指针
 * @return HAL_OK 成功，否则返回错误码
 */
int hal_device_register(hal_device_t *dev);

/**
 * @brief 从全局设备表中注销设备
 * @param dev 设备指针
 * @return HAL_OK 成功
 */
int hal_device_unregister(hal_device_t *dev);

/**
 * @brief 按名称查找设备
 * @param name 设备名称字符串 (如 "tof_front", "imu_main")
 * @return 设备指针，未找到返回 NULL
 */
hal_device_t *hal_device_find(const char *name);

/**
 * @brief 按类型和索引查找设备
 * @param type 设备类型
 * @param index 同类型设备索引序号 (从 0 开始)
 * @return 设备指针，未找到返回 NULL
 */
hal_device_t *hal_device_find_by_type(hal_device_type_t type, size_t index);

/**
 * @brief 获取当前已注册设备总数
 * @return 设备数量
 */
size_t hal_device_count(void);

/**
 * @brief 清空设备注册表并注销所有设备
 */
void hal_device_registry_reset(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_DEVICE_H
