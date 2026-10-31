/**
 * @file mock_hardware.h
 * @brief [ZH] 宿主机硬件抽象层 (I2C、GPIO、定时器与主流外设) 模拟头文件。
 *        [EN] Host Hardware Mock Header (I2C, GPIO, Timers, and Sensor Register Emulation).
 *
 * 设计意图 / Design Rationale:
 *  - 提供脱离物理硬件环境的 I2C 寄存器级读写模拟；
 *  - 模拟 TCA9548A 8 通道 I2C 选通切换与总线隔离；
 *  - 模拟 VL53L0X/VL53L1X ToF 激光、HC-SR04 超声波、MPU6050/ICM20948 IMU 读数与打滑波形注入；
 *  - 模拟微动触须开关 GPIO 中断与弹跳。
 */

#ifndef MOCK_HARDWARE_H
#define MOCK_HARDWARE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// 常用模拟设备 I2C 地址与引脚常量
// ==============================================================================
#define MOCK_I2C_ADDR_TCA9548A  0x70
#define MOCK_I2C_ADDR_VL53LXX   0x29
#define MOCK_I2C_ADDR_MPU6050   0x68
#define MOCK_I2C_ADDR_ICM20948  0x69

#define MOCK_PIN_BUMPER_FRONT_L 14
#define MOCK_PIN_BUMPER_FRONT_R 15
#define MOCK_PIN_BUMPER_REAR    16

// ==============================================================================
// 硬件模拟器生命周期与控制
// ==============================================================================

/**
 * @brief 初始化硬件模拟环境
 */
void mock_hardware_init(void);

/**
 * @brief 重置所有模拟外设状态至上电默认值
 */
void mock_hardware_reset(void);

/**
 * @brief 获取宿主机模拟高精度微秒时间戳
 * @return 运行微秒数 (µs)
 */
uint64_t mock_hardware_get_time_us(void);

// ==============================================================================
// GPIO 模拟 API
// ==============================================================================

void mock_gpio_set_level(uint32_t pin, uint8_t level);
uint8_t mock_gpio_get_level(uint32_t pin);
void mock_gpio_pulse(uint32_t pin, uint32_t duration_us);

// ==============================================================================
// I2C 总线读写模拟 API
// ==============================================================================

int mock_i2c_write(uint8_t dev_addr, const uint8_t *data, size_t len);
int mock_i2c_read(uint8_t dev_addr, uint8_t *data, size_t len);
int mock_i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val);
int mock_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *out_val);

// ==============================================================================
// 模拟传感器注入 API
// ==============================================================================

/**
 * @brief 注入特定 TCA9548A 通道上的 ToF 测距读数
 * @param channel TCA9548A 通道 (0~7)
 * @param distance_mm 测距距离 (毫米)
 * @param range_status 测距状态码 (0: 正常有效, 1: 信号弱, 2: 超出量程, 4: 相位失效)
 */
void mock_hardware_inject_tof(uint8_t channel, uint16_t distance_mm, uint8_t range_status);

/**
 * @brief 注入超声波传感器脉冲宽度 / 距离
 * @param trigger_pin 触发引脚
 * @param distance_mm 测距距离 (毫米)
 * @param timeout 是否模拟超时无响应 (>25ms)
 */
void mock_hardware_inject_ultrasonic(uint32_t trigger_pin, uint16_t distance_mm, bool timeout);

/**
 * @brief 注入 IMU 原始读数 (加计与陀螺仪)
 * @param ax_mg 纵向加速度 (mg, 毫重力加速度)
 * @param ay_mg 横向加速度 (mg)
 * @param az_mg 垂向加速度 (mg, 静止平放通常为 1000mg)
 * @param gx_dps 滚转角速度 (deg/s)
 * @param gy_dps 俯仰角速度 (deg/s)
 * @param gz_dps 偏航角速度 (deg/s)
 */
void mock_hardware_inject_imu(
    int16_t ax_mg, int16_t ay_mg, int16_t az_mg,
    int16_t gx_dps, int16_t gy_dps, int16_t gz_dps
);

/**
 * @brief 注入编码器转速与脉冲计数
 * @param left_ticks 左轮累计编码器脉冲数
 * @param right_ticks 右轮累计编码器脉冲数
 */
void mock_hardware_inject_encoders(int32_t left_ticks, int32_t right_ticks);

/**
 * @brief 获取当前 TCA9548A 激活的通道掩码 (0x01 ~ 0x80)
 */
uint8_t mock_hardware_get_tca9548a_channel_mask(void);

#ifdef __cplusplus
}
#endif

#endif // MOCK_HARDWARE_H
