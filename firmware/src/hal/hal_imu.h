/**
 * @file hal_imu.h
 * @brief [ZH] 6轴 IMU (MPU6050 / ICM20948) 姿态解算与打滑/堵转检测 HAL 驱动头文件。
 *        [EN] 6-Axis IMU (MPU6050 / ICM20948) Attitude & Slip/Stall Detection HAL Driver Header.
 *
 * 特性与设计意图 / Features:
 *  - 兼容 MPU6050 (0x68) 与 ICM20948 (0x69) 主流开源机器人 IMU 芯片；
 *  - 轻量互补滤波 (Complementary Filter) 实时解算 Pitch / Roll 姿态角 (Q4 定点数)；
 *  - 结合轮式电机动力学判定车轮打滑 (Wheel Slip) 与机身堵转卡死 (Motor Stall)；
 *  - 向应用层抛出打滑标志，引导 EPGM 判定行动受阻并进行脱困策略学习。
 */

#ifndef HAL_IMU_H
#define HAL_IMU_H

#include "hal_device.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_MODEL_MPU6050  = 0,
    IMU_MODEL_ICM20948 = 1
} imu_model_t;

// IMU 读数输出结构体
typedef struct {
    int16_t accel_x_mg;      ///< 纵向加速度 (mg, 毫重力加速度)
    int16_t accel_y_mg;      ///< 横向加速度 (mg)
    int16_t accel_z_mg;      ///< 垂向加速度 (mg, 平放静止为 1000mg)
    int16_t gyro_x_dps;      ///< 滚转角速度 (deg/s)
    int16_t gyro_y_dps;      ///< 俯仰角速度 (deg/s)
    int16_t gyro_z_dps;      ///< 偏航角速度 (deg/s)
    int16_t pitch_deg_q4;    ///< 俯仰角 (Q4 定点格式, 数值乘以 16)
    int16_t roll_deg_q4;     ///< 翻滚角 (Q4 定点格式, 数值乘以 16)
    bool    is_skidding;     ///< 是否检测到车轮打滑空转
    bool    is_stalled;      ///< 是否检测到底盘受阻堵转
} epgm_imu_data_t;

// IMU 配置参数
typedef struct {
    imu_model_t model;                 ///< 芯片型号
    uint8_t     i2c_addr;              ///< I2C 地址 (0x68 或 0x69)
    uint16_t    accel_scale_g;         ///< 加速度计满量程 (2, 4, 8, 16)
    uint16_t    gyro_scale_dps;        ///< 陀螺仪满量程 (250, 500, 1000, 2000)
    uint16_t    filter_alpha_q8;       ///< 互补滤波系数 (Q8 格式, 251 约对应 0.98)
    int16_t     stall_accel_thresh_mg; ///< 堵转加速度门限 (如 30mg)
    int16_t     slip_gyro_thresh_dps;  ///< 打滑偏航角速度突变门限 (如 50 deg/s)
} imu_config_t;

// 动力学交互参数 (供控制任务传入电机速度指令进行联动检测)
typedef struct {
    int16_t commanded_speed_mms;       ///< 电机目标线速度 (mm/s)
    int16_t measured_wheel_speed_mms;  ///< 编码器实测轮速 (mm/s)
} imu_motion_context_t;

#define HAL_IOCTL_IMU_FEED_MOTION  0x0041 ///< 注入电机运动上下文进行打滑联合检测

/**
 * @brief 创建并实例化 IMU HAL 设备
 * @param dev 设备指针
 * @param name 设备名称 (如 "imu_main")
 * @return HAL_OK 成功
 */
int hal_imu_create(hal_device_t *dev, const char *name);

#ifdef __cplusplus
}
#endif

#endif // HAL_IMU_H
