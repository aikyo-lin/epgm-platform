/**
 * @file hal_encoder_pid.h
 * @brief [ZH] 差速双轮编码器与闭环速度 PID 控制器 HAL 驱动头文件。
 *        [EN] Differential Wheel Encoder & Closed-Loop Speed PID Controller HAL Driver Header.
 *
 * 特性与设计意图 / Features:
 *  - 双轮独立增量式数字 PID 控制器 (位置式 / 增量式可选)；
 *  - 集成抗积分饱和限制 (Anti-Windup) 与电机死区补偿 (Deadband Compensation)；
 *  - 支持 EPGM 5种离散动作 (0:停, 1:前, 2:后, 3:左转, 4:右转) 的目标轮速解算；
 *  - 提供里程计位置与实际转速反馈。
 */

#ifndef HAL_ENCODER_PID_H
#define HAL_ENCODER_PID_H

#include "hal_device.h"

#ifdef __cplusplus
extern "C" {
#endif

// PID 增益参数 (Q8 定点数, 256 表示 1.0)
typedef struct {
    int16_t kp_q8;              ///< 比例系数 Kp (Q8)
    int16_t ki_q8;              ///< 积分系数 Ki (Q8)
    int16_t kd_q8;              ///< 微分系数 Kd (Q8)
    int16_t integral_max;       ///< 积分限幅 (防饱和)
    int16_t deadband_pwm;       ///< 电机死区补偿 PWM 阈值 (如 30)
    int16_t max_output_pwm;     ///< 最大 PWM 输出 (如 255)
} pid_params_t;

// 控制器配置参数
typedef struct {
    pid_params_t left_pid;      ///< 左轮 PID
    pid_params_t right_pid;     ///< 右轮 PID
    int16_t      cruise_speed;  ///< 巡航标称速度 (mm/s, 默认 200)
    int16_t      turn_speed;    ///< 转向轮速 (mm/s, 默认 150)
    uint16_t     wheel_diameter_mm; ///< 车轮直径 (mm)
    uint16_t     wheel_base_mm;     ///< 双轮轮距 (mm)
    uint16_t     cpr;               ///< 编码器每转脉冲数
} encoder_pid_config_t;

// 双轮运行实时遥测数据
typedef struct {
    int32_t left_ticks;         ///< 左轮累计脉冲数
    int32_t right_ticks;        ///< 右轮累计脉冲数
    int16_t left_speed_mms;     ///< 左轮实测速度 (mm/s)
    int16_t right_speed_mms;    ///< 右轮实测速度 (mm/s)
    int16_t left_target_mms;    ///< 左轮目标速度 (mm/s)
    int16_t right_target_mms;   ///< 右轮目标速度 (mm/s)
    int16_t left_pwm_out;       ///< 左轮控制输出 PWM
    int16_t right_pwm_out;      ///< 右轮控制输出 PWM
    uint8_t current_action;     ///< 当前执行动作 (0~4)
} encoder_pid_status_t;

#define HAL_IOCTL_ENCODER_SET_ACTION   0x0050 ///< 设置 EPGM 离散动作指令 (0~4)
#define HAL_IOCTL_ENCODER_FEED_TICKS   0x0051 ///< 传入硬件编码器新采样的脉冲数
#define HAL_IOCTL_ENCODER_STEP_PID     0x0052 ///< 触发单步 PID 计算

/**
 * @brief 创建并实例化差速轮编码器 PID HAL 设备
 * @param dev 设备指针
 * @param name 设备名称 (如 "motor_diff_drive")
 * @return HAL_OK 成功
 */
int hal_encoder_pid_create(hal_device_t *dev, const char *name);

#ifdef __cplusplus
}
#endif

#endif // HAL_ENCODER_PID_H
