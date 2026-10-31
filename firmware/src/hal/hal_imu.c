/**
 * @file hal_imu.c
 * @brief [ZH] 6轴 IMU (MPU6050 / ICM20948) 姿态解算与打滑/堵转检测 HAL 驱动实现。
 *        [EN] 6-Axis IMU (MPU6050 / ICM20948) Attitude & Slip/Stall Detection HAL Driver Implementation.
 */

#include "hal_imu.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

typedef struct {
    imu_config_t         config;
    epgm_imu_data_t      current_data;
    imu_motion_context_t motion_ctx;
    int32_t              slip_counter;
    int32_t              stall_counter;
} imu_priv_t;

static int imu_init(hal_device_t *dev, const void *config) {
    if (!dev) return HAL_ERR_PARAM;

    imu_priv_t *priv = (imu_priv_t *)malloc(sizeof(imu_priv_t));
    if (!priv) return HAL_ERR_IO;
    memset(priv, 0, sizeof(imu_priv_t));

    if (config) {
        memcpy(&priv->config, config, sizeof(imu_config_t));
    } else {
        priv->config.model = IMU_MODEL_MPU6050;
        priv->config.i2c_addr = 0x68;
        priv->config.accel_scale_g = 4;
        priv->config.gyro_scale_dps = 1000;
        priv->config.filter_alpha_q8 = 251; // ~0.98 in Q8
        priv->config.stall_accel_thresh_mg = 30;
        priv->config.slip_gyro_thresh_dps = 50;
    }

    // 初始化静止平放状态
    priv->current_data.accel_x_mg = 0;
    priv->current_data.accel_y_mg = 0;
    priv->current_data.accel_z_mg = 1000; // 1g
    priv->current_data.gyro_x_dps = 0;
    priv->current_data.gyro_y_dps = 0;
    priv->current_data.gyro_z_dps = 0;
    priv->current_data.pitch_deg_q4 = 0;
    priv->current_data.roll_deg_q4 = 0;
    priv->current_data.is_skidding = false;
    priv->current_data.is_stalled = false;

    dev->priv_data = priv;
    dev->is_initialized = true;
    return HAL_OK;
}

static int imu_read(hal_device_t *dev, void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    if (len < sizeof(epgm_imu_data_t)) return HAL_ERR_PARAM;

    imu_priv_t *priv = (imu_priv_t *)dev->priv_data;

    // 1. 运行轻量互补滤波解算姿态角 (Pitch / Roll)
    // 加速度计粗估俯仰: pitch_acc = atan2(-ax, sqrt(ay^2 + az^2))
    int16_t ax = priv->current_data.accel_x_mg;
    int16_t ay = priv->current_data.accel_y_mg;
    int16_t az = priv->current_data.accel_z_mg;
    if (az == 0) az = 1;

    // 定点轻量估计
    int32_t pitch_acc_deg = (int32_t)(-ax * 180) / (abs(az) * 3);
    int32_t roll_acc_deg  = (int32_t)(ay * 180) / (abs(az) * 3);

    // 滤波系数 alpha = 0.98 (Q8: 251)
    int32_t alpha = priv->config.filter_alpha_q8;
    int32_t pitch_q4 = (alpha * (priv->current_data.pitch_deg_q4 + (priv->current_data.gyro_y_dps / 3)) +
                        (256 - alpha) * (pitch_acc_deg << 4)) >> 8;
    int32_t roll_q4  = (alpha * (priv->current_data.roll_deg_q4 + (priv->current_data.gyro_x_dps / 3)) +
                        (256 - alpha) * (roll_acc_deg << 4)) >> 8;

    priv->current_data.pitch_deg_q4 = (int16_t)pitch_q4;
    priv->current_data.roll_deg_q4  = (int16_t)roll_q4;

    // 2. 动力学打滑 (Wheel Slip) 与堵转 (Motor Stall) 综合判定与解耦 (修复缺陷 3)
    int16_t cmd_speed = priv->motion_ctx.commanded_speed_mms;
    int16_t wheel_spd = priv->motion_ctx.measured_wheel_speed_mms;

    bool slip_condition = false;
    bool stall_condition = false;

    // A. 判定电机受阻堵转 (Locked Rotor / Motor Stall):
    // 下发较高前进指令速度 (|cmd_speed| > 100mm/s)，但编码器实测轮速卡死接近于零 (|wheel_spd| < 20mm/s)，且纵向加计无加速度 (|ax| < stall_thresh)
    // 此时严格判定为电机堵转，解耦车轮打滑标志 (is_stalled = true, is_skidding = false)
    if (abs(cmd_speed) > 100 && abs(wheel_spd) < 20 && abs(ax) < priv->config.stall_accel_thresh_mg) {
        priv->stall_counter++;
        if (priv->stall_counter > 2) {
            stall_condition = true;
            slip_condition = false; // 轮速为 0 时解耦车轮空转打滑标志
        }
    }
    // B. 判定冰面车轮空转打滑 (Wheel Slip / Ice Skid):
    // 轮速发生超速空转 (|wheel_spd| > 200mm/s 或 |cmd_speed| > 200mm/s)，但机身加速度缺失 (|ax| < stall_thresh)
    // 连续 3 周期滤波确认，置位打滑标志与底盘受困标志
    else if ((abs(wheel_spd) > 200 || abs(cmd_speed) > 200) && abs(ax) < priv->config.stall_accel_thresh_mg) {
        priv->stall_counter++;
        if (priv->stall_counter > 2) {
            slip_condition = true;
            stall_condition = true; // 包含底盘失步受困
        }
    }
    // C. 正常匀速直线巡航工况 (Constant-Speed Cruise 消除假阳性误报):
    // 当速度处于标称巡航工况 (|cmd_speed| <= 200 且 |wheel_spd| <= 200)，且指令与实测轮速匹配 (|cmd - wheel| <= 50) 时，
    // 根据牛顿第一定律，匀速巡航时加速度 dv/dt 严格为 0 (|ax| 约为 0)，绝不可误报为打滑或堵转，立即清零计数器
    else if (abs(cmd_speed - wheel_spd) <= 50) {
        priv->stall_counter = 0;
        slip_condition = false;
        stall_condition = false;
    } else {
        priv->stall_counter = 0;
    }

    // D. 偏航角速度突变检测 (车身侧向打滑 / 剧烈自旋)
    if (abs(priv->current_data.gyro_z_dps) > priv->config.slip_gyro_thresh_dps) {
        slip_condition = true;
    }

    priv->current_data.is_skidding = slip_condition;
    priv->current_data.is_stalled  = stall_condition;

    memcpy(buf, &priv->current_data, sizeof(epgm_imu_data_t));
    return sizeof(epgm_imu_data_t);
}

static int imu_write(hal_device_t *dev, const void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    imu_priv_t *priv = (imu_priv_t *)dev->priv_data;

    // 允许通过 write 注入原始加计/陀螺仪数据 (用于仿真与测试)
    if (len >= sizeof(epgm_imu_data_t)) {
        memcpy(&priv->current_data, buf, sizeof(epgm_imu_data_t));
        return (int)len;
    }
    return HAL_ERR_PARAM;
}

static int imu_ioctl(hal_device_t *dev, uint32_t cmd, void *arg) {
    if (!dev || !dev->is_initialized) return HAL_ERR_NOT_INIT;
    imu_priv_t *priv = (imu_priv_t *)dev->priv_data;

    switch (cmd) {
        case HAL_IOCTL_IMU_FEED_MOTION:
            if (arg) {
                memcpy(&priv->motion_ctx, arg, sizeof(imu_motion_context_t));
                return HAL_OK;
            }
            return HAL_ERR_PARAM;

        case HAL_IOCTL_CLEAR_FLAG:
            priv->current_data.is_skidding = false;
            priv->current_data.is_stalled = false;
            priv->stall_counter = 0;
            return HAL_OK;

        case HAL_IOCTL_RESET:
            memset(&priv->current_data, 0, sizeof(epgm_imu_data_t));
            priv->current_data.accel_z_mg = 1000;
            priv->stall_counter = 0;
            return HAL_OK;

        default:
            return HAL_ERR_PARAM;
    }
}

static int imu_deinit(hal_device_t *dev) {
    if (!dev) return HAL_ERR_PARAM;
    if (dev->priv_data) {
        free(dev->priv_data);
        dev->priv_data = NULL;
    }
    dev->is_initialized = false;
    return HAL_OK;
}

int hal_imu_create(hal_device_t *dev, const char *name) {
    if (!dev || !name) return HAL_ERR_PARAM;
    memset(dev, 0, sizeof(hal_device_t));

    dev->name = name;
    dev->type = HAL_DEV_TYPE_IMU;
    dev->is_initialized = false;
    dev->init = imu_init;
    dev->read = imu_read;
    dev->write = imu_write;
    dev->ioctl = imu_ioctl;
    dev->deinit = imu_deinit;
    dev->priv_data = NULL;

    return HAL_OK;
}
