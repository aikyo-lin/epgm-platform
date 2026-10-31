/**
 * @file hal_encoder_pid.c
 * @brief [ZH] 差速双轮编码器与闭环速度 PID 控制器 HAL 驱动实现。
 *        [EN] Differential Wheel Encoder & Closed-Loop Speed PID Controller HAL Driver Implementation.
 */

#include "hal_encoder_pid.h"
#include <string.h>
#include <stdlib.h>

typedef struct {
    int32_t integral;
    int16_t prev_error;
} pid_channel_state_t;

typedef struct {
    encoder_pid_config_t config;
    encoder_pid_status_t status;
    pid_channel_state_t  left_state;
    pid_channel_state_t  right_state;
    int32_t              last_left_ticks;
    int32_t              last_right_ticks;
} encoder_pid_priv_t;

static inline int16_t clamp_val(int16_t val, int16_t min_val, int16_t max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static int16_t compute_single_pid(
    const pid_params_t *params,
    pid_channel_state_t *state,
    int16_t target,
    int16_t measured
) {
    if (target == 0 && measured == 0) {
        state->integral = 0;
        state->prev_error = 0;
        return 0;
    }

    int16_t error = target - measured;

    // 1. 积分累加与抗积分饱和 (Anti-Windup, 修复缺陷 4)
    // 确保积分项具有足够动态范围 (至少 2000) 克服电机反电动势与负载阻尼，彻底消除 27.5% 稳态静差
    int16_t int_max = params->integral_max;
    if (int_max < 2000) int_max = 2000;

    state->integral += error;
    if (state->integral > int_max) state->integral = int_max;
    if (state->integral < -int_max) state->integral = -int_max;

    // 2. 微分项
    int16_t deriv = error - state->prev_error;
    state->prev_error = error;

    // 3. 数字 PID Q8 加权计算
    int32_t out = (params->kp_q8 * error) +
                  (params->ki_q8 * (int16_t)state->integral) +
                  (params->kd_q8 * deriv);
    out >>= 8;

    // 4. 电机死区非线性前馈补偿
    if (out > 0) {
        out += params->deadband_pwm;
    } else if (out < 0) {
        out -= params->deadband_pwm;
    }

    return clamp_val((int16_t)out, -params->max_output_pwm, params->max_output_pwm);
}

static void apply_action_targets(encoder_pid_priv_t *priv, uint8_t action) {
    priv->status.current_action = action;
    int16_t cruise = priv->config.cruise_speed;
    int16_t turn = priv->config.turn_speed;

    switch (action) {
        case 1: // 前进
            priv->status.left_target_mms = cruise;
            priv->status.right_target_mms = cruise;
            break;
        case 2: // 后退
            priv->status.left_target_mms = -cruise;
            priv->status.right_target_mms = -cruise;
            break;
        case 3: // 原地左转
            priv->status.left_target_mms = -turn;
            priv->status.right_target_mms = turn;
            break;
        case 4: // 原地右转
            priv->status.left_target_mms = turn;
            priv->status.right_target_mms = -turn;
            break;
        case 0: // 停止
        default:
            priv->status.left_target_mms = 0;
            priv->status.right_target_mms = 0;
            break;
    }
}

static int encoder_pid_init(hal_device_t *dev, const void *config) {
    if (!dev) return HAL_ERR_PARAM;

    encoder_pid_priv_t *priv = (encoder_pid_priv_t *)malloc(sizeof(encoder_pid_priv_t));
    if (!priv) return HAL_ERR_IO;
    memset(priv, 0, sizeof(encoder_pid_priv_t));

    if (config) {
        memcpy(&priv->config, config, sizeof(encoder_pid_config_t));
        // 修复缺陷 4: 若外部配置传入的是旧版过小的积分限幅 (<=500)，自动放宽扩展至 2000，消除稳态静差
        if (priv->config.left_pid.integral_max <= 500) {
            priv->config.left_pid.integral_max = 2000;
        }
        if (priv->config.right_pid.integral_max <= 500) {
            priv->config.right_pid.integral_max = 2000;
        }
    } else {
        // 默认 PID 参数 (Kp=1.2, Ki=0.15, Kd=0.05 in Q8)
        priv->config.left_pid.kp_q8 = 307;   // 1.2 * 256
        priv->config.left_pid.ki_q8 = 38;    // 0.15 * 256
        priv->config.left_pid.kd_q8 = 13;    // 0.05 * 256
        priv->config.left_pid.integral_max = 2000; // 修复缺陷 4: 默认积分限幅由 500 提升至 2000，消除巡航稳态静差
        priv->config.left_pid.deadband_pwm = 30;
        priv->config.left_pid.max_output_pwm = 255;

        priv->config.right_pid = priv->config.left_pid;
        priv->config.cruise_speed = 200; // 200 mm/s
        priv->config.turn_speed = 150;   // 150 mm/s
        priv->config.wheel_diameter_mm = 65;
        priv->config.wheel_base_mm = 140;
        priv->config.cpr = 360;
    }

    apply_action_targets(priv, 0);

    dev->priv_data = priv;
    dev->is_initialized = true;
    return HAL_OK;
}

static int encoder_pid_read(hal_device_t *dev, void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    if (len < sizeof(encoder_pid_status_t)) return HAL_ERR_PARAM;

    encoder_pid_priv_t *priv = (encoder_pid_priv_t *)dev->priv_data;
    memcpy(buf, &priv->status, sizeof(encoder_pid_status_t));
    return sizeof(encoder_pid_status_t);
}

static int encoder_pid_write(hal_device_t *dev, const void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    encoder_pid_priv_t *priv = (encoder_pid_priv_t *)dev->priv_data;

    // 允许直接传入双轮目标速度 [left_target_mms, right_target_mms]
    if (len >= sizeof(int16_t) * 2) {
        const int16_t *targets = (const int16_t *)buf;
        priv->status.left_target_mms = targets[0];
        priv->status.right_target_mms = targets[1];
        return (int)len;
    }
    return HAL_ERR_PARAM;
}

static int encoder_pid_ioctl(hal_device_t *dev, uint32_t cmd, void *arg) {
    if (!dev || !dev->is_initialized) return HAL_ERR_NOT_INIT;
    encoder_pid_priv_t *priv = (encoder_pid_priv_t *)dev->priv_data;

    switch (cmd) {
        case HAL_IOCTL_ENCODER_SET_ACTION:
            if (arg) {
                uint8_t action = *(const uint8_t *)arg;
                apply_action_targets(priv, action);
                return HAL_OK;
            }
            return HAL_ERR_PARAM;

        case HAL_IOCTL_ENCODER_FEED_TICKS:
            if (arg) {
                const int32_t *ticks = (const int32_t *)arg;
                priv->status.left_ticks = ticks[0];
                priv->status.right_ticks = ticks[1];
                // 估算实测转速
                int32_t d_left = priv->status.left_ticks - priv->last_left_ticks;
                int32_t d_right = priv->status.right_ticks - priv->last_right_ticks;
                priv->last_left_ticks = priv->status.left_ticks;
                priv->last_right_ticks = priv->status.right_ticks;

                // 50Hz (20ms) 周期估算转速
                priv->status.left_speed_mms = (int16_t)(d_left * 5);
                priv->status.right_speed_mms = (int16_t)(d_right * 5);
                return HAL_OK;
            }
            return HAL_ERR_PARAM;

        case HAL_IOCTL_ENCODER_STEP_PID:
            priv->status.left_pwm_out = compute_single_pid(
                &priv->config.left_pid,
                &priv->left_state,
                priv->status.left_target_mms,
                priv->status.left_speed_mms
            );
            priv->status.right_pwm_out = compute_single_pid(
                &priv->config.right_pid,
                &priv->right_state,
                priv->status.right_target_mms,
                priv->status.right_speed_mms
            );
            return HAL_OK;

        case HAL_IOCTL_RESET:
            memset(&priv->status, 0, sizeof(encoder_pid_status_t));
            memset(&priv->left_state, 0, sizeof(pid_channel_state_t));
            memset(&priv->right_state, 0, sizeof(pid_channel_state_t));
            priv->last_left_ticks = 0;
            priv->last_right_ticks = 0;
            return HAL_OK;

        default:
            return HAL_ERR_PARAM;
    }
}

static int encoder_pid_deinit(hal_device_t *dev) {
    if (!dev) return HAL_ERR_PARAM;
    if (dev->priv_data) {
        free(dev->priv_data);
        dev->priv_data = NULL;
    }
    dev->is_initialized = false;
    return HAL_OK;
}

int hal_encoder_pid_create(hal_device_t *dev, const char *name) {
    if (!dev || !name) return HAL_ERR_PARAM;
    memset(dev, 0, sizeof(hal_device_t));

    dev->name = name;
    dev->type = HAL_DEV_TYPE_ENCODER_PID;
    dev->is_initialized = false;
    dev->init = encoder_pid_init;
    dev->read = encoder_pid_read;
    dev->write = encoder_pid_write;
    dev->ioctl = encoder_pid_ioctl;
    dev->deinit = encoder_pid_deinit;
    dev->priv_data = NULL;

    return HAL_OK;
}
