/**
 * @file hal_bumper.c
 * @brief [ZH] 机械防碰撞微动触须开关 HAL 驱动实现。
 *        [EN] Mechanical Collision Bumper / Whisker Microswitch HAL Driver Implementation.
 */

#include "hal_bumper.h"
#include <string.h>
#include <stdlib.h>

#if __has_include("mock_hardware.h")
#include "mock_hardware.h"
#elif __has_include("../../test/mock_hardware.h")
#include "../../test/mock_hardware.h"
#endif

#if __has_include("mock_freertos.h")
#include "mock_freertos.h"
#elif __has_include("../../test/mock_freertos.h")
#include "../../test/mock_freertos.h"
#elif defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

typedef struct {
    bumper_config_t config;               ///< 微动开关配置（引脚分配、消抖时间、极性）
    bumper_status_t status;               ///< 当前对外上报状态（掩码、触发标志、急停标志）
    uint32_t        last_trigger_tick;    ///< 初次检测到电平跳变的时间戳（毫秒）
    uint8_t         raw_pins_state;       ///< 上一次采样的原始物理引脚电平掩码
    bool            is_software_injected; ///< 是否处于纯软件仿真注入模式
} bumper_priv_t;

static int bumper_init(hal_device_t *dev, const void *config) {
    if (!dev) return HAL_ERR_PARAM;

    bumper_priv_t *priv = (bumper_priv_t *)malloc(sizeof(bumper_priv_t));
    if (!priv) return HAL_ERR_IO;
    memset(priv, 0, sizeof(bumper_priv_t));

    if (config) {
        memcpy(&priv->config, config, sizeof(bumper_config_t));
    } else {
        priv->config.pin_front_left = 14;
        priv->config.pin_front_right = 15;
        priv->config.pin_rear = 16;
        priv->config.debounce_ms = 5;
        priv->config.active_low = true;
    }

    if (priv->config.debounce_ms == 0) {
        priv->config.debounce_ms = 5;
    }

    // 初始化运行状态与消抖时间戳
    priv->last_trigger_tick = 0;
    priv->raw_pins_state = 0;
    priv->is_software_injected = false;

    dev->priv_data = priv;
    dev->is_initialized = true;
    return HAL_OK;
}

static int bumper_read(hal_device_t *dev, void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    bumper_priv_t *priv = (bumper_priv_t *)dev->priv_data;

    // 修复缺陷 1 (CRITICAL BUG): 物理引脚采样临时掩码必须初始化为 0，严禁继承上一轮状态导致自锁死锁
    uint8_t raw_mask = 0;

#if defined(MOCK_HARDWARE_H) || defined(_MOCK_HARDWARE_H_)
    // 若在测试环境下，实时采样硬件 GPIO 引脚电平 (active_low: 低电平有效)
    if (priv->config.active_low) {
        if (mock_gpio_get_level(priv->config.pin_front_left) == 0)  raw_mask |= BUMPER_MASK_FRONT_LEFT;
        if (mock_gpio_get_level(priv->config.pin_front_right) == 0) raw_mask |= BUMPER_MASK_FRONT_RIGHT;
        if (mock_gpio_get_level(priv->config.pin_rear) == 0)        raw_mask |= BUMPER_MASK_REAR;
    } else {
        // [ZH] 补充 active_low == false 分支：高电平有效时采样逻辑电平 1 (防范 Finding 2.1 缺陷)
        // [EN] Add active_low == false branch: sample high level (1) when active-high
        if (mock_gpio_get_level(priv->config.pin_front_left) == 1)  raw_mask |= BUMPER_MASK_FRONT_LEFT;
        if (mock_gpio_get_level(priv->config.pin_front_right) == 1) raw_mask |= BUMPER_MASK_FRONT_RIGHT;
        if (mock_gpio_get_level(priv->config.pin_rear) == 1)        raw_mask |= BUMPER_MASK_REAR;
    }
#endif

    // 获取当前系统高精度毫秒时间戳，用于 5ms 软件消抖滤波
    uint32_t now_ms = 0;
#if defined(MOCK_FREERTOS_H) || defined(_MOCK_FREERTOS_H_)
    now_ms = (uint32_t)xTaskGetTickCount();
#elif defined(MOCK_HARDWARE_H) || defined(_MOCK_HARDWARE_H_)
    now_ms = (uint32_t)(mock_hardware_get_time_us() / 1000ULL);
#endif

    // 修复缺陷 2 (HIGH BUG): 实现 5ms 软件消抖数字滤波 (Debounce Filter)
    if (raw_mask != 0) {
        // 探测到物理微动触碰，退出纯软件注入模式，以真实硬件引脚状态为最高优先级
        priv->is_software_injected = false;

        // 多触须同时触碰工况（如前左+前右同时触发，掩码位 > 1）：判定为确定性正面硬碰撞，立即置位急停
        if ((raw_mask & (raw_mask - 1)) != 0) {
            priv->status.bumper_mask = raw_mask;
            priv->status.is_triggered = true;
            priv->status.emergency_stop = true;
            priv->status.trigger_count++;
            priv->last_trigger_tick = now_ms;
            priv->raw_pins_state = raw_mask;
        } else {
            // 单触须微动状态：可能受到机械弹跳 (1~3ms) 或接触毛刺干扰，启动 5ms 软件滤波
            if (priv->raw_pins_state != raw_mask) {
                // 首次由释放跳变为触碰，记录起始时间戳，开启消抖时间窗口
                priv->raw_pins_state = raw_mask;
                priv->last_trigger_tick = now_ms;
                // [ZH] 防御加固：消抖窗口期间隔离未确认掩码，防止单字节读取绕过消抖滤波 (防范 Finding 2.2 缺陷)
                // [EN] Defensive hardening: do not expose unconfirmed bounce mask during debounce window
                if (!priv->status.is_triggered) {
                    priv->status.bumper_mask = 0;
                }
            } else {
                // 持续保持有效电平，检查持续时间是否达到或超过 debounce_ms
                uint32_t elapsed = now_ms - priv->last_trigger_tick;
                if (elapsed >= priv->config.debounce_ms) {
                    priv->status.bumper_mask = raw_mask; // 消抖确认后方可透出掩码
                    if (!priv->status.is_triggered) {
                        priv->status.is_triggered = true;
                        priv->status.emergency_stop = true;
                        priv->status.trigger_count++;
                    }
                }
            }
        }
    } else {
        // 修复缺陷 1: 物理微动触须完全弹起释放 (raw_mask == 0) 时，彻底清除碰撞与急停状态
        if (!priv->is_software_injected) {
            priv->status.bumper_mask = 0;
            priv->status.is_triggered = false;
            priv->status.emergency_stop = false;
            priv->last_trigger_tick = 0;
            priv->raw_pins_state = 0;
        }
    }

    if (len >= sizeof(bumper_status_t)) {
        memcpy(buf, &priv->status, sizeof(bumper_status_t));
        return sizeof(bumper_status_t);
    } else if (len >= sizeof(uint8_t)) {
        *(uint8_t *)buf = priv->status.bumper_mask;
        return sizeof(uint8_t);
    }

    return HAL_ERR_PARAM;
}

static int bumper_write(hal_device_t *dev, const void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    bumper_priv_t *priv = (bumper_priv_t *)dev->priv_data;

    // 允许通过 write 显式注入触碰掩码 (用于宿主机测试与仿真多模态融合校验)
    if (len >= sizeof(uint8_t)) {
        uint8_t mask = *(const uint8_t *)buf;
        priv->status.bumper_mask = mask;
        priv->status.is_triggered = (mask != 0);
        priv->status.emergency_stop = (mask != 0);
        priv->is_software_injected = (mask != 0);
        if (mask != 0) {
            priv->status.trigger_count++;
        }
        return sizeof(uint8_t);
    }

    return HAL_ERR_PARAM;
}

static int bumper_ioctl(hal_device_t *dev, uint32_t cmd, void *arg) {
    if (!dev || !dev->is_initialized) return HAL_ERR_NOT_INIT;
    bumper_priv_t *priv = (bumper_priv_t *)dev->priv_data;

    switch (cmd) {
        case HAL_IOCTL_CLEAR_FLAG:
            // 显式清除碰撞标志与急停状态，重置消抖时间戳与软件注入标记
            priv->status.bumper_mask = 0;
            priv->status.is_triggered = false;
            priv->status.emergency_stop = false;
            priv->last_trigger_tick = 0;
            priv->raw_pins_state = 0;
            priv->is_software_injected = false;
            return HAL_OK;

        case HAL_IOCTL_RESET:
            memset(&priv->status, 0, sizeof(bumper_status_t));
            priv->last_trigger_tick = 0;
            priv->raw_pins_state = 0;
            priv->is_software_injected = false;
            return HAL_OK;

        default:
            return HAL_ERR_PARAM;
    }
}

static int bumper_deinit(hal_device_t *dev) {
    if (!dev) return HAL_ERR_PARAM;
    if (dev->priv_data) {
        free(dev->priv_data);
        dev->priv_data = NULL;
    }
    dev->is_initialized = false;
    return HAL_OK;
}

int hal_bumper_create(hal_device_t *dev, const char *name) {
    if (!dev || !name) return HAL_ERR_PARAM;
    memset(dev, 0, sizeof(hal_device_t));

    dev->name = name;
    dev->type = HAL_DEV_TYPE_BUMPER;
    dev->is_initialized = false;
    dev->init = bumper_init;
    dev->read = bumper_read;
    dev->write = bumper_write;
    dev->ioctl = bumper_ioctl;
    dev->deinit = bumper_deinit;
    dev->priv_data = NULL;

    return HAL_OK;
}
