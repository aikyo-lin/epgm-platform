/**
 * @file hal_hcsr04.c
 * @brief [ZH] HC-SR04 超声波测距传感器 HAL 驱动实现。
 *        [EN] HC-SR04 Ultrasonic Distance Sensor HAL Driver Implementation.
 */

#include "hal_hcsr04.h"
#include <string.h>
#include <stdlib.h>

typedef struct {
    hcsr04_config_t config;
    uint16_t        simulated_distance;
    bool            simulated_timeout;
} hcsr04_priv_t;

static int hcsr04_init(hal_device_t *dev, const void *config) {
    if (!dev) return HAL_ERR_PARAM;

    hcsr04_priv_t *priv = (hcsr04_priv_t *)malloc(sizeof(hcsr04_priv_t));
    if (!priv) return HAL_ERR_IO;
    memset(priv, 0, sizeof(hcsr04_priv_t));

    if (config) {
        memcpy(&priv->config, config, sizeof(hcsr04_config_t));
    } else {
        priv->config.pin_trigger = 12;
        priv->config.pin_echo = 13;
        priv->config.timeout_ms = HCSR04_TIMEOUT_MS;
        priv->config.max_range_mm = HCSR04_MAX_RANGE_MM;
    }

    if (priv->config.timeout_ms == 0) {
        priv->config.timeout_ms = HCSR04_TIMEOUT_MS;
    }
    if (priv->config.max_range_mm == 0) {
        priv->config.max_range_mm = HCSR04_MAX_RANGE_MM;
    }

    priv->simulated_distance = 800;
    priv->simulated_timeout = false;

    dev->priv_data = priv;
    dev->is_initialized = true;
    return HAL_OK;
}

static int hcsr04_read(hal_device_t *dev, void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    hcsr04_priv_t *priv = (hcsr04_priv_t *)dev->priv_data;

    // 25ms 保护超时判断
    bool is_timeout = priv->simulated_timeout;
    uint16_t final_dist = priv->simulated_distance;
    uint32_t echo_us = 0;

    if (is_timeout) {
        // 超时看门狗保护生效: 强制返回超量程并置位超时标记，绝不挂起当前线程
        final_dist = priv->config.max_range_mm;
        echo_us = priv->config.timeout_ms * 1000;
    } else {
        // 距离公式计算: D_mm = (echo_us * 343) / 2000
        // 反推 echo_us: echo_us = (D_mm * 2000) / 343
        echo_us = ((uint32_t)final_dist * 2000) / 343;
        if (final_dist > priv->config.max_range_mm) {
            final_dist = priv->config.max_range_mm;
        }
    }

    if (len >= sizeof(hcsr04_reading_t)) {
        hcsr04_reading_t *out = (hcsr04_reading_t *)buf;
        out->distance_mm = final_dist;
        out->is_timeout = is_timeout;
        out->echo_duration_us = echo_us;
        return sizeof(hcsr04_reading_t);
    } else if (len >= sizeof(uint16_t)) {
        *(uint16_t *)buf = final_dist;
        return sizeof(uint16_t);
    }

    return HAL_ERR_PARAM;
}

static int hcsr04_write(hal_device_t *dev, const void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    hcsr04_priv_t *priv = (hcsr04_priv_t *)dev->priv_data;

    if (len >= sizeof(hcsr04_reading_t)) {
        const hcsr04_reading_t *in = (const hcsr04_reading_t *)buf;
        priv->simulated_distance = in->distance_mm;
        priv->simulated_timeout = in->is_timeout;
        return (int)len;
    } else if (len >= sizeof(uint16_t)) {
        priv->simulated_distance = *(const uint16_t *)buf;
        priv->simulated_timeout = false;
        return (int)len;
    }

    return HAL_ERR_PARAM;
}

static int hcsr04_ioctl(hal_device_t *dev, uint32_t cmd, void *arg) {
    if (!dev || !dev->is_initialized) return HAL_ERR_NOT_INIT;
    hcsr04_priv_t *priv = (hcsr04_priv_t *)dev->priv_data;

    switch (cmd) {
        case HAL_IOCTL_RESET:
            priv->simulated_distance = priv->config.max_range_mm;
            priv->simulated_timeout = false;
            return HAL_OK;

        case HAL_IOCTL_SET_CONFIG:
            if (arg) {
                memcpy(&priv->config, arg, sizeof(hcsr04_config_t));
                return HAL_OK;
            }
            return HAL_ERR_PARAM;

        default:
            return HAL_ERR_PARAM;
    }
}

static int hcsr04_deinit(hal_device_t *dev) {
    if (!dev) return HAL_ERR_PARAM;
    if (dev->priv_data) {
        free(dev->priv_data);
        dev->priv_data = NULL;
    }
    dev->is_initialized = false;
    return HAL_OK;
}

int hal_hcsr04_create(hal_device_t *dev, const char *name) {
    if (!dev || !name) return HAL_ERR_PARAM;
    memset(dev, 0, sizeof(hal_device_t));

    dev->name = name;
    dev->type = HAL_DEV_TYPE_ULTRASONIC;
    dev->is_initialized = false;
    dev->init = hcsr04_init;
    dev->read = hcsr04_read;
    dev->write = hcsr04_write;
    dev->ioctl = hcsr04_ioctl;
    dev->deinit = hcsr04_deinit;
    dev->priv_data = NULL;

    return HAL_OK;
}
