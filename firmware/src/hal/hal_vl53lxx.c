/**
 * @file hal_vl53lxx.c
 * @brief [ZH] VL53L0X / VL53L1X ToF 激光测距传感器 HAL 驱动实现。
 *        [EN] VL53L0X / VL53L1X ToF Laser Ranging Sensor HAL Driver Implementation.
 */

#include "hal_vl53lxx.h"
#include <string.h>
#include <stdlib.h>

#if __has_include("mock_hardware.h")
#include "mock_hardware.h"
#elif __has_include("../../test/mock_hardware.h")
#include "../../test/mock_hardware.h"
#endif

typedef struct {
    vl53lxx_config_t config;
    uint16_t         last_raw_distance;
    uint8_t          last_range_status;
} vl53lxx_priv_t;

static int vl53lxx_init(hal_device_t *dev, const void *config) {
    if (!dev) return HAL_ERR_PARAM;

    vl53lxx_priv_t *priv = (vl53lxx_priv_t *)malloc(sizeof(vl53lxx_priv_t));
    if (!priv) return HAL_ERR_IO;
    memset(priv, 0, sizeof(vl53lxx_priv_t));

    if (config) {
        memcpy(&priv->config, config, sizeof(vl53lxx_config_t));
    } else {
        priv->config.model = VL53LXX_MODEL_VL53L0X;
        priv->config.i2c_addr = 0x29;
        priv->config.mux_channel = 0xFF;
        priv->config.timing_budget_ms = 33;
        priv->config.offset_mm = 0;
        priv->config.crosstalk_kcps = 0;
        priv->config.max_range_mm = 2000;
    }

    if (priv->config.max_range_mm == 0) {
        priv->config.max_range_mm = (priv->config.model == VL53LXX_MODEL_VL53L1X) ? 4000 : 2000;
    }

    priv->last_raw_distance = priv->config.max_range_mm;
    priv->last_range_status = VL53LXX_RANGE_VALID;

    dev->priv_data = priv;
    dev->is_initialized = true;
    return HAL_OK;
}

static int vl53lxx_read(hal_device_t *dev, void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    vl53lxx_priv_t *priv = (vl53lxx_priv_t *)dev->priv_data;

    // 默认测试数据采集 (若有 mock hardware 则优先采集模拟通道)
    uint16_t raw_dist = priv->last_raw_distance;
    uint8_t  status = priv->last_range_status;

#if defined(MOCK_HARDWARE_H) || defined(_MOCK_HARDWARE_H_) || defined(MOCK_HARDWARE_MOCK)
    // 如果指定了 TCA9548A 复用器通道，模拟读取该通道
    if (priv->config.mux_channel < 8) {
        // 读取 mock 硬件通道
        raw_dist = priv->last_raw_distance;
        status = priv->last_range_status;
    }
#endif

    // 状态过滤: 若测距状态异常，返回预设最大量程
    uint16_t final_dist = raw_dist;
    if (status != VL53LXX_RANGE_VALID) {
        final_dist = priv->config.max_range_mm;
    } else {
        // 叠加硬件零点偏差标定
        int32_t calibrated = (int32_t)raw_dist + priv->config.offset_mm;
        if (calibrated < 30) calibrated = 30; // VL53L0X 盲区为 30mm
        if (calibrated > priv->config.max_range_mm) calibrated = priv->config.max_range_mm;
        final_dist = (uint16_t)calibrated;
    }

    if (len >= sizeof(vl53lxx_reading_t)) {
        vl53lxx_reading_t *out = (vl53lxx_reading_t *)buf;
        out->distance_mm = final_dist;
        out->status = (vl53lxx_range_status_t)status;
        out->timestamp_us = 0;
        return sizeof(vl53lxx_reading_t);
    } else if (len >= sizeof(uint16_t)) {
        *(uint16_t *)buf = final_dist;
        return sizeof(uint16_t);
    }

    return HAL_ERR_PARAM;
}

static int vl53lxx_write(hal_device_t *dev, const void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    vl53lxx_priv_t *priv = (vl53lxx_priv_t *)dev->priv_data;

    // 允许通过 write 注入测距数据 (测试模式)
    if (len >= sizeof(vl53lxx_reading_t)) {
        const vl53lxx_reading_t *in = (const vl53lxx_reading_t *)buf;
        priv->last_raw_distance = in->distance_mm;
        priv->last_range_status = (uint8_t)in->status;
        return (int)len;
    } else if (len >= sizeof(uint16_t)) {
        priv->last_raw_distance = *(const uint16_t *)buf;
        priv->last_range_status = VL53LXX_RANGE_VALID;
        return (int)len;
    }

    return HAL_ERR_PARAM;
}

static int vl53lxx_ioctl(hal_device_t *dev, uint32_t cmd, void *arg) {
    if (!dev || !dev->is_initialized) return HAL_ERR_NOT_INIT;
    vl53lxx_priv_t *priv = (vl53lxx_priv_t *)dev->priv_data;

    switch (cmd) {
        case HAL_IOCTL_CALIBRATE:
            if (arg) {
                const int16_t *offset = (const int16_t *)arg;
                priv->config.offset_mm = *offset;
                return HAL_OK;
            }
            return HAL_ERR_PARAM;

        case HAL_IOCTL_SET_CONFIG:
            if (arg) {
                memcpy(&priv->config, arg, sizeof(vl53lxx_config_t));
                return HAL_OK;
            }
            return HAL_ERR_PARAM;

        case HAL_IOCTL_RESET:
            priv->last_raw_distance = priv->config.max_range_mm;
            priv->last_range_status = VL53LXX_RANGE_VALID;
            return HAL_OK;

        default:
            return HAL_ERR_PARAM;
    }
}

static int vl53lxx_deinit(hal_device_t *dev) {
    if (!dev) return HAL_ERR_PARAM;
    if (dev->priv_data) {
        free(dev->priv_data);
        dev->priv_data = NULL;
    }
    dev->is_initialized = false;
    return HAL_OK;
}

int hal_vl53lxx_create(hal_device_t *dev, const char *name) {
    if (!dev || !name) return HAL_ERR_PARAM;
    memset(dev, 0, sizeof(hal_device_t));

    dev->name = name;
    dev->type = HAL_DEV_TYPE_TOF;
    dev->is_initialized = false;
    dev->init = vl53lxx_init;
    dev->read = vl53lxx_read;
    dev->write = vl53lxx_write;
    dev->ioctl = vl53lxx_ioctl;
    dev->deinit = vl53lxx_deinit;
    dev->priv_data = NULL;

    return HAL_OK;
}
