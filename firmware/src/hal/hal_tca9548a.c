/**
 * @file hal_tca9548a.c
 * @brief [ZH] TCA9548A 8通道 I2C 多路复用器 HAL 驱动实现。
 *        [EN] TCA9548A 8-Channel I2C Multiplexer HAL Driver Implementation.
 */

#include "hal_tca9548a.h"
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
#include "freertos/semphr.h"
#endif

typedef struct {
    tca9548a_config_t config;               ///< 复用器配置（I2C 地址、默认通道、互斥锁使能）
    uint8_t           current_channel_mask; ///< 当前选通的通道位掩码 (0x01~0x80)
    SemaphoreHandle_t bus_mutex;            ///< FreeRTOS 总线原子互斥锁句柄 (修复缺陷 5)
} tca9548a_priv_t;

static int tca9548a_init(hal_device_t *dev, const void *config) {
    if (!dev) return HAL_ERR_PARAM;

    tca9548a_priv_t *priv = (tca9548a_priv_t *)malloc(sizeof(tca9548a_priv_t));
    if (!priv) return HAL_ERR_IO;
    memset(priv, 0, sizeof(tca9548a_priv_t));

    if (config) {
        memcpy(&priv->config, config, sizeof(tca9548a_config_t));
    } else {
        priv->config.i2c_addr = TCA9548A_DEFAULT_I2C_ADDR;
        priv->config.default_channel = 0;
        priv->config.enable_mutex = true;
    }

    uint8_t ch = priv->config.default_channel;
    priv->current_channel_mask = (ch < 8) ? (uint8_t)(1 << ch) : 0x01;

    // 修复缺陷 5: 若使能互斥锁，创建 FreeRTOS 互斥锁句柄以保护多任务并发选通
    if (priv->config.enable_mutex) {
        priv->bus_mutex = xSemaphoreCreateMutex();
    } else {
        priv->bus_mutex = NULL;
    }

    dev->priv_data = priv;
    dev->is_initialized = true;

#if defined(MOCK_HARDWARE_H) || defined(_MOCK_HARDWARE_H_)
    mock_i2c_write(priv->config.i2c_addr, &priv->current_channel_mask, 1);
#endif

    return HAL_OK;
}

static int tca9548a_read(hal_device_t *dev, void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    if (len < sizeof(uint8_t)) return HAL_ERR_PARAM;

    tca9548a_priv_t *priv = (tca9548a_priv_t *)dev->priv_data;

    // 互斥锁并发保护
    if (priv->bus_mutex) {
        if (xSemaphoreTake(priv->bus_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return HAL_ERR_TIMEOUT;
        }
    }

    *(uint8_t *)buf = priv->current_channel_mask;

    if (priv->bus_mutex) {
        xSemaphoreGive(priv->bus_mutex);
    }
    return sizeof(uint8_t);
}

static int tca9548a_write(hal_device_t *dev, const void *buf, size_t len) {
    if (!dev || !dev->is_initialized || !buf) return HAL_ERR_NOT_INIT;
    if (len < sizeof(uint8_t)) return HAL_ERR_PARAM;

    tca9548a_priv_t *priv = (tca9548a_priv_t *)dev->priv_data;

    // 互斥锁并发保护
    if (priv->bus_mutex) {
        if (xSemaphoreTake(priv->bus_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return HAL_ERR_TIMEOUT;
        }
    }

    priv->current_channel_mask = *(const uint8_t *)buf;

#if defined(MOCK_HARDWARE_H) || defined(_MOCK_HARDWARE_H_)
    mock_i2c_write(priv->config.i2c_addr, &priv->current_channel_mask, 1);
#endif

    if (priv->bus_mutex) {
        xSemaphoreGive(priv->bus_mutex);
    }
    return sizeof(uint8_t);
}

static int tca9548a_ioctl(hal_device_t *dev, uint32_t cmd, void *arg) {
    if (!dev || !dev->is_initialized) return HAL_ERR_NOT_INIT;
    tca9548a_priv_t *priv = (tca9548a_priv_t *)dev->priv_data;

    switch (cmd) {
        case HAL_IOCTL_SET_CHANNEL:
            if (arg) {
                uint8_t ch = *(const uint8_t *)arg;
                if (ch < 8) {
                    if (priv->bus_mutex) {
                        if (xSemaphoreTake(priv->bus_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
                            return HAL_ERR_TIMEOUT;
                        }
                    }

                    priv->current_channel_mask = (uint8_t)(1 << ch);
#if defined(MOCK_HARDWARE_H) || defined(_MOCK_HARDWARE_H_)
                    mock_i2c_write(priv->config.i2c_addr, &priv->current_channel_mask, 1);
#endif

                    if (priv->bus_mutex) {
                        xSemaphoreGive(priv->bus_mutex);
                    }
                    return HAL_OK;
                }
            }
            return HAL_ERR_PARAM;

        case HAL_IOCTL_RESET:
            if (priv->bus_mutex) {
                if (xSemaphoreTake(priv->bus_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
                    return HAL_ERR_TIMEOUT;
                }
            }

            priv->current_channel_mask = 0x01; // 重置为通道 0
#if defined(MOCK_HARDWARE_H) || defined(_MOCK_HARDWARE_H_)
            mock_i2c_write(priv->config.i2c_addr, &priv->current_channel_mask, 1);
#endif

            if (priv->bus_mutex) {
                xSemaphoreGive(priv->bus_mutex);
            }
            return HAL_OK;

        default:
            return HAL_ERR_PARAM;
    }
}

static int tca9548a_deinit(hal_device_t *dev) {
    if (!dev) return HAL_ERR_PARAM;
    if (dev->priv_data) {
        tca9548a_priv_t *priv = (tca9548a_priv_t *)dev->priv_data;
        // 销毁总线原子互斥锁
        if (priv->bus_mutex) {
            vSemaphoreDelete(priv->bus_mutex);
            priv->bus_mutex = NULL;
        }
        free(dev->priv_data);
        dev->priv_data = NULL;
    }
    dev->is_initialized = false;
    return HAL_OK;
}

int hal_tca9548a_create(hal_device_t *dev, const char *name) {
    if (!dev || !name) return HAL_ERR_PARAM;
    memset(dev, 0, sizeof(hal_device_t));

    dev->name = name;
    dev->type = HAL_DEV_TYPE_I2C_MUX;
    dev->is_initialized = false;
    dev->init = tca9548a_init;
    dev->read = tca9548a_read;
    dev->write = tca9548a_write;
    dev->ioctl = tca9548a_ioctl;
    dev->deinit = tca9548a_deinit;
    dev->priv_data = NULL;

    return HAL_OK;
}

int hal_tca9548a_select_channel(hal_device_t *dev, uint8_t channel_id) {
    return dev->ioctl(dev, HAL_IOCTL_SET_CHANNEL, &channel_id);
}

int hal_tca9548a_get_channel_mask(hal_device_t *dev, uint8_t *out_mask) {
    if (!dev || !out_mask) return HAL_ERR_PARAM;
    int res = dev->read(dev, out_mask, sizeof(uint8_t));
    return (res == sizeof(uint8_t)) ? HAL_OK : HAL_ERR_IO;
}
