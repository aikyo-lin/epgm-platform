/**
 * @file hal_device.c
 * @brief [ZH] 通用机器人外设硬件抽象层 (HAL) 设备基类与驱动注册表实现。
 *        [EN] Universal Robot Peripheral HAL Device Base & Registry Implementation.
 */

#include "hal_device.h"
#include <string.h>

static hal_device_t *s_device_table[HAL_MAX_DEVICES] = {NULL};
static size_t        s_device_count = 0;

int hal_device_register(hal_device_t *dev) {
    if (!dev || !dev->name) {
        return HAL_ERR_PARAM;
    }

    // 检查重复注册
    for (size_t i = 0; i < s_device_count; ++i) {
        if (s_device_table[i] == dev || strcmp(s_device_table[i]->name, dev->name) == 0) {
            return HAL_OK; // 已存在则直接成功返回
        }
    }

    if (s_device_count >= HAL_MAX_DEVICES) {
        return HAL_ERR_FULL;
    }

    s_device_table[s_device_count++] = dev;
    return HAL_OK;
}

int hal_device_unregister(hal_device_t *dev) {
    if (!dev) return HAL_ERR_PARAM;

    for (size_t i = 0; i < s_device_count; ++i) {
        if (s_device_table[i] == dev) {
            // 将后续元素前移填补空缺
            for (size_t j = i; j < s_device_count - 1; ++j) {
                s_device_table[j] = s_device_table[j + 1];
            }
            s_device_table[--s_device_count] = NULL;
            return HAL_OK;
        }
    }

    return HAL_ERR_NOT_FOUND;
}

hal_device_t *hal_device_find(const char *name) {
    if (!name) return NULL;
    for (size_t i = 0; i < s_device_count; ++i) {
        if (s_device_table[i] && strcmp(s_device_table[i]->name, name) == 0) {
            return s_device_table[i];
        }
    }
    return NULL;
}

hal_device_t *hal_device_find_by_type(hal_device_type_t type, size_t index) {
    size_t match_count = 0;
    for (size_t i = 0; i < s_device_count; ++i) {
        if (s_device_table[i] && s_device_table[i]->type == type) {
            if (match_count == index) {
                return s_device_table[i];
            }
            match_count++;
        }
    }
    return NULL;
}

size_t hal_device_count(void) {
    return s_device_count;
}

void hal_device_registry_reset(void) {
    for (size_t i = 0; i < HAL_MAX_DEVICES; ++i) {
        s_device_table[i] = NULL;
    }
    s_device_count = 0;
}
