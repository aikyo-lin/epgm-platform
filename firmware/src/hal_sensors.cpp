/**
 * @file hal_sensors.cpp
 * @brief [ZH] ESP32-S3 传感器统一采集适配层实现 (接入 HAL 传感器聚合器)。
 *        [EN] ESP32-S3 Sensor Unified Acquisition Implementation.
 */

#include "hal_sensors.h"
#include "hal/hal_sensors_aggregator.h"
#if defined(ARDUINO)
#include <Arduino.h>
#endif

void hal_sensors_init(void) {
    hal_sensors_aggregator_init();
}

void hal_sensors_read_distances(uint16_t distances[8]) {
    hal_sensors_aggregator_read_distances(distances);
}

bool hal_sensors_check_bumper(void) {
    return hal_sensors_aggregator_check_bumper();
}
