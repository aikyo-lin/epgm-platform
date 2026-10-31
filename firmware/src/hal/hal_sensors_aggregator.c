/**
 * @file hal_sensors_aggregator.c
 * @brief [ZH] 机器人全向传感器统一聚合抽象层实现。
 *        [EN] Robot Omnidirectional Sensor Unified Aggregator HAL Implementation.
 */

#include "hal_sensors_aggregator.h"
#include <string.h>
#include <stdio.h>

static hal_device_t s_tof_devices[EPGM_SECTORS_COUNT];
static hal_device_t s_ultrasonic_dev;
static hal_device_t s_imu_dev;
static hal_device_t s_i2c_mux_dev;
static hal_device_t s_bumper_dev;

static bool s_aggregator_initialized = false;

static const char *s_tof_names[EPGM_SECTORS_COUNT] = {
    "tof_sector_0", "tof_sector_1", "tof_sector_2", "tof_sector_3",
    "tof_sector_4", "tof_sector_5", "tof_sector_6", "tof_sector_7"
};

int hal_sensors_aggregator_init(void) {
    if (s_aggregator_initialized) return HAL_OK;

    // 1. 初始化 I2C 多路复用器 TCA9548A
    hal_tca9548a_create(&s_i2c_mux_dev, "i2c_mux_0");
    tca9548a_config_t mux_cfg = {
        .i2c_addr = TCA9548A_DEFAULT_I2C_ADDR,
        .default_channel = 0,
        .enable_mutex = true
    };
    s_i2c_mux_dev.init(&s_i2c_mux_dev, &mux_cfg);
    hal_device_register(&s_i2c_mux_dev);

    // 2. 初始化 8 路 ToF 激光测距传感器 (分别挂载于 TCA9548A 通道 0~7)
    for (int i = 0; i < EPGM_SECTORS_COUNT; ++i) {
        hal_vl53lxx_create(&s_tof_devices[i], s_tof_names[i]);

        vl53lxx_config_t tof_cfg = {
            .model = VL53LXX_MODEL_VL53L0X,
            .i2c_addr = 0x29,
            .mux_channel = (uint8_t)i,
            .timing_budget_ms = 33,
            .offset_mm = 0,
            .crosstalk_kcps = 0,
            .max_range_mm = 2000
        };
        s_tof_devices[i].init(&s_tof_devices[i], &tof_cfg);
        hal_device_register(&s_tof_devices[i]);
    }

    // 3. 初始化 HC-SR04 超声波测距传感器 (补充正前方向)
    hal_hcsr04_create(&s_ultrasonic_dev, "us_front");
    hcsr04_config_t us_cfg = {
        .pin_trigger = 12,
        .pin_echo = 13,
        .timeout_ms = 25,
        .max_range_mm = 4000
    };
    s_ultrasonic_dev.init(&s_ultrasonic_dev, &us_cfg);
    hal_device_register(&s_ultrasonic_dev);

    // 4. 初始化 6轴 IMU (MPU6050)
    hal_imu_create(&s_imu_dev, "imu_main");
    imu_config_t imu_cfg = {
        .model = IMU_MODEL_MPU6050,
        .i2c_addr = 0x68,
        .accel_scale_g = 4,
        .gyro_scale_dps = 1000,
        .filter_alpha_q8 = 251,
        .stall_accel_thresh_mg = 30,
        .slip_gyro_thresh_dps = 50
    };
    s_imu_dev.init(&s_imu_dev, &imu_cfg);
    hal_device_register(&s_imu_dev);

    // 5. 初始化微动碰撞触须
    hal_bumper_create(&s_bumper_dev, "bumper_array");
    bumper_config_t bmp_cfg = {
        .pin_front_left = 14,
        .pin_front_right = 15,
        .pin_rear = 16,
        .debounce_ms = 5,
        .active_low = true
    };
    s_bumper_dev.init(&s_bumper_dev, &bmp_cfg);
    hal_device_register(&s_bumper_dev);

    s_aggregator_initialized = true;
    return HAL_OK;
}

int hal_sensors_aggregator_read(epgm_sensor_frame_t *frame) {
    if (!frame) return HAL_ERR_PARAM;
    if (!s_aggregator_initialized) {
        hal_sensors_aggregator_init();
    }

    memset(frame, 0, sizeof(epgm_sensor_frame_t));

    // 1. 采集 8 路 ToF 激光读数
    for (int i = 0; i < EPGM_SECTORS_COUNT; ++i) {
        // 动态选通 TCA9548A 对应通道
        hal_tca9548a_select_channel(&s_i2c_mux_dev, (uint8_t)i);

        uint16_t dist = 2000;
        s_tof_devices[i].read(&s_tof_devices[i], &dist, sizeof(uint16_t));
        frame->distances_mm[i] = dist;
    }

    // 2. 采集 HC-SR04 超声波测距读数 (与正前方 ToF 互补融合)
    uint16_t us_dist = 4000;
    if (s_ultrasonic_dev.read(&s_ultrasonic_dev, &us_dist, sizeof(uint16_t)) == sizeof(uint16_t)) {
        // 正前方 (Ray 0) 取 ToF 与超声波两者的安全最小值
        if (us_dist < frame->distances_mm[0]) {
            frame->distances_mm[0] = us_dist;
        }
    }

    // 3. 采集 6轴 IMU 姿态与打滑数据
    s_imu_dev.read(&s_imu_dev, &frame->imu, sizeof(epgm_imu_data_t));

    // 4. 采集微动触须碰撞状态
    bumper_status_t bmp_status;
    if (s_bumper_dev.read(&s_bumper_dev, &bmp_status, sizeof(bumper_status_t)) == sizeof(bumper_status_t)) {
        frame->bumper_hit_mask = bmp_status.bumper_mask;
        frame->emergency_stop = bmp_status.emergency_stop;
    }

    return HAL_OK;
}

void hal_sensors_aggregator_read_distances(uint16_t distances[EPGM_SECTORS_COUNT]) {
    if (!distances) return;
    epgm_sensor_frame_t frame;
    hal_sensors_aggregator_read(&frame);
    memcpy(distances, frame.distances_mm, sizeof(uint16_t) * EPGM_SECTORS_COUNT);
}

bool hal_sensors_aggregator_check_bumper(void) {
    if (!s_aggregator_initialized) {
        hal_sensors_aggregator_init();
    }
    uint8_t mask = 0;
    s_bumper_dev.read(&s_bumper_dev, &mask, sizeof(uint8_t));
    return (mask != 0);
}

void hal_sensors_aggregator_deinit(void) {
    if (!s_aggregator_initialized) return;

    for (int i = 0; i < EPGM_SECTORS_COUNT; ++i) {
        if (s_tof_devices[i].deinit) s_tof_devices[i].deinit(&s_tof_devices[i]);
    }
    if (s_ultrasonic_dev.deinit) s_ultrasonic_dev.deinit(&s_ultrasonic_dev);
    if (s_imu_dev.deinit) s_imu_dev.deinit(&s_imu_dev);
    if (s_i2c_mux_dev.deinit) s_i2c_mux_dev.deinit(&s_i2c_mux_dev);
    if (s_bumper_dev.deinit) s_bumper_dev.deinit(&s_bumper_dev);

    // 清空注册表
    for (size_t i = 0; i < hal_device_count(); ++i) {
        hal_device_t *d = hal_device_find_by_type(HAL_DEV_TYPE_UNKNOWN, 0); // 或者直接 reset
        (void)d;
    }
    hal_device_registry_reset();
    s_aggregator_initialized = false;
}
