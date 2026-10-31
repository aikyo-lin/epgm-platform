/**
 * @file mock_hardware.c
 * @brief [ZH] 宿主机硬件抽象层 (I2C、GPIO、定时器与主流外设) 模拟实现。
 *        [EN] Host Hardware Mock Implementation.
 */

#include "mock_hardware.h"
#include <string.h>

#if defined(_WIN32) || defined(_WIN64)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/time.h>
#endif

// ==============================================================================
// 模拟硬件内部状态
// ==============================================================================
static uint8_t s_gpio_levels[64];
static uint8_t s_tca9548a_mask = 0x01; // 默认通道 0 选通

typedef struct {
    uint16_t distance_mm;
    uint8_t  range_status;
} mock_tof_state_t;

static mock_tof_state_t s_tof_channels[8];

typedef struct {
    uint16_t distance_mm;
    bool     is_timeout;
} mock_ultrasonic_state_t;

static mock_ultrasonic_state_t s_ultrasonic_state;

typedef struct {
    int16_t ax_mg;
    int16_t ay_mg;
    int16_t az_mg;
    int16_t gx_dps;
    int16_t gy_dps;
    int16_t gz_dps;
} mock_imu_state_t;

static mock_imu_state_t s_imu_state;

typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
} mock_encoder_state_t;

static mock_encoder_state_t s_encoder_state;

// ==============================================================================
// 硬件生命周期
// ==============================================================================

void mock_hardware_init(void) {
    mock_hardware_reset();
}

void mock_hardware_reset(void) {
    // GPIO 默认全部为高电平 (带上拉电阻)
    memset(s_gpio_levels, 1, sizeof(s_gpio_levels));
    s_tca9548a_mask = 0x01;

    for (int i = 0; i < 8; ++i) {
        s_tof_channels[i].distance_mm = 800; // 默认开阔 800mm
        s_tof_channels[i].range_status = 0;   // 状态正常
    }

    s_ultrasonic_state.distance_mm = 800;
    s_ultrasonic_state.is_timeout = false;

    s_imu_state.ax_mg = 0;
    s_imu_state.ay_mg = 0;
    s_imu_state.az_mg = 1000; // 1g 重力
    s_imu_state.gx_dps = 0;
    s_imu_state.gy_dps = 0;
    s_imu_state.gz_dps = 0;

    s_encoder_state.left_ticks = 0;
    s_encoder_state.right_ticks = 0;
}

uint64_t mock_hardware_get_time_us(void) {
#if defined(_WIN32) || defined(_WIN64)
    static LARGE_INTEGER freq;
    static BOOL initialized = FALSE;
    if (!initialized) {
        QueryPerformanceFrequency(&freq);
        initialized = TRUE;
    }
    LARGE_INTEGER counter;
    QueryPerformanceCounter(&counter);
    return (uint64_t)((counter.QuadPart * 1000000ULL) / freq.QuadPart);
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((uint64_t)tv.tv_sec * 1000000ULL) + (uint64_t)tv.tv_usec;
#endif
}

// ==============================================================================
// GPIO 模拟
// ==============================================================================

void mock_gpio_set_level(uint32_t pin, uint8_t level) {
    if (pin < sizeof(s_gpio_levels)) {
        s_gpio_levels[pin] = level;
    }
}

uint8_t mock_gpio_get_level(uint32_t pin) {
    if (pin < sizeof(s_gpio_levels)) {
        return s_gpio_levels[pin];
    }
    return 1;
}

void mock_gpio_pulse(uint32_t pin, uint32_t duration_us) {
    (void)duration_us;
    mock_gpio_set_level(pin, 0);
}

// ==============================================================================
// I2C 模拟
// ==============================================================================

int mock_i2c_write(uint8_t dev_addr, const uint8_t *data, size_t len) {
    if (!data || len == 0) return -1;

    // 1. TCA9548A 写入: 切换选通通道
    if (dev_addr == MOCK_I2C_ADDR_TCA9548A) {
        s_tca9548a_mask = data[0];
        return 0;
    }

    return 0;
}

int mock_i2c_read(uint8_t dev_addr, uint8_t *data, size_t len) {
    if (!data || len == 0) return -1;

    if (dev_addr == MOCK_I2C_ADDR_TCA9548A) {
        data[0] = s_tca9548a_mask;
        return 0;
    }

    return 0;
}

int mock_i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    (void)dev_addr;
    (void)reg;
    (void)val;
    return 0;
}

int mock_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *out_val) {
    if (!out_val) return -1;

    // 默认成功读取 0
    *out_val = 0;
    return 0;
}

// ==============================================================================
// 模拟传感器注入
// ==============================================================================

void mock_hardware_inject_tof(uint8_t channel, uint16_t distance_mm, uint8_t range_status) {
    if (channel < 8) {
        s_tof_channels[channel].distance_mm = distance_mm;
        s_tof_channels[channel].range_status = range_status;
    }
}

void mock_hardware_inject_ultrasonic(uint32_t trigger_pin, uint16_t distance_mm, bool timeout) {
    (void)trigger_pin;
    s_ultrasonic_state.distance_mm = distance_mm;
    s_ultrasonic_state.is_timeout = timeout;
}

void mock_hardware_inject_imu(
    int16_t ax_mg, int16_t ay_mg, int16_t az_mg,
    int16_t gx_dps, int16_t gy_dps, int16_t gz_dps
) {
    s_imu_state.ax_mg = ax_mg;
    s_imu_state.ay_mg = ay_mg;
    s_imu_state.az_mg = az_mg;
    s_imu_state.gx_dps = gx_dps;
    s_imu_state.gy_dps = gy_dps;
    s_imu_state.gz_dps = gz_dps;
}

void mock_hardware_inject_encoders(int32_t left_ticks, int32_t right_ticks) {
    s_encoder_state.left_ticks = left_ticks;
    s_encoder_state.right_ticks = right_ticks;
}

uint8_t mock_hardware_get_tca9548a_channel_mask(void) {
    return s_tca9548a_mask;
}
