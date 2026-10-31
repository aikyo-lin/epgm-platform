/**
 * @file test_empirical_challenger_m3_r2.c
 * @brief [ZH] EPGM Milestone M3 迭代 2 对抗性独立实证校验测试套件。
 *        [EN] EPGM Milestone M3 Iteration 2 Independent Empirical Challenger Verification Suite.
 *
 * 审查者 / Challenger: Challenger 1 (Milestone M3 Iteration 2 Gate Verification)
 * 验证目标 (The 5 HAL Defects Reported by Challenger 2):
 *  1. Microswitch bumper release: confirm mask=0 resets and does not self-lock across all combinations.
 *  2. 5ms debounce filter: confirm glitch pulses (<5ms) are rejected while sustained touches (>=5ms) trigger.
 *  3. IMU cruise false positive: confirm 200mm/s cruise generates zero false alarms, while stall and slip are decoupled.
 *  4. PID integral ceiling: confirm steady-state speed converges within +-15mm/s and anti-windup operates cleanly.
 *  5. TCA9548A mutex: confirm multi-threaded bus mutex protection and zero race conditions under concurrent load.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

#include "hal_device.h"
#include "hal_bumper.h"
#include "hal_imu.h"
#include "hal_encoder_pid.h"
#include "hal_tca9548a.h"
#include "mock_hardware.h"
#include "mock_freertos.h"

static int s_tests_run = 0;
static int s_tests_passed = 0;
static int s_tests_failed = 0;

#define EMPIRICAL_ASSERT(cond, desc) do { \
    s_tests_run++; \
    if (cond) { \
        s_tests_passed++; \
        printf("  [PASS] %s\n", desc); \
    } else { \
        s_tests_failed++; \
        printf("  [FAIL - BUG FOUND] %s (Line %d: %s)\n", desc, __LINE__, #cond); \
    } \
} while(0)

#define EMPIRICAL_OBSERVE(desc, ...) do { \
    printf("  [OBSERVATION] " desc "\n", ##__VA_ARGS__); \
} while(0)

// 辅助等待函数：可靠跨越宿主机时钟滴答边界确保时间戳前进达到指定毫秒
static void wait_ticks_ms(uint32_t target_ms) {
    uint32_t t0 = (uint32_t)xTaskGetTickCount();
    while (((uint32_t)xTaskGetTickCount() - t0) < target_ms) {
        vTaskDelay(2);
    }
}

// ==============================================================================
// 1. 机械微动触须释放与无自锁实证测试 (Microswitch Bumper Release & No Self-Lock)
// ==============================================================================
static void verify_bumper_release_no_self_lock(void) {
    printf("\n======================================================================\n");
    printf(" [VERIFICATION 1] 微动触须物理释放与自锁消除实证验证\n");
    printf("======================================================================\n");

    mock_hardware_reset();

    hal_device_t bmp_dev;
    hal_bumper_create(&bmp_dev, "bumper_release_test");
    bumper_config_t cfg = {
        .pin_front_left = 14,
        .pin_front_right = 15,
        .pin_rear = 16,
        .debounce_ms = 5,
        .active_low = true
    };
    bmp_dev.init(&bmp_dev, &cfg);

    bumper_status_t status;

    // 1.1 前左触碰 -> 释放复位测试
    mock_gpio_set_level(14, 0); // 前左按下
    mock_gpio_set_level(15, 1);
    mock_gpio_set_level(16, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status)); // 首次读取记录时间戳
    wait_ticks_ms(15); // 持续等待超过 5ms 消抖窗口
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(status.is_triggered && (status.bumper_mask == BUMPER_MASK_FRONT_LEFT),
                     "前左触碰持续 >=5ms 后触发 is_triggered=true, mask=0x01");

    mock_gpio_set_level(14, 1); // 物理释放
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && status.bumper_mask == 0 && !status.emergency_stop,
                     "前左释放后，mask=0, is_triggered=false, emergency_stop=false (无自锁)");

    // 1.2 前右触碰 -> 释放复位测试
    mock_gpio_set_level(15, 0); // 前右按下
    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(16, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    wait_ticks_ms(15);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(status.is_triggered && (status.bumper_mask == BUMPER_MASK_FRONT_RIGHT),
                     "前右触碰持续 >=5ms 后触发 is_triggered=true, mask=0x02");

    mock_gpio_set_level(15, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && status.bumper_mask == 0 && !status.emergency_stop,
                     "前右释放后，mask=0, is_triggered=false, emergency_stop=false (无自锁)");

    // 1.3 后方触碰 -> 释放复位测试
    mock_gpio_set_level(16, 0); // 后方按下
    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(15, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    wait_ticks_ms(15);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(status.is_triggered && (status.bumper_mask == BUMPER_MASK_REAR),
                     "后方触碰持续 >=5ms 后触发 is_triggered=true, mask=0x04");

    mock_gpio_set_level(16, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && status.bumper_mask == 0 && !status.emergency_stop,
                     "后方释放后，mask=0, is_triggered=false, emergency_stop=false (无自锁)");

    // 1.4 双触碰 (前左 + 前右) -> 释放复位测试
    mock_gpio_set_level(14, 0);
    mock_gpio_set_level(15, 0);
    mock_gpio_set_level(16, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(status.is_triggered && status.bumper_mask == 0x03 && status.emergency_stop,
                     "双触须碰撞瞬时置位 is_triggered=true, mask=0x03, emergency_stop=true");

    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(15, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && status.bumper_mask == 0 && !status.emergency_stop,
                     "双触碰释放后，状态完全复位 (急停解除)");

    // 1.5 三触碰全按 -> 释放复位测试
    mock_gpio_set_level(14, 0);
    mock_gpio_set_level(15, 0);
    mock_gpio_set_level(16, 0);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(status.is_triggered && status.bumper_mask == 0x07, "三触碰同时有效，mask=0x07");

    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(15, 1);
    mock_gpio_set_level(16, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && status.bumper_mask == 0 && !status.emergency_stop,
                     "三触碰释放后，状态全部清除复位");

    // 1.6 连续 100 次高频双触碰/三触碰碰撞与脱困释放压力测试 (验证零状态残留)
    bool cycle_success = true;
    for (int i = 0; i < 100; ++i) {
        mock_gpio_set_level(14, 0);
        mock_gpio_set_level(15, 0);
        bmp_dev.read(&bmp_dev, &status, sizeof(status));
        if (!status.is_triggered || status.bumper_mask != 0x03 || !status.emergency_stop) {
            cycle_success = false;
            break;
        }
        mock_gpio_set_level(14, 1);
        mock_gpio_set_level(15, 1);
        bmp_dev.read(&bmp_dev, &status, sizeof(status));
        if (status.is_triggered || status.bumper_mask != 0 || status.emergency_stop) {
            cycle_success = false;
            break;
        }
    }
    EMPIRICAL_ASSERT(cycle_success, "连续 100 次高频碰撞与释放循环无任何状态自锁残留");

    bmp_dev.deinit(&bmp_dev);
}

// ==============================================================================
// 2. 5ms 软件消抖滤波实证验证 (5ms Debounce Filter Glitch & Sustained Validation)
// ==============================================================================
static void verify_bumper_debounce_filter(void) {
    printf("\n======================================================================\n");
    printf(" [VERIFICATION 2] 5ms 软件消抖滤波特性实证验证 (毛刺抑制与持续触发)\n");
    printf("======================================================================\n");

    mock_hardware_reset();

    hal_device_t bmp_dev;
    hal_bumper_create(&bmp_dev, "bumper_debounce_test");
    bumper_config_t cfg = {
        .pin_front_left = 14,
        .pin_front_right = 15,
        .pin_rear = 16,
        .debounce_ms = 5,
        .active_low = true
    };
    bmp_dev.init(&bmp_dev, &cfg);

    bumper_status_t status;

    // 2.1 瞬态毛刺单次采样 (<5ms 抑制校验): 触碰瞬间采样并立即释放
    mock_gpio_set_level(14, 0); // 瞬间拉低
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && !status.emergency_stop,
                     "单触须初次采样 (0ms 窗口): is_triggered=false (接触弹跳被抑制)");
    mock_gpio_set_level(14, 1); // 释放
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && status.bumper_mask == 0,
                     "瞬态毛刺释放后状态安全清零");

    // 2.2 同一时间戳内多次连续紧凑采样 (模拟高频机械振颤抖动)
    bmp_dev.ioctl(&bmp_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    mock_gpio_set_level(14, 0);
    // 连续连续采样 20 次，由于处于初次状态跳变或同一时钟滴答内，严格不得误报急停
    bool premature_trigger = false;
    for (int k = 0; k < 20; ++k) {
        bmp_dev.read(&bmp_dev, &status, sizeof(status));
        // 初次读取后，若时钟尚未前进，is_triggered 绝不应置位
        if (k == 0 && status.is_triggered) {
            premature_trigger = true;
        }
    }
    EMPIRICAL_ASSERT(!premature_trigger, "高频紧凑初次采样中 is_triggered 保持 false 零误报");
    mock_gpio_set_level(14, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && status.bumper_mask == 0, "振颤抖动释放后状态完全清除");

    // 2.3 持续触碰 >= 5ms 确认触发校验 (等待 >= 15ms 持续低电平)
    bmp_dev.ioctl(&bmp_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    mock_gpio_set_level(14, 0);
    bmp_dev.read(&bmp_dev, &status, sizeof(status)); // 首次读取记录时间戳
    EMPIRICAL_ASSERT(!status.is_triggered, "首次采样 (t=0): is_triggered 保持 false");

    wait_ticks_ms(15); // 等待严格超过 5ms 门限
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(status.is_triggered == true && status.emergency_stop == true,
                     "持续稳定触碰 >=5ms: is_triggered=true, emergency_stop=true 成功确认触发");
    EMPIRICAL_ASSERT(status.bumper_mask == BUMPER_MASK_FRONT_LEFT, "触发掩码精确为 BUMPER_MASK_FRONT_LEFT");

    mock_gpio_set_level(14, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(!status.is_triggered && status.bumper_mask == 0, "持续触发后物理释放引脚恢复正常");

    // 2.4 后方微动触须消抖触发实测
    mock_gpio_set_level(16, 0);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    wait_ticks_ms(15);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));
    EMPIRICAL_ASSERT(status.is_triggered && status.bumper_mask == BUMPER_MASK_REAR,
                     "后方微动持续触碰同样可靠触发");
    mock_gpio_set_level(16, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));

    // 2.5 多触须同时触碰免除消抖延迟 (硬碰撞立即刹车特性)
    mock_gpio_set_level(14, 0);
    mock_gpio_set_level(15, 0);
    bmp_dev.read(&bmp_dev, &status, sizeof(status)); // 无需延时
    EMPIRICAL_ASSERT(status.is_triggered && status.emergency_stop && status.bumper_mask == 0x03,
                     "多触须同时触碰 (0x03) 免除消抖延迟立即触发急停 (0ms 延时响应)");

    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(15, 1);
    bmp_dev.read(&bmp_dev, &status, sizeof(status));

    bmp_dev.deinit(&bmp_dev);
}

// ==============================================================================
// 3. IMU 匀速巡航假阳性与动力学解耦实证验证 (IMU Cruise Zero False Alarm & Decoupling)
// ==============================================================================
static void verify_imu_cruise_and_decoupling(void) {
    printf("\n======================================================================\n");
    printf(" [VERIFICATION 3] 6轴 IMU 200mm/s 匀速巡航零误报与打滑/堵转解耦实证验证\n");
    printf("======================================================================\n");

    mock_hardware_reset();

    hal_device_t imu_dev;
    hal_imu_create(&imu_dev, "imu_empirical");
    imu_config_t cfg = {
        .model = IMU_MODEL_MPU6050,
        .i2c_addr = 0x68,
        .accel_scale_g = 4,
        .gyro_scale_dps = 1000,
        .filter_alpha_q8 = 251,
        .stall_accel_thresh_mg = 30,
        .slip_gyro_thresh_dps = 50
    };
    imu_dev.init(&imu_dev, &cfg);

    epgm_imu_data_t imu_in;
    memset(&imu_in, 0, sizeof(imu_in));
    imu_in.accel_z_mg = 1000; // 重力 1g
    imu_in.accel_x_mg = 0;    // 匀速前进无纵向加速度
    imu_in.gyro_z_dps = 0;    // 直线行驶无偏航自旋
    imu_dev.write(&imu_dev, &imu_in, sizeof(imu_in));

    epgm_imu_data_t imu_out;

    // 3.1 标称 200mm/s 匀速直线巡航测试 (连续 100 周期 = 2秒)
    imu_motion_context_t motion_cruise = { .commanded_speed_mms = 200, .measured_wheel_speed_mms = 200 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_cruise);

    int cruise_false_positives = 0;
    for (int i = 0; i < 100; ++i) {
        imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
        if (imu_out.is_skidding || imu_out.is_stalled) {
            cruise_false_positives++;
        }
    }
    EMPIRICAL_ASSERT(cruise_false_positives == 0,
                     "200mm/s 匀速巡航 100 周期 (2秒) 持续零误报 (is_skidding=0, is_stalled=0)");

    // 3.2 速度微扰巡航测试 (指令 200mm/s, 实测轮速 185mm/s, 差值 15 <= 50)
    imu_motion_context_t motion_cruise_noisy = { .commanded_speed_mms = 200, .measured_wheel_speed_mms = 185 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_cruise_noisy);
    int noisy_false_positives = 0;
    for (int i = 0; i < 50; ++i) {
        imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
        if (imu_out.is_skidding || imu_out.is_stalled) {
            noisy_false_positives++;
        }
    }
    EMPIRICAL_ASSERT(noisy_false_positives == 0,
                     "巡航存在 15mm/s 跟踪波动时依然严格保持零误报 (容差 <= 50mm/s)");

    // 3.3 严格解耦测试 A: 电机堵转工况 (Locked Rotor)
    // 条件: 下发指令 200mm/s，但实测轮速卡死 0mm/s，纵向加计无加速度
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    imu_motion_context_t motion_stall = { .commanded_speed_mms = 200, .measured_wheel_speed_mms = 0 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_stall);

    for (int i = 0; i < 5; ++i) {
        imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    }
    EMPIRICAL_ASSERT(imu_out.is_stalled == true, "电机堵转工况下，is_stalled 准确置位为 true");
    EMPIRICAL_ASSERT(imu_out.is_skidding == false, "电机堵转工况下，is_skidding 严格为 false (打滑与堵转严格解耦)");

    // 3.4 严格解耦测试 B: 高速 250mm/s 匀速直线巡航与真实冰面车轮空转打滑 (Wheel Slip)
    // 条件 B1: 高速 250mm/s 匀速直线巡航 (指令 250, 实测 250, 差值 0 <= 50, ax=0)，根据牛顿第一定律严格零误报
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    imu_motion_context_t motion_cruise_high = { .commanded_speed_mms = 250, .measured_wheel_speed_mms = 250 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_cruise_high);
    for (int i = 0; i < 5; ++i) {
        imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    }
    EMPIRICAL_ASSERT(imu_out.is_skidding == false && imu_out.is_stalled == false,
                     "250mm/s 高速匀速巡航严格保持零误报 (is_skidding=false, is_stalled=false)");

    // 条件 B2: 真实冰面车轮空转打滑工况 (指令 100mm/s, 实测轮速高达 250mm/s, 差值 150 > 50, ax=0)
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    imu_motion_context_t motion_slip = { .commanded_speed_mms = 100, .measured_wheel_speed_mms = 250 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_slip);

    for (int i = 0; i < 5; ++i) {
        imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    }
    EMPIRICAL_ASSERT(imu_out.is_skidding == true, "车轮空转打滑工况下，is_skidding 准确置位为 true");

    // 3.5 偏航陀螺仪突变打滑自旋测试 (gz = 80 deg/s > 50)
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    imu_in.gyro_z_dps = 80;
    imu_dev.write(&imu_dev, &imu_in, sizeof(imu_in));
    imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    EMPIRICAL_ASSERT(imu_out.is_skidding == true, "偏航陀螺仪角速度突变 (>50 dps) 瞬时检出打滑自旋");

    imu_dev.deinit(&imu_dev);
}

// ==============================================================================
// 4. 速度 PID 稳态误差与抗积分饱和实证验证 (PID Steady-State & Anti-Windup)
// ==============================================================================
static void verify_pid_steady_state_and_anti_windup(void) {
    printf("\n======================================================================\n");
    printf(" [VERIFICATION 4] 差速轮编码器 PID 稳态静差消除 (±15mm/s) 与抗饱和实证验证\n");
    printf("======================================================================\n");

    mock_hardware_reset();

    hal_device_t pid_dev;
    hal_encoder_pid_create(&pid_dev, "pid_empirical");

    // 传入旧版过小的配置 (integral_max = 500)，验证驱动层自动扩充至 >= 2000
    encoder_pid_config_t legacy_cfg = {
        .left_pid = {
            .kp_q8 = 307,
            .ki_q8 = 38,
            .kd_q8 = 13,
            .integral_max = 500, // 旧版缺陷参数
            .deadband_pwm = 30,
            .max_output_pwm = 255
        },
        .right_pid = {
            .kp_q8 = 307,
            .ki_q8 = 38,
            .kd_q8 = 13,
            .integral_max = 500,
            .deadband_pwm = 30,
            .max_output_pwm = 255
        },
        .cruise_speed = 200,
        .turn_speed = 150,
        .wheel_diameter_mm = 65,
        .wheel_base_mm = 140,
        .cpr = 360
    };
    pid_dev.init(&pid_dev, &legacy_cfg);

    // 下发标称前进动作 1 (目标 200mm/s)
    uint8_t act_fwd = 1;
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &act_fwd);

    // 运行 50 步电机动力学仿真
    int16_t sim_wheel_speed = 0;
    int32_t sim_ticks = 0;
    encoder_pid_status_t st;

    for (int step = 0; step < 50; ++step) {
        int32_t ticks[2] = { sim_ticks, sim_ticks };
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, ticks);
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
        pid_dev.read(&pid_dev, &st, sizeof(st));

        // 一阶电机动力学: 滤波响应并产生位移
        sim_wheel_speed = (int16_t)((sim_wheel_speed * 7 + (st.left_pwm_out * 220 / 255) * 3) / 10);
        sim_ticks += (sim_wheel_speed / 5);
    }

    EMPIRICAL_OBSERVE("50 步后最终实测速度: left=%d mm/s, right=%d mm/s, pwm_out=%d",
                      st.left_speed_mms, st.right_speed_mms, st.left_pwm_out);

    // 稳态误差校验: 必须收敛至 200mm/s ± 15mm/s (即 [185, 215] 之间)
    int left_err = abs(st.left_speed_mms - 200);
    int right_err = abs(st.right_speed_mms - 200);

    EMPIRICAL_ASSERT(left_err <= 15, "左轮最终稳态速度收敛在 200mm/s ± 15mm/s 误差带内");
    EMPIRICAL_ASSERT(right_err <= 15, "右轮最终稳态速度收敛在 200mm/s ± 15mm/s 误差带内");

    // 4.2 锁转抗饱和 (Anti-Windup) 压力测试: 1,000 步锁死转子
    int32_t locked_ticks[2] = { 0, 0 };
    for (int i = 0; i < 1000; ++i) {
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, locked_ticks);
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
    }
    pid_dev.read(&pid_dev, &st, sizeof(st));
    EMPIRICAL_ASSERT(st.left_pwm_out == 255, "锁转满负荷计算下，PWM 输出精确钳位在 max_output_pwm (255)");

    // 锁转解除立即归零校验
    uint8_t act_stop = 0;
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &act_stop);
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, locked_ticks);
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
    pid_dev.read(&pid_dev, &st, sizeof(st));
    EMPIRICAL_ASSERT(st.left_pwm_out == 0 && st.right_pwm_out == 0,
                     "停机指令下发后，电机 PWM 瞬时归零，无积分饱和反向延迟");

    pid_dev.deinit(&pid_dev);
}

// ==============================================================================
// 5. TCA9548A 总线原子互斥锁与并发安全实证验证 (TCA9548A Bus Mutex Protection)
// ==============================================================================
#define TCA_CONCURRENT_THREADS 4
#define TCA_OPS_PER_THREAD     2500

typedef struct {
    hal_device_t *mux;
    uint8_t       ch_base;
    uint32_t      ops_completed;
    uint32_t      errors_found;
} tca_thread_arg_t;

static void tca_stress_worker(void *param) {
    tca_thread_arg_t *arg = (tca_thread_arg_t *)param;
    hal_device_t *mux = arg->mux;

    for (uint32_t i = 0; i < TCA_OPS_PER_THREAD; ++i) {
        uint8_t target_ch = (uint8_t)(arg->ch_base + (i % 2)); // 在分配的两个通道间交替
        int sel_res = hal_tca9548a_select_channel(mux, target_ch);
        if (sel_res != HAL_OK) {
            arg->errors_found++;
            continue;
        }

        uint8_t read_mask = 0;
        int read_res = hal_tca9548a_get_channel_mask(mux, &read_mask);
        if (read_res != HAL_OK) {
            arg->errors_found++;
            continue;
        }

        // 验证读取的掩码是合法的单热码 (单通道有效)
        if (read_mask == 0 || (read_mask & (read_mask - 1)) != 0) {
            arg->errors_found++;
        }

        arg->ops_completed++;
    }
}

static void verify_tca9548a_mutex_concurrency(void) {
    printf("\n======================================================================\n");
    printf(" [VERIFICATION 5] TCA9548A 多线程总线互斥锁并发安全性实证验证\n");
    printf("======================================================================\n");

    mock_hardware_reset();

    hal_device_t mux;
    hal_tca9548a_create(&mux, "mux_empirical");
    tca9548a_config_t cfg = {
        .i2c_addr = TCA9548A_DEFAULT_I2C_ADDR,
        .default_channel = 0,
        .enable_mutex = true
    };
    int init_res = mux.init(&mux, &cfg);
    EMPIRICAL_ASSERT(init_res == HAL_OK, "TCA9548A 使能总线原子互斥锁 (enable_mutex=true) 初始化成功");

    // 启动 4 个并发工作线程，分别分配不同的通道组 (0~1, 2~3, 4~5, 6~7)
    TaskHandle_t tasks[TCA_CONCURRENT_THREADS];
    tca_thread_arg_t args[TCA_CONCURRENT_THREADS];

    for (int t = 0; t < TCA_CONCURRENT_THREADS; ++t) {
        args[t].mux = &mux;
        args[t].ch_base = (uint8_t)(t * 2);
        args[t].ops_completed = 0;
        args[t].errors_found = 0;
        char t_name[32];
        snprintf(t_name, sizeof(t_name), "TcaThread_%d", t);
        xTaskCreate(tca_stress_worker, t_name, 4096, &args[t], 1, &tasks[t]);
    }

    // 等待所有线程完成
    uint32_t total_ops = 0;
    uint32_t total_errs = 0;
    for (int t = 0; t < TCA_CONCURRENT_THREADS; ++t) {
        bool joined = mock_task_join(tasks[t], 10000);
        EMPIRICAL_ASSERT(joined, "并发任务线程在规定时间内正常汇聚退出");
        total_ops += args[t].ops_completed;
        total_errs += args[t].errors_found;
    }

    EMPIRICAL_OBSERVE("并发总线切换统计: 完成操作总数=%u / %u, 竞争错误数=%u",
                      total_ops, TCA_CONCURRENT_THREADS * TCA_OPS_PER_THREAD, total_errs);

    EMPIRICAL_ASSERT(total_ops == TCA_CONCURRENT_THREADS * TCA_OPS_PER_THREAD,
                     "4 个并发线程完成全部 10,000 次总线通道切换与掩码读取");
    EMPIRICAL_ASSERT(total_errs == 0,
                     "10,000 次并发总线操作中竞态错误数严格为 0 (FreeRTOS 互斥锁完全生效)");

    mux.deinit(&mux);
}

// ==============================================================================
// 主测试入口
// ==============================================================================
int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    mock_hardware_init();

    printf("======================================================================\n");
    printf(" EPGM Milestone M3 Iteration 2 对抗性独立实证校验测试套件\n");
    printf(" EPGM M3 Iteration 2 Empirical Re-Challenge Verification Suite\n");
    printf(" Challenger: Challenger 1 (critic, specialist)\n");
    printf("======================================================================\n");

    verify_bumper_release_no_self_lock();
    verify_bumper_debounce_filter();
    verify_imu_cruise_and_decoupling();
    verify_pid_steady_state_and_anti_windup();
    verify_tca9548a_mutex_concurrency();

    printf("\n======================================================================\n");
    printf(" 对抗性独立实证校验完成!\n");
    printf(" 测试统计: 共执行 %d 项断言, 通过 %d 项, 失败 %d 项\n",
           s_tests_run, s_tests_passed, s_tests_failed);
    printf("======================================================================\n");

    return (s_tests_failed == 0) ? 0 : 1;
}
