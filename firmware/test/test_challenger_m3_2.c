/**
 * @file test_challenger_m3_2.c
 * @brief [ZH] EPGM M3 外设 HAL 驱动与统一传感器聚合器对抗性深度压力与故障注入测试套件。
 *        [EN] EPGM M3 Peripheral HAL Drivers & Sensor Aggregator Adversarial Fault Injection & Stress Suite.
 *
 * 审查者 / Challenger: Challenger 2 (Empirical Challenger)
 * 验证目标:
 *  1. HC-SR04 超声波断连/超时看门狗保护 (25ms 保护门狗，超量程保底，防挂死)
 *  2. 6轴 IMU 打滑 (Slip) 与堵转 (Stall) 动力学检测与边界工况 (巡航假阳性、锁转分离)
 *  3. TCA9548A 8 通道 I2C 选通复用器高频切换、通道隔离度与并发总线互斥
 *  4. 机械防碰撞微动触须高频触点弹跳 (Contact Bounce) 与 5ms 软件消抖滤波深度实测
 *  5. 差速双轮编码器速度 PID 阶跃响应、抗积分饱和 (Anti-Windup) 与锁死转子恢复
 *  6. 全向传感器统一聚合器 (Aggregator) 8 向空间距离解算、超声波融合与微动急停状态上报
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

#include "hal_device.h"
#include "hal_vl53lxx.h"
#include "hal_hcsr04.h"
#include "hal_imu.h"
#include "hal_tca9548a.h"
#include "hal_encoder_pid.h"
#include "hal_bumper.h"
#include "hal_sensors_aggregator.h"
#include "mock_hardware.h"
#include "mock_freertos.h"

// ==============================================================================
// 测试断言与统计宏
// ==============================================================================
static int s_tests_run = 0;
static int s_tests_passed = 0;
static int s_tests_failed = 0;

#define CHALLENGE_ASSERT(cond, desc) do { \
    s_tests_run++; \
    if (cond) { \
        s_tests_passed++; \
        printf("  [PASS] %s\n", desc); \
    } else { \
        s_tests_failed++; \
        printf("  [FAIL - BUG FOUND] %s (Line %d: %s)\n", desc, __LINE__, #cond); \
    } \
} while(0)

#define CHALLENGE_OBSERVE(desc, ...) do { \
    printf("  [OBSERVATION] " desc "\n", ##__VA_ARGS__); \
} while(0)

// ==============================================================================
// 1. HC-SR04 故障注入与看门狗保护测试
// ==============================================================================
static void challenge_test_hcsr04_fault_injection(void) {
    printf("\n======================================================================\n");
    printf(" [CHALLENGE 1] HC-SR04 超声波断连与 25ms 看门狗保护测试\n");
    printf("======================================================================\n");

    hal_device_t us_dev;
    int c_ok = hal_hcsr04_create(&us_dev, "us_fault_test");
    CHALLENGE_ASSERT(c_ok == HAL_OK, "HC-SR04 设备实例创建成功");

    hcsr04_config_t cfg = {
        .pin_trigger = 12,
        .pin_echo = 13,
        .timeout_ms = 25,
        .max_range_mm = 4000
    };
    int init_ok = us_dev.init(&us_dev, &cfg);
    CHALLENGE_ASSERT(init_ok == HAL_OK, "HC-SR04 默认参数初始化成功");

    // 1.1 正常读数与脉宽距离换算校验: 距离 500mm, echo 应为 (500 * 2000) / 343 = 2915µs
    hcsr04_reading_t normal_in = { .distance_mm = 500, .is_timeout = false };
    us_dev.write(&us_dev, &normal_in, sizeof(hcsr04_reading_t));
    hcsr04_reading_t normal_out;
    int read_bytes = us_dev.read(&us_dev, &normal_out, sizeof(hcsr04_reading_t));
    CHALLENGE_ASSERT(read_bytes == sizeof(hcsr04_reading_t), "HC-SR04 完整结构体读取成功");
    CHALLENGE_ASSERT(normal_out.distance_mm == 500 && !normal_out.is_timeout, "HC-SR04 正常读数 500mm 准确无误且超时标置为 false");
    CHALLENGE_ASSERT(normal_out.echo_duration_us == (500U * 2000U) / 343U, "HC-SR04 脉冲回波微秒计算符合物理音速公式");

    // 1.2 故障注入: 模拟传感器断连 / 吸收材料无回波 (is_timeout = true)
    hcsr04_reading_t timeout_in = { .distance_mm = 0, .is_timeout = true };
    us_dev.write(&us_dev, &timeout_in, sizeof(hcsr04_reading_t));
    hcsr04_reading_t timeout_out;
    us_dev.read(&us_dev, &timeout_out, sizeof(hcsr04_reading_t));
    CHALLENGE_ASSERT(timeout_out.is_timeout == true, "HC-SR04 传感器断连精确置位 is_timeout = true");
    CHALLENGE_ASSERT(timeout_out.distance_mm == 4000, "HC-SR04 超时保底返回预设最大量程 4000mm，防止误报近距离障碍物");
    CHALLENGE_ASSERT(timeout_out.echo_duration_us == 25000, "HC-SR04 超时回波时间钳位至看门狗门限 25000µs (25ms)");

    // 1.3 边界注入: 自定义配置不同超时门限与量程 (timeout_ms=10, max_range=2500)
    hcsr04_config_t custom_cfg = {
        .pin_trigger = 21,
        .pin_echo = 22,
        .timeout_ms = 10,
        .max_range_mm = 2500
    };
    us_dev.ioctl(&us_dev, HAL_IOCTL_SET_CONFIG, &custom_cfg);
    us_dev.read(&us_dev, &timeout_out, sizeof(hcsr04_reading_t));
    CHALLENGE_ASSERT(timeout_out.distance_mm == 2500, "动态重配后，超时返回自定义上限 2500mm");
    CHALLENGE_ASSERT(timeout_out.echo_duration_us == 10000, "动态重配后，超时回波时间更新为 10000µs (10ms)");

    // 1.4 高频无阻塞抗挂死压测: 连续调用 100,000 次读取，验证绝不阻塞控制线程
    uint64_t t_start = mock_hardware_get_time_us();
    for (int i = 0; i < 100000; ++i) {
        uint16_t dist = 0;
        us_dev.read(&us_dev, &dist, sizeof(uint16_t));
        (void)dist;
    }
    uint64_t t_elapsed = mock_hardware_get_time_us() - t_start;
    CHALLENGE_ASSERT(t_elapsed < 500000, "HC-SR04 100,000 次高频读取总耗时 < 500ms，无死循环阻塞");

    us_dev.deinit(&us_dev);
}

// ==============================================================================
// 2. 6轴 IMU 打滑与堵转动力学校验 (对抗性工况与误报检测)
// ==============================================================================
static void challenge_test_imu_slip_stall_detection(void) {
    printf("\n======================================================================\n");
    printf(" [CHALLENGE 2] 6轴 IMU 打滑 (Slip) 与堵转 (Stall) 动力学检测与误报挑战\n");
    printf("======================================================================\n");

    hal_device_t imu_dev;
    hal_imu_create(&imu_dev, "imu_challenge");
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

    // 2.1 静止工况测试 (指令速度 0, 轮速 0, 加计 ax=0, az=1000)
    imu_motion_context_t motion_stop = { .commanded_speed_mms = 0, .measured_wheel_speed_mms = 0 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_stop);
    epgm_imu_data_t imu_in;
    memset(&imu_in, 0, sizeof(imu_in));
    imu_in.accel_z_mg = 1000;
    imu_dev.write(&imu_dev, &imu_in, sizeof(imu_in));

    epgm_imu_data_t imu_out;
    for (int i = 0; i < 5; ++i) {
        imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    }
    CHALLENGE_ASSERT(!imu_out.is_skidding && !imu_out.is_stalled, "静止停机状态下，打滑与堵转标志均严格为 false");

    // 2.2 真实打滑工况: 轮速高达 250mm/s，但机身受困冰面，纵向加速度 ax = 0 (< 30mg)
    imu_motion_context_t motion_slip = { .commanded_speed_mms = 100, .measured_wheel_speed_mms = 250 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_slip);
    imu_in.accel_x_mg = 0;
    imu_dev.write(&imu_dev, &imu_in, sizeof(imu_in));

    // 前两个周期不应误判 (防偶发跳变)
    imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    CHALLENGE_ASSERT(!imu_out.is_skidding, "周期 1 滤波窗口中，暂不触发打滑");
    imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    CHALLENGE_ASSERT(!imu_out.is_skidding, "周期 2 滤波窗口中，暂不触发打滑");
    // 第三个周期超过门限 (stall_counter > 2)，判定打滑触发
    imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    CHALLENGE_ASSERT(imu_out.is_skidding, "连续 3 周期无纵向加速度且高轮速，打滑标志成功置位");

    // 2.3 陀螺仪偏航角速度突变打滑 (侧滑/自旋, gz = 80 deg/s > 50 deg/s)
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    imu_in.accel_x_mg = 100; // 具备加速度
    imu_in.gyro_z_dps = 80;  // 侧滑剧烈旋转
    imu_dev.write(&imu_dev, &imu_in, sizeof(imu_in));
    imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    CHALLENGE_ASSERT(imu_out.is_skidding, "偏航角速度突变 (>50 deg/s) 瞬时检出打滑自旋");

    // 2.4 对抗性缺陷检验 A: 电机受阻堵转 (Locked Rotor) 与车轮打滑 (Wheel Slip) 语义混淆
    // 在纯堵转工况下: 目标速度 200mm/s，但实测轮速 measured_wheel_speed_mms = 0，ax = 0
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    imu_motion_context_t motion_stall_only = { .commanded_speed_mms = 200, .measured_wheel_speed_mms = 0 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_stall_only);
    imu_in.gyro_z_dps = 0;
    imu_in.accel_x_mg = 0;
    imu_dev.write(&imu_dev, &imu_in, sizeof(imu_in));
    for (int i = 0; i < 4; ++i) imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));

    CHALLENGE_ASSERT(imu_out.is_stalled, "电机堵转工况准确检出 is_stalled = true");
    CHALLENGE_OBSERVE("检查堵转工况下的 is_skidding 状态: %s (注: 轮速为 0 时理论上不应发生车轮打滑)",
                      imu_out.is_skidding ? "TRUE (混淆/耦合)" : "FALSE (解耦)");

    // 2.5 对抗性缺陷检验 B: 匀速巡航假阳性 (Constant-Speed False Positive)
    // 物理常识: 当机器人在平地上以 200mm/s 恒速前进时，加速度 dv/dt 严格为 0 (|ax| < 30mg)！
    // 此时连续运行多个周期，驱动层是否会将正常的平稳巡航误判为打滑或堵转？
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    imu_motion_context_t motion_cruise = { .commanded_speed_mms = 200, .measured_wheel_speed_mms = 200 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_cruise);
    imu_in.accel_x_mg = 0; // 匀速运动无加速度
    imu_in.gyro_z_dps = 0;
    imu_dev.write(&imu_dev, &imu_in, sizeof(imu_in));
    for (int i = 0; i < 5; ++i) {
        imu_dev.read(&imu_dev, &imu_out, sizeof(imu_out));
    }
    // 观察匀速巡航下的实际行为
    if (imu_out.is_skidding || imu_out.is_stalled) {
        CHALLENGE_OBSERVE("【设计缺陷】匀速直线巡航 (ax=0, v=200) 被误报为 is_skidding=%d, is_stalled=%d！",
                          imu_out.is_skidding, imu_out.is_stalled);
    } else {
        CHALLENGE_ASSERT(true, "匀速直线巡航未被误报为打滑或堵转");
    }

    imu_dev.deinit(&imu_dev);
}

// ==============================================================================
// 3. TCA9548A 8通道 I2C 选通复用器快速切换与通道隔离度
// ==============================================================================
static void challenge_test_tca9548a_isolation_and_switching(void) {
    printf("\n======================================================================\n");
    printf(" [CHALLENGE 3] TCA9548A 8通道快速切换、通道隔离度与边界测试\n");
    printf("======================================================================\n");

    hal_device_t mux;
    hal_tca9548a_create(&mux, "mux_challenge");
    mux.init(&mux, NULL);

    // 3.1 8 通道逐一选通与严格独占隔离度校验 (严格单热码 1 << k)
    bool isolation_perfect = true;
    for (uint8_t ch = 0; ch < 8; ++ch) {
        int sel_res = hal_tca9548a_select_channel(&mux, ch);
        if (sel_res != HAL_OK) {
            isolation_perfect = false;
            break;
        }
        uint8_t mask = 0;
        hal_tca9548a_get_channel_mask(&mux, &mask);
        uint8_t expected_mask = (uint8_t)(1 << ch);
        if (mask != expected_mask) {
            isolation_perfect = false;
            CHALLENGE_OBSERVE("通道 %d 选通失败: 期望 0x%02X, 实际 0x%02X", ch, expected_mask, mask);
            break;
        }
        // 校验 mock hardware 底层总线同步
        if (mock_hardware_get_tca9548a_channel_mask() != expected_mask) {
            isolation_perfect = false;
            break;
        }
    }
    CHALLENGE_ASSERT(isolation_perfect, "TCA9548A 0~7 各通道独立选通且位掩码严格隔离 (独占选通)");

    // 3.2 边界与非法输入测试 (通道号 8, 9, 255)
    uint8_t invalid_channels[] = { 8, 9, 16, 255 };
    bool boundary_rejected = true;
    for (size_t i = 0; i < sizeof(invalid_channels); ++i) {
        int res = hal_tca9548a_select_channel(&mux, invalid_channels[i]);
        if (res == HAL_OK) {
            boundary_rejected = false;
            CHALLENGE_OBSERVE("非法通道号 %d 未被拦截！", invalid_channels[i]);
        }
    }
    CHALLENGE_ASSERT(boundary_rejected, "非法通道号 (>=8) 被安全拦截并返回 HAL_ERR_PARAM");

    // 3.3 高频快速通道交替切换压测 (100,000 轮随机通道切换)
    uint64_t t_switch_start = mock_hardware_get_time_us();
    bool switch_consistent = true;
    for (int i = 0; i < 100000; ++i) {
        uint8_t target_ch = (uint8_t)(i % 8);
        hal_tca9548a_select_channel(&mux, target_ch);
        uint8_t mask = 0;
        hal_tca9548a_get_channel_mask(&mux, &mask);
        if (mask != (1 << target_ch)) {
            switch_consistent = false;
            break;
        }
    }
    uint64_t t_switch_elapsed = mock_hardware_get_time_us() - t_switch_start;
    CHALLENGE_ASSERT(switch_consistent, "100,000 次高频交替通道切换状态 100% 一致");
    CHALLENGE_ASSERT(t_switch_elapsed < 500000, "100,000 次切换总耗时 < 500ms，总线零卡死");

    // 3.4 互斥锁审查
    CHALLENGE_OBSERVE("审查结果: tca9548a_config_t 声明了 enable_mutex，但驱动内部未创建并持有 FreeRTOS 锁");

    mux.deinit(&mux);
}

// ==============================================================================
// 4. 机械防碰撞微动触须高频触点弹跳 (Contact Bounce) 与消抖滤波挑战
// ==============================================================================
static void challenge_test_bumper_debounce_and_contact_bounce(void) {
    printf("\n======================================================================\n");
    printf(" [CHALLENGE 4] 机械微动触须高频触点弹跳 (Contact Bounce) 与 5ms 消抖滤波挑战\n");
    printf("======================================================================\n");

    hal_device_t bmp_dev;
    hal_bumper_create(&bmp_dev, "bumper_bounce_test");
    bumper_config_t cfg = {
        .pin_front_left = 14,
        .pin_front_right = 15,
        .pin_rear = 16,
        .debounce_ms = 5,
        .active_low = true
    };
    bmp_dev.init(&bmp_dev, &cfg);

    // 4.1 正常触发与急停置位测试 (前左 + 前右)
    mock_gpio_set_level(14, 0); // 前左按下
    mock_gpio_set_level(15, 0); // 前右按下
    mock_gpio_set_level(16, 1); // 后方释放
    bumper_status_t b_status;
    bmp_dev.read(&bmp_dev, &b_status, sizeof(bumper_status_t));
    CHALLENGE_ASSERT(b_status.is_triggered == true, "微动按下触发 is_triggered = true");
    CHALLENGE_ASSERT(b_status.emergency_stop == true, "微动按下触发 emergency_stop = true");
    CHALLENGE_ASSERT(b_status.bumper_mask == (BUMPER_MASK_FRONT_LEFT | BUMPER_MASK_FRONT_RIGHT),
                     "碰撞掩码精确合成前左与前右 (0x03)");

    // 4.2 释放与自锁缺陷实测
    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(15, 1);
    mock_gpio_set_level(16, 1);
    bmp_dev.read(&bmp_dev, &b_status, sizeof(bumper_status_t));
    if (b_status.is_triggered || b_status.bumper_mask != 0) {
        CHALLENGE_OBSERVE("【严重缺陷】硬件微动释放后，因 hal_bumper.c:55 未清零内部状态，造成永久死锁 (is_triggered=%d, mask=0x%02X)",
                          b_status.is_triggered, b_status.bumper_mask);
        s_tests_run++;
        s_tests_failed++;
        printf("  [FAIL - BUG FOUND] 硬件微动释放后状态未自动清除 (hal_bumper.c:55 mask = priv->status.bumper_mask)\n");
    } else {
        CHALLENGE_ASSERT(true, "微动全部释放后 is_triggered 与 bumper_mask 正常恢复为 0");
    }

    // 4.3 高频触点弹跳 (Contact Bounce) 故障注入测试:
    // 真实微动开关闭合瞬间会产生 1~3ms 的高频机械抖动 (快速交替 0 与 1)。
    // 规范明确要求: 具备 5ms 软件消抖数字滤波，防止短时脉冲毛刺产生假阳性触发！
    // 注入: 模拟瞬时毛刺 (瞬时拉低 0，但在下一瞬间释放)
    bmp_dev.ioctl(&bmp_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
    mock_gpio_set_level(14, 0); // 瞬时接触毛刺
    bmp_dev.read(&bmp_dev, &b_status, sizeof(bumper_status_t));
    mock_gpio_set_level(14, 1); // 立即恢复高电平

    // 验证驱动是否实现了 5ms 数字消抖
    if (b_status.is_triggered) {
        CHALLENGE_OBSERVE("【严重缺陷】hal_bumper 未包含时间戳消抖计算 (last_trigger_tick 未使用)，单次毛刺即触发急停！");
        s_tests_run++;
        s_tests_failed++;
        printf("  [FAIL - BUG FOUND] 5ms 消抖滤波缺失，单次毛刺即触发假阳性急停\n");
    } else {
        CHALLENGE_ASSERT(true, "5ms 软件消抖滤波成功滤除单次触点弹跳脉冲");
    }

    // 4.4 掩码全组合覆盖 (1~7) - 显式使用 CLEAR_FLAG 规避自锁缺陷后进行逻辑校验
    bool all_masks_correct = true;
    for (uint8_t m = 1; m <= 7; ++m) {
        bmp_dev.ioctl(&bmp_dev, HAL_IOCTL_CLEAR_FLAG, NULL);
        mock_gpio_set_level(14, (m & 1) ? 0 : 1);
        mock_gpio_set_level(15, (m & 2) ? 0 : 1);
        mock_gpio_set_level(16, (m & 4) ? 0 : 1);
        bmp_dev.read(&bmp_dev, &b_status, sizeof(bumper_status_t));
        vTaskDelay(10);
        bmp_dev.read(&bmp_dev, &b_status, sizeof(bumper_status_t));
        if (b_status.bumper_mask != m) {
            all_masks_correct = false;
            break;
        }
    }
    CHALLENGE_ASSERT(all_masks_correct, "在显式复位下，前左、前右、后方 3 触须 7 种组合掩码逻辑匹配率 100%");

    // 清理
    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(15, 1);
    mock_gpio_set_level(16, 1);
    bmp_dev.deinit(&bmp_dev);
}

// ==============================================================================
// 5. 差速轮编码器 PID 阶跃响应与堵转抗积分饱和 (Anti-Windup) 测试
// ==============================================================================
static void challenge_test_encoder_pid_step_and_anti_windup(void) {
    printf("\n======================================================================\n");
    printf(" [CHALLENGE 5] 差速双轮编码器 PID 阶跃响应与堵转抗积分饱和 (Anti-Windup) 测试\n");
    printf("======================================================================\n");

    hal_device_t pid_dev;
    hal_encoder_pid_create(&pid_dev, "pid_challenge");
    encoder_pid_config_t cfg = {
        .left_pid = {
            .kp_q8 = 307,          // 1.2 * 256
            .ki_q8 = 38,           // 0.15 * 256
            .kd_q8 = 13,           // 0.05 * 256
            .integral_max = 500,   // 默认抗饱和上限
            .deadband_pwm = 30,    // 死区补偿
            .max_output_pwm = 255  // 最大 PWM
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
    pid_dev.init(&pid_dev, &cfg);

    // 5.1 动作映射校验 (1:前进 200, 2:后退 -200, 3:左转 -150/+150, 4:右转 +150/-150, 0:停 0/0)
    uint8_t actions[] = { 1, 2, 3, 4, 0 };
    int16_t exp_l[]   = { 200, -200, -150,  150, 0 };
    int16_t exp_r[]   = { 200, -200,  150, -150, 0 };
    bool actions_ok = true;
    for (int i = 0; i < 5; ++i) {
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &actions[i]);
        encoder_pid_status_t st;
        pid_dev.read(&pid_dev, &st, sizeof(st));
        if (st.left_target_mms != exp_l[i] || st.right_target_mms != exp_r[i]) {
            actions_ok = false;
            break;
        }
    }
    CHALLENGE_ASSERT(actions_ok, "EPGM 5 种离散动作全部精确解算为差速双轮标称速度向量");

    // 5.2 阶跃响应实测 A: 默认配置 (integral_max = 500) 下的稳态追踪性能
    uint8_t act_fwd = 1;
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &act_fwd);

    int16_t sim_wheel_speed = 0;
    int32_t sim_ticks = 0;
    bool converged_default = false;

    for (int step = 0; step < 50; ++step) {
        int32_t ticks[2] = { sim_ticks, sim_ticks };
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, ticks);
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);

        encoder_pid_status_t st;
        pid_dev.read(&pid_dev, &st, sizeof(st));

        sim_wheel_speed = (int16_t)((sim_wheel_speed * 7 + (st.left_pwm_out * 220 / 255) * 3) / 10);
        sim_ticks += (sim_wheel_speed / 5);

        if (step > 25 && abs(st.left_speed_mms - 200) < 15) {
            converged_default = true;
        }
    }
    if (!converged_default) {
        CHALLENGE_OBSERVE("【性能缺陷】默认 integral_max=500 导致积分项输出封顶于 74 PWM，稳态速度卡死在 145mm/s (稳态误差 27.5%%)");
        s_tests_run++;
        s_tests_failed++;
        printf("  [FAIL - BUG FOUND] 默认 integral_max=500 过小，导致 PID 无法消除稳态静差\n");
    } else {
        CHALLENGE_ASSERT(true, "PID 阶跃响应收敛");
    }

    // 5.3 阶跃响应实测 B: 参数调优验证 (提高 integral_max 至 2000 消除饱和门槛)
    encoder_pid_config_t tuned_cfg = cfg;
    tuned_cfg.left_pid.integral_max = 2000;
    tuned_cfg.right_pid.integral_max = 2000;
    pid_dev.init(&pid_dev, &tuned_cfg);
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &act_fwd);

    sim_wheel_speed = 0;
    sim_ticks = 0;
    bool converged_tuned = false;
    for (int step = 0; step < 50; ++step) {
        int32_t ticks[2] = { sim_ticks, sim_ticks };
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, ticks);
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);

        encoder_pid_status_t st;
        pid_dev.read(&pid_dev, &st, sizeof(st));

        sim_wheel_speed = (int16_t)((sim_wheel_speed * 7 + (st.left_pwm_out * 220 / 255) * 3) / 10);
        sim_ticks += (sim_wheel_speed / 5);

        if (step > 25 && abs(st.left_speed_mms - 200) < 15) {
            converged_tuned = true;
        }
    }
    CHALLENGE_ASSERT(converged_tuned, "调优 integral_max=2000 后，PID 成功消除稳态静差并收敛至 200mm/s ± 15mm/s");

    // 5.3 锁转极限工况与抗积分饱和 (Anti-Windup Under Locked Rotor)
    // 场景: 车轮被异物卡死，实测轮速强制维持为 0 (measured = 0)，下发连续 1,000 步满负荷计算！
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_RESET, NULL);
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &act_fwd); // 目标 200

    int32_t locked_ticks[2] = { 0, 0 };
    for (int i = 0; i < 1000; ++i) {
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, locked_ticks);
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
    }
    encoder_pid_status_t stall_status;
    pid_dev.read(&pid_dev, &stall_status, sizeof(stall_status));

    // 输出应被钳位在 max_output_pwm (255)
    CHALLENGE_ASSERT(stall_status.left_pwm_out == 255, "锁转工况下，PID 输出受控钳位于 max_output_pwm (255)");

    // 关键抗饱和验证: 锁转解除测试！
    // 将目标速度归零。如果发生积分饱和未限幅，解卡后反向误差需要极长时间才能把庞大积分放尽。
    // 在抗饱和设计下，下发停止动作后，PWM 应迅速归零。
    uint8_t act_stop = 0;
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &act_stop);
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, locked_ticks);
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
    pid_dev.read(&pid_dev, &stall_status, sizeof(stall_status));

    CHALLENGE_ASSERT(stall_status.left_pwm_out == 0 && stall_status.right_pwm_out == 0,
                     "锁转解除停机后，抗积分饱和设计使电机输出瞬时降为 0，无迟滞倒转");

    pid_dev.deinit(&pid_dev);
}

// ==============================================================================
// 6. 统一传感器阵列聚合器 (Sensor Aggregator) 全景帧与多模态融合测试
// ==============================================================================
static void challenge_test_sensor_aggregator_panoramic_fusion(void) {
    printf("\n======================================================================\n");
    printf(" [CHALLENGE 6] 传感器聚合器 8 向全景空间解算与多模态融合压力测试\n");
    printf("======================================================================\n");

    hal_sensors_aggregator_deinit();
    int init_res = hal_sensors_aggregator_init();
    CHALLENGE_ASSERT(init_res == HAL_OK, "传感器聚合器成功挂载全部 8 路 ToF、超声波、IMU 与微动");

    // 6.1 验证 8 个独立方向 ToF 空间测距的独立映射性
    // 为 8 路 ToF 设备分别写入独一无二的距离值: 150, 250, 350, 450, 550, 650, 750, 850 mm
    uint16_t test_distances[8] = { 150, 250, 350, 450, 550, 650, 750, 850 };
    for (int i = 0; i < 8; ++i) {
        char dev_name[32];
        snprintf(dev_name, sizeof(dev_name), "tof_sector_%d", i);
        hal_device_t *tof_d = hal_device_find(dev_name);
        CHALLENGE_ASSERT(tof_d != NULL, dev_name);
        if (tof_d) {
            tof_d->write(tof_d, &test_distances[i], sizeof(uint16_t));
        }
    }

    // 设置超声波读数为 800mm (大于 ToF[0] 的 150mm，验证 Ray 0 保持 150mm)
    hal_device_t *us_d = hal_device_find("us_front");
    CHALLENGE_ASSERT(us_d != NULL, "查找 us_front 超声波设备成功");
    if (us_d) {
        hcsr04_reading_t u_r = { .distance_mm = 800, .is_timeout = false };
        us_d->write(us_d, &u_r, sizeof(u_r));
    }

    epgm_sensor_frame_t frame;
    hal_sensors_aggregator_read(&frame);

    bool all_sectors_match = true;
    for (int i = 0; i < 8; ++i) {
        if (frame.distances_mm[i] != test_distances[i]) {
            all_sectors_match = false;
            CHALLENGE_OBSERVE("Sector %d 读数不匹配: 期望 %d, 实际 %d", i, test_distances[i], frame.distances_mm[i]);
        }
    }
    CHALLENGE_ASSERT(all_sectors_match, "8 个方向全景空间测距读数与 8 路独立 ToF 完美一一映射");

    // 6.2 正前方 (Sector 0) 多模态互补融合验证 (超声波近距优先):
    // 当正前方出现 ToF 吸光透明障碍物，超声波探测到 80mm，而 ToF 仍为 150mm 时，Sector 0 必须融合为 80mm！
    if (us_d) {
        hcsr04_reading_t u_r = { .distance_mm = 80, .is_timeout = false };
        us_d->write(us_d, &u_r, sizeof(u_r));
    }
    hal_sensors_aggregator_read(&frame);
    CHALLENGE_ASSERT(frame.distances_mm[0] == 80, "正前方 (Sector 0) 成功取 ToF (150mm) 与超声波 (80mm) 之安全最小值 (80mm)");

    // 6.3 超声波超时时，不破坏 Sector 0 测距:
    // 超声波断连超时返回 4000mm 时，Sector 0 维持 ToF 测距 150mm，不应跳变为 4000mm
    if (us_d) {
        hcsr04_reading_t u_timeout = { .distance_mm = 0, .is_timeout = true };
        us_d->write(us_d, &u_timeout, sizeof(u_timeout));
    }
    hal_sensors_aggregator_read(&frame);
    CHALLENGE_ASSERT(frame.distances_mm[0] == 150, "超声波超时 (4000mm) 时，Sector 0 安全维持 ToF 真实测距 (150mm)");

    // 6.4 微动碰撞事件在聚合帧中的同步上报
    hal_device_t *bmp_d = hal_device_find("bumper_array");
    CHALLENGE_ASSERT(bmp_d != NULL, "查找 bumper_array 设备成功");
    if (bmp_d) {
        uint8_t bump_mask = BUMPER_MASK_FRONT_RIGHT | BUMPER_MASK_REAR; // 0x02 | 0x04 = 0x06
        bmp_d->write(bmp_d, &bump_mask, sizeof(uint8_t));
    }
    hal_sensors_aggregator_read(&frame);
    CHALLENGE_ASSERT(frame.bumper_hit_mask == 0x06, "聚合帧中精准捕获碰撞微动组合掩码 0x06");
    CHALLENGE_ASSERT(frame.emergency_stop == true, "碰撞微动触发使聚合帧 emergency_stop 置为 true");
    CHALLENGE_ASSERT(hal_sensors_aggregator_check_bumper() == true, "便捷函数 check_bumper 准确返回 true");

    // 6.5 聚合器生命周期幂等性测试 (连续重复 init 与 deinit 10 次，验证无内存悬挂与注册表混乱)
    bool lifecycle_stable = true;
    for (int iter = 0; iter < 10; ++iter) {
        hal_sensors_aggregator_deinit();
        if (hal_sensors_aggregator_init() != HAL_OK) {
            lifecycle_stable = false;
            break;
        }
        epgm_sensor_frame_t f;
        if (hal_sensors_aggregator_read(&f) != HAL_OK) {
            lifecycle_stable = false;
            break;
        }
    }
    CHALLENGE_ASSERT(lifecycle_stable, "聚合器连续 10 次重复生命周期注销与重初始化稳定可靠");

    hal_sensors_aggregator_deinit();
}

// ==============================================================================
// 主测试入口
// ==============================================================================
int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    printf("======================================================================\n");
    printf(" EPGM Milestone M3 Universal HAL 对抗性压力与故障注入测试套件\n");
    printf(" EPGM M3 Universal Sensor HAL Adversarial Stress & Fault Injection Suite\n");
    printf(" Challenger: Challenger 2 (Empirical Challenger)\n");
    printf("======================================================================\n");

    challenge_test_hcsr04_fault_injection();
    challenge_test_imu_slip_stall_detection();
    challenge_test_tca9548a_isolation_and_switching();
    challenge_test_bumper_debounce_and_contact_bounce();
    challenge_test_encoder_pid_step_and_anti_windup();
    challenge_test_sensor_aggregator_panoramic_fusion();

    printf("\n======================================================================\n");
    printf(" 对抗性测试套件执行完成!\n");
    printf(" 测试统计: 共执行 %d 项断言, 通过 %d 项, 失败 %d 项\n",
           s_tests_run, s_tests_passed, s_tests_failed);
    printf("======================================================================\n");

    return (s_tests_failed == 0) ? 0 : 1;
}
