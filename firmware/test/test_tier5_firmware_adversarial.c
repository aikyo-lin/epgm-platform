/**
 * @file test_tier5_firmware_adversarial.c
 * @brief [ZH] EPGM Phase 2 (Adversarial Coverage Hardening - Tier 5) 固件与 HAL 对抗性白盒穿透测试套件。
 *        [EN] EPGM Phase 2 Tier 5 Firmware & HAL White-Box Adversarial Penetration Test Suite.
 *
 * 审查专员 / Challenger: challenger_tier5_2 (Empirical Challenger: critic, specialist)
 * 验证范畴 / Verification Scope:
 *  1. 机械微动触须 (hal_bumper.c): 消抖滤波毛刺注入、多开关并发击中、掩码泄露、时间戳回绕、极性失效
 *  2. 6轴 IMU 动力学 (hal_imu.c): 高速巡航误判、打滑/堵转解耦、陀螺仪极值饱和、加速度姿态角溢出
 *  3. 轮式编码器 PID (hal_encoder_pid.c): 锁转抗饱和极限、突变阶跃、死区前馈跃迁、初始脉冲速度溢出
 *  4. TCA9548A I2C 多路复用器 (hal_tca9548a.c): 空指针保护、越界通道阻断、总线互斥锁争用超时
 *  5. ToF & 超声波测距 (hal_vl53lxx.c, hal_hcsr04.c): 盲区标定钳位、看门狗超时保底、全向多模态融合
 *  6. LittleFS 掉电固化 (epgm_storage.c): 掉电截断、载荷残缺、单比特翻转 CRC32 检错、Magic/版本损坏
 *  7. EPGM 因果核心 (epgm_core.c): 动态量化临界边界 (62500/62501)、多分支决策屏蔽漏洞、512边满载置换
 *  8. FreeRTOS 多核并发安全 (mock_freertos.c): 读者/写者高频并发读写、零数据撕裂、5ms 超时安全降级
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

#include "epgm_core.h"
#include "epgm_storage.h"
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
#include "mock_littlefs.h"

// ==============================================================================
// 测试断言与统计宏
// ==============================================================================
static int s_tier5_tests_run = 0;
static int s_tier5_tests_passed = 0;
static int s_tier5_tests_failed = 0;

#define TIER5_ASSERT(cond, desc) do { \
    s_tier5_tests_run++; \
    if (cond) { \
        s_tier5_tests_passed++; \
        printf("  [PASS] %s\n", desc); \
    } else { \
        s_tier5_tests_failed++; \
        printf("  [FAIL - BUG CONFIRMED] %s (Line %d: %s)\n", desc, __LINE__, #cond); \
    } \
} while(0)

// ==============================================================================
// 1. 机械微动触须 HAL 对抗性测试 (hal_bumper.c)
// ==============================================================================
static void test_adversarial_bumper(void) {
    printf("\n======================================================================\n");
    printf(" [TIER 5 - TEST 1] 机械微动触须 HAL 对抗性毛刺注入与边界穿透测试\n");
    printf("======================================================================\n");

    hal_device_t bumper_dev;
    hal_bumper_create(&bumper_dev, "bumper_adv");
    bumper_config_t cfg = {
        .pin_front_left = 14,
        .pin_front_right = 15,
        .pin_rear = 16,
        .debounce_ms = 5,
        .active_low = true
    };
    bumper_dev.init(&bumper_dev, &cfg);

    mock_hardware_reset();
    // 初始所有引脚高电平 (未按下)
    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(15, 1);
    mock_gpio_set_level(16, 1);

    // 1.1 亚 5ms 高频毛刺滤波验证 (1ms, 2ms, 3ms, 4ms 单触点弹跳)
    bumper_status_t status;
    for (int glitch_ms = 1; glitch_ms <= 4; ++glitch_ms) {
        mock_gpio_set_level(14, 0); // 拉低产生毛刺
        bumper_dev.read(&bumper_dev, &status, sizeof(status));
        // 单次毛刺尚未持续 5ms，不得触发 is_triggered 和 emergency_stop
        TIER5_ASSERT(!status.is_triggered, "亚5ms毛刺期间 is_triggered 保持 false");
        TIER5_ASSERT(!status.emergency_stop, "亚5ms毛刺期间 emergency_stop 保持 false");

        // 模拟瞬时回弹拉高
        mock_gpio_set_level(14, 1);
        bumper_dev.read(&bumper_dev, &status, sizeof(status));
    }

    // 1.2 持续 >5ms 触碰，验证正常触发急停 (Windows 宿主机下延迟 25ms 跨越系统 Tick 分辨率)
    mock_gpio_set_level(14, 0);
    bumper_dev.read(&bumper_dev, &status, sizeof(status)); // t = 0
    vTaskDelay(25); // 延迟 25ms，确保 GetTickCount() 步进
    bumper_dev.read(&bumper_dev, &status, sizeof(status));
    TIER5_ASSERT(status.is_triggered, "持续触碰超过 5ms 后成功触发 is_triggered = true");
    TIER5_ASSERT(status.emergency_stop, "持续触碰超过 5ms 后成功置位 emergency_stop = true");
    TIER5_ASSERT(status.bumper_mask == BUMPER_MASK_FRONT_LEFT, "掩码精确识别前左微动");

    // 1.3 多开关同时并发触碰 (前左 + 前右同时按下): 判定为严重正面撞击，立即急停绕过消抖
    mock_gpio_set_level(14, 0);
    mock_gpio_set_level(15, 0);
    bumper_dev.read(&bumper_dev, &status, sizeof(status));
    TIER5_ASSERT(status.is_triggered && status.emergency_stop, "双触须同时触碰瞬时触发急停");
    TIER5_ASSERT(status.bumper_mask == (BUMPER_MASK_FRONT_LEFT | BUMPER_MASK_FRONT_RIGHT), "双触须掩码精确置位 (0x03)");

    // 1.4 全部释放后状态完全清除
    mock_gpio_set_level(14, 1);
    mock_gpio_set_level(15, 1);
    mock_gpio_set_level(16, 1);
    bumper_dev.read(&bumper_dev, &status, sizeof(status));
    TIER5_ASSERT(!status.is_triggered && !status.emergency_stop && status.bumper_mask == 0,
                 "物理释放后全部碰撞与急停状态自动清除归零");

    // 1.5 掩码防泄露验证 (White-box Finding 2.2 防御加固验证):
    // 在消抖期间读取单个字节 uint8_t 时，内部 bumper_mask 受到隔离，不再提前反映未确认的 raw_mask
    mock_gpio_set_level(16, 0); // 后方触碰
    uint8_t raw_byte = 0;
    bumper_dev.read(&bumper_dev, &raw_byte, sizeof(uint8_t));
    TIER5_ASSERT(raw_byte == 0, "加固后: 单字节读取在消抖窗口期间不再透出未确认的原始掩码");

    // 1.6 active_low = false (高电平有效微动) 测试 (White-box Finding 2.1 防御加固验证)
    bumper_config_t cfg_active_high = {
        .pin_front_left = 14,
        .pin_front_right = 15,
        .pin_rear = 16,
        .debounce_ms = 5,
        .active_low = false // 配置为高电平有效
    };
    bumper_dev.init(&bumper_dev, &cfg_active_high);
    mock_gpio_set_level(14, 1); // 传入高电平
    bumper_dev.read(&bumper_dev, &status, sizeof(status));
    vTaskDelay(25); // 跨越 5ms 消抖窗口
    bumper_dev.read(&bumper_dev, &status, sizeof(status));
    printf("  [REMEDIATION VERIFIED] Finding 2.1: active_low=false 时引脚高电平被驱动成功识别并触发急停 (triggered=%d)\n", status.is_triggered);
    TIER5_ASSERT(status.is_triggered, "加固后: active_low=false 配置下高电平触发被驱动成功识别并触发急停");

    bumper_dev.deinit(&bumper_dev);
}

// ==============================================================================
// 2. 6轴 IMU 动力学与打滑/堵转解耦对抗测试 (hal_imu.c)
// ==============================================================================
static void test_adversarial_imu(void) {
    printf("\n======================================================================\n");
    printf(" [TIER 5 - TEST 2] 6轴 IMU 姿态解算与打滑/堵转动力学对抗测试\n");
    printf("======================================================================\n");

    hal_device_t imu_dev;
    hal_imu_create(&imu_dev, "imu_adv");
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

    epgm_imu_data_t imu_data;
    memset(&imu_data, 0, sizeof(imu_data));
    imu_data.accel_z_mg = 1000;

    // 2.1 标称巡航工况 (|v| <= 200 mm/s): 匀速直线巡航，ax=0，不发生误报
    imu_motion_context_t motion_normal = {
        .commanded_speed_mms = 180,
        .measured_wheel_speed_mms = 180
    };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_normal);
    imu_data.accel_x_mg = 0;
    imu_data.gyro_z_dps = 0;
    imu_dev.write(&imu_dev, &imu_data, sizeof(imu_data));
    for (int i = 0; i < 5; ++i) {
        imu_dev.read(&imu_dev, &imu_data, sizeof(imu_data));
    }
    TIER5_ASSERT(!imu_data.is_skidding && !imu_data.is_stalled,
                 "标称速度 (180mm/s) 匀速巡航无打滑/堵转假阳性误报");

    // 2.2 高速直线巡航工况 (|v| = 250 mm/s > 200 mm/s) (White-box Finding 3.1):
    // 轮速与指令完全吻合 (250mm/s)，但因超过 200 且 ax=0，触发分支 B 误报为打滑与堵转
    imu_motion_context_t motion_high_speed = {
        .commanded_speed_mms = 250,
        .measured_wheel_speed_mms = 250
    };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_high_speed);
    imu_data.accel_x_mg = 0;
    imu_data.gyro_z_dps = 0;
    imu_dev.write(&imu_dev, &imu_data, sizeof(imu_data));
    for (int i = 0; i < 4; ++i) {
        imu_dev.read(&imu_dev, &imu_data, sizeof(imu_data));
    }
    printf("  [REMEDIATION VERIFIED] Finding 3.1 实测: 250mm/s 匀速直线巡航 (ax=0) 保持正常 (is_skidding=%d, is_stalled=%d)\n",
           imu_data.is_skidding, imu_data.is_stalled);
    TIER5_ASSERT(!imu_data.is_skidding && !imu_data.is_stalled,
                 "加固后: 高速匀速巡航 (>200mm/s) 优先判定巡航条件，消除打滑/堵转假阳性误报");

    // 清除标志
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_CLEAR_FLAG, NULL);

    // 2.3 真实堵转工况 (指令 150mm/s，实测轮速 0mm/s，ax=0):
    // 严格解耦: is_stalled = true, is_skidding = false
    imu_motion_context_t motion_stall = {
        .commanded_speed_mms = 150,
        .measured_wheel_speed_mms = 0
    };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion_stall);
    imu_data.accel_x_mg = 0;
    imu_data.gyro_z_dps = 0;
    imu_dev.write(&imu_dev, &imu_data, sizeof(imu_data));
    for (int i = 0; i < 4; ++i) {
        imu_dev.read(&imu_dev, &imu_data, sizeof(imu_data));
    }
    TIER5_ASSERT(imu_data.is_stalled && !imu_data.is_skidding,
                 "电机真实堵转时严格解耦: is_stalled=true 且 is_skidding=false");

    // 2.4 剧烈旋转打滑 (gz = 120 dps > 50 dps)
    imu_data.gyro_z_dps = 120;
    imu_dev.write(&imu_dev, &imu_data, sizeof(imu_data));
    imu_dev.read(&imu_dev, &imu_data, sizeof(imu_data));
    TIER5_ASSERT(imu_data.is_skidding, "车身偏航角速度突变 (>50dps) 瞬时检测出打滑");

    // 2.5 极值姿态解算 (az -> 0 接近失重状态)
    imu_data.accel_x_mg = 2000;
    imu_data.accel_z_mg = 0; // az=0 保护测试
    imu_dev.write(&imu_dev, &imu_data, sizeof(imu_data));
    int res = imu_dev.read(&imu_dev, &imu_data, sizeof(imu_data));
    TIER5_ASSERT(res == sizeof(imu_data), "az=0 奇异工况下防除零保护生效");

    imu_dev.deinit(&imu_dev);
}

// ==============================================================================
// 3. 差速编码器速度 PID 对抗测试 (hal_encoder_pid.c)
// ==============================================================================
static void test_adversarial_encoder_pid(void) {
    printf("\n======================================================================\n");
    printf(" [TIER 5 - TEST 3] 差速编码器速度 PID 抗饱和与阶跃冲击对抗测试\n");
    printf("======================================================================\n");

    hal_device_t pid_dev;
    hal_encoder_pid_create(&pid_dev, "pid_adv");
    encoder_pid_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.left_pid.kp_q8 = 307;
    cfg.left_pid.ki_q8 = 38;
    cfg.left_pid.kd_q8 = 13;
    cfg.left_pid.integral_max = 2000;
    cfg.left_pid.deadband_pwm = 30;
    cfg.left_pid.max_output_pwm = 255;
    cfg.right_pid = cfg.left_pid;
    cfg.cruise_speed = 200;
    cfg.turn_speed = 150;
    pid_dev.init(&pid_dev, &cfg);

    encoder_pid_status_t status;

    // 3.1 设定前进动作 (目标速度 200mm/s)
    uint8_t action = 1;
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &action);
    pid_dev.read(&pid_dev, &status, sizeof(status));
    TIER5_ASSERT(status.left_target_mms == 200 && status.right_target_mms == 200,
                 "前进动作准确配置双轮目标速度为 200mm/s");

    // 3.2 长期锁转堵转 (轮速卡死在 0) 1000 周期抗积分饱和验证
    for (int step = 0; step < 1000; ++step) {
        int32_t ticks[2] = {0, 0};
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, ticks);
        pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
    }
    pid_dev.read(&pid_dev, &status, sizeof(status));
    TIER5_ASSERT(status.left_pwm_out == 255 && status.right_pwm_out == 255,
                 "持续锁转下 PWM 严格被钳位于 max_output_pwm (255)");

    // 3.3 锁转解除并立即停机 (target = 0, speed = 0)
    action = 0;
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &action);
    int32_t zero_ticks[2] = {0, 0};
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, zero_ticks);
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
    pid_dev.read(&pid_dev, &status, sizeof(status));
    TIER5_ASSERT(status.left_pwm_out == 0 && status.right_pwm_out == 0,
                 "停机时 (target=0, measured=0) 积分项瞬时清零，PWM 输出严格为 0");

    // 3.4 极值反向阶跃 (+200 -> -200 mm/s)
    int16_t reverse_targets[2] = {-200, -200};
    pid_dev.write(&pid_dev, reverse_targets, sizeof(reverse_targets));
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
    pid_dev.read(&pid_dev, &status, sizeof(status));
    TIER5_ASSERT(status.left_pwm_out == -255 && status.right_pwm_out == -255,
                 "极值反向阶跃下 PWM 迅速响应并钳位至 -255");

    // 3.5 初始脉冲大跳变溢出测试 (White-box Finding 4.1)
    // 首次调用 FEED_TICKS 传入 20000 脉冲 (未清零)
    int32_t initial_jump_ticks[2] = {20000, 20000};
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_FEED_TICKS, initial_jump_ticks);
    pid_dev.read(&pid_dev, &status, sizeof(status));
    printf("  [REMEDIATION VERIFIED] Finding 4.1 实测: 初始脉冲跳变 20000 经饱和钳位后速度计算为 %d mm/s\n",
           status.left_speed_mms);
    TIER5_ASSERT(status.left_speed_mms >= -2000 && status.left_speed_mms <= 2000,
                 "加固后: 编码器初次大跳变被饱和钳位于 [-2000, 2000]");

    pid_dev.deinit(&pid_dev);
}

// ==============================================================================
// 4. TCA9548A I2C 多路复用器与并发互斥测试 (hal_tca9548a.c)
// ==============================================================================
static void test_adversarial_tca9548a(void) {
    printf("\n======================================================================\n");
    printf(" [TIER 5 - TEST 4] TCA9548A 多路复用器通道边界与互斥争用测试\n");
    printf("======================================================================\n");

    hal_device_t mux_dev;
    hal_tca9548a_create(&mux_dev, "mux_adv");
    tca9548a_config_t cfg = {
        .i2c_addr = TCA9548A_DEFAULT_I2C_ADDR,
        .default_channel = 0,
        .enable_mutex = true
    };
    mux_dev.init(&mux_dev, &cfg);

    // 4.1 通道边界: 0~7 合法选通
    for (uint8_t ch = 0; ch < 8; ++ch) {
        int rc = hal_tca9548a_select_channel(&mux_dev, ch);
        TIER5_ASSERT(rc == HAL_OK, "合法通道选通成功");
        uint8_t mask = 0;
        hal_tca9548a_get_channel_mask(&mux_dev, &mask);
        TIER5_ASSERT(mask == (uint8_t)(1 << ch), "通道单热码掩码严格匹配");
    }

    // 4.2 非法通道: 8, 9, 255 返回 HAL_ERR_PARAM
    TIER5_ASSERT(hal_tca9548a_select_channel(&mux_dev, 8) == HAL_ERR_PARAM, "越界通道 8 被安全拦截");
    TIER5_ASSERT(hal_tca9548a_select_channel(&mux_dev, 255) == HAL_ERR_PARAM, "越界通道 255 被安全拦截");

    // 4.3 NULL 指针安全
    TIER5_ASSERT(hal_tca9548a_get_channel_mask(NULL, NULL) == HAL_ERR_PARAM, "get_channel_mask 空指针安全拦截");

    mux_dev.deinit(&mux_dev);
}

// ==============================================================================
// 5. ToF & 超声波测距外设对抗测试 (hal_vl53lxx.c, hal_hcsr04.c)
// ==============================================================================
static void test_adversarial_ranging_sensors(void) {
    printf("\n======================================================================\n");
    printf(" [TIER 5 - TEST 5] ToF 与超声波测距盲区标定与看门狗超时测试\n");
    printf("======================================================================\n");

    // 5.1 ToF 盲区标定钳位测试
    hal_device_t tof_dev;
    hal_vl53lxx_create(&tof_dev, "tof_adv");
    vl53lxx_config_t tof_cfg = {
        .model = VL53LXX_MODEL_VL53L0X,
        .i2c_addr = 0x29,
        .mux_channel = 0,
        .timing_budget_ms = 33,
        .offset_mm = -50, // 故意设置 -50mm 零点负偏移
        .crosstalk_kcps = 0,
        .max_range_mm = 2000
    };
    tof_dev.init(&tof_dev, &tof_cfg);

    uint16_t dist = 0;
    // 注入 40mm 读数，加上 -50mm 偏差后为 -10mm -> 必须钳位至硬件物理盲区 30mm
    uint16_t raw_40 = 40;
    tof_dev.write(&tof_dev, &raw_40, sizeof(uint16_t));
    tof_dev.read(&tof_dev, &dist, sizeof(uint16_t));
    TIER5_ASSERT(dist == 30, "ToF 负标定偏移下自动钳位于物理盲区 30mm");

    // 注入异常测距状态码 (状态码 2: 超出量程)
    vl53lxx_reading_t invalid_reading = {
        .distance_mm = 100,
        .status = VL53LXX_RANGE_SIGNAL_FAIL,
        .timestamp_us = 0
    };
    tof_dev.write(&tof_dev, &invalid_reading, sizeof(invalid_reading));
    tof_dev.read(&tof_dev, &dist, sizeof(uint16_t));
    TIER5_ASSERT(dist == 2000, "ToF 异常状态码下安全保底返回最大量程 2000mm");
    tof_dev.deinit(&tof_dev);

    // 5.2 HC-SR04 看门狗超时保护测试
    hal_device_t us_dev;
    hal_hcsr04_create(&us_dev, "us_adv");
    hcsr04_config_t us_cfg = {
        .pin_trigger = 12,
        .pin_echo = 13,
        .timeout_ms = 25,
        .max_range_mm = 4000
    };
    us_dev.init(&us_dev, &us_cfg);

    hcsr04_reading_t us_reading = {
        .distance_mm = 500,
        .is_timeout = true, // 模拟硬件断连或无回波超时
        .echo_duration_us = 0
    };
    us_dev.write(&us_dev, &us_reading, sizeof(us_reading));
    us_dev.read(&us_dev, &us_reading, sizeof(us_reading));
    TIER5_ASSERT(us_reading.distance_mm == 4000, "超声波超时强制上报超量程 4000mm");
    TIER5_ASSERT(us_reading.echo_duration_us == 25000, "回波时间严格钳位于看门狗 25ms (25000us)");
    us_dev.deinit(&us_dev);
}

// ==============================================================================
// 6. LittleFS 掉电固化与 CRC32 容灾测试 (epgm_storage.c)
// ==============================================================================
static void test_adversarial_storage_powerloss(void) {
    printf("\n======================================================================\n");
    printf(" [TIER 5 - TEST 6] LittleFS 掉电原子事务与 CRC32 容灾测试\n");
    printf("======================================================================\n");

    mock_littlefs_init("./test_adv_fs");

    epgm_runtime_t rt_orig;
    epgm_init(&rt_orig);
    rt_orig.step_counter = 1234;
    rt_orig.total_hits = 1000;
    rt_orig.total_surprises = 234;

    // 构造 30 条边
    for (uint16_t i = 0; i < 30; ++i) {
        rt_orig.edges[i].source_event = (i % 81) + 1;
        rt_orig.edges[i].action = i % 5;
        rt_orig.edges[i].target_event = ((i + 1) % 81) + 1;
        rt_orig.edges[i].expected_reward_q8 = (int16_t)(i * 10);
        rt_orig.edges[i].observation_count = i + 1;
        rt_orig.edges[i].confidence = 100;
        rt_orig.edges[i].is_active = true;
    }
    rt_orig.active_edge_count = 30;

    const char *save_file = "/littlefs/adv_graph.bin";
    int save_rc = epgm_storage_save_atomic(&rt_orig, save_file);
    TIER5_ASSERT(save_rc == EPGM_STORAGE_OK, "初始两阶段原子固化成功");

    // 6.1 验证正常冷启动恢复
    epgm_runtime_t rt_restored;
    epgm_init(&rt_restored);
    int load_rc = epgm_storage_load(&rt_restored, save_file);
    TIER5_ASSERT(load_rc == EPGM_STORAGE_OK, "冷启动加载图拓扑成功");
    TIER5_ASSERT(rt_restored.active_edge_count == 30, "恢复边数严格等于 30");
    TIER5_ASSERT(rt_restored.step_counter == 1234, "累计步数严格恢复为 1234");

    // 6.2 模拟掉电截断 (.tmp 文件只有 16 字节头部，断电发生)
    char host_path[256];
    mock_littlefs_resolve_path("/littlefs/adv_graph.bin.tmp", host_path, sizeof(host_path));
    FILE *fp_cut = fopen(host_path, "wb");
    if (fp_cut) {
        uint8_t garbage[16] = {0x45, 0x50, 0x47, 0x4D}; // 只有半截
        fwrite(garbage, 1, 16, fp_cut);
        fclose(fp_cut);
    }
    // 验证冷启动依然安全加载原文件 adv_graph.bin，未被残缺 .tmp 破坏
    epgm_init(&rt_restored);
    load_rc = epgm_storage_load(&rt_restored, save_file);
    TIER5_ASSERT(load_rc == EPGM_STORAGE_OK && rt_restored.active_edge_count == 30,
                 "断电残留 .tmp 存在时，主文件状态安全不受影响并成功恢复");
    remove(host_path);

    // 6.3 模拟 Flash 介质比特翻转 (Byte 40 异或 0x01)
    mock_littlefs_corrupt_file(save_file, 40);
    load_rc = epgm_storage_load(&rt_restored, save_file);
    TIER5_ASSERT(load_rc == EPGM_STORAGE_ERR_CRC, "Flash 单比特翻转损坏被 CRC32 拦截 (EPGM_STORAGE_ERR_CRC)");

    // 6.4 模拟 Magic 损坏
    mock_littlefs_resolve_path(save_file, host_path, sizeof(host_path));
    FILE *fp_corrupt = fopen(host_path, "r+b");
    if (fp_corrupt) {
        uint32_t bad_magic = 0xDEADBEEF;
        fseek(fp_corrupt, 0, SEEK_SET);
        fwrite(&bad_magic, sizeof(bad_magic), 1, fp_corrupt);
        fclose(fp_corrupt);
    }
    load_rc = epgm_storage_load(&rt_restored, save_file);
    TIER5_ASSERT(load_rc == EPGM_STORAGE_ERR_MAGIC, "非法 Magic 被精确拦截 (EPGM_STORAGE_ERR_MAGIC)");

    mock_littlefs_cleanup();
}

// ==============================================================================
// 7. EPGM 因果核心与量化器边界测试 (epgm_core.c)
// ==============================================================================
static void test_adversarial_core_and_quantizer(void) {
    printf("\n======================================================================\n");
    printf(" [TIER 5 - TEST 7] EPGM 因果核心算法、多分支屏蔽与量化边界测试\n");
    printf("======================================================================\n");

    // 7.1 有状态差分量化器边界: 62500 与 62501
    epgm_quantizer_t q;
    epgm_quantizer_init(&q);

    uint16_t d0[8] = {500, 500, 500, 500, 500, 500, 500, 500};
    uint16_t ev0 = epgm_quantize_stateful(&q, d0, false);
    TIER5_ASSERT(ev0 == 41, "初始基线 500mm 量化为静态事件 41");

    // 临界差分: 单路跳变 250mm -> diff^2 = 62500. 规范规定 > 62500 才触发动态脉冲
    uint16_t d_exact[8] = {750, 500, 500, 500, 500, 500, 500, 500};
    uint16_t ev_exact = epgm_quantize_stateful(&q, d_exact, false);
    TIER5_ASSERT(ev_exact <= 81, "diff^2 严格等于 62500 时维持静态分箱，不触发动态脉冲");

    // 重新基线
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d0, false);

    // 临界差分: 251mm -> diff^2 = 63001 > 62500. 必须触发动态脉冲 (+100)
    uint16_t d_pulse[8] = {751, 500, 500, 500, 500, 500, 500, 500};
    uint16_t ev_pulse = epgm_quantize_stateful(&q, d_pulse, false);
    TIER5_ASSERT(ev_pulse >= 101 && ev_pulse <= 181, "diff^2 > 62500 严格触发动态脉冲 [101, 181]");

    // 7.2 多分支决策屏蔽漏洞审计验证 (White-box Finding 1.1)
    // 场景: 事件 10 采取动作 1，历史有两条边:
    // 边 1 (较早生成): target = 20, expected_reward = +100
    // 边 2 (后生成): target = 0 (微动碰撞), expected_reward = -1000
    epgm_runtime_t rt;
    epgm_init(&rt);

    // 边 1: 前进 -> 目标 20
    rt.edges[0].source_event = 10;
    rt.edges[0].action = 1;
    rt.edges[0].target_event = 20;
    rt.edges[0].expected_reward_q8 = 256 * 2; // +2.0
    rt.edges[0].observation_count = 10;
    rt.edges[0].confidence = 200;
    rt.edges[0].is_active = true;

    // 边 2: 前进 -> 碰撞事件 0 (致命碰撞)
    rt.edges[1].source_event = 10;
    rt.edges[1].action = 1;
    rt.edges[1].target_event = 0; // 碰撞!
    rt.edges[1].expected_reward_q8 = -256 * 10; // -10.0
    rt.edges[1].observation_count = 5;
    rt.edges[1].confidence = 250;
    rt.edges[1].is_active = true;
    rt.active_edge_count = 2;

    // 调用推理函数
    uint8_t chosen_act = epgm_infer_action(&rt, 10);
    printf("  [REMEDIATION VERIFIED] Finding 1.1 实测: 存在严重碰撞边 (target=0) 时，全面评估惩罚后推荐动作 %d\n", chosen_act);
    TIER5_ASSERT(chosen_act != 1, "加固后: 动作推理全面评估同源碰撞边，成功规避碰撞动作 1");

    // 7.3 512 条因果边满载容量与置换淘汰测试
    // 使用 512 组完全不重复的 (source_event, action) 触发 512 次 Surprise 生枝
    epgm_init(&rt);
    for (uint16_t i = 0; i < EPGM_MAX_EDGES; ++i) {
        uint16_t src = (i % 256);
        uint8_t act = (uint8_t)((i / 256) % 5);
        epgm_plasticity_update(&rt, src, act, (src + 1) % 256, 100);
    }
    TIER5_ASSERT(rt.active_edge_count == EPGM_MAX_EDGES, "成功达到 512 条因果边满载上限");

    // 插入第 513 条新边 (触发淘汰置换)
    epgm_plasticity_update(&rt, 255, 4, 1, 200);
    TIER5_ASSERT(rt.active_edge_count == EPGM_MAX_EDGES, "513条边更新后容量维持严格等于 512，无内存越界");
}

// ==============================================================================
// 8. FreeRTOS 多核并发压力测试 (mock_freertos.c)
// ==============================================================================
typedef struct {
    epgm_runtime_t *rt;
    bool            stop_requested;
    uint32_t        write_count;
    uint32_t        read_count;
    uint32_t        torn_read_count;
} thread_stress_ctx_t;

static void writer_task(void *pvParameters) {
    thread_stress_ctx_t *ctx = (thread_stress_ctx_t *)pvParameters;
    for (int i = 0; i < 50000 && !ctx->stop_requested; ++i) {
        uint16_t src = (i % 50) + 1;
        uint8_t act = (i % 5);
        uint16_t tgt = ((i + 1) % 50) + 1;
        epgm_plasticity_update_safe(ctx->rt, src, act, tgt, 50, 5);
        ctx->write_count++;
    }
}

static void reader_task(void *pvParameters) {
    thread_stress_ctx_t *ctx = (thread_stress_ctx_t *)pvParameters;
    epgm_runtime_t snapshot;
    epgm_init(&snapshot);
    epgm_mutex_init(&snapshot);

    while (!ctx->stop_requested) {
        if (epgm_snapshot_copy(ctx->rt, &snapshot, 5)) {
            ctx->read_count++;
            // 守恒一致性检验: step_counter == hits + surprises
            uint32_t sum = (uint32_t)snapshot.total_hits + (uint32_t)snapshot.total_surprises;
            if (snapshot.step_counter != sum) {
                ctx->torn_read_count++;
            }
        }
    }
    epgm_mutex_deinit(&snapshot);
}

static void contention_blocker_worker(void *pvParameters) {
    epgm_runtime_t *rt = (epgm_runtime_t *)pvParameters;
    if (epgm_mutex_lock(rt, 1000)) {
        vTaskDelay(25); // 持有锁 25ms，触发对端 5ms 超时保底
        epgm_mutex_unlock(rt);
    }
}

static void test_adversarial_multicore_concurrency(void) {
    printf("\n======================================================================\n");
    printf(" [TIER 5 - TEST 8] FreeRTOS 多核并发极限争用与数据撕裂压力测试\n");
    printf("======================================================================\n");

    epgm_runtime_t rt;
    epgm_init(&rt);
    epgm_mutex_init(&rt);

    thread_stress_ctx_t ctx = {
        .rt = &rt,
        .stop_requested = false,
        .write_count = 0,
        .read_count = 0,
        .torn_read_count = 0
    };

    TaskHandle_t t_writer = NULL;
    TaskHandle_t t_reader = NULL;

    xTaskCreate(writer_task, "writer", 4096, &ctx, 2, &t_writer);
    xTaskCreate(reader_task, "reader", 4096, &ctx, 1, &t_reader);

    // 等待写线程完成
    mock_task_join(t_writer, 5000);
    ctx.stop_requested = true;
    mock_task_join(t_reader, 1000);

    printf("  >> 写者执行次数: %u, 读者成功捕获快照: %u, 数据撕裂: %u\n",
           ctx.write_count, ctx.read_count, ctx.torn_read_count);

    TIER5_ASSERT(ctx.write_count == 50000, "写者线程安全执行 50,000 次突触更新");
    TIER5_ASSERT(ctx.read_count > 1000, "读者线程高频捕获快照 > 1000 次");
    TIER5_ASSERT(ctx.torn_read_count == 0, "双缓冲锁机制完全消除数据撕裂 (Torn Read = 0)");

    // 8.2 互斥锁超时 5ms 降级测试 (由独立工作线程持有互斥锁)
    TaskHandle_t t_blocker = NULL;
    xTaskCreate(contention_blocker_worker, "blocker", 4096, &rt, 3, &t_blocker);
    vTaskDelay(5); // 确保 blocker 线程已获取锁

    // 在锁被占有时调用 infer_action_safe，必须在 5ms 内超时返回保底动作 4 (原地右转)
    uint8_t fallback = epgm_infer_action_safe(&rt, 10, 4, 5);
    TIER5_ASSERT(fallback == 4, "独立线程锁争用下 5ms 超时安全返回保底动作 4");

    mock_task_join(t_blocker, 1000);
    epgm_mutex_deinit(&rt);
}

// ==============================================================================
// 主测试入口
// ==============================================================================
int main(void) {
    printf("======================================================================\n");
    printf(" EPGM Phase 2 (Adversarial Coverage Hardening - Tier 5) C Test Suite\n");
    printf(" Firmware & HAL White-Box Penetration & Extreme Boundary Verification\n");
    printf(" Subagent: challenger_tier5_2\n");
    printf("======================================================================\n");

    test_adversarial_bumper();
    test_adversarial_imu();
    test_adversarial_encoder_pid();
    test_adversarial_tca9548a();
    test_adversarial_ranging_sensors();
    test_adversarial_storage_powerloss();
    test_adversarial_core_and_quantizer();
    test_adversarial_multicore_concurrency();

    printf("\n======================================================================\n");
    printf(" TIER 5 C 固件与 HAL 对抗性测试执行统计:\n");
    printf("  总执行断言数: %d\n", s_tier5_tests_run);
    printf("  成功通过项数: %d\n", s_tier5_tests_passed);
    printf("  验证失败项数: %d\n", s_tier5_tests_failed);
    printf("======================================================================\n");

    if (s_tier5_tests_failed == 0) {
        printf("\n\033[92m[SUCCESS] Tier 5 Firmware & HAL Adversarial Suite 100%% PASSED!\033[0m\n\n");
        return 0;
    } else {
        printf("\n\033[91m[FAILURE] Tier 5 Firmware & HAL Adversarial Suite Encountered Failures!\033[0m\n\n");
        return 1;
    }
}
