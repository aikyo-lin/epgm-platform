/**
 * @file test_epgm_host.c
 * @brief [ZH] EPGM 固件全功能宿主机跨平台单元与并发压力测试套件。
 *        [EN] EPGM Firmware Comprehensive Host Unit & Concurrency Stress Test Suite.
 *
 * 验证模块覆盖 / Test Coverage:
 *  1. C 量化器与 Python 端空间/动态事件数学对齐验证 (1~81 静态, 101~181 动态, 0 碰撞)；
 *  2. FreeRTOS 多核并发安全压力测试 (50,000 次读写争用无数据竞争与死锁, 5ms 超时保底验证)；
 *  3. LittleFS Flash 二进制原子固化、CRC32 检错与冷启动恢复验证；
 *  4. 通用传感器 HAL 设备注册表、ToF、超声波、IMU 打滑检测、I2C 多路复用、编码器 PID 与微动触须驱动验证；
 *  5. 传感器聚合层 (Sensor Aggregator) 8 向全景帧集成验证。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "../src/epgm_core.h"
#include "../src/storage/epgm_storage.h"
#include "../src/hal/hal_device.h"
#include "../src/hal/hal_vl53lxx.h"
#include "../src/hal/hal_hcsr04.h"
#include "../src/hal/hal_imu.h"
#include "../src/hal/hal_tca9548a.h"
#include "../src/hal/hal_encoder_pid.h"
#include "../src/hal/hal_bumper.h"
#include "../src/hal/hal_sensors_aggregator.h"

#include "mock_freertos.h"
#include "mock_littlefs.h"
#include "mock_hardware.h"

// 统计计数
static int s_tests_run = 0;
static int s_tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    s_tests_run++; \
    if (!(cond)) { \
        printf("[FAIL] %s:%d: %s\n", __FILE__, __LINE__, msg); \
        assert(cond); \
    } else { \
        s_tests_passed++; \
        printf("[PASS] %s\n", msg); \
    } \
} while(0)

// ==============================================================================
// 1. 量化器对齐测试用例
// ==============================================================================
static void test_quantizer_alignment(void) {
    printf("\n--- [测试模块 1: 量化器与端云空间数学对齐] ---\n");

    epgm_quantizer_t q;
    epgm_quantizer_init(&q);

    // 1.1 物理微动碰撞开关 (最高优先级专属事件 0)
    uint16_t dist_safe[8] = {800, 800, 800, 800, 800, 800, 800, 800};
    uint16_t ev_bump = epgm_quantize_stateful(&q, dist_safe, true);
    TEST_ASSERT(ev_bump == 0, "微动开关触发时无条件输出碰撞专属事件 ID: 0");

    // 1.2 静态全向开阔远距 (>667mm)
    epgm_quantizer_reset(&q);
    uint16_t ev_open = epgm_quantize_stateful(&q, dist_safe, false);
    TEST_ASSERT(ev_open == 81, "全向开阔 (所有分箱为2) 输出静态最大事件 ID: 81");

    // 1.3 静态全向危险近距 (<333mm)
    epgm_quantizer_reset(&q);
    uint16_t dist_danger[8] = {200, 200, 200, 200, 200, 200, 200, 200};
    uint16_t ev_danger = epgm_quantize_stateful(&q, dist_danger, false);
    TEST_ASSERT(ev_danger == 1, "全向危险 (所有分箱为0) 输出静态最小事件 ID: 1");

    // 1.4 象限聚合测试: 仅正前方危险 (Ray 0 = 200mm, 其余 800mm)
    epgm_quantizer_reset(&q);
    uint16_t dist_front_danger[8] = {200, 800, 800, 800, 800, 800, 800, 800};
    uint16_t ev_fd = epgm_quantize_stateful(&q, dist_front_danger, false);
    // front=0, right=2, back=2, left=2 -> 1 + (0*27 + 2*9 + 2*3 + 2) = 1 + 26 = 27
    TEST_ASSERT(ev_fd == 27, "仅正前方危险时象限聚合计算得到事件 ID: 27");

    // 1.5 动态一阶差分突变测试 (delta^2 > 62500 -> +100 偏置)
    // 首次观测: 确立 baseline (静止状态)
    epgm_quantizer_reset(&q);
    uint16_t d_step1[8] = {800, 800, 800, 800, 800, 800, 800, 800};
    uint16_t ev_step1 = epgm_quantize_stateful(&q, d_step1, false);
    TEST_ASSERT(ev_step1 == 81, "初次输入无前序观测，输出静态 ID: 81");

    // 微小变化 (单方向变动 20mm, diff^2 = 400 < 62500)
    uint16_t d_step2[8] = {820, 800, 800, 800, 800, 800, 800, 800};
    uint16_t ev_step2 = epgm_quantize_stateful(&q, d_step2, false);
    TEST_ASSERT(ev_step2 == 81, "平缓微小变化不触发动态突变，维持静态 ID: 81");

    // 剧烈跳变 (正前突现障碍物: 从 800mm 骤降至 400mm, diff = 400, diff^2 = 160000 > 62500)
    // 此时距离 400mm 属于接近分箱 1，其余 800mm 属于安全分箱 2
    // front=1, right=2, back=2, left=2 -> 静态 ID = 1 + (1*27 + 2*9 + 2*3 + 2) = 54
    // 触发动态突变 -> 54 + 100 = 154
    uint16_t d_step3[8] = {400, 800, 800, 800, 800, 800, 800, 800};
    uint16_t ev_step3 = epgm_quantize_stateful(&q, d_step3, false);
    TEST_ASSERT(ev_step3 == 154, "欧氏距离平方和 > 62500 成功触发动态突变事件 ID: 154 (101~181)");

    // 动态事件后再次平稳保持在 400mm (diff = 0)
    uint16_t d_step4[8] = {400, 800, 800, 800, 800, 800, 800, 800};
    uint16_t ev_step4 = epgm_quantize_stateful(&q, d_step4, false);
    TEST_ASSERT(ev_step4 == 54, "突变后恢复平稳，事件 ID 自动回退为静态基准 ID: 54");
}

// ==============================================================================
// 2. FreeRTOS 多核并发安全压力测试用例 (50,000 次无数据撕裂与死锁)
// ==============================================================================
#define STRESS_ITERATIONS 50000

typedef struct {
    epgm_runtime_t *rt;
    volatile bool   stop_flag;
    uint32_t        writer_steps;
    uint32_t        reader_snapshots;
    uint32_t        contention_fallbacks;
} stress_context_t;

static void writer_task_func(void *param) {
    stress_context_t *ctx = (stress_context_t *)param;
    epgm_runtime_t *rt = ctx->rt;

    for (uint32_t i = 0; i < STRESS_ITERATIONS; ++i) {
        uint16_t src = (uint16_t)((i % 80) + 1);
        uint8_t  act = (uint8_t)(i % EPGM_ACTION_DIM);
        uint16_t tgt = (uint16_t)(((i * 7) % 80) + 1);
        int16_t  rew = (int16_t)((i % 10) * 25);

        // 模拟 Core 1 在 5ms 超时下安全更新
        bool updated = epgm_plasticity_update_safe(rt, src, act, tgt, rew, 5);
        if (!updated && rt->active_edge_count == 0) {
            ctx->contention_fallbacks++;
        }

        // 推理动作
        uint8_t safe_act = epgm_infer_action_safe(rt, src, 1, 5);
        (void)safe_act;

        ctx->writer_steps++;
    }
}

static void reader_task_func(void *param) {
    stress_context_t *ctx = (stress_context_t *)param;
    epgm_runtime_t *rt = ctx->rt;
    epgm_runtime_t local_snapshot;
    memset(&local_snapshot, 0, sizeof(epgm_runtime_t));

    while (!ctx->stop_flag) {
        // 模拟 Core 0 执行双缓冲快照拷贝
        if (epgm_snapshot_copy(rt, &local_snapshot, 10)) {
            // 校验快照数据一致性: 绝不出现无效动作或破损边
            assert(local_snapshot.active_edge_count <= EPGM_MAX_EDGES);
            for (uint16_t j = 0; j < local_snapshot.active_edge_count; ++j) {
                if (local_snapshot.edges[j].is_active) {
                    assert(local_snapshot.edges[j].action < EPGM_ACTION_DIM);
                }
            }
            ctx->reader_snapshots++;
        }
        vTaskDelay(1); // 模拟网络与后台存盘调度间隙
    }
}

static void locker_thread_func(void *param) {
    epgm_runtime_t *rt = (epgm_runtime_t *)param;
    if (epgm_mutex_lock(rt, 1000)) {
        vTaskDelay(100); // 占用互斥锁 100ms
        epgm_mutex_unlock(rt);
    }
}

static void test_concurrency_stress(void) {
    printf("\n--- [测试模块 2: FreeRTOS 多核并发安全压力测试 (%d 轮)] ---\n", STRESS_ITERATIONS);

    epgm_runtime_t shared_rt;
    epgm_init(&shared_rt);
    bool m_ok = epgm_mutex_init(&shared_rt);
    TEST_ASSERT(m_ok, "FreeRTOS 优先级继承互斥锁初始化成功");

    stress_context_t ctx;
    ctx.rt = &shared_rt;
    ctx.stop_flag = false;
    ctx.writer_steps = 0;
    ctx.reader_snapshots = 0;
    ctx.contention_fallbacks = 0;

    TaskHandle_t reader_task = NULL;
    TaskHandle_t writer_task = NULL;

    // 启动读者线程 (模拟 Core 0)
    xTaskCreate(reader_task_func, "TaskReader", 4096, &ctx, 1, &reader_task);
    // 启动写者线程 (模拟 Core 1)
    xTaskCreate(writer_task_func, "TaskWriter", 4096, &ctx, 5, &writer_task);

    // 等待写者完成 50,000 次并发读写
    printf("等待写者任务执行 50,000 次安全迭代...\n");
    bool w_ok = mock_task_join(writer_task, 30000);
    printf("写者任务完成状态: %d, writer_steps: %u\n", w_ok, ctx.writer_steps);
    ctx.stop_flag = true;
    bool r_ok = mock_task_join(reader_task, 5000);
    printf("读者任务完成状态: %d, reader_snapshots: %u\n", r_ok, ctx.reader_snapshots);

    TEST_ASSERT(ctx.writer_steps == STRESS_ITERATIONS, "Core 1 控制任务完成全量 50,000 次安全迭代更新");
    TEST_ASSERT(ctx.reader_snapshots > 0, "Core 0 快照任务并发提取有效一致性快照多次");
    TEST_ASSERT(shared_rt.active_edge_count > 0 && shared_rt.active_edge_count <= EPGM_MAX_EDGES,
                "图活跃边表维护完好，未发生越界溢出或数组损坏");

    // 2.2 测试 5ms 互斥锁超时保底机制 (在独立工作线程中占用互斥锁，触发主线程跨线程锁争用)
    TaskHandle_t locker_task = NULL;
    xTaskCreate(locker_thread_func, "TaskLocker", 4096, &shared_rt, 1, &locker_task);
    vTaskDelay(20); // 确保锁定线程已成功获取互斥锁

    // 此时调用安全推理，由于锁已被另一线程持有，主线程在 5ms 超时后安全降级返回 fallback 动作 3
    uint8_t fallback_result = epgm_infer_action_safe(&shared_rt, 10, 3, 5);
    TEST_ASSERT(fallback_result == 3, "互斥锁争用超时触发 5ms 保底机制，安全返回降级动作 3");

    mock_task_join(locker_task, 2000);
    epgm_mutex_deinit(&shared_rt);
}

// ==============================================================================
// 3. LittleFS Flash 掉电固化与冷启动恢复测试用例
// ==============================================================================
static void test_littlefs_storage(void) {
    printf("\n--- [测试模块 3: LittleFS Flash 掉电记忆与原子固化恢复] ---\n");

    mock_littlefs_init("./test_storage_fs");

    epgm_runtime_t orig_rt;
    epgm_init(&orig_rt);

    // 构造具有 50 条因果边的复杂因果图
    for (uint16_t i = 0; i < 50; ++i) {
        epgm_plasticity_update(&orig_rt, (i % 30) + 1, i % 5, ((i + 3) % 30) + 1, (int16_t)(i * 10));
    }
    orig_rt.step_counter = 12345;
    orig_rt.total_hits = 678;

    const char *test_path = "/littlefs/epgm_graph.bin";

    // 3.1 两阶段原子保存
    int save_res = epgm_storage_save_atomic(&orig_rt, test_path);
    TEST_ASSERT(save_res == EPGM_STORAGE_OK, "两阶段原子固化存盘成功并生成有效 CRC32");

    // 3.2 冷启动恢复
    epgm_runtime_t restored_rt;
    epgm_init(&restored_rt);

    int load_res = epgm_storage_load(&restored_rt, test_path);
    TEST_ASSERT(load_res == EPGM_STORAGE_OK, "冷启动成功从 LittleFS 反序列化因果图");
    TEST_ASSERT(restored_rt.active_edge_count == orig_rt.active_edge_count, "恢复的活跃边数与原图严格一致");
    TEST_ASSERT(restored_rt.step_counter == orig_rt.step_counter, "恢复的探索步数统计完全一致");
    TEST_ASSERT(restored_rt.total_hits == orig_rt.total_hits, "恢复的预测命中数完全一致");

    // 验证每条边的内容
    bool edges_identical = true;
    for (uint16_t i = 0; i < orig_rt.active_edge_count; ++i) {
        if (restored_rt.edges[i].source_event != orig_rt.edges[i].source_event ||
            restored_rt.edges[i].action != orig_rt.edges[i].action ||
            restored_rt.edges[i].target_event != orig_rt.edges[i].target_event ||
            restored_rt.edges[i].expected_reward_q8 != orig_rt.edges[i].expected_reward_q8) {
            edges_identical = false;
            break;
        }
    }
    TEST_ASSERT(edges_identical, "所有 50 条因果边字段数据恢复比特级完全吻合");

    // 3.3 故障注入与 CRC32 检错恢复测试 (模拟 Flash 掉电导致数据损坏)
    int corrupt_res = mock_littlefs_corrupt_file(test_path, 40); // 篡改边数据区的字节
    TEST_ASSERT(corrupt_res == 0, "成功注入模拟 Flash 单字节比特翻转损坏");

    epgm_runtime_t corrupted_rt;
    epgm_init(&corrupted_rt);
    int fail_load_res = epgm_storage_load(&corrupted_rt, test_path);
    TEST_ASSERT(fail_load_res == EPGM_STORAGE_ERR_CRC, "CRC32 校验精确拦截损坏数据并拒绝加载");

    mock_littlefs_cleanup();
}

// ==============================================================================
// 4. 通用外设 HAL 驱动单元测试用例
// ==============================================================================
static void test_hal_drivers(void) {
    printf("\n--- [测试模块 4: 通用外设 HAL 驱动架构与各外设功能] ---\n");

    hal_device_registry_reset();
    mock_hardware_init();

    // 4.1 ToF VL53L0X 驱动测试
    hal_device_t tof_dev;
    hal_vl53lxx_create(&tof_dev, "tof_test");
    vl53lxx_config_t tof_cfg = {
        .model = VL53LXX_MODEL_VL53L0X,
        .i2c_addr = 0x29,
        .mux_channel = 0,
        .timing_budget_ms = 33,
        .offset_mm = 25, // 标定补偿 +25mm
        .crosstalk_kcps = 0,
        .max_range_mm = 2000
    };
    tof_dev.init(&tof_dev, &tof_cfg);
    hal_device_register(&tof_dev);

    // 写入模拟测距 500mm
    uint16_t test_raw = 500;
    tof_dev.write(&tof_dev, &test_raw, sizeof(uint16_t));
    vl53lxx_reading_t tof_out;
    tof_dev.read(&tof_dev, &tof_out, sizeof(vl53lxx_reading_t));
    TEST_ASSERT(tof_out.distance_mm == 525, "VL53Lxx 正确叠加 +25mm 零点标定偏置 (500->525mm)");

    // 状态失效测试
    vl53lxx_reading_t bad_reading = { .distance_mm = 100, .status = VL53LXX_RANGE_SIGNAL_FAIL };
    tof_dev.write(&tof_dev, &bad_reading, sizeof(vl53lxx_reading_t));
    tof_dev.read(&tof_dev, &tof_out, sizeof(vl53lxx_reading_t));
    TEST_ASSERT(tof_out.distance_mm == 2000, "VL53Lxx 测量失效时自动安全钳位至最大量程 2000mm");

    // 4.2 超声波 HC-SR04 驱动测试
    hal_device_t us_dev;
    hal_hcsr04_create(&us_dev, "us_test");
    hcsr04_config_t us_cfg = {
        .pin_trigger = 12,
        .pin_echo = 13,
        .timeout_ms = 25,
        .max_range_mm = 4000
    };
    us_dev.init(&us_dev, &us_cfg);
    hal_device_register(&us_dev);

    hcsr04_reading_t us_in = { .distance_mm = 650, .is_timeout = false };
    us_dev.write(&us_dev, &us_in, sizeof(hcsr04_reading_t));
    hcsr04_reading_t us_out;
    us_dev.read(&us_dev, &us_out, sizeof(hcsr04_reading_t));
    TEST_ASSERT(us_out.distance_mm == 650 && !us_out.is_timeout, "HC-SR04 正常采集测距读数");

    // 25ms 保护超时测试
    hcsr04_reading_t us_timeout_in = { .distance_mm = 0, .is_timeout = true };
    us_dev.write(&us_dev, &us_timeout_in, sizeof(hcsr04_reading_t));
    us_dev.read(&us_dev, &us_out, sizeof(hcsr04_reading_t));
    TEST_ASSERT(us_out.is_timeout && us_out.distance_mm == 4000, "HC-SR04 触发 25ms 看门狗保护并返回 4000mm 极大值");

    // 4.3 6轴 IMU 姿态与打滑/堵转检测测试
    hal_device_t imu_dev;
    hal_imu_create(&imu_dev, "imu_test");
    imu_config_t imu_cfg = {
        .model = IMU_MODEL_MPU6050,
        .i2c_addr = 0x68,
        .accel_scale_g = 4,
        .gyro_scale_dps = 1000,
        .filter_alpha_q8 = 251,
        .stall_accel_thresh_mg = 30,
        .slip_gyro_thresh_dps = 50
    };
    imu_dev.init(&imu_dev, &imu_cfg);
    hal_device_register(&imu_dev);

    // 注入冰面打滑合成工况: 轮速指令 250mm/s，但纵向加计 ax = 5mg (几乎无加速度)
    imu_motion_context_t motion = { .commanded_speed_mms = 250, .measured_wheel_speed_mms = 250 };
    imu_dev.ioctl(&imu_dev, HAL_IOCTL_IMU_FEED_MOTION, &motion);

    epgm_imu_data_t slip_imu_in;
    memset(&slip_imu_in, 0, sizeof(epgm_imu_data_t));
    slip_imu_in.accel_x_mg = 5;    // 纵向无加速度
    slip_imu_in.accel_z_mg = 1000;
    slip_imu_in.gyro_z_dps = 10;
    imu_dev.write(&imu_dev, &slip_imu_in, sizeof(epgm_imu_data_t));

    // 连续读取 3 次触发超时计数门限
    epgm_imu_data_t imu_readout;
    imu_dev.read(&imu_dev, &imu_readout, sizeof(epgm_imu_data_t));
    imu_dev.read(&imu_dev, &imu_readout, sizeof(epgm_imu_data_t));
    imu_dev.read(&imu_dev, &imu_readout, sizeof(epgm_imu_data_t));
    TEST_ASSERT(imu_readout.is_skidding && imu_readout.is_stalled, "IMU 成功检出“高轮速+零加速度”车轮空转打滑与堵转状态");

    // 4.4 I2C 多路复用器 TCA9548A 测试
    hal_device_t mux_dev;
    hal_tca9548a_create(&mux_dev, "mux_test");
    mux_dev.init(&mux_dev, NULL);
    hal_device_register(&mux_dev);

    hal_tca9548a_select_channel(&mux_dev, 5);
    uint8_t current_mask = 0;
    hal_tca9548a_get_channel_mask(&mux_dev, &current_mask);
    TEST_ASSERT(current_mask == (1 << 5), "TCA9548A 成功切换至通道 5 (掩码 0x20)");

    // 4.5 差速轮编码器 PID 控制器测试
    hal_device_t pid_dev;
    hal_encoder_pid_create(&pid_dev, "pid_test");
    pid_dev.init(&pid_dev, NULL);
    hal_device_register(&pid_dev);

    // 下发动作 1 (前进, 目标速度 200mm/s)
    uint8_t action_fwd = 1;
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_SET_ACTION, &action_fwd);
    encoder_pid_status_t pid_status;
    pid_dev.read(&pid_dev, &pid_status, sizeof(encoder_pid_status_t));
    TEST_ASSERT(pid_status.left_target_mms == 200 && pid_status.right_target_mms == 200, "PID 目标轮速正确映射动作 1 为 +200mm/s");

    // 执行单步 PID 计算
    pid_dev.ioctl(&pid_dev, HAL_IOCTL_ENCODER_STEP_PID, NULL);
    pid_dev.read(&pid_dev, &pid_status, sizeof(encoder_pid_status_t));
    TEST_ASSERT(pid_status.left_pwm_out > 30 && pid_status.right_pwm_out > 30, "PID 输出有效 PWM 驱动电机并包含死区补偿");

    // 4.6 机械防碰撞微动触须测试
    hal_device_t bmp_dev;
    hal_bumper_create(&bmp_dev, "bumper_test");
    bmp_dev.init(&bmp_dev, NULL);
    hal_device_register(&bmp_dev);

    uint8_t hit_mask = BUMPER_MASK_FRONT_LEFT | BUMPER_MASK_FRONT_RIGHT;
    bmp_dev.write(&bmp_dev, &hit_mask, sizeof(uint8_t));
    bumper_status_t b_out;
    bmp_dev.read(&bmp_dev, &b_out, sizeof(bumper_status_t));
    TEST_ASSERT(b_out.is_triggered && b_out.emergency_stop, "微动开关触碰正确产生急停信号与碰撞掩码 0x03");

    // 4.7 验证设备注册表查找
    hal_device_t *found_tof = hal_device_find("tof_test");
    TEST_ASSERT(found_tof != NULL && found_tof->type == HAL_DEV_TYPE_TOF, "设备注册表按名称精确查找成功");

    hal_device_t *found_imu = hal_device_find_by_type(HAL_DEV_TYPE_IMU, 0);
    TEST_ASSERT(found_imu != NULL && strcmp(found_imu->name, "imu_test") == 0, "设备注册表按类型精确枚举成功");

    // 清理局部测试设备并重置注册表
    tof_dev.deinit(&tof_dev);
    us_dev.deinit(&us_dev);
    imu_dev.deinit(&imu_dev);
    mux_dev.deinit(&mux_dev);
    pid_dev.deinit(&pid_dev);
    bmp_dev.deinit(&bmp_dev);
    hal_device_registry_reset();
}

// ==============================================================================
// 5. 传感器阵列统一聚合器 (Sensor Aggregator) 验证
// ==============================================================================
static void test_sensor_aggregator(void) {
    printf("\n--- [测试模块 5: 传感器阵列统一聚合器 (Aggregator) 验证] ---\n");

    hal_sensors_aggregator_deinit();
    mock_hardware_init();

    int init_res = hal_sensors_aggregator_init();
    TEST_ASSERT(init_res == HAL_OK, "传感器聚合器初始化并注册全部 8 路 ToF, 超声波, IMU 与微动开关");

    epgm_sensor_frame_t frame;
    int read_res = hal_sensors_aggregator_read(&frame);
    TEST_ASSERT(read_res == HAL_OK, "传感器聚合器一键读取全景传感器帧数据成功");

    // 校验 8 个方向距离均有有效数据
    bool all_distances_valid = true;
    for (int i = 0; i < EPGM_SECTORS_COUNT; ++i) {
        if (frame.distances_mm[i] == 0) {
            all_distances_valid = false;
            break;
        }
    }
    TEST_ASSERT(all_distances_valid, "8 个方向全向空间测距均获得有效毫米级读数");
    TEST_ASSERT(frame.imu.accel_z_mg != 0, "聚合帧中包含有效的 6轴 IMU 加速度计重力参考");

    hal_sensors_aggregator_deinit();
}

// ==============================================================================
// 主测试入口
// ==============================================================================
int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    printf("======================================================================\n");
    printf(" EPGM 机器人边缘固件宿主机跨平台单元与并发压力测试套件\n");
    printf(" EPGM Robot Edge Firmware Host Unit & Concurrency Stress Test Suite\n");
    printf("======================================================================\n");

    test_quantizer_alignment();
    test_concurrency_stress();
    test_littlefs_storage();
    test_hal_drivers();
    test_sensor_aggregator();

    printf("\n======================================================================\n");
    printf(" 测试执行统计: 共运行 %d 项断言, 全部 %d 项通过 (100%% PASS)\n", s_tests_run, s_tests_passed);
    printf(" All %d test assertions PASSED successfully!\n", s_tests_passed);
    printf("======================================================================\n");

    return 0;
}
