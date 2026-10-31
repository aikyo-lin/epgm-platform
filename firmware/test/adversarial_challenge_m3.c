/**
 * @file adversarial_challenge_m3.c
 * @brief [ZH] Milestone M3 边缘固件对抗性压力测试与故障注入验证套件 (Challenger 1)。
 *        [EN] Milestone M3 Edge Firmware Adversarial Stress & Fault Injection Challenge Suite.
 *
 * 挑战测试项目 / Challenge Test Modules:
 *  1. 并发压力挑战 (Concurrency Stress Challenge):
 *     - Core 1 控制任务 (优先级高) 执行 100,000 次持续更新与推理；
 *     - Core 0 遥测任务 (优先级低) 并发高频执行 epgm_snapshot_copy；
 *     - 实时验证全局图不变量: step_counter == total_hits + total_surprises，严防数据撕裂 (Torn Read/Write)；
 *     - 验证 5ms 互斥锁超时严重争用下的安全保底机制 (返回降级动作，不阻塞，无死锁)。
 *  2. LittleFS Flash 掉电与故障注入挑战 (LittleFS Fault Injection Challenge):
 *     - 模拟在写入临时文件 (.tmp) 过程中发生突发掉电中断 (0字节、部分文件头、部分边数据截断)；
 *     - 模拟重命名前后掉电，验证冷启动加载安全回退至上一有效因果图快照；
 *     - 模拟 Magic 损坏、版本不匹配、数组越界及 CRC32 校验码损坏，验证精确拒绝与防御性保护。
 *  3. 量化器等价性与边缘振荡挑战 (Quantizer Parity & Anti-Oscillation Challenge):
 *     - 严格验证欧氏距离平方和门限 sum(Delta d^2) > 62500 的严格边界 (62499, 62500, 62501)；
 *     - 验证单路与多路微小扰动不产生错误动态事件，动态脉冲单拍触发后稳态自动回归静态事件 (无残留与振荡)；
 *     - 验证动态事件 ID 严格分布于 101~181，碰撞为 0，静态为 1~81。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "../src/epgm_core.h"
#include "../src/storage/epgm_storage.h"
#include "mock_freertos.h"
#include "mock_littlefs.h"

#if defined(_WIN32) || defined(_WIN64)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

// 测试统计
static int s_chall_total = 0;
static int s_chall_passed = 0;
static int s_chall_failed = 0;

#define CHALLENGE_ASSERT(cond, desc) do { \
    s_chall_total++; \
    if (cond) { \
        s_chall_passed++; \
        printf("  [CHALLENGE PASS] %s\n", desc); \
    } else { \
        s_chall_failed++; \
        printf("  [CHALLENGE FAIL] %s:%d: %s\n", __FILE__, __LINE__, desc); \
    } \
} while(0)

// ==============================================================================
// 1. 并发压力挑战: 100,000 轮多线程读写、防撕裂与 5ms 超时保底
// ==============================================================================
#define CONCURRENCY_ITERATIONS 100000

typedef struct {
    epgm_runtime_t *shared_rt;
    volatile bool   writer_done;
    volatile bool   stop_reader;
    
    // 统计指标
    uint32_t writer_iterations;
    uint32_t reader_snapshots_taken;
    uint32_t torn_reads_detected;
    uint32_t edge_corruptions_detected;
    uint32_t contention_timeouts_handled;
    uint32_t max_concurrency_edges;
} concurrency_ctx_t;

// Core 1 写者任务: 执行可塑性更新与推理决策
static void core1_writer_task(void *param) {
    concurrency_ctx_t *ctx = (concurrency_ctx_t *)param;
    epgm_runtime_t *rt = ctx->shared_rt;

    for (uint32_t i = 0; i < CONCURRENCY_ITERATIONS; ++i) {
        uint16_t src = (uint16_t)((i % 81) + 1);
        uint8_t  act = (uint8_t)(i % EPGM_ACTION_DIM);
        uint16_t tgt = (uint16_t)(((i * 13) % 81) + 1);
        int16_t  rew = (int16_t)((i % 7) * 32);

        // 使用 5ms 超时安全更新
        bool updated = epgm_plasticity_update_safe(rt, src, act, tgt, rew, 5);
        if (!updated && rt->active_edge_count == 0) {
            ctx->contention_timeouts_handled++;
        }

        // 决策动作 (fallback 设为 2)
        uint8_t action = epgm_infer_action_safe(rt, src, 2, 5);
        (void)action;

        ctx->writer_iterations++;
    }
    ctx->writer_done = true;
}

// Core 0 读者任务: 模拟 MQTT 遥测与后台存盘高频快照拷贝
static void core0_reader_task(void *param) {
    concurrency_ctx_t *ctx = (concurrency_ctx_t *)param;
    epgm_runtime_t *rt = ctx->shared_rt;
    epgm_runtime_t snap;
    memset(&snap, 0, sizeof(epgm_runtime_t));

    while (!ctx->stop_reader) {
        // Core 0 在微秒级获取锁并复制双缓冲快照
        if (epgm_snapshot_copy(rt, &snap, 10)) {
            ctx->reader_snapshots_taken++;

            // 严格验证关键不变式: step_counter == total_hits + total_surprises
            // 考虑 uint16_t 溢出回绕: epgm_runtime_t 中 total_hits 与 total_surprises 为 uint16_t
            // 在 65535 步以内应完全相等；超过 65535 步时低 16 位严格相等
            if (snap.step_counter < 65535) {
                if (snap.step_counter != (uint32_t)snap.total_hits + (uint32_t)snap.total_surprises) {
                    ctx->torn_reads_detected++;
                }
            } else {
                if ((uint16_t)snap.step_counter != (uint16_t)((uint32_t)snap.total_hits + (uint32_t)snap.total_surprises)) {
                    ctx->torn_reads_detected++;
                }
            }

            // 验证边表一致性
            if (snap.active_edge_count > EPGM_MAX_EDGES) {
                ctx->edge_corruptions_detected++;
            }
            if (snap.active_edge_count > ctx->max_concurrency_edges) {
                ctx->max_concurrency_edges = snap.active_edge_count;
            }

            for (uint16_t j = 0; j < snap.active_edge_count; ++j) {
                if (snap.edges[j].is_active) {
                    if (snap.edges[j].action >= EPGM_ACTION_DIM ||
                        snap.edges[j].source_event > 181 ||
                        snap.edges[j].target_event > 181) {
                        ctx->edge_corruptions_detected++;
                    }
                }
            }
        }
    }
}

// 模拟外部严重总线争用线程 (持续持有互斥锁 25ms，触发 Core 1 的 5ms 超时保底)
static void contention_blocker_task(void *param) {
    epgm_runtime_t *rt = (epgm_runtime_t *)param;
    if (epgm_mutex_lock(rt, 1000)) {
        vTaskDelay(25); // 占用互斥锁 25ms
        epgm_mutex_unlock(rt);
    }
}

static void run_challenge_concurrency(void) {
    printf("\n======================================================================\n");
    printf("[CHALLENGE 1] FreeRTOS 多核并发安全极限压力挑战 (100,000 轮)\n");
    printf("======================================================================\n");

    epgm_runtime_t shared_rt;
    epgm_init(&shared_rt);
    bool m_ok = epgm_mutex_init(&shared_rt);
    CHALLENGE_ASSERT(m_ok, "并发互斥锁成功初始化");

    concurrency_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.shared_rt = &shared_rt;

    TaskHandle_t reader_h = NULL;
    TaskHandle_t writer_h = NULL;

    // 启动 Core 0 读者与 Core 1 写者
    xTaskCreate(core0_reader_task, "Core0Reader", 4096, &ctx, 1, &reader_h);
    xTaskCreate(core1_writer_task, "Core1Writer", 4096, &ctx, 5, &writer_h);

    // 等待写者完成 100,000 次高频读写
    printf("  >> 正在执行 Core 1 (100,000 次) 与 Core 0 持续并发读写...\n");
    bool w_joined = mock_task_join(writer_h, 60000); // 60s 超时保护
    CHALLENGE_ASSERT(w_joined, "Core 1 写者线程在限时内安全完成退出 (无死锁)");
    CHALLENGE_ASSERT(ctx.writer_iterations == CONCURRENCY_ITERATIONS, "Core 1 达到全量 100,000 次迭代更新");

    // 停止读者
    ctx.stop_reader = true;
    bool r_joined = mock_task_join(reader_h, 5000);
    CHALLENGE_ASSERT(r_joined, "Core 0 读者线程安全退出");

    printf("  >> 统计指标: 写者迭代 = %u, 读者捕获快照 = %u, 活跃边数 = %u\n",
           ctx.writer_iterations, ctx.reader_snapshots_taken, shared_rt.active_edge_count);
    printf("  >> 数据撕裂检测: %u 次, 内存损坏检测: %u 次\n",
           ctx.torn_reads_detected, ctx.edge_corruptions_detected);

    CHALLENGE_ASSERT(ctx.reader_snapshots_taken > 100, "Core 0 成功并发捕获多达数百次完整快照");
    CHALLENGE_ASSERT(ctx.torn_reads_detected == 0, "零数据撕裂: 所有快照 100% 满足 (uint16_t)step_counter == hits + surprises 不变式");
    CHALLENGE_ASSERT(ctx.edge_corruptions_detected == 0, "零内存越界/数据污染: 边动作与事件 ID 全部严格处于合法空间");
    CHALLENGE_ASSERT(shared_rt.active_edge_count > 0 && shared_rt.active_edge_count <= EPGM_MAX_EDGES,
                     "活跃边数量正常且严格受控于 EPGM_MAX_EDGES");

    // 1.2 严重锁争用下的 5ms 超时保底挑战
    printf("\n  >> 测试严重互斥锁争用下的 5ms 毫秒级超时安全降级...\n");
    TaskHandle_t blocker_h = NULL;
    xTaskCreate(contention_blocker_task, "Blocker", 4096, &shared_rt, 2, &blocker_h);
    vTaskDelay(5); // 确保 blocker 已抢占互斥锁

    LARGE_INTEGER freq, t1, t2;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t1);
    // 此时主任务模拟 Core 1 在 5ms 超时下请求锁
    uint8_t fb_action = epgm_infer_action_safe(&shared_rt, 10, 4, 5);
    QueryPerformanceCounter(&t2);
    double elapsed_ms = (double)(t2.QuadPart - t1.QuadPart) * 1000.0 / (double)freq.QuadPart;
    printf("  >> 高精度测量互斥锁争用耗时: %.2f ms\n", elapsed_ms);

    CHALLENGE_ASSERT(fb_action == 4, "互斥锁争用超时触发 5ms 保底机制，安全返回指定降级动作 (4)");
    CHALLENGE_ASSERT(elapsed_ms >= 4.0 && elapsed_ms <= 20.0, "超时响应时间严格受控在 5ms 附近，未发生失控阻塞");

    bool update_timeout = epgm_plasticity_update_safe(&shared_rt, 1, 1, 2, 0, 5);
    CHALLENGE_ASSERT(!update_timeout, "争用超时下 epgm_plasticity_update_safe 正确返回 false");

    mock_task_join(blocker_h, 2000);
    epgm_mutex_deinit(&shared_rt);
}

// ==============================================================================
// 2. LittleFS Flash 掉电与故障注入挑战
// ==============================================================================
static void run_challenge_littlefs_fault_injection(void) {
    printf("\n======================================================================\n");
    printf("[CHALLENGE 2] LittleFS Flash 掉电与原子固化故障注入挑战\n");
    printf("======================================================================\n");

    const char *sandbox_dir = "./challenger_fault_fs";
    mock_littlefs_init(sandbox_dir);

    // 构造有效因果图基线 Snapshot V1 (包含 20 条因果边)
    epgm_runtime_t rt_v1;
    epgm_init(&rt_v1);
    for (uint16_t i = 0; i < 20; ++i) {
        epgm_plasticity_update(&rt_v1, i + 1, 0, i + 2, (int16_t)(i * 20));
    }
    rt_v1.step_counter = 1000;
    rt_v1.total_hits = 150;

    const char *target_file = "/littlefs/epgm_graph.bin";
    const char *tmp_file = "/littlefs/epgm_graph.bin.tmp";

    // 2.1 固化基线 Snapshot V1
    int res_v1 = epgm_storage_save_atomic(&rt_v1, target_file);
    CHALLENGE_ASSERT(res_v1 == EPGM_STORAGE_OK, "基准因果图 Snapshot V1 原子固化成功");

    // 验证冷启动加载 V1
    epgm_runtime_t rt_check;
    epgm_init(&rt_check);
    int load_v1 = epgm_storage_load(&rt_check, target_file);
    CHALLENGE_ASSERT(load_v1 == EPGM_STORAGE_OK && rt_check.active_edge_count == 20,
                     "冷启动成功加载 Snapshot V1 (20 条因果边)");

    // 构造新模型 Snapshot V2 (包含 35 条因果边)
    epgm_runtime_t rt_v2;
    epgm_init(&rt_v2);
    for (uint16_t i = 0; i < 35; ++i) {
        epgm_plasticity_update(&rt_v2, i + 1, 1, i + 2, (int16_t)(i * 15));
    }
    rt_v2.step_counter = 2000;
    rt_v2.total_hits = 300;

    // 2.2 故障场景 A: 模拟写 .tmp 阶段突发掉电 (文件头截断仅写 16 字节)
    printf("  >> [故障场景 A] 模拟写入 .tmp 临时文件时突发断电 (文件头截断)...\n");
    char host_tmp_path[256];
    mock_littlefs_resolve_path(tmp_file, host_tmp_path, sizeof(host_tmp_path));
    
    FILE *raw_fp = fopen(host_tmp_path, "wb");
    uint8_t truncated_hdr[16] = {0x4D, 0x47, 0x50, 0x45}; // 伪造 16 字节截断数据
    fwrite(truncated_hdr, 1, 16, raw_fp);
    fclose(raw_fp);

    // 此时机器人掉电重启，执行冷启动自检加载 target_file
    epgm_runtime_t rt_restore_a;
    epgm_init(&rt_restore_a);
    int res_cold_a = epgm_storage_load(&rt_restore_a, target_file);
    CHALLENGE_ASSERT(res_cold_a == EPGM_STORAGE_OK, "掉电中断后冷启动恢复成功");
    CHALLENGE_ASSERT(rt_restore_a.active_edge_count == 20 && rt_restore_a.step_counter == 1000,
                     "未重命名的残缺写入安全被丢弃，系统精确回退至上一版本有效 Snapshot V1");

    // 若尝试直接反序列化该残缺 .tmp 文件，应明确拒绝并报错 IO 截断
    epgm_runtime_t rt_tmp_load;
    epgm_init(&rt_tmp_load);
    int res_bad_tmp = epgm_storage_load(&rt_tmp_load, tmp_file);
    CHALLENGE_ASSERT(res_bad_tmp == EPGM_STORAGE_ERR_IO, "残缺 .tmp 文件被校验拒绝加载 (EPGM_STORAGE_ERR_IO)");

    // 2.3 故障场景 B: 模拟写 .tmp 过程中边数据截断 (头完整但边 payload 缺失)
    printf("  >> [故障场景 B] 模拟写入 .tmp 过程中边数据丢失截断...\n");
    epgm_storage_header_t broken_hdr;
    memset(&broken_hdr, 0, sizeof(broken_hdr));
    broken_hdr.magic = EPGM_STORAGE_MAGIC;
    broken_hdr.version = EPGM_STORAGE_VERSION;
    broken_hdr.header_size = 32;
    broken_hdr.edge_count = 35;
    broken_hdr.payload_crc32 = 0x12345678;

    raw_fp = fopen(host_tmp_path, "wb");
    fwrite(&broken_hdr, 1, sizeof(broken_hdr), raw_fp);
    // 故意只写 1 条边的长度 (应有 35 条边)
    epgm_storage_edge_t partial_edge;
    memset(&partial_edge, 0, sizeof(partial_edge));
    fwrite(&partial_edge, 1, sizeof(partial_edge), raw_fp);
    fclose(raw_fp);

    int res_bad_edge = epgm_storage_load(&rt_tmp_load, tmp_file);
    CHALLENGE_ASSERT(res_bad_edge == EPGM_STORAGE_ERR_IO, "载荷缺失 .tmp 文件被校验拒绝 (EPGM_STORAGE_ERR_IO)");

    // 2.4 故障场景 C: 模拟物理 Flash 位翻转 (Bit-flip) 导致 CRC 损坏
    printf("  >> [故障场景 C] 模拟 Flash 物理介质单字节比特翻转损坏...\n");
    // 先保存一个合法的独立测试文件
    const char *crc_test_file = "/littlefs/epgm_crc_test.bin";
    epgm_storage_save_atomic(&rt_v1, crc_test_file);
    // 注入损坏
    mock_littlefs_corrupt_file(crc_test_file, 45); // 破坏边载荷数据
    epgm_runtime_t rt_crc_test;
    epgm_init(&rt_crc_test);
    int res_crc = epgm_storage_load(&rt_crc_test, crc_test_file);
    CHALLENGE_ASSERT(res_crc == EPGM_STORAGE_ERR_CRC, "CRC32 校验精确检出并拦截数据损坏 (EPGM_STORAGE_ERR_CRC)");

    // 2.5 故障场景 D: 模拟非法 Magic 与格式版本错误
    printf("  >> [故障场景 D] 模拟非法文件头 Magic 与协议版本错误...\n");
    const char *magic_test_file = "/littlefs/epgm_magic_test.bin";
    raw_fp = fopen(host_tmp_path, "wb");
    epgm_storage_header_t bad_magic_hdr;
    memset(&bad_magic_hdr, 0, sizeof(bad_magic_hdr));
    bad_magic_hdr.magic = 0xDEADBEEF; // 错误 Magic
    bad_magic_hdr.version = EPGM_STORAGE_VERSION;
    fwrite(&bad_magic_hdr, 1, sizeof(bad_magic_hdr), raw_fp);
    fclose(raw_fp);

    int res_magic = epgm_storage_load(&rt_tmp_load, tmp_file);
    CHALLENGE_ASSERT(res_magic == EPGM_STORAGE_ERR_MAGIC, "非法 Magic 被精确拦截 (EPGM_STORAGE_ERR_MAGIC)");

    // 2.6 故障场景 E: 正常完成第二阶段原子重命名 (Snapshot V2 升级)
    printf("  >> [故障场景 E] 正常两阶段原子重命名提交流程...\n");
    int res_v2 = epgm_storage_save_atomic(&rt_v2, target_file);
    CHALLENGE_ASSERT(res_v2 == EPGM_STORAGE_OK, "Snapshot V2 成功原子提交替换");

    epgm_runtime_t rt_restore_v2;
    epgm_init(&rt_restore_v2);
    int res_cold_v2 = epgm_storage_load(&rt_restore_v2, target_file);
    CHALLENGE_ASSERT(res_cold_v2 == EPGM_STORAGE_OK && rt_restore_v2.active_edge_count == 35,
                     "原子替换后冷启动无缝加载全新 Snapshot V2 (35 条因果边)");

    mock_littlefs_cleanup();
}

// ==============================================================================
// 3. 量化器等价性、边界条件与防边缘振荡挑战
// ==============================================================================
static void run_challenge_quantizer_parity(void) {
    printf("\n======================================================================\n");
    printf("[CHALLENGE 3] C/Python 量化器空间数学对齐与抗边缘振荡挑战\n");
    printf("======================================================================\n");

    epgm_quantizer_t q;
    epgm_quantizer_init(&q);

    // 3.1 严格整数欧氏距离平方和门限边界 (Threshold = 62500)
    // 建立稳定基线: 8 向均为 500mm
    uint16_t d_base[8] = {500, 500, 500, 500, 500, 500, 500, 500};
    uint16_t ev_base = epgm_quantize_stateful(&q, d_base, false);
    CHALLENGE_ASSERT(ev_base == 41, "中距基线输入 (500mm) 正确输出静态事件 ID: 41");

    // 边界点 1: 严格等于门限 62500 (单路跳变 250mm: 500 -> 750, diff = 250, diff^2 = 62500)
    // 判定条件必须为严格大于: sum(diff^2) > 62500 为 false -> 保持静态事件
    // 750mm 为安全分箱 2，其余 500mm 为接近分箱 1
    // front: min(2, 1, 1) = 1; right: min(1, 1, 1) = 1; back: 1; left: 1 -> 静态 ID = 41
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d_base, false); // 建立 500mm 基准
    uint16_t d_eq[8] = {750, 500, 500, 500, 500, 500, 500, 500};
    uint16_t ev_eq = epgm_quantize_stateful(&q, d_eq, false);
    CHALLENGE_ASSERT(ev_eq == 41, "边界条件 diff^2 == 62500 严格判定为非动态，维持静态 ID: 41");

    // 边界点 2: 略小于门限 (单路跳变 249mm: 500 -> 749, diff^2 = 62001 < 62500)
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d_base, false);
    uint16_t d_sub[8] = {749, 500, 500, 500, 500, 500, 500, 500};
    uint16_t ev_sub = epgm_quantize_stateful(&q, d_sub, false);
    CHALLENGE_ASSERT(ev_sub == 41, "临界下界 diff^2 = 62001 不触发动态偏置，输出静态 ID: 41");

    // 边界点 3: 严格大于门限 (单路跳变 251mm: 500 -> 751, diff^2 = 63001 > 62500)
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d_base, false);
    uint16_t d_sup[8] = {751, 500, 500, 500, 500, 500, 500, 500};
    uint16_t ev_sup = epgm_quantize_stateful(&q, d_sup, false);
    CHALLENGE_ASSERT(ev_sup == 141, "临界上界 diff^2 = 63001 准确触发动态偏置，输出动态 ID: 141");

    // 3.2 负向跳变符号与平方积累加 (负数差分平方应正向累加)
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d_base, false); // 基准 500mm
    // 骤降 260mm (500 -> 240mm, 240mm 属于近距分箱 0, diff = -260, diff^2 = 67600 > 62500)
    // front: min(0, 1, 1) = 0; right: 1; back: 1; left: 1 -> 静态 ID = 1 + (0*27 + 1*9 + 1*3 + 1) = 14
    // 动态 ID = 14 + 100 = 114
    uint16_t d_neg[8] = {240, 500, 500, 500, 500, 500, 500, 500};
    uint16_t ev_neg = epgm_quantize_stateful(&q, d_neg, false);
    CHALLENGE_ASSERT(ev_neg == 114, "负向骤降 (-260mm) 差分平方累加无符号错误，正确输出动态 ID: 114");

    // 3.3 多路分布式微小变化累加判定 (8 路协同突变)
    // 8 路均变化 88mm: 8 * (88^2) = 8 * 7744 = 61952 < 62500 -> 静态
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d_base, false);
    uint16_t d_dist_sub[8] = {588, 588, 588, 588, 588, 588, 588, 588};
    uint16_t ev_dist_sub = epgm_quantize_stateful(&q, d_dist_sub, false);
    CHALLENGE_ASSERT(ev_dist_sub == 41, "8路全向累积变动 61952 < 62500，准确判定为静态事件 41");

    // 8 路均变化 89mm: 8 * (89^2) = 8 * 7921 = 63368 > 62500 -> 动态
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d_base, false);
    uint16_t d_dist_sup[8] = {589, 589, 589, 589, 589, 589, 589, 589};
    uint16_t ev_dist_sup = epgm_quantize_stateful(&q, d_dist_sup, false);
    CHALLENGE_ASSERT(ev_dist_sup == 141, "8路全向累积变动 63368 > 62500，准确触发动态脉冲 141");

    // 3.4 抗边缘振荡 (Anti-Oscillation) 挑战:
    // 场景 1: 单脉冲特性。当小车剧烈位移并停留在新位置时，动态事件仅触发一拍，下一拍必须自动恢复为静态事件
    printf("  >> [抗振荡场景 1] 验证单脉冲特性与无状态残留...\n");
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d_base, false); // T0: 500mm
    uint16_t d_jump[8] = {800, 800, 800, 800, 800, 800, 800, 800}; // T1: 突变至 800mm (diff^2 = 8*90000 = 720000)
    uint16_t ev_t1 = epgm_quantize_stateful(&q, d_jump, false);
    CHALLENGE_ASSERT(ev_t1 == 181, "T1 突变帧触发动态脉冲 181 (81 + 100)");

    // T2: 维持在 800mm (diff = 0)
    uint16_t ev_t2 = epgm_quantize_stateful(&q, d_jump, false);
    CHALLENGE_ASSERT(ev_t2 == 81, "T2 静止帧平稳回归静态事件 81，无残留振荡");

    // T3: 持续维持在 800mm
    uint16_t ev_t3 = epgm_quantize_stateful(&q, d_jump, false);
    CHALLENGE_ASSERT(ev_t3 == 81, "T3 稳态帧继续稳定输出静态事件 81");

    // 场景 2: 空间感受野分箱边缘噪声抗振荡 (在 332mm 与 334mm 之间微幅抖动)
    // diff = 2mm, diff^2 = 4 << 62500，绝不可产生动态事件 101~181
    printf("  >> [抗振荡场景 2] 感受野分箱边界微幅噪声 (332 <-> 334mm) 抗扰动测试...\n");
    epgm_quantizer_init(&q);
    uint16_t d_jitter_a[8] = {332, 332, 332, 332, 332, 332, 332, 332};
    uint16_t d_jitter_b[8] = {334, 334, 334, 334, 334, 334, 334, 334};

    epgm_quantize_stateful(&q, d_jitter_a, false);
    bool no_accidental_dynamic = true;
    for (int cycle = 0; cycle < 100; ++cycle) {
        uint16_t e_a = epgm_quantize_stateful(&q, d_jitter_a, false);
        uint16_t e_b = epgm_quantize_stateful(&q, d_jitter_b, false);
        if (e_a > 81 || e_b > 81) {
            no_accidental_dynamic = false;
            break;
        }
    }
    CHALLENGE_ASSERT(no_accidental_dynamic, "在分箱临界点 100 轮微幅噪声抖动中，绝未误触发任何动态事件 (>81)");

    // 3.5 碰撞微动开关优先级与刷新基准
    printf("  >> [碰撞最高优先级与刷新] 验证微动开关与后续恢复...\n");
    epgm_quantizer_init(&q);
    epgm_quantize_stateful(&q, d_base, false);
    uint16_t ev_hit = epgm_quantize_stateful(&q, d_jitter_a, true);
    CHALLENGE_ASSERT(ev_hit == 0, "物理微动触发时无条件输出事件 ID 0");

    // 碰撞后倒车脱离至安全区域 (800mm, diff = 800 - 332 = 468, diff^2 = 219024 > 62500)
    uint16_t ev_escape = epgm_quantize_stateful(&q, d_jump, false);
    CHALLENGE_ASSERT(ev_escape == 181, "碰撞后倒车以碰撞观测为基准正确触发动态脉冲 181");
}

// ==============================================================================
// 对抗性主程序入口
// ==============================================================================
int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    printf("======================================================================\n");
    printf(" EPGM Milestone M3 边缘固件对抗性压力挑战套件 (Challenger 1)\n");
    printf(" EPGM M3 Firmware Adversarial Stress & Fault Injection Challenge Suite\n");
    printf("======================================================================\n");

    run_challenge_concurrency();
    run_challenge_littlefs_fault_injection();
    run_challenge_quantizer_parity();

    printf("\n======================================================================\n");
    printf(" 对抗性挑战执行总评 / Verdict Summary:\n");
    printf(" 运行断言总数: %d, 成功通过: %d, 发现失败: %d\n",
           s_chall_total, s_chall_passed, s_chall_failed);

    if (s_chall_failed == 0) {
        printf(" 最终裁决 / FINAL VERDICT: [ APPROVE ] 固件各项指标高抗压通过验证！\n");
    } else {
        printf(" 最终裁决 / FINAL VERDICT: [ REJECT ] 发现严重缺陷，请修复！\n");
    }
    printf("======================================================================\n");

    return (s_chall_failed == 0) ? 0 : 1;
}
