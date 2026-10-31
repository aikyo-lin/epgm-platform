/**
 * @file test_epgm_c.c
 * @brief [ZH] ESP32-S3 原生 C 语言 EPGM 引擎单元测试验证程序。
 *        [EN] Native C EPGM engine standalone unit test verification.
 */

#include <stdio.h>
#include <assert.h>
#include "../src/epgm_core.h"

int main() {
    printf("==================================================\n");
    printf("[ZH] 运行 ESP32-S3 纯 C 语言版 EPGM 引擎单元测试...\n");
    printf("[EN] Running Native C EPGM Engine Unit Tests...\n");
    printf("==================================================\n");

    epgm_runtime_t rt;
    epgm_init(&rt);
    assert(rt.active_edge_count == 0);
    assert(rt.step_counter == 0);

    // 1. 测试量化器
    uint16_t dist_safe[8] = {900, 900, 900, 900, 900, 900, 900, 900};
    uint16_t e1 = epgm_quantize(dist_safe, false);
    assert(e1 > 0);
    printf("[PASS] 空间安全距离量化为非零事件 ID: %u\n", e1);

    // 碰撞量化
    uint16_t e_bump = epgm_quantize(dist_safe, true);
    assert(e_bump == 0);
    printf("[PASS] 碰撞微动开关触发正确映射为专属事件 0\n");

    // 2. 测试初始推理决策 (未探索时具有好奇心激励)
    uint8_t a1 = epgm_infer_action(&rt, e1);
    printf("[PASS] 初始状态推荐探索动作: %u\n", a1);

    // 3. 测试可塑性生枝 (Budding)
    bool is_hit = epgm_plasticity_update(&rt, e1, a1, 5, 256); // 回报 +1.0 (Q8.8=256)
    assert(is_hit == false); // 首次应该为 surprise (生枝)
    assert(rt.active_edge_count == 1);
    assert(rt.total_surprises == 1);
    printf("[PASS] 首次因果转移正确触发生枝 (Budding), 当前活跃边数: %u\n", rt.active_edge_count);

    // 4. 测试预测命中 (Hit)
    is_hit = epgm_plasticity_update(&rt, e1, a1, 5, 256);
    assert(is_hit == true);
    assert(rt.total_hits == 1);
    assert(rt.edges[0].observation_count == 2);
    printf("[PASS] 二次同向因果转移正确触发突触增强 (Hit), 观测样本计数: %u\n", rt.edges[0].observation_count);

    // 5. 测试碰撞惩罚规避推理
    // 若向事件 0 (碰撞) 添加一条大惩罚边
    epgm_plasticity_update(&rt, e1, 1, 0, -(10 << 8)); // 动作 1 撞墙
    // 现在重新推理，决策器必须主动避开动作 1！
    uint8_t safe_action = epgm_infer_action(&rt, e1);
    assert(safe_action != 1);
    printf("[PASS] 决策器成功利用因果预测边规避碰撞动作 1 (选择安全动作: %u)\n", safe_action);

    printf("==================================================\n");
    printf("[ZH] ESP32 纯 C 语言运行时全部 5 项测试 100%% 通过！\n");
    printf("[EN] All 5 native C EPGM tests PASSED successfully!\n");
    printf("==================================================\n");

    return 0;
}
