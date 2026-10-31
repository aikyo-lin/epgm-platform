/**
 * @file epgm_core.c
 * @brief [ZH] ESP32-S3 原生 C 语言超轻量 EPGM 运行时引擎实现。
 *        [EN] Native C lightweight EPGM runtime engine implementation for ESP32-S3.
 */

#include "epgm_core.h"
#include <string.h>

void epgm_init(epgm_runtime_t *rt) {
    if (!rt) return;
    memset(rt, 0, sizeof(epgm_runtime_t));
}

uint16_t epgm_quantize(const uint16_t distances[8], bool bumper_hit) {
    // 1. 物理微动开关触发: 优先级最高
    if (bumper_hit) {
        return 0; // 事件 0 预留为碰撞事件
    }

    // 2. 距离分箱: 近距离危险(<300mm)=0, 中距离接近(300~700mm)=1, 远距离安全(>700mm)=2
    uint8_t bin[8];
    for (int i = 0; i < 8; ++i) {
        if (distances[i] < 300) {
            bin[i] = 0;
        } else if (distances[i] < 700) {
            bin[i] = 1;
        } else {
            bin[i] = 2;
        }
    }

    // 3. 象限聚合 (前、右、后、左)
    uint8_t front = (bin[0] < bin[1]) ? ((bin[0] < bin[7]) ? bin[0] : bin[7]) : ((bin[1] < bin[7]) ? bin[1] : bin[7]);
    uint8_t right = (bin[1] < bin[2]) ? ((bin[1] < bin[3]) ? bin[1] : bin[3]) : ((bin[2] < bin[3]) ? bin[2] : bin[3]);
    uint8_t back  = (bin[3] < bin[4]) ? ((bin[3] < bin[5]) ? bin[3] : bin[5]) : ((bin[4] < bin[5]) ? bin[4] : bin[5]);
    uint8_t left  = (bin[5] < bin[6]) ? ((bin[5] < bin[7]) ? bin[5] : bin[7]) : ((bin[6] < bin[7]) ? bin[6] : bin[7]);

    return (uint16_t)(1 + (front * 27 + right * 9 + back * 3 + left));
}

uint8_t epgm_infer_action(const epgm_runtime_t *rt, uint16_t current_event) {
    if (!rt) return 0;

    int32_t best_score = -999999;
    uint8_t best_action = 1; // 默认尝试前进

    for (uint8_t a = 0; a < EPGM_ACTION_DIM; ++a) {
        int32_t score = 0;
        bool found = false;

        // 检索匹配边
        for (uint16_t i = 0; i < rt->active_edge_count; ++i) {
            const epgm_edge_entry_t *edge = &rt->edges[i];
            if (edge->is_active && edge->source_event == current_event && edge->action == a) {
                found = true;
                score = (int32_t)edge->expected_reward_q8;
                // 若预测发生碰撞 (target == 0)，施加致命扣分
                if (edge->target_event == 0) {
                    score -= (10 << 8); // -10.0 in Q8.8
                }
                // 不确定性好奇心奖励: 观测越少奖励越高
                int32_t curiosity_bonus = (256 * 2) / (edge->observation_count + 1);
                score += curiosity_bonus;
                break;
            }
        }

        // 若该动作为完全未探索，赋予最高好奇心激励
        if (!found) {
            score = 512; // 相当于 +2.0 好奇心激励
        }

        if (score > best_score) {
            best_score = score;
            best_action = a;
        }
    }

    return best_action;
}

bool epgm_plasticity_update(
    epgm_runtime_t *rt,
    uint16_t source_event,
    uint8_t action,
    uint16_t target_event,
    int16_t reward_q8
) {
    if (!rt) return false;
    rt->step_counter++;

    // 1. 查找是否存在完全匹配的边
    for (uint16_t i = 0; i < rt->active_edge_count; ++i) {
        epgm_edge_entry_t *edge = &rt->edges[i];
        if (edge->is_active && edge->source_event == source_event && edge->action == action) {
            if (edge->target_event == target_event) {
                // [ZH] 预测命中 (Hit) -> 强化边
                rt->total_hits++;
                edge->observation_count++;
                if (edge->confidence < 250) edge->confidence += 5;
                // 平滑更新回报 Q8.8: R = R + 0.25 * (r - R)
                int32_t diff = (int32_t)reward_q8 - (int32_t)edge->expected_reward_q8;
                edge->expected_reward_q8 += (int16_t)(diff >> 2);
                return true;
            }
        }
    }

    // 2. 预测失配/缺失 (Surprise) -> 拓扑生枝 (Budding)
    rt->total_surprises++;
    if (rt->active_edge_count < EPGM_MAX_EDGES) {
        epgm_edge_entry_t *new_edge = &rt->edges[rt->active_edge_count++];
        new_edge->source_event = source_event;
        new_edge->action = action;
        new_edge->target_event = target_event;
        new_edge->expected_reward_q8 = reward_q8;
        new_edge->observation_count = 1;
        new_edge->confidence = 100;
        new_edge->is_active = true;
    }

    return false;
}
