/**
 * @file epgm_core.c
 * @brief [ZH] ESP32-S3 原生 C 语言超轻量 EPGM 运行时引擎实现。
 *        [EN] Native C lightweight EPGM runtime engine implementation for ESP32-S3.
 */

#include "epgm_core.h"
#include <string.h>

// ==============================================================================
// 内部辅助函数: 空间感受野离散分箱与象限聚合
// ==============================================================================

static inline uint8_t quantize_single_ray(uint16_t d_mm) {
    if (d_mm < 333) {
        return 0; // 危险近距
    } else if (d_mm < 667) {
        return 1; // 接近中距
    } else {
        return 2; // 开阔远距
    }
}

static inline uint8_t min3(uint8_t a, uint8_t b, uint8_t c) {
    uint8_t m = a;
    if (b < m) m = b;
    if (c < m) m = c;
    return m;
}

static uint16_t compute_spatial_event_id(const uint16_t distances[8]) {
    uint8_t bin[8];
    for (int i = 0; i < 8; ++i) {
        bin[i] = quantize_single_ray(distances[i]);
    }

    // 象限显著聚合:
    // 前方: Ray 0, 1, 7; 右方: Ray 1, 2, 3; 后方: Ray 3, 4, 5; 左方: Ray 5, 6, 7
    uint8_t front = min3(bin[0], bin[1], bin[7]);
    uint8_t right = min3(bin[1], bin[2], bin[3]);
    uint8_t back  = min3(bin[3], bin[4], bin[5]);
    uint8_t left  = min3(bin[5], bin[6], bin[7]);

    return (uint16_t)(1 + (front * 27 + right * 9 + back * 3 + left));
}

// ==============================================================================
// 基础生命周期实现
// ==============================================================================

void epgm_init(epgm_runtime_t *rt) {
    if (!rt) return;
    memset(rt, 0, sizeof(epgm_runtime_t));
}

// ==============================================================================
// 量化器实现
// ==============================================================================

void epgm_quantizer_init(epgm_quantizer_t *q) {
    if (!q) return;
    memset(q, 0, sizeof(epgm_quantizer_t));
    q->has_last_observation = false;
}

void epgm_quantizer_reset(epgm_quantizer_t *q) {
    if (!q) return;
    q->has_last_observation = false;
}

uint16_t epgm_quantize(const uint16_t distances[8], bool bumper_hit) {
    if (bumper_hit) {
        return 0; // 事件 0 预留为绝对碰撞事件
    }
    return compute_spatial_event_id(distances);
}

uint16_t epgm_quantize_stateful(
    epgm_quantizer_t *q,
    const uint16_t distances[8],
    bool bumper_hit
) {
    // 1. 物理微动开关触发: 优先级最高
    if (bumper_hit) {
        if (q) {
            for (int i = 0; i < 8; ++i) {
                q->last_distances[i] = distances[i];
            }
            q->has_last_observation = true;
        }
        return 0;
    }

    // 2. 空间局部感受野分箱 (1 ~ 81)
    uint16_t spatial_id = compute_spatial_event_id(distances);

    // 3. 一阶差分动态脉冲检测 (欧氏距离平方和比较: sum(diff^2) > 62500, 等价于 ||Delta||_2 > 250mm)
    bool is_dynamic = false;
    if (q && q->has_last_observation) {
        uint64_t sum_sq_diff = 0;
        for (int i = 0; i < 8; ++i) {
            int32_t diff = (int32_t)distances[i] - (int32_t)q->last_distances[i];
            sum_sq_diff += (uint64_t)(diff * diff);
        }

        if (sum_sq_diff > 62500ULL) {
            is_dynamic = true;
        }
    }

    // 4. 更新历史观测缓存
    if (q) {
        for (int i = 0; i < 8; ++i) {
            q->last_distances[i] = distances[i];
        }
        q->has_last_observation = true;
    }

    // 5. 动态事件偏置 (+100)
    if (is_dynamic) {
        return (uint16_t)(spatial_id + 100);
    }
    return spatial_id;
}

// ==============================================================================
// 推理与突触可塑性更新
// ==============================================================================

uint8_t epgm_infer_action(const epgm_runtime_t *rt, uint16_t current_event) {
    if (!rt) return 0;

    int32_t best_score = -999999;
    uint8_t best_action = 1; // 默认尝试前进

    for (uint8_t a = 0; a < EPGM_ACTION_DIM; ++a) {
        int32_t score = 0;
        bool found = false;
        bool has_collision_hazard = false;
        int32_t total_reward_q8 = 0;
        uint32_t match_count = 0;
        uint32_t min_obs = 0xFFFFFFFF;

        // [ZH] 遍历所有活跃因果边，消除早退遮蔽漏洞 (防范 Finding 1.1 缺陷)
        // [EN] Traverse all active edges to prevent early-break masking of collision hazard
        for (uint16_t i = 0; i < rt->active_edge_count; ++i) {
            const epgm_edge_entry_t *edge = &rt->edges[i];
            if (edge->is_active && edge->source_event == current_event && edge->action == a) {
                found = true;
                match_count++;
                total_reward_q8 += (int32_t)edge->expected_reward_q8;
                if (edge->observation_count < min_obs) {
                    min_obs = edge->observation_count;
                }
                // 若任意同源因果分支指向碰撞事件 (target == 0)，置位危险标志
                if (edge->target_event == 0) {
                    has_collision_hazard = true;
                }
            }
        }

        if (found) {
            // 计算匹配分支的平均预期回报
            score = total_reward_q8 / (int32_t)match_count;

            // 若存在任何碰撞危险分支，执行硬安全扣分/遮蔽惩罚 (-10.0 in Q8.8)
            if (has_collision_hazard) {
                score -= (10 << 8); // -2560
            }

            // 好奇心探索奖励：以最小观测次数激励充分探索
            int32_t curiosity_bonus = (256 * 2) / (min_obs + 1);
            score += curiosity_bonus;
        } else {
            // 若动作为完全未探索状态，赋予高好奇心激励
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

    // 1. 查找是否存在匹配的边
    for (uint16_t i = 0; i < rt->active_edge_count; ++i) {
        epgm_edge_entry_t *edge = &rt->edges[i];
        if (edge->is_active && edge->source_event == source_event && edge->action == action) {
            if (edge->target_event == target_event) {
                // [ZH] 预测命中 (Hit) -> 强化边置信度与平滑回报
                rt->total_hits++;
                edge->observation_count++;
                if (edge->confidence < 250) edge->confidence += 5;

                // 平滑更新回报 Q8.8: R = R + 0.25 * (r - R)，带四舍五入
                int32_t diff = (int32_t)reward_q8 - (int32_t)edge->expected_reward_q8;
                int32_t delta = (diff >= 0) ? ((diff + 2) >> 2) : ((diff - 2) >> 2);
                edge->expected_reward_q8 += (int16_t)delta;
                return true;
            }
        }
    }

    // 2. 预测失配/缺失 (Surprise) -> 拓扑生枝 (Budding)
    rt->total_surprises++;

    if (rt->active_edge_count < EPGM_MAX_EDGES) {
        // 容量未满，直接追加新边
        epgm_edge_entry_t *new_edge = &rt->edges[rt->active_edge_count++];
        new_edge->source_event = source_event;
        new_edge->action = action;
        new_edge->target_event = target_event;
        new_edge->expected_reward_q8 = reward_q8;
        new_edge->observation_count = 1;
        new_edge->confidence = 100;
        new_edge->is_active = true;
    } else {
        // [ZH] 容量已满 (512条边)，执行淘汰置换 (淘汰 observation_count 最少且 confidence 最低的边)
        uint16_t victim_idx = 0;
        uint32_t lowest_score = 0xFFFFFFFFUL;

        for (uint16_t i = 0; i < EPGM_MAX_EDGES; ++i) {
            if (!rt->edges[i].is_active) {
                victim_idx = i;
                break;
            }
            uint32_t score = ((uint32_t)rt->edges[i].observation_count << 8) + rt->edges[i].confidence;
            if (score < lowest_score) {
                lowest_score = score;
                victim_idx = i;
            }
        }

        epgm_edge_entry_t *victim = &rt->edges[victim_idx];
        victim->source_event = source_event;
        victim->action = action;
        victim->target_event = target_event;
        victim->expected_reward_q8 = reward_q8;
        victim->observation_count = 1;
        victim->confidence = 100;
        victim->is_active = true;
    }

    return false;
}

// ==============================================================================
// FreeRTOS 多核并发互斥与双缓冲快照实现
// ==============================================================================

bool epgm_mutex_init(epgm_runtime_t *rt) {
    if (!rt) return false;
    if (rt->mutex != NULL) {
        return true; // 已初始化
    }
    rt->mutex = xSemaphoreCreateMutex();
    return (rt->mutex != NULL);
}

void epgm_mutex_deinit(epgm_runtime_t *rt) {
    if (!rt || !rt->mutex) return;
    vSemaphoreDelete(rt->mutex);
    rt->mutex = NULL;
}

bool epgm_mutex_lock(epgm_runtime_t *rt, uint32_t timeout_ms) {
    if (!rt || !rt->mutex) return false;
    return (xSemaphoreTake(rt->mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
}

void epgm_mutex_unlock(epgm_runtime_t *rt) {
    if (!rt || !rt->mutex) return;
    xSemaphoreGive(rt->mutex);
}

bool epgm_snapshot_copy(const epgm_runtime_t *src, epgm_runtime_t *dst, uint32_t timeout_ms) {
    if (!src || !dst || !src->mutex) return false;

    if (xSemaphoreTake(src->mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        // 保存目标结构体现有的互斥锁句柄
        SemaphoreHandle_t dst_mutex = dst->mutex;
        memcpy(dst, src, sizeof(epgm_runtime_t));
        dst->mutex = dst_mutex;
        xSemaphoreGive(src->mutex);
        return true;
    }
    return false;
}

uint8_t epgm_infer_action_safe(
    epgm_runtime_t *rt,
    uint16_t current_event,
    uint8_t fallback_action,
    uint32_t timeout_ms
) {
    if (!rt) return fallback_action;
    if (rt->mutex) {
        if (xSemaphoreTake(rt->mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            uint8_t act = epgm_infer_action(rt, current_event);
            xSemaphoreGive(rt->mutex);
            return act;
        } else {
            // 超时降级保底，保障 20ms 硬实时控制闭环不卡死
            return fallback_action;
        }
    }
    return epgm_infer_action(rt, current_event);
}

bool epgm_plasticity_update_safe(
    epgm_runtime_t *rt,
    uint16_t source_event,
    uint8_t action,
    uint16_t target_event,
    int16_t reward_q8,
    uint32_t timeout_ms
) {
    if (!rt) return false;
    if (rt->mutex) {
        if (xSemaphoreTake(rt->mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            bool res = epgm_plasticity_update(rt, source_event, action, target_event, reward_q8);
            xSemaphoreGive(rt->mutex);
            return res;
        }
        return false;
    }
    return epgm_plasticity_update(rt, source_event, action, target_event, reward_q8);
}
