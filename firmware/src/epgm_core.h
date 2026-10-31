/**
 * @file epgm_core.h
 * @brief [ZH] ESP32-S3 原生 C 语言超轻量 EPGM 运行时引擎头文件。
 *        [EN] Native C lightweight EPGM runtime engine header for ESP32-S3.
 */

#ifndef EPGM_CORE_H
#define EPGM_CORE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EPGM_MAX_NODES 128
#define EPGM_MAX_EDGES 512
#define EPGM_ACTION_DIM 5

/**
 * @brief [ZH] 单条因果边存储结构
 *        [EN] Single causal edge storage structure
 */
typedef struct {
    uint16_t source_event;
    uint8_t  action;
    uint16_t target_event;
    int16_t  expected_reward_q8;  ///< Q8.8 格式定点数 / Q8.8 format fixed-point
    uint16_t observation_count;
    uint8_t  confidence;
    bool     is_active;
} epgm_edge_entry_t;

/**
 * @brief [ZH] 嵌入式 EPGM 运行时上下文句柄
 *        [EN] Embedded EPGM runtime context handle
 */
typedef struct {
    epgm_edge_entry_t edges[EPGM_MAX_EDGES];
    uint16_t active_edge_count;
    uint32_t step_counter;
    uint16_t total_hits;
    uint16_t total_surprises;
} epgm_runtime_t;

void epgm_init(epgm_runtime_t *rt);

uint16_t epgm_quantize(const uint16_t distances[8], bool bumper_hit);

uint8_t epgm_infer_action(const epgm_runtime_t *rt, uint16_t current_event);

bool epgm_plasticity_update(
    epgm_runtime_t *rt,
    uint16_t source_event,
    uint8_t action,
    uint16_t target_event,
    int16_t reward_q8
);

#ifdef __cplusplus
}
#endif

#endif // EPGM_CORE_H
