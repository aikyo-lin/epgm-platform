/**
 * @file epgm_core.h
 * @brief [ZH] ESP32-S3 原生 C 语言超轻量 EPGM 运行时引擎头文件。
 *        [EN] Native C lightweight EPGM runtime engine header for ESP32-S3.
 *
 * 核心升级与工程架构 / Core Enhancements:
 *  1. 节点容量扩容: EPGM_MAX_NODES 扩至 256，完整支持 101~181 动态事件 ID；
 *  2. 有状态差分量化器: 引入 epgm_quantizer_t，实现与 Python 端一致的动态欧氏突变检测；
 *  3. 多核 FreeRTOS 互斥保护: 引入 SemaphoreHandle_t 互斥锁与双缓冲内存快照 (RAM Snapshot)；
 *  4. 毫秒级超时保底: 5ms 获取超时自动回退至安全历史动作，避免控制闭环抖动。
 */

#ifndef EPGM_CORE_H
#define EPGM_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#if defined(ESP_PLATFORM) || (defined(ARDUINO) && !defined(EPGM_HOST_TEST))
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#else
#if __has_include("mock_freertos.h")
#include "mock_freertos.h"
#elif __has_include("../test/mock_freertos.h")
#include "../test/mock_freertos.h"
#else
typedef void *SemaphoreHandle_t;
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define EPGM_MAX_NODES   256   ///< 支持 0(碰撞) + 1~81(静态) + 101~181(动态)
#define EPGM_MAX_EDGES   512   ///< 固件最大容纳活跃因果边数
#define EPGM_ACTION_DIM  5     ///< 动作空间维度 (0:停, 1:前, 2:后, 3:左, 4:右)

/**
 * @brief [ZH] 单条因果边存储结构
 *        [EN] Single causal edge storage structure
 */
typedef struct {
    uint16_t source_event;        ///< 源事件 ID (0~181)
    uint8_t  action;              ///< 动作编号 (0~4)
    uint16_t target_event;        ///< 预测目标事件 ID (0~181)
    int16_t  expected_reward_q8;  ///< Q8.8 格式定点数预期回报 (如 256 表示 +1.0)
    uint16_t observation_count;   ///< 观测样本累计次数
    uint8_t  confidence;          ///< 置信度 (0~255)
    bool     is_active;           ///< 活跃使能标志
} epgm_edge_entry_t;

/**
 * @brief [ZH] 嵌入式 EPGM 运行时上下文句柄 (带多核并发互斥保护)
 *        [EN] Embedded EPGM runtime context handle with multicore concurrency protection
 */
typedef struct {
    epgm_edge_entry_t edges[EPGM_MAX_EDGES];  ///< 因果边数组
    uint16_t          active_edge_count;     ///< 当前活跃边计数
    uint32_t          step_counter;          ///< 累计步数计数器
    uint16_t          total_hits;            ///< 累计预测命中数
    uint16_t          total_surprises;       ///< 累计失配生枝数
    SemaphoreHandle_t mutex;                 ///< FreeRTOS 优先级继承互斥锁
} epgm_runtime_t;

/**
 * @brief [ZH] 有状态事件量化器上下文
 *        [EN] Stateful event quantizer context
 */
typedef struct {
    uint16_t last_distances[8];     ///< 上一时刻 8 向测距距离 (单位: 毫米 mm)
    bool     has_last_observation;  ///< 是否具有上一时刻的有效观测
} epgm_quantizer_t;

// ==============================================================================
// 基础生命周期与无状态量化 API
// ==============================================================================

/**
 * @brief 初始化 EPGM 运行时引擎
 * @param rt 运行时句柄指针
 */
void epgm_init(epgm_runtime_t *rt);

/**
 * @brief 无状态快速空间量化 (向后兼容，输出 0 或 1~81)
 * @param distances 8 向毫米距离
 * @param bumper_hit 微动碰撞状态
 * @return 0 为碰撞，1~81 为静态空间事件
 */
uint16_t epgm_quantize(const uint16_t distances[8], bool bumper_hit);

// ==============================================================================
// 有状态差分量化 API (与 Python 端对齐)
// ==============================================================================

/**
 * @brief 初始化有状态量化器
 * @param q 量化器句柄指针
 */
void epgm_quantizer_init(epgm_quantizer_t *q);

/**
 * @brief 重置量化器历史观测缓存
 * @param q 量化器句柄指针
 */
void epgm_quantizer_reset(epgm_quantizer_t *q);

/**
 * @brief 对齐 Python 端的一体化空间-动态事件量化函数
 * @param q 量化器句柄指针
 * @param distances 8 向毫米距离 [0, 1000]
 * @param bumper_hit 物理微动开关状态
 * @return 0: 碰撞; 1~81: 静态事件; 101~181: 动态突变事件
 */
uint16_t epgm_quantize_stateful(
    epgm_quantizer_t *q,
    const uint16_t distances[8],
    bool bumper_hit
);

// ==============================================================================
// 因果推理与可塑性突触更新 API
// ==============================================================================

/**
 * @brief 根据当前事件预测推理最优动作 (贪心结合好奇心探索)
 * @param rt 运行时句柄指针
 * @param current_event 当前感知事件 ID
 * @return 推荐动作 (0~4)
 */
uint8_t epgm_infer_action(const epgm_runtime_t *rt, uint16_t current_event);

/**
 * @brief 在线因果可塑性更新 (Hit 强化 / Surprise 生枝)
 * @param rt 运行时句柄指针
 * @param source_event 转移源事件
 * @param action 采取的动作
 * @param target_event 转移目标事件
 * @param reward_q8 即时回报 Q8.8
 * @return true 预测命中强化，false 失配生枝
 */
bool epgm_plasticity_update(
    epgm_runtime_t *rt,
    uint16_t source_event,
    uint8_t action,
    uint16_t target_event,
    int16_t reward_q8
);

// ==============================================================================
// FreeRTOS 多核并发互斥与双缓冲快照 API
// ==============================================================================

/**
 * @brief 初始化运行时的 FreeRTOS 互斥锁
 * @param rt 运行时句柄
 * @return true 成功，false 失败
 */
bool epgm_mutex_init(epgm_runtime_t *rt);

/**
 * @brief 释放并销毁互斥锁
 * @param rt 运行时句柄
 */
void epgm_mutex_deinit(epgm_runtime_t *rt);

/**
 * @brief 获取互斥锁
 * @param rt 运行时句柄
 * @param timeout_ms 超时毫秒数
 * @return true 成功获取，false 超时
 */
bool epgm_mutex_lock(epgm_runtime_t *rt, uint32_t timeout_ms);

/**
 * @brief 释放互斥锁
 * @param rt 运行时句柄
 */
void epgm_mutex_unlock(epgm_runtime_t *rt);

/**
 * @brief 极速内存快照复制 (Core 0 在微秒级持有锁完成复制，彻底消除与 Core 1 控制闭环争用)
 * @param src 源图结构体指针
 * @param dst 快照目标图结构体指针
 * @param timeout_ms 获取锁最大超时 (毫秒)
 * @return true 复制成功，false 超时未获取锁
 */
bool epgm_snapshot_copy(const epgm_runtime_t *src, epgm_runtime_t *dst, uint32_t timeout_ms);

/**
 * @brief 线程安全推理函数 (带 5ms 超时保底降级)
 * @param rt 运行时句柄
 * @param current_event 当前事件
 * @param fallback_action 若发生锁争用超时的保底动作
 * @param timeout_ms 最大等待超时 (推荐 5ms)
 * @return 决策动作
 */
uint8_t epgm_infer_action_safe(
    epgm_runtime_t *rt,
    uint16_t current_event,
    uint8_t fallback_action,
    uint32_t timeout_ms
);

/**
 * @brief 线程安全因果更新函数
 * @param rt 运行时句柄
 * @param source_event 源事件
 * @param action 动作
 * @param target_event 目标事件
 * @param reward_q8 回报
 * @param timeout_ms 获取锁超时 (毫秒)
 * @return true 命中强化，false 生枝或超时
 */
bool epgm_plasticity_update_safe(
    epgm_runtime_t *rt,
    uint16_t source_event,
    uint8_t action,
    uint16_t target_event,
    int16_t reward_q8,
    uint32_t timeout_ms
);

#ifdef __cplusplus
}
#endif

#endif // EPGM_CORE_H
