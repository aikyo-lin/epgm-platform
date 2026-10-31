/**
 * @file protocol_schema.h
 * @brief [ZH] EPGM 机器人边缘端与云端通信协议数据结构规范 (C/C++ 头文件)。
 *        [EN] EPGM Robot Edge-to-Cloud Communication Protocol Schema (C/C++ Header).
 *
 * 主题规则 / MQTT Topic Conventions:
 *  - 机器人遥测与心跳 / Telemetry: `epgm/robot/{robot_id}/telemetry`
 *  - 增量图碎片上报 / Graph Delta: `epgm/robot/{robot_id}/graph_delta`
 *  - 全局优化图下发 / Model Downlink: `epgm/robot/{robot_id}/model_update`
 *  - 云端控制指令 / Control Command: `epgm/robot/{robot_id}/command`
 */

#ifndef EPGM_PROTOCOL_SCHEMA_H
#define EPGM_PROTOCOL_SCHEMA_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EPGM_PROTOCOL_VERSION 1
#define EPGM_MAX_SENSORS      8
#define EPGM_ROBOT_ID_LEN     16

/**
 * @brief [ZH] 机器人遥测与实时物理状态数据包 (50Hz / 10Hz)
 *        [EN] Robot Telemetry & Real-time Physical Status Packet
 */
typedef struct __attribute__((packed)) {
    uint8_t  protocol_ver;                  ///< [ZH] 协议版本号 / [EN] Protocol version
    char     robot_id[EPGM_ROBOT_ID_LEN];   ///< [ZH] 机器人设备唯一标识 / [EN] Unique robot ID
    uint32_t timestamp_ms;                  ///< [ZH] 毫秒时间戳 / [EN] Timestamp in ms
    uint16_t current_event_id;              ///< [ZH] 当前量化事件ID / [EN] Current quantized event ID
    uint8_t  last_action;                   ///< [ZH] 最近执行动作 / [EN] Last executed action
    int16_t  last_reward_q8;                ///< [ZH] Q8.8 定点即时回报 / [EN] Immediate reward (Q8.8 fixed-point)
    uint8_t  battery_percentage;            ///< [ZH] 电池电量 (0~100) / [EN] Battery percentage (0-100)
    uint8_t  bumper_state;                  ///< [ZH] 碰撞传感器状态掩码 / [EN] Bumper hit bitmask
    uint16_t sensor_distances[EPGM_MAX_SENSORS]; ///< [ZH] 各测距传感器毫米读数 / [EN] Distance in mm
} epgm_telemetry_packet_t;

/**
 * @brief [ZH] 增量因果拓扑边更新项
 *        [EN] Incremental Causal Topological Edge Item
 */
typedef struct __attribute__((packed)) {
    uint16_t source_event;                  ///< [ZH] 源事件ID / [EN] Source event ID
    uint8_t  action;                        ///< [ZH] 执行动作 / [EN] Executed action
    uint16_t target_event;                  ///< [ZH] 后继事件ID / [EN] Target event ID
    int16_t  expected_reward_q8;            ///< [ZH] 预期回报 (Q8.8) / [EN] Expected reward (Q8.8)
    uint16_t observation_count;             ///< [ZH] 观测样本计数 / [EN] Observation count
    uint8_t  confidence;                    ///< [ZH] 置信度 (0~255) / [EN] Confidence score (0-255)
} epgm_edge_delta_item_t;

/**
 * @brief [ZH] 机器人上报给云端的增量图碎片包头
 *        [EN] Robot-to-Cloud Incremental Graph Fragment Header
 */
typedef struct __attribute__((packed)) {
    uint8_t  protocol_ver;
    char     robot_id[EPGM_ROBOT_ID_LEN];
    uint32_t sequence_id;                   ///< [ZH] 增量包递增序号 / [EN] Monotonic sequence counter
    uint16_t edge_count;                    ///< [ZH] 携带的更新边数量 / [EN] Number of edge items attached
    uint32_t step_index;                    ///< [ZH] 机器人内部全局执行步数 / [EN] Edge robot step index
} epgm_graph_delta_header_t;

#ifdef __cplusplus
}
#endif

#endif // EPGM_PROTOCOL_SCHEMA_H
