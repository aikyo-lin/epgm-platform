/**
 * @file epgm_storage.h
 * @brief [ZH] ESP32-S3 LittleFS Flash 掉电记忆与冷启动图恢复头文件。
 *        [EN] ESP32-S3 LittleFS Flash Non-volatile Persistence & Cold-boot Recovery Header.
 *
 * 设计意图 / Design Rationale:
 *  1. 紧凑二进制格式: 32 字节文件头 + 10 字节紧凑编码因果边，512 条边仅需 5.15 KB；
 *  2. 两阶段原子提交 (Two-Phase Atomic Commit): 先写入 .tmp 临时文件并计算 CRC32，
 *     自检通过后通过 POSIX rename() 原子替换，确保任何时刻断电均无破坏态；
 *  3. 完备冷启动恢复: 上电自动自检 Magic、版本号与 CRC32，数据损坏时安全告警回退。
 */

#ifndef EPGM_STORAGE_H
#define EPGM_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../epgm_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EPGM_STORAGE_MAGIC         0x4550474D  ///< ASCII "EPGM" (0x45, 0x50, 0x47, 0x4D)
#define EPGM_STORAGE_VERSION       0x0001      ///< 存储协议格式版本
#define EPGM_STORAGE_HEADER_SIZE   32          ///< 文件头严格固定 32 字节

#define EPGM_STORAGE_DEFAULT_PATH      "/littlefs/epgm_graph.bin"
#define EPGM_STORAGE_DEFAULT_TMP_PATH  "/littlefs/epgm_graph.bin.tmp"

// 状态错误码
typedef enum {
    EPGM_STORAGE_OK                = 0,
    EPGM_STORAGE_ERR_INVALID_PARAM = -1,
    EPGM_STORAGE_ERR_IO            = -2,
    EPGM_STORAGE_ERR_MAGIC         = -3,
    EPGM_STORAGE_ERR_VERSION       = -4,
    EPGM_STORAGE_ERR_CRC           = -5,
    EPGM_STORAGE_ERR_NOT_FOUND     = -6,
    EPGM_STORAGE_ERR_OVERFLOW      = -7
} epgm_storage_status_t;

#pragma pack(push, 1)

/**
 * @brief [ZH] 32 字节固定二进制持久化文件头
 *        [EN] 32-byte fixed binary persistence header
 */
typedef struct {
    uint32_t magic;             ///< 必须为 0x4550474D ("EPGM")
    uint16_t version;           ///< 固化版本号 (0x0001)
    uint16_t header_size;       ///< 头大小 (32 字节)
    uint16_t node_count;        ///< 活跃节点数
    uint16_t edge_count;        ///< 活跃边数
    uint32_t step_counter;      ///< 累计探索步数
    uint32_t total_hits;        ///< 累计命中数
    uint32_t total_surprises;   ///< 累计失配生枝数
    uint32_t timestamp_ms;      ///< 保存时时间戳
    uint32_t payload_crc32;     ///< 边数组的 CRC32 循环冗余校验码
} epgm_storage_header_t;

/**
 * @brief [ZH] 单条因果边紧凑二进制存储结构 (严格 10 字节)
 *        [EN] Single causal edge compact binary storage structure (strictly 10 bytes)
 */
typedef struct {
    uint16_t source_event;        ///< 源事件 ID (0~181)
    uint8_t  action;              ///< 动作 (0~4)
    uint16_t target_event;        ///< 目标事件 ID (0~181)
    int16_t  expected_reward_q8;  ///< Q8.8 预期回报
    uint16_t observation_count;   ///< 观测样本计数
    uint8_t  confidence;          ///< 置信度 (0~255)
} epgm_storage_edge_t;

#pragma pack(pop)

/**
 * @brief 计算标准的 IEEE 802.3 CRC32 循环冗余校验值
 * @param data 输入数据指针
 * @param length 数据字节长度
 * @return 32 位 CRC 校验码
 */
uint32_t epgm_storage_crc32(const uint8_t *data, size_t length);

/**
 * @brief 两阶段原子提交: 将因果图固化保存至 LittleFS
 * @param rt 待保存的因果图运行时句柄 (建议传入 RAM 快照)
 * @param filepath 目标保存文件路径 (为 NULL 时采用默认路径)
 * @return EPGM_STORAGE_OK 成功，其余为错误状态码
 */
int epgm_storage_save_atomic(const epgm_runtime_t *rt, const char *filepath);

/**
 * @brief 冷启动恢复: 从 LittleFS 读取因果图并校验 CRC32
 * @param rt 接收反序列化数据的运行时句柄
 * @param filepath 源文件路径 (为 NULL 时采用默认路径)
 * @return EPGM_STORAGE_OK 成功恢复，其余为错误状态码
 */
int epgm_storage_load(epgm_runtime_t *rt, const char *filepath);

#ifdef __cplusplus
}
#endif

#endif // EPGM_STORAGE_H
