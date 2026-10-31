/**
 * @file epgm_storage.c
 * @brief [ZH] ESP32-S3 LittleFS Flash 掉电记忆与冷启动图恢复实现。
 *        [EN] ESP32-S3 LittleFS Flash Non-volatile Persistence & Cold-boot Recovery Implementation.
 */

#include "epgm_storage.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#if defined(ESP_PLATFORM) || (defined(ARDUINO) && !defined(EPGM_HOST_TEST))
#define LFS_FOPEN(p, m)        fopen(p, m)
#define LFS_FWRITE(p, s, n, f) fwrite(p, s, n, f)
#define LFS_FREAD(p, s, n, f)  fread(p, s, n, f)
#define LFS_FFLUSH(f)          fflush(f)
#define LFS_FCLOSE(f)          fclose(f)
#define LFS_RENAME(o, n)       rename(o, n)
#define LFS_REMOVE(p)          remove(p)
#else
#if __has_include("mock_littlefs.h")
#include "mock_littlefs.h"
#elif __has_include("../../test/mock_littlefs.h")
#include "../../test/mock_littlefs.h"
#elif __has_include("../test/mock_littlefs.h")
#include "../test/mock_littlefs.h"
#endif
#define LFS_FOPEN(p, m)        mock_lfs_fopen(p, m)
#define LFS_FWRITE(p, s, n, f) mock_lfs_fwrite(p, s, n, f)
#define LFS_FREAD(p, s, n, f)  mock_lfs_fread(p, s, n, f)
#define LFS_FFLUSH(f)          mock_lfs_fflush(f)
#define LFS_FCLOSE(f)          mock_lfs_fclose(f)
#define LFS_RENAME(o, n)       mock_lfs_rename(o, n)
#define LFS_REMOVE(p)          mock_lfs_remove(p)
#endif

// 编译期断言确保二进制布局精确对齐
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(epgm_storage_header_t) == 32, "epgm_storage_header_t size must be 32 bytes");
_Static_assert(sizeof(epgm_storage_edge_t) == 10, "epgm_storage_edge_t size must be 10 bytes");
#endif

// ==============================================================================
// IEEE 802.3 标准 CRC32 计算实现
// ==============================================================================

uint32_t epgm_storage_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    if (!data || length == 0) return ~crc;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

// ==============================================================================
// 两阶段原子安全保存 (Two-Phase Atomic Commit)
// ==============================================================================

int epgm_storage_save_atomic(const epgm_runtime_t *rt, const char *filepath) {
    if (!rt) return EPGM_STORAGE_ERR_INVALID_PARAM;

    const char *target_path = (filepath && strlen(filepath) > 0) ? filepath : EPGM_STORAGE_DEFAULT_PATH;
    char tmp_path[256];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", target_path);

    // 1. 提取所有有效因果边至紧凑存储结构缓冲
    epgm_storage_edge_t packed_edges[EPGM_MAX_EDGES];
    uint16_t active_count = 0;

    for (uint16_t i = 0; i < rt->active_edge_count && i < EPGM_MAX_EDGES; ++i) {
        if (rt->edges[i].is_active) {
            packed_edges[active_count].source_event = rt->edges[i].source_event;
            packed_edges[active_count].action = rt->edges[i].action;
            packed_edges[active_count].target_event = rt->edges[i].target_event;
            packed_edges[active_count].expected_reward_q8 = rt->edges[i].expected_reward_q8;
            packed_edges[active_count].observation_count = rt->edges[i].observation_count;
            packed_edges[active_count].confidence = rt->edges[i].confidence;
            active_count++;
        }
    }

    // 2. 计算因果边紧凑二进制载荷的 CRC32
    size_t payload_bytes = active_count * sizeof(epgm_storage_edge_t);
    uint32_t crc = epgm_storage_crc32((const uint8_t *)packed_edges, payload_bytes);

    // 3. 构建 32 字节标准文件头
    epgm_storage_header_t header;
    memset(&header, 0, sizeof(header));
    header.magic = EPGM_STORAGE_MAGIC;
    header.version = EPGM_STORAGE_VERSION;
    header.header_size = (uint16_t)sizeof(epgm_storage_header_t);
    header.node_count = active_count; // 活跃边/节点索引
    header.edge_count = active_count;
    header.step_counter = rt->step_counter;
    header.total_hits = rt->total_hits;
    header.total_surprises = rt->total_surprises;
    header.timestamp_ms = 0; // 可由上层传入系统时间戳
    header.payload_crc32 = crc;

    // 4. 第一阶段写入临时文件: .tmp
    FILE *fp = LFS_FOPEN(tmp_path, "wb");
    if (!fp) {
        return EPGM_STORAGE_ERR_IO;
    }

    if (LFS_FWRITE(&header, sizeof(header), 1, fp) != 1) {
        LFS_FCLOSE(fp);
        LFS_REMOVE(tmp_path);
        return EPGM_STORAGE_ERR_IO;
    }

    if (active_count > 0) {
        if (LFS_FWRITE(packed_edges, sizeof(epgm_storage_edge_t), active_count, fp) != active_count) {
            LFS_FCLOSE(fp);
            LFS_REMOVE(tmp_path);
            return EPGM_STORAGE_ERR_IO;
        }
    }

    LFS_FFLUSH(fp);
    LFS_FCLOSE(fp);

    // 5. 校验阶段: 读回临时文件自检 Header 与 CRC32 确保 Flash 物理落盘无误
    FILE *verify_fp = LFS_FOPEN(tmp_path, "rb");
    if (!verify_fp) {
        LFS_REMOVE(tmp_path);
        return EPGM_STORAGE_ERR_IO;
    }

    epgm_storage_header_t verify_hdr;
    if (LFS_FREAD(&verify_hdr, sizeof(verify_hdr), 1, verify_fp) != 1 ||
        verify_hdr.magic != EPGM_STORAGE_MAGIC ||
        verify_hdr.payload_crc32 != crc) {
        LFS_FCLOSE(verify_fp);
        LFS_REMOVE(tmp_path);
        return EPGM_STORAGE_ERR_CRC;
    }
    LFS_FCLOSE(verify_fp);

    // 6. 第二阶段原子替换: 通过 POSIX rename() 瞬间更新目录项，实现抗掉电原子提交
    if (LFS_RENAME(tmp_path, target_path) != 0) {
        LFS_REMOVE(tmp_path);
        return EPGM_STORAGE_ERR_IO;
    }

    return EPGM_STORAGE_OK;
}

// ==============================================================================
// 冷启动数据加载与 CRC32 自检恢复 (Cold Boot Recovery)
// ==============================================================================

int epgm_storage_load(epgm_runtime_t *rt, const char *filepath) {
    if (!rt) return EPGM_STORAGE_ERR_INVALID_PARAM;

    const char *target_path = (filepath && strlen(filepath) > 0) ? filepath : EPGM_STORAGE_DEFAULT_PATH;

    FILE *fp = LFS_FOPEN(target_path, "rb");
    if (!fp) {
        return EPGM_STORAGE_ERR_NOT_FOUND;
    }

    // 1. 读取并校验 32 字节文件头
    epgm_storage_header_t header;
    if (LFS_FREAD(&header, sizeof(header), 1, fp) != 1) {
        LFS_FCLOSE(fp);
        return EPGM_STORAGE_ERR_IO;
    }

    if (header.magic != EPGM_STORAGE_MAGIC) {
        LFS_FCLOSE(fp);
        return EPGM_STORAGE_ERR_MAGIC;
    }

    if (header.version != EPGM_STORAGE_VERSION) {
        LFS_FCLOSE(fp);
        return EPGM_STORAGE_ERR_VERSION;
    }

    if (header.edge_count > EPGM_MAX_EDGES) {
        LFS_FCLOSE(fp);
        return EPGM_STORAGE_ERR_OVERFLOW;
    }

    // 2. 读取紧凑边二进制载荷
    epgm_storage_edge_t packed_edges[EPGM_MAX_EDGES];
    if (header.edge_count > 0) {
        size_t read_edges = LFS_FREAD(packed_edges, sizeof(epgm_storage_edge_t), header.edge_count, fp);
        if (read_edges != header.edge_count) {
            LFS_FCLOSE(fp);
            return EPGM_STORAGE_ERR_IO;
        }
    }
    LFS_FCLOSE(fp);

    // 3. CRC32 数据完整性校验 (防 Flash 掉电位翻转或截断损坏)
    size_t payload_bytes = header.edge_count * sizeof(epgm_storage_edge_t);
    uint32_t calc_crc = epgm_storage_crc32((const uint8_t *)packed_edges, payload_bytes);
    if (calc_crc != header.payload_crc32) {
        return EPGM_STORAGE_ERR_CRC;
    }

    // 4. 校验通过，原子恢复因果图运行时拓扑
    SemaphoreHandle_t existing_mutex = rt->mutex;
    memset(rt, 0, sizeof(epgm_runtime_t));
    rt->mutex = existing_mutex;

    rt->step_counter = header.step_counter;
    rt->total_hits = header.total_hits;
    rt->total_surprises = header.total_surprises;
    rt->active_edge_count = header.edge_count;

    for (uint16_t i = 0; i < header.edge_count; ++i) {
        rt->edges[i].source_event = packed_edges[i].source_event;
        rt->edges[i].action = packed_edges[i].action;
        rt->edges[i].target_event = packed_edges[i].target_event;
        rt->edges[i].expected_reward_q8 = packed_edges[i].expected_reward_q8;
        rt->edges[i].observation_count = packed_edges[i].observation_count;
        rt->edges[i].confidence = packed_edges[i].confidence;
        rt->edges[i].is_active = true;
    }

    return EPGM_STORAGE_OK;
}
