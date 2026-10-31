/**
 * @file mock_littlefs.h
 * @brief [ZH] 宿主机 LittleFS 文件系统模拟层头文件。
 *        [EN] Host LittleFS File System Mock Header.
 *
 * 设计意图 / Design Rationale:
 *  - 将嵌入式 SPI Flash 上的 LittleFS POSIX 文件路径 (/littlefs/...) 映射至宿主机沙盒目录；
 *  - 模拟原子重命名 (rename)、掉电写入中断 (Fault Injection) 与单字节损坏 (Bit-flip/Corruption)；
 *  - 支持非易失性存储与冷启动恢复验证。
 */

#ifndef MOCK_LITTLEFS_H
#define MOCK_LITTLEFS_H

#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOCK_LITTLEFS_DEFAULT_DIR "./test_storage_fs"

/**
 * @brief 初始化宿主机 LittleFS 沙盒环境 (创建本地测试目录)
 * @param sandbox_path 本地测试沙盒目录路径，为 NULL 时采用默认值
 * @return 0 成功，非 0 失败
 */
int mock_littlefs_init(const char *sandbox_path);

/**
 * @brief 清理并移除沙盒测试环境
 */
void mock_littlefs_cleanup(void);

/**
 * @brief 将嵌入式虚拟路径 (/littlefs/xxx) 映射为宿主机本地真实文件路径
 * @param vfs_path 虚拟路径 (如 "/littlefs/epgm_graph.bin")
 * @param out_host_path 宿主机真实路径输出缓冲
 * @param max_len 输出缓冲最大长度
 * @return 0 成功，非 0 失败
 */
int mock_littlefs_resolve_path(const char *vfs_path, char *out_host_path, size_t max_len);

/**
 * @brief 故障注入: 设置是否模拟写入失败/中断
 * @param fail_next_write 若为 true，下一次写操作将模拟 Flash 掉电直接中断返回 -1
 */
void mock_littlefs_set_write_fault(bool fail_next_write);

/**
 * @brief 故障注入: 对指定文件注入损坏 (单字节反转或破坏，用于测试 CRC32 检错)
 * @param vfs_path 目标虚拟路径
 * @param byte_offset 损坏注入字节偏移
 * @return 0 成功，非 0 失败
 */
int mock_littlefs_corrupt_file(const char *vfs_path, size_t byte_offset);

// POSIX 文件操作标准封装 (与真实 LittleFS VFS 接口保持一致)
FILE *mock_lfs_fopen(const char *path, const char *mode);
size_t mock_lfs_fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream);
size_t mock_lfs_fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
int mock_lfs_fflush(FILE *stream);
int mock_lfs_fclose(FILE *stream);
int mock_lfs_rename(const char *oldpath, const char *newpath);
int mock_lfs_remove(const char *pathname);
bool mock_lfs_exists(const char *pathname);

#ifdef __cplusplus
}
#endif

#endif // MOCK_LITTLEFS_H
