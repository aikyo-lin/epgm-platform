/**
 * @file mock_littlefs.c
 * @brief [ZH] 宿主机 LittleFS 文件系统模拟层实现。
 *        [EN] Host LittleFS File System Mock Implementation.
 */

#include "mock_littlefs.h"
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32) || defined(_WIN64)
#include <direct.h>
#include <io.h>
#define mkdir_compat(path) _mkdir(path)
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#define mkdir_compat(path) mkdir(path, 0755)
#endif

static char s_sandbox_dir[256] = MOCK_LITTLEFS_DEFAULT_DIR;
static bool s_inject_write_fault = false;

int mock_littlefs_init(const char *sandbox_path) {
    if (sandbox_path && strlen(sandbox_path) > 0) {
        strncpy(s_sandbox_dir, sandbox_path, sizeof(s_sandbox_dir) - 1);
    } else {
        strncpy(s_sandbox_dir, MOCK_LITTLEFS_DEFAULT_DIR, sizeof(s_sandbox_dir) - 1);
    }
    s_inject_write_fault = false;
    mkdir_compat(s_sandbox_dir);
    return 0;
}

void mock_littlefs_cleanup(void) {
    // 简单重置故障注入标志
    s_inject_write_fault = false;
}

int mock_littlefs_resolve_path(const char *vfs_path, char *out_host_path, size_t max_len) {
    if (!vfs_path || !out_host_path || max_len == 0) return -1;

    const char *rel_name = vfs_path;
    if (strncmp(vfs_path, "/littlefs/", 10) == 0) {
        rel_name = vfs_path + 10;
    } else if (strncmp(vfs_path, "/littlefs", 9) == 0) {
        rel_name = vfs_path + 9;
        if (*rel_name == '/' || *rel_name == '\\') rel_name++;
    } else if (strncmp(vfs_path, "/spiffs/", 8) == 0) {
        rel_name = vfs_path + 8;
    }

#if defined(_WIN32) || defined(_WIN64)
    snprintf(out_host_path, max_len, "%s\\%s", s_sandbox_dir, rel_name);
#else
    snprintf(out_host_path, max_len, "%s/%s", s_sandbox_dir, rel_name);
#endif
    return 0;
}

void mock_littlefs_set_write_fault(bool fail_next_write) {
    s_inject_write_fault = fail_next_write;
}

int mock_littlefs_corrupt_file(const char *vfs_path, size_t byte_offset) {
    char host_path[256];
    if (mock_littlefs_resolve_path(vfs_path, host_path, sizeof(host_path)) != 0) {
        return -1;
    }

    FILE *fp = fopen(host_path, "r+b");
    if (!fp) return -1;

    if (fseek(fp, (long)byte_offset, SEEK_SET) != 0) {
        fclose(fp);
        return -2;
    }

    uint8_t byte = 0;
    if (fread(&byte, 1, 1, fp) != 1) {
        fclose(fp);
        return -3;
    }

    byte ^= 0xFF; // 反转所有比特位，必定造成 CRC32 校验失败

    if (fseek(fp, (long)byte_offset, SEEK_SET) != 0) {
        fclose(fp);
        return -4;
    }

    if (fwrite(&byte, 1, 1, fp) != 1) {
        fclose(fp);
        return -5;
    }

    fflush(fp);
    fclose(fp);
    return 0;
}

FILE *mock_lfs_fopen(const char *path, const char *mode) {
    char host_path[256];
    mock_littlefs_resolve_path(path, host_path, sizeof(host_path));
    return fopen(host_path, mode);
}

size_t mock_lfs_fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream) {
    if (s_inject_write_fault) {
        // 模拟掉电中断: 仅写入部分或返回 0 错误
        s_inject_write_fault = false;
        return 0;
    }
    return fwrite(ptr, size, nmemb, stream);
}

size_t mock_lfs_fread(void *ptr, size_t size, size_t nmemb, FILE *stream) {
    return fread(ptr, size, nmemb, stream);
}

int mock_lfs_fflush(FILE *stream) {
    return fflush(stream);
}

int mock_lfs_fclose(FILE *stream) {
    return fclose(stream);
}

int mock_lfs_rename(const char *oldpath, const char *newpath) {
    char host_old[256];
    char host_new[256];
    mock_littlefs_resolve_path(oldpath, host_old, sizeof(host_old));
    mock_littlefs_resolve_path(newpath, host_new, sizeof(host_new));

#if defined(_WIN32) || defined(_WIN64)
    // Windows 下如果目标文件已存在，标准 rename 会失败，因此先删除目标文件实现 POSIX 原子覆盖语义
    remove(host_new);
#endif
    return rename(host_old, host_new);
}

int mock_lfs_remove(const char *pathname) {
    char host_path[256];
    mock_littlefs_resolve_path(pathname, host_path, sizeof(host_path));
    return remove(host_path);
}

bool mock_lfs_exists(const char *pathname) {
    char host_path[256];
    mock_littlefs_resolve_path(pathname, host_path, sizeof(host_path));
    FILE *fp = fopen(host_path, "rb");
    if (fp) {
        fclose(fp);
        return true;
    }
    return false;
}
