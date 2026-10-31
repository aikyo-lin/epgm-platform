/**
 * @file mock_freertos.c
 * @brief [ZH] 宿主机跨平台 FreeRTOS API 模拟层实现 (Windows Win32 / POSIX)。
 *        [EN] Host Cross-platform FreeRTOS API Mock Implementation (Windows & POSIX).
 */

#include "mock_freertos.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32) || defined(_WIN64)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <process.h>
#else
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#endif

// ==============================================================================
// 内部互斥锁与信号量实现结构
// ==============================================================================
typedef struct {
    bool is_mutex;
#if defined(_WIN32) || defined(_WIN64)
    HANDLE handle;
#else
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    int32_t         count;
#endif
} mock_semaphore_t;

// ==============================================================================
// 内部消息队列实现结构 (环形队列 + 互斥保护)
// ==============================================================================
typedef struct {
    uint8_t        *storage;
    UBaseType_t     length;
    UBaseType_t     item_size;
    UBaseType_t     head;
    UBaseType_t     tail;
    UBaseType_t     count;
#if defined(_WIN32) || defined(_WIN64)
    CRITICAL_SECTION cs;
    HANDLE           not_empty_event;
    HANDLE           not_full_event;
#else
    pthread_mutex_t  lock;
    pthread_cond_t   not_empty;
    pthread_cond_t   not_full;
#endif
} mock_queue_t;

// ==============================================================================
// 内部任务句柄结构
// ==============================================================================
typedef struct {
    TaskFunction_t task_func;
    void          *param;
    char           name[32];
    BaseType_t     core_id;
#if defined(_WIN32) || defined(_WIN64)
    HANDLE         thread_handle;
    unsigned int   thread_id;
#else
    pthread_t      thread;
#endif
} mock_task_t;

// ==============================================================================
// 互斥量与信号量函数实现
// ==============================================================================

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    mock_semaphore_t *sem = (mock_semaphore_t *)calloc(1, sizeof(mock_semaphore_t));
    if (!sem) return NULL;
    sem->is_mutex = true;
#if defined(_WIN32) || defined(_WIN64)
    sem->handle = CreateMutexA(NULL, FALSE, NULL);
    if (!sem->handle) {
        free(sem);
        return NULL;
    }
#else
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&sem->mutex, &attr);
    pthread_mutexattr_destroy(&attr);
    pthread_cond_init(&sem->cond, NULL);
    sem->count = 1;
#endif
    return (SemaphoreHandle_t)sem;
}

SemaphoreHandle_t xSemaphoreCreateBinary(void) {
    mock_semaphore_t *sem = (mock_semaphore_t *)calloc(1, sizeof(mock_semaphore_t));
    if (!sem) return NULL;
    sem->is_mutex = false;
#if defined(_WIN32) || defined(_WIN64)
    // 初始计数 0，最大计数 1 的二值信号量
    sem->handle = CreateSemaphoreA(NULL, 0, 1, NULL);
    if (!sem->handle) {
        free(sem);
        return NULL;
    }
#else
    pthread_mutex_init(&sem->mutex, NULL);
    pthread_cond_init(&sem->cond, NULL);
    sem->count = 0;
#endif
    return (SemaphoreHandle_t)sem;
}

void vSemaphoreDelete(SemaphoreHandle_t xSemaphore) {
    if (!xSemaphore) return;
    mock_semaphore_t *sem = (mock_semaphore_t *)xSemaphore;
#if defined(_WIN32) || defined(_WIN64)
    if (sem->handle) {
        CloseHandle(sem->handle);
        sem->handle = NULL;
    }
#else
    pthread_mutex_destroy(&sem->mutex);
    pthread_cond_destroy(&sem->cond);
#endif
    free(sem);
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xBlockTime) {
    if (!xSemaphore) return pdFALSE;
    mock_semaphore_t *sem = (mock_semaphore_t *)xSemaphore;

#if defined(_WIN32) || defined(_WIN64)
    DWORD wait_ms = (xBlockTime == portMAX_DELAY) ? INFINITE : (DWORD)xBlockTime;
    DWORD res = WaitForSingleObject(sem->handle, wait_ms);
    return (res == WAIT_OBJECT_0) ? pdTRUE : pdFALSE;
#else
    if (xBlockTime == portMAX_DELAY) {
        pthread_mutex_lock(&sem->mutex);
        if (sem->is_mutex) {
            return pdTRUE;
        } else {
            while (sem->count == 0) {
                pthread_cond_wait(&sem->cond, &sem->mutex);
            }
            sem->count = 0;
            pthread_mutex_unlock(&sem->mutex);
            return pdTRUE;
        }
    } else {
        struct timeval now;
        gettimeofday(&now, NULL);
        struct timespec ts;
        uint64_t ns = (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_usec * 1000ULL + (uint64_t)xBlockTime * 1000000ULL;
        ts.tv_sec = (time_t)(ns / 1000000000ULL);
        ts.tv_nsec = (long)(ns % 1000000000ULL);

        if (sem->is_mutex) {
            int ret = pthread_mutex_timedlock(&sem->mutex, &ts);
            return (ret == 0) ? pdTRUE : pdFALSE;
        } else {
            pthread_mutex_lock(&sem->mutex);
            while (sem->count == 0) {
                int ret = pthread_cond_timedwait(&sem->cond, &sem->mutex, &ts);
                if (ret == ETIMEDOUT) {
                    pthread_mutex_unlock(&sem->mutex);
                    return pdFALSE;
                }
            }
            sem->count = 0;
            pthread_mutex_unlock(&sem->mutex);
            return pdTRUE;
        }
    }
#endif
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    if (!xSemaphore) return pdFALSE;
    mock_semaphore_t *sem = (mock_semaphore_t *)xSemaphore;

#if defined(_WIN32) || defined(_WIN64)
    if (sem->is_mutex) {
        return ReleaseMutex(sem->handle) ? pdTRUE : pdFALSE;
    } else {
        return ReleaseSemaphore(sem->handle, 1, NULL) ? pdTRUE : pdFALSE;
    }
#else
    pthread_mutex_lock(&sem->mutex);
    if (sem->is_mutex) {
        pthread_mutex_unlock(&sem->mutex);
        return pdTRUE;
    } else {
        if (sem->count == 0) {
            sem->count = 1;
            pthread_cond_signal(&sem->cond);
        }
        pthread_mutex_unlock(&sem->mutex);
        return pdTRUE;
    }
#endif
}

// ==============================================================================
// 消息队列函数实现
// ==============================================================================

QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize) {
    if (uxQueueLength == 0 || uxItemSize == 0) return NULL;
    mock_queue_t *q = (mock_queue_t *)calloc(1, sizeof(mock_queue_t));
    if (!q) return NULL;

    q->length = uxQueueLength;
    q->item_size = uxItemSize;
    q->storage = (uint8_t *)malloc(uxQueueLength * uxItemSize);
    if (!q->storage) {
        free(q);
        return NULL;
    }

#if defined(_WIN32) || defined(_WIN64)
    InitializeCriticalSection(&q->cs);
    q->not_empty_event = CreateEventA(NULL, FALSE, FALSE, NULL);
    q->not_full_event = CreateEventA(NULL, FALSE, TRUE, NULL);
#else
    pthread_mutex_init(&q->lock, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
#endif
    return (QueueHandle_t)q;
}

void vQueueDelete(QueueHandle_t xQueue) {
    if (!xQueue) return;
    mock_queue_t *q = (mock_queue_t *)xQueue;
#if defined(_WIN32) || defined(_WIN64)
    DeleteCriticalSection(&q->cs);
    CloseHandle(q->not_empty_event);
    CloseHandle(q->not_full_event);
#else
    pthread_mutex_destroy(&q->lock);
    pthread_cond_destroy(&q->not_empty);
    pthread_cond_destroy(&q->not_full);
#endif
    if (q->storage) free(q->storage);
    free(q);
}

BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait) {
    if (!xQueue || !pvItemToQueue) return pdFAIL;
    mock_queue_t *q = (mock_queue_t *)xQueue;

#if defined(_WIN32) || defined(_WIN64)
    DWORD wait_ms = (xTicksToWait == portMAX_DELAY) ? INFINITE : (DWORD)xTicksToWait;
    DWORD start_time = GetTickCount();

    for (;;) {
        EnterCriticalSection(&q->cs);
        if (q->count < q->length) {
            uint8_t *dst = q->storage + (q->tail * q->item_size);
            memcpy(dst, pvItemToQueue, q->item_size);
            q->tail = (q->tail + 1) % q->length;
            q->count++;
            SetEvent(q->not_empty_event);
            if (q->count >= q->length) {
                ResetEvent(q->not_full_event);
            }
            LeaveCriticalSection(&q->cs);
            return pdPASS;
        }
        LeaveCriticalSection(&q->cs);

        if (xTicksToWait == 0) return pdFAIL;
        DWORD elapsed = GetTickCount() - start_time;
        if (wait_ms != INFINITE && elapsed >= wait_ms) return pdFAIL;
        DWORD remaining = (wait_ms == INFINITE) ? INFINITE : (wait_ms - elapsed);

        if (WaitForSingleObject(q->not_full_event, remaining) != WAIT_OBJECT_0) {
            return pdFAIL;
        }
    }
#else
    struct timeval now;
    gettimeofday(&now, NULL);
    struct timespec ts;
    uint64_t ns = (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_usec * 1000ULL + (uint64_t)xTicksToWait * 1000000ULL;
    ts.tv_sec = (time_t)(ns / 1000000000ULL);
    ts.tv_nsec = (long)(ns % 1000000000ULL);

    pthread_mutex_lock(&q->lock);
    while (q->count >= q->length) {
        if (xTicksToWait == 0) {
            pthread_mutex_unlock(&q->lock);
            return pdFAIL;
        }
        if (xTicksToWait == portMAX_DELAY) {
            pthread_cond_wait(&q->not_full, &q->lock);
        } else {
            int ret = pthread_cond_timedwait(&q->not_full, &q->lock, &ts);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&q->lock);
                return pdFAIL;
            }
        }
    }

    uint8_t *dst = q->storage + (q->tail * q->item_size);
    memcpy(dst, pvItemToQueue, q->item_size);
    q->tail = (q->tail + 1) % q->length;
    q->count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->lock);
    return pdPASS;
#endif
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait) {
    if (!xQueue || !pvBuffer) return pdFAIL;
    mock_queue_t *q = (mock_queue_t *)xQueue;

#if defined(_WIN32) || defined(_WIN64)
    DWORD wait_ms = (xTicksToWait == portMAX_DELAY) ? INFINITE : (DWORD)xTicksToWait;
    DWORD start_time = GetTickCount();

    for (;;) {
        EnterCriticalSection(&q->cs);
        if (q->count > 0) {
            uint8_t *src = q->storage + (q->head * q->item_size);
            memcpy(pvBuffer, src, q->item_size);
            q->head = (q->head + 1) % q->length;
            q->count--;
            SetEvent(q->not_full_event);
            if (q->count == 0) {
                ResetEvent(q->not_empty_event);
            }
            LeaveCriticalSection(&q->cs);
            return pdPASS;
        }
        LeaveCriticalSection(&q->cs);

        if (xTicksToWait == 0) return pdFAIL;
        DWORD elapsed = GetTickCount() - start_time;
        if (wait_ms != INFINITE && elapsed >= wait_ms) return pdFAIL;
        DWORD remaining = (wait_ms == INFINITE) ? INFINITE : (wait_ms - elapsed);

        if (WaitForSingleObject(q->not_empty_event, remaining) != WAIT_OBJECT_0) {
            return pdFAIL;
        }
    }
#else
    struct timeval now;
    gettimeofday(&now, NULL);
    struct timespec ts;
    uint64_t ns = (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_usec * 1000ULL + (uint64_t)xTicksToWait * 1000000ULL;
    ts.tv_sec = (time_t)(ns / 1000000000ULL);
    ts.tv_nsec = (long)(ns % 1000000000ULL);

    pthread_mutex_lock(&q->lock);
    while (q->count == 0) {
        if (xTicksToWait == 0) {
            pthread_mutex_unlock(&q->lock);
            return pdFAIL;
        }
        if (xTicksToWait == portMAX_DELAY) {
            pthread_cond_wait(&q->not_empty, &q->lock);
        } else {
            int ret = pthread_cond_timedwait(&q->not_empty, &q->lock, &ts);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&q->lock);
                return pdFAIL;
            }
        }
    }

    uint8_t *src = q->storage + (q->head * q->item_size);
    memcpy(pvBuffer, src, q->item_size);
    q->head = (q->head + 1) % q->length;
    q->count--;
    pthread_cond_signal(&q->not_full);
    pthread_mutex_unlock(&q->lock);
    return pdPASS;
#endif
}

// ==============================================================================
// 系统时间与任务延时实现
// ==============================================================================

void vTaskDelay(const TickType_t xTicksToDelay) {
#if defined(_WIN32) || defined(_WIN64)
    Sleep((DWORD)xTicksToDelay);
#else
    usleep((useconds_t)xTicksToDelay * 1000);
#endif
}

TickType_t xTaskGetTickCount(void) {
#if defined(_WIN32) || defined(_WIN64)
    return (TickType_t)GetTickCount();
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (TickType_t)((tv.tv_sec * 1000) + (tv.tv_usec / 1000));
#endif
}

// ==============================================================================
// 任务线程创建实现
// ==============================================================================

#if defined(_WIN32) || defined(_WIN64)
static unsigned int __stdcall win32_task_wrapper(void *arg) {
    mock_task_t *task = (mock_task_t *)arg;
    if (task && task->task_func) {
        task->task_func(task->param);
    }
    return 0;
}
#else
static void *posix_task_wrapper(void *arg) {
    mock_task_t *task = (mock_task_t *)arg;
    if (task && task->task_func) {
        task->task_func(task->param);
    }
    return NULL;
}
#endif

BaseType_t xTaskCreate(
    TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint32_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask
) {
    return xTaskCreatePinnedToCore(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, 0);
}

BaseType_t xTaskCreatePinnedToCore(
    TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint32_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask,
    const BaseType_t xCoreID
) {
    (void)usStackDepth;
    (void)uxPriority;

    if (!pxTaskCode) return pdFAIL;
    mock_task_t *task = (mock_task_t *)calloc(1, sizeof(mock_task_t));
    if (!task) return pdFAIL;

    task->task_func = pxTaskCode;
    task->param = pvParameters;
    task->core_id = xCoreID;
    if (pcName) {
        strncpy(task->name, pcName, sizeof(task->name) - 1);
    }

#if defined(_WIN32) || defined(_WIN64)
    task->thread_handle = (HANDLE)_beginthreadex(NULL, 0, win32_task_wrapper, task, 0, &task->thread_id);
    if (!task->thread_handle) {
        free(task);
        return pdFAIL;
    }
#else
    int res = pthread_create(&task->thread, NULL, posix_task_wrapper, task);
    if (res != 0) {
        free(task);
        return pdFAIL;
    }
#endif

    if (pxCreatedTask) {
        *pxCreatedTask = (TaskHandle_t)task;
    }
    return pdPASS;
}

bool mock_task_join(TaskHandle_t xTask, uint32_t timeout_ms) {
    if (!xTask) return false;
    mock_task_t *task = (mock_task_t *)xTask;
    bool success = false;

#if defined(_WIN32) || defined(_WIN64)
    if (task->thread_handle) {
        DWORD res = WaitForSingleObject(task->thread_handle, (DWORD)timeout_ms);
        if (res == WAIT_OBJECT_0) {
            success = true;
        }
        CloseHandle(task->thread_handle);
        task->thread_handle = NULL;
    }
#else
    // POSIX pthread_join (timedjoin where supported, or regular join)
    pthread_join(task->thread, NULL);
    success = true;
#endif

    free(task);
    return success;
}
