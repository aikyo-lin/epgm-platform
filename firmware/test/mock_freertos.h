/**
 * @file mock_freertos.h
 * @brief [ZH] 宿主机跨平台 FreeRTOS API 模拟层头文件 (支持 Windows 与 Linux 原生多线程)。
 *        [EN] Host Cross-platform FreeRTOS API Mock Header (Windows & POSIX native threading).
 *
 * 设计意图 / Design Rationale:
 *  - 允许因果图核心、并发控制与外设驱动在宿主机 (PC/CI) 上直接使用原生编译器 (MSVC / GCC) 进行测试；
 *  - 提供与真实 FreeRTOS 语义一致的互斥锁、队列、定时延时与任务创建功能；
 *  - 支持毫秒级超时等待与多线程竞态检测。
 */

#ifndef MOCK_FREERTOS_H
#define MOCK_FREERTOS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// FreeRTOS 基础类型与宏定义
// ==============================================================================
typedef int32_t   BaseType_t;
typedef uint32_t  UBaseType_t;
typedef uint32_t  TickType_t;

#define pdTRUE          ((BaseType_t)1)
#define pdFALSE         ((BaseType_t)0)
#define pdPASS          (pdTRUE)
#define pdFAIL          (pdFALSE)

#define portMAX_DELAY   ((TickType_t)0xFFFFFFFFUL)

#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#endif

typedef void *SemaphoreHandle_t;
typedef void *QueueHandle_t;
typedef void *TaskHandle_t;

typedef void (*TaskFunction_t)(void *pvParameters);

// ==============================================================================
// 互斥量与信号量 API
// ==============================================================================

/**
 * @brief 创建互斥信号量 (模拟 FreeRTOS xSemaphoreCreateMutex)
 * @return 信号量句柄，失败返回 NULL
 */
SemaphoreHandle_t xSemaphoreCreateMutex(void);

/**
 * @brief 创建二值信号量 (模拟 FreeRTOS xSemaphoreCreateBinary)
 * @return 信号量句柄，失败返回 NULL
 */
SemaphoreHandle_t xSemaphoreCreateBinary(void);

/**
 * @brief 释放并销毁信号量 (模拟 FreeRTOS vSemaphoreDelete)
 * @param xSemaphore 待销毁信号量句柄
 */
void vSemaphoreDelete(SemaphoreHandle_t xSemaphore);

/**
 * @brief 获取信号量/互斥锁 (模拟 FreeRTOS xSemaphoreTake)
 * @param xSemaphore 信号量句柄
 * @param xBlockTime 等待超时时间 (毫秒 / Ticks)
 * @return pdTRUE 成功获取，pdFALSE 超时失败
 */
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xBlockTime);

/**
 * @brief 归还/释放信号量或互斥锁 (模拟 FreeRTOS xSemaphoreGive)
 * @param xSemaphore 信号量句柄
 * @return pdTRUE 成功释放，pdFALSE 失败
 */
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);

// ==============================================================================
// 消息队列 API
// ==============================================================================

/**
 * @brief 创建线程安全消息队列 (模拟 FreeRTOS xQueueCreate)
 * @param uxQueueLength 队列最大容量项数
 * @param uxItemSize 单项数据大小 (字节)
 * @return 队列句柄，失败返回 NULL
 */
QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize);

/**
 * @brief 销毁消息队列
 * @param xQueue 队列句柄
 */
void vQueueDelete(QueueHandle_t xQueue);

/**
 * @brief 向队列发送数据 (模拟 FreeRTOS xQueueSend)
 * @param xQueue 队列句柄
 * @param pvItemToQueue 发送数据指针
 * @param xTicksToWait 等待超时 (毫秒)
 * @return pdPASS 成功入队，pdFAIL 队列满超时
 */
BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait);

/**
 * @brief 从队列接收数据 (模拟 FreeRTOS xQueueReceive)
 * @param xQueue 队列句柄
 * @param pvBuffer 接收数据缓冲指针
 * @param xTicksToWait 等待超时 (毫秒)
 * @return pdPASS 成功出队，pdFAIL 队列空超时
 */
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait);

// ==============================================================================
// 任务与系统控制 API
// ==============================================================================

/**
 * @brief 任务毫秒延时 (模拟 FreeRTOS vTaskDelay)
 * @param xTicksToDelay 延时毫秒数
 */
void vTaskDelay(const TickType_t xTicksToDelay);

/**
 * @brief 获取系统运行滴答数 (毫秒)
 * @return 自系统启动以来的毫秒计数
 */
TickType_t xTaskGetTickCount(void);

/**
 * @brief 创建原生线程以模拟 FreeRTOS 任务
 * @param pxTaskCode 任务函数指针
 * @param pcName 任务名称
 * @param usStackDepth 堆栈大小 (宿主机仅记录)
 * @param pvParameters 传入参数指针
 * @param uxPriority 任务优先级
 * @param pxCreatedTask 创建的任务句柄指针 (可为 NULL)
 * @return pdPASS 成功，pdFAIL 失败
 */
BaseType_t xTaskCreate(
    TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint32_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask
);

/**
 * @brief 创建并绑定指定 CPU 核心的任务 (双核模拟)
 */
BaseType_t xTaskCreatePinnedToCore(
    TaskFunction_t pxTaskCode,
    const char * const pcName,
    const uint32_t usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pxCreatedTask,
    const BaseType_t xCoreID
);

/**
 * @brief 等待模拟任务结束 (用于宿主机测试同步)
 * @param xTask 任务句柄
 * @param timeout_ms 最大等待毫秒数
 * @return true 正常退出，false 超时
 */
bool mock_task_join(TaskHandle_t xTask, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // MOCK_FREERTOS_H
