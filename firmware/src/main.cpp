/**
 * @file main.cpp
 * @brief [ZH] ESP32-S3 机器人边缘端主固件 (FreeRTOS 双核并发架构 + 传感器通用 HAL + LittleFS 掉电持久化)。
 *        [EN] ESP32-S3 Robot Edge Main Firmware (FreeRTOS Dual-Core + Universal HAL + LittleFS Persistence).
 *
 * 架构模型 / Architectural Model:
 *  - Core 1: 50Hz 实时控制闭环 (Control Task, 优先级 5)
 *      * 传感器数据采集 (hal_sensors_aggregator)
 *      * 差分动态事件量化 (epgm_quantize_stateful)
 *      * 5ms 互斥锁超时保底与在线局部因果拓扑更新 (Plasticity Update)
 *      * 预测动作推理 (Action Inference) 与差速电机闭环驱动
 *  - Core 0: 异步 Wi-Fi/MQTT 网络同步与 LittleFS 掉电持久化 (Network Task, 优先级 1)
 *      * 维持 Wi-Fi 与 EMQX MQTT Broker 连接
 *      * 接收并解析云端全局模型增量下发
 *      * 双缓冲内存快照 (RAM Snapshot): <2微秒瞬时复制整张图，彻底解耦慢速 Flash/MQTT I/O
 *      * 周期性 LittleFS 两阶段原子固化存盘 (CRC32 校验)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "epgm_core.h"
#include "hal_motor.h"
#include "hal_sensors.h"
#include "hal/hal_sensors_aggregator.h"
#include "storage/epgm_storage.h"
#include "../include/protocol_schema.h"

// ==============================================================================
// 配置项 / Configuration Parameters
// ==============================================================================
const char *WIFI_SSID = "EPGM_AP_ROBOT";
const char *WIFI_PASS = "epgm_wifi_password";
const char *MQTT_HOST = "192.168.1.100"; // 云端 EMQX Broker IP
const int   MQTT_PORT = 1883;

static char g_robot_id[EPGM_ROBOT_ID_LEN] = "robot_esp32_01";

// 全局因果图运行时句柄与网络导出快照双缓冲区
static epgm_runtime_t   g_epgm_runtime;   ///< Core 1 高频维护的全局活跃因果图
static epgm_runtime_t   g_graph_snapshot; ///< Core 0 专用的无锁内存快照缓冲区
static epgm_quantizer_t g_quantizer;      ///< 有状态一阶差分事件量化器

static WiFiClient     g_esp_client;
static PubSubClient   g_mqtt_client(g_esp_client);

// 线程安全遥测队列 (Core 1 发送给 Core 0)
static QueueHandle_t g_telemetry_queue = NULL;

// ==============================================================================
// Core 1: 50Hz 实时控制与局部可塑性学习任务
// ==============================================================================
void TaskControl(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz (20ms)

    epgm_sensor_frame_t sensor_frame;
    uint16_t prev_event = 0;
    uint8_t  last_action = 0;

    Serial.println("[Core 1] 50Hz 实时控制任务启动就绪 (带 5ms 互斥锁超时保底)");

    for (;;) {
        // 1. 采集全向传感器阵列数据 (ToF, 超声波, IMU, 微动)
        hal_sensors_aggregator_read(&sensor_frame);
        bool bumper_hit = (sensor_frame.bumper_hit_mask != 0) || sensor_frame.emergency_stop;

        // 2. 有状态差分量化 (输出 0:碰撞, 1~81:静态, 101~181:动态突变)
        uint16_t current_event = epgm_quantize_stateful(
            &g_quantizer,
            sensor_frame.distances_mm,
            bumper_hit
        );

        // 3. 计算即时回报 (碰撞惩罚 -10.0，打滑惩罚 -2.0，常态能耗 -0.25)
        int16_t reward_q8 = -(1 << 6); // -0.25
        if (bumper_hit) {
            reward_q8 = -(10 << 8); // -10.0
        } else if (sensor_frame.imu.is_skidding || sensor_frame.imu.is_stalled) {
            reward_q8 = -(2 << 8);  // -2.0
        }

        // 4. 并发安全局部因果拓扑更新与推理决策 (带 5ms 超时保底)
        bool is_hit = false;
        uint8_t next_action = last_action; // 默认保底

        if (epgm_mutex_lock(&g_epgm_runtime, 5)) {
            if (prev_event != 0) {
                is_hit = epgm_plasticity_update(&g_epgm_runtime, prev_event, last_action, current_event, reward_q8);
            }
            next_action = epgm_infer_action(&g_epgm_runtime, current_event);
            epgm_mutex_unlock(&g_epgm_runtime);
        } else {
            // 获取互斥锁超时: 说明 Core 0 正在瞬时拷贝快照，立即执行上一动作安全降级，不破坏 20ms 硬实时
            next_action = last_action;
        }

        // 5. 驱动执行电机
        hal_motor_set_action(next_action);

        // 6. 若发生碰撞、打滑或新拓扑生枝 (Surprise)，压入遥测队列
        if (bumper_hit || !is_hit || sensor_frame.imu.is_skidding) {
            epgm_telemetry_packet_t packet;
            packet.protocol_ver = EPGM_PROTOCOL_VERSION;
            strncpy(packet.robot_id, g_robot_id, EPGM_ROBOT_ID_LEN);
            packet.timestamp_ms = millis();
            packet.current_event_id = current_event;
            packet.last_action = next_action;
            packet.last_reward_q8 = reward_q8;
            packet.battery_percentage = 95;
            packet.bumper_state = bumper_hit ? 1 : 0;
            memcpy(packet.sensor_distances, sensor_frame.distances_mm, sizeof(packet.sensor_distances));

            if (g_telemetry_queue) {
                xQueueSend(g_telemetry_queue, &packet, 0); // 非阻塞入队
            }
        }

        prev_event = current_event;
        last_action = next_action;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================================================
// Core 0: 异步 Wi-Fi、MQTT 网络同步与 LittleFS 掉电持久化任务
// ==============================================================================
void TaskNetwork(void *pvParameters) {
    Serial.println("[Core 0] 异步网络同步与持久化任务启动就绪");

    g_mqtt_client.setServer(MQTT_HOST, MQTT_PORT);

    char topic_telemetry[64];
    snprintf(topic_telemetry, sizeof(topic_telemetry), "epgm/robot/%s/telemetry", g_robot_id);

    epgm_telemetry_packet_t rx_packet;
    uint32_t last_persist_tick = millis();

    for (;;) {
        // 维持 Wi-Fi 连接
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 维持 MQTT Broker 连接
        if (!g_mqtt_client.connected()) {
            if (g_mqtt_client.connect(g_robot_id)) {
                Serial.println("[MQTT] 成功连上云端 EMQX Broker");
            } else {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
        }

        g_mqtt_client.loop();

        // 1. 处理待发送的遥测与图碎片队列
        if (xQueueReceive(g_telemetry_queue, &rx_packet, pdMS_TO_TICKS(50)) == pdTRUE) {
            JsonDocument doc;
            doc["robot_id"] = rx_packet.robot_id;
            doc["timestamp_ms"] = rx_packet.timestamp_ms;
            doc["current_event_id"] = rx_packet.current_event_id;
            doc["last_action"] = rx_packet.last_action;
            doc["last_reward"] = (float)rx_packet.last_reward_q8 / 256.0f;
            doc["battery_percentage"] = rx_packet.battery_percentage;
            doc["bumper_hit"] = (rx_packet.bumper_state != 0);

            // [ZH] 序列化 8 路空间雷达测距读数 (mm) 对齐全链路遥测 JSON Schema
            // [EN] Serialize 8-channel distance readings (mm) to align with telemetry JSON schema
            JsonArray dist_array = doc["sensor_distances"].to<JsonArray>();
            for (int i = 0; i < 8; ++i) {
                dist_array.add(rx_packet.sensor_distances[i]);
            }

            char buffer[512];
            size_t len = serializeJson(doc, buffer);
            g_mqtt_client.publish(topic_telemetry, buffer, len);
        }

        // 2. 双缓冲内存快照与 LittleFS 周期性掉电存盘 (每 30 秒执行一次)
        if (millis() - last_persist_tick > 30000) {
            last_persist_tick = millis();
            // 在微秒级持有互斥锁完成瞬时内存快照拷贝
            if (epgm_snapshot_copy(&g_epgm_runtime, &g_graph_snapshot, 10)) {
                // 在完全释放互斥锁后，安全执行耗时 Flash 写入，绝不阻塞 Core 1 实时控制！
                int res = epgm_storage_save_atomic(&g_graph_snapshot, NULL);
                if (res == EPGM_STORAGE_OK) {
                    Serial.printf("[LittleFS] 因果图原子存盘成功 (活跃边数: %u)\n", g_graph_snapshot.active_edge_count);
                } else {
                    Serial.printf("[LittleFS] 因果图原子存盘失败, 错误码: %d\n", res);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==============================================================================
// Arduino 框架启动入口
// ==============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("==================================================");
    Serial.println(" EPGM Robot Edge Firmware Booting (ESP32-S3)");
    Serial.println("==================================================");

    // 1. 初始化电机与通用传感器聚合 HAL
    hal_motor_init();
    hal_sensors_aggregator_init();

    // 2. 初始化 EPGM 因果图运行时、量化器与并发互斥锁
    epgm_init(&g_epgm_runtime);
    epgm_quantizer_init(&g_quantizer);
    epgm_mutex_init(&g_epgm_runtime);
    memset(&g_graph_snapshot, 0, sizeof(epgm_runtime_t));

    // 3. 冷启动检查并恢复 LittleFS Flash 持久化因果图
    int load_res = epgm_storage_load(&g_epgm_runtime, NULL);
    if (load_res == EPGM_STORAGE_OK) {
        Serial.printf("[LittleFS] 冷启动成功恢复历史因果图! 边数: %u, 步数: %u, 命中: %u\n",
                      g_epgm_runtime.active_edge_count,
                      g_epgm_runtime.step_counter,
                      g_epgm_runtime.total_hits);
    } else {
        Serial.printf("[LittleFS] 未发现历史因果图或校验失败 (代码 %d)，初始化为空白图运行\n", load_res);
    }

    // 4. 创建 FreeRTOS 遥测通信队列
    g_telemetry_queue = xQueueCreate(16, sizeof(epgm_telemetry_packet_t));

    // 5. 创建并绑定双核多任务 (Core 1: 控制; Core 0: 网络与存盘)
    xTaskCreatePinnedToCore(TaskControl, "TaskControl", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskNetwork, "TaskNetwork", 4096, NULL, 1, NULL, 0);
}

void loop() {
    // FreeRTOS 任务已全面接管调度，主循环休眠
    vTaskDelay(pdMS_TO_TICKS(1000));
}
