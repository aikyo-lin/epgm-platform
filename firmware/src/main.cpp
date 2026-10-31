/**
 * @file main.cpp
 * @brief [ZH] ESP32-S3 机器人边缘端主固件 (FreeRTOS 双核并发架构)。
 *        [EN] ESP32-S3 Robot Edge Main Firmware (FreeRTOS Dual-Core Architecture).
 *
 * 架构模型 / Architectural Model:
 *  - Core 1: 50Hz 实时感知-动作-局部因果学习闭环 (Control Task)
 *  - Core 0: 异步 Wi-Fi 与 MQTT 端云通信及全局模型同步 (Network Task)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "epgm_core.h"
#include "hal_motor.h"
#include "hal_sensors.h"
#include "../include/protocol_schema.h"

// ==============================================================================
// 配置项 / Configuration Parameters
// ==============================================================================
const char *WIFI_SSID = "EPGM_AP_ROBOT";
const char *WIFI_PASS = "epgm_wifi_password";
const char *MQTT_HOST = "192.168.1.100"; // 云端 EMQX Broker IP
const int   MQTT_PORT = 1883;

static char g_robot_id[EPGM_ROBOT_ID_LEN] = "robot_esp32_01";

// 全局运行时与网络客户端
static epgm_runtime_t g_epgm_runtime;
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

    uint16_t distances[8];
    uint16_t prev_event = 0;
    uint8_t  last_action = 0;

    Serial.println("[Core 1] 50Hz 实时控制任务启动就绪");

    for (;;) {
        // 1. 采集传感器与碰撞检测 / Read sensors
        hal_sensors_read_distances(distances);
        bool bumper_hit = hal_sensors_check_bumper();

        // 2. 离散事件量化 / Quantize event
        uint16_t current_event = epgm_quantize(distances, bumper_hit);

        // 3. 计算即时回报 / Compute immediate reward
        int16_t reward_q8 = bumper_hit ? -(10 << 8) : -(1 << 6); // 碰撞惩罚 -10.0，常态移动耗费 -0.25

        // 4. 局部因果拓扑更新 (如果在运动中)
        bool is_hit = false;
        if (prev_event != 0) {
            is_hit = epgm_plasticity_update(&g_epgm_runtime, prev_event, last_action, current_event, reward_q8);
        }

        // 5. EPGM 推理决策下一步动作 / Action inference
        uint8_t next_action = epgm_infer_action(&g_epgm_runtime, current_event);

        // 6. 驱动执行电机 / Actuate motors
        hal_motor_set_action(next_action);

        // 7. 若发生碰撞或高惊喜度事件，压入队列触发后台 MQTT 上报
        if (bumper_hit || !is_hit) {
            epgm_telemetry_packet_t packet;
            packet.protocol_ver = EPGM_PROTOCOL_VERSION;
            strncpy(packet.robot_id, g_robot_id, EPGM_ROBOT_ID_LEN);
            packet.timestamp_ms = millis();
            packet.current_event_id = current_event;
            packet.last_action = next_action;
            packet.last_reward_q8 = reward_q8;
            packet.battery_percentage = 95;
            packet.bumper_state = bumper_hit ? 1 : 0;
            memcpy(packet.sensor_distances, distances, sizeof(distances));

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
// Core 0: 异步 Wi-Fi 与 MQTT 网络同步任务
// ==============================================================================
void TaskNetwork(void *pvParameters) {
    Serial.println("[Core 0] 异步网络同步任务启动就绪");

    g_mqtt_client.setServer(MQTT_HOST, MQTT_PORT);

    char topic_telemetry[64];
    snprintf(topic_telemetry, sizeof(topic_telemetry), "epgm/robot/%s/telemetry", g_robot_id);

    epgm_telemetry_packet_t rx_packet;

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

        // 处理待发送的遥测与图碎片队列
        if (xQueueReceive(g_telemetry_queue, &rx_packet, pdMS_TO_TICKS(50)) == pdTRUE) {
            // 将二进制数据包转换为轻量 JSON 上报
            JsonDocument doc;
            doc["robot_id"] = rx_packet.robot_id;
            doc["timestamp_ms"] = rx_packet.timestamp_ms;
            doc["current_event_id"] = rx_packet.current_event_id;
            doc["last_action"] = rx_packet.last_action;
            doc["last_reward"] = (float)rx_packet.last_reward_q8 / 256.0f;
            doc["battery_percentage"] = rx_packet.battery_percentage;
            doc["bumper_hit"] = (rx_packet.bumper_state != 0);

            char buffer[256];
            size_t len = serializeJson(doc, buffer);
            g_mqtt_client.publish(topic_telemetry, buffer, len);
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

    // 初始化硬件驱动与 EPGM 运行时
    hal_motor_init();
    hal_sensors_init();
    epgm_init(&g_epgm_runtime);

    // 创建 FreeRTOS 队列
    g_telemetry_queue = xQueueCreate(16, sizeof(epgm_telemetry_packet_t));

    // 创建并绑定双核多任务
    xTaskCreatePinnedToCore(TaskControl, "TaskControl", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskNetwork, "TaskNetwork", 4096, NULL, 1, NULL, 0);
}

void loop() {
    // FreeRTOS 任务已全面托管，主循环休眠
    vTaskDelay(pdMS_TO_TICKS(1000));
}
