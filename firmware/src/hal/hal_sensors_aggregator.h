/**
 * @file hal_sensors_aggregator.h
 * @brief [ZH] 机器人全向传感器统一聚合抽象层头文件。
 *        [EN] Robot Omnidirectional Sensor Unified Aggregator HAL Header.
 *
 * 架构模型 / Architectural Model:
 *  - 集中纳管 8 路 ToF 激光、超声波测距、6轴 IMU 与触须微动；
 *  - 统一解算输出标准 8 向毫米级空间距离向量 (0:正前, 1:右前, 2:正右, 3:右后, 4:正后, 5:左后, 6:正左, 7:左前)；
 *  - 输出标准 epgm_sensor_frame_t 帧结构体，直接对接 EPGM 量化器与上层决策。
 */

#ifndef HAL_SENSORS_AGGREGATOR_H
#define HAL_SENSORS_AGGREGATOR_H

#include "hal_device.h"
#include "hal_vl53lxx.h"
#include "hal_hcsr04.h"
#include "hal_imu.h"
#include "hal_tca9548a.h"
#include "hal_bumper.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EPGM_SECTORS_COUNT 8

/**
 * @brief [ZH] 统一传感器帧数据结构 (与 Survey 3 架构规范 100% 对齐)
 *        [EN] Unified Sensor Frame Data Structure
 */
typedef struct {
    uint16_t        distances_mm[EPGM_SECTORS_COUNT]; ///< 8 个方向测距读数 (mm)
    epgm_imu_data_t imu;                              ///< 6轴 IMU 姿态与打滑动力学
    uint8_t         bumper_hit_mask;                  ///< 碰撞触须掩码 (bit0:前左, bit1:前右, bit2:后方)
    bool            emergency_stop;                   ///< 物理急停触发标志
} epgm_sensor_frame_t;

/**
 * @brief 初始化传感器聚合器并自动挂载/配置所有标准外设驱动
 * @return HAL_OK 成功
 */
int hal_sensors_aggregator_init(void);

/**
 * @brief 一键同步采集所有传感器数据并填入统一帧
 * @param frame 输出传感器帧结构体指针
 * @return HAL_OK 成功
 */
int hal_sensors_aggregator_read(epgm_sensor_frame_t *frame);

/**
 * @brief 便捷函数: 读取 8 向测距距离 (mm)
 * @param distances 长度为 8 的输出缓冲
 */
void hal_sensors_aggregator_read_distances(uint16_t distances[EPGM_SECTORS_COUNT]);

/**
 * @brief 便捷函数: 检查是否触发碰撞微动
 * @return true 触发碰撞
 */
bool hal_sensors_aggregator_check_bumper(void);

/**
 * @brief 释放注销所有聚合外设资源
 */
void hal_sensors_aggregator_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_SENSORS_AGGREGATOR_H
