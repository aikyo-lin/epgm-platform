-- ==============================================================================
-- [ZH] EPGM 云端持久化关系数据库模式定义 (PostgreSQL 16 DDL)
-- [EN] EPGM Cloud Persistence Relational Database Schema Definition (PostgreSQL 16)
-- ==============================================================================
-- 包含核心表 / Core Tables:
-- 1. robots: 具身智能机器人设备实体资产注册与在线状态表
-- 2. telemetry_logs: 机器人高频物理遥测时序日志表 (包含事件ID、动作、即时回报、传感器读数)
-- 3. graph_snapshots: 全局分布式拓扑因果图版本快照归档表 (记录贝叶斯共轭演化历史)
-- ==============================================================================

-- ------------------------------------------------------------------------------
-- 1. 机器人设备注册表 (robots)
-- ------------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS robots (
    robot_id VARCHAR(64) PRIMARY KEY,                         -- 机器人唯一硬件/系统标识 (如 robot_alpha)
    name VARCHAR(128) NOT NULL,                               -- 设备直观显示名称
    hardware_rev VARCHAR(32) DEFAULT 'ESP32-S3-N16R8',        -- 边缘硬件版本型号
    firmware_version VARCHAR(32) DEFAULT 'v1.0.0',            -- 运行的固件版本号
    status VARCHAR(16) DEFAULT 'offline',                     -- 在线状态: 'online', 'offline', 'error'
    battery_level INTEGER DEFAULT 100,                        -- 当前剩余电池百分比 (0~100)
    last_seen_at TIMESTAMP WITH TIME ZONE,                    -- 最近一次遥测心跳上报时间
    registered_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP, -- 首次注册纳管时间
    config_json JSONB DEFAULT '{}'::jsonb                     -- 机器人可扩展配置参数 (如传感器标定参数)
);

-- 状态与最后活跃时间索引，用于集群健康度与在线设备快速检索
CREATE INDEX IF NOT EXISTS idx_robots_status ON robots(status);
CREATE INDEX IF NOT EXISTS idx_robots_last_seen ON robots(last_seen_at DESC);

-- ------------------------------------------------------------------------------
-- 2. 遥测时序日志表 (telemetry_logs)
-- ------------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS telemetry_logs (
    id BIGSERIAL PRIMARY KEY,                                 -- 自增主键
    robot_id VARCHAR(64) NOT NULL REFERENCES robots(robot_id) ON DELETE CASCADE, -- 关联设备ID
    timestamp_ms BIGINT NOT NULL,                             -- 边缘端本地毫秒时间戳
    current_event_id INTEGER NOT NULL,                        -- 当前量化感知事件ID (1~81 静态, 101~181 动态, 0 碰撞)
    last_action SMALLINT NOT NULL,                            -- 上一步决策动作索引 (0:停, 1:前, 2:后, 3:左, 4:右)
    last_reward NUMERIC(8, 4) NOT NULL,                       -- 执行上一步动作后的即时物理标量回报
    battery_percentage SMALLINT NOT NULL,                     -- 电池电量 (0~100)
    bumper_hit BOOLEAN NOT NULL DEFAULT FALSE,                -- 物理微动碰撞传感器是否触发
    sensor_distances JSONB DEFAULT '[]'::jsonb,               -- 8 方向测距传感器读数数组 (毫米 mm)
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP -- 云端落库持久化时间戳
);

-- 组合索引：按机器人与时间戳倒序索引，实现单车历史轨迹的高性能窗口切片查询
CREATE INDEX IF NOT EXISTS idx_telemetry_robot_time ON telemetry_logs (robot_id, timestamp_ms DESC);
CREATE INDEX IF NOT EXISTS idx_telemetry_created_at ON telemetry_logs (created_at DESC);

-- ------------------------------------------------------------------------------
-- 3. 全局因果拓扑图版本快照表 (graph_snapshots)
-- ------------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS graph_snapshots (
    id BIGSERIAL PRIMARY KEY,                                 -- 自增主键
    version_tag VARCHAR(64) UNIQUE NOT NULL,                  -- 全局图版本唯一标签 (如 v_42_1730386200)
    step_index BIGINT NOT NULL DEFAULT 0,                     -- 全局累计交互步数
    total_nodes INTEGER NOT NULL,                             -- 拓扑图总节点基元数 |V|
    total_edges INTEGER NOT NULL,                             -- 拓扑图总因果预测边数 |E|
    graph_json JSONB NOT NULL,                                -- 完整的全局因果图拓扑结构 (包含节点集与带权转移边集)
    trigger_robot_id VARCHAR(64),                             -- 触发本次快照生成或融合的来源机器人标识
    metrics JSONB DEFAULT '{}'::jsonb,                        -- 拓扑收敛指标 (预测命中率、平均置信度、惊奇事件数等)
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP -- 快照归档生成时间
);

-- 步数与时间索引，用于按训练进度回溯因果图演化过程
CREATE INDEX IF NOT EXISTS idx_snapshots_step ON graph_snapshots(step_index DESC);
CREATE INDEX IF NOT EXISTS idx_snapshots_created ON graph_snapshots(created_at DESC);
