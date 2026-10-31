"""
[ZH] EPGM 云端常驻 MQTT 消息中继网关与双路数据摄入中继器。
[EN] EPGM Cloud Resident MQTT Message Relay Gateway and Dual-Path Ingestion Dispatcher.

架构设计职责 / Architectural Responsibilities:
1. 可靠连接与自愈退避 (Auto-reconnection with Exponential Backoff & Jitter):
   连接 EMQX IoT Broker，遇网络中断或节点漂移自动带抖动指数退避重连。
2. 双协议解包 (Dual-protocol Deserialization):
   首选 JSON 字符串解析，失败时自动兼容嵌入式二进制结构体 (Packed Struct)。
3. 双路数据路由 (Dual-path Ingestion Routing):
   - 遥测流 (Telemetry Stream):
     高频刷新 Redis 实时哈希缓存与活跃机器人集合 (TTL 60s)；
     向 Redis PubSub 广播频道 `epgm:channel:telemetry` 推送轻量级遥测事件；
     写入/批量归档至 PostgreSQL `telemetry_logs` 表，同步更新 `robots` 状态。
   - 图碎片流 (Graph Delta Stream):
     解析因果拓扑碎片，分发 Celery 异步图融合任务 `tasks.fuse_graph_deltas`。
4. 仿真兼容性 (Backward Compatibility):
   保留 `CloudGraphAggregator` 类，确保现有 `swarm_simulation.py` 科学评测平滑运行。
"""

import os
import sys
import json
import time
import struct
import random
import logging
from typing import Dict, Any, List, Optional, Tuple

import redis
from celery import Celery
import paho.mqtt.client as mqtt

# 导入数据库模块
try:
    from .db.session import get_db_session, init_db
    from .db.models import Robot, TelemetryLog
except ImportError:
    # 支持从 cloud 根目录直接执行 `python mqtt_bridge.py`
    from db.session import get_db_session, init_db
    from db.models import Robot, TelemetryLog

# 引入算法库以兼容 CloudGraphAggregator 单机模拟
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "epgm-algorithm")))
try:
    from epgm.core import EventPredictiveGraph, EPGMEdge
except ImportError:
    EventPredictiveGraph = None
    EPGMEdge = None

# 日志配置
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(name)s]: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("EPGMMQTTBridge")

# 环境变量配置
MQTT_HOST = os.getenv("MQTT_BROKER_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_BROKER_PORT", 1883))
MQTT_KEEPALIVE = int(os.getenv("MQTT_KEEPALIVE", 60))
REDIS_URL = os.getenv("REDIS_URL", "redis://localhost:6379/0")
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://epgm_admin:epgm_db_password@localhost:5432/epgm_knowledge")

# 二进制协议格式常量 (对应 protocol_schema.h)
# typedef struct __attribute__((packed)) { uint8_t ver; char id[16]; uint32_t ts; uint16_t evt; uint8_t act; int16_t rew; uint8_t bat; uint8_t bmp; uint16_t dist[8]; }
TELEMETRY_PACKET_FORMAT = "<B16sIHBhBB8H"
TELEMETRY_PACKET_SIZE = struct.calcsize(TELEMETRY_PACKET_FORMAT)  # 44 字节

# typedef struct __attribute__((packed)) { uint8_t ver; char id[16]; uint32_t seq; uint16_t cnt; uint32_t step; }
DELTA_HEADER_FORMAT = "<B16sIHI"
DELTA_HEADER_SIZE = struct.calcsize(DELTA_HEADER_FORMAT)  # 27 字节

# typedef struct __attribute__((packed)) { uint16_t src; uint8_t act; uint16_t dst; int16_t rew; uint16_t cnt; uint8_t conf; }
DELTA_EDGE_FORMAT = "<HBHhHB"
DELTA_EDGE_SIZE = struct.calcsize(DELTA_EDGE_FORMAT)  # 10 字节


class EPGMMQTTBridge:
    """
    [ZH] 生产级常驻 MQTT 消息中继网关。
    [EN] Production-grade Resident MQTT Message Relay Gateway.
    """

    def __init__(
        self,
        broker_host: str = MQTT_HOST,
        broker_port: int = MQTT_PORT,
        redis_url: str = REDIS_URL,
        client_id: str = "epgm_cloud_bridge",
    ):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client_id = client_id

        # 初始化 Redis 客户端
        self.redis_client = redis.Redis.from_url(redis_url, decode_responses=True)

        # 初始化 Celery 客户端 (任务分发代理)
        self.celery_app = Celery("epgm_worker", broker=redis_url, backend=redis_url)

        # 初始化 Paho MQTT 客户端 (自适应 Paho 1.x 与 2.x API)
        if hasattr(mqtt, "CallbackAPIVersion"):
            self.client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION1,
                client_id=self.client_id,
                clean_session=False,
            )
        else:
            self.client = mqtt.Client(client_id=self.client_id, clean_session=False)

        # 注册 MQTT 事件回调
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

        # 重连退避参数
        self.base_delay: float = 1.0
        self.max_delay: float = 30.0
        self.current_delay: float = self.base_delay

        # 运行状态
        self.is_running: bool = False

    def on_connect(self, client: mqtt.Client, userdata: Any, flags: Any, rc: int) -> None:
        """
        [ZH] MQTT Broker 连接建立回调。
        [EN] MQTT Broker connection established callback.
        """
        if rc == 0:
            logger.info(f"Connected to EMQX MQTT Broker ({self.broker_host}:{self.broker_port}) successfully.")
            self.current_delay = self.base_delay

            # 订阅群体机器人遥测流与图碎片主题 (通配符 + 匹配所有设备)
            telemetry_topic = "epgm/robot/+/telemetry"
            delta_topic = "epgm/robot/+/graph_delta"

            client.subscribe(telemetry_topic, qos=0)
            client.subscribe(delta_topic, qos=1)
            logger.info(f"Subscribed to MQTT topics: '{telemetry_topic}' (QoS 0), '{delta_topic}' (QoS 1)")
        else:
            logger.error(f"Failed to connect to MQTT broker, return code: {rc}")

    def on_disconnect(self, client: mqtt.Client, userdata: Any, rc: int) -> None:
        """
        [ZH] MQTT Broker 连接断开回调。
        [EN] MQTT Broker disconnection callback.
        """
        if rc != 0:
            logger.warning(f"Unexpected MQTT disconnection (rc={rc}). Will attempt reconnection.")
        else:
            logger.info("MQTT client disconnected gracefully.")

    def on_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        """
        [ZH] 接收到 MQTT 订阅消息的统一分发入口。
        [EN] Unified dispatch handler for incoming subscribed MQTT messages.
        """
        try:
            topic = msg.topic
            payload = msg.payload
            parts = topic.split("/")

            # 严格匹配规范主题: epgm/robot/{robot_id}/{msg_type}
            if len(parts) != 4 or parts[0] != "epgm" or parts[1] != "robot":
                logger.warning(f"Received message with non-standard topic structure: {topic}")
                return

            robot_id = parts[2]
            msg_type = parts[3]

            if msg_type == "telemetry":
                self.handle_telemetry(robot_id, payload)
            elif msg_type == "graph_delta":
                self.handle_graph_delta(robot_id, payload)
            else:
                logger.debug(f"Ignored unhandled message type '{msg_type}' for robot '{robot_id}'")

        except Exception as e:
            logger.error(f"Error processing message from topic '{msg.topic}': {e}", exc_info=True)

    def unpack_telemetry(self, raw_payload: bytes) -> Optional[Dict[str, Any]]:
        """
        [ZH] 遥测包反序列化，优先 JSON，降级二进制结构体。
        [EN] Telemetry packet deserialization, JSON first, fallback to binary struct.
        """
        # 1. 尝试 JSON 解码
        try:
            return json.loads(raw_payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            pass

        # 2. 尝试二进制 Packed Struct 解包
        if len(raw_payload) == TELEMETRY_PACKET_SIZE:
            try:
                (
                    ver,
                    raw_id,
                    ts,
                    evt_id,
                    act,
                    rew_q8,
                    battery,
                    bumper,
                    d0,
                    d1,
                    d2,
                    d3,
                    d4,
                    d5,
                    d6,
                    d7,
                ) = struct.unpack(TELEMETRY_PACKET_FORMAT, raw_payload)
                robot_id = raw_id.decode("utf-8", errors="ignore").rstrip("\x00")
                return {
                    "protocol_ver": ver,
                    "robot_id": robot_id,
                    "timestamp_ms": ts,
                    "current_event_id": evt_id,
                    "last_action": act,
                    "last_reward": rew_q8 / 256.0,  # Q8.8 转换浮点
                    "battery_percentage": battery,
                    "bumper_hit": bool(bumper),
                    "sensor_distances": [d0, d1, d2, d3, d4, d5, d6, d7],
                }
            except Exception as e:
                logger.warning(f"Failed to unpack binary telemetry packet: {e}")

        logger.error(f"Unsupported payload format for telemetry (len={len(raw_payload)})")
        return None

    def unpack_graph_delta(self, raw_payload: bytes, robot_id: str) -> Optional[Dict[str, Any]]:
        """
        [ZH] 图碎片反序列化，优先 JSON，降级二进制结构体。
        [EN] Graph delta deserialization, JSON first, fallback to binary struct.
        """
        # 1. 尝试 JSON 解码
        try:
            delta = json.loads(raw_payload.decode("utf-8"))
            if "robot_id" not in delta:
                delta["robot_id"] = robot_id
            return delta
        except (UnicodeDecodeError, json.JSONDecodeError):
            pass

        # 2. 尝试二进制包头 + 边数组解包
        if len(raw_payload) >= DELTA_HEADER_SIZE:
            try:
                ver, raw_id, seq_id, edge_count, step_index = struct.unpack(
                    DELTA_HEADER_FORMAT, raw_payload[:DELTA_HEADER_SIZE]
                )
                parsed_robot_id = raw_id.decode("utf-8", errors="ignore").rstrip("\x00") or robot_id

                edges = []
                offset = DELTA_HEADER_SIZE
                for _ in range(edge_count):
                    if offset + DELTA_EDGE_SIZE > len(raw_payload):
                        break
                    src, act, dst, rew_q8, cnt, conf_u8 = struct.unpack(
                        DELTA_EDGE_FORMAT, raw_payload[offset : offset + DELTA_EDGE_SIZE]
                    )
                    edges.append({
                        "source_event": src,
                        "action": act,
                        "target_event": dst,
                        "expected_reward": round(rew_q8 / 256.0, 4),
                        "observation_count": cnt,
                        "confidence": round(conf_u8 / 255.0, 4),
                    })
                    offset += DELTA_EDGE_SIZE

                return {
                    "protocol_ver": ver,
                    "robot_id": parsed_robot_id,
                    "sequence_id": seq_id,
                    "step_index": step_index,
                    "nodes": [],
                    "edges": edges,
                }
            except Exception as e:
                logger.warning(f"Failed to unpack binary graph delta: {e}")

        logger.error(f"Unsupported payload format for graph delta (len={len(raw_payload)})")
        return None

    def handle_telemetry(self, robot_id: str, raw_payload: bytes) -> None:
        """
        [ZH] 处理单车高频物理遥测数据流 (更新 Redis 缓存、发布 PubSub、入库 PostgreSQL)。
        [EN] Process robot high-frequency physical telemetry stream.
        """
        data = self.unpack_telemetry(raw_payload)
        if not data:
            return

        now_ms = int(time.time() * 1000)
        timestamp_ms = data.get("timestamp_ms", now_ms)
        battery = data.get("battery_percentage", 100)
        bumper = bool(data.get("bumper_hit", False))
        event_id = int(data.get("current_event_id", 0))
        last_action = int(data.get("last_action", 0))
        last_reward = float(data.get("last_reward", 0.0))
        sensor_distances = data.get("sensor_distances", [])

        # ----------------------------------------------------------------------
        # 1. 刷新 Redis 实时热点缓存
        # ----------------------------------------------------------------------
        try:
            live_key = f"epgm:robot:{robot_id}:live"
            pipe = self.redis_client.pipeline()
            pipe.hset(
                live_key,
                mapping={
                    "robot_id": robot_id,
                    "last_seen": now_ms,
                    "battery": battery,
                    "bumper_hit": int(bumper),
                    "current_event": event_id,
                    "last_action": last_action,
                    "last_reward": str(last_reward),
                    "sensor_distances": json.dumps(sensor_distances),
                },
            )
            pipe.expire(live_key, 60)  # 60 秒 TTL 自动淘汰失联节点

            # 更新活跃机器人有序集合 (score 为时间戳)
            pipe.zadd("epgm:fleet:active_robots", {robot_id: time.time()})

            # 发布轻量实时广播消息给 FastAPI WebSocket Hub
            pipe.publish(
                "epgm:channel:telemetry",
                json.dumps({
                    "type": "telemetry_update",
                    "robot_id": robot_id,
                    "timestamp_ms": timestamp_ms,
                    "data": {
                        "robot_id": robot_id,
                        "battery_percentage": battery,
                        "bumper_hit": bumper,
                        "current_event_id": event_id,
                        "last_action": last_action,
                        "last_reward": last_reward,
                        "sensor_distances": sensor_distances,
                    },
                }),
            )
            pipe.execute()
        except Exception as e:
            logger.warning(f"Redis telemetry cache write error for {robot_id}: {e}")

        # ----------------------------------------------------------------------
        # 2. 持久化落库 PostgreSQL (robots 表状态刷新 + telemetry_logs 写入)
        # ----------------------------------------------------------------------
        try:
            with get_db_session() as session:
                # 检查机器人设备是否已注册，若无则自动注册纳管
                robot = session.query(Robot).filter_by(robot_id=robot_id).first()
                if not robot:
                    robot = Robot(
                        robot_id=robot_id,
                        name=f"EPGM Agent ({robot_id})",
                        status="online",
                        battery_level=battery,
                    )
                    session.add(robot)
                else:
                    robot.status = "online"
                    robot.battery_level = battery

                # 插入遥测时序日志
                telemetry_log = TelemetryLog(
                    robot_id=robot_id,
                    timestamp_ms=timestamp_ms,
                    current_event_id=event_id,
                    last_action=last_action,
                    last_reward=last_reward,
                    battery_percentage=battery,
                    bumper_hit=bumper,
                    sensor_distances=sensor_distances,
                )
                session.add(telemetry_log)
        except Exception as e:
            logger.warning(f"PostgreSQL telemetry logging error for {robot_id}: {e}")

    def handle_graph_delta(self, robot_id: str, raw_payload: bytes) -> None:
        """
        [ZH] 处理小车上报的因果拓扑碎片，分发 Celery 异步贝叶斯图融合任务。
        [EN] Handle uploaded causal graph delta, dispatching Celery async Bayesian fusion task.
        """
        delta = self.unpack_graph_delta(raw_payload, robot_id)
        if not delta:
            return

        logger.info(
            f"Ingested graph delta from [{robot_id}]: "
            f"{len(delta.get('nodes', []))} nodes, {len(delta.get('edges', []))} edges. "
            f"Enqueuing Celery Bayesian fusion task..."
        )

        try:
            # 异步非阻塞推入 Celery 任务队列
            self.celery_app.send_task("tasks.fuse_graph_deltas", args=[robot_id, delta])
        except Exception as e:
            logger.error(f"Failed to enqueue Celery fusion task for robot {robot_id}: {e}", exc_info=True)

    def run(self) -> None:
        """
        [ZH] 启动常驻运行循环，包含指数退避自动重连。
        [EN] Start resident run loop with exponential backoff and jittered reconnect.
        """
        self.is_running = True
        logger.info("Starting EPGM MQTT Bridge resident process...")

        while self.is_running:
            try:
                logger.info(f"Connecting to MQTT Broker at {self.broker_host}:{self.broker_port}...")
                self.client.connect(self.broker_host, self.broker_port, keepalive=MQTT_KEEPALIVE)
                # 阻塞直到显式断开
                self.client.loop_forever()
            except KeyboardInterrupt:
                logger.info("MQTT Bridge stopped by operator.")
                self.is_running = False
                break
            except Exception as e:
                # 增加随机抖动防止并发雪崩 (Jitter: 0~0.5s)
                jitter = random.uniform(0.0, 0.5)
                wait_time = self.current_delay + jitter
                logger.error(f"MQTT connection failed ({e}). Reconnecting in {wait_time:.2f}s...")
                time.sleep(wait_time)
                # 指数递增退避时间
                self.current_delay = min(self.current_delay * 2, self.max_delay)

        self.client.disconnect()
        logger.info("EPGM MQTT Bridge exited.")


class CloudGraphAggregator:
    """
    [ZH] 兼容现有仿真与评测脚本的单机因果图聚合器封装。
    [EN] Standalone Causal Graph Aggregator wrapper maintaining backward compatibility.
    """

    def __init__(self, action_dim: int = 5):
        if EventPredictiveGraph is not None:
            self.global_graph = EventPredictiveGraph(action_dim=action_dim)
        else:
            self.global_graph = None
        self.registered_robots: Dict[str, Dict[str, Any]] = {}
        self.total_fragments_fused: int = 0
        self.last_sync_timestamp: float = time.time()

    def process_telemetry(self, robot_id: str, payload: Dict[str, Any]) -> None:
        """
        [ZH] 模拟处理遥测。
        [EN] Simulate processing telemetry.
        """
        self.registered_robots[robot_id] = {
            "last_seen": time.time(),
            "battery": payload.get("battery_percentage", 100),
            "current_event": payload.get("current_event_id", 0),
            "bumper_hit": payload.get("bumper_hit", False),
            "last_reward": payload.get("last_reward", 0.0),
        }

    def process_graph_delta(self, robot_id: str, fragment: Dict[str, Any]) -> Dict[str, Any]:
        """
        [ZH] 执行因果图碎片融合。
        [EN] Execute causal graph fragment fusion.
        """
        if self.global_graph is not None:
            nodes_up, edges_up = self.global_graph.import_fragment(fragment)
            self.total_fragments_fused += 1

            total_edges = sum(len(el) for el in self.global_graph.adjacency.values())
            print(
                f"[Cloud Fusion] 来自小车 [{robot_id}] 的图碎片已融合: "
                f"+{nodes_up} 节点, +{edges_up} 因果边 | "
                f"全局累计节点: {len(self.global_graph.nodes)}, 全局因果边: {total_edges}"
            )
            return {
                "status": "success",
                "nodes_updated": nodes_up,
                "edges_updated": edges_up,
                "global_total_nodes": len(self.global_graph.nodes),
                "global_total_edges": total_edges,
            }
        return {"status": "success", "nodes_updated": 0, "edges_updated": 0}

    def get_distilled_global_model(self) -> Dict[str, Any]:
        """
        [ZH] 导出提炼优化后的全局因果图模型。
        [EN] Export distilled and optimized global causal model.
        """
        if self.global_graph is not None:
            return self.global_graph.export_fragment(since_step=0)
        return {"meta": {"node_count": 0, "edge_count": 0}, "nodes": [], "edges": []}


if __name__ == "__main__":
    bridge = EPGMMQTTBridge()
    bridge.run()
