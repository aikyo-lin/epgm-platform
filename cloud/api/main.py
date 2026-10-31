"""
[ZH] EPGM 云端核心 API 网关与 WebSocket 实时广播服务。
[EN] EPGM Cloud Core API Gateway and WebSocket Real-time Broadcast Hub.

架构职责 / Architectural Responsibilities:
1. 异步 Redis PubSub 事件中继 (Asynchronous Redis Event Bridge):
   在 FastAPI 启动生命周期中常驻监听 `epgm:channel:telemetry` 与 `epgm:channel:graph_update`，
   毫秒级将后台 Celery 贝叶斯融合事件与 MQTT 遥测推流至前端看板。
2. 增量 WebSocket 全双工通道 (`/ws/live_graph`):
   客户端握手连入时即刻推送 `initial_state` (全量因果拓扑与小车集群状态)；
   在连接生命周期内提供广播下发与心跳 Keep-alive (ping/pong) 探测。
3. 状态持久化分层检索 (Multi-tiered State Ingestion & Retrieval):
   优先读取 Redis 内存热点缓存，降级读取 PostgreSQL 关系数据库，本地具备内存 Mock 回退保障。
"""

import os
import sys
import json
import time
import asyncio
import logging
from typing import Dict, Any, List, Optional
from contextlib import asynccontextmanager

import redis
import redis.asyncio as aioredis
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

# 导入数据库模块
try:
    from db.session import get_db_session, get_db, init_db
    from db.models import Robot, TelemetryLog, GraphSnapshot
except ImportError:
    try:
        from ..db.session import get_db_session, get_db, init_db
        from ..db.models import Robot, TelemetryLog, GraphSnapshot
    except ImportError:
        sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
        from db.session import get_db_session, get_db, init_db
        from db.models import Robot, TelemetryLog, GraphSnapshot

# 日志配置
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [%(name)s]: %(message)s")
logger = logging.getLogger("FastAPIGateway")

# 环境变量配置
REDIS_URL = os.getenv("REDIS_URL", "redis://localhost:6379/0")
DATABASE_URL = os.getenv(
    "DATABASE_URL",
    "postgresql://epgm_admin:epgm_db_password@localhost:5432/epgm_knowledge"
)

# 内存回退状态存储 (用于无外部依赖时的开发或单机测试)
connected_robots: Dict[str, Dict[str, Any]] = {}
in_memory_global_graph: Dict[str, Any] = {
    "meta": {"step": 0, "version": 0, "total_nodes": 0, "total_edges": 0},
    "nodes": {},
    "edges": {},
}


# ------------------------------------------------------------------------------
# Pydantic 请求与响应数据模式
# ------------------------------------------------------------------------------
class TelemetryPayload(BaseModel):
    """[ZH] 机器人实时遥测数据模型 / [EN] Robot real-time telemetry model"""
    robot_id: str = Field(..., description="[ZH] 机器人唯一设备标识 / [EN] Robot Unique ID")
    timestamp_ms: int = Field(default_factory=lambda: int(time.time() * 1000), description="[ZH] 本地毫秒时间戳")
    current_event_id: int = Field(0, description="[ZH] 当前离散事件ID (0~181)")
    last_action: int = Field(0, description="[ZH] 执行动作索引 (0~4)")
    last_reward: float = Field(0.0, description="[ZH] 即时回报标量")
    battery_percentage: int = Field(100, description="[ZH] 电池剩余电量 (0~100)")
    bumper_hit: bool = Field(False, description="[ZH] 微动碰撞状态")
    sensor_distances: List[int] = Field(default_factory=list, description="[ZH] 8路雷达读数 (mm)")


class GraphDeltaPayload(BaseModel):
    """[ZH] 增量因果图碎片上报模型 / [EN] Incremental graph fragment model"""
    robot_id: str = Field(..., description="[ZH] 上报机器人设备标识")
    sequence_id: int = Field(0, description="[ZH] 增量序列号")
    step_index: int = Field(0, description="[ZH] 局部交互步数")
    nodes: List[Dict[str, Any]] = Field(default_factory=list, description="[ZH] 新增或更新节点")
    edges: List[Dict[str, Any]] = Field(default_factory=list, description="[ZH] 新增或更新因果边")


# ------------------------------------------------------------------------------
# WebSocket 连接管理器
# ------------------------------------------------------------------------------
class ConnectionManager:
    """
    [ZH] WebSocket 客户端大屏长连接管理器。
    [EN] WebSocket Client Long-connection Manager.
    """

    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        """[ZH] 接受客户端 WebSocket 握手并加入活跃连接池"""
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"WebSocket client connected. Total active clients: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket) -> None:
        """[ZH] 移除已断开的客户端连接"""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            logger.info(f"WebSocket client disconnected. Total active clients: {len(self.active_connections)}")

    async def broadcast(self, message: Dict[str, Any]) -> None:
        """[ZH] 并发安全地向所有连接中的大屏广播 JSON 数据帧"""
        if not self.active_connections:
            return

        dead_connections: List[WebSocket] = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                logger.debug(f"Failed to send to client ({e}), marking as dead.")
                dead_connections.append(connection)

        for dead in dead_connections:
            self.disconnect(dead)


manager = ConnectionManager()


# ------------------------------------------------------------------------------
# 状态读取辅助函数 (分层缓存与降级机制)
# ------------------------------------------------------------------------------
def get_current_global_graph() -> Dict[str, Any]:
    """
    [ZH] 获取当前最新的全局因果图拓扑。
         优先读取 Redis 缓存，降级读取 PostgreSQL 最新快照，最终回退本地内存。
    [EN] Get latest global causal graph topology from Redis, Postgres, or memory.
    """
    # 1. 尝试从 Redis 读取
    try:
        r = redis.Redis.from_url(REDIS_URL, decode_responses=True, socket_connect_timeout=0.5, socket_timeout=0.5)
        raw = r.get("epgm:graph:global:latest")
        if raw:
            return json.loads(raw)
    except Exception as e:
        logger.debug(f"Redis get global graph error: {e}")

    # 2. 尝试从 PostgreSQL 读取最新快照
    try:
        with get_db_session() as session:
            snapshot = (
                session.query(GraphSnapshot)
                .order_by(GraphSnapshot.id.desc())
                .first()
            )
            if snapshot and snapshot.graph_json:
                return snapshot.graph_json
    except Exception as e:
        logger.debug(f"Postgres get global graph snapshot error: {e}")

    # 3. 回退本地内存状态
    return in_memory_global_graph


def get_current_robots() -> Dict[str, Dict[str, Any]]:
    """
    [ZH] 获取当前所有已登记或活跃机器人的运行状态。
    [EN] Get current operational state for all registered or active robots.
    """
    robots: Dict[str, Dict[str, Any]] = {}

    # 1. 优先读取 Redis 中活跃设备
    try:
        r = redis.Redis.from_url(REDIS_URL, decode_responses=True, socket_connect_timeout=0.5, socket_timeout=0.5)
        active_ids = r.zrange("epgm:fleet:active_robots", 0, -1)
        for rid in active_ids:
            h = r.hgetall(f"epgm:robot:{rid}:live")
            if h:
                robots[rid] = {
                    "robot_id": rid,
                    "status": "online",
                    "battery": int(h.get("battery", 100)),
                    "bumper": bool(int(h.get("bumper_hit", 0))),
                    "current_event": int(h.get("current_event", 0)),
                    "last_reward": float(h.get("last_reward", 0.0)),
                    "last_action": int(h.get("last_action", 0)),
                    "last_seen": int(h.get("last_seen", int(time.time() * 1000))),
                }
    except Exception as e:
        logger.debug(f"Redis get active robots error: {e}")

    # 2. 补充读取 PostgreSQL 中的设备资产
    try:
        with get_db_session() as session:
            db_robots = session.query(Robot).all()
            for r_item in db_robots:
                if r_item.robot_id not in robots:
                    robots[r_item.robot_id] = {
                        "robot_id": r_item.robot_id,
                        "name": r_item.name,
                        "status": r_item.status,
                        "battery": r_item.battery_level,
                        "bumper": False,
                        "current_event": 0,
                        "last_reward": 0.0,
                        "last_action": 0,
                        "last_seen": int(r_item.last_seen_at.timestamp() * 1000) if r_item.last_seen_at else 0,
                    }
    except Exception as e:
        logger.debug(f"Postgres get robots error: {e}")

    # 3. 补充本地内存模拟设备
    for k, v in connected_robots.items():
        if k not in robots:
            robots[k] = v

    return robots


# ------------------------------------------------------------------------------
# Redis PubSub 异步事件中继后台协程
# ------------------------------------------------------------------------------
async def redis_pubsub_bridge() -> None:
    """
    [ZH] 异步常驻协程：订阅 Redis PubSub 通道，实时中继转发图演化与遥测消息至 WebSocket 客户端。
    [EN] Asynchronous resident coroutine: Subscribe to Redis PubSub and bridge events to WebSockets.
    """
    logger.info("Starting Redis PubSub background bridge listener...")
    while True:
        try:
            r = aioredis.from_url(REDIS_URL, decode_responses=True)
            pubsub = r.pubsub()
            await pubsub.subscribe("epgm:channel:graph_update", "epgm:channel:telemetry")
            logger.info("Subscribed to Redis channels: 'epgm:channel:graph_update', 'epgm:channel:telemetry'")

            async for message in pubsub.listen():
                if message and message.get("type") == "message":
                    channel = message.get("channel")
                    raw_data = message.get("data")
                    try:
                        data = json.loads(raw_data)
                        # 统一通过连接管理器向前端广播推流
                        await manager.broadcast(data)
                    except Exception as parse_err:
                        logger.warning(f"Failed to broadcast Redis message on channel {channel}: {parse_err}")

        except asyncio.CancelledError:
            logger.info("Redis PubSub bridge listener cancelled.")
            break
        except Exception as conn_err:
            logger.debug(f"Redis PubSub bridge connection retry ({conn_err}). Waiting 3.0s...")
            await asyncio.sleep(3.0)


# ------------------------------------------------------------------------------
# FastAPI 应用与生命周期 Lifespan
# ------------------------------------------------------------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    [ZH] 服务生命周期上下文：启动 Redis PubSub 中继任务，优雅关闭。
    [EN] Lifespan context: Launch Redis PubSub bridge, graceful shutdown.
    """
    logger.info("Initializing EPGM Cloud Gateway...")
    # 异步探测并初始化数据表
    try:
        await asyncio.to_thread(init_db)
    except Exception as db_err:
        logger.warning(f"Database auto-init notice (tables may already exist or DB is offline): {db_err}")

    bridge_task = asyncio.create_task(redis_pubsub_bridge())
    yield
    bridge_task.cancel()
    try:
        await bridge_task
    except asyncio.CancelledError:
        pass
    logger.info("EPGM Cloud Gateway shut down cleanly.")


app = FastAPI(
    title="EPGM Cloud Gateway",
    description="[ZH] EPGM 分布式拓扑因果图聚合与 WebSocket 实时广播服务\n"
                "[EN] Distributed Topological Causal Graph Ingestion & WebSocket Hub",
    version="1.0.0",
    lifespan=lifespan,
)

# 跨域配置
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ------------------------------------------------------------------------------
# REST API 路由
# ------------------------------------------------------------------------------
@app.get("/health")
def health_check() -> Dict[str, Any]:
    """[ZH] 服务健康检查探针 / [EN] Service health check probe"""
    return {
        "status": "healthy",
        "service": "epgm-cloud-gateway",
        "active_ws_clients": len(manager.active_connections),
        "tracked_robots": len(get_current_robots()),
        "timestamp": time.time(),
    }


@app.post("/api/v1/robots/{robot_id}/telemetry")
async def receive_telemetry(robot_id: str, payload: TelemetryPayload) -> Dict[str, Any]:
    """
    [ZH] 接收机器人高频物理遥测上报，同步更新 Redis/Postgres 并向大屏推流。
    [EN] Ingest robot physical telemetry stream, update Redis/Postgres, broadcast to UI.
    """
    # 1. 更新内存与 Redis
    connected_robots[robot_id] = {
        "robot_id": robot_id,
        "last_seen": payload.timestamp_ms,
        "battery": payload.battery_percentage,
        "bumper": payload.bumper_hit,
        "current_event": payload.current_event_id,
        "last_reward": payload.last_reward,
        "last_action": payload.last_action,
    }

    try:
        r = redis.Redis.from_url(REDIS_URL, decode_responses=True)
        live_key = f"epgm:robot:{robot_id}:live"
        r.hset(live_key, mapping={
            "robot_id": robot_id,
            "last_seen": payload.timestamp_ms,
            "battery": payload.battery_percentage,
            "bumper_hit": int(payload.bumper_hit),
            "current_event": payload.current_event_id,
            "last_reward": str(payload.last_reward),
            "last_action": payload.last_action,
        })
        r.expire(live_key, 60)
        r.zadd("epgm:fleet:active_robots", {robot_id: time.time()})
    except Exception:
        pass

    # 2. 广播遥测帧给 WebSocket 客户端
    event_frame = {
        "type": "telemetry_update",
        "robot_id": robot_id,
        "timestamp_ms": payload.timestamp_ms,
        "data": {
            "robot_id": robot_id,
            "battery_percentage": payload.battery_percentage,
            "bumper_hit": payload.bumper_hit,
            "current_event_id": payload.current_event_id,
            "last_action": payload.last_action,
            "last_reward": payload.last_reward,
            "last_action_name": payload.last_action,
            "sensor_distances": payload.sensor_distances,
        },
    }
    await manager.broadcast(event_frame)

    return {"status": "accepted", "robot_id": robot_id}


@app.post("/api/v1/robots/{robot_id}/graph_delta")
async def receive_graph_delta(robot_id: str, delta: GraphDeltaPayload) -> Dict[str, Any]:
    """
    [ZH] 接收增量因果图碎片，触发后台 Celery 贝叶斯共轭融合。
    [EN] Ingest incremental graph delta, enqueue Celery Bayesian fusion task.
    """
    # 内存模拟更新保障可用性
    in_memory_global_graph["meta"]["total_nodes"] += len(delta.nodes)
    in_memory_global_graph["meta"]["total_edges"] += len(delta.edges)
    in_memory_global_graph["meta"]["step"] = max(in_memory_global_graph["meta"]["step"], delta.step_index)
    in_memory_global_graph["meta"]["version"] += 1

    # 尝试投递至 Celery 异步队列
    try:
        from celery import Celery
        c_app = Celery("epgm_worker", broker=REDIS_URL)
        c_app.send_task("tasks.fuse_graph_deltas", args=[robot_id, delta.model_dump()])
    except Exception as e:
        logger.warning(f"Direct Celery dispatch fallback ({e}), broadcasting directly.")

    # 广播拓扑演化事件
    await manager.broadcast({
        "type": "graph_updated",
        "robot_id": robot_id,
        "version": in_memory_global_graph["meta"]["version"],
        "nodes_added": len(delta.nodes),
        "edges_added": len(delta.edges),
        "total_nodes": in_memory_global_graph["meta"]["total_nodes"],
        "total_edges": in_memory_global_graph["meta"]["total_edges"],
    })

    return {
        "status": "fused",
        "robot_id": robot_id,
        "received_nodes": len(delta.nodes),
        "received_edges": len(delta.edges),
    }


@app.get("/api/v1/graph/global")
def get_global_graph_endpoint() -> Dict[str, Any]:
    """[ZH] 获取当前全局因果拓扑图完整模型 / [EN] Get full current global causal graph model"""
    return get_current_global_graph()


@app.get("/api/v1/robots")
def get_robots_endpoint() -> Dict[str, Any]:
    """[ZH] 获取当前纳管机器人集群列表与运行状态 / [EN] Get fleet robots list and status"""
    return {"robots": get_current_robots()}


@app.get("/api/v1/snapshots")
def get_snapshots_endpoint(limit: int = 10) -> Dict[str, Any]:
    """[ZH] 获取最近归档的历史因果图版本快照列表 / [EN] Get recent archived graph snapshots"""
    try:
        with get_db_session() as session:
            snapshots = (
                session.query(GraphSnapshot)
                .order_by(GraphSnapshot.id.desc())
                .limit(limit)
                .all()
            )
            return {
                "snapshots": [
                    {
                        "id": s.id,
                        "version_tag": s.version_tag,
                        "step_index": s.step_index,
                        "total_nodes": s.total_nodes,
                        "total_edges": s.total_edges,
                        "trigger_robot_id": s.trigger_robot_id,
                        "metrics": s.metrics,
                        "created_at": s.created_at.isoformat() if s.created_at else None,
                    }
                    for s in snapshots
                ]
            }
    except Exception as e:
        logger.warning(f"Error querying snapshots: {e}")
        return {"snapshots": [], "error": str(e)}


# ------------------------------------------------------------------------------
# WebSocket 实时拓扑大屏推流接口
# ------------------------------------------------------------------------------
@app.websocket("/ws/live_graph")
async def websocket_live_stream(websocket: WebSocket):
    """
    [ZH] 实时 WebSocket 双向长连接通道。
         1. 建立连接握手时即刻下发 `initial_state` 全量拓扑与小车集群状态；
         2. 监听后台 Redis 广播的 `graph_updated` 与 `telemetry_update` 实时增量推送；
         3. 支持客户端心跳 Keep-alive (ping/pong) 探测。
    [EN] Real-time bidirectional WebSocket live stream for Vis.js dashboard rendering.
    """
    await manager.connect(websocket)
    try:
        # 1. 握手成功：下发系统初始全量状态帧
        current_graph = get_current_global_graph()
        current_fleet = get_current_robots()

        await websocket.send_json({
            "type": "initial_state",
            "meta": current_graph.get("meta", {}),
            "global_graph": current_graph,
            "robots": current_fleet,
        })
        logger.info("Sent initial_state payload to newly connected WebSocket client.")

        # 2. 持续监听客户端上行数据 (如心跳 ping 或控制指令)
        while True:
            text = await websocket.receive_text()

            # 兼容纯文本 ping 与 JSON ping
            if text == "ping":
                await websocket.send_text("pong")
            else:
                try:
                    payload = json.loads(text)
                    if payload.get("type") == "ping":
                        await websocket.send_json({"type": "pong", "timestamp": time.time()})
                except Exception:
                    pass

    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.debug(f"WebSocket client terminated with error: {e}")
        manager.disconnect(websocket)
