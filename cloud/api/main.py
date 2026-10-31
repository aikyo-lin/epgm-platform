"""
[ZH] EPGM 云端核心 API 网关与 WebSocket 实时广播服务。
[EN] EPGM Cloud Core API Gateway and WebSocket Real-time Broadcast Service.
"""

from typing import Dict, Any, List
import json
import os
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

app = FastAPI(
    title="EPGM Cloud Gateway",
    description="[ZH] 面向具身智能机器人的分布式拓扑因果图聚合与遥测网关\n[EN] Distributed Topological Causal Graph Ingestion & Telemetry Gateway",
    version="0.1.0",
)

# 允许跨域以便可视化前端直接调用 / CORS Middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 内存模拟全局状态库 (真实生产环境连接 Postgres / Redis)
# In-memory mock storage (connected to Postgres/Redis in full production)
connected_robots: Dict[str, Dict[str, Any]] = {}
global_graph: Dict[str, Any] = {
    "meta": {"step": 0, "node_count": 0, "edge_count": 0},
    "nodes": [],
    "edges": [],
}


class TelemetryPayload(BaseModel):
    """[ZH] 机器人实时遥测数据模型 / [EN] Robot real-time telemetry model"""
    robot_id: str = Field(..., description="机器人唯一设备标识")
    timestamp_ms: int = Field(..., description="本地毫秒时间戳")
    current_event_id: int = Field(..., description="当前离散事件ID")
    last_action: int = Field(..., description="执行动作索引")
    last_reward: float = Field(..., description="即时回报标量")
    battery_percentage: int = Field(100, description="电池剩余电量")
    bumper_hit: bool = Field(False, description="微动碰撞状态")
    sensor_distances: List[float] = Field(default_factory=list, description="测距雷达读数")


class GraphDeltaPayload(BaseModel):
    """[ZH] 增量因果图碎片上报模型 / [EN] Incremental graph fragment model"""
    robot_id: str
    sequence_id: int
    step_index: int
    nodes: List[Dict[str, Any]] = Field(default_factory=list)
    edges: List[Dict[str, Any]] = Field(default_factory=list)


# ------------------------------------------------------------------------------
# WebSocket 连接管理器 (用于向大屏实时推流)
# ------------------------------------------------------------------------------
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception:
                pass

manager = ConnectionManager()


# ------------------------------------------------------------------------------
# REST API 路由
# ------------------------------------------------------------------------------
@app.get("/health")
def health_check():
    """[ZH] 服务健康检查探针 / [EN] Health check probe"""
    return {
        "status": "healthy",
        "service": "epgm-cloud-gateway",
        "connected_clients": len(manager.active_connections),
        "tracked_robots": len(connected_robots),
    }


@app.post("/api/v1/robots/{robot_id}/telemetry")
async def receive_telemetry(robot_id: str, payload: TelemetryPayload):
    """
    [ZH] 接收机器人 50Hz/10Hz 高频物理遥测上报。
    [EN] Ingest robot physical telemetry stream.
    """
    connected_robots[robot_id] = {
        "last_seen": payload.timestamp_ms,
        "battery": payload.battery_percentage,
        "bumper": payload.bumper_hit,
        "current_event": payload.current_event_id,
        "last_reward": payload.last_reward,
    }

    # 向所有打开的 Web 控制台大屏广播动态 / Broadcast to UI
    await manager.broadcast({
        "type": "telemetry_update",
        "robot_id": robot_id,
        "data": connected_robots[robot_id],
    })

    return {"status": "accepted"}


@app.post("/api/v1/robots/{robot_id}/graph_delta")
async def receive_graph_delta(robot_id: str, delta: GraphDeltaPayload):
    """
    [ZH] 接收增量图碎片并触发分布式因果拓扑融合。
    [EN] Ingest incremental graph fragment and trigger topological fusion.
    """
    # 模拟增量合并至全局图
    global_graph["meta"]["node_count"] += len(delta.nodes)
    global_graph["meta"]["edge_count"] += len(delta.edges)
    global_graph["meta"]["step"] = max(global_graph["meta"]["step"], delta.step_index)

    # 广播图拓扑演化事件
    await manager.broadcast({
        "type": "graph_updated",
        "robot_id": robot_id,
        "nodes_added": len(delta.nodes),
        "edges_added": len(delta.edges),
        "total_nodes": global_graph["meta"]["node_count"],
        "total_edges": global_graph["meta"]["edge_count"],
    })

    return {
        "status": "fused",
        "received_nodes": len(delta.nodes),
        "received_edges": len(delta.edges),
    }


@app.get("/api/v1/graph/global")
def get_global_graph():
    """[ZH] 获取当前全局因果拓扑先验模型 / [EN] Get current global causal graph"""
    return global_graph


@app.websocket("/ws/live_graph")
async def websocket_live_stream(websocket: WebSocket):
    """
    [ZH] 实时 WebSocket 双向通道，用于大屏渲染动态图演化与下发控制指令。
    [EN] Real-time bidirectional WebSocket stream for dashboard rendering & control.
    """
    await manager.connect(websocket)
    try:
        # 初始推送当前全量图状态
        await websocket.send_json({
            "type": "initial_state",
            "global_graph": global_graph,
            "robots": connected_robots,
        })
        while True:
            # 持续监听客户端指令 / Keep-alive & message receiving
            data = await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
