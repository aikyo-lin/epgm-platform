"""
[ZH] EPGM 云端持久化 ORM 模型定义 (SQLAlchemy 2.0 规范)。
[EN] EPGM Cloud Persistence ORM Model Definitions (SQLAlchemy 2.0 Specification).

包含核心实体 / Core Entities:
1. Robot: 机器人设备资产模型与实时连接状态
2. TelemetryLog: 高频遥测时序日志模型
3. GraphSnapshot: 全局因果拓扑图版本快照归档模型
"""

from datetime import datetime, timezone
from typing import Dict, Any, List, Optional
from sqlalchemy import (
    String,
    Integer,
    BigInteger,
    SmallInteger,
    Numeric,
    Boolean,
    DateTime,
    JSON,
    ForeignKey,
    Index,
    desc,
)
from sqlalchemy.orm import (
    DeclarativeBase,
    Mapped,
    mapped_column,
    relationship,
)


class Base(DeclarativeBase):
    """
    [ZH] 声明式基础基类 (SQLAlchemy 2.0 DeclarativeBase)
    [EN] Declarative base class
    """
    pass


class Robot(Base):
    """
    [ZH] 机器人设备实体模型，映射 `robots` 表。
    [EN] Robot device entity model, mapping to `robots` table.
    """
    __tablename__ = "robots"

    robot_id: Mapped[str] = mapped_column(
        String(64),
        primary_key=True,
        comment="[ZH] 机器人设备唯一标识 / [EN] Robot Unique ID"
    )
    name: Mapped[str] = mapped_column(
        String(128),
        nullable=False,
        comment="[ZH] 机器人显示名称 / [EN] Robot display name"
    )
    hardware_rev: Mapped[str] = mapped_column(
        String(32),
        default="ESP32-S3-N16R8",
        comment="[ZH] 硬件平台版本型号 / [EN] Hardware revision"
    )
    firmware_version: Mapped[str] = mapped_column(
        String(32),
        default="v1.0.0",
        comment="[ZH] 固件程序版本号 / [EN] Firmware version"
    )
    status: Mapped[str] = mapped_column(
        String(16),
        default="offline",
        index=True,
        comment="[ZH] 当前在线状态 ('online', 'offline', 'error') / [EN] Online status"
    )
    battery_level: Mapped[int] = mapped_column(
        Integer,
        default=100,
        comment="[ZH] 剩余电池电量百分比 (0~100) / [EN] Battery percentage"
    )
    last_seen_at: Mapped[Optional[datetime]] = mapped_column(
        DateTime(timezone=True),
        nullable=True,
        index=True,
        comment="[ZH] 最近一次心跳接收时间戳 / [EN] Last heartbeat timestamp"
    )
    registered_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        default=lambda: datetime.now(timezone.utc),
        comment="[ZH] 首次入网注册时间 / [EN] Registration timestamp"
    )
    config_json: Mapped[Dict[str, Any]] = mapped_column(
        JSON,
        default=dict,
        comment="[ZH] 机器人定制化参数与标定配置 / [EN] Robot custom configuration"
    )

    # 一对多关联：该小车上报的所有时序遥测日志
    telemetry_logs: Mapped[List["TelemetryLog"]] = relationship(
        "TelemetryLog",
        back_populates="robot",
        cascade="all, delete-orphan",
        passive_deletes=True,
    )

    def to_dict(self) -> Dict[str, Any]:
        """[ZH] 序列化为 Python 字典 / [EN] Serialize to dictionary"""
        return {
            "robot_id": self.robot_id,
            "name": self.name,
            "hardware_rev": self.hardware_rev,
            "firmware_version": self.firmware_version,
            "status": self.status,
            "battery_level": self.battery_level,
            "last_seen_at": self.last_seen_at.isoformat() if self.last_seen_at else None,
            "registered_at": self.registered_at.isoformat() if self.registered_at else None,
            "config_json": self.config_json,
        }

    def __repr__(self) -> str:
        return f"<Robot(robot_id='{self.robot_id}', name='{self.name}', status='{self.status}')>"


class TelemetryLog(Base):
    """
    [ZH] 遥测时序日志实体模型，映射 `telemetry_logs` 表。
    [EN] Telemetry time-series log entity model, mapping to `telemetry_logs` table.
    """
    __tablename__ = "telemetry_logs"

    id: Mapped[int] = mapped_column(
        BigInteger().with_variant(Integer, "sqlite"),
        primary_key=True,
        autoincrement=True,
        comment="[ZH] 日志主键自增 ID / [EN] Log Primary Key"
    )
    robot_id: Mapped[str] = mapped_column(
        String(64),
        ForeignKey("robots.robot_id", ondelete="CASCADE"),
        nullable=False,
        index=True,
        comment="[ZH] 外键关联机器人设备 ID / [EN] Foreign key referencing robot_id"
    )
    timestamp_ms: Mapped[int] = mapped_column(
        BigInteger,
        nullable=False,
        comment="[ZH] 边缘端毫秒时间戳 / [EN] Edge millisecond timestamp"
    )
    current_event_id: Mapped[int] = mapped_column(
        Integer,
        nullable=False,
        comment="[ZH] 当前量化空间离散事件 ID / [EN] Quantized event ID"
    )
    last_action: Mapped[int] = mapped_column(
        SmallInteger,
        nullable=False,
        comment="[ZH] 上一步执行动作 / [EN] Last executed action"
    )
    last_reward: Mapped[float] = mapped_column(
        Numeric(8, 4),
        nullable=False,
        comment="[ZH] 物理即时回报值 / [EN] Physical immediate reward"
    )
    battery_percentage: Mapped[int] = mapped_column(
        SmallInteger,
        nullable=False,
        default=100,
        comment="[ZH] 电池电量 (0~100) / [EN] Battery percentage"
    )
    bumper_hit: Mapped[bool] = mapped_column(
        Boolean,
        default=False,
        nullable=False,
        comment="[ZH] 碰撞触须开关是否激活 / [EN] Collision bumper hit trigger"
    )
    sensor_distances: Mapped[List[Any]] = mapped_column(
        JSON,
        default=list,
        comment="[ZH] 8路测距传感器读数数组 (mm) / [EN] 8-channel distance readings"
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        default=lambda: datetime.now(timezone.utc),
        index=True,
        comment="[ZH] 云端持久化时间戳 / [EN] Cloud ingestion timestamp"
    )

    # 反向多对一关联
    robot: Mapped["Robot"] = relationship("Robot", back_populates="telemetry_logs")

    __table_args__ = (
        Index("idx_telemetry_robot_time", "robot_id", desc("timestamp_ms")),
    )

    def to_dict(self) -> Dict[str, Any]:
        """[ZH] 序列化为字典 / [EN] Serialize to dictionary"""
        return {
            "id": self.id,
            "robot_id": self.robot_id,
            "timestamp_ms": self.timestamp_ms,
            "current_event_id": self.current_event_id,
            "last_action": self.last_action,
            "last_reward": float(self.last_reward),
            "battery_percentage": self.battery_percentage,
            "bumper_hit": self.bumper_hit,
            "sensor_distances": self.sensor_distances,
            "created_at": self.created_at.isoformat() if self.created_at else None,
        }

    def __repr__(self) -> str:
        return (
            f"<TelemetryLog(id={self.id}, robot_id='{self.robot_id}', "
            f"event={self.current_event_id}, action={self.last_action}, reward={self.last_reward})>"
        )


class GraphSnapshot(Base):
    """
    [ZH] 全局因果拓扑图版本快照模型，映射 `graph_snapshots` 表。
    [EN] Global causal topological graph snapshot model, mapping to `graph_snapshots` table.
    """
    __tablename__ = "graph_snapshots"

    id: Mapped[int] = mapped_column(
        BigInteger().with_variant(Integer, "sqlite"),
        primary_key=True,
        autoincrement=True,
        comment="[ZH] 快照自增主键 / [EN] Snapshot Primary Key"
    )
    version_tag: Mapped[str] = mapped_column(
        String(64),
        unique=True,
        nullable=False,
        comment="[ZH] 快照全局唯一版本标签 / [EN] Globally unique version tag"
    )
    step_index: Mapped[int] = mapped_column(
        BigInteger,
        default=0,
        nullable=False,
        index=True,
        comment="[ZH] 快照时对应的系统累计步数 / [EN] Global step index"
    )
    total_nodes: Mapped[int] = mapped_column(
        Integer,
        nullable=False,
        comment="[ZH] 全局图节点总数 / [EN] Total node count"
    )
    total_edges: Mapped[int] = mapped_column(
        Integer,
        nullable=False,
        comment="[ZH] 全局图因果预测边总数 / [EN] Total causal edge count"
    )
    graph_json: Mapped[Dict[str, Any]] = mapped_column(
        JSON,
        nullable=False,
        comment="[ZH] 全量拓扑图数据结构 (包含节点和转移边矩阵) / [EN] Full graph topology JSON"
    )
    trigger_robot_id: Mapped[Optional[str]] = mapped_column(
        String(64),
        nullable=True,
        comment="[ZH] 触发快照上报或融合的机器人 ID / [EN] Triggering robot ID"
    )
    metrics: Mapped[Dict[str, Any]] = mapped_column(
        JSON,
        default=dict,
        comment="[ZH] 拓扑图收敛评估统计指标 (命中率、平均置信度等) / [EN] Graph evaluation metrics"
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        default=lambda: datetime.now(timezone.utc),
        index=True,
        comment="[ZH] 快照生成归档时间 / [EN] Snapshot generation timestamp"
    )

    def to_dict(self) -> Dict[str, Any]:
        """[ZH] 序列化为字典 / [EN] Serialize to dictionary"""
        return {
            "id": self.id,
            "version_tag": self.version_tag,
            "step_index": self.step_index,
            "total_nodes": self.total_nodes,
            "total_edges": self.total_edges,
            "graph_json": self.graph_json,
            "trigger_robot_id": self.trigger_robot_id,
            "metrics": self.metrics,
            "created_at": self.created_at.isoformat() if self.created_at else None,
        }

    def __repr__(self) -> str:
        return (
            f"<GraphSnapshot(id={self.id}, version='{self.version_tag}', "
            f"nodes={self.total_nodes}, edges={self.total_edges}, step={self.step_index})>"
        )
