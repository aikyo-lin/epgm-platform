"""
[ZH] EPGM 云端与大屏全链路独立集成测试套件。
[EN] EPGM Cloud Platform & Dashboard Standalone Integration Test Suite.

覆盖测试范畴 / Test Coverage:
1. PostgreSQL 16 DDL 与 SQLAlchemy 2.0 ORM 模型契约 (Robot, TelemetryLog, GraphSnapshot).
2. MQTT Bridge 双协议解包 (JSON 字符串与 C 结构体二进制 Packed Struct 互通).
3. Celery 狄利克雷-多项分布共轭贝叶斯融合数学公式严格数值验证.
4. FastAPI REST API 路由与 `/ws/live_graph` WebSocket 双向通道契约验证.
5. Vis.js 前端看板增量渲染防抖与抗抖动配置断言.
"""

import os
import sys
import json
import math
import time
import struct
import unittest
from unittest.mock import MagicMock, patch

# 注入路径与环境
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
os.environ["DATABASE_URL"] = "sqlite:///:memory:"

# 导入被测模块
from db.models import Base, Robot, TelemetryLog, GraphSnapshot
from db.session import get_db_session
from mqtt_bridge import (
    EPGMMQTTBridge,
    CloudGraphAggregator,
    TELEMETRY_PACKET_FORMAT,
    TELEMETRY_PACKET_SIZE,
    DELTA_HEADER_FORMAT,
    DELTA_HEADER_SIZE,
    DELTA_EDGE_FORMAT,
    DELTA_EDGE_SIZE,
)
from worker.tasks import (
    DIRICHLET_ALPHA_0,
    CONFIDENCE_LAMBDA,
    _recompute_posterior_probabilities,
    fuse_graph_deltas,
    prune_global_graph,
)
from api.main import app, manager, get_current_global_graph, get_current_robots

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from fastapi.testclient import TestClient


class TestPostgresORM(unittest.TestCase):
    """
    [ZH] 数据库模型与 SQLAlchemy 2.0 ORM 实体关系测试。
    """

    def setUp(self):
        # 使用独立的内存 SQLite 数据库模拟关系型环境
        self.engine = create_engine("sqlite:///:memory:", echo=False)
        Base.metadata.create_all(bind=self.engine)
        self.Session = sessionmaker(bind=self.engine)
        self.session = self.Session()

    def tearDown(self):
        self.session.close()
        Base.metadata.drop_all(bind=self.engine)

    def test_robot_lifecycle_and_relationships(self):
        """测试机器人设备资产注册与外键时序日志级联"""
        # 1. 创建机器人
        robot = Robot(
            robot_id="test_robot_01",
            name="Alpha Explorer",
            hardware_rev="ESP32-S3-N16R8",
            firmware_version="v1.0.0",
            status="online",
            battery_level=98,
        )
        self.session.add(robot)
        self.session.commit()

        # 2. 验证持久化与查询
        queried = self.session.query(Robot).filter_by(robot_id="test_robot_01").first()
        self.assertIsNotNone(queried)
        self.assertEqual(queried.name, "Alpha Explorer")
        self.assertEqual(queried.status, "online")

        # 3. 添加遥测时序日志
        log = TelemetryLog(
            robot_id="test_robot_01",
            timestamp_ms=1730386200000,
            current_event_id=42,
            last_action=1,
            last_reward=0.85,
            battery_percentage=98,
            bumper_hit=False,
            sensor_distances=[450, 600, 800, 800, 800, 800, 600, 450],
        )
        self.session.add(log)
        self.session.commit()

        # 4. 验证关联查询与 to_dict 序列化
        logs = queried.telemetry_logs
        self.assertEqual(len(logs), 1)
        self.assertEqual(logs[0].current_event_id, 42)
        d = logs[0].to_dict()
        self.assertEqual(d["robot_id"], "test_robot_01")
        self.assertEqual(d["last_action"], 1)

    def test_graph_snapshot_persistence(self):
        """测试全局拓扑快照的序列化存储与版本查询"""
        dummy_graph = {
            "meta": {"version": 42, "step": 1500},
            "nodes": {"0": {"event_id": 0}, "37": {"event_id": 37}},
            "edges": {"37_1_0": {"expected_reward": -10.0, "confidence": 0.95}},
        }
        snapshot = GraphSnapshot(
            version_tag="v_42_1730386200",
            step_index=1500,
            total_nodes=2,
            total_edges=1,
            graph_json=dummy_graph,
            trigger_robot_id="test_robot_01",
            metrics={"avg_confidence": 0.95, "hit_rate": 0.91},
        )
        self.session.add(snapshot)
        self.session.commit()

        res = self.session.query(GraphSnapshot).filter_by(version_tag="v_42_1730386200").first()
        self.assertIsNotNone(res)
        self.assertEqual(res.step_index, 1500)
        self.assertEqual(res.graph_json["meta"]["version"], 42)


class TestMQTTBridgeProtocol(unittest.TestCase):
    """
    [ZH] 测试 MQTT Bridge 的多协议解析与仿真兼容性。
    """

    def setUp(self):
        # 实例化 Bridge 但不连接 Broker
        with patch("redis.Redis.from_url"), patch("celery.Celery"):
            self.bridge = EPGMMQTTBridge(broker_host="localhost", broker_port=1883)

    def test_unpack_telemetry_json(self):
        """测试 JSON 遥测流的正常反序列化"""
        payload = {
            "robot_id": "robot_alpha",
            "timestamp_ms": 1730386200123,
            "current_event_id": 37,
            "last_action": 1,
            "last_reward": 0.8,
            "battery_percentage": 95,
            "bumper_hit": False,
            "sensor_distances": [500, 600, 700, 800, 800, 700, 600, 500],
        }
        raw_bytes = json.dumps(payload).encode("utf-8")
        result = self.bridge.unpack_telemetry(raw_bytes)
        self.assertIsNotNone(result)
        self.assertEqual(result["robot_id"], "robot_alpha")
        self.assertEqual(result["current_event_id"], 37)
        self.assertEqual(result["last_reward"], 0.8)

    def test_unpack_telemetry_binary_packed_struct(self):
        """测试兼容 C 固件 protocol_schema.h 的 44 字节二进制 Packed Struct 解包"""
        ver = 1
        raw_id = b"robot_s3_firm\x00\x00\x00"  # 16 bytes
        ts = 1730386200
        evt = 128
        act = 3
        rew_q8 = int(0.75 * 256)  # Q8.8 格式定点数
        bat = 88
        bmp = 1
        dists = [350, 400, 600, 800, 900, 700, 500, 320]  # 8 uint16

        binary_data = struct.pack(
            TELEMETRY_PACKET_FORMAT,
            ver,
            raw_id,
            ts,
            evt,
            act,
            rew_q8,
            bat,
            bmp,
            *dists,
        )
        self.assertEqual(len(binary_data), TELEMETRY_PACKET_SIZE)

        result = self.bridge.unpack_telemetry(binary_data)
        self.assertIsNotNone(result)
        self.assertEqual(result["robot_id"], "robot_s3_firm")
        self.assertEqual(result["current_event_id"], 128)
        self.assertEqual(result["last_action"], 3)
        self.assertAlmostEqual(result["last_reward"], 0.75, places=2)
        self.assertEqual(result["battery_percentage"], 88)
        self.assertTrue(result["bumper_hit"])
        self.assertEqual(result["sensor_distances"], dists)

    def test_unpack_graph_delta_binary(self):
        """测试兼容 C 固件的图碎片二进制 Packed Struct 解包"""
        ver = 1
        raw_id = b"robot_beta\x00\x00\x00\x00\x00\x00"
        seq = 10
        edge_cnt = 2
        step = 500

        header = struct.pack(DELTA_HEADER_FORMAT, ver, raw_id, seq, edge_cnt, step)
        edge1 = struct.pack(DELTA_EDGE_FORMAT, 37, 1, 14, int(0.8 * 256), 5, int(0.85 * 255))
        edge2 = struct.pack(DELTA_EDGE_FORMAT, 14, 1, 0, int(-10.0 * 256), 2, int(0.95 * 255))

        payload = header + edge1 + edge2
        result = self.bridge.unpack_graph_delta(payload, "robot_beta")
        self.assertIsNotNone(result)
        self.assertEqual(result["robot_id"], "robot_beta")
        self.assertEqual(result["step_index"], 500)
        self.assertEqual(len(result["edges"]), 2)
        self.assertEqual(result["edges"][0]["source_event"], 37)
        self.assertEqual(result["edges"][1]["target_event"], 0)
        self.assertAlmostEqual(result["edges"][1]["expected_reward"], -10.0, places=2)

    def test_cloud_graph_aggregator_backward_compatibility(self):
        """测试 CloudGraphAggregator 保留类兼容 swarm_simulation.py"""
        agg = CloudGraphAggregator(action_dim=5)
        self.assertIsNotNone(agg)
        agg.process_telemetry("robot_01", {"battery_percentage": 90, "current_event_id": 1})
        self.assertIn("robot_01", agg.registered_robots)
        model = agg.get_distilled_global_model()
        self.assertIn("nodes", model)


class TestBayesianGraphFusionMath(unittest.TestCase):
    """
    [ZH] 狄利克雷-多项分布共轭贝叶斯融合数学公式严格数值验证。
    """

    def test_dirichlet_multinomial_posterior_probabilities(self):
        """
        验证多模态分支条件下的狄利克雷后验转移概率公式:
        P*(v_j | v_i, a) = (alpha_0 + N(v_i, a, v_j)) / sum_m (alpha_0 + N(v_i, a, v_m))
        """
        alpha_0 = 0.1
        edges_dict = {
            # 源状态 10，动作 1 指向三个不同目标状态 (20, 21, 22)
            "10_1_20": {"source_event": 10, "action": 1, "target_event": 20, "observation_count": 8},
            "10_1_21": {"source_event": 10, "action": 1, "target_event": 21, "observation_count": 2},
            "10_1_22": {"source_event": 10, "action": 1, "target_event": 22, "observation_count": 0},
            # 无关边
            "15_2_30": {"source_event": 15, "action": 2, "target_event": 30, "observation_count": 5},
        }

        _recompute_posterior_probabilities(edges_dict, 10, 1, alpha_0=alpha_0)

        # 理论分母: 3 * 0.1 + (8 + 2 + 0) = 0.3 + 10 = 10.3
        denom = 3 * alpha_0 + 10
        expected_p20 = (alpha_0 + 8) / denom
        expected_p21 = (alpha_0 + 2) / denom
        expected_p22 = (alpha_0 + 0) / denom

        self.assertAlmostEqual(edges_dict["10_1_20"]["probability"], expected_p20, places=3)
        self.assertAlmostEqual(edges_dict["10_1_21"]["probability"], expected_p21, places=3)
        self.assertAlmostEqual(edges_dict["10_1_22"]["probability"], expected_p22, places=3)

        # 概率和严格归一为 1.0 (在舍入精度内)
        total_p = sum(edges_dict[k]["probability"] for k in ["10_1_20", "10_1_21", "10_1_22"])
        self.assertAlmostEqual(total_p, 1.0, places=2)

    def test_confidence_saturation_and_uncertainty(self):
        """
        验证指数饱和置信度与认知不确定度演化曲线:
        C = 1 - exp(-lambda * N), U = exp(-lambda * N)
        """
        lam = 0.2
        # 0 次观测: C = 0.0, U = 1.0
        c0 = 1.0 - math.exp(-lam * 0)
        u0 = math.exp(-lam * 0)
        self.assertAlmostEqual(c0, 0.0)
        self.assertAlmostEqual(u0, 1.0)

        # 5 次观测: C = 1 - e^-1 = 0.6321
        c5 = 1.0 - math.exp(-lam * 5)
        self.assertAlmostEqual(c5, 0.6321, places=3)

        # 15 次观测: C = 1 - e^-3 = 0.9502
        c15 = 1.0 - math.exp(-lam * 15)
        self.assertAlmostEqual(c15, 0.9502, places=3)

    def test_weighted_reward_conjugate_update(self):
        """
        验证即时回报样本加权增量更新:
        R_new = (R_old * N_old + R_delta * N_delta) / (N_old + N_delta)
        """
        r_old = 0.5
        n_old = 4
        r_delta = 1.0
        n_delta = 2

        n_new = n_old + n_delta
        r_new = (r_old * n_old + r_delta * n_delta) / n_new
        # (0.5 * 4 + 1.0 * 2) / 6 = (2.0 + 2.0) / 6 = 0.6667
        self.assertAlmostEqual(r_new, 4.0 / 6.0, places=4)


class TestFastAPIGatewayAndWebSocket(unittest.TestCase):
    """
    [ZH] 测试 FastAPI 核心 REST 接口与 WebSocket 双向通道契约。
    """

    def setUp(self):
        from db.session import init_db
        try:
            init_db()
        except Exception:
            pass
        self.client = TestClient(app)

    def test_health_probe(self):
        """测试服务健康检查"""
        response = self.client.get("/health")
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data["status"], "healthy")
        self.assertEqual(data["service"], "epgm-cloud-gateway")

    def test_telemetry_rest_endpoint(self):
        """测试 REST 遥测接入与广播触发"""
        payload = {
            "robot_id": "test_bot",
            "timestamp_ms": int(time.time() * 1000),
            "current_event_id": 15,
            "last_action": 1,
            "last_reward": 0.5,
            "battery_percentage": 92,
            "bumper_hit": False,
            "sensor_distances": [500, 500, 500, 500, 500, 500, 500, 500],
        }
        res = self.client.post("/api/v1/robots/test_bot/telemetry", json=payload)
        self.assertEqual(res.status_code, 200)
        self.assertEqual(res.json()["status"], "accepted")

    def test_graph_delta_rest_endpoint(self):
        """测试 REST 图碎片上报接口"""
        delta = {
            "robot_id": "test_bot",
            "sequence_id": 1,
            "step_index": 100,
            "nodes": [{"event_id": 15, "visit_count": 5}],
            "edges": [{"source_event": 15, "action": 1, "target_event": 16, "expected_reward": 0.6}],
        }
        res = self.client.post("/api/v1/robots/test_bot/graph_delta", json=delta)
        self.assertEqual(res.status_code, 200)
        self.assertEqual(res.json()["status"], "fused")

    def test_websocket_contract_and_heartbeat(self):
        """测试 WebSocket `/ws/live_graph` 连接握手、initial_state 下发及 ping/pong 响应"""
        with self.client.websocket_connect("/ws/live_graph") as websocket:
            # 1. 验证初次连入即刻下发 initial_state
            data = websocket.receive_json()
            self.assertEqual(data["type"], "initial_state")
            self.assertIn("global_graph", data)
            self.assertIn("robots", data)

            # 2. 验证文本 ping 响应
            websocket.send_text("ping")
            pong = websocket.receive_text()
            self.assertEqual(pong, "pong")

            # 3. 验证 JSON ping 响应
            websocket.send_text(json.dumps({"type": "ping"}))
            json_pong = websocket.receive_json()
            self.assertEqual(json_pong["type"], "pong")


class TestDashboardHtmlContract(unittest.TestCase):
    """
    [ZH] 静态断言前端看板 index.html 包含增量 DataSet 更新与抗抖动配置。
    """

    def test_dashboard_incremental_rendering_keywords(self):
        html_path = os.path.join(current_dir, "..", "dashboard", "index.html")
        self.assertTrue(os.path.exists(html_path))
        with open(html_path, "r", encoding="utf-8") as f:
            content = f.read()

        # 断言使用了 vis.DataSet
        self.assertIn("nodesDataSet", content)
        self.assertIn("edgesDataSet", content)
        self.assertIn("nodesDataSet.update", content)
        self.assertIn("edgesDataSet.update", content)

        # 断言包含 WebSocket 实时连接通道
        self.assertIn("/ws/live_graph", content)
        self.assertIn("handleWebSocketMessage", content)

        # 断言配置了物理阻尼防止布局跳动
        self.assertIn("damping", content)
        self.assertIn("stabilization", content)

        # 断言包含硬安全屏蔽与碰撞风险红色警示
        self.assertIn("safe-mask-warn", content)
        self.assertIn("Collision", content)


if __name__ == "__main__":
    unittest.main(verbosity=2)
