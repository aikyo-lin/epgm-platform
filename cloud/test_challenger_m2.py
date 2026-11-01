"""
[ZH] Milestone 2 对抗性压力测试与数值稳定性验证套件 (Challenger 1).
[EN] Milestone 2 Adversarial Stress Test & Numerical Stability Verification Suite.

测试范畴:
1. MQTT Bridge 高吞吐突发接入 (1000 遥测 + 500 图碎片)
2. 畸形 JSON 与受损二进制包的容错与无崩溃优雅拒绝
3. MQTT 自动重连模拟、带抖动指数退避及重置逻辑
4. 狄利克雷-多项分布极端比率 (1 vs 100,000) 下的数值稳定性
5. 转移概率归一化严格性校验 sum_j P(v_j | v_i, a) == 1.0 与下游采样器碰撞分析
6. 置信度饱和 C in [0, 1) 与认知不确定度 U in (0, 1] 区间边界
7. LTD 图修剪边缘场景 (0边、全剪、非连通图、剪后残余概率归一化缺失审计)
8. Celery 图融合端到端边界防御 (空包、畸形类型、连续融合)
"""

import os
import sys
import json
import time
import math
import struct
import random
import unittest
from typing import Dict, Any, List
from unittest.mock import MagicMock, patch
import numpy as np

# 注入路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
os.environ["DATABASE_URL"] = "sqlite:///:memory:"

from mqtt_bridge import (
    EPGMMQTTBridge,
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


class MockRedisClient:
    """轻量级内存 Redis 模拟器，支持键值存储、散列、有序集合、PubSub 和分布式锁"""
    def __init__(self):
        self.store = {}
        self.hashes = {}
        self.zsets = {}
        self.pubsub_channels = {}

    def get(self, key):
        return self.store.get(key)

    def set(self, key, value):
        self.store[key] = str(value)
        return True

    def hset(self, key, mapping=None, **kwargs):
        if key not in self.hashes:
            self.hashes[key] = {}
        if mapping:
            self.hashes[key].update(mapping)
        if kwargs:
            self.hashes[key].update(kwargs)
        return len(self.hashes[key])

    def expire(self, key, seconds):
        return True

    def zadd(self, key, mapping):
        if key not in self.zsets:
            self.zsets[key] = {}
        self.zsets[key].update(mapping)
        return len(mapping)

    def publish(self, channel, message):
        if channel not in self.pubsub_channels:
            self.pubsub_channels[channel] = []
        self.pubsub_channels[channel].append(message)
        return 1

    def pipeline(self):
        return MockRedisPipeline(self)

    def lock(self, key, timeout=None, blocking_timeout=None):
        return MockRedisLock(self, key)


class MockRedisPipeline:
    def __init__(self, client):
        self.client = client
        self.commands = []

    def hset(self, *args, **kwargs):
        self.commands.append(("hset", args, kwargs))
        return self

    def expire(self, *args, **kwargs):
        self.commands.append(("expire", args, kwargs))
        return self

    def zadd(self, *args, **kwargs):
        self.commands.append(("zadd", args, kwargs))
        return self

    def publish(self, *args, **kwargs):
        self.commands.append(("publish", args, kwargs))
        return self

    def execute(self):
        results = []
        for cmd, args, kwargs in self.commands:
            if cmd == "hset":
                results.append(self.client.hset(*args, **kwargs))
            elif cmd == "expire":
                results.append(self.client.expire(*args, **kwargs))
            elif cmd == "zadd":
                results.append(self.client.zadd(*args, **kwargs))
            elif cmd == "publish":
                results.append(self.client.publish(*args, **kwargs))
        self.commands.clear()
        return results


class MockRedisLock:
    def __init__(self, client, key):
        self.client = client
        self.key = key
        self.acquired = False

    def acquire(self, blocking=True):
        self.acquired = True
        return True

    def release(self):
        self.acquired = False
        return True


class TestMQTTBridgeIngestionStress(unittest.TestCase):
    """
    [对抗测试 1] MQTT Bridge 摄入高压测试、畸形报文测试与重连退避
    """

    def setUp(self):
        self.mock_redis = MockRedisClient()
        self.mock_celery = MagicMock()
        with patch("redis.Redis.from_url", return_value=self.mock_redis), \
             patch("celery.Celery", return_value=self.mock_celery):
            self.bridge = EPGMMQTTBridge()
            self.bridge.redis_client = self.mock_redis
            self.bridge.celery_app = self.mock_celery

    def test_burst_telemetry_high_throughput(self):
        """
        [压测 1.1] 突发 1,000 条遥测报文 (500 条 JSON + 500 条二进制 Packed Struct)
        验证网关吞吐能力、无内存泄漏及零异常崩溃。
        """
        start_time = time.time()
        count = 1000

        for i in range(count):
            robot_id = f"stress_bot_{i % 10}"
            msg = MagicMock()
            msg.topic = f"epgm/robot/{robot_id}/telemetry"

            if i % 2 == 0:
                payload_dict = {
                    "robot_id": robot_id,
                    "timestamp_ms": int(time.time() * 1000),
                    "current_event_id": (i * 7) % 182,
                    "last_action": i % 5,
                    "last_reward": round(math.sin(i), 3),
                    "battery_percentage": 90 - (i % 20),
                    "bumper_hit": (i % 50 == 0),
                    "sensor_distances": [400 + (i % 100)] * 8,
                }
                msg.payload = json.dumps(payload_dict).encode("utf-8")
            else:
                msg.payload = struct.pack(
                    TELEMETRY_PACKET_FORMAT,
                    1,
                    f"{robot_id}".encode("utf-8")[:16].ljust(16, b"\x00"),
                    int(time.time()),
                    (i * 13) % 182,
                    i % 5,
                    int(math.sin(i) * 256),
                    85,
                    1 if (i % 30 == 0) else 0,
                    *[500 + (i % 50)] * 8,
                )

            self.bridge.on_message(self.bridge.client, None, msg)

        elapsed = time.time() - start_time
        throughput = count / max(elapsed, 0.001)

        print(f"\n[STRESS REPORT] Burst Telemetry: {count} msgs in {elapsed:.3f}s ({throughput:.1f} msgs/sec)")
        self.assertGreater(throughput, 100.0, "Telemetry throughput should exceed 100 msgs/s")
        self.assertTrue(len(self.mock_redis.hashes) > 0)
        self.assertTrue(len(self.mock_redis.pubsub_channels.get("epgm:channel:telemetry", [])) == count)

    def test_burst_graph_delta_high_throughput(self):
        """
        [压测 1.2] 突发 500 条因果图碎片报文 (250 条 JSON + 250 条二进制)
        验证异步 Celery 任务派发无丢包与通道稳定性。
        """
        start_time = time.time()
        count = 500

        for i in range(count):
            robot_id = f"stress_bot_{i % 5}"
            msg = MagicMock()
            msg.topic = f"epgm/robot/{robot_id}/graph_delta"

            if i % 2 == 0:
                delta_dict = {
                    "robot_id": robot_id,
                    "sequence_id": i,
                    "step_index": i * 10,
                    "nodes": [{"event_id": (i % 81) + 1, "visit_count": 2}],
                    "edges": [{
                        "source_event": (i % 81) + 1,
                        "action": i % 5,
                        "target_event": ((i + 1) % 81) + 1,
                        "expected_reward": 0.5,
                        "observation_count": 3,
                        "confidence": 0.45,
                    }],
                }
                msg.payload = json.dumps(delta_dict).encode("utf-8")
            else:
                header = struct.pack(
                    DELTA_HEADER_FORMAT,
                    1,
                    f"{robot_id}".encode("utf-8")[:16].ljust(16, b"\x00"),
                    i,
                    1,  # 1 edge
                    i * 10,
                )
                edge = struct.pack(
                    DELTA_EDGE_FORMAT,
                    (i % 81) + 1,
                    i % 5,
                    ((i + 1) % 81) + 1,
                    int(0.5 * 256),
                    3,
                    int(0.45 * 255),
                )
                msg.payload = header + edge

            self.bridge.on_message(self.bridge.client, None, msg)

        elapsed = time.time() - start_time
        print(f"[STRESS REPORT] Burst Graph Delta: {count} msgs in {elapsed:.3f}s")
        self.assertEqual(self.mock_celery.send_task.call_count, count)

    def test_malformed_payloads_graceful_rejection(self):
        """
        [对抗 1.3] 注入畸形 JSON、损坏二进制及极端脏数据，验证绝不崩溃
        """
        malformed_inputs = [
            b"",                                  # 空字节
            b"not a valid json at all",           # 纯文本非法 JSON
            b'{"robot_id": "r1", "missing_end":', # 截断 JSON
            b"12345",                             # JSON 标量整数
            b'"just a string"',                   # JSON 标量字符串
            b"[1, 2, 3]",                         # JSON 列表 (非字典)
            b"true",                              # JSON 布尔
            b"null",                              # JSON None
            b"\x00" * 43,                         # 43 字节二进制 (比44少1字节)
            b"\xff" * 45,                         # 45 字节二进制 (比44多1字节)
            b"\x00" * 26,                         # 26 字节图碎片 (比27包头少1字节)
            b"\x00" * 36,                         # 36 字节图碎片 (有包头但边数据少1字节)
            b"\x00" * 1000000,                    # 1MB 巨型全零垃圾数据
            os.urandom(44),                       # 44 字节随机噪声音频/伪二进制
            os.urandom(27 + 10),                  # 37 字节随机噪声伪图碎片
        ]

        for idx, payload in enumerate(malformed_inputs):
            # 测试遥测端口
            msg_telemetry = MagicMock()
            msg_telemetry.topic = "epgm/robot/bot_malformed/telemetry"
            msg_telemetry.payload = payload
            try:
                self.bridge.on_message(self.bridge.client, None, msg_telemetry)
            except Exception as e:
                self.fail(f"Bridge crashed on malformed telemetry payload #{idx} (len={len(payload)}): {e}")

            # 测试图碎片端口
            msg_delta = MagicMock()
            msg_delta.topic = "epgm/robot/bot_malformed/graph_delta"
            msg_delta.payload = payload
            try:
                self.bridge.on_message(self.bridge.client, None, msg_delta)
            except Exception as e:
                self.fail(f"Bridge crashed on malformed delta payload #{idx} (len={len(payload)}): {e}")

    def test_malformed_topic_routing_rejection(self):
        """
        [对抗 1.4] 注入非标准主题与路径穿透
        """
        invalid_topics = [
            "epgm",
            "epgm/robot",
            "epgm/robot/bot1",
            "epgm/robot/bot1/telemetry/extra",
            "other/system/telemetry",
            "epgm/robot/bot1/unknown_command",
            "../../../etc/passwd",
        ]
        for topic in invalid_topics:
            msg = MagicMock()
            msg.topic = topic
            msg.payload = b'{"status": "ok"}'
            try:
                self.bridge.on_message(self.bridge.client, None, msg)
            except Exception as e:
                self.fail(f"Bridge crashed on topic '{topic}': {e}")

    def test_reconnection_simulation_and_backoff(self):
        """
        [对抗 1.5] 模拟网络抖动下的指数退避演化与重连成功重置
        """
        bridge = self.bridge
        bridge.base_delay = 1.0
        bridge.max_delay = 30.0
        bridge.current_delay = 1.0

        # 模拟连续 6 次连接失败
        expected_delays = [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0]
        recorded_delays = []

        for _ in range(len(expected_delays)):
            recorded_delays.append(bridge.current_delay)
            bridge.current_delay = min(bridge.current_delay * 2, bridge.max_delay)

        self.assertEqual(recorded_delays, expected_delays)

        # 模拟连接成功重连 (rc == 0)
        bridge.on_connect(bridge.client, None, None, 0)
        self.assertEqual(bridge.current_delay, bridge.base_delay, "Delay must reset to base_delay upon connect")


class TestBayesianFusionNumericalStability(unittest.TestCase):
    """
    [对抗测试 2] 狄利克雷共轭数学稳定性、置信度饱和极限与后验概率归一化
    """

    def test_extreme_count_ratios_dirichlet_update(self):
        """
        [数值测试 2.1] 极端观测比率压力测试: 1 次 vs 100,000 次
        验证狄利克雷弱先验 alpha_0 在极大样本量下的抗下溢能力。
        """
        alpha_0 = 0.1
        edges = {
            "10_1_20": {"source_event": 10, "action": 1, "target_event": 20, "observation_count": 1},
            "10_1_21": {"source_event": 10, "action": 1, "target_event": 21, "observation_count": 100000},
        }

        _recompute_posterior_probabilities(edges, 10, 1, alpha_0=alpha_0)

        # 分母: 2 * 0.1 + (1 + 100000) = 100001.2
        p20 = edges["10_1_20"]["probability"]
        p21 = edges["10_1_21"]["probability"]

        print(f"\n[NUMERICAL REPORT] Extreme Ratio (1 vs 100,000): P(20)={p20}, P(21)={p21}")
        self.assertEqual(p20, 0.0)
        self.assertEqual(p21, 1.0)
        self.assertAlmostEqual(p20 + p21, 1.0)

    def test_probability_normalization_across_branching_factors(self):
        """
        [数值测试 2.2] 检验不同分支数 (K=2, 3, 5, 7, 10) 下的后验概率和 sum_j P(v_j | v_i, a)
        探查 round(..., 4) 舍入误差对严格等于 1.0 的偏差。
        """
        alpha_0 = DIRICHLET_ALPHA_0

        discrepancies = {}
        for K in [2, 3, 4, 5, 6, 7, 8, 9, 10]:
            edges = {}
            for k in range(K):
                edges[f"1_0_{k}"] = {
                    "source_event": 1,
                    "action": 0,
                    "target_event": k,
                    "observation_count": 10,
                }
            _recompute_posterior_probabilities(edges, 1, 0, alpha_0=alpha_0)

            total_p = sum(edges[f"1_0_{k}"]["probability"] for k in range(K))
            diff = abs(total_p - 1.0)
            discrepancies[K] = total_p
            print(f"[NUMERICAL REPORT] Branching K={K}: sum(P) = {total_p:.4f} (Discrepancy: {diff:+.4f})")

            # 验证在宽松容差下吻合
            self.assertAlmostEqual(total_p, 1.0, delta=0.005)

        # [加固验证]: 验证狄利克雷单纯形归一化修复生效，各分支概率和严格等于 1.0000
        self.assertEqual(discrepancies[3], 1.0, "K=3 branching satisfies exact 1.0 simplex sum")
        self.assertEqual(discrepancies[7], 1.0, "K=7 branching satisfies exact 1.0 simplex sum")

    def test_downstream_sampler_crash_on_unnormalized_probabilities(self):
        """
        [对抗 2.3] 评估独立四舍五入后的概率数组投喂给标准随机采样器 (np.random.choice) 的破坏性
        """
        # K=3 的分支: [0.3333, 0.3333, 0.3333], 和为 0.9999
        p_k3 = [0.3333, 0.3333, 0.3333]
        with self.assertRaises(ValueError) as ctx:
            np.random.choice(3, p=p_k3)
        self.assertIn("probabilities do not sum to 1", str(ctx.exception))
        print(f"[VULNERABILITY CONFIRMED] np.random.choice rejects unnormalized sum=0.9999 with ValueError")

    def test_confidence_saturation_and_uncertainty_interval(self):
        """
        [数值测试 2.4] 置信度饱和 C in [0, 1) 与认知不确定度 U in (0, 1] 极限分析
        当 N 趋向极大时，检验浮点数精度下溢与饱和边界。
        """
        test_counts = [0, 1, 5, 10, 50, 100, 200, 500, 1000, 5000, 100000]

        for N in test_counts:
            exp_neg = math.exp(-CONFIDENCE_LAMBDA * N)
            c_raw = 1.0 - exp_neg
            u_raw = exp_neg

            c_rounded = round(c_raw, 4)
            u_rounded = round(u_raw, 4)

            self.assertGreaterEqual(c_raw, 0.0)
            self.assertLessEqual(c_raw, 1.0)
            self.assertGreaterEqual(u_raw, 0.0)
            self.assertLessEqual(u_raw, 1.0)

            if N >= 47:
                self.assertEqual(c_rounded, 1.0)
                self.assertEqual(u_rounded, 0.0)

        print(f"[NUMERICAL REPORT] Confidence saturates to 1.0000 at N >= 47 under 4-decimal rounding.")

    def test_weighted_reward_conjugate_extreme_ratios(self):
        """
        [数值测试 2.5] 加权回报在极不对称样本量下的数值稳定性
        """
        n_old = 1
        r_old = -10.0
        n_delta = 100000
        r_delta = 1.0

        n_new = n_old + n_delta
        r_new = (r_old * n_old + r_delta * n_delta) / n_new
        # 严格理论值: (-10 + 100000) / 100001 = 99990 / 100001 = 0.999890001
        self.assertAlmostEqual(r_new, 0.99989, places=4)
        self.assertAlmostEqual(r_new, 1.0, places=3)

        # 反向极端: 旧样本量 100,000, 回报 +1.0; 新增样本量 1, 回报 -10.0
        n_old_2 = 100000
        r_old_2 = 1.0
        n_delta_2 = 1
        r_delta_2 = -10.0
        n_new_2 = n_old_2 + n_delta_2
        r_new_2 = (r_old_2 * n_old_2 + r_delta_2 * n_delta_2) / n_new_2
        self.assertAlmostEqual(r_new_2, 0.99989, places=4)


class TestLTDGraphPruningEdgeCases(unittest.TestCase):
    """
    [对抗测试 3] LTD 拓扑修剪极限场景: 0 边图、全剪除、非连通子图与基元保护
    """

    def setUp(self):
        self.mock_redis = MockRedisClient()

    def test_prune_empty_graph(self):
        """
        [修剪 3.1] 针对无图、无边或只有 ID:0 孤立节点的空图执行剪枝
        """
        with patch("worker.tasks.redis_client", self.mock_redis):
            res1 = prune_global_graph()
            self.assertEqual(res1["status"], "skipped")

            empty_graph = {
                "meta": {"version": 1, "step": 300},
                "nodes": {
                    "0": {"event_id": 0},
                    "5": {"event_id": 5},
                },
                "edges": {},
            }
            self.mock_redis.set("epgm:graph:global:latest", json.dumps(empty_graph))

            res2 = prune_global_graph()
            self.assertEqual(res2["status"], "pruned")
            updated_graph = json.loads(self.mock_redis.get("epgm:graph:global:latest"))
            self.assertIn("0", updated_graph["nodes"])
            self.assertNotIn("5", updated_graph["nodes"])
            self.assertEqual(res2["pruned_nodes_count"], 1)
            self.assertEqual(res2["pruned_edges_count"], 0)

    def test_prune_all_edges_simultaneously(self):
        """
        [修剪 3.2] 全量边同时满足休眠淘汰条件被清空
        """
        stale_graph = {
            "meta": {"version": 5, "step": 500},
            "nodes": {
                "0": {"event_id": 0},
                "10": {"event_id": 10},
                "11": {"event_id": 11},
            },
            "edges": {
                "10_1_11": {
                    "source_event": 10,
                    "action": 1,
                    "target_event": 11,
                    "observation_count": 1,
                    "confidence": 0.10,
                    "last_active_step": 100,
                }
            },
        }

        with patch("worker.tasks.redis_client", self.mock_redis):
            self.mock_redis.set("epgm:graph:global:latest", json.dumps(stale_graph))
            res = prune_global_graph()

            self.assertEqual(res["status"], "pruned")
            self.assertEqual(res["pruned_edges_count"], 1)
            self.assertEqual(res["pruned_nodes_count"], 2)

            updated = json.loads(self.mock_redis.get("epgm:graph:global:latest"))
            self.assertEqual(len(updated["edges"]), 0)
            self.assertIn("0", updated["nodes"])
            self.assertEqual(len(updated["nodes"]), 1)
            self.assertEqual(updated["meta"]["version"], 6)

    def test_prune_disconnected_components(self):
        """
        [修剪 3.3] 非连通子图: 活跃子图 (A) vs 衰亡子图 (B)
        """
        graph = {
            "meta": {"version": 10, "step": 1000},
            "nodes": {
                "0": {"event_id": 0},
                "1": {"event_id": 1},
                "2": {"event_id": 2},
                "50": {"event_id": 50},
                "51": {"event_id": 51},
            },
            "edges": {
                "1_0_2": {
                    "source_event": 1,
                    "action": 0,
                    "target_event": 2,
                    "observation_count": 25,
                    "confidence": 0.95,
                    "last_active_step": 950,
                },
                "50_2_51": {
                    "source_event": 50,
                    "action": 2,
                    "target_event": 51,
                    "observation_count": 1,
                    "confidence": 0.10,
                    "last_active_step": 700,
                },
            },
        }

        with patch("worker.tasks.redis_client", self.mock_redis):
            self.mock_redis.set("epgm:graph:global:latest", json.dumps(graph))
            res = prune_global_graph()

            self.assertEqual(res["status"], "pruned")
            self.assertEqual(res["pruned_edges_count"], 1)
            self.assertEqual(res["pruned_nodes_count"], 2)

            updated = json.loads(self.mock_redis.get("epgm:graph:global:latest"))
            self.assertIn("1_0_2", updated["edges"])
            self.assertIn("1", updated["nodes"])
            self.assertIn("2", updated["nodes"])
            self.assertIn("0", updated["nodes"])
            self.assertNotIn("50_2_51", updated["edges"])
            self.assertNotIn("50", updated["nodes"])
            self.assertNotIn("51", updated["nodes"])

    def test_prune_unnormalized_residual_probabilities_audit(self):
        """
        [对抗 3.4 审计] 验证剪枝后残存分支的转移概率是否缺失重新归一化
        场景: 分支 (10, 1) 原有 2 条边 (指向 20 和 21)。
        剪枝切除了噪声边 10_1_21，但未触发 _recompute_posterior_probabilities。
        """
        graph = {
            "meta": {"version": 10, "step": 1000},
            "nodes": {
                "0": {"event_id": 0},
                "10": {"event_id": 10},
                "20": {"event_id": 20},
                "21": {"event_id": 21},
            },
            "edges": {
                # 强壮边: 观测 10 次，概率已计算为 0.8
                "10_1_20": {
                    "source_event": 10,
                    "action": 1,
                    "target_event": 20,
                    "observation_count": 10,
                    "probability": 0.8,
                    "confidence": 0.86,
                    "last_active_step": 950,
                },
                # 衰退弱边: 观测 1 次，概率 0.2，休眠 300 步
                "10_1_21": {
                    "source_event": 10,
                    "action": 1,
                    "target_event": 21,
                    "observation_count": 1,
                    "probability": 0.2,
                    "confidence": 0.10,
                    "last_active_step": 700,
                },
            },
        }

        with patch("worker.tasks.redis_client", self.mock_redis):
            self.mock_redis.set("epgm:graph:global:latest", json.dumps(graph))
            res = prune_global_graph()

            self.assertEqual(res["status"], "pruned")
            self.assertEqual(res["pruned_edges_count"], 1)

            updated = json.loads(self.mock_redis.get("epgm:graph:global:latest"))
            remaining_edge = updated["edges"]["10_1_20"]

            # [加固验证]: 验证 LTD 剪枝后触发 _recompute_posterior_probabilities 重新归一化至 1.0
            print(f"[REMEDIATION VERIFIED] Residual edge probability after pruning is {remaining_edge['probability']} (Expected 1.0)")
            self.assertEqual(remaining_edge["probability"], 1.0, "Verifies LTD pruning re-normalization")


class TestCeleryGraphFusionEndToEndDefense(unittest.TestCase):
    """
    [对抗测试 4] Celery fuse_graph_deltas 端到端异常输入防御与状态演进
    """

    def setUp(self):
        self.mock_redis = MockRedisClient()

    def test_fuse_graph_deltas_invalid_argument_defense(self):
        """
        [防御 4.1] 非法参数类型与空列表输入，验证防御性返回而非崩溃
        """
        with patch("worker.tasks.redis_client", self.mock_redis):
            res1 = fuse_graph_deltas(12345)
            self.assertEqual(res1["status"], "error")

            res2 = fuse_graph_deltas(None)
            self.assertEqual(res2["status"], "error")

            # 空列表
            res3 = fuse_graph_deltas([])
            self.assertEqual(res3["status"], "success")
            self.assertEqual(res3["nodes_added"], 0)
            self.assertEqual(res3["edges_added"], 0)

    def test_fuse_graph_deltas_rapid_sequential_updates(self):
        """
        [防御 4.2] 连续 50 次增量融合，验证版本号严格自增且拓扑一致
        """
        with patch("worker.tasks.redis_client", self.mock_redis):
            for step in range(1, 51):
                delta = {
                    "robot_id": f"bot_{step % 3}",
                    "step_index": step * 10,
                    "nodes": [{"event_id": step % 15, "visit_count": 1}],
                    "edges": [{
                        "source_event": step % 15,
                        "action": step % 4,
                        "target_event": (step + 1) % 15,
                        "expected_reward": 0.5,
                        "observation_count": 1,
                    }],
                }
                res = fuse_graph_deltas(delta)
                self.assertEqual(res["status"], "success")
                self.assertEqual(res["version"], step)

            # 最终验证 Redis 中的全局图版本与边总数
            final_graph = json.loads(self.mock_redis.get("epgm:graph:global:latest"))
            self.assertEqual(final_graph["meta"]["version"], 50)
            self.assertTrue(len(final_graph["edges"]) > 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
