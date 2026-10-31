"""
[ZH] EPGM 云端 MQTT 消息网关与分布式因果图聚合中继器。
[EN] EPGM Cloud MQTT Message Gateway and Distributed Causal Graph Aggregator Relay.

数学原理 / Mathematical Theory:
    实现群体因果图的狄利克雷-多项分布共轭贝叶斯融合 (Dirichlet-Multinomial Bayesian Conjugate Fusion)。
    接收各小车因果子图碎片并进行加权聚合后，生成全局优化模型并下发。
"""

import sys
import os
import json
import time
from typing import Dict, Any, List

# 将算法核心库引入以支持拓扑图合并
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "epgm-algorithm")))
from epgm.core import EventPredictiveGraph, EPGMEdge


class CloudGraphAggregator:
    """
    [ZH] 云端分布式因果图中心聚合器。
    [EN] Cloud Distributed Causal Graph Central Aggregator.
    """

    def __init__(self, action_dim: int = 5):
        self.global_graph = EventPredictiveGraph(action_dim=action_dim)
        self.registered_robots: Dict[str, Dict[str, Any]] = {}
        self.total_fragments_fused: int = 0
        self.last_sync_timestamp: float = time.time()

    def process_telemetry(self, robot_id: str, payload: Dict[str, Any]) -> None:
        """
        [ZH] 处理小车实时心跳与物理状态遥测。
        [EN] Process robot heartbeat and physical status telemetry.
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
        [ZH] 执行因果图碎片融合，更新全局先验知识图谱。
        [EN] Execute causal graph fragment fusion, updating global prior knowledge graph.
        """
        nodes_up, edges_up = self.global_graph.import_fragment(fragment)
        self.total_fragments_fused += 1

        print(
            f"[Cloud Fusion] 来自小车 [{robot_id}] 的图碎片已融合: "
            f"+{nodes_up} 节点, +{edges_up} 因果边 | "
            f"全局累计节点: {len(self.global_graph.nodes)}, 全局因果边: {sum(len(el) for el in self.global_graph.adjacency.values())}"
        )

        return {
            "status": "success",
            "nodes_updated": nodes_up,
            "edges_updated": edges_up,
            "global_total_nodes": len(self.global_graph.nodes),
            "global_total_edges": sum(len(el) for el in self.global_graph.adjacency.values()),
        }

    def get_distilled_global_model(self) -> Dict[str, Any]:
        """
        [ZH] 导出提炼优化后的全局因果图模型（用于增量下发给小车）。
        [EN] Export distilled and optimized global causal model (for downlink distribution).
        """
        return self.global_graph.export_fragment(since_step=0)


if __name__ == "__main__":
    aggregator = CloudGraphAggregator()
    print("EPGM Cloud Graph Aggregator initialized successfully.")
