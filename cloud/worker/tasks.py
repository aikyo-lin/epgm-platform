"""
[ZH] Celery 分布式异步因果图融合与长时程抑制 (LTD) 拓扑维护任务。
[EN] Celery Distributed Asynchronous Causal Graph Fusion & LTD Topological Maintenance Tasks.

数学理论与算法规范 / Mathematical Theory & Algorithmic Specifications:
1. 狄利克雷-多项分布共轭贝叶斯融合 (Dirichlet-Multinomial Bayesian Conjugate Update):
   对于条件动作分支 (v_i, a)，转移到目标事件 v_j 的后验概率期望:
   P*(v_j | v_i, a) = (alpha_0 + sum N_k(v_i, a, v_j)) / sum_m (alpha_0 + sum N_k(v_i, a, v_m))
   其中 alpha_0 为狄利克雷对称先验超参数 (弱信息先验 alpha_0 = 0.1)。
2. 加权即时回报更新 (Weighted Reward Expectation):
   R*(v_i, a, v_j) = (R_old * N_old + R_delta * N_delta) / (N_old + N_delta)
3. 指数饱和置信度与认知不确定度 (Exponential Confidence Saturation):
   C*(v_i, a, v_j) = 1 - exp(-lambda * N)
   U*(v_i, a, v_j) = exp(-lambda * N) = 1 - C*
   其中 lambda = 0.2 (相当于 N_0 = 5 次观测达到约 63.2% 置信度)。
4. 分布式并发原子性 (Distributed Lock):
   采用 Redis 分布式锁 `epgm:lock:graph_fusion` 保证多车并发合并时数据一致。
5. 拓扑演化多播与持久化快照:
   融合结果写入 Redis `epgm:graph:global:latest`，向 `epgm:channel:graph_update` 广播，
   并按版本周期归档至 PostgreSQL `graph_snapshots` 表。
"""

import os
import sys
import json
import time
import math
import logging
from typing import Dict, Any, List, Optional, Union

import redis
from celery import Celery

# 引入数据库会话与模型
try:
    from db.session import get_db_session
    from db.models import GraphSnapshot
except ImportError:
    try:
        from ..db.session import get_db_session
        from ..db.models import GraphSnapshot
    except ImportError:
        # 支持相对路径或外部测试环境
        sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
        from db.session import get_db_session
        from db.models import GraphSnapshot

# 日志配置
logger = logging.getLogger("CeleryGraphWorker")

# 环境变量配置
REDIS_URL = os.getenv("REDIS_URL", "redis://localhost:6379/0")
DATABASE_URL = os.getenv(
    "DATABASE_URL",
    "postgresql://epgm_admin:epgm_db_password@localhost:5432/epgm_knowledge"
)

# 贝叶斯融合数学超参数
DIRICHLET_ALPHA_0 = 0.1   # 狄利克雷对称弱先验 (Dirichlet concentration prior)
CONFIDENCE_LAMBDA = 0.2   # 指数置信度饱和系数 (lambda = 1/N_0, N_0 = 5)
SNAPSHOT_INTERVAL = 10    # 每 10 次版本融合生成一次 PostgreSQL 快照

# 初始化 Celery 应用
app = Celery("epgm_worker", broker=REDIS_URL, backend=REDIS_URL)

app.conf.update(
    task_serializer="json",
    accept_content=["json"],
    result_serializer="json",
    timezone="UTC",
    enable_utc=True,
    # 定时调度 Celery Beat 任务
    beat_schedule={
        "ltd_topological_pruning_60s": {
            "task": "tasks.prune_global_graph",
            "schedule": 60.0,
            "args": (),
        },
    },
)

# Redis 客户端 (用于读取最新图、获取分布式锁及发布广播)
redis_client = redis.Redis.from_url(REDIS_URL, decode_responses=True)


def _recompute_posterior_probabilities(
    edges_dict: Dict[str, Dict[str, Any]],
    source_event: int,
    action: int,
    alpha_0: float = DIRICHLET_ALPHA_0,
) -> None:
    """
    [ZH] 针对特定 (source_event, action) 条件分支，利用狄利克雷-多项分布共轭规则
         重新归一化计算所有目标后继事件的后验转移概率 P*(v_j | v_i, a)。
    [EN] Recompute posterior transition probabilities using Dirichlet-Multinomial
         conjugate rule for specific (source_event, action) conditional branch.
    """
    prefix = f"{source_event}_{action}_"
    branch_edges = [edge for key, edge in edges_dict.items() if key.startswith(prefix)]

    if not branch_edges:
        return

    # 分母：sum_m (alpha_0 + N_m)
    total_denominator = len(branch_edges) * alpha_0 + sum(e["observation_count"] for e in branch_edges)

    for edge in branch_edges:
        numerator = alpha_0 + edge["observation_count"]
        edge["probability"] = round(numerator / total_denominator, 4)


@app.task(name="tasks.fuse_graph_deltas")
def fuse_graph_deltas(
    arg1: Union[str, Dict[str, Any], List[Dict[str, Any]]],
    arg2: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    [ZH] 异步执行群体因果图增量碎片的狄利克雷-多项分布共轭贝叶斯融合。
         支持两种调用签名：
         - fuse_graph_deltas(robot_id: str, delta: dict)
         - fuse_graph_deltas(deltas: list) 或 fuse_graph_deltas(delta: dict)
    [EN] Asynchronously execute Dirichlet-Multinomial Bayesian Conjugate Fusion.
    """
    # 统一参数解构
    robot_id = "swarm_agent"
    deltas: List[Dict[str, Any]] = []

    if isinstance(arg1, str) and isinstance(arg2, dict):
        robot_id = arg1
        deltas = [arg2]
    elif isinstance(arg1, dict):
        robot_id = arg1.get("robot_id", "unknown")
        deltas = [arg1]
    elif isinstance(arg1, list):
        deltas = arg1
        if deltas and isinstance(deltas[0], dict):
            robot_id = deltas[0].get("robot_id", "unknown")
    else:
        logger.error(f"Invalid arguments to fuse_graph_deltas: arg1={type(arg1)}, arg2={type(arg2)}")
        return {"status": "error", "message": "Invalid argument types"}

    logger.info(f"Starting Bayesian Graph Fusion for robot [{robot_id}] ({len(deltas)} deltas)...")

    lock_key = "epgm:lock:graph_fusion"
    lock = None
    lock_acquired = False

    try:
        # 获取分布式锁确保全局图读写原子性 (超时 10 秒)
        try:
            lock = redis_client.lock(lock_key, timeout=10.0, blocking_timeout=5.0)
            lock_acquired = lock.acquire(blocking=True)
        except Exception as lock_err:
            logger.warning(f"Failed to acquire Redis distributed lock ({lock_err}), proceeding with caution.")

        # ----------------------------------------------------------------------
        # 1. 从 Redis 读取全局因果图 (无缓存则建立空图)
        # ----------------------------------------------------------------------
        graph_raw = None
        try:
            graph_raw = redis_client.get("epgm:graph:global:latest")
        except Exception as e:
            logger.warning(f"Error reading global graph from Redis: {e}")

        if graph_raw:
            try:
                global_graph = json.loads(graph_raw)
            except Exception:
                global_graph = {"meta": {"version": 0, "step": 0}, "nodes": {}, "edges": {}}
        else:
            global_graph = {"meta": {"version": 0, "step": 0}, "nodes": {}, "edges": {}}

        nodes_dict: Dict[str, Dict[str, Any]] = global_graph.setdefault("nodes", {})
        edges_dict: Dict[str, Dict[str, Any]] = global_graph.setdefault("edges", {})

        total_nodes_added = 0
        total_edges_added = 0
        total_edges_updated = 0
        latest_step = global_graph.get("meta", {}).get("step", 0)

        affected_branches = set()
        primary_edge = None

        # ----------------------------------------------------------------------
        # 2. 遍历合并节点集 (Nodes Fusion)
        # ----------------------------------------------------------------------
        for delta in deltas:
            delta_step = delta.get("step_index", 0)
            if delta_step > latest_step:
                latest_step = delta_step

            for node in delta.get("nodes", []):
                nid = str(node.get("event_id"))
                visit_cnt = node.get("visit_count", 1)
                proto = node.get("prototype", [])

                if nid not in nodes_dict:
                    nodes_dict[nid] = {
                        "event_id": int(nid),
                        "visit_count": visit_cnt,
                        "prototype": proto,
                        "created_step": delta_step,
                    }
                    total_nodes_added += 1
                else:
                    nodes_dict[nid]["visit_count"] += visit_cnt
                    if proto and not nodes_dict[nid].get("prototype"):
                        nodes_dict[nid]["prototype"] = proto

        # ----------------------------------------------------------------------
        # 3. 遍历共轭融合因果边 (Edges Conjugate Fusion)
        # ----------------------------------------------------------------------
        for delta in deltas:
            delta_step = delta.get("step_index", 0)

            for edge in delta.get("edges", []):
                src = int(edge.get("source_event"))
                act = int(edge.get("action"))
                dst = int(edge.get("target_event"))
                delta_n = edge.get("observation_count", 1)
                delta_r = float(edge.get("expected_reward", 0.0))

                edge_key = f"{src}_{act}_{dst}"
                affected_branches.add((src, act))

                if edge_key not in edges_dict:
                    # 3.1 新因果预测边生长 (New Causal Edge Sprouting)
                    new_n = delta_n
                    new_r = delta_r
                    new_conf = 1.0 - math.exp(-CONFIDENCE_LAMBDA * new_n)
                    new_unc = 1.0 - new_conf

                    edge_entry = {
                        "source_event": src,
                        "action": act,
                        "target_event": dst,
                        "expected_reward": round(new_r, 4),
                        "observation_count": new_n,
                        "confidence": round(new_conf, 4),
                        "uncertainty": round(new_unc, 4),
                        "probability": 1.0,
                        "weight": round(math.log1p(new_n), 4),
                        "last_active_step": delta_step,
                    }
                    edges_dict[edge_key] = edge_entry
                    total_edges_added += 1
                    primary_edge = edge_entry
                else:
                    # 3.2 贝叶斯共轭更新已有因果边 (Conjugate Bayesian Aggregation)
                    cur = edges_dict[edge_key]
                    old_n = cur["observation_count"]
                    old_r = cur["expected_reward"]

                    # 加权更新样本均值
                    new_n = old_n + delta_n
                    new_r = (old_r * old_n + delta_r * delta_n) / new_n

                    # 指数饱和置信度与认知不确定度
                    new_conf = 1.0 - math.exp(-CONFIDENCE_LAMBDA * new_n)
                    new_unc = 1.0 - new_conf

                    cur["expected_reward"] = round(new_r, 4)
                    cur["observation_count"] = new_n
                    cur["confidence"] = round(new_conf, 4)
                    cur["uncertainty"] = round(new_unc, 4)
                    cur["weight"] = round(math.log1p(new_n), 4)
                    cur["last_active_step"] = max(cur.get("last_active_step", 0), delta_step)

                    total_edges_updated += 1
                    primary_edge = cur

        # ----------------------------------------------------------------------
        # 4. 重新计算受影响分支的狄利克雷后验转移概率 P*(v_j | v_i, a)
        # ----------------------------------------------------------------------
        for src, act in affected_branches:
            _recompute_posterior_probabilities(edges_dict, src, act, DIRICHLET_ALPHA_0)

        # ----------------------------------------------------------------------
        # 5. 更新元数据并回写 Redis 缓存与发布 PubSub
        # ----------------------------------------------------------------------
        cur_version = global_graph.get("meta", {}).get("version", 0) + 1
        global_graph["meta"] = {
            "version": cur_version,
            "step": latest_step,
            "total_nodes": len(nodes_dict),
            "total_edges": len(edges_dict),
            "last_updated_at": time.time(),
        }

        # 序列化为 JSON 写入 Redis 核心缓存
        try:
            redis_client.set("epgm:graph:global:latest", json.dumps(global_graph))
            redis_client.set("epgm:graph:global:version", cur_version)
        except Exception as e:
            logger.warning(f"Error persisting global graph to Redis: {e}")

        # 格式化代表性边供客户端即时渲染
        broadcast_edge = None
        if primary_edge:
            broadcast_edge = {
                "source": primary_edge["source_event"],
                "action": primary_edge["action"],
                "target": primary_edge["target_event"],
                "weight": primary_edge.get("weight", 1.0),
                "probability": primary_edge.get("probability", 1.0),
                "confidence": primary_edge.get("confidence", 0.5),
                "observation_count": primary_edge["observation_count"],
                "reward": primary_edge["expected_reward"],
            }

        # 向 Redis 广播频道推流
        try:
            broadcast_msg = {
                "type": "graph_updated",
                "timestamp": time.time(),
                "robot_id": robot_id,
                "version": cur_version,
                "total_nodes": len(nodes_dict),
                "total_edges": len(edges_dict),
                "nodes_added": total_nodes_added,
                "edges_added": total_edges_added,
                "edges_updated": total_edges_updated,
                "edge": broadcast_edge,
            }
            redis_client.publish("epgm:channel:graph_update", json.dumps(broadcast_msg))
        except Exception as e:
            logger.warning(f"Error publishing to epgm:channel:graph_update: {e}")

        # ----------------------------------------------------------------------
        # 6. 周期性归档拓扑快照至 PostgreSQL graph_snapshots 表
        # ----------------------------------------------------------------------
        if cur_version % SNAPSHOT_INTERVAL == 0 or total_nodes_added > 0:
            try:
                avg_conf = (
                    sum(e.get("confidence", 0.0) for e in edges_dict.values()) / max(len(edges_dict), 1)
                )
                with get_db_session() as session:
                    snapshot = GraphSnapshot(
                        version_tag=f"v_{cur_version}_{int(time.time())}",
                        step_index=latest_step,
                        total_nodes=len(nodes_dict),
                        total_edges=len(edges_dict),
                        graph_json=global_graph,
                        trigger_robot_id=robot_id,
                        metrics={
                            "avg_confidence": round(avg_conf, 4),
                            "nodes_added": total_nodes_added,
                            "edges_added": total_edges_added,
                        },
                    )
                    session.add(snapshot)
                logger.info(f"Persisted GraphSnapshot version {cur_version} to PostgreSQL.")
            except Exception as e:
                logger.warning(f"PostgreSQL graph snapshot persistence failed: {e}")

    finally:
        # 确保安全释放分布式锁
        if lock and lock_acquired:
            try:
                lock.release()
            except Exception:
                pass

    logger.info(
        f"Graph Fusion Complete: Version {cur_version} | "
        f"Nodes: {len(nodes_dict)} (+{total_nodes_added}) | "
        f"Edges: {len(edges_dict)} (+{total_edges_added}, ~{total_edges_updated})"
    )

    return {
        "status": "success",
        "version": cur_version,
        "total_nodes": len(nodes_dict),
        "total_edges": len(edges_dict),
        "nodes_added": total_nodes_added,
        "edges_added": total_edges_added,
    }


@app.task(name="tasks.prune_global_graph")
def prune_global_graph() -> Dict[str, Any]:
    """
    [ZH] 周期性全局因果拓扑图长时程抑制 (LTD, Long-Term Depression) 剪枝维护。
         规则:
         1. 淘汰低置信度、低观测频次且长期休眠的偶然噪声因果边；
         2. 移除入度与出度均为 0 的非核心孤立节点 (保留 ID:0 碰撞基元)；
         3. 重新归一化转移概率并回写 Redis 和持久化快照。
    [EN] Periodic Global Graph Long-Term Depression (LTD) Topological Pruning Task.
    """
    logger.info("Executing periodic LTD topological pruning task...")
    lock_key = "epgm:lock:graph_fusion"
    lock = None
    lock_acquired = False

    try:
        try:
            lock = redis_client.lock(lock_key, timeout=10.0, blocking_timeout=5.0)
            lock_acquired = lock.acquire(blocking=True)
        except Exception:
            pass

        graph_raw = redis_client.get("epgm:graph:global:latest")
        if not graph_raw:
            return {"status": "skipped", "reason": "No graph in cache"}

        global_graph = json.loads(graph_raw)
        nodes_dict: Dict[str, Dict[str, Any]] = global_graph.get("nodes", {})
        edges_dict: Dict[str, Dict[str, Any]] = global_graph.get("edges", {})

        current_step = global_graph.get("meta", {}).get("step", 0)
        pruned_edges = 0
        pruned_nodes = 0

        # ----------------------------------------------------------------------
        # 1. 剪除低置信度且休眠的弱因果边
        # ----------------------------------------------------------------------
        keys_to_delete = []
        for key, edge in edges_dict.items():
            obs = edge.get("observation_count", 1)
            conf = edge.get("confidence", 0.0)
            last_active = edge.get("last_active_step", 0)

            # 休眠超过 200 步且观测次数低于 2 且置信度极低
            if obs < 2 and conf < 0.15 and (current_step - last_active > 200):
                keys_to_delete.append(key)

        for k in keys_to_delete:
            del edges_dict[k]
            pruned_edges += 1

        # ----------------------------------------------------------------------
        # 2. 剪除完全孤立的非核心事件节点 (保留 ID:0 物理碰撞)
        # ----------------------------------------------------------------------
        connected_node_ids = set()
        for edge in edges_dict.values():
            connected_node_ids.add(str(edge["source_event"]))
            connected_node_ids.add(str(edge["target_event"]))

        node_keys_to_delete = []
        for nid in nodes_dict.keys():
            if nid != "0" and nid not in connected_node_ids:
                node_keys_to_delete.append(nid)

        for nid in node_keys_to_delete:
            del nodes_dict[nid]
            pruned_nodes += 1

        # ----------------------------------------------------------------------
        # 3. 若有剪枝变动，更新全局版本并广播
        # ----------------------------------------------------------------------
        if pruned_edges > 0 or pruned_nodes > 0:
            cur_version = global_graph.get("meta", {}).get("version", 0) + 1
            global_graph["meta"]["version"] = cur_version
            global_graph["meta"]["total_nodes"] = len(nodes_dict)
            global_graph["meta"]["total_edges"] = len(edges_dict)

            redis_client.set("epgm:graph:global:latest", json.dumps(global_graph))

            # 广播剪枝通知
            redis_client.publish(
                "epgm:channel:graph_update",
                json.dumps({
                    "type": "graph_pruned",
                    "version": cur_version,
                    "pruned_edges": pruned_edges,
                    "pruned_nodes": pruned_nodes,
                    "total_nodes": len(nodes_dict),
                    "total_edges": len(edges_dict),
                }),
            )
            logger.info(f"LTD Pruning Complete: -{pruned_edges} edges, -{pruned_nodes} nodes.")

        return {
            "status": "pruned",
            "pruned_edges_count": pruned_edges,
            "pruned_nodes_count": pruned_nodes,
            "total_nodes": len(nodes_dict),
            "total_edges": len(edges_dict),
        }

    finally:
        if lock and lock_acquired:
            try:
                lock.release()
            except Exception:
                pass
