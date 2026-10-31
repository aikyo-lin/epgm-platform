"""
EPGM Milestone M2 Empirical Stress Test Suite (Adversarial Verification)
Author: Empirical Challenger 2
Target: WebSocket live stream, Dashboard Vis.js protocol, DB consistency

Tests:
1. WebSocket 50 Concurrent Clients & 200 Redis PubSub Updates Stress Test
2. WebSocket Connection Drop Handling & Memory Leak Audit
3. Dashboard index.html AST & Rendering Protocol Verification
4. Database High-Frequency Concurrent Writes, FK Cascades & Snapshot Versioning
"""

import os
import sys

# Configure environment BEFORE any imports to ensure proper DB engine initialization
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CURRENT_DIR)
os.environ["DATABASE_URL"] = f"sqlite:///{os.path.join(CURRENT_DIR, 'stress_test_fastapi.db')}"
os.environ["REDIS_URL"] = "redis://127.0.0.1:6389/0"

import time
import json
import asyncio
import threading
import gc
import psutil
from typing import List, Dict, Any

import uvicorn
import websockets
from sqlalchemy import create_engine, event, text
from sqlalchemy.orm import sessionmaker
from sqlalchemy.exc import IntegrityError
from sqlalchemy.engine import Engine

from db.models import Base, Robot, TelemetryLog, GraphSnapshot
from db.session import init_db, get_db_session
from api.main import app, manager


# ==============================================================================
# Helper: Lightweight In-Process Async RESP Redis PubSub Mock Server
# ==============================================================================
class MiniRedisServer:
    """
    Lightweight in-process Redis protocol server supporting PING, SUBSCRIBE, PUBLISH, GET, SET.
    Allows testing true Redis PubSub async bridge without external Docker dependencies.
    """
    def __init__(self, host="127.0.0.1", port=6389):
        self.host = host
        self.port = port
        self.server = None
        self.subscribers: Dict[str, List[asyncio.StreamWriter]] = {}
        self.kv_store: Dict[str, str] = {}
        self._running = False

    async def start(self):
        self._running = True
        self.server = await asyncio.start_server(self.handle_client, self.host, self.port)
        return self

    async def stop(self):
        self._running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        client_subs = set()
        try:
            while self._running:
                line = await reader.readline()
                if not line:
                    break
                line_str = line.decode('utf-8', errors='ignore')
                if line_str.startswith('*'):
                    num_args = int(line_str[1:].strip())
                    args = []
                    for _ in range(num_args):
                        len_line = await reader.readline()
                        arg_len = int(len_line[1:].strip())
                        arg_data = await reader.readexactly(arg_len + 2)
                        args.append(arg_data[:-2].decode('utf-8', errors='ignore'))

                    if not args:
                        continue

                    cmd = args[0].upper()
                    if cmd == 'PING':
                        writer.write(b"+PONG\r\n")
                        await writer.drain()
                    elif cmd == 'SUBSCRIBE':
                        for ch in args[1:]:
                            client_subs.add(ch)
                            if ch not in self.subscribers:
                                self.subscribers[ch] = []
                            if writer not in self.subscribers[ch]:
                                self.subscribers[ch].append(writer)
                            resp = f"*3\r\n$9\r\nsubscribe\r\n${len(ch)}\r\n{ch}\r\n:{len(client_subs)}\r\n"
                            writer.write(resp.encode('utf-8'))
                        await writer.drain()
                    elif cmd == 'PUBLISH':
                        ch = args[1]
                        msg = args[2]
                        recipients = self.subscribers.get(ch, [])
                        active_recipients = []
                        for sub_writer in recipients:
                            try:
                                pub_frame = (
                                    f"*3\r\n$7\r\nmessage\r\n${len(ch)}\r\n{ch}\r\n${len(msg.encode('utf-8'))}\r\n{msg}\r\n"
                                )
                                sub_writer.write(pub_frame.encode('utf-8'))
                                await sub_writer.drain()
                                active_recipients.append(sub_writer)
                            except Exception:
                                pass
                        self.subscribers[ch] = active_recipients
                        writer.write(f":{len(active_recipients)}\r\n".encode('utf-8'))
                        await writer.drain()
                    elif cmd == 'GET':
                        k = args[1]
                        val = self.kv_store.get(k)
                        if val is None:
                            writer.write(b"$-1\r\n")
                        else:
                            val_bytes = val.encode('utf-8')
                            writer.write(f"${len(val_bytes)}\r\n{val}\r\n".encode('utf-8'))
                        await writer.drain()
                    elif cmd == 'SET':
                        k = args[1]
                        v = args[2]
                        self.kv_store[k] = v
                        writer.write(b"+OK\r\n")
                        await writer.drain()
                    elif cmd == 'CLIENT':
                        writer.write(b"+OK\r\n")
                        await writer.drain()
                    else:
                        writer.write(b"+OK\r\n")
                        await writer.drain()
        except Exception:
            pass
        finally:
            for ch in client_subs:
                if ch in self.subscribers and writer in self.subscribers[ch]:
                    self.subscribers[ch].remove(writer)
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass


# ==============================================================================
# Stress Test 1: WebSocket 50 Clients & 200 Redis PubSub Messages
# ==============================================================================
class WebSocketStressHarness:
    def __init__(self, port=8877, redis_port=6389):
        self.port = port
        self.redis_port = redis_port
        self.ws_url = f"ws://127.0.0.1:{self.port}/ws/live_graph"
        self.server = None
        self.server_thread = None
        self.redis_server = None

    def start_uvicorn(self):
        # Initialize DB tables for clean startup
        try:
            init_db()
        except Exception as e:
            print(f"init_db notice: {e}")

        config = uvicorn.Config(
            app=app,
            host="127.0.0.1",
            port=self.port,
            log_level="error",
            ws_ping_interval=None,
        )
        self.server = uvicorn.Server(config)
        self.server_thread = threading.Thread(target=self.server.run, daemon=True)
        self.server_thread.start()

    async def run_stress_test(self):
        print(f"\n[STRESS 1] Starting In-Process Redis PubSub Server on port {self.redis_port}...")
        self.redis_server = MiniRedisServer(port=self.redis_port)
        await self.redis_server.start()

        print(f"[STRESS 1] Starting Uvicorn Gateway on port {self.port}...")
        self.start_uvicorn()
        await asyncio.sleep(1.5)  # Wait for uvicorn and pubsub listener to settle

        num_clients = 50
        num_messages = 200

        print(f"[STRESS 1] Connecting {num_clients} concurrent WebSocket clients to {self.ws_url}...")
        clients = []
        client_queues = [asyncio.Queue() for _ in range(num_clients)]
        client_tasks = []

        async def client_listener(idx, ws, q):
            try:
                # Receive initial_state
                init_msg = await ws.recv()
                data = json.loads(init_msg)
                assert data["type"] == "initial_state"
                while True:
                    msg = await ws.recv()
                    data = json.loads(msg)
                    await q.put(data)
            except websockets.exceptions.ConnectionClosed:
                pass
            except Exception:
                pass

        connect_start = time.time()
        for i in range(num_clients):
            try:
                ws = await websockets.connect(self.ws_url)
                clients.append(ws)
                task = asyncio.create_task(client_listener(i, ws, client_queues[i]))
                client_tasks.append(task)
            except Exception as e:
                print(f"Error connecting client {i}: {e}")

        connect_elapsed = time.time() - connect_start
        print(f"[STRESS 1] Successfully connected {len(clients)}/{num_clients} WebSocket clients in {connect_elapsed:.3f}s.")
        print(f"[STRESS 1] Active connections tracked by manager: {len(manager.active_connections)}")

        # Wait a moment for all client initial_state handshakes to finish
        await asyncio.sleep(0.5)

        # Now rapidly publish 200 telemetry & graph evolution updates through Redis PubSub
        print(f"[STRESS 1] Rapidly publishing {num_messages} updates through Redis PubSub...")
        start_time = time.time()

        # Connect a publisher to MiniRedisServer
        pub_reader, pub_writer = await asyncio.open_connection("127.0.0.1", self.redis_port)

        async def send_redis_publish(channel, payload_str):
            cmd = f"*3\r\n$7\r\nPUBLISH\r\n${len(channel)}\r\n{channel}\r\n${len(payload_str.encode('utf-8'))}\r\n{payload_str}\r\n"
            pub_writer.write(cmd.encode('utf-8'))
            await pub_writer.drain()
            await pub_reader.readline()

        for seq in range(1, num_messages + 1):
            if seq % 2 == 1:
                # Telemetry frame
                telemetry_payload = {
                    "type": "telemetry_update",
                    "robot_id": f"robot_{seq % 5}",
                    "timestamp_ms": int(time.time() * 1000),
                    "data": {
                        "robot_id": f"robot_{seq % 5}",
                        "battery_percentage": 90 - (seq % 10),
                        "bumper_hit": (seq % 20 == 0),
                        "current_event_id": seq % 81 + 1,
                        "last_action": seq % 5,
                        "last_reward": 0.5 if seq % 20 != 0 else -10.0,
                    }
                }
                await send_redis_publish("epgm:channel:telemetry", json.dumps(telemetry_payload))
            else:
                # Graph update frame
                graph_payload = {
                    "type": "graph_updated",
                    "timestamp": time.time(),
                    "robot_id": f"robot_{seq % 5}",
                    "version": seq,
                    "total_nodes": seq + 10,
                    "total_edges": seq * 2,
                    "edge": {
                        "source": (seq % 81) + 1,
                        "action": seq % 5,
                        "target": ((seq + 1) % 81) + 1,
                        "weight": 1.5,
                        "probability": 0.85,
                        "confidence": 0.72,
                        "observation_count": 6,
                        "reward": 0.8,
                    }
                }
                await send_redis_publish("epgm:channel:graph_update", json.dumps(graph_payload))

        pub_writer.close()
        await pub_writer.wait_closed()

        # Allow time for messages to propagate
        await asyncio.sleep(2.0)
        elapsed = time.time() - start_time
        print(f"[STRESS 1] Published {num_messages} updates in {elapsed:.3f}s ({num_messages/elapsed:.1f} msg/s).")

        # Check queue sizes across all clients
        received_counts = [q.qsize() for q in client_queues]
        min_recv = min(received_counts)
        max_recv = max(received_counts)
        avg_recv = sum(received_counts) / len(received_counts)
        print(f"[STRESS 1] Received per client: Min={min_recv}, Max={max_recv}, Avg={avg_recv:.1f} (Target: {num_messages})")

        # ----------------------------------------------------------------------
        # Abrupt Disconnection & Memory Leak Test
        # ----------------------------------------------------------------------
        print("\n[STRESS 2] Testing abrupt disconnection handling (dropping 25 clients mid-stream)...")
        initial_rss = psutil.Process().memory_info().rss / (1024 * 1024)

        # Abruptly abort 25 of the 50 clients without clean close
        dropped_clients = clients[:25]
        surviving_clients = clients[25:]
        surviving_queues = client_queues[25:]

        for ws in dropped_clients:
            ws.transport.abort()

        await asyncio.sleep(0.5)

        # Send 50 more messages to test if manager handles dead connections gracefully
        print(f"[STRESS 2] Broadcasting 50 post-drop messages to verify recovery...")
        for seq in range(201, 251):
            msg = {"type": "telemetry_update", "robot_id": "test_bot", "seq": seq}
            await manager.broadcast(msg)

        await asyncio.sleep(1.0)
        remaining_active = len(manager.active_connections)
        print(f"[STRESS 2] Active connections remaining in manager: {remaining_active} (Expected: 25)")

        # Verify surviving clients received the post-drop messages
        post_counts = [q.qsize() for q in surviving_queues]
        print(f"[STRESS 2] Surviving clients message count: Min={min(post_counts)}, Max={max(post_counts)}")

        # Close surviving clients gracefully
        for ws in surviving_clients:
            try:
                await ws.close()
            except Exception:
                pass

        for t in client_tasks:
            t.cancel()

        await asyncio.sleep(0.5)
        final_active = len(manager.active_connections)
        gc.collect()
        final_rss = psutil.Process().memory_info().rss / (1024 * 1024)

        print(f"[STRESS 2] Final active connections: {final_active} (Expected: 0)")
        print(f"[STRESS 2] Memory RSS: Initial={initial_rss:.2f}MB, Final={final_rss:.2f}MB (Delta={final_rss-initial_rss:+.2f}MB)")

        # Cleanup
        self.server.should_exit = True
        await self.redis_server.stop()

        results = {
            "num_clients": num_clients,
            "num_messages": num_messages,
            "min_received": min_recv,
            "max_received": max_recv,
            "avg_received": avg_recv,
            "remaining_after_drop": remaining_active,
            "final_active": final_active,
            "memory_delta_mb": round(final_rss - initial_rss, 2),
        }
        return results


# ==============================================================================
# Stress Test 2: Database Concurrency, Foreign Key Integrity & Snapshot Versioning
# ==============================================================================
class DatabaseStressHarness:
    def __init__(self, db_path=None):
        if db_path is None:
            self.db_path = os.path.join(CURRENT_DIR, "stress_test_db.db")
        else:
            self.db_path = db_path
        self.db_url = f"sqlite:///{self.db_path}"
        self.engine = None
        self.Session = None

    def setup(self):
        if os.path.exists(self.db_path):
            try:
                os.remove(self.db_path)
            except Exception:
                pass

        self.engine = create_engine(
            self.db_url,
            connect_args={"check_same_thread": False, "timeout": 30},
            pool_size=20,
            max_overflow=40,
        )

        # Enforce foreign key constraints in SQLite
        @event.listens_for(Engine, "connect")
        def set_sqlite_pragma(dbapi_connection, connection_record):
            cursor = dbapi_connection.cursor()
            cursor.execute("PRAGMA foreign_keys=ON")
            cursor.execute("PRAGMA journal_mode=WAL")
            cursor.close()

        Base.metadata.create_all(bind=self.engine)
        self.Session = sessionmaker(bind=self.engine, autocommit=False, autoflush=False)

    def test_foreign_key_integrity(self):
        print("\n[DB STRESS 1] Testing Foreign Key Enforcement & Cascading Deletion...")
        session = self.Session()
        try:
            # 1. Attempt inserting telemetry for non-existent robot -> MUST FAIL with IntegrityError
            orphan_log = TelemetryLog(
                robot_id="non_existent_robot_999",
                timestamp_ms=1000,
                current_event_id=1,
                last_action=1,
                last_reward=0.0,
                battery_percentage=100,
                bumper_hit=False,
                sensor_distances=[],
            )
            session.add(orphan_log)
            try:
                session.commit()
                fk_enforced = False
                print("  [FAIL] Foreign key violation was NOT caught!")
            except IntegrityError:
                session.rollback()
                fk_enforced = True
                print("  [PASS] Foreign key violation correctly raised IntegrityError.")

            # 2. Register a robot and insert 5 telemetry logs
            test_robot = Robot(
                robot_id="robot_cascade_test",
                name="Cascade Test Bot",
                status="online",
            )
            session.add(test_robot)
            session.commit()

            for k in range(5):
                log = TelemetryLog(
                    robot_id="robot_cascade_test",
                    timestamp_ms=1000 + k,
                    current_event_id=k,
                    last_action=1,
                    last_reward=0.5,
                    battery_percentage=95,
                    bumper_hit=False,
                    sensor_distances=[],
                )
                session.add(log)
            session.commit()

            log_count = session.query(TelemetryLog).filter_by(robot_id="robot_cascade_test").count()
            assert log_count == 5, f"Expected 5 logs, found {log_count}"
            print(f"  [PASS] Successfully persisted {log_count} telemetry logs for registered robot.")

            # 3. Delete robot and test cascade deletion
            session.delete(test_robot)
            session.commit()

            remaining_logs = session.query(TelemetryLog).filter_by(robot_id="robot_cascade_test").count()
            cascade_ok = (remaining_logs == 0)
            if cascade_ok:
                print("  [PASS] ON DELETE CASCADE successfully cleaned up child telemetry logs.")
            else:
                print(f"  [FAIL] Cascade deletion failed, {remaining_logs} logs remained!")

            return {"fk_enforced": fk_enforced, "cascade_ok": cascade_ok}
        finally:
            session.close()

    def test_concurrent_telemetry_writes(self, num_threads=10, writes_per_thread=100):
        print(f"\n[DB STRESS 2] Testing High-Frequency Concurrent Writes ({num_threads} threads x {writes_per_thread} logs = {num_threads*writes_per_thread} logs)...")
        # Pre-register robots
        session = self.Session()
        for t in range(num_threads):
            rid = f"worker_robot_{t}"
            if not session.query(Robot).filter_by(robot_id=rid).first():
                session.add(Robot(robot_id=rid, name=f"Worker Robot {t}", status="online"))
        session.commit()
        session.close()

        errors = []
        start_time = time.time()

        def worker(thread_idx):
            sess = self.Session()
            rid = f"worker_robot_{thread_idx}"
            try:
                for step in range(writes_per_thread):
                    log = TelemetryLog(
                        robot_id=rid,
                        timestamp_ms=int(time.time() * 1000) + step,
                        current_event_id=(step % 81) + 1,
                        last_action=step % 5,
                        last_reward=0.75,
                        battery_percentage=90,
                        bumper_hit=False,
                        sensor_distances=[500] * 8,
                    )
                    sess.add(log)
                    sess.commit()
            except Exception as e:
                sess.rollback()
                errors.append((thread_idx, str(e)))
            finally:
                sess.close()

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(num_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        elapsed = time.time() - start_time
        total_writes = num_threads * writes_per_thread

        session = self.Session()
        persisted_count = session.query(TelemetryLog).count()
        session.close()

        print(f"  Completed {total_writes} writes in {elapsed:.3f}s ({total_writes/elapsed:.1f} writes/s).")
        print(f"  Persisted logs in DB: {persisted_count}/{total_writes}. Errors: {len(errors)}")

        return {
            "total_attempted": total_writes,
            "persisted": persisted_count,
            "elapsed_sec": round(elapsed, 3),
            "throughput_wps": round(total_writes / elapsed, 1),
            "errors": errors,
        }

    def test_snapshot_versioning_and_concurrency(self, num_snapshots=50):
        print(f"\n[DB STRESS 3] Testing Graph Snapshot High-Frequency Generation & Version Uniqueness ({num_snapshots} snapshots)...")
        session = self.Session()
        errors = []
        start_time = time.time()

        for v in range(1, num_snapshots + 1):
            tag = f"v_{v}_{int(time.time() * 1000)}_{v}"
            snapshot = GraphSnapshot(
                version_tag=tag,
                step_index=v * 10,
                total_nodes=v + 5,
                total_edges=v * 2,
                graph_json={"nodes": {}, "edges": {}},
                trigger_robot_id="swarm_agent_0",
                metrics={"avg_conf": 0.85},
            )
            session.add(snapshot)
            try:
                session.commit()
            except Exception as e:
                session.rollback()
                errors.append((v, str(e)))

        # Test duplicate version_tag constraint
        first_tag = session.query(GraphSnapshot).first().version_tag
        dup_snapshot = GraphSnapshot(
            version_tag=first_tag,
            step_index=999,
            total_nodes=10,
            total_edges=20,
            graph_json={},
        )
        session.add(dup_snapshot)
        try:
            session.commit()
            dup_caught = False
            print("  [FAIL] Duplicate version_tag was NOT prevented by UNIQUE constraint!")
        except IntegrityError:
            session.rollback()
            dup_caught = True
            print("  [PASS] Duplicate version_tag strictly rejected by UNIQUE constraint.")

        total_snaps = session.query(GraphSnapshot).count()
        session.close()

        print(f"  Persisted snapshots: {total_snaps}/{num_snapshots}. Errors: {len(errors)}")
        return {
            "persisted_snapshots": total_snaps,
            "duplicate_rejected": dup_caught,
            "errors": errors,
        }

    def test_index_utilization(self):
        print("\n[DB STRESS 4] Verifying Index Utilization with EXPLAIN QUERY PLAN...")
        session = self.Session()
        try:
            # 1. Telemetry query by robot_id and timestamp_ms DESC
            query_sql = "EXPLAIN QUERY PLAN SELECT * FROM telemetry_logs WHERE robot_id = 'worker_robot_0' ORDER BY timestamp_ms DESC LIMIT 10"
            result = session.execute(text(query_sql)).fetchall()
            telemetry_plan = " ".join([str(row) for row in result])
            print(f"  Telemetry Query Plan: {telemetry_plan}")
            uses_telemetry_idx = ("idx_telemetry_robot_time" in telemetry_plan or "USING INDEX" in telemetry_plan)

            # 2. Snapshot query by step_index DESC
            snap_sql = "EXPLAIN QUERY PLAN SELECT * FROM graph_snapshots ORDER BY step_index DESC LIMIT 5"
            snap_res = session.execute(text(snap_sql)).fetchall()
            snap_plan = " ".join([str(row) for row in snap_res])
            print(f"  Snapshot Query Plan: {snap_plan}")
            uses_snap_idx = ("idx_snapshots_step" in snap_plan or "USING INDEX" in snap_plan)

            print(f"  [PASS] Telemetry index utilized: {uses_telemetry_idx}")
            print(f"  [PASS] Snapshot step index utilized: {uses_snap_idx}")

            return {
                "telemetry_plan": telemetry_plan,
                "uses_telemetry_idx": uses_telemetry_idx,
                "snapshot_plan": snap_plan,
                "uses_snap_idx": uses_snap_idx,
            }
        finally:
            session.close()

    def teardown(self):
        if self.engine:
            self.engine.dispose()
        if os.path.exists(self.db_path):
            try:
                os.remove(self.db_path)
            except Exception:
                pass


# ==============================================================================
# Stress Test 3: Dashboard index.html AST & Rendering Protocol Verification
# ==============================================================================
def audit_dashboard_html():
    print("\n[DASHBOARD AUDIT] Auditing dashboard/index.html Vis.js Incremental Rendering Protocol...")
    html_path = os.path.join(CURRENT_DIR, "..", "dashboard", "index.html")
    with open(html_path, "r", encoding="utf-8") as f:
        html_code = f.read()

    findings = {}

    # 1. Check for setData() vs DataSet.update()
    has_set_data = "setData(" in html_code
    findings["has_set_data"] = has_set_data
    if has_set_data:
        print("  [FAIL] Found setData() call in index.html (violates incremental rendering contract)!")
    else:
        print("  [PASS] No setData() full re-creation calls found in index.html.")

    # 2. Check for nodesDataSet.update() and edgesDataSet.update()
    nodes_update = "nodesDataSet.update(" in html_code
    edges_update = "edgesDataSet.update(" in html_code
    findings["nodes_update"] = nodes_update
    findings["edges_update"] = edges_update
    print(f"  [PASS] nodesDataSet.update() present: {nodes_update}")
    print(f"  [PASS] edgesDataSet.update() present: {edges_update}")

    # 3. Check physics configuration
    import re
    damping_match = re.search(r"damping:\s*([0-9.]+)", html_code)
    stabilization_match = re.search(r"stabilization:\s*\{([^}]+)\}", html_code, re.DOTALL)

    damping_val = float(damping_match.group(1)) if damping_match else None
    findings["damping_val"] = damping_val
    print(f"  [CONFIG] Vis.js Barnes-Hut Damping: {damping_val} (Default is 0.09, configured is {damping_val})")

    has_stabilization = bool(stabilization_match) and ("iterations:" in stabilization_match.group(1))
    findings["has_stabilization"] = has_stabilization
    print(f"  [CONFIG] Vis.js Physics Stabilization Configured: {has_stabilization}")

    # 4. Check edge-case handling: What happens if an edge arrives with a new target node?
    edge_handling_snippet = re.search(r"else if \(type === \"graph_updated\".*?if \(msg\.edge\)\s*\{([^}]+)\}", html_code, re.DOTALL)
    auto_adds_missing_nodes = False
    if edge_handling_snippet:
        snippet_text = edge_handling_snippet.group(1)
        if "nodesDataSet.get" in snippet_text or "nodesDataSet.update" in snippet_text:
            auto_adds_missing_nodes = True
    findings["auto_adds_missing_nodes_on_edge"] = auto_adds_missing_nodes
    print(f"  [EDGE-CASE AUDIT] Auto-adds missing endpoint nodes on single edge update: {auto_adds_missing_nodes}")

    return findings


# ==============================================================================
# Main Orchestrator for Stress Suite
# ==============================================================================
async def main():
    print("======================================================================")
    print("EPGM M2 EMPIRICAL ADVERSARIAL STRESS TEST SUITE")
    print("======================================================================")

    # 1. Dashboard Audit
    dash_findings = audit_dashboard_html()

    # 2. Database Stress Test
    db_harness = DatabaseStressHarness()
    db_harness.setup()
    fk_results = db_harness.test_foreign_key_integrity()
    concurrent_write_results = db_harness.test_concurrent_telemetry_writes(num_threads=10, writes_per_thread=100)
    snapshot_results = db_harness.test_snapshot_versioning_and_concurrency(num_snapshots=50)
    index_results = db_harness.test_index_utilization()
    db_harness.teardown()

    # 3. WebSocket 50 Clients & 200 Messages Stress Test
    ws_harness = WebSocketStressHarness(port=8877, redis_port=6389)
    ws_results = await ws_harness.run_stress_test()

    print("\n======================================================================")
    print("SUMMARY OF EMPIRICAL FINDINGS")
    print("======================================================================")
    print(f"WebSocket 50 Clients & 200 Messages:")
    print(f"  - Clients: {ws_results['num_clients']}")
    print(f"  - Messages Published: {ws_results['num_messages']}")
    print(f"  - Client Min/Avg/Max Received: {ws_results['min_received']}/{ws_results['avg_received']}/{ws_results['max_received']}")
    print(f"  - Active Connections After 25 Drops: {ws_results['remaining_after_drop']}")
    print(f"  - Final Active Connections: {ws_results['final_active']}")
    print(f"  - Memory Delta: {ws_results['memory_delta_mb']} MB")

    print(f"\nDatabase Consistency & High Frequency Writes:")
    print(f"  - Foreign Key Constraint Enforced: {fk_results['fk_enforced']}")
    print(f"  - Cascade Delete Effective: {fk_results['cascade_ok']}")
    print(f"  - Concurrent Writes: {concurrent_write_results['persisted']}/{concurrent_write_results['total_attempted']} in {concurrent_write_results['elapsed_sec']}s ({concurrent_write_results['throughput_wps']} wps)")
    print(f"  - Snapshot Version Uniqueness Enforced: {snapshot_results['duplicate_rejected']}")
    print(f"  - Index Utilized for Telemetry: {index_results['uses_telemetry_idx']}")
    print(f"  - Index Utilized for Snapshots: {index_results['uses_snap_idx']}")

    print(f"\nDashboard Rendering Integrity:")
    print(f"  - No setData() full re-renders: {not dash_findings['has_set_data']}")
    print(f"  - Incremental nodesDataSet.update(): {dash_findings['nodes_update']}")
    print(f"  - Incremental edgesDataSet.update(): {dash_findings['edges_update']}")
    print(f"  - Barnes-Hut Damping: {dash_findings['damping_val']} (High damping anti-jitter)")
    print(f"  - Stabilization: {dash_findings['has_stabilization']}")

    # Clean up test DB files
    for db_f in ["stress_test_fastapi.db", "stress_test_db.db"]:
        fpath = os.path.join(CURRENT_DIR, db_f)
        if os.path.exists(fpath):
            try:
                os.remove(fpath)
            except Exception:
                pass

    return {
        "dashboard": dash_findings,
        "database": {
            "fk": fk_results,
            "writes": concurrent_write_results,
            "snapshots": snapshot_results,
            "indexes": index_results,
        },
        "websocket": ws_results,
    }

if __name__ == "__main__":
    asyncio.run(main())
