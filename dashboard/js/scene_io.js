/**
 * EPGM (Event-predictive Graph Machine) - 3D Scene Serialization Protocol (scene.json)
 * ==============================================================================
 * [ZH] Web 3D 工作台场景导入、导出与预设竞技场重建引擎。
 * [EN] Web 3D Scene Import, Export, and Preset Reconstruction Engine (scene.json).
 *
 * 核心架构职责 / Architectural Responsibilities:
 * 1. 场景标准格式序列化 (Standard Scene Serialization):
 *    提取高程图阵列、构件参数列表、世界动力学配置与小车出生点并导出为 scene.json。
 * 2. 浏览器一键导出下载与云端同步 (Local Download & Cloud Gateway Sync):
 *    通过 Blob 自动触发客户端本地下载，并异步同步至云端 `/api/v1/scenes/save`。
 * 3. 场景无损反序列化重建 (Full In-Memory Scene Reconstruction):
 *    校验 schema 必填项，彻底重塑地形高程、重建障碍物网格与重设出生点。
 * 4. 内置 4 套经典科学基准场景预设 (4 Built-in Benchmark Scene Presets):
 *    - 平坦竞技场 (Flat Benchmark Arena)
 *    - 极速攻坡与台阶挑战 (Stairs & Ramp Challenge)
 *    - 复杂立交拱桥迷宫 (Overpass & Maze Arena)
 *    - 极端起伏高程越野 (Rugged Heightmap Wilderness)
 */

(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define([], factory);
  } else if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    root.SceneIO = factory();
  }
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // 4 套内置预设竞技场字典
  const PRESET_SCENES = {
    flat: {
      version: "1.0.0",
      metadata: { name: "平坦基础竞技场 (Flat Benchmark Arena)", author: "EPGM Studio", created_at: new Date().toISOString() },
      world: { gravity: [0.0, 0.0, -9.81], timestep: 0.005, substeps: 4, arena_size: [10.0, 10.0, 3.0] },
      terrain: { type: "heightmap", resolution: [16, 16], horizontal_scale: [0.2, 0.2], vertical_scale: 0.5, data: Array(16).fill(null).map(() => Array(16).fill(0.0)) },
      obstacles: [
        { id: "pillar_center", type: "pillar", position: [0.0, 2.5, 0.0], orientation_yaw: 0.0, params: { radius: 0.4, height: 1.0 } },
        { id: "goal_01", type: "goal", position: [0.0, 4.0, 0.0], orientation_yaw: 0.0, params: {} }
      ],
      custom_assets: [],
      robot_spawns: [{ sim_id: "sim_bot_01", config_file: "robot_config.json", position: [0.0, 0.0, 0.1], orientation_yaw: 0.0 }]
    },
    stairs_ramp: {
      version: "1.0.0",
      metadata: { name: "极速攻坡与台阶挑战 (Stairs & Ramp Challenge)", author: "EPGM Studio", created_at: new Date().toISOString() },
      world: { gravity: [0.0, 0.0, -9.81], timestep: 0.005, substeps: 4, arena_size: [10.0, 10.0, 3.0] },
      terrain: { type: "heightmap", resolution: [16, 16], horizontal_scale: [0.2, 0.2], vertical_scale: 0.5, data: Array(16).fill(null).map(() => Array(16).fill(0.0)) },
      obstacles: [
        { id: "stairs_01", type: "stairs", position: [2.5, 1.0, 0.0], orientation_yaw: 0.0, params: { step_count: 5, step_width: 1.4, step_height: 0.05, step_depth: 0.22 } },
        { id: "ramp_01", type: "ramp", position: [-2.5, 1.0, 0.0], orientation_yaw: 1.57, params: { length: 3.0, width: 1.5, slope_deg: 18.0, friction: 0.8 } },
        { id: "goal_01", type: "goal", position: [2.5, 2.5, 0.25], orientation_yaw: 0.0, params: {} }
      ],
      custom_assets: [],
      robot_spawns: [{ sim_id: "sim_bot_01", config_file: "robot_config.json", position: [0.0, -2.0, 0.1], orientation_yaw: 0.0 }]
    },
    overpass_maze: {
      version: "1.0.0",
      metadata: { name: "复杂立交拱桥迷宫 (Overpass & Maze Arena)", author: "EPGM Studio", created_at: new Date().toISOString() },
      world: { gravity: [0.0, 0.0, -9.81], timestep: 0.005, substeps: 4, arena_size: [10.0, 10.0, 3.0] },
      terrain: { type: "heightmap", resolution: [16, 16], horizontal_scale: [0.2, 0.2], vertical_scale: 0.5, data: Array(16).fill(null).map(() => Array(16).fill(0.0)) },
      obstacles: [
        { id: "bridge_01", type: "bridge", position: [0.0, 1.5, 0.0], orientation_yaw: 0.0, params: { span: 4.0, width: 1.2, clearance: 0.65, ramp_slope_deg: 15.0 } },
        { id: "corridor_01", type: "corridor", position: [-3.0, -1.5, 0.0], orientation_yaw: 0.0, params: { wall_height: 0.6, corridor_width: 1.0 } },
        { id: "pillar_left", type: "pillar", position: [3.0, -1.0, 0.0], orientation_yaw: 0.0, params: { radius: 0.35, height: 0.8 } }
      ],
      custom_assets: [],
      robot_spawns: [{ sim_id: "sim_bot_01", config_file: "robot_config.json", position: [0.0, -3.0, 0.1], orientation_yaw: 0.0 }]
    },
    rugged_heightmap: {
      version: "1.0.0",
      metadata: { name: "极端起伏高程越野 (Rugged Heightmap Wilderness)", author: "EPGM Studio", created_at: new Date().toISOString() },
      world: { gravity: [0.0, 0.0, -9.81], timestep: 0.005, substeps: 4, arena_size: [10.0, 10.0, 3.0] },
      terrain: {
        type: "heightmap",
        resolution: [16, 16],
        horizontal_scale: [0.2, 0.2],
        vertical_scale: 0.5,
        data: (function () {
          const matrix = [];
          for (let r = 0; r < 16; r++) {
            const row = [];
            for (let c = 0; c < 16; c++) {
              const dist = Math.sqrt((r - 8) ** 2 + (c - 8) ** 2);
              row.push(parseFloat((Math.sin(dist * 0.8) * 0.35).toFixed(3)));
            }
            matrix.push(row);
          }
          return matrix;
        })()
      },
      obstacles: [
        { id: "rock_pillar", type: "pillar", position: [2.0, 2.0, 0.0], orientation_yaw: 0.0, params: { radius: 0.4, height: 0.9 } },
        { id: "goal_01", type: "goal", position: [0.0, 0.0, 0.3], orientation_yaw: 0.0, params: {} }
      ],
      custom_assets: [],
      robot_spawns: [{ sim_id: "sim_bot_01", config_file: "robot_config.json", position: [-3.0, -3.0, 0.1], orientation_yaw: 0.0 }]
    }
  };

  /**
   * [ZH] 导出当前工作区场景状态为 scene.json 字符串
   * [EN] Export current workspace scene state to formatted JSON string
   */
  function exportSceneToJson() {
    const obstaclesList = [];
    if (window.EditorTools) {
      const registry = window.EditorTools.getRegistry();
      for (const id in registry) {
        const item = registry[id];
        // [ZH] Three.js (Y-Up: x_t, y_t, z_t) 逆变换为 PyBullet (Z-Up: x_b, y_b, z_b) 空间坐标:
        //      x_bullet = x_three
        //      y_bullet = -z_three
        //      z_bullet = y_three
        // [EN] Convert Three.js (Y-Up) back to PyBullet (Z-Up) basis:
        //      x_b = x_t, y_b = -z_t, z_b = y_t
        const posX = item.mesh ? item.mesh.position.x : item.position[0];
        const posY = item.mesh ? item.mesh.position.y : item.position[1];
        const posZ = item.mesh ? item.mesh.position.z : item.position[2];
        const yaw = item.mesh ? item.mesh.rotation.y : (item.orientation_yaw || 0.0);

        obstaclesList.push({
          id: item.id,
          type: item.type,
          position: [posX, -posZ, posY],
          orientation_yaw: yaw,
          params: item.params || {}
        });
      }
    }

    // 采样高程图顶点高度 (构建 16x16 矩阵)
    const heightMatrix = sampleHeightmapMatrix(16, 16);

    const spawnPose = window.EditorTools ? window.EditorTools.getSpawnPose() : { x: 0, y: 0.15, z: 0, yawDeg: 0 };

    const sceneDict = {
      version: "1.0.0",
      metadata: {
        name: "Arena3D_CustomExport",
        author: "EPGM Studio Visualizer",
        created_at: new Date().toISOString()
      },
      world: {
        gravity: [0.0, 0.0, -9.81],
        timestep: 0.005,
        substeps: 4,
        arena_size: [10.0, 10.0, 3.0]
      },
      terrain: {
        type: "heightmap",
        resolution: [16, 16],
        horizontal_scale: [0.2, 0.2],
        vertical_scale: 0.5,
        data: heightMatrix
      },
      obstacles: obstaclesList,
      custom_assets: [],
      robot_spawns: [
        {
          sim_id: "sim_bot_01",
          config_file: "robot_config.json",
          position: [spawnPose.x, -spawnPose.z, spawnPose.y],
          orientation_yaw: (spawnPose.yawDeg * Math.PI) / 180.0
        }
      ]
    };

    return JSON.stringify(sceneDict, null, 2);
  }

  /**
   * [ZH] 采样高程图为二维矩阵
   * [EN] Sample terrain mesh heights into 2D matrix
   */
  function sampleHeightmapMatrix(rows, cols) {
    const matrix = [];
    const terrain = window.Studio3D ? window.Studio3D.getTerrainMesh() : null;

    if (!terrain) {
      for (let r = 0; r < rows; r++) {
        matrix.push(new Array(cols).fill(0.0));
      }
      return matrix;
    }

    const posAttr = terrain.geometry.attributes.position;
    const vertexCount = posAttr.count;
    const step = Math.max(1, Math.floor(vertexCount / (rows * cols)));

    for (let r = 0; r < rows; r++) {
      const row = [];
      for (let c = 0; c < cols; c++) {
        const idx = Math.min((r * cols + c) * step, vertexCount - 1);
        row.push(parseFloat(posAttr.getY(idx).toFixed(3)));
      }
      matrix.push(row);
    }
    return matrix;
  }

  /**
   * [ZH] 触发浏览器本地下载 scene.json 文件并推送到云端存盘
   * [EN] Trigger browser file download and sync to cloud
   */
  function downloadSceneJson(filename) {
    const jsonStr = exportSceneToJson();
    const name = filename || "scene.json";

    const blob = new Blob([jsonStr], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    console.log(`[SceneIO] Downloaded scene to '${name}'.`);

    // 异步同步到后端
    try {
      fetch("/api/v1/scenes/save", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: jsonStr
      }).catch(() => {});
    } catch (_) {}
  }

  /**
   * [ZH] 从 JSON 字符串解析反序列化场景并重塑 3D 世界
   * [EN] Import and reconstruct 3D scene from JSON string
   */
  function importSceneFromJson(jsonStr) {
    let data;
    try {
      data = JSON.parse(jsonStr);
    } catch (err) {
      alert(`[JSON解析失败] scene.json 文件内容不符合合法 JSON 语法：${err.message}`);
      return false;
    }

    // 必填契约校验
    const required = ["version", "world", "terrain", "obstacles"];
    for (let i = 0; i < required.length; i++) {
      const field = required[i];
      if (!data[field]) {
        alert(`[协议错误] 场景描述文件缺少必需字段: '${field}'`);
        return false;
      }
    }

    // 1. 清理当前既有全部障碍物
    if (window.EditorTools) {
      window.EditorTools.clearAllObstacles();
    }

    // 2. 重置并根据高程图矩阵重塑地形
    applyHeightmapData(data.terrain);

    // 3. 批量生成障碍物
    const scene = window.Studio3D ? window.Studio3D.getScene() : null;
    if (scene && window.EditorTools && Array.isArray(data.obstacles)) {
      data.obstacles.forEach(obs => {
        const mesh = window.EditorTools.createObstacleMesh(obs.type, obs.params);
        if (mesh) {
          const pos = obs.position || [0, 0, 0];
          // [ZH] PyBullet (Z-Up: x_b, y_b, z_b) 映射至 Three.js (Y-Up: x_t = x_b, y_t = z_b, z_t = -y_b)
          // [EN] Map PyBullet (Z-Up) to Three.js (Y-Up: x_t = x_b, y_t = z_b, z_t = -y_b)
          const px = pos[0];
          const py = pos[2];
          const pz = -pos[1];
          mesh.position.set(px, py, pz);
          if (obs.orientation_yaw) {
            mesh.rotation.y = obs.orientation_yaw;
          }
          mesh.name = obs.id;
          scene.add(mesh);

          const registry = window.EditorTools.getRegistry();
          registry[obs.id] = {
            id: obs.id,
            type: obs.type,
            position: [px, py, pz],
            orientation_yaw: obs.orientation_yaw || 0.0,
            mesh: mesh,
            params: obs.params || {}
          };
        }
      });
    }

    // 4. 重置出生点位姿
    if (data.robot_spawns && data.robot_spawns.length > 0 && window.EditorTools) {
      const sp = data.robot_spawns[0];
      const pos = sp.position || [0, 0, 0.1];
      const yawDeg = (sp.orientation_yaw || 0.0) * (180.0 / Math.PI);
      // [ZH] 出生点基底变换: x_t = x_b, y_t = z_b, z_t = -y_b
      // [EN] Spawn pose basis transformation: x_t = x_b, y_t = z_b, z_t = -y_b
      window.EditorTools.setSpawnPose(pos[0], pos[2], -pos[1], yawDeg);
    }

    console.log(`[SceneIO] Successfully reconstructed scene: ${data.metadata ? data.metadata.name : 'Custom Scene'}`);
    return true;
  }

  function applyHeightmapData(terrainConfig) {
    const terrain = window.Studio3D ? window.Studio3D.getTerrainMesh() : null;
    if (!terrain || !terrainConfig || !terrainConfig.data) return;

    const data = terrainConfig.data;
    const rows = data.length;
    const cols = data[0] ? data[0].length : 0;
    if (rows === 0 || cols === 0) return;

    const posAttr = terrain.geometry.attributes.position;
    const count = posAttr.count;

    for (let i = 0; i < count; i++) {
      const r = Math.floor((i / count) * rows);
      const c = i % cols;
      const h = (data[r] && data[r][c] !== undefined) ? data[r][c] : 0.0;
      posAttr.setY(i, h);
    }
    posAttr.needsUpdate = true;
    terrain.geometry.computeVertexNormals();
  }

  /**
   * [ZH] 加载内置经典预设场景
   * [EN] Load built-in preset scene
   */
  function loadPresetScene(presetKey) {
    const preset = PRESET_SCENES[presetKey];
    if (!preset) {
      console.warn(`[SceneIO] Unknown preset '${presetKey}'`);
      return false;
    }
    const jsonStr = JSON.stringify(preset);
    return importSceneFromJson(jsonStr);
  }

  // 公开接口定义
  return {
    exportSceneToJson: exportSceneToJson,
    downloadSceneJson: downloadSceneJson,
    importSceneFromJson: importSceneFromJson,
    loadPresetScene: loadPresetScene,
    getPresets: function () { return PRESET_SCENES; }
  };
}));
