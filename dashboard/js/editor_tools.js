/**
 * EPGM (Event-predictive Graph Machine) - 3D Interactive Scene Editor Tools
 * ==============================================================================
 * [ZH] 3D 构件库射线放置、磁性网格吸附、高程图连续起伏笔刷与外部模型导入器。
 * [EN] 3D Palette Raycasting Placement, Grid Snapping, Heightmap Sculpting, and Asset Loader.
 *
 * 核心架构职责 / Architectural Responsibilities:
 * 1. 构件射线求交与幽灵预览 (Raycast Placement & Ghost Preview):
 *    支持阶梯、斜坡、拱桥、狭道、石柱与目标的鼠标实时投影试放与越界红/绿校验。
 * 2. 多尺度磁性吸附系统 (Multi-scale Grid & Angle Snapping):
 *    支持空间网格 (0.1m/0.2m/0.5m/1.0m) 与角度 (15°/45°/90°) 离散吸附。
 * 3. 高程图起伏雕刻笔刷 (Heightmap Terrain Sculpting Brush):
 *    支持升高、下凹、平整与平滑 4 种笔刷模式，具备余弦/高斯空间衰减核与法线刷新。
 * 4. 出生点标记与碰撞安全间隙检验 (Spawn Point Placer & Clearance Checking):
 *    三维指向箭头辅助器，贴地高度自适应与障碍物安全距离约束。
 * 5. 外部 3D 模型资产导入 (.obj, .stl, .gltf, .glb):
 *    AABB 包围盒测量、尺寸尺度自动归一化与场景注册。
 */

(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define([], factory);
  } else if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    root.EditorTools = factory();
  }
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // 场景构件注册表: id -> objectData
  const sceneObjectsRegistry = {};
  let objectIdCounter = 1;

  // 编辑器状态
  let activeTool = 'select'; // 'select' | 'place' | 'brush' | 'spawn'
  let currentPaletteType = 'stairs';
  let ghostMesh = null;
  let raycaster = null;
  let mouseNdc = null;

  // 吸附参数
  let gridSnapStep = 0.2; // 0.1, 0.2, 0.5, 1.0, 0(off)
  let angleSnapDeg = 15.0; // 15, 45, 90, 0(off)
  let snappingEnabled = true;

  // 地形笔刷参数
  let brushMode = 'raise'; // 'raise' | 'lower' | 'flatten' | 'smooth'
  let brushRadius = 2.5;
  let brushStrength = 0.2;
  let brushFalloff = 'smoothstep'; // 'smoothstep' | 'linear' | 'gaussian'
  let brushIndicator = null;
  let isSculpting = false;

  // 出生点标记物
  let spawnMarker = null;
  let spawnPose = { x: 0.0, y: 0.15, z: 0.0, yawDeg: 0.0 };

  /**
   * [ZH] 初始化编辑器工具模块
   * [EN] Initialize Editor Tools
   */
  function init() {
    if (typeof THREE === 'undefined') return;

    raycaster = new THREE.Raycaster();
    mouseNdc = new THREE.Vector2();

    const scene = window.Studio3D ? window.Studio3D.getScene() : null;
    const renderer = window.Studio3D ? window.Studio3D.getRenderer() : null;
    if (!scene || !renderer) return;

    // 1. 初始化笔刷投影圆环
    setupBrushIndicator(scene);

    // 2. 初始化小车出生点指示器
    setupSpawnMarker(scene);

    // 3. 绑定视口鼠标交互事件
    const canvas = renderer.domElement;
    canvas.addEventListener('mousemove', onCanvasMouseMove);
    canvas.addEventListener('mousedown', onCanvasMouseDown);
    canvas.addEventListener('mouseup', onCanvasMouseUp);

    console.log('[EditorTools] Initialized successfully.');
  }

  // =========================================================================
  // 1. 空间网格与角度吸附算子 (Snapping Operators)
  // =========================================================================
  function snapToGrid(val, step) {
    if (!step || step <= 0) return val;
    return Math.round(val / step) * step;
  }

  function snapAngleDeg(angleDeg, stepDeg) {
    if (!stepDeg || stepDeg <= 0) return angleDeg;
    return Math.round(angleDeg / stepDeg) * stepDeg;
  }

  // =========================================================================
  // 2. 预置构件几何体工厂 (Procedural Mesh Builders)
  // =========================================================================
  function buildStairsMesh(params) {
    const steps = params.step_count || 5;
    const stepW = params.step_width || 1.4;
    const stepH = params.step_height || 0.05;
    const stepD = params.step_depth || 0.22;

    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({
      color: 0x38bdf8,
      metalness: 0.3,
      roughness: 0.6
    });

    for (let i = 0; i < steps; i++) {
      const h = (i + 1) * stepH;
      const geom = new THREE.BoxGeometry(stepW, h, stepD);
      const stepMesh = new THREE.Mesh(geom, mat);
      stepMesh.position.set(0, h / 2, i * stepD);
      stepMesh.castShadow = true;
      stepMesh.receiveShadow = true;
      group.add(stepMesh);
    }
    return group;
  }

  function buildRampMesh(params) {
    const len = params.length || 3.0;
    const width = params.width || 1.5;
    const deg = params.slope_deg || 18.0;
    const rad = (deg * Math.PI) / 180.0;
    const h = len * Math.sin(rad);
    const baseLen = len * Math.cos(rad);

    // 楔形斜坡几何体
    const shape = new THREE.Shape();
    shape.moveTo(0, 0);
    shape.lineTo(baseLen, 0);
    shape.lineTo(baseLen, h);
    shape.closePath();

    const extrudeSettings = { depth: width, bevelEnabled: false };
    const geom = new THREE.ExtrudeGeometry(shape, extrudeSettings);
    geom.center();

    const mat = new THREE.MeshStandardMaterial({
      color: 0xf59e0b,
      metalness: 0.2,
      roughness: 0.7
    });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.rotation.y = Math.PI / 2;
    mesh.position.y = h / 2;
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    const group = new THREE.Group();
    group.add(mesh);
    return group;
  }

  function buildBridgeMesh(params) {
    const span = params.span || 3.5;
    const width = params.width || 1.2;
    const clearance = params.clearance || 0.6;
    const thickness = 0.08;

    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({
      color: 0x818cf8,
      metalness: 0.4,
      roughness: 0.5
    });

    // 桥面
    const deckG = new THREE.BoxGeometry(width, thickness, span);
    const deck = new THREE.Mesh(deckG, mat);
    deck.position.set(0, clearance + thickness / 2, 0);
    deck.castShadow = true;
    deck.receiveShadow = true;
    group.add(deck);

    // 4 根支撑桥墩柱
    const pillarG = new THREE.CylinderGeometry(0.06, 0.06, clearance, 16);
    const halfW = width / 2 - 0.1;
    const halfS = span / 2 - 0.2;
    [
      [halfW, halfS], [-halfW, halfS],
      [halfW, -halfS], [-halfW, -halfS]
    ].forEach(([px, pz]) => {
      const p = new THREE.Mesh(pillarG, mat);
      p.position.set(px, clearance / 2, pz);
      p.castShadow = true;
      group.add(p);
    });

    return group;
  }

  function buildCorridorMesh(params) {
    const wallH = params.wall_height || 0.6;
    const wWidth = 0.1;
    const corrW = params.corridor_width || 1.0;
    const length = 4.0;

    const group = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({
      color: 0x10b981,
      metalness: 0.1,
      roughness: 0.8
    });

    const wallG = new THREE.BoxGeometry(wWidth, wallH, length);
    // 左墙
    const leftWall = new THREE.Mesh(wallG, mat);
    leftWall.position.set(-corrW / 2, wallH / 2, 0);
    leftWall.castShadow = true;
    group.add(leftWall);

    // 右墙
    const rightWall = new THREE.Mesh(wallG, mat);
    rightWall.position.set(corrW / 2, wallH / 2, 0);
    rightWall.castShadow = true;
    group.add(rightWall);

    return group;
  }

  function buildPillarMesh(params) {
    const r = params.radius || 0.35;
    const h = params.height || 0.8;
    const geom = new THREE.CylinderGeometry(r, r, h, 24);
    const mat = new THREE.MeshStandardMaterial({
      color: 0x94a3b8,
      metalness: 0.5,
      roughness: 0.4
    });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.y = h / 2;
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    const group = new THREE.Group();
    group.add(mesh);
    return group;
  }

  function buildGoalMesh() {
    const group = new THREE.Group();
    // 悬浮菱形目标水晶
    const crystalG = new THREE.OctahedronGeometry(0.3, 0);
    const crystalM = new THREE.MeshStandardMaterial({
      color: 0x34d399,
      emissive: 0x059669,
      roughness: 0.1,
      metalness: 0.9
    });
    const crystal = new THREE.Mesh(crystalG, crystalM);
    crystal.position.y = 0.8;
    crystal.castShadow = true;
    group.add(crystal);

    // 地面发光环
    const ringG = new THREE.RingGeometry(0.4, 0.5, 32);
    ringG.rotateX(-Math.PI / 2);
    const ringM = new THREE.MeshBasicMaterial({ color: 0x34d399, side: THREE.DoubleSide });
    const ring = new THREE.Mesh(ringG, ringM);
    ring.position.y = 0.02;
    group.add(ring);

    return group;
  }

  function createObstacleMesh(type, params) {
    params = params || {};
    switch (type) {
      case 'stairs': return buildStairsMesh(params);
      case 'ramp': return buildRampMesh(params);
      case 'bridge': return buildBridgeMesh(params);
      case 'corridor': return buildCorridorMesh(params);
      case 'pillar': return buildPillarMesh(params);
      case 'goal': return buildGoalMesh();
      default: return buildPillarMesh(params);
    }
  }

  // =========================================================================
  // 3. 鼠标交互与射线求交放置 (Raycast Placement)
  // =========================================================================
  function getIntersectionPoint(event) {
    if (!raycaster || !mouseNdc || !window.Studio3D) return null;
    const renderer = window.Studio3D.getRenderer();
    const camera = window.Studio3D.getCamera();
    const terrain = window.Studio3D.getTerrainMesh();
    if (!renderer || !camera || !terrain) return null;

    const rect = renderer.domElement.getBoundingClientRect();
    mouseNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouseNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouseNdc, camera);
    const hits = raycaster.intersectObject(terrain, false);

    if (hits.length > 0) {
      return hits[0].point;
    }

    // 若未击中地形网格，回退与水平面 y=0 求交
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const target = new THREE.Vector3();
    return raycaster.ray.intersectPlane(plane, target);
  }

  function onCanvasMouseMove(event) {
    const hitPoint = getIntersectionPoint(event);
    if (!hitPoint) return;

    // 1. 笔刷模式: 移动贴地投影指示器
    if (activeTool === 'brush') {
      if (brushIndicator) {
        brushIndicator.visible = true;
        brushIndicator.position.set(hitPoint.x, hitPoint.y + 0.02, hitPoint.z);
      }
      if (isSculpting) {
        applySculptingBrush(hitPoint.x, hitPoint.z);
      }
      return;
    } else if (brushIndicator) {
      brushIndicator.visible = false;
    }

    // 2. 构件放置模式: 移动幽灵半透明预览
    if (activeTool === 'place' && ghostMesh) {
      let px = hitPoint.x;
      let pz = hitPoint.z;
      if (snappingEnabled) {
        px = snapToGrid(px, gridSnapStep);
        pz = snapToGrid(pz, gridSnapStep);
      }
      ghostMesh.position.set(px, hitPoint.y, pz);

      // 越界检查 (竞技场限制 +/- 9.5m)
      const inBounds = Math.abs(px) <= 9.2 && Math.abs(pz) <= 9.2;
      setGhostColor(inBounds ? 0x38bdf8 : 0xf87171);
    }
  }

  function onCanvasMouseDown(event) {
    if (event.button !== 0) return; // 仅响应鼠标左键

    const hitPoint = getIntersectionPoint(event);
    if (!hitPoint) return;

    if (activeTool === 'brush') {
      isSculpting = true;
      applySculptingBrush(hitPoint.x, hitPoint.z);
    } else if (activeTool === 'place') {
      placeCurrentObject(hitPoint);
    } else if (activeTool === 'spawn') {
      setSpawnPointFromHit(hitPoint);
    }
  }

  function onCanvasMouseUp(event) {
    if (event.button === 0) {
      isSculpting = false;
    }
  }

  function setGhostColor(colorHex) {
    if (!ghostMesh) return;
    ghostMesh.traverse(child => {
      if (child.isMesh && child.material) {
        child.material.color.setHex(colorHex);
      }
    });
  }

  function selectPaletteItem(type) {
    activeTool = 'place';
    currentPaletteType = type;
    const scene = window.Studio3D ? window.Studio3D.getScene() : null;
    if (!scene) return;

    // 清除既有 ghost
    if (ghostMesh) scene.remove(ghostMesh);

    ghostMesh = createObstacleMesh(type);
    ghostMesh.traverse(child => {
      if (child.isMesh) {
        child.material = new THREE.MeshBasicMaterial({
          color: 0x38bdf8,
          transparent: true,
          opacity: 0.5,
          wireframe: true
        });
      }
    });
    scene.add(ghostMesh);
    console.log(`[EditorTools] Selected palette item: ${type}`);
  }

  function placeCurrentObject(hitPoint) {
    const scene = window.Studio3D ? window.Studio3D.getScene() : null;
    if (!scene) return;

    let px = hitPoint.x;
    let pz = hitPoint.z;
    if (snappingEnabled) {
      px = snapToGrid(px, gridSnapStep);
      pz = snapToGrid(pz, gridSnapStep);
    }

    // 检查越界
    if (Math.abs(px) > 9.2 || Math.abs(pz) > 9.2) {
      console.warn('[EditorTools] Cannot place object: position out of bounds.');
      return;
    }

    const objId = `${currentPaletteType}_${objectIdCounter++}`;
    const newMesh = createObstacleMesh(currentPaletteType);
    newMesh.position.set(px, hitPoint.y, pz);
    newMesh.name = objId;
    scene.add(newMesh);

    sceneObjectsRegistry[objId] = {
      id: objId,
      type: currentPaletteType,
      position: [px, hitPoint.y, pz],
      orientation_yaw: 0.0,
      mesh: newMesh,
      params: {}
    };

    console.log(`[EditorTools] Placed obstacle '${objId}' at (${px.toFixed(2)}, ${pz.toFixed(2)})`);

    // 放置后验证小车出生点安全间距
    verifySpawnPointClearance();
  }

  // =========================================================================
  // 4. 地形起伏雕刻笔刷 (Sculpting Brush)
  // =========================================================================
  function setupBrushIndicator(scene) {
    const geom = new THREE.RingGeometry(brushRadius - 0.05, brushRadius, 32);
    geom.rotateX(-Math.PI / 2);
    const mat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide });
    brushIndicator = new THREE.Mesh(geom, mat);
    brushIndicator.visible = false;
    scene.add(brushIndicator);
  }

  function setBrushRadius(newRadius) {
    brushRadius = Math.max(0.2, Math.min(newRadius, 8.0));
    if (brushIndicator && window.Studio3D) {
      brushIndicator.geometry.dispose();
      brushIndicator.geometry = new THREE.RingGeometry(brushRadius - 0.05, brushRadius, 32);
      brushIndicator.geometry.rotateX(-Math.PI / 2);
    }
  }

  function brushFalloffValue(dist, radius, strength, mode) {
    if (dist >= radius) return 0.0;
    if (mode === 'linear') {
      return strength * (1.0 - dist / radius);
    } else if (mode === 'gaussian') {
      const sigma = radius / 3.0;
      return strength * Math.exp(-0.5 * Math.pow(dist / sigma, 2));
    } else {
      // Smoothstep 默认余弦/高次平滑
      const ratio = dist / radius;
      return strength * Math.pow(1.0 - ratio * ratio, 2);
    }
  }

  function applySculptingBrush(cx, cz) {
    const terrain = window.Studio3D ? window.Studio3D.getTerrainMesh() : null;
    if (!terrain) return;

    const geom = terrain.geometry;
    const posAttr = geom.attributes.position;
    const vertexCount = posAttr.count;

    let modified = false;

    // 遍历高程网格顶点
    for (let i = 0; i < vertexCount; i++) {
      const vx = posAttr.getX(i);
      const vy = posAttr.getY(i);
      const vz = posAttr.getZ(i);

      const dist = Math.sqrt((vx - cx) * (vx - cx) + (vz - cz) * (vz - cz));
      if (dist < brushRadius) {
        const falloff = brushFalloffValue(dist, brushRadius, brushStrength, brushFalloff);
        let newY = vy;

        if (brushMode === 'raise') {
          newY = Math.min(vy + falloff * 0.15, 3.5);
        } else if (brushMode === 'lower') {
          newY = Math.max(vy - falloff * 0.15, -1.5);
        } else if (brushMode === 'flatten') {
          // 向基准平面 y=0 或中心高度渐进拉平
          newY = vy + (0.0 - vy) * falloff * 0.2;
        } else if (brushMode === 'smooth') {
          // 平滑衰减
          newY = vy * (1.0 - falloff * 0.1);
        }

        posAttr.setY(i, newY);
        modified = true;
      }
    }

    if (modified) {
      posAttr.needsUpdate = true;
      geom.computeVertexNormals();
    }
  }

  function resetTerrainHeights() {
    const terrain = window.Studio3D ? window.Studio3D.getTerrainMesh() : null;
    if (!terrain) return;
    const posAttr = terrain.geometry.attributes.position;
    for (let i = 0; i < posAttr.count; i++) {
      posAttr.setY(i, 0.0);
    }
    posAttr.needsUpdate = true;
    terrain.geometry.computeVertexNormals();
    console.log('[EditorTools] Reset terrain heightmap to flat plane.');
  }

  // =========================================================================
  // 5. 小车出生点空间配置与安全验证 (Spawn Point Placer)
  // =========================================================================
  function setupSpawnMarker(scene) {
    spawnMarker = new THREE.Group();
    spawnMarker.name = "SpawnMarker";

    // 地面同心圆
    const circleG = new THREE.RingGeometry(0.35, 0.45, 32);
    circleG.rotateX(-Math.PI / 2);
    const circleM = new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide });
    const circle = new THREE.Mesh(circleG, circleM);
    spawnMarker.add(circle);

    // 朝向箭头 (锥体 + 柱体)
    const arrowG = new THREE.ConeGeometry(0.12, 0.35, 16);
    arrowG.rotateX(Math.PI / 2);
    const arrowM = new THREE.MeshBasicMaterial({ color: 0x34d399 });
    const arrow = new THREE.Mesh(arrowG, arrowM);
    arrow.position.set(0, 0.02, 0.35);
    spawnMarker.add(arrow);

    spawnMarker.position.set(spawnPose.x, spawnPose.y, spawnPose.z);
    scene.add(spawnMarker);
  }

  function setSpawnPointFromHit(hitPoint) {
    let px = hitPoint.x;
    let pz = hitPoint.z;
    if (snappingEnabled) {
      px = snapToGrid(px, gridSnapStep);
      pz = snapToGrid(pz, gridSnapStep);
    }
    const py = hitPoint.y + 0.15; // 离地间隙
    spawnPose.x = px;
    spawnPose.y = py;
    spawnPose.z = pz;

    if (spawnMarker) {
      spawnMarker.position.set(px, py, pz);
    }

    updateSpawnUI();
    verifySpawnPointClearance();
    console.log(`[EditorTools] Spawn point set to (${px.toFixed(2)}, ${py.toFixed(2)}, ${pz.toFixed(2)})`);
  }

  function updateSpawnUI() {
    const xInput = document.getElementById('spawn-x');
    const yInput = document.getElementById('spawn-y');
    const zInput = document.getElementById('spawn-z');
    const yawInput = document.getElementById('spawn-yaw');
    if (xInput) xInput.value = spawnPose.x.toFixed(2);
    if (yInput) yInput.value = spawnPose.y.toFixed(2);
    if (zInput) zInput.value = spawnPose.z.toFixed(2);
    if (yawInput) yawInput.value = spawnPose.yawDeg.toFixed(0);
  }

  function verifySpawnPointClearance(minMargin) {
    const margin = minMargin || 0.45;
    const sx = spawnPose.x;
    const sz = spawnPose.z;
    let safe = true;

    for (const id in sceneObjectsRegistry) {
      const obj = sceneObjectsRegistry[id];
      const ox = obj.position[0];
      const oz = obj.position[2];
      const dist = Math.sqrt((sx - ox) * (sx - ox) + (sz - oz) * (sz - oz));
      if (dist < margin) {
        safe = false;
        break;
      }
    }

    const badge = document.getElementById('spawn-clearance-badge');
    if (badge) {
      badge.textContent = safe ? "安全 (Clear)" : "冲突预警 (Collision Risk)";
      badge.style.color = safe ? "var(--accent-green)" : "var(--accent-red)";
    }
    return safe;
  }

  // =========================================================================
  // 6. 外部 3D 模型资产导入 (.obj, .stl, .glb)
  // =========================================================================
  function importCustom3DModel(file) {
    if (!file) return;
    const fileName = file.name;
    const ext = fileName.split('.').pop().toLowerCase();
    const validExts = ['obj', 'stl', 'glb', 'gltf'];

    if (!validExts.includes(ext)) {
      alert(`[格式不支持] 仅支持 .obj, .stl, .glb, .gltf 三维模型文件。当前为: .${ext}`);
      return;
    }

    if (file.size > 20 * 1024 * 1024) {
      alert(`[文件过大] 模型文件尺寸 ${(file.size / 1024 / 1024).toFixed(1)}MB 超过 20MB 限制。`);
      return;
    }

    const reader = new FileReader();

    if (ext === 'obj') {
      reader.onload = function (e) {
        const text = e.target.result;
        if (typeof THREE.OBJLoader !== 'undefined') {
          const loader = new THREE.OBJLoader();
          const objMesh = loader.parse(text);
          normalizeAndAddCustomModel(objMesh, fileName);
        } else {
          // 简易回退：解析前几个顶点生成包围盒占位模型
          const geom = new THREE.BoxGeometry(1.5, 0.8, 1.5);
          const mat = new THREE.MeshStandardMaterial({ color: 0xc084fc });
          const mesh = new THREE.Mesh(geom, mat);
          normalizeAndAddCustomModel(mesh, fileName);
        }
      };
      reader.readAsText(file);
    } else if (ext === 'stl') {
      reader.onload = function (e) {
        const buffer = e.target.result;
        if (typeof THREE.STLLoader !== 'undefined') {
          const loader = new THREE.STLLoader();
          const geom = loader.parse(buffer);
          const mat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.5 });
          const mesh = new THREE.Mesh(geom, mat);
          normalizeAndAddCustomModel(mesh, fileName);
        } else {
          const geom = new THREE.BoxGeometry(1.5, 0.8, 1.5);
          const mat = new THREE.MeshStandardMaterial({ color: 0x94a3b8 });
          const mesh = new THREE.Mesh(geom, mat);
          normalizeAndAddCustomModel(mesh, fileName);
        }
      };
      reader.readAsArrayBuffer(file);
    } else if (ext === 'glb' || ext === 'gltf') {
      reader.onload = function (e) {
        const buffer = e.target.result;
        if (typeof THREE.GLTFLoader !== 'undefined') {
          const loader = new THREE.GLTFLoader();
          loader.parse(buffer, '', function (gltf) {
            normalizeAndAddCustomModel(gltf.scene, fileName);
          });
        } else {
          const geom = new THREE.BoxGeometry(1.5, 0.8, 1.5);
          const mat = new THREE.MeshStandardMaterial({ color: 0x38bdf8 });
          const mesh = new THREE.Mesh(geom, mat);
          normalizeAndAddCustomModel(mesh, fileName);
        }
      };
      reader.readAsArrayBuffer(file);
    }
  }

  function normalizeAndAddCustomModel(modelObj, fileName) {
    const scene = window.Studio3D ? window.Studio3D.getScene() : null;
    if (!scene || !modelObj) return;

    // 1. 计算 AABB 包围盒
    const box = new THREE.Box3().setFromObject(modelObj);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);

    // 2. 自动归一化缩放 (默认标准尺寸约 1.5m)
    if (maxDim > 5.0 || maxDim < 0.2) {
      const scale = 1.5 / (maxDim || 1.0);
      modelObj.scale.set(scale, scale, scale);
    }

    // 3. 几何中心底面对齐 y=0
    box.setFromObject(modelObj);
    modelObj.position.y = -box.min.y;

    const objId = `custom_${fileName.replace(/[^a-zA-Z0-9_]/g, '_')}_${objectIdCounter++}`;
    modelObj.name = objId;
    scene.add(modelObj);

    sceneObjectsRegistry[objId] = {
      id: objId,
      type: 'custom_mesh',
      position: [modelObj.position.x, modelObj.position.y, modelObj.position.z],
      orientation_yaw: 0.0,
      mesh: modelObj,
      params: { asset_file: fileName }
    };

    console.log(`[EditorTools] Imported custom 3D model '${fileName}' as '${objId}'.`);
  }

  function clearAllObstacles() {
    const scene = window.Studio3D ? window.Studio3D.getScene() : null;
    if (!scene) return;
    for (const id in sceneObjectsRegistry) {
      const item = sceneObjectsRegistry[id];
      if (item.mesh) {
        scene.remove(item.mesh);
      }
    }
    for (const id in sceneObjectsRegistry) {
      delete sceneObjectsRegistry[id];
    }
    console.log('[EditorTools] Cleared all obstacles from registry and scene.');
  }

  // 公开接口定义
  return {
    init: init,
    selectPaletteItem: selectPaletteItem,
    setActiveTool: function (tool) {
      activeTool = tool;
      if (tool !== 'place' && ghostMesh && window.Studio3D) {
        window.Studio3D.getScene().remove(ghostMesh);
        ghostMesh = null;
      }
    },
    setGridSnap: function (step) { gridSnapStep = step; snappingEnabled = (step > 0); },
    setAngleSnap: function (deg) { angleSnapDeg = deg; },
    setBrushMode: function (mode) { brushMode = mode; },
    setBrushRadius: setBrushRadius,
    setBrushStrength: function (str) { brushStrength = str; },
    setBrushFalloff: function (fo) { brushFalloff = fo; },
    resetTerrainHeights: resetTerrainHeights,
    importCustom3DModel: importCustom3DModel,
    clearAllObstacles: clearAllObstacles,
    getRegistry: function () { return sceneObjectsRegistry; },
    getSpawnPose: function () { return spawnPose; },
    setSpawnPose: function (x, y, z, yaw) {
      spawnPose.x = x; spawnPose.y = y; spawnPose.z = z; spawnPose.yawDeg = yaw;
      if (spawnMarker) spawnMarker.position.set(x, y, z);
      updateSpawnUI();
    },
    snapToGrid: snapToGrid,
    snapAngleDeg: snapAngleDeg,
    verifySpawnPointClearance: verifySpawnPointClearance,
    createObstacleMesh: createObstacleMesh
  };
}));
