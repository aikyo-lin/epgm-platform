/**
 * EPGM (Event-predictive Graph Machine) - 3D WebGL Studio & Rendering Pipeline
 * ==============================================================================
 * [ZH] Three.js 三维动力学仿真视口渲染管线与视口控制器。
 * [EN] Three.js 3D Dynamics Simulation Viewport Rendering Pipeline and Controls.
 *
 * 核心架构职责 / Architectural Responsibilities:
 * 1. 独立 60 FPS 渲染管线 (Decoupled 60 FPS Render Loop):
 *    与 WebSocket (20~50Hz) 网络推流严格解耦，维持高帧率画面流畅度与阻尼物理平滑。
 * 2. 真实感光影与空间雾效 (Cinematic Cyber Lighting & Fog):
 *    集成 ACESFilmic 级色调映射、PCFSoftShadowMap 软阴影正交视锥与双色半球光。
 * 3. 多视角自由调度 (Multi-Mode Camera Director):
 *    支持自由轨道漫游 (Orbit)、小车第三人称跟随 (Chase Cam) 与正交俯视 (Top-Down)。
 * 4. 视口自适应重绘 (ResizeObserver Integration):
 *    响应式监听 DOM 容器尺寸，动态重算投影宽高比与视口分辨率。
 */

(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define([], factory);
  } else if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    root.Studio3D = factory();
  }
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // 视口与渲染器内部状态
  let container = null;
  let scene = null;
  let camera = null;
  let renderer = null;
  let controls = null;
  let dirLight = null;
  let ambientLight = null;
  let terrainMesh = null;
  let arenaGrid = null;
  let boundaryWalls = null;

  // 摄像机控制模式: 'orbit' | 'follow' | 'top'
  let cameraMode = 'orbit';
  let targetToFollow = null;

  // 相机飞行动画状态 (Fly-to interpolation)
  let isFlying = false;
  let flyStartTime = 0;
  let flyDuration = 1000;
  let flyStartPos = null;
  let flyEndPos = null;
  let flyStartLook = null;
  let flyEndLook = null;

  // 帧率统计器
  let lastFrameTime = performance.now();
  let frameCount = 0;
  let fpsTimer = performance.now();
  let currentFps = 60;

  // 竞技场几何尺寸
  const ARENA_WIDTH = 20.0;
  const ARENA_LENGTH = 20.0;
  const TERRAIN_SEGMENTS = 64;

  /**
   * [ZH] 初始化 Three.js 视口与 WebGL 渲染管线
   * [EN] Initialize Three.js viewport and WebGL rendering pipeline
   */
  function init(containerElementId) {
    container = document.getElementById(containerElementId || 'viewport-3d');
    if (!container) {
      console.warn(`[Studio3D] Container element '#${containerElementId}' not found.`);
      return false;
    }

    if (typeof THREE === 'undefined') {
      console.warn('[Studio3D] Three.js core library is not loaded.');
      return false;
    }

    // 1. 创建三维场景与环境雾
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x090d16); // 深度暗黑科幻背景色
    scene.fog = new THREE.FogExp2(0x090d16, 0.018);

    // 2. 创建透视投影相机
    const aspect = (container.clientWidth || 800) / (container.clientHeight || 460);
    camera = new THREE.PerspectiveCamera(55, aspect, 0.1, 1000);
    camera.position.set(0, 10, 14);
    camera.lookAt(0, 0, 0);

    // 3. 配置 WebGL 渲染器 (高动态软阴影 + 抗锯齿)
    renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: 'high-performance',
      alpha: false
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(container.clientWidth || 800, container.clientHeight || 460);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.1;

    // 清空容器并插入 canvas
    const oldCanvas = container.querySelector('canvas.three-canvas');
    if (oldCanvas) oldCanvas.remove();
    renderer.domElement.className = 'three-canvas';
    renderer.domElement.style.width = '100%';
    renderer.domElement.style.height = '100%';
    renderer.domElement.style.display = 'block';
    container.appendChild(renderer.domElement);

    // 4. 配置轨道漫游控制器 (OrbitControls)
    if (typeof THREE.OrbitControls !== 'undefined') {
      controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.05;
      controls.maxPolarAngle = Math.PI / 2 - 0.04; // 锁定在地平线以上，严禁钻入地底
      controls.minDistance = 2.0;
      controls.maxDistance = 50.0;
      controls.target.set(0, 0, 0);
    }

    // 5. 构筑光影系统
    setupLighting();

    // 6. 构筑竞技场底座与高程图地形
    setupArenaEnvironment();

    // 7. 启动自适应尺寸监听
    setupResizeObserver();

    // 8. 启动独立 60 FPS 渲染循环
    requestAnimationFrame(animate);

    console.log('[Studio3D] Viewport initialized successfully at 60 FPS target.');
    return true;
  }

  /**
   * [ZH] 配置高动态物理光影系统
   * [EN] Setup cinematic lighting system
   */
  function setupLighting() {
    // 环境微光 (深青色填充)
    ambientLight = new THREE.AmbientLight(0x38bdf8, 0.35);
    scene.add(ambientLight);

    // 天空与地面双色半球光
    const hemiLight = new THREE.HemisphereLight(0x818cf8, 0x090d16, 0.4);
    hemiLight.position.set(0, 30, 0);
    scene.add(hemiLight);

    // 主投射日光 (阴影光源)
    dirLight = new THREE.DirectionalLight(0xffffff, 0.85);
    dirLight.position.set(12, 24, 12);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.5;
    dirLight.shadow.camera.far = 70.0;
    const d = 16;
    dirLight.shadow.camera.left = -d;
    dirLight.shadow.camera.right = d;
    dirLight.shadow.camera.top = d;
    dirLight.shadow.camera.bottom = -d;
    dirLight.shadow.bias = -0.0005;
    scene.add(dirLight);

    // 目标瞄准原点
    const target = new THREE.Object3D();
    target.position.set(0, 0, 0);
    scene.add(target);
    dirLight.target = target;
  }

  /**
   * [ZH] 构建 3D 物理竞技场地标、边界墙与基础高程图地形网格
   * [EN] Setup arena ground, boundary walls, and heightmap plane geometry
   */
  function setupArenaEnvironment() {
    // 1. 竞技场空间地面网格标线
    arenaGrid = new THREE.GridHelper(ARENA_WIDTH, 40, 0x38bdf8, 0x1e293b);
    arenaGrid.position.y = 0.002;
    scene.add(arenaGrid);

    // 2. 地形网格平面 (沿 X-Z 平面水平展开)
    const geom = new THREE.PlaneGeometry(ARENA_WIDTH, ARENA_LENGTH, TERRAIN_SEGMENTS, TERRAIN_SEGMENTS);
    geom.rotateX(-Math.PI / 2); // 旋转为水平面 (Y朝上)

    // 创建赛博暗色科技材质
    const mat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.85,
      metalness: 0.25,
      wireframe: false,
      flatShading: true
    });

    terrainMesh = new THREE.Mesh(geom, mat);
    terrainMesh.receiveShadow = true;
    terrainMesh.name = "TerrainMesh";
    scene.add(terrainMesh);

    // 3. 竞技场外围发光边界防护墙
    setupBoundaryWalls();
  }

  /**
   * [ZH] 构建竞技场外围透明边界防坠落墙
   * [EN] Build boundary safety walls around arena
   */
  function setupBoundaryWalls() {
    boundaryWalls = new THREE.Group();
    boundaryWalls.name = "BoundaryWalls";

    const wallHeight = 0.6;
    const wallThickness = 0.1;
    const halfW = ARENA_WIDTH / 2;
    const halfL = ARENA_LENGTH / 2;

    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.25,
      roughness: 0.3,
      metalness: 0.8
    });

    // 4 面边界外墙
    const configs = [
      { size: [ARENA_WIDTH, wallHeight, wallThickness], pos: [0, wallHeight / 2, -halfL] },
      { size: [ARENA_WIDTH, wallHeight, wallThickness], pos: [0, wallHeight / 2, halfL] },
      { size: [wallThickness, wallHeight, ARENA_LENGTH], pos: [-halfW, wallHeight / 2, 0] },
      { size: [wallThickness, wallHeight, ARENA_LENGTH], pos: [halfW, wallHeight / 2, 0] }
    ];

    configs.forEach(cfg => {
      const g = new THREE.BoxGeometry(cfg.size[0], cfg.size[1], cfg.size[2]);
      const m = new THREE.Mesh(g, wallMat);
      m.position.set(cfg.pos[0], cfg.pos[1], cfg.pos[2]);
      boundaryWalls.add(m);
    });

    scene.add(boundaryWalls);
  }

  /**
   * [ZH] 监听视口尺寸调整并自适应投影矩阵
   * [EN] Monitor viewport container resizing
   */
  function setupResizeObserver() {
    if (!window.ResizeObserver || !container) return;
    const observer = new ResizeObserver(() => {
      onWindowResize();
    });
    observer.observe(container);
  }

  function onWindowResize() {
    if (!container || !camera || !renderer) return;
    const width = container.clientWidth || 800;
    const height = container.clientHeight || 460;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  }

  /**
   * [ZH] 摄像机视线飞行动画插值
   * [EN] Smooth camera fly-to interpolation
   */
  function flyTo(targetPosition, targetLookAt, durationMs) {
    if (!camera || !controls) return;
    isFlying = true;
    flyStartTime = performance.now();
    flyDuration = durationMs || 1000;

    flyStartPos = camera.position.clone();
    flyEndPos = new THREE.Vector3(targetPosition[0], targetPosition[1], targetPosition[2]);

    flyStartLook = controls.target.clone();
    flyEndLook = new THREE.Vector3(targetLookAt[0], targetLookAt[1], targetLookAt[2]);
  }

  /**
   * [ZH] 切换摄像机工作模式
   * [EN] Switch camera mode ('orbit' | 'follow' | 'top')
   */
  function setCameraMode(mode, targetMesh) {
    cameraMode = mode;
    targetToFollow = targetMesh || targetToFollow;

    if (mode === 'top') {
      flyTo([0, 24, 0.01], [0, 0, 0], 800);
    } else if (mode === 'orbit') {
      if (controls) controls.enabled = true;
    }
  }

  /**
   * [ZH] 独立 60 FPS 主渲染循环
   * [EN] Standalone 60 FPS Main Render Loop
   */
  function animate(timestamp) {
    requestAnimationFrame(animate);

    const now = performance.now();
    const dt = (now - lastFrameTime) / 1000;
    lastFrameTime = now;

    // 1. 帧率统计更新 (每秒结算)
    frameCount++;
    if (now - fpsTimer >= 1000) {
      currentFps = Math.round((frameCount * 1000) / (now - fpsTimer));
      frameCount = 0;
      fpsTimer = now;
      const fpsElem = document.getElementById('val-fps');
      if (fpsElem) fpsElem.textContent = currentFps;
    }

    // 2. 摄像机飞行插值动画
    if (isFlying) {
      const elapsed = now - flyStartTime;
      const progress = Math.min(elapsed / flyDuration, 1.0);
      // Smoothstep 缓动曲线
      const t = progress * progress * (3 - 2 * progress);

      camera.position.lerpVectors(flyStartPos, flyEndPos, t);
      controls.target.lerpVectors(flyStartLook, flyEndLook, t);

      if (progress >= 1.0) {
        isFlying = false;
      }
    } else if (cameraMode === 'follow' && targetToFollow) {
      // 车身跟随第三人称视角: 挂靠在小车后上方
      const robotPos = targetToFollow.position;
      const robotYaw = targetToFollow.rotation.y;
      const dist = 4.0;
      const height = 2.5;

      const targetCamX = robotPos.x - dist * Math.sin(robotYaw);
      const targetCamZ = robotPos.z - dist * Math.cos(robotYaw);
      const targetCamY = robotPos.y + height;

      camera.position.lerp(new THREE.Vector3(targetCamX, targetCamY, targetCamZ), 0.08);
      controls.target.lerp(new THREE.Vector3(robotPos.x, robotPos.y + 0.3, robotPos.z), 0.1);
    }

    // 3. 轨道控制器物理阻尼平滑更新
    if (controls && controls.enabled && !isFlying) {
      controls.update();
    }

    // 4. 数字孪生动画扩展钩子 (小车位姿平滑与激光束)
    if (window.TwinSync && typeof window.TwinSync.update === 'function') {
      window.TwinSync.update(dt);
    }

    // 5. 提交 WebGL 渲染
    if (renderer && scene && camera) {
      renderer.render(scene, camera);
    }
  }

  // 公开接口定义
  return {
    init: init,
    getScene: function () { return scene; },
    getCamera: function () { return camera; },
    getRenderer: function () { return renderer; },
    getControls: function () { return controls; },
    getTerrainMesh: function () { return terrainMesh; },
    flyTo: flyTo,
    setCameraMode: setCameraMode,
    onResize: onWindowResize,
    getFps: function () { return currentFps; },
    getArenaDimensions: function () { return { width: ARENA_WIDTH, length: ARENA_LENGTH }; }
  };
}));
