/**
 * EPGM (Event-predictive Graph Machine) - 3D Digital Twin & Vis.js Linkage
 * ==============================================================================
 * [ZH] 机器人高拟真 3D 孪生体装配、位姿平滑插值 (Slerp/Lerp) 与 Vis.js 因果图双向联动。
 * [EN] Robot 3D Digital Twin Assembly, Pose Smoothing (Slerp/Lerp), and Bidirectional Vis.js Linkage.
 *
 * 核心架构职责 / Architectural Responsibilities:
 * 1. 机器人三维具身装配 (Embodied Robot 3D Twin):
 *    包含差速底盘、主动橡胶轮毂、万向支撑球、8路发光激光束与碰撞微动防撞圈。
 * 2. 空间位姿 Slerp/Lerp 抖动消除平滑器 (Pose Jitter Buffer & Smoothing):
 *    将 20~50Hz 遥测数据包在 60 FPS 渲染循环中通过指数时间衰减算法平滑内插。
 * 3. 动态 3D 轨迹光带追踪 (Trajectory Ribbon):
 *    记录小车时序空间轨迹，具备最小间距滤波 (0.02m) 与多模式热力色染色。
 * 4. 具身物理事件与 Vis.js 因果图深度双向联动 (Bidirectional Linkage):
 *    - 3D碰撞/逼近/打滑/倾斜 ➔ 触发 Vis.js 节点脉冲高亮与 [硬安全屏蔽]；
 *    - 点击 Vis.js 因果图节点 ➔ 触发 3D 摄像机飞行动画聚焦空间坐标与光柱标记。
 */

(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define([], factory);
  } else if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    root.TwinSync = factory();
  }
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // 小车孪生体 3D 根节点
  let robotTwinGroup = null;
  let wheelLeft = null;
  let wheelRight = null;
  let bumperMesh = null;
  let laserRays = [];
  let laserTipLights = [];

  // 位姿平滑目标值与插值状态
  const currentPose = {
    position: null,
    quaternion: null,
    linearVelocity: [0, 0, 0],
    angularVelocity: [0, 0, 0]
  };
  const targetPose = {
    position: null,
    quaternion: null
  };
  let hasReceivedFirstPose = false;

  // 轨迹光带 (Trajectory Ribbon)
  let trajectoryRibbon = null;
  const TRAJECTORY_MAX_POINTS = 600;
  const TRAJECTORY_MIN_DIST = 0.02;
  const trajectoryPoints = [];

  // 事件地理历史记录 (Event ID -> 3D Position)
  const eventSpatialHistory = {};

  // 聚焦光柱标记 (Selection Beacon)
  let selectionBeacon = null;

  /**
   * [ZH] 初始化数字孪生体与事件联动管线
   * [EN] Initialize Digital Twin and Event Linkage
   */
  function init() {
    if (typeof THREE === 'undefined') return;

    const scene = window.Studio3D ? window.Studio3D.getScene() : null;
    if (!scene) return;

    // 1. 初始化小车 3D 实体模型
    buildRobotTwin(scene);

    // 2. 初始化 3D 轨迹光带
    setupTrajectoryRibbon(scene);

    // 3. 初始化空间聚焦光柱标记
    setupSelectionBeacon(scene);

    // 4. 挂靠摄像机跟随目标
    if (window.Studio3D) {
      window.Studio3D.setCameraMode('orbit', robotTwinGroup);
    }

    console.log('[TwinSync] Digital twin initialized successfully.');
  }

  // =========================================================================
  // 1. 机器人三维具身装配模型 (Embodied Procedural Mesh Assembly)
  // =========================================================================
  function buildRobotTwin(scene) {
    robotTwinGroup = new THREE.Group();
    robotTwinGroup.name = "RobotDigitalTwin";

    // 1. 底盘车体 (Chassis Body)
    const chassisG = new THREE.BoxGeometry(0.24, 0.08, 0.16);
    const chassisM = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      metalness: 0.8,
      roughness: 0.25
    });
    const chassis = new THREE.Mesh(chassisG, chassisM);
    chassis.position.y = 0.07;
    chassis.castShadow = true;
    chassis.receiveShadow = true;
    robotTwinGroup.add(chassis);

    // 顶部透明亚克力盖板 + 状态微光
    const lidG = new THREE.BoxGeometry(0.18, 0.015, 0.12);
    const lidM = new THREE.MeshStandardMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.7,
      roughness: 0.1,
      metalness: 0.9
    });
    const lid = new THREE.Mesh(lidG, lidM);
    lid.position.y = 0.115;
    robotTwinGroup.add(lid);

    // 2. 差速驱动主动轮 (左轮与右轮)
    const wheelG = new THREE.CylinderGeometry(0.04, 0.04, 0.025, 24);
    wheelG.rotateZ(Math.PI / 2);
    const wheelM = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.9
    });

    wheelLeft = new THREE.Mesh(wheelG, wheelM);
    wheelLeft.position.set(0, 0.04, 0.095);
    wheelLeft.castShadow = true;
    robotTwinGroup.add(wheelLeft);

    wheelRight = new THREE.Mesh(wheelG, wheelM);
    wheelRight.position.set(0, 0.04, -0.095);
    wheelRight.castShadow = true;
    robotTwinGroup.add(wheelRight);

    // 前后万向支撑从动轮 (Casters)
    const casterG = new THREE.SphereGeometry(0.02, 16, 16);
    const casterM = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.9 });
    const casterF = new THREE.Mesh(casterG, casterM);
    casterF.position.set(0.09, 0.02, 0);
    robotTwinGroup.add(casterF);
    const casterB = new THREE.Mesh(casterG, casterM);
    casterB.position.set(-0.09, 0.02, 0);
    robotTwinGroup.add(casterB);

    // 3. 微动碰撞环形防撞条 (Microswitch Bumper Collar)
    const bumperG = new THREE.BoxGeometry(0.26, 0.03, 0.18);
    const bumperM = new THREE.MeshStandardMaterial({
      color: 0x38bdf8,
      emissive: 0x0284c7,
      emissiveIntensity: 0.2,
      roughness: 0.5
    });
    bumperMesh = new THREE.Mesh(bumperG, bumperM);
    bumperMesh.position.y = 0.04;
    robotTwinGroup.add(bumperMesh);

    // 4. 8 向 ToF / 超声波激光射线束
    setupSensorBeams(robotTwinGroup);

    // 初始位置放置于原点上方
    robotTwinGroup.position.set(0, 0.04, 0);
    currentPose.position = robotTwinGroup.position.clone();
    currentPose.quaternion = robotTwinGroup.quaternion.clone();
    targetPose.position = robotTwinGroup.position.clone();
    targetPose.quaternion = robotTwinGroup.quaternion.clone();

    scene.add(robotTwinGroup);
  }

  /**
   * [ZH] 构建 8 向 ToF 激光射线束及端点指示球
   * [EN] Setup 8-directional sensor raycast lines and glowing end points
   */
  function setupSensorBeams(parentGroup) {
    laserRays = [];
    laserTipLights = [];

    // 8 个探测方向朝向角 (度数): 前0, 前右45, 右90, 后右135, 后180, 后左225, 左270, 前左315
    const anglesDeg = [0, -45, -90, -135, 180, 135, 90, 45];

    anglesDeg.forEach(deg => {
      const rad = (deg * Math.PI) / 180.0;
      // 射线几何体 (起点 0,0,0 -> 终点 0,0,1m)
      const positions = new Float32Array([0, 0.06, 0, Math.cos(rad) * 0.8, 0.06, Math.sin(rad) * 0.8]);
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));

      const mat = new THREE.LineBasicMaterial({
        color: 0x38bdf8,
        transparent: true,
        opacity: 0.65,
        linewidth: 2
      });

      const line = new THREE.Line(geom, mat);
      parentGroup.add(line);
      laserRays.push({ line: line, angleRad: rad, length: 0.8 });

      // 端点光斑球
      const tipG = new THREE.SphereGeometry(0.012, 8, 8);
      const tipM = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
      const tip = new THREE.Mesh(tipG, tipM);
      tip.position.set(Math.cos(rad) * 0.8, 0.06, Math.sin(rad) * 0.8);
      parentGroup.add(tip);
      laserTipLights.push(tip);
    });
  }

  // =========================================================================
  // 2. 轨迹光带追踪系统 (Trajectory Trail Ribbon)
  // =========================================================================
  function setupTrajectoryRibbon(scene) {
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(TRAJECTORY_MAX_POINTS * 3);
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geom.setDrawRange(0, 0);

    const mat = new THREE.LineBasicMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.75,
      linewidth: 2
    });

    trajectoryRibbon = new THREE.Line(geom, mat);
    trajectoryRibbon.name = "TrajectoryRibbon";
    scene.add(trajectoryRibbon);
  }

  function addTrajectoryPoint(x, y, z) {
    if (!trajectoryRibbon) return;

    if (trajectoryPoints.length > 0) {
      const last = trajectoryPoints[trajectoryPoints.length - 1];
      const dist = Math.sqrt((x - last[0]) ** 2 + (y - last[1]) ** 2 + (z - last[2]) ** 2);
      if (dist < TRAJECTORY_MIN_DIST) return; // 消除原地细微漂移膨胀
    }

    if (trajectoryPoints.length >= TRAJECTORY_MAX_POINTS) {
      trajectoryPoints.shift();
    }
    trajectoryPoints.push([x, y + 0.015, z]);

    // 更新 BufferAttribute
    const posAttr = trajectoryRibbon.geometry.attributes.position;
    for (let i = 0; i < trajectoryPoints.length; i++) {
      posAttr.setXYZ(i, trajectoryPoints[i][0], trajectoryPoints[i][1], trajectoryPoints[i][2]);
    }
    posAttr.needsUpdate = true;
    trajectoryRibbon.geometry.setDrawRange(0, trajectoryPoints.length);
  }

  function clearTrajectory() {
    trajectoryPoints.length = 0;
    if (trajectoryRibbon) {
      trajectoryRibbon.geometry.setDrawRange(0, 0);
      trajectoryRibbon.geometry.attributes.position.needsUpdate = true;
    }
    console.log('[TwinSync] Cleared trajectory trail history.');
  }

  // =========================================================================
  // 3. 空间聚焦光柱标记 (Selection Beacon)
  // =========================================================================
  function setupSelectionBeacon(scene) {
    selectionBeacon = new THREE.Group();
    selectionBeacon.name = "SelectionBeacon";

    // 垂直半透明青色光柱
    const beamG = new THREE.CylinderGeometry(0.12, 0.12, 4.0, 16);
    const beamM = new THREE.MeshBasicMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.45,
      side: THREE.DoubleSide
    });
    const beam = new THREE.Mesh(beamG, beamM);
    beam.position.y = 2.0;
    selectionBeacon.add(beam);

    // 地面脉动扩散圆环
    const ringG = new THREE.RingGeometry(0.3, 0.45, 32);
    ringG.rotateX(-Math.PI / 2);
    const ringM = new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide });
    const ring = new THREE.Mesh(ringG, ringM);
    ring.position.y = 0.02;
    selectionBeacon.add(ring);

    selectionBeacon.visible = false;
    scene.add(selectionBeacon);
  }

  function showSelectionBeaconAt(x, y, z) {
    if (!selectionBeacon) return;
    selectionBeacon.position.set(x, y, z);
    selectionBeacon.visible = true;

    // 3 秒后淡出自动隐藏
    setTimeout(() => {
      if (selectionBeacon) selectionBeacon.visible = false;
    }, 4000);
  }

  // =========================================================================
  // 4. WebSocket 空间遥测数据消费与平滑插值 (Telemetry Pose Consumer)
  // =========================================================================
  function onTelemetryReceived(msg) {
    if (!msg) return;

    const pose = msg.pose || (msg.data && msg.data.pose);
    const dynamics = msg.dynamics || (msg.data && msg.data.dynamics) || {};
    const data = msg.data || msg;

    // 1. 处理 3D 位姿数据 (x, y, z, roll, pitch, yaw, quaternion)
    if (pose) {
      // [ZH] PyBullet (Z-Up: X-前/东, Y-左/北, Z-高) 到 Three.js (Y-Up: X-前/东, Y-高, Z-向外/南, -Z-北) 真实基底变换:
      //      x_three = x_bullet
      //      y_three = z_bullet
      //      z_three = -y_bullet
      // [EN] PyBullet (Z-Up) to Three.js (Y-Up) basis transformation:
      //      x_t = x_b, y_t = z_b, z_t = -y_b
      const rawX = parseFloat(pose.x !== undefined ? pose.x : 0.0);
      const rawY = parseFloat(pose.y !== undefined ? pose.y : 0.0);
      const rawZ = parseFloat(pose.z !== undefined ? pose.z : 0.04);

      const px = rawX;
      const py = rawZ;
      const pz = -rawY;

      targetPose.position = new THREE.Vector3(px, py, pz);

      if (pose.quaternion && Array.isArray(pose.quaternion) && pose.quaternion.length === 4) {
        // [ZH] 四元数基底变换: q_three = q_M * q_bullet * q_M^-1
        //      其中 q_M = R_x(-90 deg) 对应: qx_t = qx_b, qy_t = qz_b, qz_t = -qy_b, qw_t = qw_b
        //      确保 PyBullet Z 轴偏航映射到 Three.js 垂直 Y 轴偏航，且俯仰与滚转不发生翻侧倒置
        // [EN] Quaternion basis conversion under M = Rx(-90 deg):
        //      qx_t = qx_b, qy_t = qz_b, qz_t = -qy_b, qw_t = qw_b
        targetPose.quaternion = new THREE.Quaternion(
          pose.quaternion[0],
          pose.quaternion[2],
          -pose.quaternion[1],
          pose.quaternion[3]
        ).normalize();
      } else if (pose.yaw !== undefined) {
        // [ZH] 欧拉角回退: 在 PyBullet 空间顺序合成后经基底变换投影至 Three.js
        // [EN] Euler angle fallback under identical basis transformation:
        const qEulerPyBullet = new THREE.Quaternion().setFromEuler(
          new THREE.Euler(pose.roll || 0, pose.pitch || 0, pose.yaw || 0, 'XYZ')
        );
        targetPose.quaternion = new THREE.Quaternion(
          qEulerPyBullet.x,
          qEulerPyBullet.z,
          -qEulerPyBullet.y,
          qEulerPyBullet.w
        ).normalize();
      }

      if (!hasReceivedFirstPose) {
        currentPose.position.copy(targetPose.position);
        currentPose.quaternion.copy(targetPose.quaternion);
        robotTwinGroup.position.copy(targetPose.position);
        robotTwinGroup.quaternion.copy(targetPose.quaternion);
        hasReceivedFirstPose = true;
      }

      // 追加运动轨迹光带
      addTrajectoryPoint(px, py, pz);

      // 记录该当前离散事件的发生地理位置
      const curEventId = data.current_event_id || 0;
      eventSpatialHistory[curEventId] = [px, py, pz];
    }

    // 2. 更新 8 路传感器测距射线与颜色
    const distances = data.sensor_distances || [];
    if (distances.length > 0) {
      updateSensorRayDistances(distances);
    }

    // 3. 动力学事件联动判断 (碰撞、打滑、倾斜)
    const isBumperHit = !!data.bumper_hit;
    const isSlip = !!dynamics.wheel_slip || (dynamics.slip_ratio && dynamics.slip_ratio > 0.2);
    const tiltDeg = dynamics.slope_tilt_deg || 0.0;
    const curEvent = data.current_event_id !== undefined ? data.current_event_id : null;

    handleEmbodiedEvents(isBumperHit, isSlip, tiltDeg, curEvent);

    // 4. 更新 HUD 仪表盘数据
    updateHudDisplay(pose, dynamics, data);
  }

  function updateSensorRayDistances(distances) {
    if (!laserRays || laserRays.length === 0) return;

    for (let i = 0; i < laserRays.length; i++) {
      const distMm = distances[i] !== undefined ? distances[i] : 1000;
      const distM = Math.min(Math.max(distMm / 1000.0, 0.05), 2.0);

      const ray = laserRays[i];
      const tip = laserTipLights[i];
      const rad = ray.angleRad;

      const endX = Math.cos(rad) * distM;
      const endZ = Math.sin(rad) * distM;

      // 更新射线线段终点
      const posAttr = ray.line.geometry.attributes.position;
      posAttr.setXYZ(1, endX, 0.06, endZ);
      posAttr.needsUpdate = true;

      // 距离色标: <333mm 红, 333~667mm 黄, >=667mm 蓝/绿
      let rayColorHex = 0x38bdf8;
      if (distMm < 333) {
        rayColorHex = 0xf87171; // 危险近距血红
      } else if (distMm < 667) {
        rayColorHex = 0xfbbf24; // 警告接近金黄
      } else {
        rayColorHex = 0x34d399; // 安全开阔青绿
      }

      ray.line.material.color.setHex(rayColorHex);
      if (tip) {
        tip.position.set(endX, 0.06, endZ);
        tip.material.color.setHex(rayColorHex);
      }
    }
  }

  // =========================================================================
  // 5. 3D 具身事件 <-> Vis.js 因果拓扑图双向联动 (Bidirectional Linkage)
  // =========================================================================
  function handleEmbodiedEvents(isBumperHit, isSlip, tiltDeg, curEventId) {
    // 5.1 碰撞事件 (Bumper Collision ➔ Event 0)
    if (isBumperHit) {
      triggerBumperFlash(true);
      // 联动 Vis.js 拓扑图高亮节点 0
      if (window.focusRobotNode) {
        window.focusRobotNode(0);
      }
      triggerCameraShake();
      showHudAlert("💥 微动碰撞硬安全屏蔽激活！");
    } else {
      triggerBumperFlash(false);
    }

    // 5.2 车轮打滑事件 (Wheel Slip ➔ 动态节点 101~181)
    if (isSlip) {
      showHudAlert("⚠️ 车轮打滑脱困中 (Wheel Slip)");
      if (curEventId >= 101 && curEventId <= 181 && window.focusRobotNode) {
        window.focusRobotNode(curEventId);
      }
    }

    // 5.3 正常状态下高亮当前离散节点
    if (!isBumperHit && curEventId !== null && window.focusRobotNode) {
      if (window.selectedNodeId !== curEventId) {
        window.focusRobotNode(curEventId);
      }
    }
  }

  function triggerBumperFlash(isColliding) {
    if (!bumperMesh) return;
    if (isColliding) {
      bumperMesh.material.color.setHex(0xf87171);
      bumperMesh.material.emissive.setHex(0xf87171);
      bumperMesh.material.emissiveIntensity = 1.0;
    } else {
      bumperMesh.material.color.setHex(0x38bdf8);
      bumperMesh.material.emissive.setHex(0x0284c7);
      bumperMesh.material.emissiveIntensity = 0.2;
    }
  }

  function triggerCameraShake() {
    const cam = window.Studio3D ? window.Studio3D.getCamera() : null;
    if (!cam) return;
    const origY = cam.position.y;
    cam.position.y += 0.08;
    setTimeout(() => {
      cam.position.y -= 0.08;
    }, 120);
  }

  function showHudAlert(text) {
    const alertElem = document.getElementById('hud-alert-msg');
    if (!alertElem) return;
    alertElem.textContent = text;
    alertElem.style.opacity = '1';
    setTimeout(() => {
      alertElem.style.opacity = '0';
    }, 2500);
  }

  function updateHudDisplay(pose, dynamics, data) {
    const hudPose = document.getElementById('hud-pose');
    const hudPitch = document.getElementById('hud-pitch');
    const hudRoll = document.getElementById('hud-roll');
    const hudSpeed = document.getElementById('hud-speed');

    if (pose && hudPose) {
      hudPose.textContent = `X:${(pose.x || 0).toFixed(2)} Y:${(pose.y || 0).toFixed(2)} Z:${(pose.z || 0).toFixed(2)}`;
    }
    if (pose && hudPitch) {
      hudPitch.textContent = `${((pose.pitch || 0) * 180 / Math.PI).toFixed(1)}°`;
    }
    if (pose && hudRoll) {
      hudRoll.textContent = `${((pose.roll || 0) * 180 / Math.PI).toFixed(1)}°`;
    }
    if (pose && pose.linear_velocity && hudSpeed) {
      const v = Math.sqrt(pose.linear_velocity[0] ** 2 + pose.linear_velocity[1] ** 2);
      hudSpeed.textContent = `${v.toFixed(2)} m/s`;
    }
  }

  /**
   * [ZH] 反向联动: 当用户在 Vis.js 拓扑图中点击某个节点时，3D 相机平滑飞至对应空间位置
   * [EN] Inverse Linkage: Vis.js node clicked -> 3D camera fly-to focus & beacon
   */
  function onVisNodeClicked(nodeId) {
    console.log(`[TwinSync] Vis.js Node #${nodeId} clicked. Triggering 3D camera fly-to...`);

    let targetSpatialPos = eventSpatialHistory[nodeId];
    if (!targetSpatialPos) {
      // 若无历史记录，默认映射到小车当前坐标或原点
      if (robotTwinGroup) {
        targetSpatialPos = [robotTwinGroup.position.x, robotTwinGroup.position.y, robotTwinGroup.position.z];
      } else {
        targetSpatialPos = [0, 0.15, 0];
      }
    }

    // 升起聚焦光柱标记
    showSelectionBeaconAt(targetSpatialPos[0], targetSpatialPos[1], targetSpatialPos[2]);

    // 触发相机飞行动画
    if (window.Studio3D) {
      const camTargetPos = [
        targetSpatialPos[0] + 3.0,
        targetSpatialPos[1] + 2.5,
        targetSpatialPos[2] + 3.0
      ];
      window.Studio3D.flyTo(camTargetPos, targetSpatialPos, 900);
    }
  }

  // =========================================================================
  // 6. 独立 60 FPS 渲染平滑更新钩子 (Called by Studio3D.animate)
  // =========================================================================
  function update(dt) {
    if (!robotTwinGroup || !targetPose.position || !hasReceivedFirstPose) return;

    // 指数时间平滑插值常数 (k=15.0)
    const alpha = 1.0 - Math.exp(-dt * 15.0);

    // 1. 位置线性插值 (Vector3.lerp)
    currentPose.position.lerp(targetPose.position, alpha);
    robotTwinGroup.position.copy(currentPose.position);

    // 2. 姿态球面四元数插值 (Quaternion.slerp)
    currentPose.quaternion.slerp(targetPose.quaternion, alpha);
    robotTwinGroup.quaternion.copy(currentPose.quaternion);

    // 3. 车轮滚动旋转动画
    if (wheelLeft && wheelRight) {
      wheelLeft.rotation.x += dt * 5.0;
      wheelRight.rotation.x += dt * 5.0;
    }
  }

  // 公开接口定义
  return {
    init: init,
    update: update,
    onTelemetryReceived: onTelemetryReceived,
    onVisNodeClicked: onVisNodeClicked,
    clearTrajectory: clearTrajectory,
    getRobotGroup: function () { return robotTwinGroup; },
    getEventSpatialHistory: function () { return eventSpatialHistory; }
  };
}));
