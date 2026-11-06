/**
 * BANTAY-SUNOG Virtual Sensor Suite
 * Simulates:
 * - 3x HC-SR04 Ultrasonic Sensors with 3D ray visualizations
 * - Hardware IR Flame Sensor with cone & occlusion line-of-sight check
 * - Virtual USB Webcam with PIP rendering and normalized flame bounding box calculation
 * - Extinguisher Solenoid spray particle cone
 */

class SensorSimulator {
  constructor(scene, robot, fireSystem, obstacleManager, pipCanvas) {
    this.scene = scene;
    this.robot = robot;
    this.fireSystem = fireSystem;
    this.obstacleManager = obstacleManager;
    this.pipCanvas = pipCanvas;

    // -------------------------------------------------------------------------
    // 1. Virtual Camera (USB Webcam matching Pi resolution & FOV)
    // -------------------------------------------------------------------------
    this.cameraFov = 65; // ~65 deg FOV standard USB webcam
    this.camAspect = 4 / 3; // 640x480
    this.virtualCamera = new THREE.PerspectiveCamera(this.cameraFov, this.camAspect, 0.05, 25.0);

    // Create a secondary renderer for the Picture-in-Picture PIP canvas
    this.pipRenderer = new THREE.WebGLRenderer({
      canvas: this.pipCanvas,
      antialias: true,
      alpha: false
    });
    this.pipRenderer.setSize(250, 187);
    this.pipRenderer.setClearColor(0x0a0e14);
    this.pipRenderer.shadowMap.enabled = true;

    // Attach camera to robot tilt mount facing forward along optical axis (+Z)
    this.virtualCamera.rotation.y = Math.PI;
    this.robot.cameraMount.add(this.virtualCamera);

    // -------------------------------------------------------------------------
    // 2. Ultrasonic Raycasters & Visual Beams
    // -------------------------------------------------------------------------
    this.maxUltrasonicDistMeters = 3.0; // 300 cm
    this.raycaster = new THREE.Raycaster();
    this.raycaster.far = this.maxUltrasonicDistMeters;

    this.distances = {
      front: 300.0,
      left: 300.0,
      right: 300.0
    };

    // 3D Visualizer lines for sensor rays
    this.rayVisuals = {};
    const rayColors = { front: 0x00d2ff, left: 0x38bdf8, right: 0x38bdf8 };

    ['front', 'left', 'right'].forEach((key) => {
      const geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0),
        new THREE.Vector3(0, 0, 1)
      ]);
      const mat = new THREE.LineBasicMaterial({
        color: rayColors[key],
        transparent: true,
        opacity: 0.85
      });
      const line = new THREE.Line(geo, mat);
      this.scene.add(line);
      this.rayVisuals[key] = { line, material: mat };
    });

    // -------------------------------------------------------------------------
    // 3. Flame Sensor State
    // -------------------------------------------------------------------------
    this.flameDetected = false;
    this.detectedFlameTarget = null; // { x, y, confidence, bbox, distance }

    // -------------------------------------------------------------------------
    // 4. Extinguisher Spray Particles
    // -------------------------------------------------------------------------
    this.initSprayParticles();
  }

  initSprayParticles() {
    this.sprayCount = 80;
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(this.sprayCount * 3);
    const opacities = new Float32Array(this.sprayCount);

    this.sprayData = [];
    for (let i = 0; i < this.sprayCount; i++) {
      positions[i * 3] = 0;
      positions[i * 3 + 1] = 0;
      positions[i * 3 + 2] = 0;
      opacities[i] = 0.0;
      this.sprayData.push({
        vx: 0,
        vy: 0,
        vz: 0,
        life: 1.0,
        maxLife: 0.35 + Math.random() * 0.2
      });
    }

    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    const mat = new THREE.PointsMaterial({
      color: 0xe0f2fe,
      size: 0.12,
      transparent: true,
      opacity: 0.8,
      blending: THREE.AdditiveBlending
    });

    this.sprayCloud = new THREE.Points(geo, mat);
    this.sprayCloud.visible = false;
    this.scene.add(this.sprayCloud);
  }

  update(dt, arenaWalls = []) {
    // -------------------------------------------------------------------------
    // 1. Raycast Ultrasonic Sensors
    // -------------------------------------------------------------------------
    const collidableMeshes = [
      ...this.obstacleManager.getRaycastMeshes(),
      ...arenaWalls
    ];

    const sensorConfigs = [
      { key: 'front', mount: this.robot.ultrasonicMounts.front, angleOffset: 0 },
      { key: 'left', mount: this.robot.ultrasonicMounts.left, angleOffset: -Math.PI / 4 },
      { key: 'right', mount: this.robot.ultrasonicMounts.right, angleOffset: Math.PI / 4 }
    ];

    sensorConfigs.forEach((cfg) => {
      if (!cfg.mount) return;

      const origin = new THREE.Vector3();
      cfg.mount.getWorldPosition(origin);

      // Ray direction based on robot yaw + sensor angle offset
      const rayYaw = this.robot.yaw + cfg.angleOffset;
      const direction = new THREE.Vector3(Math.sin(rayYaw), 0, Math.cos(rayYaw)).normalize();

      this.raycaster.set(origin, direction);
      const intersects = this.raycaster.intersectObjects(collidableMeshes, false);

      let distMeters = this.maxUltrasonicDistMeters;
      if (intersects.length > 0) {
        distMeters = intersects[0].distance;
      }

      const distCm = Math.round(distMeters * 100.0 * 10.0) / 10.0;
      this.distances[cfg.key] = distCm;

      // Update 3D visual ray
      const vis = this.rayVisuals[cfg.key];
      const endPoint = origin.clone().add(direction.clone().multiplyScalar(distMeters));
      const posAttr = vis.line.geometry.attributes.position;
      posAttr.setXYZ(0, origin.x, origin.y, origin.z);
      posAttr.setXYZ(1, endPoint.x, endPoint.y, endPoint.z);
      posAttr.needsUpdate = true;

      // Color coding: Red if obstacle < 25 cm, Yellow if < 40 cm, Cyan if clear
      if (distCm < 25.0) {
        vis.material.color.setHex(0xff334b);
      } else if (distCm < 40.0) {
        vis.material.color.setHex(0xffaa00);
      } else {
        vis.material.color.setHex(0x00d2ff);
      }
    });

    // -------------------------------------------------------------------------
    // 2. Hardware IR Flame Sensor Simulation (FOV + Occlusion)
    // -------------------------------------------------------------------------
    this.flameDetected = false;
    this.detectedFlameTarget = null;

    const flameOrigin = new THREE.Vector3();
    this.robot.flameSensorMount.getWorldPosition(flameOrigin);

    // Optical forward vector of tilt turret
    const opticalDir = new THREE.Vector3();
    this.robot.cameraMount.getWorldDirection(opticalDir);

    const activeFires = this.fireSystem.getActiveFires();

    for (const fire of activeFires) {
      const fireCenter = fire.position.clone().add(new THREE.Vector3(0, 0.25, 0));
      const toFire = new THREE.Vector3().subVectors(fireCenter, flameOrigin);
      const dist = toFire.length();

      // Check max detection range (~2.5m)
      if (dist <= 2.5) {
        const toFireNorm = toFire.clone().normalize();
        const dot = opticalDir.dot(toFireNorm);
        const angle = Math.acos(Math.min(1.0, Math.max(-1.0, dot)));

        // Check sensor FOV cone (+/- 30 deg = 0.52 rad)
        if (angle <= 0.52) {
          // Check line-of-sight obstruction by obstacles
          this.raycaster.set(flameOrigin, toFireNorm);
          const hits = this.raycaster.intersectObjects(collidableMeshes, false);

          let occluded = false;
          if (hits.length > 0 && hits[0].distance < dist - 0.2) {
            occluded = true; // Obstacle is between sensor and flame
          }

          if (!occluded) {
            this.flameDetected = true;

            // Calculate projected normalized screen coordinates (x, y) in camera view
            const screenPos = fireCenter.clone().project(this.virtualCamera);
            // screenPos.x and screenPos.y are in [-1, 1] NDC
            if (screenPos.z > 0 && screenPos.z < 1) {
              const normX = (1 - screenPos.x) / 2; // Inverted for forward camera horizontal mirroring
              const normY = (1 - screenPos.y) / 2; // Inverted Y for image coords

              // Estimate bounding box in 640x480 pixel space
              const boxScale = Math.min(220, Math.max(30, 180 / dist));
              const pixelX = Math.round(normX * 640 - boxScale / 2);
              const pixelY = Math.round(normY * 480 - boxScale / 2);

              this.detectedFlameTarget = {
                fireId: fire.id,
                confidence: Math.min(0.98, Math.max(0.65, 1.0 - (dist / 3.0) * 0.4)),
                x: Math.round(normX * 1000) / 1000,
                y: Math.round(normY * 1000) / 1000,
                bbox: [pixelX, pixelY, Math.round(boxScale), Math.round(boxScale * 1.2)],
                distanceCm: Math.round(dist * 100)
              };
            }
            break;
          }
        }
      }
    }

    // -------------------------------------------------------------------------
    // 3. Extinguisher Spray Simulation
    // -------------------------------------------------------------------------
    this.updateSpray(dt);

    // -------------------------------------------------------------------------
    // 4. Render Virtual Camera PIP
    // -------------------------------------------------------------------------
    this.renderPip();
  }

  updateSpray(dt) {
    if (!this.robot.solenoidActive) {
      this.sprayCloud.visible = false;
      return;
    }

    this.sprayCloud.visible = true;

    const nozzlePos = new THREE.Vector3();
    this.robot.nozzleMount.getWorldPosition(nozzlePos);

    const sprayDir = new THREE.Vector3();
    this.robot.nozzleMount.getWorldDirection(sprayDir);

    // Apply extinguisher suppression effect on fire system
    this.fireSystem.applyExtinguisher(nozzlePos, sprayDir, 1.2, 0.4, 0.55, dt);

    // Update spray particle dynamics
    const posAttr = this.sprayCloud.geometry.attributes.position;
    for (let i = 0; i < this.sprayCount; i++) {
      const p = this.sprayData[i];
      p.life += dt;

      if (p.life >= p.maxLife) {
        p.life = 0;
        // Particle origin at nozzle tip with small conical spread
        const spreadX = (Math.random() - 0.5) * 0.35;
        const spreadY = (Math.random() - 0.5) * 0.35;
        p.vx = sprayDir.x * 2.8 + spreadX;
        p.vy = sprayDir.y * 2.8 + spreadY;
        p.vz = sprayDir.z * 2.8 + (Math.random() - 0.5) * 0.2;

        posAttr.setXYZ(i, nozzlePos.x, nozzlePos.y, nozzlePos.z);
      } else {
        const curX = posAttr.getX(i) + p.vx * dt;
        const curY = posAttr.getY(i) + p.vy * dt - 0.2 * dt; // Slight gravity
        const curZ = posAttr.getZ(i) + p.vz * dt;
        posAttr.setXYZ(i, curX, curY, curZ);
      }
    }
    posAttr.needsUpdate = true;
  }

  renderPip() {
    // Hide ray visualizers in camera view so robot's camera sees realistic clean world
    Object.values(this.rayVisuals).forEach(v => v.line.visible = false);
    this.pipRenderer.render(this.scene, this.virtualCamera);
    Object.values(this.rayVisuals).forEach(v => v.line.visible = true);
  }

  getTelemetry() {
    return {
      dist_front: this.distances.front,
      dist_left: this.distances.left,
      dist_right: this.distances.right,
      flame_detected: this.flameDetected,
      tilt_angle: Math.round(this.robot.currentTiltDeg),
      solenoid_active: this.robot.solenoidActive,
      left_pwm: this.robot.leftPwm,
      right_pwm: this.robot.rightPwm,
      target: this.detectedFlameTarget
    };
  }
}
