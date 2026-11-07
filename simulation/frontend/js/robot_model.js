/**
 * BANTAY-SUNOG Robot Digital Twin
 * 4WD Skid-Steer Kinematics, Articulated Tilt Gimbal, and Modular 3D Mesh Slot.
 */

class RobotModel {
  constructor(scene) {
    this.scene = scene;

    // Kinematic & hardware parameters
    this.trackWidth = 0.24;    // Meters between left and right wheels
    this.wheelRadius = 0.042;  // Wheel radius in meters
    this.maxSpeed = 0.65;      // Max linear speed (m/s at PWM 255)
    this.maxTurnRate = 2.8;    // Max yaw rate (rad/s at pivot PWM 255)
    this.minPwm = 70;          // Min threshold to overcome static friction

    // State
    this.position = new THREE.Vector3(0, 0, 0);
    this.yaw = 0.0;            // Heading in radians (0 = forward +Z, or -Z)
    this.leftPwm = 0;
    this.rightPwm = 0;
    this.currentLinearSpeed = 0.0;  // m/s (smoothed for physics inertia)
    this.currentAngularSpeed = 0.0; // rad/s (smoothed for rotational inertia)
    this.maxLinearAcc = 2.6;        // m/s^2 linear acceleration/braking
    this.maxAngularAcc = 10.5;      // rad/s^2 angular acceleration
    this.tiltAngleDeg = 90.0;  // 90 = level horizon, 45 = down, 135 = up
    this.currentTiltDeg = 90.0;
    this.solenoidActive = false;

    // Root Group
    this.root = new THREE.Group();
    this.wheels = [];
    this.tiltGimbal = null;
    this.cameraMount = null;
    this.nozzleMount = null;
    this.flameSensorMount = null;
    this.ultrasonicMounts = { front: null, left: null, right: null };

    // Custom mesh container (for user's 3D model)
    this.customMesh = null;
    this.proceduralGroup = new THREE.Group();
    this.root.add(this.proceduralGroup);

    this.initProceduralModel();
    this.scene.add(this.root);
  }

  initProceduralModel() {
    // -------------------------------------------------------------------------
    // 1. Chassis Body
    // -------------------------------------------------------------------------
    const chassisGeo = new THREE.BoxGeometry(0.22, 0.07, 0.30);
    const chassisMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      metalness: 0.5,
      roughness: 0.4
    });
    const chassisMesh = new THREE.Mesh(chassisGeo, chassisMat);
    chassisMesh.position.y = 0.065;
    chassisMesh.castShadow = true;
    chassisMesh.receiveShadow = true;
    this.proceduralGroup.add(chassisMesh);

    // Chassis accent stripe
    const stripeGeo = new THREE.BoxGeometry(0.222, 0.015, 0.12);
    const stripeMat = new THREE.MeshStandardMaterial({
      color: 0x00d2ff,
      emissive: 0x005577,
      roughness: 0.2
    });
    const stripeMesh = new THREE.Mesh(stripeGeo, stripeMat);
    stripeMesh.position.set(0, 0.065, 0.02);
    this.proceduralGroup.add(stripeMesh);

    // Electronics Deck (Raspberry Pi & ESP32 mockup)
    const deckGeo = new THREE.BoxGeometry(0.18, 0.01, 0.22);
    const deckMat = new THREE.MeshStandardMaterial({ color: 0x064e3b, roughness: 0.6 });
    const deckMesh = new THREE.Mesh(deckGeo, deckMat);
    deckMesh.position.set(0, 0.105, -0.02);
    this.proceduralGroup.add(deckMesh);

    // ESP32 Status LED (Glowing Blue)
    const ledGeo = new THREE.SphereGeometry(0.006, 8, 8);
    const ledMat = new THREE.MeshBasicMaterial({ color: 0x00d2ff });
    const ledMesh = new THREE.Mesh(ledGeo, ledMat);
    ledMesh.position.set(0.05, 0.115, -0.05);
    this.proceduralGroup.add(ledMesh);

    // Battery Pack
    const battGeo = new THREE.BoxGeometry(0.09, 0.035, 0.13);
    const battMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.8 });
    const battMesh = new THREE.Mesh(battGeo, battMat);
    battMesh.position.set(-0.03, 0.12, -0.05);
    this.proceduralGroup.add(battMesh);

    // -------------------------------------------------------------------------
    // 2. Wheels (4WD Differential/Skid Drive)
    // -------------------------------------------------------------------------
    const wheelGeo = new THREE.CylinderGeometry(this.wheelRadius, this.wheelRadius, 0.032, 20);
    wheelGeo.rotateZ(Math.PI / 2);
    const wheelMat = new THREE.MeshStandardMaterial({
      color: 0x111111,
      roughness: 0.9,
      metalness: 0.1
    });
    const rimMat = new THREE.MeshStandardMaterial({
      color: 0xd97706,
      metalness: 0.7,
      roughness: 0.3
    });

    const wheelOffsets = [
      { x: -0.125, y: this.wheelRadius, z: 0.10, isLeft: true },   // Front Left
      { x: -0.125, y: this.wheelRadius, z: -0.10, isLeft: true },  // Rear Left
      { x: 0.125, y: this.wheelRadius, z: 0.10, isLeft: false },   // Front Right
      { x: 0.125, y: this.wheelRadius, z: -0.10, isLeft: false }   // Rear Right
    ];

    wheelOffsets.forEach((cfg) => {
      const wheelGroup = new THREE.Group();
      wheelGroup.position.set(cfg.x, cfg.y, cfg.z);

      const tire = new THREE.Mesh(wheelGeo, wheelMat);
      tire.castShadow = true;
      wheelGroup.add(tire);

      // Rim hub
      const rimGeo = new THREE.CylinderGeometry(this.wheelRadius * 0.55, this.wheelRadius * 0.55, 0.033, 12);
      rimGeo.rotateZ(Math.PI / 2);
      const rim = new THREE.Mesh(rimGeo, rimMat);
      wheelGroup.add(rim);

      this.proceduralGroup.add(wheelGroup);
      this.wheels.push({ group: wheelGroup, isLeft: cfg.isLeft, angle: 0 });
    });

    // -------------------------------------------------------------------------
    // 3. 3x HC-SR04 Ultrasonic Mounts (Front, Left, Right)
    // -------------------------------------------------------------------------
    this.createUltrasonicSensor('front', 0, 0.065, 0.155, 0);
    this.createUltrasonicSensor('left', -0.09, 0.065, 0.13, Math.PI / 4);
    this.createUltrasonicSensor('right', 0.09, 0.065, 0.13, -Math.PI / 4);

    // -------------------------------------------------------------------------
    // 4. Articulated Tilt Gimbal (Pitch Servo Assembly)
    // -------------------------------------------------------------------------
    const gimbalBase = new THREE.Group();
    gimbalBase.position.set(0, 0.12, 0.08); // Riser position
    this.proceduralGroup.add(gimbalBase);

    // Servo Bracket
    const bracketGeo = new THREE.BoxGeometry(0.04, 0.035, 0.04);
    const bracketMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.6 });
    const bracketMesh = new THREE.Mesh(bracketGeo, bracketMat);
    bracketMesh.position.y = 0.015;
    gimbalBase.add(bracketMesh);

    // Tilt Pivot Group (rotates around X axis)
    this.tiltGimbal = new THREE.Group();
    this.tiltGimbal.position.set(0, 0.03, 0);
    gimbalBase.add(this.tiltGimbal);

    // Camera module
    const camGeo = new THREE.BoxGeometry(0.06, 0.03, 0.035);
    const camMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.3 });
    const camMesh = new THREE.Mesh(camGeo, camMat);
    camMesh.castShadow = true;
    this.tiltGimbal.add(camMesh);

    // Camera lens barrel
    const lensGeo = new THREE.CylinderGeometry(0.012, 0.012, 0.018, 16);
    lensGeo.rotateX(Math.PI / 2);
    const lensMat = new THREE.MeshStandardMaterial({
      color: 0x111111,
      roughness: 0.1,
      metalness: 0.9
    });
    const lensMesh = new THREE.Mesh(lensGeo, lensMat);
    lensMesh.position.set(0, 0, 0.024);
    this.tiltGimbal.add(lensMesh);

    // Camera Viewport Anchor
    this.cameraMount = new THREE.Object3D();
    this.cameraMount.position.set(0, 0, 0.035);
    this.tiltGimbal.add(this.cameraMount);

    // Hardware Flame Sensor Module (Mounted co-aligned above camera)
    const flameSensorGeo = new THREE.BoxGeometry(0.02, 0.01, 0.03);
    const flameSensorMat = new THREE.MeshStandardMaterial({ color: 0x1e3a8a });
    const flameSensorMesh = new THREE.Mesh(flameSensorGeo, flameSensorMat);
    flameSensorMesh.position.set(0, 0.02, 0.01);
    this.tiltGimbal.add(flameSensorMesh);

    // Black IR Receiver Diode tube
    const diodeGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.01, 8);
    diodeGeo.rotateX(Math.PI / 2);
    const diodeMat = new THREE.MeshBasicMaterial({ color: 0x050505 });
    const diodeMesh = new THREE.Mesh(diodeGeo, diodeMat);
    diodeMesh.position.set(0, 0.02, 0.028);
    this.tiltGimbal.add(diodeMesh);

    this.flameSensorMount = new THREE.Object3D();
    this.flameSensorMount.position.set(0, 0.02, 0.03);
    this.tiltGimbal.add(this.flameSensorMount);

    // Fire Extinguisher Solenoid Nozzle (Brass tip below camera)
    const nozzleGeo = new THREE.CylinderGeometry(0.006, 0.008, 0.035, 12);
    nozzleGeo.rotateX(Math.PI / 2);
    const nozzleMat = new THREE.MeshStandardMaterial({
      color: 0xd97706,
      metalness: 0.8,
      roughness: 0.2
    });
    const nozzleMesh = new THREE.Mesh(nozzleGeo, nozzleMat);
    nozzleMesh.position.set(0, -0.02, 0.025);
    this.tiltGimbal.add(nozzleMesh);

    this.nozzleMount = new THREE.Object3D();
    this.nozzleMount.position.set(0, -0.02, 0.042);
    this.tiltGimbal.add(this.nozzleMount);
  }

  createUltrasonicSensor(key, x, y, z, yaw) {
    const group = new THREE.Group();
    group.position.set(x, y, z);
    group.rotation.y = yaw;

    // PCB Backplate
    const pcbGeo = new THREE.BoxGeometry(0.045, 0.02, 0.004);
    const pcbMat = new THREE.MeshStandardMaterial({ color: 0x1d4ed8 });
    const pcb = new THREE.Mesh(pcbGeo, pcbMat);
    group.add(pcb);

    // Dual silver transducer cylinders ("eyes")
    const transGeo = new THREE.CylinderGeometry(0.007, 0.007, 0.012, 12);
    transGeo.rotateX(Math.PI / 2);
    const transMat = new THREE.MeshStandardMaterial({
      color: 0xc0c0c0,
      metalness: 0.8,
      roughness: 0.3
    });

    const leftEye = new THREE.Mesh(transGeo, transMat);
    leftEye.position.set(-0.013, 0, 0.007);
    group.add(leftEye);

    const rightEye = new THREE.Mesh(transGeo, transMat);
    rightEye.position.set(0.013, 0, 0.007);
    group.add(rightEye);

    this.proceduralGroup.add(group);
    this.ultrasonicMounts[key] = group;
  }

  /**
   * Set motor PWM values (-255 to +255).
   */
  setMotorPwm(left, right) {
    this.leftPwm = Math.max(-255, Math.min(255, left));
    this.rightPwm = Math.max(-255, Math.min(255, right));
  }

  /**
   * Set target tilt servo angle in degrees (45 to 135 deg).
   */
  setTiltAngle(deg) {
    this.tiltAngleDeg = Math.max(45, Math.min(135, deg));
  }

  /**
   * Replace procedural body with custom user-imported 3D mesh.
   */
  replaceWithCustomMesh(loadedMesh, options = {}) {
    if (this.customMesh) {
      this.root.remove(this.customMesh);
    }
    // Hide procedural body meshes while keeping kinematic pivots
    this.proceduralGroup.visible = false;

    this.customMesh = loadedMesh;
    const scale = options.scale || 1.0;
    this.customMesh.scale.set(scale, scale, scale);
    if (options.positionOffset) {
      this.customMesh.position.copy(options.positionOffset);
    }
    if (options.rotationOffset) {
      this.customMesh.rotation.copy(options.rotationOffset);
    }

    this.root.add(this.customMesh);
  }

  /**
   * Revert to procedural CAD model.
   */
  useProceduralModel() {
    if (this.customMesh) {
      this.customMesh.visible = false;
    }
    this.proceduralGroup.visible = true;
  }

  update(dt, obstacles = [], arenaRadius = 4.8) {
    // -------------------------------------------------------------------------
    // 1. Tilt Servo Smoothing
    // -------------------------------------------------------------------------
    const tiltDiff = this.tiltAngleDeg - this.currentTiltDeg;
    const maxTiltSpeed = 120.0 * dt; // Degrees per second
    if (Math.abs(tiltDiff) <= maxTiltSpeed) {
      this.currentTiltDeg = this.tiltAngleDeg;
    } else {
      this.currentTiltDeg += Math.sign(tiltDiff) * maxTiltSpeed;
    }

    // Physical 90 deg = 0 rad pitch (level horizon).
    // Physical 45 deg = looking downward (-45 deg = -0.785 rad).
    // Physical 135 deg = looking upward (+45 deg = +0.785 rad).
    const pitchRad = ((this.currentTiltDeg - 90.0) * Math.PI) / 180.0;
    this.tiltGimbal.rotation.x = -pitchRad; // Negative pitch tilts forward/downward

    // -------------------------------------------------------------------------
    // 2. Differential Drive Kinematics with Vehicle Dynamics & Inertia
    // -------------------------------------------------------------------------
    let targetVLeft = 0;
    let targetVRight = 0;

    if (Math.abs(this.leftPwm) >= this.minPwm) {
      targetVLeft = (this.leftPwm / 255.0) * this.maxSpeed;
    }
    if (Math.abs(this.rightPwm) >= this.minPwm) {
      targetVRight = (this.rightPwm / 255.0) * this.maxSpeed;
    }

    // Target linear and angular speeds (matching arena_sim.py: v_left > v_right turns right)
    const targetLinearSpeed = (targetVLeft + targetVRight) / 2.0;
    const targetAngularSpeed = (targetVLeft - targetVRight) / this.trackWidth;

    // Apply linear acceleration / braking inertia
    const linearDiff = targetLinearSpeed - this.currentLinearSpeed;
    const maxLinearStep = this.maxLinearAcc * dt;
    if (Math.abs(linearDiff) <= maxLinearStep) {
      this.currentLinearSpeed = targetLinearSpeed;
    } else {
      this.currentLinearSpeed += Math.sign(linearDiff) * maxLinearStep;
    }

    // Apply rotational acceleration / inertia
    const angularDiff = targetAngularSpeed - this.currentAngularSpeed;
    const maxAngularStep = this.maxAngularAcc * dt;
    if (Math.abs(angularDiff) <= maxAngularStep) {
      this.currentAngularSpeed = targetAngularSpeed;
    } else {
      this.currentAngularSpeed += Math.sign(angularDiff) * maxAngularStep;
    }

    // Update heading (yaw)
    this.yaw += this.currentAngularSpeed * dt;

    // Desired translation vector (in robot local forward +Z direction)
    const forwardX = Math.sin(this.yaw);
    const forwardZ = Math.cos(this.yaw);

    const nextX = this.position.x + forwardX * this.currentLinearSpeed * dt;
    const nextZ = this.position.z + forwardZ * this.currentLinearSpeed * dt;

    // -------------------------------------------------------------------------
    // 3. Accurate Two-Circle Capsule Collision Resolution & Sliding Physics
    // -------------------------------------------------------------------------
    // Two overlapping circles along chassis longitudinal centerline:
    // Front circle and Rear circle eliminate corner clipping during rotation & doorways
    const circleRadius = 0.13;
    const circleOffset = 0.075;
    let resolvedX = nextX;
    let resolvedZ = nextZ;

    // Helper: Resolve rectangular boundary limits (9.0m x 9.0m arena by default)
    const clampArenaRect = (x, z, r = 0.16) => {
      let boundX = 4.5 - 0.075 - r;
      let boundZ = 4.5 - 0.075 - r;
      if (typeof arenaRadius === 'object' && arenaRadius !== null) {
        boundX = arenaRadius.maxX !== undefined ? arenaRadius.maxX : boundX;
        boundZ = arenaRadius.maxZ !== undefined ? arenaRadius.maxZ : boundZ;
      }
      return {
        x: Math.max(-boundX, Math.min(boundX, x)),
        z: Math.max(-boundZ, Math.min(boundZ, z))
      };
    };

    let totalPushX = 0;
    let totalPushZ = 0;

    // 3-pass iterative constraint solver for zero clipping & multi-contact corner wedging
    for (let iter = 0; iter < 3; iter++) {
      // Check both front and rear circles of the robot capsule
      const circles = [
        { x: resolvedX + forwardX * circleOffset, z: resolvedZ + forwardZ * circleOffset },
        { x: resolvedX - forwardX * circleOffset, z: resolvedZ - forwardZ * circleOffset }
      ];

      for (const circle of circles) {
        // 1. Boundary walls
        const clamped = clampArenaRect(circle.x, circle.z, circleRadius);
        const wallDx = clamped.x - circle.x;
        const wallDz = clamped.z - circle.z;
        if (Math.abs(wallDx) > 0.0001 || Math.abs(wallDz) > 0.0001) {
          resolvedX += wallDx;
          resolvedZ += wallDz;
          totalPushX += wallDx;
          totalPushZ += wallDz;
        }

        // 2. Obstacles (boxes, walls, cylinders)
        if (obstacles && obstacles.length > 0) {
          for (const obs of obstacles) {
            if (!obs.userData) continue;
            const oPos = obs.position;
            const oType = obs.userData.type;

            if (oType === 'cylinder') {
              const cRadius = obs.userData.radius || 0.3;
              const minSep = circleRadius + cRadius;
              const dx = circle.x - oPos.x;
              const dz = circle.z - oPos.z;
              const dist = Math.hypot(dx, dz);

              if (dist < minSep) {
                const pushDist = minSep - dist + 0.002;
                const px = (dist > 0.001) ? (dx / dist) * pushDist : pushDist;
                const pz = (dist > 0.001) ? (dz / dist) * pushDist : 0;
                resolvedX += px;
                resolvedZ += pz;
                totalPushX += px;
                totalPushZ += pz;
              }
            } else if (oType === 'box' || oType === 'wall') {
              const halfW = (obs.userData.width || 0.6) / 2;
              const halfD = (obs.userData.depth || 0.6) / 2;
              const rotY = obs.rotation.y || 0;

              const dx = circle.x - oPos.x;
              const dz = circle.z - oPos.z;

              const cosA = Math.cos(-rotY);
              const sinA = Math.sin(-rotY);
              const localX = dx * cosA - dz * sinA;
              const localZ = dx * sinA + dz * cosA;

              const closestX = Math.max(-halfW, Math.min(halfW, localX));
              const closestZ = Math.max(-halfD, Math.min(halfD, localZ));

              const diffX = localX - closestX;
              const diffZ = localZ - closestZ;
              const localDist = Math.hypot(diffX, diffZ);

              if (localDist < circleRadius) {
                let pushLocalX = 0;
                let pushLocalZ = 0;

                if (localDist > 0.001) {
                  const pushDist = circleRadius - localDist + 0.002;
                  pushLocalX = (diffX / localDist) * pushDist;
                  pushLocalZ = (diffZ / localDist) * pushDist;
                } else {
                  const penLeft = localX - (-halfW);
                  const penRight = halfW - localX;
                  const penBottom = localZ - (-halfD);
                  const penTop = halfD - localZ;
                  const minPen = Math.min(penLeft, penRight, penBottom, penTop);

                  if (minPen === penLeft) {
                    pushLocalX = -(penLeft + circleRadius + 0.002);
                  } else if (minPen === penRight) {
                    pushLocalX = penRight + circleRadius + 0.002;
                  } else if (minPen === penBottom) {
                    pushLocalZ = -(penBottom + circleRadius + 0.002);
                  } else {
                    pushLocalZ = penTop + circleRadius + 0.002;
                  }
                }

                const cosB = Math.cos(rotY);
                const sinB = Math.sin(rotY);
                const pushWorldX = pushLocalX * cosB - pushLocalZ * sinB;
                const pushWorldZ = pushLocalX * sinB + pushLocalZ * cosB;

                resolvedX += pushWorldX;
                resolvedZ += pushWorldZ;
                totalPushX += pushWorldX;
                totalPushZ += pushWorldZ;
              }
            }
          }
        }
      }
    }

    // Velocity Normal Cancellation: Smooth Wall Sliding Physics
    const pushMag = Math.hypot(totalPushX, totalPushZ);
    if (pushMag > 0.002) {
      const normX = totalPushX / pushMag;
      const normZ = totalPushZ / pushMag;

      // Robot velocity vector
      const vx = forwardX * this.currentLinearSpeed;
      const vz = forwardZ * this.currentLinearSpeed;
      const vDotN = vx * normX + vz * normZ;

      // If velocity is moving into the contact surface normal, cancel the penetrating normal component
      if (vDotN < 0) {
        const slideVx = vx - vDotN * normX;
        const slideVz = vz - vDotN * normZ;
        // Project slide velocity back onto robot forward heading with friction damping (0.88)
        const forwardProjection = (slideVx * forwardX + slideVz * forwardZ);
        this.currentLinearSpeed = Math.max(0, forwardProjection) * 0.88;
      }
    }

    // Final boundary clamp
    const clampedFinal = clampArenaRect(resolvedX, resolvedZ, 0.16);
    this.position.x = clampedFinal.x;
    this.position.z = clampedFinal.z;

    // Synchronize 3D Root Transform
    this.root.position.copy(this.position);
    this.root.rotation.y = this.yaw;

    // -------------------------------------------------------------------------
    // 4. Animate Wheel Rotations Proportional to Actual Speed
    // -------------------------------------------------------------------------
    const actualVLeft = this.currentLinearSpeed + (this.currentAngularSpeed * this.trackWidth / 2.0);
    const actualVRight = this.currentLinearSpeed - (this.currentAngularSpeed * this.trackWidth / 2.0);

    const leftSpin = (actualVLeft / this.wheelRadius) * dt;
    const rightSpin = (actualVRight / this.wheelRadius) * dt;

    this.wheels.forEach((w) => {
      w.angle += w.isLeft ? leftSpin : rightSpin;
      w.group.rotation.x = w.angle;
    });
  }

  /**
   * Reset position to origin.
   */
  reset(x = 0, z = 0, yaw = 0) {
    this.position.set(x, 0, z);
    this.yaw = yaw;
    this.leftPwm = 0;
    this.rightPwm = 0;
    this.currentLinearSpeed = 0.0;
    this.currentAngularSpeed = 0.0;
    this.tiltAngleDeg = 90.0;
    this.currentTiltDeg = 90.0;
    this.root.position.copy(this.position);
    this.root.rotation.y = this.yaw;
  }
}
