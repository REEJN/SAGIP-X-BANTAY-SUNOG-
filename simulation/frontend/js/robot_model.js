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
    this.initSensorHotspotsAndLabels();
    this.scene.add(this.root);
  }

  initProceduralModel() {
    // -------------------------------------------------------------------------
    // 1. EMBER Hexagonal Rescue Bot Chassis & Body Enclosure
    // -------------------------------------------------------------------------
    // Main Hexagonal Outer Body Shell (#d9382b Rescue Red)
    const hexRadius = 0.16; // ~300mm diameter
    const hexHeight = 0.11;
    const hexGeo = new THREE.CylinderGeometry(hexRadius, hexRadius, hexHeight, 6);
    hexGeo.rotateY(Math.PI / 6); // Front flat face with sensor mouth
    const hexMat = new THREE.MeshStandardMaterial({
      color: 0xd9382b, // EMBER Rescue Red
      metalness: 0.35,
      roughness: 0.35
    });
    const hexMesh = new THREE.Mesh(hexGeo, hexMat);
    hexMesh.position.y = 0.08;
    hexMesh.castShadow = true;
    hexMesh.receiveShadow = true;
    this.proceduralGroup.add(hexMesh);

    // Hazard Stripes (9mm Yellow #ffc21a + 3mm White accent ring around base)
    const stripeGeo = new THREE.CylinderGeometry(hexRadius + 0.002, hexRadius + 0.002, 0.016, 6);
    stripeGeo.rotateY(Math.PI / 6);
    const stripeMat = new THREE.MeshStandardMaterial({
      color: 0xffc21a, // Yellow hazard
      roughness: 0.3,
      metalness: 0.2
    });
    const stripeMesh = new THREE.Mesh(stripeGeo, stripeMat);
    stripeMesh.position.y = 0.038;
    this.proceduralGroup.add(stripeMesh);

    // Dark Top Deck Plate
    const deckPlateGeo = new THREE.CylinderGeometry(hexRadius - 0.005, hexRadius - 0.005, 0.008, 6);
    deckPlateGeo.rotateY(Math.PI / 6);
    const deckPlateMat = new THREE.MeshStandardMaterial({
      color: 0x181e29,
      roughness: 0.5,
      metalness: 0.6
    });
    const deckPlateMesh = new THREE.Mesh(deckPlateGeo, deckPlateMat);
    deckPlateMesh.position.y = 0.136;
    this.proceduralGroup.add(deckPlateMesh);

    // Glowing LED Round Eyes (48mm round rings on front wall)
    const eyeRingGeo = new THREE.RingGeometry(0.012, 0.022, 24);
    const eyeRingMat = new THREE.MeshBasicMaterial({
      color: 0x00f0ff,
      side: THREE.DoubleSide
    });
    const leftEyeRing = new THREE.Mesh(eyeRingGeo, eyeRingMat);
    leftEyeRing.position.set(-0.048, 0.088, 0.141);
    this.proceduralGroup.add(leftEyeRing);

    const rightEyeRing = new THREE.Mesh(eyeRingGeo, eyeRingMat);
    rightEyeRing.position.set(0.048, 0.088, 0.141);
    this.proceduralGroup.add(rightEyeRing);

    // White Pressurized Water Tank Cylinder (white Ø80 x 229 mm horizontal on rear deck)
    const tankRadius = 0.038;
    const tankLength = 0.20;
    const tankGeo = new THREE.CylinderGeometry(tankRadius, tankRadius, tankLength, 24);
    tankGeo.rotateZ(Math.PI / 2);
    const tankMat = new THREE.MeshStandardMaterial({
      color: 0xf8fafc,
      roughness: 0.2,
      metalness: 0.15
    });
    const tankMesh = new THREE.Mesh(tankGeo, tankMat);
    tankMesh.position.set(0, 0.182, -0.05);
    tankMesh.castShadow = true;
    this.proceduralGroup.add(tankMesh);

    // Tank Mounting Brackets
    const tankBracketMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.7 });
    [-0.065, 0.065].forEach(x => {
      const bGeo = new THREE.BoxGeometry(0.016, 0.04, 0.06);
      const bMesh = new THREE.Mesh(bGeo, tankBracketMat);
      bMesh.position.set(x, 0.16, -0.05);
      this.proceduralGroup.add(bMesh);
    });

    // Yellow Nozzle Hose Tube (routed from tank to turret)
    const hoseCurve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(-0.085, 0.182, -0.05),
      new THREE.Vector3(-0.095, 0.185, 0.02),
      new THREE.Vector3(-0.05, 0.175, 0.07),
      new THREE.Vector3(0, 0.16, 0.09)
    ]);
    const hoseGeo = new THREE.TubeGeometry(hoseCurve, 16, 0.009, 8, false);
    const hoseMat = new THREE.MeshStandardMaterial({
      color: 0xf59e0b, // Yellow hose
      roughness: 0.6,
      metalness: 0.2
    });
    const hoseMesh = new THREE.Mesh(hoseGeo, hoseMat);
    this.proceduralGroup.add(hoseMesh);

    // Top Amber Beacon Dome (pulses on alert / fire detection)
    const beaconBaseGeo = new THREE.CylinderGeometry(0.018, 0.022, 0.01, 16);
    const beaconBaseMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, metalness: 0.8 });
    const beaconBase = new THREE.Mesh(beaconBaseGeo, beaconBaseMat);
    beaconBase.position.set(0.075, 0.145, 0.045);
    this.proceduralGroup.add(beaconBase);

    const beaconDomeGeo = new THREE.SphereGeometry(0.015, 16, 16, 0, Math.PI * 2, 0, Math.PI / 2);
    this.beaconMat = new THREE.MeshStandardMaterial({
      color: 0xffb703,
      emissive: 0xff9900,
      emissiveIntensity: 0.6,
      roughness: 0.1,
      transparent: true,
      opacity: 0.92
    });
    this.beaconDome = new THREE.Mesh(beaconDomeGeo, this.beaconMat);
    this.beaconDome.position.set(0.075, 0.150, 0.045);
    this.proceduralGroup.add(this.beaconDome);

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

  initSensorHotspotsAndLabels() {
    // -------------------------------------------------------------------------
    // Sensor Hotspots Definition (Coordinates relative to robot root)
    // -------------------------------------------------------------------------
    this.sensorHotspots = {
      front_sonar: { pos: new THREE.Vector3(0, 0.08, 0.17), name: 'Front HC-SR04 (0°)' },
      left_sonar: { pos: new THREE.Vector3(-0.11, 0.08, 0.14), name: 'Left HC-SR04 (+45°)' },
      right_sonar: { pos: new THREE.Vector3(0.11, 0.08, 0.14), name: 'Right HC-SR04 (-45°)' },
      imu: { pos: new THREE.Vector3(0, 0.09, 0), name: 'MPU-6050 6-Axis IMU' },
      flame_ir: { pos: new THREE.Vector3(0, 0.17, 0.10), name: 'LM393 IR Flame Sensor' },
      camera: { pos: new THREE.Vector3(0, 0.15, 0.11), name: 'Vision AI Optical Camera' },
      turret: { pos: new THREE.Vector3(0, 0.13, 0.08), name: 'MG90S Pitch Tilt Gimbal' },
      solenoid: { pos: new THREE.Vector3(0, 0.19, -0.05), name: 'Suppression Canister & MG996R' },
      motors: { pos: new THREE.Vector3(0, 0.05, 0), name: 'TB6612FNG 4WD Skid Drive' },
      compute: { pos: new THREE.Vector3(0, 0.14, 0.01), name: 'Raspberry Pi 3B & ESP32' }
    };

    // -------------------------------------------------------------------------
    // 3D Floating Sprite Labels Group
    // -------------------------------------------------------------------------
    this.labelsGroup = new THREE.Group();
    this.labelsGroup.visible = false;
    this.root.add(this.labelsGroup);

    const labelDefs = [
      { key: 'front_sonar', num: '1', text: 'HC-SR04 FRONT 0°', offset: new THREE.Vector3(0, 0.07, 0.04) },
      { key: 'left_sonar', num: '2', text: 'HC-SR04 LEFT +45°', offset: new THREE.Vector3(-0.06, 0.06, 0.03) },
      { key: 'right_sonar', num: '3', text: 'HC-SR04 RIGHT -45°', offset: new THREE.Vector3(0.06, 0.06, 0.03) },
      { key: 'imu', num: '4', text: 'MPU-6050 6-AXIS IMU', offset: new THREE.Vector3(0, 0.09, 0) },
      { key: 'flame_ir', num: '5', text: 'LM393 OPTICAL IR', offset: new THREE.Vector3(0.07, 0.05, 0.02) },
      { key: 'camera', num: '6', text: 'OPTICAL CAM / YOLO', offset: new THREE.Vector3(-0.07, 0.05, 0.02) },
      { key: 'turret', num: '7', text: 'MG90S TILT GIMBAL', offset: new THREE.Vector3(0, 0.06, -0.03) },
      { key: 'solenoid', num: '8', text: 'MG996R EXTINGUISHER', offset: new THREE.Vector3(0, 0.07, -0.02) },
      { key: 'motors', num: '9', text: 'TB6612FNG 4WD DRIVE', offset: new THREE.Vector3(0, -0.04, 0) },
      { key: 'compute', num: '10', text: 'ESP32 & PI 3B CORE', offset: new THREE.Vector3(0, 0.08, -0.05) }
    ];

    labelDefs.forEach(def => {
      const hotspot = this.sensorHotspots[def.key];
      if (hotspot) {
        const sprite = this.createLabelSprite(def.text, def.num);
        sprite.position.copy(hotspot.pos).add(def.offset);
        this.labelsGroup.add(sprite);

        // Thin connector line from hotspot to sprite
        const lineGeo = new THREE.BufferGeometry().setFromPoints([
          hotspot.pos,
          sprite.position
        ]);
        const lineMat = new THREE.LineBasicMaterial({ color: 0x00f0ff, transparent: true, opacity: 0.6 });
        const line = new THREE.Line(lineGeo, lineMat);
        this.labelsGroup.add(line);
      }
    });

    // -------------------------------------------------------------------------
    // Spotlight Highlight Marker Mesh (Animates when a sensor is clicked)
    // -------------------------------------------------------------------------
    const ringGeo = new THREE.RingGeometry(0.022, 0.028, 24);
    ringGeo.rotateX(-Math.PI / 2);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x00ff88,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.0
    });
    this.spotlightRing = new THREE.Mesh(ringGeo, ringMat);
    this.root.add(this.spotlightRing);
    this.spotlightAnim = 0;
  }

  createLabelSprite(text, numberTag) {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    // Translucent dark box
    ctx.fillStyle = 'rgba(7, 11, 18, 0.90)';
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 2.5;
    if (ctx.roundRect) {
      ctx.beginPath();
      ctx.roundRect(4, 4, 248, 56, 10);
      ctx.fill();
      ctx.stroke();
    } else {
      ctx.fillRect(4, 4, 248, 56);
      ctx.strokeRect(4, 4, 248, 56);
    }

    // Badge circle
    ctx.fillStyle = '#00f0ff';
    ctx.beginPath();
    ctx.arc(30, 32, 16, 0, Math.PI * 2);
    ctx.fill();

    // Badge number
    ctx.fillStyle = '#000';
    ctx.font = 'bold 18px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(numberTag, 30, 32);

    // Text label
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 15px monospace';
    ctx.textAlign = 'left';
    ctx.fillText(text, 54, 32);

    const texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(0.16, 0.04, 1);
    return sprite;
  }

  setRenderMode(mode) {
    this.currentRenderMode = mode;

    if (mode === 'solid') {
      // Restore standard colored materials
      this.proceduralGroup.traverse((child) => {
        if (child.isMesh && child.userData.origMaterial) {
          child.material = child.userData.origMaterial;
        }
      });
      if (this.labelsGroup) this.labelsGroup.visible = false;
    } else if (mode === 'hologram') {
      // Tactical emerald/cyan wireframe + glowing edge style (matching reference image)
      this.proceduralGroup.traverse((child) => {
        if (child.isMesh) {
          if (!child.userData.origMaterial) {
            child.userData.origMaterial = child.material;
          }
          child.material = new THREE.MeshBasicMaterial({
            color: 0x00f0ff,
            wireframe: true,
            transparent: true,
            opacity: 0.55
          });
        }
      });
      if (this.labelsGroup) this.labelsGroup.visible = false;
    } else if (mode === 'labels') {
      // Keep solid or hologram and show 3D labels
      this.proceduralGroup.traverse((child) => {
        if (child.isMesh && child.userData.origMaterial) {
          child.material = child.userData.origMaterial;
        }
      });
      if (this.labelsGroup) this.labelsGroup.visible = true;
    }
  }

  highlightHotspot(key) {
    const spot = this.sensorHotspots ? this.sensorHotspots[key] : null;
    if (spot && this.spotlightRing) {
      this.spotlightRing.position.copy(spot.pos);
      this.spotlightRing.material.opacity = 1.0;
      this.spotlightAnim = 1.0;
    }
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

    // Pulse Top Amber Beacon Dome on alert / fire spray
    if (this.beaconMat) {
      const pulseSpeed = this.solenoidActive ? 14.0 : 4.5;
      const intensity = 0.35 + 0.65 * (0.5 + 0.5 * Math.sin(Date.now() * 0.001 * pulseSpeed));
      this.beaconMat.emissiveIntensity = intensity;
    }

    // Spotlight Ring decay animation
    if (this.spotlightRing && this.spotlightAnim > 0) {
      this.spotlightAnim = Math.max(0, this.spotlightAnim - (dt * 0.4));
      this.spotlightRing.material.opacity = this.spotlightAnim;
      const s = 1.0 + (1.0 - this.spotlightAnim) * 0.5;
      this.spotlightRing.scale.set(s, s, s);
    }
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
