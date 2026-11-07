/**
 * BANTAY-SUNOG UI Controller & Telemetry Engine
 * Manages dashboard HUD, WASD teleop, sync modes (HW->Sim, Sim->HW, SIL Autonomous),
 * and WebSocket bridge connectivity.
 */

class UIController {
  constructor(simWorld) {
    this.sim = simWorld;
    this.robot = simWorld.robot;
    this.sensors = simWorld.sensors;
    this.fireSystem = simWorld.fireSystem;
    this.obstacleManager = simWorld.obstacleManager;
    this.assetLoader = simWorld.assetLoader;

    // Modes: 'neural' (TinyML navigation policy), 'sil' (Legacy Rule-Based FSM), 'teleop' (Manual WASD), 'sim2hw', 'hw2sim'
    this.syncMode = 'neural';
    this.navModel = (typeof NavModelRunner !== 'undefined') ? new NavModelRunner() : null;

    // Simulated FSM State (matches bantay_sunog_esp32 types.h)
    this.robotState = 'SEARCHING';
    this.stateTimer = 0;
    this.roamTurnDir = 1;

    // --- Improved Autonomous Navigation State ---
    // Zigzag scan
    this.scanPhase = 'SWEEP';     // 'SWEEP','TURN_90_OUT','SHIFT_LANE','TURN_90_IN'
    this.scanDirection = 1;        // +1 right, -1 left
    this.phaseTimer = 0;
    this.turretSweepTimer = 0;

    // Yaw-based precise turning (replaces hardcoded timers)
    this.targetYaw = null;         // radians — target heading for current turn
    this.turnTolerance = 0.08;     // ~4.5 degrees

    // Multi-behavior obstacle avoidance
    this.avoidPhase = 'NONE';      // 'NONE','BACKOUT','PIVOT','WALL_FOLLOW','ESCAPE'
    this.avoidTimer = 0;
    this.avoidTurnLeft = true;
    this.avoidMaxTimer = 0;        // absolute timeout for entire avoid sequence
    this.avoidAttempts = 0;        // escalating recovery counter

    // Stuck detection
    this.prevPosX = 0;
    this.prevPosZ = 0;
    this.stuckTimer = 0;           // accumulates time when robot hasn't moved
    this.stuckThreshold = 1.5;     // seconds before declaring stuck
    this.posCheckTimer = 0;        // sample position every N seconds

    // Optical flame proximity estimator & 2D radar mapper
    this.recentBboxAreas = [];
    this.mapper = null;

    // Teleop key state
    this.activeKeys = new Set();

    // WebSocket link to sync_server.py
    this.socket = null;
    this.wsConnected = false;

    this.bindDomElements();
    this.bindKeyboard();
    this.initWebSocket();
  }

  bindDomElements() {
    // View Buttons
    document.querySelectorAll('.view-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.view-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const mode = btn.dataset.view;
        this.sim.setViewMode(mode);
      });
    });

    // Placement Tools
    const hint = document.getElementById('placement-hint');

    document.getElementById('btn-add-fire').addEventListener('click', () => {
      this.sim.setPlacementMode('fire');
      hint.textContent = 'Click anywhere on the arena floor to ignite a fire';
      hint.style.display = 'block';
    });

    document.getElementById('btn-add-box').addEventListener('click', () => {
      this.sim.setPlacementMode('box');
      hint.textContent = 'Click anywhere on the floor to drop a crate obstacle';
      hint.style.display = 'block';
    });

    document.getElementById('btn-add-wall').addEventListener('click', () => {
      this.sim.setPlacementMode('wall');
      hint.textContent = 'Click on the floor to build a divider wall';
      hint.style.display = 'block';
    });

    document.getElementById('btn-add-cylinder').addEventListener('click', () => {
      this.sim.setPlacementMode('cylinder');
      hint.textContent = 'Click on the floor to place a barrel obstacle';
      hint.style.display = 'block';
    });

    this.sim.onPlacementComplete = () => {
      hint.style.display = 'none';
    };

    this.sim.onFireSpawned = () => {
      if (this.robotState === 'TASK_COMPLETE' || this.robotState === 'IDLE') {
        this.robotState = 'SEARCHING';
        this.scanPhase = 'SWEEP';
        this.phaseTimer = 0;
        this.stateTimer = 0;
      }
    };

    document.getElementById('btn-clear-fires').addEventListener('click', () => {
      this.fireSystem.clearAllFires();
      if (this.robotState !== 'MANUAL_TEST' && this.syncMode === 'sil') {
        this.robotState = 'TASK_COMPLETE';
        this.robot.setMotorPwm(0, 0);
      }
    });

    document.getElementById('btn-randomize-arena').addEventListener('click', () => {
      this.obstacleManager.randomizeLayout();
      this.fireSystem.clearAllFires();
      this.fireSystem.spawnFire(1.5, 1.5, 1.0);
      this.robotState = 'SEARCHING';
      this.scanPhase = 'SWEEP';
      this.phaseTimer = 0;
      this.stateTimer = 0;
    });

    document.getElementById('btn-reset-robot').addEventListener('click', () => {
      this.robot.reset(0, 0, 0);
      this.robotState = 'SEARCHING';
      this.scanPhase = 'SWEEP';
      this.phaseTimer = 0;
      this.stateTimer = 0;
    });

    // Sync Mode Radio Buttons
    document.querySelectorAll('.mode-radio').forEach((radio) => {
      radio.addEventListener('click', () => {
        document.querySelectorAll('.mode-radio').forEach(r => r.classList.remove('selected'));
        radio.classList.add('selected');
        this.syncMode = radio.dataset.mode;
        console.log(`[UI] Sync Mode Changed: ${this.syncMode}`);
      });
    });

    // 3D Model File Input
    const fileInput = document.getElementById('model-file-input');
    document.getElementById('btn-upload-model').addEventListener('click', () => {
      fileInput.click();
    });

    fileInput.addEventListener('change', (e) => {
      if (e.target.files.length > 0) {
        const file = e.target.files[0];
        this.assetLoader.loadFromFile(
          file,
          (model) => {
            alert(`Custom 3D model loaded: ${file.name}`);
          },
          (err) => {
            alert(`Error loading 3D model: ${err.message}`);
          }
        );
      }
    });

    // Teleop On-Screen Keypad
    document.querySelectorAll('.key-btn').forEach((k) => {
      const code = k.dataset.key;
      if (!code) return;

      const press = () => {
        this.activeKeys.add(code);
        k.classList.add('pressed');
      };
      const release = () => {
        this.activeKeys.delete(code);
        k.classList.remove('pressed');
      };

      k.addEventListener('mousedown', press);
      k.addEventListener('mouseup', release);
      k.addEventListener('mouseleave', release);
      k.addEventListener('touchstart', (e) => { e.preventDefault(); press(); });
      k.addEventListener('touchend', (e) => { e.preventDefault(); release(); });
    });
  }

  bindKeyboard() {
    window.addEventListener('keydown', (e) => {
      if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyU', 'KeyJ', 'KeyH', 'KeyF'].includes(e.code)) {
        this.activeKeys.add(e.code);
        const btn = document.querySelector(`.key-btn[data-key="${e.code}"]`);
        if (btn) btn.classList.add('pressed');
      }
    });

    window.addEventListener('keyup', (e) => {
      if (this.activeKeys.has(e.code)) {
        this.activeKeys.delete(e.code);
        const btn = document.querySelector(`.key-btn[data-key="${e.code}"]`);
        if (btn) btn.classList.remove('pressed');
      }
    });
  }

  initWebSocket() {
    const wsPort = 8765;
    const host = window.location.hostname || 'localhost';
    const wsUrl = `ws://${host}:${wsPort}/ws/telemetry`;
    try {
      this.socket = new WebSocket(wsUrl);

      this.socket.onopen = () => {
        this.wsConnected = true;
        this.updateSyncBadge(true);
        console.log('[WS] Connected to BANTAY-SUNOG sync server');
      };

      this.socket.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          this.handleIncomingTelemetry(msg);
        } catch (err) {
          console.error('[WS] Parse error:', err);
        }
      };

      this.socket.onclose = () => {
        this.wsConnected = false;
        this.updateSyncBadge(false);
        // Retry connection in 3 seconds
        setTimeout(() => this.initWebSocket(), 3000);
      };

      this.socket.onerror = () => {
        this.wsConnected = false;
        this.updateSyncBadge(false);
      };
    } catch (e) {
      this.updateSyncBadge(false);
    }
  }

  updateSyncBadge(connected) {
    const dot = document.getElementById('ws-status-dot');
    const label = document.getElementById('ws-status-text');
    if (dot && label) {
      if (connected) {
        dot.classList.add('connected');
        label.textContent = 'BRIDGE ONLINE';
      } else {
        dot.classList.remove('connected');
        label.textContent = 'STANDALONE (LOCAL)';
      }
    }
  }

  /**
   * Handle telemetry from physical ESP32 or external controller.
   */
  handleIncomingTelemetry(data) {
    if (data.type === 'INFERENCE_RESULT') {
      this.handleInferenceResult(data);
      return;
    }

    if (this.syncMode === 'hw2sim') {
      // Physical ESP32 drives the simulated robot!
      if (data.robot_state) this.robotState = data.robot_state;
      if (data.tilt_angle !== undefined) this.robot.setTiltAngle(data.tilt_angle);
      if (data.solenoid_active !== undefined) this.robot.solenoidActive = data.solenoid_active;
      if (data.left_pwm !== undefined && data.right_pwm !== undefined) {
        this.robot.setMotorPwm(data.left_pwm, data.right_pwm);
      }
    }
  }

  /**
   * Processes real-time detection results returned from BANTAY-SUNOG Python AI model.
   */
  handleInferenceResult(res) {
    this.aiInferenceResult = res;

    if (res.best_target) {
      // Calculate normalized bounding box area (w*h / W*H)
      let bboxArea = 0;
      if (res.best_target.bbox && res.best_target.frame_dims) {
        const frameArea = (res.best_target.frame_dims[0] || 640) * (res.best_target.frame_dims[1] || 480);
        const w = res.best_target.bbox[2] || 0;
        const h = res.best_target.bbox[3] || 0;
        bboxArea = (w * h) / frameArea;
      }

      this.recentBboxAreas.push(bboxArea);
      if (this.recentBboxAreas.length > 5) this.recentBboxAreas.shift();
      const avgBboxArea = this.recentBboxAreas.reduce((a, b) => a + b, 0) / this.recentBboxAreas.length;

      // Estimate distance in cm from bounding box if missing or 0
      let estimatedDistCm = res.best_target.distanceCm;
      if (!estimatedDistCm || estimatedDistCm <= 0) {
        const currentTelem = this.sensors ? this.sensors.getTelemetry() : null;
        if (currentTelem && currentTelem.dist_front && currentTelem.dist_front < 280) {
          estimatedDistCm = Math.round(currentTelem.dist_front);
        } else if (avgBboxArea > 0.001) {
          // Empirical pinhole optical formula: d ~ k / sqrt(area)
          estimatedDistCm = Math.round(45.0 / Math.sqrt(avgBboxArea));
        }
      }

      this.realVisionTarget = {
        confidence: res.best_target.confidence,
        x: res.best_target.x,
        y: res.best_target.y,
        bbox: res.best_target.bbox,
        frame_dims: res.best_target.frame_dims,
        bbox_area: avgBboxArea,
        distanceCm: estimatedDistCm,
        isConfirmed: (res.state === 'FIRE_CONFIRMED'),
        backend: res.backend,
        inferenceMs: res.inference_ms
      };
    } else {
      this.recentBboxAreas = [];
      this.realVisionTarget = null;
    }

    // Update PIP header with live AI metrics
    const pipHeader = document.querySelector('.pip-header');
    if (pipHeader) {
      if (res.state === 'FIRE_CONFIRMED') {
        pipHeader.innerHTML = `<span>🔥 FIRE CONFIRMED</span><span style="color: var(--accent-orange);">${res.inference_ms}ms (${res.backend})</span>`;
      } else if (res.state === 'FIRE_CANDIDATE') {
        pipHeader.innerHTML = `<span>⚠️ CANDIDATE</span><span style="color: var(--accent-yellow);">${res.inference_ms}ms</span>`;
      } else {
        pipHeader.innerHTML = `<span>📷 VIRTUAL WEBCAM</span><span style="color: var(--accent-cyan);">${res.inference_ms}ms (AI ACTIVE)</span>`;
      }
    }
  }

  /**
   * Streams virtual camera frames to the Python detection model (~10 FPS).
   */
  sendCameraFrameForInference(dt) {
    if (!this.wsConnected || !this.socket || this.socket.readyState !== WebSocket.OPEN) return;

    this.frameStreamTimer = (this.frameStreamTimer || 0) + dt;
    if (this.frameStreamTimer >= 0.09) { // ~11 FPS
      this.frameStreamTimer = 0;
      try {
        const frameData = this.sensors.pipCanvas.toDataURL('image/jpeg', 0.65);
        this.socket.send(JSON.stringify({
          type: 'FRAME_INFERENCE_REQUEST',
          image: frameData,
          timestamp: Date.now()
        }));
      } catch (e) {}
    }
  }

  /**
   * Send simulated or teleop telemetry out to ESP32 / Pi bridge.
   */
  broadcastTelemetry(telem) {
    if (this.wsConnected && this.socket && this.socket.readyState === WebSocket.OPEN) {
      telem.sync_mode = this.syncMode;
      telem.robot_state = this.robotState;
      this.socket.send(JSON.stringify(telem));
    }
  }

  /**
   * Main logic update: runs teleop or autonomous SIL state machine.
   */
  update(dt) {
    const telem = this.sensors.getTelemetry();

    // Stream Virtual Camera to Python Detection Model
    this.sendCameraFrameForInference(dt);

    // 1. Update 2D Radar Projection Mapping & Predict Avoidance Trajectory
    this.updateRadarMap(telem);

    // 2. Navigation Policies
    if (this.syncMode === 'teleop') {
      this.handleTeleopControls();
    } else if (this.syncMode === 'neural') {
      this.runNeuralSilNavigation(dt, telem);
    } else if (this.syncMode === 'sil') {
      this.runAutonomousSilFsm(dt, telem);
    } else if (this.syncMode === 'sim2hw') {
      this.handleTeleopControls();
    }

    // 3. Visualization & Telemetry Broadcast
    this.computePredictionPath(telem);
    this.updateHud(telem);
    this.broadcastTelemetry(telem);
  }

  updateRadarMap(telem) {
    if (!this.mapper && typeof UltrasonicMapper !== 'undefined') {
      this.mapper = new UltrasonicMapper('map-canvas');
    }
    if (this.mapper) {
      const target = this.realVisionTarget || telem.target;
      const hasFire = !!(telem.flame_detected || (target && target.confidence > 0.4));

      this.latestAvoidance = this.mapper.update({
        robotPos: this.robot.position,
        robotYaw: this.robot.yaw,
        distF: telem.dist_front,
        distL: telem.dist_left,
        distR: telem.dist_right,
        hasFire: hasFire,
        target: target,
        goalPos: null,
        isSpraying: !!this.robot.solenoidActive,
        robotState: this.robotState
      });
    }
  }

  handleTeleopControls() {
    let left = 0;
    let right = 0;
    const speed = 160;
    const pivot = 130;

    if (this.activeKeys.has('KeyW')) {
      left += speed;
      right += speed;
    }
    if (this.activeKeys.has('KeyS')) {
      left -= speed;
      right -= speed;
    }
    if (this.activeKeys.has('KeyA')) {
      left -= pivot;
      right += pivot;
    }
    if (this.activeKeys.has('KeyD')) {
      left += pivot;
      right -= pivot;
    }

    this.robot.setMotorPwm(left, right);

    // Tilt controls
    if (this.activeKeys.has('KeyU')) {
      this.robot.setTiltAngle(this.robot.tiltAngleDeg + 1);
    }
    if (this.activeKeys.has('KeyJ')) {
      this.robot.setTiltAngle(this.robot.tiltAngleDeg - 1);
    }
    if (this.activeKeys.has('KeyH')) {
      this.robot.setTiltAngle(90);
    }

    // Solenoid spray
    this.robot.solenoidActive = this.activeKeys.has('KeyF');
    this.robotState = this.robot.solenoidActive ? 'EXTINGUISHING' : (left || right ? 'MANUAL_TEST' : 'IDLE');
  }

  /**
   * Neural Local Navigation Policy (TinyML Sensor Policy)
   * Evaluates MLP local navigation model on continuous 7-input sensor space
   * for smooth obstacle avoidance, room sweep, and fire interception.
   */
  runNeuralSilNavigation(dt, telem) {
    if (!this.navModel && typeof NavModelRunner !== 'undefined') {
      this.navModel = new NavModelRunner();
    }
    if (!this.navModel) return;

    this.stateTimer += dt;
    const target = this.realVisionTarget || telem.target;
    const hasFire = !!(telem.flame_detected || (target && target.confidence > 0.4));

    // Convert cm to meters
    const distF = (telem.dist_front || 300.0) / 100.0;
    const distL = (telem.dist_left || 300.0) / 100.0;
    const distR = (telem.dist_right || 300.0) / 100.0;

    let targetRelYaw = 0.0;
    let targetDist = 5.0;

    if (hasFire && target) {
      // In camera normalized space (x in [0, 1], center is 0.5)
      // Camera FOV ~ 65 deg (1.13 rad), errX in [-0.5, 0.5] -> yaw error in [-0.56, +0.56] rad
      targetRelYaw = (target.x - 0.50) * 1.13;
      targetDist = (target.distanceCm && target.distanceCm > 0) ? (target.distanceCm / 100.0) : distF;

      // Turret vertical tilt servo centering
      const errY = target.y - 0.50;
      if (Math.abs(errY) > 0.04) {
        const tiltStep = errY > 0 ? -0.8 : 0.8;
        this.robot.setTiltAngle(this.robot.tiltAngleDeg + tiltStep);
      }
    } else {
      // Sweep tilt servo when searching
      this.turretSweepTimer += dt;
      const sweepAngle = 90.0 + Math.sin(this.turretSweepTimer * 1.5) * 20.0;
      this.robot.setTiltAngle(sweepAngle);
    }

    const headingRate = this.robot.currentAngularSpeed || 0.0;

    // Check standoff suppression zone (45 - 55 cm) with multi-signal proximity:
    // Standoff requires:
    // 1. Physical ultrasonic proximity: front ultrasonic distF <= 0.55m
    // OR 2. Close physical distance (distF <= 0.75m) coupled with optical confirmation (targetDist <= 0.55m or avgArea >= 0.18)
    const avgArea = (target && target.bbox_area !== undefined) ? target.bbox_area : 0;
    const isAtStandoffDistance = (distF <= 0.55) || (distF <= 0.75 && (targetDist <= 0.55 || avgArea >= 0.18));

    if (hasFire && isAtStandoffDistance) {
      this.robotState = 'EXTINGUISHING';
      this.robot.setMotorPwm(0, 0);
      this.robot.solenoidActive = true;

      if (this.stateTimer >= 2.0 && (!telem.flame_detected || this.stateTimer >= 4.0)) {
        this.robot.solenoidActive = false;
        this.robotState = 'SEARCHING';
        this.scanPhase = 'SWEEP';
        this.phaseTimer = 0;
        this.stateTimer = 0;
      }
    } else if (hasFire) {
      // Direct Fire Approach Mode
      this.robot.solenoidActive = false;
      this.robotState = 'APPROACHING';

      // 1. Stuck / Stall Detection
      this.posCheckTimer += dt;
      if (this.posCheckTimer >= 0.25) {
        this.posCheckTimer = 0;
        const dx = this.robot.position.x - this.prevPosX;
        const dz = this.robot.position.z - this.prevPosZ;
        const moved = Math.hypot(dx, dz);

        if (moved < 0.005) {
          this.stuckTimer += 0.25;
        } else {
          this.stuckTimer = 0;
        }
        this.prevPosX = this.robot.position.x;
        this.prevPosZ = this.robot.position.z;
      }

      const avoidance = this.latestAvoidance || (this.mapper ? this.mapper.findClearAvoidancePath(this.robot.position, this.robot.yaw, telem.dist_front, telem.dist_left, telem.dist_right, target) : null);

      if (avoidance && (avoidance.isObstacleAhead || avoidance.isPathLatched)) {
        targetRelYaw = avoidance.clearAngle;
      }

      // 2. Forward pass through Neural Local Navigation Model
      let { leftPwm, rightPwm } = this.navModel.step(
        distF,
        distL,
        distR,
        targetRelYaw,
        targetDist,
        hasFire,
        headingRate
      );

      // Deadband compensation & friction overcoming
      const overcomeFriction = (pwm) => {
        if (Math.abs(pwm) < 15) return 0;
        const sign = Math.sign(pwm);
        const mag = Math.min(255, Math.max(95, Math.round(Math.abs(pwm) * 1.5 + 30)));
        return sign * mag;
      };

      leftPwm = overcomeFriction(leftPwm);
      rightPwm = overcomeFriction(rightPwm);

      // Priority obstacle avoidance along bypass trajectory
      if (avoidance && (avoidance.isObstacleAhead || avoidance.isPathLatched)) {
        const rec = avoidance.recommendedPwm;
        leftPwm = rec.left;
        rightPwm = rec.right;
      } else if (avoidance && avoidance.recommendedPwm) {
        const rec = avoidance.recommendedPwm;
        leftPwm = Math.round(leftPwm * 0.40 + rec.left * 0.60);
        rightPwm = Math.round(rightPwm * 0.40 + rec.right * 0.60);
      }

      // Force recovery burst if stalled in front of an obstacle for > 0.35s
      if (this.stuckTimer >= 0.35) {
        this.stuckTimer = 0;
        const turnRight = (avoidance && avoidance.clearAngle > 0) || (distR >= distL);
        leftPwm = turnRight ? 165 : -35;
        rightPwm = turnRight ? -35 : 165;
      }

      this.robot.setMotorPwm(leftPwm, rightPwm);

    } else {
      // 100% Ego-Centric Sensor-Driven Boustrophedon Zigzag Room Sweep + Wall vs Obstacle Classifier
      this.robot.solenoidActive = false;
      this.robotState = 'SEARCHING';
      this.phaseTimer += dt;

      // Stuck detection
      this.posCheckTimer += dt;
      if (this.posCheckTimer >= 0.25) {
        this.posCheckTimer = 0;
        const dx = this.robot.position.x - this.prevPosX;
        const dz = this.robot.position.z - this.prevPosZ;
        const moved = Math.hypot(dx, dz);
        if (moved < 0.005) {
          this.stuckTimer += 0.25;
        } else {
          this.stuckTimer = 0;
        }
        this.prevPosX = this.robot.position.x;
        this.prevPosZ = this.robot.position.z;
      }

      const avoidance = this.latestAvoidance;
      const classification = (avoidance && avoidance.classification)
        ? avoidance.classification
        : { type: 'CLEAR', flankClearSide: (distR >= distL ? 'RIGHT' : 'LEFT') };

      // Wall detection: triggers 90deg lane-turn arc into adjacent sweep lane
      if (classification.type === 'WALL' && (distF < 0.68 || telem.dist_front < 68.0)) {
        if (this.scanPhase === 'SWEEP') {
          this.scanPhase = 'TURN_90_OUT';
          this.phaseTimer = 0;
          this.scanDirection = (classification.flankClearSide === 'RIGHT') ? 1 : -1;
          this.targetYaw = this.robot.yaw + this.scanDirection * (Math.PI / 2);
          this.robot.setMotorPwm(0, 0);
        }
      }

      switch (this.scanPhase) {
        case 'SWEEP':
          if (classification.type === 'OBSTACLE' && (avoidance && (avoidance.isObstacleAhead || avoidance.isPathLatched))) {
            // Discrete Obstacle (Crate / Barrel): Bypass around obstacle without abandoning sweep lane!
            const rec = avoidance.recommendedPwm;
            this.robot.setMotorPwm(rec.left, rec.right);
          } else {
            // Clear lane sweep forward progress
            if (this.phaseTimer > 5.5) {
              this.scanPhase = 'TURN_90_OUT';
              this.phaseTimer = 0;
              this.scanDirection = (distR >= distL) ? 1 : -1;
              this.targetYaw = this.robot.yaw + this.scanDirection * (Math.PI / 2);
              this.robot.setMotorPwm(0, 0);
            } else {
              // Cruising along sweep corridor
              let { leftPwm, rightPwm } = this.navModel.step(
                distF,
                distL,
                distR,
                0.0,
                5.0,
                false,
                headingRate
              );
              leftPwm = Math.round(leftPwm * 0.35 + 140 * 0.65);
              rightPwm = Math.round(rightPwm * 0.35 + 140 * 0.65);
              this.robot.setMotorPwm(leftPwm, rightPwm);
            }
          }
          break;

        case 'TURN_90_OUT': {
          const yawErr = this._normalizeAngle(this.targetYaw - this.robot.yaw);
          if (Math.abs(yawErr) < this.turnTolerance || this.phaseTimer > 3.0) {
            this.scanPhase = 'SHIFT_LANE';
            this.phaseTimer = 0;
            this.robot.setMotorPwm(0, 0);
          } else {
            const speed = 145;
            if (yawErr > 0) {
              this.robot.setMotorPwm(speed, -speed);
            } else {
              this.robot.setMotorPwm(-speed, speed);
            }
          }
          break;
        }

        case 'SHIFT_LANE':
          // Drive forward lane width spacing (~0.50m)
          this.robot.setMotorPwm(135, 135);
          if (distF < 0.40 || this.phaseTimer >= 0.85) {
            this.scanPhase = 'TURN_90_IN';
            this.phaseTimer = 0;
            this.targetYaw = this.robot.yaw + this.scanDirection * (Math.PI / 2);
            this.robot.setMotorPwm(0, 0);
          }
          break;

        case 'TURN_90_IN': {
          const yawErr2 = this._normalizeAngle(this.targetYaw - this.robot.yaw);
          if (Math.abs(yawErr2) < this.turnTolerance || this.phaseTimer > 3.0) {
            this.scanDirection = -this.scanDirection;
            this.scanPhase = 'SWEEP';
            this.phaseTimer = 0;
            this.robot.setMotorPwm(140, 140);
          } else {
            const speed = 145;
            if (yawErr2 > 0) {
              this.robot.setMotorPwm(speed, -speed);
            } else {
              this.robot.setMotorPwm(-speed, speed);
            }
          }
          break;
        }
      }

      // Stuck recovery guard if blocked in SWEEP
      if (this.stuckTimer >= 0.40 && this.scanPhase === 'SWEEP') {
        this.stuckTimer = 0;
        this.scanPhase = 'TURN_90_OUT';
        this.phaseTimer = 0;
        this.scanDirection = (distR >= distL) ? 1 : -1;
        this.targetYaw = this.robot.yaw + this.scanDirection * (Math.PI / 2);
        this.robot.setMotorPwm(0, 0);
      }
    }
  }

  /**
   * Emulates the physical ESP32 Autonomous Firefighting FSM (v2 — IMPROVED):
   *
   * SEARCHING (Zigzag Lane Sweep with yaw-based turns)
   *   -> ALIGNING -> APPROACHING -> EXTINGUISHING -> COOLDOWN
   *
   * Multi-phase OBSTACLE_AVOID with escalating recovery:
   *   BACKOUT -> PIVOT -> WALL_FOLLOW -> ESCAPE (random large turn)
   *
   * Features:
   *   - Yaw-delta precise turns (no hardcoded timers for rotation)
   *   - Stuck detection with automatic recovery
   *   - Absolute timeout on all avoidance phases
   *   - Side-proximity only triggers avoidance when driving forward
   *   - Relaxed clearance thresholds so robot doesn't loop
   */
  runAutonomousSilFsm(dt, telem) {
    this.stateTimer += dt;
    this.phaseTimer += dt;

    const target = this.realVisionTarget || telem.target;
    const distF = telem.dist_front;
    const distL = telem.dist_left;
    const distR = telem.dist_right;
    const robotYaw = this.robot.yaw;

    // -------------------------------------------------------------------------
    // 0. Stuck Detection (works across all states except EXTINGUISHING/COOLDOWN/TASK_COMPLETE)
    // -------------------------------------------------------------------------
    const isSuppression = (
      this.robotState === 'EXTINGUISHING' ||
      this.robotState === 'COOLDOWN' ||
      this.robotState === 'TASK_COMPLETE'
    );

    if (!isSuppression) {
      this.posCheckTimer += dt;
      if (this.posCheckTimer >= 0.25) {
        this.posCheckTimer = 0;
        const dx = this.robot.position.x - this.prevPosX;
        const dz = this.robot.position.z - this.prevPosZ;
        const moved = Math.hypot(dx, dz);

        if (moved < 0.005) {
          // Robot barely moved in 0.25s
          this.stuckTimer += 0.25;
        } else {
          this.stuckTimer = 0;
        }

        this.prevPosX = this.robot.position.x;
        this.prevPosZ = this.robot.position.z;
      }

      // If stuck too long, force an escape maneuver
      if (this.stuckTimer >= this.stuckThreshold && this.robotState !== 'OBSTACLE_AVOID') {
        this.robotState = 'OBSTACLE_AVOID';
        this.stateTimer = 0;
        this.avoidPhase = 'ESCAPE';
        this.avoidTimer = 0.6;
        this.avoidMaxTimer = 0;
        this.avoidAttempts++;
        this.stuckTimer = 0;
        // Pick a random large turn direction
        this.avoidTurnLeft = Math.random() > 0.5;
      } else if (this.stuckTimer >= this.stuckThreshold && this.robotState === 'OBSTACLE_AVOID') {
        // Already avoiding but still stuck — force phase escalation
        this.stuckTimer = 0;
        this.avoidPhase = 'ESCAPE';
        this.avoidTimer = 0.8 + Math.random() * 0.5;
        this.avoidTurnLeft = !this.avoidTurnLeft; // try opposite direction
        this.avoidAttempts++;
      }
    }

    // -------------------------------------------------------------------------
    // 1. High-Priority Obstacle Avoidance Trigger
    // -------------------------------------------------------------------------
    if (!isSuppression && this.robotState !== 'OBSTACLE_AVOID') {
      const isTargetingFire = (this.robotState === 'APPROACHING' || this.robotState === 'ALIGNING') && target;

      let frontTrigger = false;
      let sideTrigger = false;

      if (!isTargetingFire) {
        // Standard cruising obstacle trigger
        frontTrigger = (distF < 26.0);
        const isMovingForward = (this.robot.leftPwm > 50 && this.robot.rightPwm > 50);
        sideTrigger = isMovingForward && (distL < 12.0 || distR < 12.0);
      } else {
        // While approaching/aligning with fire: front sensor measures distance to fire.
        // Only trigger avoidance for tight side corridor pinches (< 10cm)
        const isMovingForward = (this.robot.leftPwm > 50 && this.robot.rightPwm > 50);
        sideTrigger = isMovingForward && (distL < 10.0 || distR < 10.0);
      }

      if (frontTrigger || sideTrigger) {
        this.robotState = 'OBSTACLE_AVOID';
        this.stateTimer = 0;
        this.avoidMaxTimer = 0;
        this.robot.solenoidActive = false;

        if (distF < 18.0) {
          // Very close — back out first
          this.avoidPhase = 'BACKOUT';
          this.avoidTimer = 0.50;
        } else {
          // Moderate distance — pivot along the predicted clear angle from mapping!
          this.avoidPhase = 'PIVOT';
          this.avoidTimer = 0;
          const avoidance = this.latestAvoidance;
          let turnAngle = (avoidance && avoidance.isObstacleAhead)
            ? avoidance.clearAngle
            : ((distL >= distR ? -1 : 1) * (Math.PI / 2));
          this.targetYaw = this._normalizeAngle(robotYaw + turnAngle);
          this.avoidTurnLeft = (turnAngle < 0);
        }
      }
    }

    // -------------------------------------------------------------------------
    // 2. FSM Execution
    // -------------------------------------------------------------------------
    switch (this.robotState) {
      case 'OBSTACLE_AVOID':
        this.robot.solenoidActive = false;
        this.avoidMaxTimer += dt;

        // Absolute timeout: if avoidance takes > 5s, force escape
        if (this.avoidMaxTimer > 5.0 && this.avoidPhase !== 'ESCAPE') {
          this.avoidPhase = 'ESCAPE';
          this.avoidTimer = 0.7;
          this.avoidTurnLeft = !this.avoidTurnLeft;
          this.avoidAttempts++;
        }
        // Ultra-timeout: if > 8s total in avoidance, just resume scanning
        if (this.avoidMaxTimer > 8.0) {
          this.robotState = 'SEARCHING';
          this.scanPhase = 'SWEEP';
          this.phaseTimer = 0;
          this.avoidPhase = 'NONE';
          this.avoidAttempts = 0;
          this.stuckTimer = 0;
          break;
        }

        switch (this.avoidPhase) {
          case 'BACKOUT':
            // Phase 1: Reverse out
            this.robot.setMotorPwm(-140, -140);
            this.avoidTimer -= dt;
            if (this.avoidTimer <= 0) {
              // Transition to PIVOT — set a target yaw
              this.avoidPhase = 'PIVOT';
              this.avoidTimer = 0;
              const turnAngle = (Math.PI / 2) + (Math.random() * 0.4 - 0.2);
              if (distL >= distR) {
                this.targetYaw = robotYaw - turnAngle;
                this.avoidTurnLeft = true;
              } else {
                this.targetYaw = robotYaw + turnAngle;
                this.avoidTurnLeft = false;
              }
            }
            break;

          case 'PIVOT': {
            // Phase 2: Yaw-based pivot toward target heading
            const yawErr = this._normalizeAngle(this.targetYaw - robotYaw);
            if (Math.abs(yawErr) < this.turnTolerance) {
              // Turn complete — check if path is clear
              if (distF > 35.0) {
                // Clear! Resume scanning
                this.robotState = 'SEARCHING';
                this.scanPhase = 'SWEEP';
                this.phaseTimer = 0;
                this.avoidPhase = 'NONE';
                this.avoidAttempts = 0;
              } else {
                // Still blocked ahead — escalate to wall follow
                this.avoidPhase = 'WALL_FOLLOW';
                this.avoidTimer = 2.0; // follow for up to 2s
              }
            } else {
              // Pivot in the direction of yaw error
              const speed = 145;
              if (yawErr > 0) {
                this.robot.setMotorPwm(speed, -speed); // turn right (+yaw)
              } else {
                this.robot.setMotorPwm(-speed, speed); // turn left (-yaw)
              }
            }
            break;
          }

          case 'WALL_FOLLOW':
            // Phase 3: Drive forward while wall-following (curved path away from obstacle)
            this.avoidTimer -= dt;
            if (distF < 20.0) {
              // Front blocked again while wall-following — pivot more
              this.avoidPhase = 'PIVOT';
              const turnAngle = (Math.PI / 3) + Math.random() * 0.3;
              if (distL >= distR) {
                this.targetYaw = robotYaw - turnAngle;
              } else {
                this.targetYaw = robotYaw + turnAngle;
              }
              break;
            }
            if (this.avoidTimer <= 0 || distF > 60.0) {
              // Wall-follow done — resume scanning
              this.robotState = 'SEARCHING';
              this.scanPhase = 'SWEEP';
              this.phaseTimer = 0;
              this.avoidPhase = 'NONE';
              this.avoidAttempts = 0;
              break;
            }
            // Drive forward with slight turn away from closer side
            if (distL < distR) {
              this.robot.setMotorPwm(140, 110); // curve right
            } else {
              this.robot.setMotorPwm(110, 140); // curve left
            }
            break;

          case 'ESCAPE':
            // Phase 4: Emergency — large reverse + random big turn
            this.avoidTimer -= dt;
            if (this.avoidTimer > 0.35) {
              // First half: reverse
              this.robot.setMotorPwm(-155, -155);
            } else if (this.avoidTimer > 0) {
              // Second half: aggressive pivot
              const speed = 160;
              if (this.avoidTurnLeft) {
                this.robot.setMotorPwm(-speed, speed);
              } else {
                this.robot.setMotorPwm(speed, -speed);
              }
            } else {
              // Escape done — check clearance
              if (distF > 30.0) {
                this.robotState = 'SEARCHING';
                this.scanPhase = 'SWEEP';
                this.phaseTimer = 0;
                this.avoidPhase = 'NONE';
                this.avoidAttempts = 0;
              } else {
                // Still blocked — try another escape in opposite direction
                this.avoidPhase = 'ESCAPE';
                this.avoidTimer = 0.9;
                this.avoidTurnLeft = !this.avoidTurnLeft;
              }
            }
            break;
        }
        break;

      case 'SEARCHING':
      case 'IDLE':
        this.robot.solenoidActive = false;

        // Active Turret Vertical Scanning (sweep smoothly between 76deg and 104deg)
        this.turretSweepTimer += dt * 1.8;
        const tiltAngle = 90.0 + Math.sin(this.turretSweepTimer) * 14.0;
        this.robot.setTiltAngle(tiltAngle);

        // Immediate fire target detection check
        if (target && target.confidence > 0.65) {
          this.robotState = 'ALIGNING';
          this.stateTimer = 0;
          this.robot.setMotorPwm(0, 0);
          break;
        }

        // --- Structured Zigzag Scanning Pattern with Yaw-Based Turns ---
        switch (this.scanPhase) {
          case 'SWEEP':
            // Drive forward along current scan lane
            this.robot.setMotorPwm(140, 140);

            // Reached lane end / approaching perimeter boundary
            if (distF < 40.0 || this.phaseTimer > 5.0) {
              this.scanPhase = 'TURN_90_OUT';
              this.phaseTimer = 0;
              // Set target yaw for a precise 90deg turn
              this.targetYaw = robotYaw + this.scanDirection * (Math.PI / 2);
              this.robot.setMotorPwm(0, 0);
            }
            break;

          case 'TURN_90_OUT': {
            // Yaw-based 90deg pivot towards next lane
            const yawErr = this._normalizeAngle(this.targetYaw - robotYaw);
            if (Math.abs(yawErr) < this.turnTolerance) {
              // Turn complete — shift lane
              this.scanPhase = 'SHIFT_LANE';
              this.phaseTimer = 0;
            } else {
              const speed = 135;
              if (yawErr > 0) {
                this.robot.setMotorPwm(speed, -speed);
              } else {
                this.robot.setMotorPwm(-speed, speed);
              }
            }
            // Safety: if turn takes too long (>3s), force next phase
            if (this.phaseTimer > 3.0) {
              this.scanPhase = 'SHIFT_LANE';
              this.phaseTimer = 0;
            }
            break;
          }

          case 'SHIFT_LANE':
            // Drive forward by lane spacing (~0.5m)
            this.robot.setMotorPwm(135, 135);

            if (distF < 30.0 || this.phaseTimer >= 0.80) {
              this.scanPhase = 'TURN_90_IN';
              this.phaseTimer = 0;
              // Second 90deg turn to face reverse direction
              this.targetYaw = robotYaw + this.scanDirection * (Math.PI / 2);
              this.robot.setMotorPwm(0, 0);
            }
            break;

          case 'TURN_90_IN': {
            // Yaw-based second 90deg pivot
            const yawErr2 = this._normalizeAngle(this.targetYaw - robotYaw);
            if (Math.abs(yawErr2) < this.turnTolerance) {
              // Turn complete — reverse scan direction and sweep
              this.scanDirection = -this.scanDirection;
              this.scanPhase = 'SWEEP';
              this.phaseTimer = 0;
            } else {
              const speed = 135;
              if (yawErr2 > 0) {
                this.robot.setMotorPwm(speed, -speed);
              } else {
                this.robot.setMotorPwm(-speed, speed);
              }
            }
            // Safety timeout
            if (this.phaseTimer > 3.0) {
              this.scanDirection = -this.scanDirection;
              this.scanPhase = 'SWEEP';
              this.phaseTimer = 0;
            }
            break;
          }
        }
        break;

      case 'ALIGNING':
        this.robot.solenoidActive = false;
        if (!target) {
          if (this.stateTimer > 2.0) {
            this.robotState = 'SEARCHING';
            this.scanPhase = 'SWEEP';
            this.phaseTimer = 0;
          }
          this.robot.setMotorPwm(0, 0);
          break;
        }

        // Visual Servoing: Center normalized flame target (0.50, 0.50)
        const errX = target.x - 0.50;
        const errY = target.y - 0.50;

        let alignL = 0;
        let alignR = 0;
        if (Math.abs(errX) > 0.05) {
          // Proportional turn speed for smoother servoing
          const turnSpeed = Math.min(150, Math.max(90, Math.abs(errX) * 250));
          if (errX > 0) {
            alignL = turnSpeed;
            alignR = -turnSpeed;
          } else {
            alignL = -turnSpeed;
            alignR = turnSpeed;
          }
        }

        if (Math.abs(errY) > 0.04) {
          const tiltStep = errY > 0 ? -0.8 : 0.8;
          this.robot.setTiltAngle(this.robot.tiltAngleDeg + tiltStep);
        }

        this.robot.setMotorPwm(alignL, alignR);

        if (Math.abs(errX) <= 0.05 && Math.abs(errY) <= 0.06) {
          this.robotState = 'APPROACHING';
          this.stateTimer = 0;
        }
        break;

      case 'APPROACHING':
        this.robot.solenoidActive = false;
        if (!target) {
          if (this.stateTimer > 2.0) {
            this.robotState = 'SEARCHING';
            this.scanPhase = 'SWEEP';
            this.phaseTimer = 0;
            this.stateTimer = 0;
          }
          break;
        }

        // Standoff spray distance: stop at 45-55cm (extinguisher spray reaches 1.35m)
        const targetDist = (target.distanceCm && target.distanceCm > 0) ? target.distanceCm : distF;
        const avgArea = (target && target.bbox_area !== undefined) ? target.bbox_area : 0;
        if (targetDist <= 55.0 || distF <= 50.0 || avgArea >= 0.10) {
          this.robot.setMotorPwm(0, 0);
          this.robotState = 'EXTINGUISHING';
          this.stateTimer = 0;
        } else {
          // Centering trim during approach
          const trim = (target.x - 0.50) * 85;
          const speed = 135;
          this.robot.setMotorPwm(speed + trim, speed - trim);
        }
        break;

      case 'EXTINGUISHING':
        this.robot.setMotorPwm(0, 0);
        this.robot.solenoidActive = true;

        // Keep spraying until flame is extinguished (minimum 1.8s, max 3.5s)
        if (this.stateTimer >= 1.8) {
          if (!telem.flame_detected || this.stateTimer >= 3.5) {
            this.robot.solenoidActive = false;
            this.robotState = 'COOLDOWN';
            this.stateTimer = 0;
          }
        }
        break;

      case 'COOLDOWN':
        this.robot.solenoidActive = false;
        this.robot.setMotorPwm(0, 0);
        if (this.stateTimer > 1.6) {
          if (telem.flame_detected) {
            // Flame still burning here — re-align and spray again
            this.robotState = 'ALIGNING';
            this.stateTimer = 0;
          } else {
            // Check if any other active fires exist in the arena
            const activeFires = this.fireSystem.getActiveFires();
            if (activeFires.length > 0) {
              // More fires detected in the building — resume systematic scan
              this.robotState = 'SEARCHING';
              this.scanPhase = 'SWEEP';
              this.phaseTimer = 0;
              this.stateTimer = 0;
            } else {
              // ALL FIRES EXTINGUISHED! Mission Accomplished!
              this.robotState = 'TASK_COMPLETE';
              this.stateTimer = 0;
              this.robot.setMotorPwm(0, 0);
              this.robot.setTiltAngle(90.0);
            }
          }
        }
        break;

      case 'TASK_COMPLETE':
        this.robot.setMotorPwm(0, 0);
        this.robot.solenoidActive = false;
        this.robot.setTiltAngle(90.0);

        // If a new fire is ignited or detected, wake up and extinguish!
        const remainingFires = this.fireSystem.getActiveFires();
        if (remainingFires.length > 0 || (target && target.confidence > 0.65)) {
          this.robotState = 'SEARCHING';
          this.scanPhase = 'SWEEP';
          this.phaseTimer = 0;
          this.stateTimer = 0;
        }
        break;
    }
  }

  /**
   * Normalize angle to [-PI, +PI] range.
   */
  _normalizeAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  /**
   * Computes 3D projected prediction trajectory in front of the robot.
   */
  computePredictionPath(telem) {
    if (!this.sim.predictionPath) return;

    // 1. Predictive Avoidance Path from Mapping: when obstacle is ahead, path is latched, or committing to bypass corridor!
    const avoidance = this.latestAvoidance;
    if (avoidance && (avoidance.isObstacleAhead || avoidance.isPathLatched || (avoidance.activeCorridor && avoidance.activeCorridor !== 'NONE')) && avoidance.worldPath && avoidance.worldPath.length > 1) {
      const avoid3DPoints = avoidance.worldPath.map(p => new THREE.Vector3(p.x, 0.015, p.z));
      this.sim.predictionPath.update(avoid3DPoints, 'avoid');
      return;
    }

    // 2. Continuous Neural Trajectory: render smooth mapper trajectory towards goal destination
    if (this.syncMode === 'neural' && avoidance && avoidance.worldPath && avoidance.worldPath.length > 1) {
      const path3D = avoidance.worldPath.map(p => new THREE.Vector3(p.x, 0.015, p.z));
      this.sim.predictionPath.update(path3D, 'scan');
      return;
    }

    const points = [];
    const robotPos = this.robot.position.clone();
    const yaw = this.robot.yaw;

    if (this.robotState === 'SEARCHING' || this.robotState === 'IDLE') {
      const fwd = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw)).normalize();
      const distF = Math.max(0.35, (telem.dist_front / 100.0) - 0.40);

      points.push(robotPos.clone());

      const numFwd = 10;
      for (let i = 1; i <= numFwd; i++) {
        const step = (distF / numFwd) * i;
        points.push(robotPos.clone().add(fwd.clone().multiplyScalar(step)));
      }

      // 2. Predict the Zigzag Turn & Shift Lane
      const turnPt = points[points.length - 1].clone();
      const turnYaw = yaw + (this.scanDirection * Math.PI / 2);
      const sideDir = new THREE.Vector3(Math.sin(turnYaw), 0, Math.cos(turnYaw)).normalize();

      const numSide = 6;
      const shiftDist = 0.55;
      for (let i = 1; i <= numSide; i++) {
        const step = (shiftDist / numSide) * i;
        points.push(turnPt.clone().add(sideDir.clone().multiplyScalar(step)));
      }

      // 3. Predict the Reverse Return Track
      const endShiftPt = points[points.length - 1].clone();
      const returnYaw = yaw + Math.PI;
      const returnDir = new THREE.Vector3(Math.sin(returnYaw), 0, Math.cos(returnYaw)).normalize();

      const numReturn = 8;
      const returnDist = 0.90;
      for (let i = 1; i <= numReturn; i++) {
        const step = (returnDist / numReturn) * i;
        points.push(endShiftPt.clone().add(returnDir.clone().multiplyScalar(step)));
      }

      this.sim.predictionPath.update(points, 'scan');

    } else if (this.robotState === 'OBSTACLE_AVOID') {
      points.push(robotPos.clone());
      const fwd = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw)).normalize();

      if (this.avoidPhase === 'BACKOUT' || this.avoidPhase === 'ESCAPE') {
        // Show reverse path
        for (let i = 1; i <= 8; i++) {
          points.push(robotPos.clone().sub(fwd.clone().multiplyScalar(i * 0.06)));
        }
        // Then show arc indicating upcoming turn
        const lastPt = points[points.length - 1].clone();
        const turnSign = this.avoidTurnLeft ? -1 : 1;
        for (let i = 1; i <= 8; i++) {
          const arcYaw = yaw + turnSign * (i * 0.15);
          const arcDir = new THREE.Vector3(Math.sin(arcYaw), 0, Math.cos(arcYaw)).normalize();
          points.push(lastPt.clone().add(arcDir.multiplyScalar(i * 0.06)));
        }
      } else if (this.avoidPhase === 'WALL_FOLLOW') {
        // Show curved forward path
        const curveBias = (telem.dist_left < telem.dist_right) ? 0.06 : -0.06;
        for (let i = 1; i <= 14; i++) {
          const curveYaw = yaw + curveBias * i;
          const curveDir = new THREE.Vector3(Math.sin(curveYaw), 0, Math.cos(curveYaw)).normalize();
          points.push(robotPos.clone().add(curveDir.multiplyScalar(i * 0.08)));
        }
      } else {
        // PIVOT: show turning arc
        const turnSign = this.avoidTurnLeft ? -1 : 1;
        for (let i = 1; i <= 14; i++) {
          const arcYaw = yaw + turnSign * (i * 0.12);
          const arcDir = new THREE.Vector3(Math.sin(arcYaw), 0, Math.cos(arcYaw)).normalize();
          points.push(robotPos.clone().add(arcDir.multiplyScalar(i * 0.05)));
        }
      }

      this.sim.predictionPath.update(points, 'avoid');

    } else if (this.robotState === 'ALIGNING' || this.robotState === 'APPROACHING') {
      points.push(robotPos.clone());
      const fwd = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw)).normalize();

      const fires = this.fireSystem.getActiveFires();
      if (fires.length > 0) {
        let closest = fires[0];
        let minDist = 999;
        fires.forEach(f => {
          const d = robotPos.distanceTo(f.position);
          if (d < minDist) { minDist = d; closest = f; }
        });

        const targetPos = closest.position.clone();
        const steps = 14;
        for (let i = 1; i <= steps; i++) {
          const t = i / steps;
          points.push(robotPos.clone().lerp(targetPos, t));
        }
      } else {
        for (let i = 1; i <= 12; i++) {
          points.push(robotPos.clone().add(fwd.clone().multiplyScalar(i * 0.15)));
        }
      }

      this.sim.predictionPath.update(points, 'target');

    } else {
      this.sim.predictionPath.update(null);
    }
  }

  updateHud(telem) {
    // Top State Badge
    const stateBadge = document.getElementById('hud-state-badge');
    if (stateBadge) {
      if (this.robotState === 'TASK_COMPLETE') {
        stateBadge.textContent = 'MISSION COMPLETED';
      } else {
        stateBadge.textContent = this.robotState.replace('_', ' ');
      }
      stateBadge.className = `state-badge ${this.robotState.toLowerCase()}`;
    }

    // Ultrasonic Radar Bars
    const updateGauge = (id, valCm) => {
      const bar = document.getElementById(`gauge-bar-${id}`);
      const txt = document.getElementById(`gauge-txt-${id}`);
      if (bar && txt) {
        const pct = Math.min(100, Math.max(0, (valCm / 300.0) * 100));
        bar.style.width = `${pct}%`;
        txt.textContent = `${valCm.toFixed(1)} cm`;

        if (valCm < 25.0) {
          bar.style.backgroundColor = 'var(--accent-red)';
        } else if (valCm < 40.0) {
          bar.style.backgroundColor = 'var(--accent-yellow)';
        } else {
          bar.style.backgroundColor = 'var(--accent-green)';
        }
      }
    };

    updateGauge('front', telem.dist_front);
    updateGauge('left', telem.dist_left);
    updateGauge('right', telem.dist_right);

    // Flame sensor indicator
    const flameDot = document.getElementById('flame-indicator-dot');
    const flameTxt = document.getElementById('flame-indicator-txt');
    if (flameDot && flameTxt) {
      if (telem.flame_detected) {
        flameDot.classList.add('active-flame');
        flameTxt.textContent = 'DETECTED';
        flameTxt.style.color = 'var(--accent-orange)';
      } else {
        flameDot.classList.remove('active-flame');
        flameTxt.textContent = 'CLEAR';
        flameTxt.style.color = '#fff';
      }
    }

    // Solenoid spray indicator
    const solDot = document.getElementById('solenoid-indicator-dot');
    const solTxt = document.getElementById('solenoid-indicator-txt');
    if (solDot && solTxt) {
      if (telem.solenoid_active) {
        solDot.classList.add('active-solenoid');
        solTxt.textContent = 'SPRAYING';
        solTxt.style.color = 'var(--accent-cyan)';
      } else {
        solDot.classList.remove('active-solenoid');
        solTxt.textContent = 'OFF';
        solTxt.style.color = '#fff';
      }
    }

    // Motor PWM & Tilt
    const pwmTxt = document.getElementById('hud-pwm-txt');
    if (pwmTxt) pwmTxt.textContent = `L:${telem.left_pwm} | R:${telem.right_pwm}`;

    const tiltTxt = document.getElementById('hud-tilt-txt');
    if (tiltTxt) tiltTxt.textContent = `${telem.tilt_angle}°`;

    // Camera PIP Bounding Box
    const bboxEl = document.getElementById('pip-bbox');
    const target = this.realVisionTarget || telem.target;
    if (bboxEl) {
      if (target && target.bbox) {
        const [bx, by, bw, bh] = target.bbox;
        const frameW = target.frame_dims ? target.frame_dims[0] : 640;
        const frameH = target.frame_dims ? target.frame_dims[1] : 480;
        const scaleX = 250 / frameW;
        const scaleY = 187 / frameH;

        bboxEl.style.display = 'block';
        bboxEl.style.left = `${Math.max(0, bx * scaleX)}px`;
        bboxEl.style.top = `${Math.max(0, by * scaleY)}px`;
        bboxEl.style.width = `${Math.max(22, bw * scaleX)}px`;
        bboxEl.style.height = `${Math.max(22, bh * scaleY)}px`;

        const label = bboxEl.querySelector('.pip-bbox-label');
        if (label) {
          const confPct = Math.round(target.confidence * 100);
          label.textContent = target.backend ? `FIRE ${confPct}% (${target.backend})` : `FIRE ${confPct}%`;
          label.style.background = target.isConfirmed ? 'var(--accent-green)' : 'var(--accent-orange)';
        }
      } else {
        bboxEl.style.display = 'none';
      }
    }
  }
}
