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
    this.sim2hwPolicy = 'neural'; // 'neural', 'sil', 'teleop' for Sim2HW dispatch
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

    // Dashboard & Page Navigation
    this.activePage = 'dashboard';
    this.piLogs = [];
    this.espLogs = [];
    this.maxLogs = 300;
    this.missionStartTime = Date.now();
    this.firesExtinguishedCount = 0;
    this.telemetryHistory = [];
    this.activeChartChannel = 'sensors';
    this.chartTimer = 0;
    this.minimapTimer = 0;

    // Physical Hardware Connection Status
    this.physicalHardwareConnected = false;
    this.physicalHardwarePort = null;
    this.physicalHardwareBaud = 115200;
    this.lastPhysicalTelem = null;
    this.lastPhysicalTelemTime = 0;

    if (typeof history !== 'undefined' && 'scrollRestoration' in history) {
      history.scrollRestoration = 'manual';
    }

    this.bindDomElements();
    this.bindKeyboard();
    this.initWebSocket();
    this.updatePhysicalHardwareStatus(false, null, 115200);
    this.renderPiLogs();
    this.renderEspLogs();
  }

  bindDomElements() {
    // Physical Hardware Port Scan Trigger
    document.getElementById('btn-scan-hw')?.addEventListener('click', () => {
      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        this.socket.send(JSON.stringify({ type: 'SCAN_PORTS' }));
        const btn = document.getElementById('btn-scan-hw');
        if (btn) {
          btn.textContent = '⏳ Scanning...';
          setTimeout(() => { if (btn) btn.textContent = '🔄 Scan Ports'; }, 1500);
        }
      }
    });

    // Navigation Tabs (Main Dashboard vs 3D Simulation Sub-Page)
    document.querySelectorAll('.nav-tab').forEach((tab) => {
      tab.addEventListener('click', () => {
        const page = tab.dataset.page;
        this.switchPage(page);
      });
    });

    // Global E-Stop Button
    document.getElementById('btn-global-estop')?.addEventListener('click', () => {
      this.emergencyStop();
    });

    // Terminal Pi Controls
    document.getElementById('btn-clear-pi')?.addEventListener('click', () => {
      this.piLogs = [];
      this.renderPiLogs();
    });

    document.getElementById('btn-copy-pi')?.addEventListener('click', () => {
      const text = this.piLogs.map(e => `[${new Date(e.timestamp * 1000).toLocaleTimeString()}] [${e.source}] ${e.message}`).join('\n');
      navigator.clipboard.writeText(text).then(() => alert('Pi console logs copied to clipboard!'));
    });

    document.getElementById('sel-filter-pi')?.addEventListener('change', () => {
      this.renderPiLogs();
    });

    // Terminal ESP32 Controls
    document.getElementById('btn-clear-esp')?.addEventListener('click', () => {
      this.espLogs = [];
      this.renderEspLogs();
    });

    document.getElementById('btn-copy-esp')?.addEventListener('click', () => {
      const text = this.espLogs.map(e => `[${new Date(e.timestamp * 1000).toLocaleTimeString()}] ${e.message}`).join('\n');
      navigator.clipboard.writeText(text).then(() => alert('ESP32 console logs copied to clipboard!'));
    });

    document.getElementById('inp-filter-esp')?.addEventListener('input', () => {
      this.renderEspLogs();
    });

    // Chart Channel Pills
    document.querySelectorAll('.chart-pill').forEach((pill) => {
      pill.addEventListener('click', () => {
        document.querySelectorAll('.chart-pill').forEach(p => p.classList.remove('active'));
        pill.classList.add('active');
        this.activeChartChannel = pill.dataset.channel;
        this.renderTelemetryChart();
      });
    });

    // Bench Test Buttons
    document.querySelectorAll('.bench-btn[data-cmd]').forEach((btn) => {
      btn.addEventListener('click', () => {
        const cmd = btn.dataset.cmd;
        this.dispatchBenchCommand(cmd);
      });
    });

    // Dashboard Policy Selectors
    const btnDashNeural = document.getElementById('btn-dash-policy-neural');
    const btnDashSil = document.getElementById('btn-dash-policy-sil');
    if (btnDashNeural && btnDashSil) {
      btnDashNeural.addEventListener('click', () => {
        this.syncMode = 'neural';
        btnDashNeural.style.background = 'var(--accent-cyan)';
        btnDashNeural.style.color = '#000';
        btnDashSil.style.background = 'rgba(255,255,255,0.05)';
        btnDashSil.style.color = '#fff';
        document.querySelectorAll('.mode-radio').forEach(r => {
          r.classList.toggle('selected', r.dataset.mode === 'neural');
        });
      });

      btnDashSil.addEventListener('click', () => {
        this.syncMode = 'sil';
        btnDashSil.style.background = 'var(--accent-cyan)';
        btnDashSil.style.color = '#000';
        btnDashNeural.style.background = 'rgba(255,255,255,0.05)';
        btnDashNeural.style.color = '#fff';
        document.querySelectorAll('.mode-radio').forEach(r => {
          r.classList.toggle('selected', r.dataset.mode === 'sil');
        });
      });
    }

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
      if (this.mapper) this.mapper.resetMap();
      this.robotState = 'SEARCHING';
      this.scanPhase = 'SWEEP';
      this.phaseTimer = 0;
      this.stateTimer = 0;
    });

    document.getElementById('btn-reset-robot').addEventListener('click', () => {
      this.robot.reset(0, 0, 0);
      if (this.mapper) this.mapper.resetMap();
      this.robotState = 'SEARCHING';
      this.scanPhase = 'SWEEP';
      this.phaseTimer = 0;
      this.stateTimer = 0;
    });

    // Environment Layout Selector
    const layoutSelect = document.getElementById('select-arena-layout');
    if (layoutSelect) {
      layoutSelect.addEventListener('change', (e) => {
        const layout = e.target.value;
        this.sim.setLayout(layout);
        if (this.mapper) this.mapper.resetMap();
        this.robotState = 'SEARCHING';
        this.scanPhase = 'SWEEP';
        this.phaseTimer = 0;
        this.stateTimer = 0;
      });
    }

    // Obstacle Inspector Card Wiring
    const inspectorCard = document.getElementById('obstacle-inspector-card');
    const inpObsId = document.getElementById('inspector-obs-id');
    const inpX = document.getElementById('inp-obs-x');
    const inpZ = document.getElementById('inp-obs-z');
    const boxDims = document.getElementById('inspector-box-dims');
    const cylDims = document.getElementById('inspector-cyl-dims');
    const inpW = document.getElementById('inp-obs-w');
    const inpD = document.getElementById('inp-obs-d');
    const inpR = document.getElementById('inp-obs-r');
    const inpH = document.getElementById('inp-obs-h');
    const inpRot = document.getElementById('inp-obs-rot');
    const txtRot = document.getElementById('txt-obs-rot');
    const btnDup = document.getElementById('btn-obs-duplicate');
    const btnDel = document.getElementById('btn-obs-delete');
    const btnDeselect = document.getElementById('btn-obs-deselect');

    const updateInspectorUI = (mesh) => {
      if (!mesh || !mesh.userData) return;
      const d = mesh.userData;
      if (inpObsId) inpObsId.textContent = `#${d.id} ${d.type.toUpperCase()}`;
      if (inpX) inpX.value = mesh.position.x.toFixed(2);
      if (inpZ) inpZ.value = mesh.position.z.toFixed(2);

      const deg = Math.round(((mesh.rotation.y * 180 / Math.PI) % 360 + 360) % 360);
      if (inpRot) inpRot.value = deg;
      if (txtRot) txtRot.textContent = `${deg}°`;

      if (d.type === 'cylinder') {
        if (boxDims) boxDims.style.display = 'none';
        if (cylDims) cylDims.style.display = 'block';
        if (inpR) inpR.value = (d.radius || 0.35).toFixed(2);
        if (inpH) inpH.value = (d.height || 0.8).toFixed(2);
      } else {
        if (boxDims) boxDims.style.display = 'flex';
        if (cylDims) cylDims.style.display = 'none';
        if (inpW) inpW.value = (d.width || 0.6).toFixed(2);
        if (inpD) inpD.value = (d.depth || 0.6).toFixed(2);
        if (inpH) inpH.value = (d.height || 0.5).toFixed(2);
      }
    };

    this.sim.onObstacleSelected = (mesh) => {
      if (inspectorCard) inspectorCard.style.display = 'block';
      updateInspectorUI(mesh);
    };

    this.sim.onObstacleDeselected = () => {
      if (inspectorCard) inspectorCard.style.display = 'none';
    };

    this.sim.onObstacleModified = (mesh) => {
      updateInspectorUI(mesh);
    };

    if (inpX && inpZ) {
      const onPosChange = () => {
        if (this.sim.selectedObstacle) {
          const id = this.sim.selectedObstacle.userData.id;
          const x = parseFloat(inpX.value) || 0;
          const z = parseFloat(inpZ.value) || 0;
          this.obstacleManager.updateObstaclePosition(id, x, z);
          if (this.sim.selectionHelper) this.sim.selectionHelper.update();
        }
      };
      inpX.addEventListener('input', onPosChange);
      inpZ.addEventListener('input', onPosChange);
    }

    const onDimsChange = () => {
      if (this.sim.selectedObstacle) {
        const id = this.sim.selectedObstacle.userData.id;
        const dims = {
          width: inpW ? parseFloat(inpW.value) : undefined,
          depth: inpD ? parseFloat(inpD.value) : undefined,
          height: inpH ? parseFloat(inpH.value) : undefined,
          radius: inpR ? parseFloat(inpR.value) : undefined
        };
        this.obstacleManager.updateObstacleDimensions(id, dims);
        if (this.sim.selectionHelper) this.sim.selectionHelper.update();
      }
    };
    if (inpW) inpW.addEventListener('input', onDimsChange);
    if (inpD) inpD.addEventListener('input', onDimsChange);
    if (inpR) inpR.addEventListener('input', onDimsChange);
    if (inpH) inpH.addEventListener('input', onDimsChange);

    if (inpRot) {
      inpRot.addEventListener('input', () => {
        if (this.sim.selectedObstacle) {
          const id = this.sim.selectedObstacle.userData.id;
          const deg = parseFloat(inpRot.value) || 0;
          const rad = deg * Math.PI / 180;
          if (txtRot) txtRot.textContent = `${deg}°`;
          this.obstacleManager.updateObstacleRotation(id, rad);
          if (this.sim.selectionHelper) this.sim.selectionHelper.update();
        }
      });
    }

    if (btnDup) {
      btnDup.addEventListener('click', () => {
        if (this.sim.selectedObstacle) {
          const newMesh = this.obstacleManager.duplicateObstacle(this.sim.selectedObstacle.userData.id);
          if (newMesh) {
            this.sim.selectObstacle(newMesh);
          }
        }
      });
    }

    if (btnDel) {
      btnDel.addEventListener('click', () => {
        if (this.sim.selectedObstacle) {
          this.obstacleManager.removeObstacle(this.sim.selectedObstacle.userData.id);
          this.sim.deselectObstacle();
        }
      });
    }

    if (btnDeselect) {
      btnDeselect.addEventListener('click', () => {
        this.sim.deselectObstacle();
      });
    }

    // Sync Mode Radio Buttons
    const sim2hwGroup = document.getElementById('sim2hw-policy-group');
    document.querySelectorAll('.mode-radio').forEach((radio) => {
      radio.addEventListener('click', () => {
        document.querySelectorAll('.mode-radio').forEach(r => r.classList.remove('selected'));
        radio.classList.add('selected');
        this.syncMode = radio.dataset.mode;
        if (sim2hwGroup) {
          sim2hwGroup.style.display = (this.syncMode === 'sim2hw') ? 'block' : 'none';
        }
        console.log(`[UI] Sync Mode Changed: ${this.syncMode}`);
      });
    });

    // Sim2HW Policy Pills
    document.querySelectorAll('.policy-pill').forEach((pill) => {
      pill.addEventListener('click', (e) => {
        e.stopPropagation();
        document.querySelectorAll('.policy-pill').forEach(p => {
          p.classList.remove('active');
          p.style.background = 'rgba(255,255,255,0.1)';
          p.style.color = '#fff';
        });
        pill.classList.add('active');
        pill.style.background = 'var(--accent-cyan)';
        pill.style.color = '#000';
        this.sim2hwPolicy = pill.dataset.policy;
        console.log(`[UI] Sim2HW Policy Changed: ${this.sim2hwPolicy}`);
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
    const keyMap = {
      'ArrowUp': 'KeyW',
      'ArrowDown': 'KeyS',
      'ArrowLeft': 'KeyA',
      'ArrowRight': 'KeyD'
    };

    window.addEventListener('keydown', (e) => {
      if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;

      const code = keyMap[e.code] || e.code;
      if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyU', 'KeyJ', 'KeyH', 'KeyF'].includes(code)) {
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(e.code)) {
          e.preventDefault();
        }
        this.activeKeys.add(code);
        const btn = document.querySelector(`.key-btn[data-key="${code}"]`);
        if (btn) btn.classList.add('pressed');
      }
    });

    window.addEventListener('keyup', (e) => {
      if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;

      const code = keyMap[e.code] || e.code;
      if (this.activeKeys.has(code)) {
        this.activeKeys.delete(code);
        const btn = document.querySelector(`.key-btn[data-key="${code}"]`);
        if (btn) btn.classList.remove('pressed');
      }
    });

    window.addEventListener('blur', () => {
      this.activeKeys.clear();
      document.querySelectorAll('.key-btn').forEach(btn => btn.classList.remove('pressed'));
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

  handleIncomingTelemetry(data) {
    if (data.type === 'HARDWARE_STATUS') {
      this.updatePhysicalHardwareStatus(data.connected, data.port, data.baudrate, data.available_ports, data.pi_metrics);
      return;
    }
    if (data.type === 'PI_LOG' && data.data) {
      this.appendPiLog(data.data);
      return;
    }
    if (data.type === 'ESP32_LOG' && data.data) {
      this.appendEspLog(data.data);
      return;
    }
    if (data.type === 'INFERENCE_RESULT') {
      this.handleInferenceResult(data);
      return;
    }

    // Physical Hardware Telemetry: ONLY ingest real physical packets for the dashboard
    if (data.is_physical === true) {
      this.lastPhysicalTelem = data;
      this.lastPhysicalTelemTime = Date.now();
      if (!this.physicalHardwareConnected) {
        this.physicalHardwareConnected = true;
        this.updatePhysicalHardwareStatus(true, this.physicalHardwarePort || data.port, this.physicalHardwareBaud);
      }
      this.updateDashboardHud(0.016, data);
      this.updateDashboardCamera(0.016, data);
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

  updatePhysicalHardwareStatus(connected, port, baud, availablePorts, piMetrics) {
    this.physicalHardwareConnected = !!connected;
    if (port) {
      // Normalize port name to eliminate twitching from alternating port strings
      if (port.includes('192.168.137') || port.includes('Wi-Fi')) {
        this.physicalHardwarePort = 'Raspberry Pi 3B (192.168.137.200 -> /dev/serial0)';
      } else {
        this.physicalHardwarePort = port;
      }
    }
    if (baud) this.physicalHardwareBaud = baud;

    const isWifiBridge = this.physicalHardwarePort && (this.physicalHardwarePort.includes('Wi-Fi') || this.physicalHardwarePort.includes('192.168.137'));
    
    // Header Pill
    const dot = document.getElementById('hw-status-dot');
    const label = document.getElementById('hw-status-text');
    if (dot && label) {
      if (this.physicalHardwareConnected) {
        dot.classList.add('connected');
        label.textContent = isWifiBridge ? 'CONNECTED (Pi Wi-Fi -> /dev/serial0)' : `CONNECTED (${this.physicalHardwarePort || 'UART'})`;
      } else {
        dot.classList.remove('connected');
        label.textContent = 'DISCONNECTED';
      }
    }

    // KPI Card 4
    const kpiStatus = document.getElementById('kpi-hw-status');
    const kpiPort = document.getElementById('kpi-hw-port');
    const kpiBaud = document.getElementById('kpi-hw-baud');
    if (kpiStatus) {
      if (this.physicalHardwareConnected) {
        kpiStatus.textContent = isWifiBridge ? 'CONNECTED (WI-FI)' : 'CONNECTED';
        kpiStatus.style.color = 'var(--accent-green)';
      } else {
        kpiStatus.textContent = 'DISCONNECTED';
        kpiStatus.style.color = 'var(--accent-red)';
      }
    }
    if (kpiPort) {
      if (this.physicalHardwareConnected) {
        kpiPort.textContent = isWifiBridge ? 'Pi 3B (192.168.137.200)' : (this.physicalHardwarePort || 'Serial Port');
      } else {
        kpiPort.textContent = 'None Detected';
      }
    }
    if (kpiBaud) {
      kpiBaud.textContent = isWifiBridge ? '115200 (UART RX/TX)' : `${this.physicalHardwareBaud || 115200}`;
    }

    // Hardware Alert Banner
    const banner = document.getElementById('hw-alert-banner');
    const bTitle = document.getElementById('hw-banner-title');
    const bSub = document.getElementById('hw-banner-sub');
    if (banner && bTitle && bSub) {
      if (this.physicalHardwareConnected) {
        banner.className = 'hw-banner online';
        const icon = banner.querySelector('.hw-banner-icon');
        if (icon) icon.textContent = '🟢';
        bTitle.textContent = isWifiBridge ? 'ESP32 & RASPBERRY PI 3B LINKED OVER WI-FI' : 'PHYSICAL HARDWARE CONNECTED';
        bSub.textContent = isWifiBridge 
          ? 'Streaming live physical telemetry: ESP32 [UART Pins 8/10] ➔ Raspberry Pi 3B [/dev/serial0] ➔ Laptop Wi-Fi [192.168.137.200]'
          : `Streaming live physical telemetry from ESP32 on ${this.physicalHardwarePort || 'UART'} @ ${this.physicalHardwareBaud} 8-N-1`;
      } else {
        banner.className = 'hw-banner offline';
        const icon = banner.querySelector('.hw-banner-icon');
        if (icon) icon.textContent = '⚠️';
        bTitle.textContent = 'PHYSICAL HARDWARE DISCONNECTED';
        bSub.textContent = 'Dashboard readings are reserved strictly for physical hardware. Connect your ESP32-WROOM via USB/UART (/dev/ttyAMA0 or COM port) to view live sensor telemetry.';
      }
    }

    // Real Pi Metrics (gathered from backend, never hardcoded fake static numbers)
    if (piMetrics) {
      if (document.getElementById('pi-stat-cpu')) document.getElementById('pi-stat-cpu').textContent = piMetrics.cpu || '--';
      if (document.getElementById('pi-stat-temp')) document.getElementById('pi-stat-temp').textContent = piMetrics.temp || '--';
      if (document.getElementById('pi-stat-ram')) document.getElementById('pi-stat-ram').textContent = piMetrics.ram || '--';
    } else if (!this.physicalHardwareConnected) {
      if (document.getElementById('pi-stat-cpu')) document.getElementById('pi-stat-cpu').textContent = '--';
      if (document.getElementById('pi-stat-temp')) document.getElementById('pi-stat-temp').textContent = '--';
      if (document.getElementById('pi-stat-ram')) document.getElementById('pi-stat-ram').textContent = '--';
    }

    // Pi Console Status LED
    const piLed = document.getElementById('pi-led');
    if (piLed) {
      if (this.physicalHardwareConnected) {
        piLed.style.background = 'var(--accent-green)';
        piLed.style.boxShadow = '0 0 8px var(--accent-green)';
      } else {
        piLed.style.background = 'var(--accent-red)';
        piLed.style.boxShadow = '0 0 8px var(--accent-red)';
      }
    }

    // ESP32 Console Footer
    const espSafety = document.getElementById('esp-stat-safety');
    if (espSafety) {
      if (this.physicalHardwareConnected) {
        espSafety.textContent = 'HARDWARE LINKED';
        espSafety.style.color = 'var(--accent-green)';
      } else {
        espSafety.textContent = 'OFFLINE (WAITING)';
        espSafety.style.color = 'var(--accent-red)';
      }
    }

    if (!this.physicalHardwareConnected) {
      this.updateDashboardHud(0.016, null);
      this.updateDashboardCamera(0.016, null);
    }
  }

  /**
   * Dedicated Dashboard-only update when on the Operations Dashboard page.
   * Simulation physics & 3D rendering are completely STOPPED.
   */
  updateDashboardOnly(dt) {
    if (this.physicalHardwareConnected && (Date.now() - this.lastPhysicalTelemTime > 3500)) {
      this.physicalHardwareConnected = false;
      this.updatePhysicalHardwareStatus(false, null, 115200);
    }

    if (this.physicalHardwareConnected && this.lastPhysicalTelem) {
      this.updateDashboardHud(dt, this.lastPhysicalTelem);
      this.updateDashboardCamera(dt, this.lastPhysicalTelem);
    } else {
      this.updateDashboardHud(dt, null);
      this.updateDashboardCamera(dt, null);
    }
  }

  /**
   * Renders the Vision AI Camera Feed on the Operations Dashboard.
   * Displays optical reticle, live pitch indicators, flame bounding box, and target lock HUD.
   */
  updateDashboardCamera(dt, telem) {
    const canvas = document.getElementById('dashboard-cam-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;

    // 1. Draw camera feed from virtual camera or dark optical canvas
    if (this.sensors && this.sensors.pipCanvas) {
      try {
        if (this.syncMode !== 'dashboard' || !this.physicalHardwareConnected) {
          this.sensors.renderPip();
        }
        ctx.drawImage(this.sensors.pipCanvas, 0, 0, w, h);
      } catch (e) {
        ctx.fillStyle = '#060a12';
        ctx.fillRect(0, 0, w, h);
      }
    } else {
      ctx.fillStyle = '#060a12';
      ctx.fillRect(0, 0, w, h);
    }

    // 2. Optical Reticle & Pitch Scale
    const tilt = (telem && telem.tilt_angle !== undefined) ? telem.tilt_angle : (this.robot ? this.robot.currentTiltDeg : 90);
    const pitchOffset = ((tilt - 90) / 45) * (h * 0.22);

    // Center pitch line
    ctx.strokeStyle = 'rgba(0, 210, 255, 0.25)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(w * 0.35, h * 0.5 - pitchOffset);
    ctx.lineTo(w * 0.65, h * 0.5 - pitchOffset);
    ctx.stroke();

    // Corner brackets
    const bLen = 22;
    ctx.strokeStyle = 'rgba(0, 210, 255, 0.45)';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(14, 14 + bLen); ctx.lineTo(14, 14); ctx.lineTo(14 + bLen, 14); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(w - 14 - bLen, 14); ctx.lineTo(w - 14, 14); ctx.lineTo(w - 14, 14 + bLen); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(14, h - 14 - bLen); ctx.lineTo(14, h - 14); ctx.lineTo(14 + bLen, h - 14); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(w - 14 - bLen, h - 14); ctx.lineTo(w - 14, h - 14); ctx.lineTo(w - 14, h - 14 - bLen); ctx.stroke();

    // 3. Update HUD Stamps & Target Lock Overlays
    const pitchEl = document.getElementById('dash-hud-pitch');
    const irEl = document.getElementById('dash-hud-ir');
    const standoffEl = document.getElementById('dash-hud-standoff');
    const confEl = document.getElementById('dash-hud-conf');
    const statusLineEl = document.getElementById('dash-hud-status-line');
    const badgeEl = document.getElementById('vision-state-badge');
    const lockPill = document.getElementById('dash-cam-lock-pill');
    const crosshairEl = document.getElementById('dash-vision-crosshair');
    const bboxEl = document.getElementById('dash-vision-bbox');
    const confTagEl = document.getElementById('dash-vision-bbox-conf');

    const hasFlame = telem ? !!telem.flame_detected : (this.realVisionTarget && this.realVisionTarget.isConfirmed);
    const standoffM = telem ? (telem.dist_front / 100).toFixed(2) : '--';

    if (pitchEl) pitchEl.textContent = `${Math.round(tilt)}°`;
    if (irEl) {
      irEl.textContent = hasFlame ? 'DETECTED' : 'CLEAR';
      irEl.style.color = hasFlame ? 'var(--accent-red)' : 'var(--accent-green)';
    }
    if (standoffEl) standoffEl.textContent = standoffM !== '--' ? `${standoffM} m` : '-- m';
    if (confEl) confEl.textContent = hasFlame ? '100%' : '0%';

    if (hasFlame) {
      if (badgeEl) {
        badgeEl.textContent = '🔥 FLAME LOCK';
        badgeEl.style.background = 'rgba(255, 51, 75, 0.2)';
        badgeEl.style.color = 'var(--accent-red)';
      }
      if (lockPill) {
        lockPill.textContent = 'TARGET LOCKED';
        lockPill.style.color = 'var(--accent-red)';
      }
      if (statusLineEl) {
        statusLineEl.textContent = 'OPTICAL TARGET LOCKED [FIRE SUPPRESSION READY]';
        statusLineEl.style.color = 'var(--accent-red)';
      }
      if (crosshairEl) crosshairEl.classList.add('locked');

      if (bboxEl) {
        bboxEl.style.display = 'block';
        const boxW = Math.max(90, Math.min(180, (w * 0.28)));
        const boxH = Math.max(100, Math.min(210, (h * 0.44)));
        const boxL = (w - boxW) / 2;
        const boxT = Math.max(10, Math.min(h - boxH - 10, (h - boxH) / 2 - pitchOffset * 0.6));
        bboxEl.style.left = `${(boxL / w) * 100}%`;
        bboxEl.style.top = `${(boxT / h) * 100}%`;
        bboxEl.style.width = `${(boxW / w) * 100}%`;
        bboxEl.style.height = `${(boxH / h) * 100}%`;
      }
      if (confTagEl) confTagEl.textContent = '100%';
    } else {
      if (badgeEl) {
        badgeEl.textContent = 'PATROL SCAN';
        badgeEl.style.background = 'rgba(0, 210, 255, 0.15)';
        badgeEl.style.color = 'var(--accent-cyan)';
      }
      if (lockPill) {
        lockPill.textContent = 'SEARCHING';
        lockPill.style.color = 'var(--accent-yellow)';
      }
      if (statusLineEl) {
        statusLineEl.textContent = 'AI VISION SENSOR: OPTICAL SCANNING SECTOR';
        statusLineEl.style.color = 'var(--accent-cyan)';
      }
      if (crosshairEl) crosshairEl.classList.remove('locked');
      if (bboxEl) bboxEl.style.display = 'none';
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
      if (this.sim2hwPolicy === 'neural') {
        this.runNeuralSilNavigation(dt, telem);
      } else if (this.sim2hwPolicy === 'sil') {
        this.runAutonomousSilFsm(dt, telem);
      } else {
        this.handleTeleopControls();
      }
    }

    // 3. Visualization & Telemetry Broadcast (Simulation Sub-Page HUD only)
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

    // Tilt controls with smooth rate limiting
    if (this.activeKeys.has('KeyU')) {
      this.robot.setTiltAngle(Math.min(135, this.robot.tiltAngleDeg + 1.2));
    }
    if (this.activeKeys.has('KeyJ')) {
      this.robot.setTiltAngle(Math.max(45, this.robot.tiltAngleDeg - 1.2));
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
      const isLooping = avoidance && avoidance.isCirculating;

      if (isLooping) {
        this.robotState = 'LOOP_ESCAPE';
        if (avoidance && avoidance.recommendedPwm) {
          this.robot.setMotorPwm(avoidance.recommendedPwm.left, avoidance.recommendedPwm.right);
        } else {
          this.robot.setMotorPwm(150, 150);
        }
        return;
      }

      const classification = (avoidance && avoidance.classification)
        ? avoidance.classification
        : { type: 'CLEAR', flankClearSide: (distR >= distL ? 'RIGHT' : 'LEFT') };

      // Wall & Dead-End Detection:
      // Uses 10-12 inch collision trigger (~0.30m / 30cm) to guarantee sharp corner & wall standoff
      if (classification.type === 'WALL' && (distF < 0.32 || telem.dist_front < 32.0)) {
        if (this.scanPhase === 'ROOM_EXPLORE' || this.scanPhase === 'SWEEP') {
          this.phaseTimer = 0;
          const isDeadEnd = avoidance && (avoidance.isDeadEnd || (avoidance.frontierGoal && avoidance.frontierGoal.isCurrentRoomExplored));

          if (isDeadEnd) {
            // Dead-End or Room Fully Explored: Execute 180deg U-Turn Backtrack to find undiscovered room!
            this.scanPhase = 'DEADEND_TURN_BACK';
            const turnSign = (distR >= distL) ? 1 : -1;
            this.targetYaw = this._normalizeAngle(this.robot.yaw + turnSign * Math.PI); // Exact 180° turnaround
            this.robot.setMotorPwm(0, 0);
          } else {
            // Uncompleted room: contour turn along clearer flank
            this.scanPhase = 'CONTOUR_TURN';
            let chosenDir = (classification.flankClearSide === 'RIGHT') ? 1 : -1;
            if (avoidance && avoidance.frontierGoal) {
              const dx = avoidance.frontierGoal.x - this.robot.position.x;
              const dz = avoidance.frontierGoal.z - this.robot.position.z;
              const goalBearing = Math.atan2(dx, dz);
              let diff = this._normalizeAngle(goalBearing - this.robot.yaw);
              if (Math.abs(diff) > 0.2) chosenDir = (diff >= 0) ? 1 : -1;
            }
            this.scanDirection = chosenDir;
            this.targetYaw = this._normalizeAngle(this.robot.yaw + this.scanDirection * (Math.PI / 2));
            this.robot.setMotorPwm(0, 0);
          }
        }
      }

      switch (this.scanPhase) {
        case 'ROOM_EXPLORE':
        case 'SWEEP':
          if (avoidance && (avoidance.isObstacleAhead || avoidance.isPathLatched)) {
            // Priority obstacle & wall avoidance along safe standoff trajectory!
            const rec = avoidance.recommendedPwm;
            this.robot.setMotorPwm(rec.left, rec.right);
          } else if (avoidance && avoidance.frontierGoal && Math.abs(avoidance.clearAngle) > 0.15) {
            // Actively steering along predicted path towards room frontier area!
            const rec = avoidance.recommendedPwm;
            this.robot.setMotorPwm(rec.left, rec.right);
          } else {
            // Cruising along room corridor adapting directly to mapped undiscovered frontier
            let targetYawBias = 0.0;
            let targetDistM = 5.0;
            if (avoidance && avoidance.frontierGoal) {
              const dx = avoidance.frontierGoal.x - this.robot.position.x;
              const dz = avoidance.frontierGoal.z - this.robot.position.z;
              const goalBearing = Math.atan2(dx, dz);
              targetYawBias = this._normalizeAngle(goalBearing - this.robot.yaw);
              targetDistM = Math.hypot(dx, dz);
            } else if (avoidance) {
              targetYawBias = avoidance.clearAngle;
            }

            let { leftPwm, rightPwm } = this.navModel.step(
              distF,
              distL,
              distR,
              targetYawBias,
              targetDistM,
              false,
              headingRate
            );
            leftPwm = Math.round(leftPwm * 0.45 + 140 * 0.55);
            rightPwm = Math.round(rightPwm * 0.45 + 140 * 0.55);
            this.robot.setMotorPwm(leftPwm, rightPwm);
          }
          break;

        case 'DEADEND_TURN_BACK': {
          // Intentional 180deg U-Turn: Turn back away from dead-end wall toward exit doorway
          const yawErr = this._normalizeAngle(this.targetYaw - this.robot.yaw);
          if (Math.abs(yawErr) < this.turnTolerance || this.phaseTimer > 2.5) {
            // Turnaround complete! Facing exit corridor / doorway -> transit to new room!
            this.scanPhase = 'ROOM_TRANSIT';
            this.phaseTimer = 0;
            this.robot.setMotorPwm(140, 140);
          } else {
            const speed = 155;
            if (yawErr > 0) {
              this.robot.setMotorPwm(speed, -speed);
            } else {
              this.robot.setMotorPwm(-speed, speed);
            }
          }
          break;
        }

        case 'CONTOUR_TURN': {
          const yawErr = this._normalizeAngle(this.targetYaw - this.robot.yaw);
          if (Math.abs(yawErr) < this.turnTolerance || distF > 0.45 || this.phaseTimer > 2.2) {
            this.scanPhase = 'ROOM_EXPLORE';
            this.phaseTimer = 0;
            this.robot.setMotorPwm(140, 140);
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

        case 'ROOM_TRANSIT': {
          // Transiting through doorway into undiscovered room
          if (avoidance && (avoidance.isObstacleAhead || avoidance.isPathLatched)) {
            const rec = avoidance.recommendedPwm;
            this.robot.setMotorPwm(rec.left, rec.right);
          } else if (avoidance && avoidance.frontierGoal) {
            const rec = avoidance.recommendedPwm;
            this.robot.setMotorPwm(rec.left, rec.right);

            // Reached new room with fresh local frontiers!
            if (!avoidance.frontierGoal.isCurrentRoomExplored && avoidance.frontierGoal.dist < 2.2) {
              this.scanPhase = 'ROOM_EXPLORE';
              this.phaseTimer = 0;
            }
          } else {
            this.robot.setMotorPwm(140, 140);
          }

          if (this.phaseTimer > 4.5) {
            this.scanPhase = 'ROOM_EXPLORE';
            this.phaseTimer = 0;
          }
          break;
        }
      }

      // Stuck recovery guard if stalled in any phase
      if (this.stuckTimer >= 0.40) {
        this.stuckTimer = 0;
        this.scanPhase = 'DEADEND_TURN_BACK';
        this.phaseTimer = 0;
        this.targetYaw = this._normalizeAngle(this.robot.yaw + Math.PI);
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

    // 2. Continuous Trajectory: render smooth mapper trajectory towards goal destination
    if (avoidance && avoidance.worldPath && avoidance.worldPath.length > 1) {
      const path3D = avoidance.worldPath.map(p => new THREE.Vector3(p.x, 0.015, p.z));
      this.sim.predictionPath.update(path3D, 'scan');
      return;
    }

    const points = [];
    const robotPos = this.robot.position.clone();
    const yaw = this.robot.yaw;

    if (this.robotState === 'SEARCHING' || this.robotState === 'IDLE') {
      const fwd = new THREE.Vector3(Math.sin(yaw), 0, Math.cos(yaw)).normalize();
      // Forward path clamped strictly by physical distance to obstacle/wall so it never penetrates
      const rawDistM = (telem.dist_front / 100.0);
      const safeDistM = Math.max(0.12, Math.min(1.4, rawDistM - 0.20));

      points.push(robotPos.clone());
      const numFwd = 8;
      for (let i = 1; i <= numFwd; i++) {
        const step = (safeDistM / numFwd) * i;
        points.push(robotPos.clone().add(fwd.clone().multiplyScalar(step)));
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

  // =========================================================================
  // MAIN OPERATIONS DASHBOARD LOGIC & HARDWARE TERMINALS
  // =========================================================================

  switchPage(page) {
    this.activePage = page;
    document.querySelectorAll('.nav-tab').forEach((t) => {
      t.classList.toggle('active', t.dataset.page === page);
    });

    const pageDash = document.getElementById('page-dashboard');
    const pageSim = document.getElementById('page-simulation');

    if (pageDash && pageSim) {
      if (page === 'dashboard') {
        pageDash.classList.add('active');
        pageDash.scrollTop = 0;
        pageSim.classList.remove('active');
        // Halt virtual simulation motors
        if (this.robot) {
          this.robot.setMotorPwm(0, 0);
          this.robot.solenoidActive = false;
        }
        if (!this.physicalHardwareConnected) {
          const topStateBadge = document.getElementById('hud-state-badge');
          if (topStateBadge) {
            topStateBadge.textContent = 'OFFLINE (NO HW)';
            topStateBadge.className = 'state-badge estop';
          }
          const topPwm = document.getElementById('hud-pwm-txt');
          if (topPwm) topPwm.textContent = 'L:0 | R:0';
        }
      } else {
        pageDash.classList.remove('active');
        pageSim.classList.add('active');
        // When entering simulation, reset clock delta and trigger canvas resize
        if (this.sim && this.sim.clock) {
          this.sim.clock.getDelta();
        }
        if (this.sim && this.sim.onWindowResize) {
          this.sim.onWindowResize();
        }
      }
    }
  }

  emergencyStop() {
    this.robot.setMotorPwm(0, 0);
    this.robot.solenoidActive = false;
    this.robotState = 'ESTOP';
    this.dispatchBenchCommand('x');
    this.appendPiLog({
      source: 'SAFETY',
      level: 'ERROR',
      message: '[SAFETY] EMERGENCY STOP ACTIVATED! Motors halted, command \'x\' dispatched to serial bridge.',
      timestamp: Date.now() / 1000
    });
  }

  dispatchBenchCommand(cmd) {
    if (this.wsConnected && this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify({
        type: 'BENCH_COMMAND',
        cmd: cmd
      }));
    }

    // Direct local simulation responses
    if (cmd === 'w') {
      this.robot.setMotorPwm(150, 150);
      setTimeout(() => { if (this.robotState !== 'SEARCHING') this.robot.setMotorPwm(0, 0); }, 1000);
    } else if (cmd === 's') {
      this.robot.setMotorPwm(-150, -150);
      setTimeout(() => { if (this.robotState !== 'SEARCHING') this.robot.setMotorPwm(0, 0); }, 1000);
    } else if (cmd === 'a') {
      this.robot.setMotorPwm(-140, 140);
      setTimeout(() => { if (this.robotState !== 'SEARCHING') this.robot.setMotorPwm(0, 0); }, 800);
    } else if (cmd === 'd') {
      this.robot.setMotorPwm(140, -140);
      setTimeout(() => { if (this.robotState !== 'SEARCHING') this.robot.setMotorPwm(0, 0); }, 800);
    } else if (cmd === 'x') {
      this.robot.setMotorPwm(0, 0);
      this.robot.solenoidActive = false;
    } else if (cmd === 'f') {
      this.robot.solenoidActive = true;
      setTimeout(() => { this.robot.solenoidActive = false; }, 1000);
    } else if (cmd === 'sweep') {
      this.robot.setTiltAngle(45);
      setTimeout(() => this.robot.setTiltAngle(135), 400);
      setTimeout(() => this.robot.setTiltAngle(90), 800);
    } else if (cmd === 'sim') {
      const fwd = new THREE.Vector3(Math.sin(this.robot.yaw), 0, Math.cos(this.robot.yaw));
      const pos = this.robot.position.clone().add(fwd.multiplyScalar(1.5));
      this.fireSystem.addFire(pos.x, pos.z, 0.4);
    } else if (cmd === 'clear') {
      this.fireSystem.clearAllFires();
    }
  }

  appendPiLog(logEntry) {
    if (!logEntry) return;
    this.piLogs.push(logEntry);
    if (this.piLogs.length > this.maxLogs) this.piLogs.shift();
    this.renderPiLogs();
  }

  appendEspLog(logEntry) {
    // STRICT: Zero fake logs! Only accept lines from physical ESP32 UART
    if (!logEntry || logEntry.is_physical === false) {
      return;
    }
    this.espLogs.push(logEntry);
    if (this.espLogs.length > this.maxLogs) this.espLogs.shift();

    const msg = logEntry.message || '';
    if (msg.includes('uptime') || msg.includes('Uptime')) {
      const mUp = msg.match(/uptime[:\s]*(\d+)/i);
      if (mUp) {
        const sec = Math.floor(parseInt(mUp[1]) / 1000);
        const mm = String(Math.floor(sec / 60)).padStart(2, '0');
        const ss = String(sec % 60).padStart(2, '0');
        const hh = String(Math.floor(sec / 3600)).padStart(2, '0');
        const upEl = document.getElementById('esp-stat-uptime');
        if (upEl) upEl.textContent = `${hh}:${mm}:${ss}`;
      }
    }

    this.renderEspLogs();
  }

  renderPiLogs() {
    const term = document.getElementById('terminal-pi');
    if (!term) return;

    const filter = document.getElementById('sel-filter-pi')?.value || 'ALL';
    const autoscroll = document.getElementById('chk-autoscroll-pi')?.checked ?? true;

    const filtered = this.piLogs.filter((e) => {
      if (filter === 'ALL') return true;
      if (filter === 'WARN') return e.level === 'WARN' || e.level === 'ERROR';
      return e.source === filter;
    });

    if (filtered.length === 0) {
      term.innerHTML = `<div class="log-empty-state" style="color: var(--text-muted); padding: 24px 12px; font-style: italic; font-size: 0.82rem; text-align: center;">
        🍓 Awaiting live events from Raspberry Pi 3B over Wi-Fi...
      </div>`;
      return;
    }

    term.innerHTML = filtered.map((e) => {
      const t = e.timestamp ? new Date(e.timestamp * 1000) : new Date();
      const timeStr = t.toLocaleTimeString();
      const badgeCls = (e.source || 'system').toLowerCase();
      const msgCls = (e.level || 'info').toLowerCase();
      return `<div class="log-entry">
        <span class="log-time">[${timeStr}]</span>
        <span class="log-badge ${badgeCls}">[${e.source || 'SYS'}]</span>
        <span class="log-msg ${msgCls}">${this.escapeHtml(e.message || '')}</span>
      </div>`;
    }).join('');

    if (autoscroll) {
      term.scrollTop = term.scrollHeight;
    }
  }

  renderEspLogs() {
    const term = document.getElementById('terminal-esp32');
    if (!term) return;

    const filterText = (document.getElementById('inp-filter-esp')?.value || '').toLowerCase();
    const autoscroll = document.getElementById('chk-autoscroll-esp')?.checked ?? true;

    const filtered = this.espLogs.filter((e) => {
      if (!filterText) return true;
      return (e.message || '').toLowerCase().includes(filterText);
    });

    if (filtered.length === 0) {
      term.innerHTML = `<div class="log-empty-state" style="color: var(--text-muted); padding: 24px 12px; font-style: italic; font-size: 0.82rem; text-align: center;">
        ⚡ Awaiting real UART transmission from physical ESP32...
      </div>`;
      return;
    }

    term.innerHTML = filtered.map((e) => {
      const t = e.timestamp ? new Date(e.timestamp * 1000) : new Date();
      const timeStr = t.toLocaleTimeString();
      const msg = e.message || '';

      let badge = 'UART';
      let badgeCls = 'uart';
      if (msg.includes('[MOTORS]')) { badge = 'MOTORS'; badgeCls = 'motors'; }
      else if (msg.includes('[SENSORS]')) { badge = 'SENSORS'; badgeCls = 'vision'; }
      else if (msg.includes('[SOLENOID]')) { badge = 'SOLENOID'; badgeCls = 'solenoid'; }
      else if (msg.includes('[TINYML]')) { badge = 'TINYML'; badgeCls = 'vision'; }
      else if (msg.includes('[FSM]')) { badge = 'FSM'; badgeCls = 'fsm'; }

      const msgCls = (e.level || 'info').toLowerCase();
      return `<div class="log-entry">
        <span class="log-time">[${timeStr}]</span>
        <span class="log-badge ${badgeCls}">[${badge}]</span>
        <span class="log-msg ${msgCls}">${this.escapeHtml(msg)}</span>
      </div>`;
    }).join('');

    if (autoscroll) {
      term.scrollTop = term.scrollHeight;
    }
  }

  escapeHtml(str) {
    return str
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#039;');
  }

  updateDashboardHud(dt, telem) {
    if (!this.physicalHardwareConnected || !telem) {
      // -----------------------------------------------------------------------
      // PHYSICAL HARDWARE OFFLINE / DISCONNECTED STATE:
      // Zero out all dashboard readings. Never display virtual simulation data!
      // -----------------------------------------------------------------------
      const kpiState = document.getElementById('kpi-robot-state');
      if (kpiState) {
        kpiState.textContent = 'OFFLINE (NO HW)';
        kpiState.style.color = 'var(--accent-red)';
      }
      const kpiMode = document.getElementById('kpi-sync-mode');
      if (kpiMode) kpiMode.textContent = 'Physical Serial (Disconnected)';

      if (this.activePage === 'dashboard') {
        const topStateBadge = document.getElementById('hud-state-badge');
        if (topStateBadge) {
          topStateBadge.textContent = 'OFFLINE (NO HW)';
          topStateBadge.className = 'state-badge estop';
        }
        const topPwm = document.getElementById('hud-pwm-txt');
        if (topPwm) topPwm.textContent = 'L:0 | R:0';
      }

      const kpiThreat = document.getElementById('kpi-fire-threat');
      const kpiThreatCard = document.getElementById('kpi-card-fire');
      if (kpiThreat && kpiThreatCard) {
        kpiThreat.textContent = 'STANDBY';
        kpiThreat.style.color = 'var(--text-muted)';
        kpiThreatCard.className = 'kpi-card';
      }

      const kpiConf = document.getElementById('kpi-fire-conf');
      if (kpiConf) kpiConf.textContent = '0%';
      const kpiDist = document.getElementById('kpi-standoff-dist');
      if (kpiDist) kpiDist.textContent = '-- m';
      const kpiSol = document.getElementById('kpi-solenoid-state');
      if (kpiSol) {
        kpiSol.textContent = 'OFF';
        kpiSol.style.color = 'var(--accent-green)';
      }
      const kpiPitch = document.getElementById('kpi-turret-pitch');
      if (kpiPitch) kpiPitch.textContent = '--°';
      const kpiOpt = document.getElementById('kpi-optical-status');
      if (kpiOpt) {
        kpiOpt.textContent = '--';
        kpiOpt.style.color = '#fff';
      }

      // Zero out HC-SR04 Dual cm / in gauges
      ['front', 'left', 'right'].forEach((prefix) => {
        const bar = document.getElementById(`dash-gauge-${prefix}`);
        const txtCm = document.getElementById(`dash-gauge-txt-${prefix}`);
        const txtIn = document.getElementById(`dash-gauge-in-${prefix}`);
        if (bar && txtCm && txtIn) {
          bar.style.width = '0%';
          bar.style.backgroundColor = 'rgba(255, 255, 255, 0.1)';
          txtCm.textContent = '-- cm';
          txtIn.textContent = '-- in';
        }
      });

      // Zero out TB6612 Motor Drive bars
      ['left', 'right'].forEach((side) => {
        const bar = document.getElementById(`dash-motor-bar-${side}`);
        const txt = document.getElementById(`dash-motor-txt-${side}`);
        if (bar && txt) {
          bar.style.width = '0%';
          txt.textContent = '0';
        }
      });

      const dFlameDot = document.getElementById('dash-flame-dot');
      const dFlameTxt = document.getElementById('dash-flame-txt');
      if (dFlameDot && dFlameTxt) {
        dFlameDot.classList.remove('active-flame');
        dFlameTxt.textContent = 'CLEAR';
        dFlameTxt.style.color = '#fff';
      }

      const dSolDot = document.getElementById('dash-solenoid-dot');
      const dSolTxt = document.getElementById('dash-solenoid-txt');
      if (dSolDot && dSolTxt) {
        dSolDot.classList.remove('active-solenoid');
        dSolTxt.textContent = 'OFF';
        dSolTxt.style.color = '#fff';
      }

      this.drawTacticalMiniMapOffline();
      this.renderTelemetryChartOffline();
      return;
    }

    // -----------------------------------------------------------------------
    // PHYSICAL HARDWARE ONLINE: Display Live Real Readings from ESP32
    // -----------------------------------------------------------------------

    // 1. KPI Cards
    const kpiState = document.getElementById('kpi-robot-state');
    if (kpiState) {
      kpiState.textContent = (telem.robot_state || 'ONLINE').replace(/_/g, ' ');
      kpiState.style.color = 'var(--accent-cyan)';
    }

    const kpiMode = document.getElementById('kpi-sync-mode');
    if (kpiMode) {
      kpiMode.textContent = `Physical UART (${this.physicalHardwarePort || 'Serial'})`;
    }

    if (this.activePage === 'dashboard') {
      const topStateBadge = document.getElementById('hud-state-badge');
      if (topStateBadge) {
        const st = (telem.robot_state || 'ONLINE').replace(/_/g, ' ');
        topStateBadge.textContent = st;
        topStateBadge.className = `state-badge ${(telem.robot_state || 'searching').toLowerCase()}`;
      }
      const topPwm = document.getElementById('hud-pwm-txt');
      if (topPwm) {
        topPwm.textContent = `L:${telem.left_pwm ?? 0} | R:${telem.right_pwm ?? 0}`;
      }
    }

    const hasFire = !!(telem.flame_detected);
    const kpiThreat = document.getElementById('kpi-fire-threat');
    const kpiThreatCard = document.getElementById('kpi-card-fire');

    if (kpiThreat && kpiThreatCard) {
      if (hasFire) {
        kpiThreat.textContent = '🔥 FLAME DETECTED';
        kpiThreat.style.color = 'var(--accent-red)';
        kpiThreatCard.className = 'kpi-card danger';
      } else {
        kpiThreat.textContent = 'CLEAR';
        kpiThreat.style.color = 'var(--accent-green)';
        kpiThreatCard.className = 'kpi-card success';
      }
    }

    const kpiConf = document.getElementById('kpi-fire-conf');
    if (kpiConf) {
      kpiConf.textContent = hasFire ? '100%' : '0%';
    }

    const kpiDist = document.getElementById('kpi-standoff-dist');
    if (kpiDist) {
      const d = (telem.dist_front !== undefined) ? (telem.dist_front / 100.0) : 0;
      kpiDist.textContent = `${d.toFixed(2)} m`;
    }

    const kpiSol = document.getElementById('kpi-solenoid-state');
    if (kpiSol) {
      kpiSol.textContent = telem.solenoid_active ? '💧 SPRAYING' : 'OFF';
      kpiSol.style.color = telem.solenoid_active ? 'var(--accent-cyan)' : 'var(--accent-green)';
    }

    const kpiPitch = document.getElementById('kpi-turret-pitch');
    if (kpiPitch) {
      kpiPitch.textContent = `${telem.tilt_angle || 90}°`;
    }

    const kpiOpt = document.getElementById('kpi-optical-status');
    if (kpiOpt) {
      kpiOpt.textContent = telem.flame_detected ? 'DETECTED' : 'Clear';
      kpiOpt.style.color = telem.flame_detected ? 'var(--accent-orange)' : '#fff';
    }

    // Mission Elapsed Time
    const kpiTime = document.getElementById('kpi-mission-time');
    if (kpiTime) {
      const elapsedSec = Math.floor((Date.now() - this.missionStartTime) / 1000);
      const mm = String(Math.floor(elapsedSec / 60)).padStart(2, '0');
      const ss = String(elapsedSec % 60).padStart(2, '0');
      kpiTime.textContent = `${mm}:${ss}`;
    }

    // 2. Sensor Gauges (dual cm / in)
    const updateDashGauge = (prefix, valCm) => {
      const bar = document.getElementById(`dash-gauge-${prefix}`);
      const txtCm = document.getElementById(`dash-gauge-txt-${prefix}`);
      const txtIn = document.getElementById(`dash-gauge-in-${prefix}`);

      if (bar && txtCm && txtIn) {
        const val = (valCm !== undefined) ? valCm : 0;
        const pct = Math.min(100, Math.max(0, (val / 300.0) * 100));
        bar.style.width = `${pct}%`;
        txtCm.textContent = `${val.toFixed(1)} cm`;
        txtIn.textContent = `${(val * 0.393701).toFixed(1)} in`;

        if (val < 25.4) { // < 10 inches
          bar.style.backgroundColor = 'var(--accent-red)';
        } else if (val < 45.0) {
          bar.style.backgroundColor = 'var(--accent-yellow)';
        } else {
          bar.style.backgroundColor = 'var(--accent-green)';
        }
      }
    };

    updateDashGauge('front', telem.dist_front);
    updateDashGauge('left', telem.dist_left);
    updateDashGauge('right', telem.dist_right);

    // Motor PWM bars
    const updateMotorBar = (side, pwm) => {
      const bar = document.getElementById(`dash-motor-bar-${side}`);
      const txt = document.getElementById(`dash-motor-txt-${side}`);
      if (bar && txt) {
        const val = pwm || 0;
        const pct = Math.min(100, Math.round((Math.abs(val) / 255.0) * 100));
        bar.style.width = `${pct}%`;
        bar.style.background = (val >= 0)
          ? 'linear-gradient(90deg, var(--accent-cyan), #3b82f6)'
          : 'linear-gradient(90deg, var(--accent-orange), var(--accent-red))';
        txt.textContent = `${val}`;
      }
    };

    updateMotorBar('left', telem.left_pwm);
    updateMotorBar('right', telem.right_pwm);

    // Flame & Solenoid Indicators in matrix
    const dFlameDot = document.getElementById('dash-flame-dot');
    const dFlameTxt = document.getElementById('dash-flame-txt');
    if (dFlameDot && dFlameTxt) {
      if (telem.flame_detected) {
        dFlameDot.classList.add('active-flame');
        dFlameTxt.textContent = 'DETECTED';
        dFlameTxt.style.color = 'var(--accent-orange)';
      } else {
        dFlameDot.classList.remove('active-flame');
        dFlameTxt.textContent = 'CLEAR';
        dFlameTxt.style.color = '#fff';
      }
    }

    const dSolDot = document.getElementById('dash-solenoid-dot');
    const dSolTxt = document.getElementById('dash-solenoid-txt');
    if (dSolDot && dSolTxt) {
      if (telem.solenoid_active) {
        dSolDot.classList.add('active-solenoid');
        dSolTxt.textContent = 'SPRAYING';
        dSolTxt.style.color = 'var(--accent-cyan)';
      } else {
        dSolDot.classList.remove('active-solenoid');
        dSolTxt.textContent = 'OFF';
        dSolTxt.style.color = '#fff';
      }
    }

    // 3. Mini-Map Drawing (10 Hz)
    this.minimapTimer = (this.minimapTimer || 0) + (dt || 0.016);
    if (this.minimapTimer >= 0.10) {
      this.minimapTimer = 0;
      this.drawTacticalMiniMap();
    }

    // 4. Strip Chart (2 Hz)
    this.chartTimer = (this.chartTimer || 0) + (dt || 0.016);
    if (this.chartTimer >= 0.50) {
      this.chartTimer = 0;
      this.telemetryHistory.push({
        time: Date.now(),
        front: telem.dist_front || 0,
        left: telem.dist_left || 0,
        right: telem.dist_right || 0,
        leftPwm: telem.left_pwm || 0,
        rightPwm: telem.right_pwm || 0,
        flameConf: telem.flame_detected ? 100 : 0
      });
      if (this.telemetryHistory.length > 60) this.telemetryHistory.shift();
      this.renderTelemetryChart();
    }
  }

  drawTacticalMiniMapOffline() {
    const canvas = document.getElementById('dashboard-map-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const w = canvas.width;
    const h = canvas.height;

    ctx.fillStyle = '#05080e';
    ctx.fillRect(0, 0, w, h);

    // Subtle dark tactical grid lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += 25) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let y = 0; y < h; y += 25) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // Centered tactical warning text
    ctx.fillStyle = 'rgba(255, 51, 75, 0.75)';
    ctx.font = 'bold 11px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('PHYSICAL HARDWARE OFFLINE', w / 2, h / 2 - 8);
    ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.font = '9px monospace';
    ctx.fillText('Waiting for ESP32 UART Telemetry', w / 2, h / 2 + 10);
  }

  renderTelemetryChartOffline() {
    const canvas = document.getElementById('dashboard-chart-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const w = canvas.width;
    const h = canvas.height;

    ctx.fillStyle = '#04070a';
    ctx.fillRect(0, 0, w, h);

    // Grid lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
    ctx.lineWidth = 1;
    for (let y = 20; y < h; y += 30) {
      ctx.beginPath();
      ctx.moveTo(40, y);
      ctx.lineTo(w - 10, y);
      ctx.stroke();
    }

    ctx.fillStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('WAITING FOR PHYSICAL HARDWARE TELEMETRY STREAM...', w / 2, h / 2);
  }

  drawTacticalMiniMap() {
    const canvas = document.getElementById('dashboard-map-canvas');
    if (!canvas || !this.mapper || !this.mapper.explorationGrid) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.fillStyle = '#05080e';
    ctx.fillRect(0, 0, w, h);

    const grid = this.mapper.explorationGrid;
    const dim = grid.gridWidth;
    const cellSizeX = w / dim;
    const cellSizeY = h / dim;

    // Draw grid cells
    for (let gy = 0; gy < dim; gy++) {
      for (let gx = 0; gx < dim; gx++) {
        const state = grid.cells[gy * dim + gx];
        if (state === 1) { // Free / Explored
          ctx.fillStyle = 'rgba(0, 210, 255, 0.15)';
          ctx.fillRect(gx * cellSizeX, gy * cellSizeY, cellSizeX, cellSizeY);
        } else if (state === 2) { // Occupied Wall
          ctx.fillStyle = 'rgba(255, 51, 75, 0.85)';
          ctx.fillRect(gx * cellSizeX, gy * cellSizeY, cellSizeX, cellSizeY);
        }
      }
    }

    // Draw Robot Position & Heading
    const rpos = this.robot ? this.robot.position : { x: 0, z: 0 };
    const ryaw = this.robot ? this.robot.yaw : 0;
    const rx = ((rpos.x + grid.halfSize) / (grid.halfSize * 2)) * w;
    const ry = ((rpos.z + grid.halfSize) / (grid.halfSize * 2)) * h;

    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(ryaw);

    // Robot body
    ctx.fillStyle = '#00d2ff';
    ctx.beginPath();
    ctx.arc(0, 0, 5, 0, Math.PI * 2);
    ctx.fill();

    // Direction pointer
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, 10);
    ctx.stroke();

    ctx.restore();

    // Draw Frontiers if available
    if (this.mapper.lastFrontierGoals && this.mapper.lastFrontierGoals.length > 0) {
      ctx.fillStyle = '#ffd600';
      for (const fg of this.mapper.lastFrontierGoals) {
        const fx = ((fg.x + grid.halfSize) / (grid.halfSize * 2)) * w;
        const fy = ((fg.z + grid.halfSize) / (grid.halfSize * 2)) * h;
        ctx.beginPath();
        ctx.arc(fx, fy, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  renderTelemetryChart() {
    const canvas = document.getElementById('dashboard-chart-canvas');
    if (!canvas || this.telemetryHistory.length === 0) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.fillStyle = '#04070a';
    ctx.fillRect(0, 0, w, h);

    // Grid lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
    ctx.lineWidth = 1;
    for (let y = 20; y < h; y += 30) {
      ctx.beginPath();
      ctx.moveTo(40, y);
      ctx.lineTo(w - 10, y);
      ctx.stroke();
    }

    const n = this.telemetryHistory.length;
    const stepX = (w - 60) / Math.max(1, n - 1);

    const drawLine = (prop, color, minVal, maxVal) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const val = this.telemetryHistory[i][prop] || 0;
        const norm = Math.max(0, Math.min(1, (val - minVal) / (maxVal - minVal)));
        const x = 45 + i * stepX;
        const y = h - 20 - norm * (h - 40);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    };

    if (this.activeChartChannel === 'sensors') {
      drawLine('front', '#00d2ff', 0, 300);
      drawLine('left', '#00e676', 0, 300);
      drawLine('right', '#ffd600', 0, 300);

      // Y-axis labels
      ctx.fillStyle = '#8b949e';
      ctx.font = '10px monospace';
      ctx.fillText('300cm', 5, 25);
      ctx.fillText('150cm', 5, (h / 2) + 5);
      ctx.fillText('0cm', 12, h - 18);
    } else if (this.activeChartChannel === 'pwm') {
      drawLine('leftPwm', '#00d2ff', -255, 255);
      drawLine('rightPwm', '#ff7b00', -255, 255);

      ctx.fillStyle = '#8b949e';
      ctx.font = '10px monospace';
      ctx.fillText('+255', 10, 25);
      ctx.fillText('0', 25, (h / 2) + 5);
      ctx.fillText('-255', 10, h - 18);
    } else if (this.activeChartChannel === 'flame') {
      drawLine('flameConf', '#ff334b', 0, 100);

      ctx.fillStyle = '#8b949e';
      ctx.font = '10px monospace';
      ctx.fillText('100%', 10, 25);
      ctx.fillText('50%', 15, (h / 2) + 5);
      ctx.fillText('0%', 20, h - 18);
    }
  }
}

