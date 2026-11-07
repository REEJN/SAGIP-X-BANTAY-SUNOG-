/**
 * BANTAY-SUNOG 3D Simulation World
 * Scene setup, arena enclosure, lighting, camera view modes, and main animation loop.
 */

class SimWorld {
  constructor(containerElement, pipCanvasElement) {
    this.container = containerElement;
    this.pipCanvas = pipCanvasElement;

    this.arenaRadius = 4.8;
    this.arenaWidth = 9.0;
    this.arenaHeight = 9.0;

    this.viewMode = 'orbit'; // 'orbit', 'follow', 'top'

    this.clock = new THREE.Clock();
    this.arenaWalls = [];

    this.initScene();
    this.initLights();
    this.initArena();
    this.initEntities();
    this.initInteractions();

    window.addEventListener('resize', () => this.onWindowResize(), false);
  }

  initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a0e14);
    this.scene.fog = new THREE.FogExp2(0x0a0e14, 0.035);

    // Main Orbit Camera
    this.camera = new THREE.PerspectiveCamera(
      55,
      this.container.clientWidth / this.container.clientHeight,
      0.1,
      100.0
    );
    this.camera.position.set(0, 5.5, 6.5);
    this.camera.layers.enable(1); // Enable Layer 1 for debug/AR visualizers on main camera

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.1;

    this.container.appendChild(this.renderer.domElement);

    // Orbit Controls
    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.08;
      this.controls.maxPolarAngle = Math.PI / 2 - 0.02; // Don't go below floor
      this.controls.minDistance = 1.0;
      this.controls.maxDistance = 25.0;
      this.controls.target.set(0, 0.2, 0);
    }
  }

  initLights() {
    // Ambient light
    const ambientLight = new THREE.AmbientLight(0x384252, 1.2);
    this.scene.add(ambientLight);

    // Main directional overhead light
    this.dirLight = new THREE.DirectionalLight(0xfff5ea, 1.5);
    this.dirLight.position.set(5, 10, 6);
    this.dirLight.castShadow = true;
    this.dirLight.shadow.mapSize.width = 2048;
    this.dirLight.shadow.mapSize.height = 2048;
    this.dirLight.shadow.camera.near = 0.5;
    this.dirLight.shadow.camera.far = 25;
    const d = 6.0;
    this.dirLight.shadow.camera.left = -d;
    this.dirLight.shadow.camera.right = d;
    this.dirLight.shadow.camera.top = d;
    this.dirLight.shadow.camera.bottom = -d;
    this.dirLight.shadow.bias = -0.0005;
    this.scene.add(this.dirLight);

    // Subtle blue rim light
    const rimLight = new THREE.DirectionalLight(0x00d2ff, 0.5);
    rimLight.position.set(-6, 4, -6);
    this.scene.add(rimLight);
  }

  initArena() {
    // Floor
    const floorGeo = new THREE.PlaneGeometry(this.arenaWidth, this.arenaHeight);
    floorGeo.rotateX(-Math.PI / 2);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x121720,
      roughness: 0.85,
      metalness: 0.15
    });
    this.floor = new THREE.Mesh(floorGeo, floorMat);
    this.floor.receiveShadow = true;
    this.scene.add(this.floor);

    // Visual Grid (0.5m grid intervals)
    const grid = new THREE.GridHelper(this.arenaWidth, 18, 0x00d2ff, 0x1e293b);
    grid.position.y = 0.002;
    this.scene.add(grid);

    // Perimeter Boundary Walls
    const wallHeight = 0.7;
    const wallThick = 0.15;
    const halfW = this.arenaWidth / 2;
    const halfH = this.arenaHeight / 2;

    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.7,
      metalness: 0.3
    });

    const wallConfigs = [
      { x: 0, z: halfH, w: this.arenaWidth + wallThick, h: wallHeight, d: wallThick },   // North
      { x: 0, z: -halfH, w: this.arenaWidth + wallThick, h: wallHeight, d: wallThick },  // South
      { x: -halfW, z: 0, w: wallThick, h: wallHeight, d: this.arenaHeight },             // West
      { x: halfW, z: 0, w: wallThick, h: wallHeight, d: this.arenaHeight }              // East
    ];

    wallConfigs.forEach((c) => {
      const geo = new THREE.BoxGeometry(c.w, c.h, c.d);
      const wall = new THREE.Mesh(geo, wallMat);
      wall.position.set(c.x, c.h / 2, c.z);
      wall.castShadow = true;
      wall.receiveShadow = true;
      wall.userData = { isWall: true };
      this.scene.add(wall);
      this.arenaWalls.push(wall);
    });
  }

  initEntities() {
    // Fire System
    this.fireSystem = new FireSystem(this.scene);

    // Obstacle Manager
    this.obstacleManager = new ObstacleManager(this.scene);
    this.obstacleManager.loadDefaultLayout();

    // Robot Digital Twin
    this.robot = new RobotModel(this.scene);

    // Sensor Simulator
    this.sensors = new SensorSimulator(
      this.scene,
      this.robot,
      this.fireSystem,
      this.obstacleManager,
      this.pipCanvas
    );

    // 3D Asset Loader for custom user model
    this.assetLoader = new AssetLoader(this.robot);

    // Prediction Path Visualizer
    this.predictionPath = new PredictionPath(this.scene);
    this.sensors.predictionPath = this.predictionPath;

    // Spawn 1 initial test fire in open central lane
    this.fireSystem.spawnFire(0.0, 1.2, 1.0);
  }

  initInteractions() {
    this.raycaster = new THREE.Raycaster();
    this.mouse = new THREE.Vector2();
    this.placementMode = null; // 'fire', 'box', 'wall', 'cylinder'

    // Interactive Sandbox Selection & Dragging
    this.selectedObstacle = null;
    this.selectionBox = null;
    this.isDraggingObstacle = false;
    this.dragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    this.dragOffset = new THREE.Vector3();

    this.renderer.domElement.addEventListener('pointerdown', (e) => this.onPointerDown(e), false);
    this.renderer.domElement.addEventListener('pointermove', (e) => this.onPointerMove(e), false);
    window.addEventListener('pointerup', () => this.onPointerUp(), false);
  }

  setPlacementMode(mode) {
    this.placementMode = mode;
    if (mode) this.deselectObstacle();
    this.renderer.domElement.style.cursor = mode ? 'crosshair' : 'default';
  }

  selectObstacle(mesh) {
    if (!mesh || !mesh.userData) return;
    this.selectedObstacle = mesh;

    if (this.selectionBox) {
      this.scene.remove(this.selectionBox);
    }
    this.selectionBox = new THREE.BoxHelper(mesh, 0x00f0ff);
    this.selectionBox.material.depthTest = false;
    this.selectionBox.material.transparent = true;
    this.selectionBox.material.opacity = 0.9;
    this.selectionBox.renderOrder = 999;
    this.scene.add(this.selectionBox);

    if (this.onObstacleSelected) {
      this.onObstacleSelected(mesh);
    }
  }

  deselectObstacle() {
    if (this.selectionBox) {
      this.scene.remove(this.selectionBox);
      this.selectionBox = null;
    }
    const hadSelection = !!this.selectedObstacle;
    this.selectedObstacle = null;
    this.isDraggingObstacle = false;
    if (this.controls) this.controls.enabled = true;

    if (hadSelection && this.onObstacleDeselected) {
      this.onObstacleDeselected();
    }
  }

  updateSelectionHelper() {
    if (this.selectionBox && this.selectedObstacle) {
      this.selectionBox.update();
    }
  }

  setLayout(layoutName) {
    this.deselectObstacle();
    this.obstacleManager.setLayout(layoutName);
    this.fireSystem.clearAllFires();

    if (layoutName === 'multi-room') {
      // Reposition robot at South entry corridor
      this.robot.reset(0.0, -3.8, 0);
      // Spawn test fire inside North-East room
      this.fireSystem.spawnFire(2.6, 2.6, 1.0);
    } else if (layoutName === 'maze') {
      // Reposition robot at bottom maze entrance
      this.robot.reset(0.0, -4.0, 0);
      // Spawn test fire deep inside northern corridor
      this.fireSystem.spawnFire(1.5, 2.8, 1.0);
    } else {
      // Open arena default
      this.robot.reset(0, 0, 0);
      this.fireSystem.spawnFire(0.0, 1.5, 1.0);
    }

    if (this.onLayoutChanged) {
      this.onLayoutChanged(layoutName);
    }
  }

  onPointerDown(e) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

    this.raycaster.setFromCamera(this.mouse, this.camera);

    // 1. If in placement mode: drop object on floor
    if (this.placementMode) {
      const intersects = this.raycaster.intersectObject(this.floor);
      if (intersects.length > 0) {
        const pt = intersects[0].point;
        const x = Math.round(pt.x * 10) / 10;
        const z = Math.round(pt.z * 10) / 10;

        if (this.placementMode === 'fire') {
          this.fireSystem.spawnFire(x, z, 1.0);
          if (this.onFireSpawned) this.onFireSpawned();
        } else if (this.placementMode === 'box') {
          const m = this.obstacleManager.createBox(x, z);
          this.selectObstacle(m);
        } else if (this.placementMode === 'wall') {
          const m = this.obstacleManager.createWall(x, z);
          this.selectObstacle(m);
        } else if (this.placementMode === 'cylinder') {
          const m = this.obstacleManager.createCylinder(x, z);
          this.selectObstacle(m);
        }

        this.setPlacementMode(null);
        if (this.onPlacementComplete) this.onPlacementComplete();
      }
      return;
    }

    // 2. Not in placement mode: check for obstacle selection & dragging
    const meshes = this.obstacleManager.getRaycastMeshes();
    const hits = this.raycaster.intersectObjects(meshes, true);

    if (hits.length > 0) {
      // Find root obstacle mesh
      let hitMesh = hits[0].object;
      while (hitMesh.parent && !hitMesh.userData.id && hitMesh.parent !== this.scene) {
        hitMesh = hitMesh.parent;
      }

      if (hitMesh && hitMesh.userData.id) {
        this.selectObstacle(hitMesh);
        this.isDraggingObstacle = true;
        if (this.controls) this.controls.enabled = false;

        const floorIntersects = this.raycaster.intersectObject(this.floor);
        if (floorIntersects.length > 0) {
          this.dragOffset.copy(floorIntersects[0].point).sub(hitMesh.position);
          this.dragOffset.y = 0;
        }
        return;
      }
    }

    // 3. Clicked empty floor: deselect
    const floorHits = this.raycaster.intersectObject(this.floor);
    if (floorHits.length > 0 && !this.isDraggingObstacle) {
      this.deselectObstacle();
    }
  }

  onPointerMove(e) {
    if (!this.isDraggingObstacle || !this.selectedObstacle) return;

    const rect = this.renderer.domElement.getBoundingClientRect();
    this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

    this.raycaster.setFromCamera(this.mouse, this.camera);
    const intersects = this.raycaster.intersectObject(this.floor);

    if (intersects.length > 0) {
      const pt = intersects[0].point;
      const newX = Math.round((pt.x - this.dragOffset.x) * 10) / 10;
      const newZ = Math.round((pt.z - this.dragOffset.z) * 10) / 10;

      // Clamp within arena boundaries
      const maxBound = (this.arenaWidth / 2) - 0.4;
      const clampedX = Math.max(-maxBound, Math.min(maxBound, newX));
      const clampedZ = Math.max(-maxBound, Math.min(maxBound, newZ));

      this.obstacleManager.updateObstaclePosition(this.selectedObstacle.userData.id, clampedX, clampedZ);
      this.updateSelectionHelper();

      if (this.onObstacleModified) {
        this.onObstacleModified(this.selectedObstacle);
      }
    }
  }

  onPointerUp() {
    if (this.isDraggingObstacle) {
      this.isDraggingObstacle = false;
      if (this.controls) this.controls.enabled = true;
      this.updateSelectionHelper();
    }
  }

  setViewMode(mode) {
    this.viewMode = mode;
    if (mode === 'top') {
      this.camera.position.set(0, 11, 0.01);
      if (this.controls) this.controls.target.set(0, 0, 0);
    } else if (mode === 'orbit') {
      this.camera.position.set(0, 5.5, 6.5);
      if (this.controls) this.controls.target.set(0, 0.2, 0);
    }
  }

  onWindowResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();

    this.renderer.setSize(width, height);
  }

  update() {
    const dt = Math.min(this.clock.getDelta(), 0.05);

    // 1. Update Robot Kinematics & Collisions (Pass rectangular arena bounds & obstacles + fire hitboxes)
    const fireHitboxes = (this.fireSystem && this.fireSystem.getHitboxMeshes)
      ? this.fireSystem.getHitboxMeshes()
      : [];
    const obstacles = [
      ...this.obstacleManager.getRaycastMeshes(),
      ...fireHitboxes
    ];
    const arenaBounds = { maxX: (this.arenaWidth / 2) - 0.255, maxZ: (this.arenaHeight / 2) - 0.255 };
    this.robot.update(dt, obstacles, arenaBounds);

    // 2. Update Sensors
    this.sensors.update(dt, this.arenaWalls);

    // 3. Update Fire particles & billboard
    this.fireSystem.update(dt, this.camera);

    // 4. Update Camera View Modes
    if (this.viewMode === 'follow') {
      // 3rd Person follow camera behind robot
      const offsetDist = 1.6;
      const offsetHeight = 0.9;
      const camYaw = this.robot.yaw;

      const targetX = this.robot.position.x - Math.sin(camYaw) * offsetDist;
      const targetZ = this.robot.position.z - Math.cos(camYaw) * offsetDist;

      this.camera.position.lerp(new THREE.Vector3(targetX, this.robot.position.y + offsetHeight, targetZ), 0.1);
      const lookTarget = this.robot.position.clone().add(new THREE.Vector3(0, 0.25, 0));
      this.camera.lookAt(lookTarget);
    } else if (this.controls) {
      this.controls.update();
    }

    // 5. Main 3D Render
    this.renderer.render(this.scene, this.camera);
  }
}

/**
 * 3D Prediction Path Visualizer
 * Projects trajectory line, waypoints, and destination target on the floor.
 */
class PredictionPath {
  constructor(scene) {
    this.scene = scene;
    this.maxPoints = 50;
    this.hasPoints = false;

    this.group = new THREE.Group();
    this.group.name = 'PredictionPathGroup';

    const positions = new Float32Array(this.maxPoints * 3);
    this.geometry = new THREE.BufferGeometry();
    this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    this.material = new THREE.LineBasicMaterial({
      color: 0x00d2ff,
      linewidth: 3,
      transparent: true,
      opacity: 0.85
    });

    this.line = new THREE.Line(this.geometry, this.material);
    this.line.frustumCulled = false;
    this.group.add(this.line);

    // Waypoint dot beads
    const dotPositions = new Float32Array(this.maxPoints * 3);
    this.dotGeometry = new THREE.BufferGeometry();
    this.dotGeometry.setAttribute('position', new THREE.BufferAttribute(dotPositions, 3));
    this.dotMaterial = new THREE.PointsMaterial({
      color: 0x00e676,
      size: 0.08,
      transparent: true,
      opacity: 0.95
    });
    this.dots = new THREE.Points(this.dotGeometry, this.dotMaterial);
    this.dots.frustumCulled = false;
    this.group.add(this.dots);

    // Target destination ring
    const ringGeo = new THREE.RingGeometry(0.06, 0.12, 16);
    ringGeo.rotateX(-Math.PI / 2);
    this.ringMat = new THREE.MeshBasicMaterial({ color: 0x00d2ff, side: THREE.DoubleSide });
    this.tipRing = new THREE.Mesh(ringGeo, this.ringMat);
    this.tipRing.position.y = 0.015;
    this.group.add(this.tipRing);

    // Assign to Layer 1 (UI/Debug Overlays) so Virtual Webcam only sees real Layer 0 physical world
    this.line.layers.set(1);
    this.dots.layers.set(1);
    this.tipRing.layers.set(1);
    this.group.layers.set(1);

    this.scene.add(this.group);
  }

  setVisible(visible) {
    if (!visible) {
      this.group.visible = false;
      this.line.visible = false;
      this.dots.visible = false;
      this.tipRing.visible = false;
    } else if (this.hasPoints) {
      this.group.visible = true;
      this.line.visible = true;
      this.dots.visible = true;
      this.tipRing.visible = true;
    }
  }

  update(points, mode = 'scan') {
    if (!points || points.length < 2) {
      this.hasPoints = false;
      this.setVisible(false);
      return;
    }

    this.hasPoints = true;
    this.setVisible(true);

    // Color by mode: scan (cyan/green), avoid (red/amber), target (orange/fire)
    if (mode === 'avoid') {
      this.material.color.setHex(0xff334b);
      this.dotMaterial.color.setHex(0xffaa00);
      this.ringMat.color.setHex(0xff334b);
    } else if (mode === 'target') {
      this.material.color.setHex(0xff7b00);
      this.dotMaterial.color.setHex(0xff334b);
      this.ringMat.color.setHex(0xff7b00);
    } else {
      this.material.color.setHex(0x00d2ff);
      this.dotMaterial.color.setHex(0x00e676);
      this.ringMat.color.setHex(0x00d2ff);
    }

    const pos = this.geometry.attributes.position;
    const dotPos = this.dotGeometry.attributes.position;
    const count = Math.min(points.length, this.maxPoints);

    for (let i = 0; i < this.maxPoints; i++) {
      if (i < count) {
        pos.setXYZ(i, points[i].x, 0.015, points[i].z);
        dotPos.setXYZ(i, points[i].x, 0.018, points[i].z);
      } else {
        const last = points[count - 1];
        pos.setXYZ(i, last.x, 0.015, last.z);
        dotPos.setXYZ(i, last.x, 0.018, last.z);
      }
    }

    pos.needsUpdate = true;
    dotPos.needsUpdate = true;

    const tip = points[count - 1];
    this.tipRing.position.set(tip.x, 0.015, tip.z);
  }
}
