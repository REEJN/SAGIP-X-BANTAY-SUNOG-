/**
 * BANTAY-SUNOG Obstacle Manager
 * Handles dynamic obstacles (crates, walls, cylinders) for collision testing and ultrasonic sensor raycasting.
 */

class ObstacleManager {
  constructor(scene) {
    this.scene = scene;
    this.obstacles = new Map();
    this.nextId = 1;

    // Reusable materials
    this.boxMaterial = new THREE.MeshStandardMaterial({
      color: 0x3d5a80,
      roughness: 0.7,
      metalness: 0.2
    });

    this.wallMaterial = new THREE.MeshStandardMaterial({
      color: 0x4a5568,
      roughness: 0.8,
      metalness: 0.1
    });

    this.cylinderMaterial = new THREE.MeshStandardMaterial({
      color: 0x9a3412,
      roughness: 0.5,
      metalness: 0.4
    });

    this.selectedMaterial = new THREE.MeshStandardMaterial({
      color: 0x00d2ff,
      wireframe: true
    });
  }

  createBox(x, z, width = 0.6, height = 0.5, depth = 0.6, rotationY = 0) {
    const id = this.nextId++;
    const geometry = new THREE.BoxGeometry(width, height, depth);
    const mesh = new THREE.Mesh(geometry, this.boxMaterial.clone());
    mesh.position.set(x, height / 2, z);
    mesh.rotation.y = rotationY;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.userData = { id, type: 'box', width, height, depth };

    // Edge wireframe accent
    const edges = new THREE.EdgesGeometry(geometry);
    const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x60a5fa }));
    mesh.add(line);

    this.scene.add(mesh);
    this.obstacles.set(id, mesh);
    return mesh;
  }

  createWall(x, z, length = 1.6, height = 0.6, thickness = 0.15, rotationY = 0) {
    const id = this.nextId++;
    const geometry = new THREE.BoxGeometry(length, height, thickness);
    const mesh = new THREE.Mesh(geometry, this.wallMaterial.clone());
    mesh.position.set(x, height / 2, z);
    mesh.rotation.y = rotationY;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.userData = { id, type: 'wall', width: length, height, depth: thickness };

    const edges = new THREE.EdgesGeometry(geometry);
    const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0xa0aec0 }));
    mesh.add(line);

    this.scene.add(mesh);
    this.obstacles.set(id, mesh);
    return mesh;
  }

  createCylinder(x, z, radius = 0.3, height = 0.7) {
    const id = this.nextId++;
    const geometry = new THREE.CylinderGeometry(radius, radius, height, 24);
    const mesh = new THREE.Mesh(geometry, this.cylinderMaterial.clone());
    mesh.position.set(x, height / 2, z);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.userData = { id, type: 'cylinder', radius, height };

    this.scene.add(mesh);
    this.obstacles.set(id, mesh);
    return mesh;
  }

  removeObstacle(id) {
    const mesh = this.obstacles.get(id);
    if (mesh) {
      this.scene.remove(mesh);
      if (mesh.geometry) mesh.geometry.dispose();
      this.obstacles.delete(id);
    }
  }

  clearAll() {
    for (const [id, mesh] of this.obstacles.entries()) {
      this.scene.remove(mesh);
      if (mesh.geometry) mesh.geometry.dispose();
    }
    this.obstacles.clear();
  }

  getRaycastMeshes() {
    return Array.from(this.obstacles.values());
  }

  getObstacle(id) {
    return this.obstacles.get(id);
  }

  updateObstaclePosition(id, x, z) {
    const mesh = this.obstacles.get(id);
    if (!mesh) return;
    mesh.position.x = x;
    mesh.position.z = z;
  }

  updateObstacleRotation(id, rotationY) {
    const mesh = this.obstacles.get(id);
    if (!mesh) return;
    mesh.rotation.y = rotationY;
  }

  updateObstacleDimensions(id, dims = {}) {
    const mesh = this.obstacles.get(id);
    if (!mesh || !mesh.userData) return;
    const type = mesh.userData.type;

    if (type === 'box' || type === 'wall') {
      const width = dims.width !== undefined ? Math.max(0.2, Number(dims.width)) : mesh.userData.width;
      const depth = dims.depth !== undefined ? Math.max(0.1, Number(dims.depth)) : mesh.userData.depth;
      const height = dims.height !== undefined ? Math.max(0.2, Number(dims.height)) : mesh.userData.height;

      mesh.geometry.dispose();
      mesh.geometry = new THREE.BoxGeometry(width, height, depth);
      mesh.userData.width = width;
      mesh.userData.depth = depth;
      mesh.userData.height = height;
      mesh.position.y = height / 2;

      // Update edge wireframe if present
      const line = mesh.children.find(c => c instanceof THREE.LineSegments);
      if (line) {
        line.geometry.dispose();
        line.geometry = new THREE.EdgesGeometry(mesh.geometry);
      }
    } else if (type === 'cylinder') {
      const radius = dims.radius !== undefined ? Math.max(0.1, Number(dims.radius)) : mesh.userData.radius;
      const height = dims.height !== undefined ? Math.max(0.2, Number(dims.height)) : mesh.userData.height;

      mesh.geometry.dispose();
      mesh.geometry = new THREE.CylinderGeometry(radius, radius, height, 24);
      mesh.userData.radius = radius;
      mesh.userData.height = height;
      mesh.position.y = height / 2;
    }
  }

  duplicateObstacle(id) {
    const mesh = this.obstacles.get(id);
    if (!mesh || !mesh.userData) return null;
    const type = mesh.userData.type;
    const offset = 0.6;

    if (type === 'box') {
      return this.createBox(mesh.position.x + offset, mesh.position.z + offset, mesh.userData.width, mesh.userData.height, mesh.userData.depth, mesh.rotation.y);
    } else if (type === 'wall') {
      return this.createWall(mesh.position.x + offset, mesh.position.z + offset, mesh.userData.width, mesh.userData.height, mesh.userData.depth, mesh.rotation.y);
    } else if (type === 'cylinder') {
      return this.createCylinder(mesh.position.x + offset, mesh.position.z + offset, mesh.userData.radius, mesh.userData.height);
    }
    return null;
  }

  /**
   * Set up standard realistic test room obstacle layout.
   */
  loadDefaultLayout() {
    this.clearAll();
    // Office/Room obstacles (crates, dividers, barrels) with realistic navigation corridors
    this.createBox(-1.8, 1.2, 0.6, 0.5, 0.6, 0.2);
    this.createBox(2.0, -1.0, 0.7, 0.6, 0.7, -0.4);
    this.createWall(0.0, 2.5, 2.0, 0.5, 0.15, 0);
    this.createWall(-1.8, -1.8, 1.5, 0.5, 0.15, Math.PI / 3);
    this.createCylinder(1.8, 1.8, 0.3, 0.7);
  }

  /**
   * 4-Room Multi-Room Floorplan with connecting central corridor & doorways.
   * Room 1: Living Area (South-West)
   * Room 2: Office (North-West)
   * Room 3: Kitchen (South-East)
   * Room 4: Storage (North-East)
   */
  loadMultiRoomLayout() {
    this.clearAll();
    const wallH = 0.65;
    const wallThick = 0.16;

    // 1. Central Hallway Spine Walls (running along Z = -4.2 to +4.2 with 0.80m doorways)
    // West Hallway Wall (X = -0.85)
    this.createWall(-0.85, 3.4, 1.4, wallH, wallThick, Math.PI / 2);     // North segment
    // Doorway gap at Z = 2.3 (width 0.80m)
    this.createWall(-0.85, 1.0, 1.8, wallH, wallThick, Math.PI / 2);     // Mid-North segment
    this.createWall(-0.85, -1.0, 1.8, wallH, wallThick, Math.PI / 2);    // Mid-South segment
    // Doorway gap at Z = -2.3 (width 0.80m)
    this.createWall(-0.85, -3.4, 1.4, wallH, wallThick, Math.PI / 2);    // South segment

    // East Hallway Wall (X = +0.85)
    this.createWall(0.85, 3.4, 1.4, wallH, wallThick, Math.PI / 2);      // North segment
    // Doorway gap at Z = 2.3 (width 0.80m)
    this.createWall(0.85, 1.0, 1.8, wallH, wallThick, Math.PI / 2);      // Mid-North segment
    this.createWall(0.85, -1.0, 1.8, wallH, wallThick, Math.PI / 2);     // Mid-South segment
    // Doorway gap at Z = -2.3 (width 0.80m)
    this.createWall(0.85, -3.4, 1.4, wallH, wallThick, Math.PI / 2);     // South segment

    // 2. Horizontal Room Dividers (along Z = 0)
    // Divides NW Office from SW Living Room
    this.createWall(-2.55, 0.0, 3.4, wallH, wallThick, 0);
    // Divides NE Storage from SE Kitchen
    this.createWall(2.55, 0.0, 3.4, wallH, wallThick, 0);

    // 3. Realistic Interior Furnishings / Obstacles
    // SW Living Room
    this.createBox(-2.6, -2.4, 0.8, 0.5, 0.8, 0.1);
    this.createCylinder(-3.2, -1.2, 0.28, 0.65);
    // NW Office
    this.createBox(-2.5, 2.5, 1.0, 0.5, 0.6, 0.2);
    this.createBox(-3.2, 1.2, 0.5, 0.5, 0.5, 0);
    // SE Kitchen
    this.createBox(2.6, -2.4, 0.9, 0.5, 0.7, -0.3);
    this.createCylinder(3.2, -1.2, 0.3, 0.7);
    // NE Storage
    this.createBox(2.4, 2.6, 0.7, 0.6, 0.7, 0.4);
    this.createBox(3.2, 1.4, 0.6, 0.5, 0.6, -0.2);
    this.createCylinder(2.0, 1.5, 0.28, 0.65);
  }

  /**
   * Search & Rescue Labyrinth Maze Layout.
   * Challenging layout with corridors, 90° bends, and dead ends for robust autonomous navigation.
   */
  loadMazeLayout() {
    this.clearAll();
    const wallH = 0.65;
    const wallThick = 0.16;

    // Perimeter entry & outer baffles
    this.createWall(0.0, -3.0, 5.0, wallH, wallThick, 0);
    this.createWall(-2.6, -1.8, 2.4, wallH, wallThick, Math.PI / 2);
    this.createWall(2.6, -1.8, 2.4, wallH, wallThick, Math.PI / 2);

    // Middle labyrinth dividers with staggered openings
    this.createWall(-1.2, -0.6, 3.4, wallH, wallThick, 0);
    this.createWall(2.0, -0.6, 2.6, wallH, wallThick, 0);

    // Center vertical spine
    this.createWall(0.4, 0.8, 2.8, wallH, wallThick, Math.PI / 2);
    this.createWall(-1.8, 1.2, 2.4, wallH, wallThick, Math.PI / 2);

    // North corridors
    this.createWall(-0.5, 2.2, 4.2, wallH, wallThick, 0);
    this.createWall(2.5, 1.8, 2.0, wallH, wallThick, Math.PI / 2);
    this.createWall(1.5, 3.2, 3.0, wallH, wallThick, 0);

    // Dispersed obstacles within corridors
    this.createBox(-3.0, 0.4, 0.55, 0.5, 0.55, 0.2);
    this.createCylinder(1.6, 0.4, 0.28, 0.65);
    this.createBox(-0.8, -1.8, 0.5, 0.5, 0.5, 0.1);
    this.createCylinder(3.2, -2.2, 0.3, 0.65);
  }

  /**
   * Switch between arena environments.
   * @param {string} layoutName - 'arena', 'multi-room', 'maze'
   */
  setLayout(layoutName) {
    if (layoutName === 'multi-room') {
      this.loadMultiRoomLayout();
    } else if (layoutName === 'maze') {
      this.loadMazeLayout();
    } else {
      this.loadDefaultLayout();
    }
  }

  /**
   * Generates a randomized test arena layout.
   */
  randomizeLayout(arenaRadius = 3.2) {
    this.clearAll();
    const count = 4 + Math.floor(Math.random() * 4);
    for (let i = 0; i < count; i++) {
      const angle = Math.random() * Math.PI * 2;
      const dist = 1.0 + Math.random() * (arenaRadius - 1.2);
      const x = Math.cos(angle) * dist;
      const z = Math.sin(angle) * dist;
      const type = Math.floor(Math.random() * 3);

      if (type === 0) {
        this.createBox(x, z, 0.5 + Math.random() * 0.4, 0.5, 0.5 + Math.random() * 0.4, Math.random() * Math.PI);
      } else if (type === 1) {
        this.createWall(x, z, 1.2 + Math.random() * 0.8, 0.5, 0.15, Math.random() * Math.PI);
      } else {
        this.createCylinder(x, z, 0.25 + Math.random() * 0.15, 0.6 + Math.random() * 0.2);
      }
    }
  }
}
