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
