/**
 * BANTAY-SUNOG 3D Fire Simulation System
 * Manages dynamic particle flames, smoke, heat emission, point lights, and extinguishing physics.
 */

class FireEntity {
  constructor(id, scene, x, z, intensity = 1.0) {
    this.id = id;
    this.scene = scene;
    this.position = new THREE.Vector3(x, 0.05, z);
    this.intensity = intensity; // 1.0 = fully burning, 0.0 = extinguished
    this.isExtinguished = false;
    this.particleCount = 120;
    this.particles = [];

    this.group = new THREE.Group();
    this.group.position.copy(this.position);

    // Dynamic flickering point light
    this.light = new THREE.PointLight(0xff6600, 2.5 * this.intensity, 6.0, 2.0);
    this.light.position.set(0, 0.4, 0);
    this.light.castShadow = true;
    this.light.shadow.bias = -0.002;
    this.group.add(this.light);

    // Base glowing burn disc on the floor
    const discGeo = new THREE.CircleGeometry(0.35, 16);
    discGeo.rotateX(-Math.PI / 2);
    const discMat = new THREE.MeshBasicMaterial({
      color: 0x221105,
      transparent: true,
      opacity: 0.8
    });
    this.baseDisc = new THREE.Mesh(discGeo, discMat);
    this.baseDisc.position.y = 0.01;
    this.group.add(this.baseDisc);

    // Particle system geometry and texture
    this.initParticles();

    // Floating Health/Intensity Bar Indicator
    this.initHealthBar();

    this.scene.add(this.group);
  }

  initParticles() {
    // Generate particle canvas sprite
    const canvas = document.createElement('canvas');
    canvas.width = 64;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');
    const grad = ctx.createRadialGradient(32, 32, 2, 32, 32, 30);
    grad.addColorStop(0, 'rgba(255, 255, 220, 1)');
    grad.addColorStop(0.3, 'rgba(255, 160, 20, 0.9)');
    grad.addColorStop(0.7, 'rgba(230, 40, 10, 0.5)');
    grad.addColorStop(1, 'rgba(40, 10, 5, 0)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 64, 64);

    const texture = new THREE.CanvasTexture(canvas);

    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(this.particleCount * 3);
    const colors = new Float32Array(this.particleCount * 3);
    const sizes = new Float32Array(this.particleCount);

    this.particleData = [];

    for (let i = 0; i < this.particleCount; i++) {
      const p = {
        x: (Math.random() - 0.5) * 0.25,
        y: Math.random() * 0.6,
        z: (Math.random() - 0.5) * 0.25,
        vx: (Math.random() - 0.5) * 0.15,
        vy: 0.4 + Math.random() * 0.6,
        vz: (Math.random() - 0.5) * 0.15,
        life: Math.random(),
        maxLife: 0.8 + Math.random() * 0.5,
        baseSize: 0.25 + Math.random() * 0.2
      };
      this.particleData.push(p);

      positions[i * 3] = p.x;
      positions[i * 3 + 1] = p.y;
      positions[i * 3 + 2] = p.z;

      colors[i * 3] = 1.0;
      colors[i * 3 + 1] = 0.6;
      colors[i * 3 + 2] = 0.1;

      sizes[i] = p.baseSize;
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geometry.setAttribute('size', new THREE.BufferAttribute(sizes, 1));

    const material = new THREE.PointsMaterial({
      size: 0.35,
      map: texture,
      transparent: true,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      vertexColors: true
    });

    this.pointCloud = new THREE.Points(geometry, material);
    this.group.add(this.pointCloud);
  }

  initHealthBar() {
    const barGeo = new THREE.PlaneGeometry(0.5, 0.06);
    const barBgMat = new THREE.MeshBasicMaterial({ color: 0x222222, side: THREE.DoubleSide });
    this.healthBarBg = new THREE.Mesh(barGeo, barBgMat);
    this.healthBarBg.position.set(0, 0.9, 0);

    const fgGeo = new THREE.PlaneGeometry(0.48, 0.04);
    this.healthBarFgMat = new THREE.MeshBasicMaterial({ color: 0xff334b, side: THREE.DoubleSide });
    this.healthBarFg = new THREE.Mesh(fgGeo, this.healthBarFgMat);
    this.healthBarFg.position.set(0, 0.9, 0.005);

    this.group.add(this.healthBarBg);
    this.group.add(this.healthBarFg);
  }

  update(dt, camera) {
    if (this.isExtinguished) return;

    // Billboard health bar towards active camera
    if (camera) {
      this.healthBarBg.lookAt(camera.position);
      this.healthBarFg.lookAt(camera.position);
    }

    // Flicker point light
    const flicker = 0.85 + Math.random() * 0.3;
    this.light.intensity = 2.8 * this.intensity * flicker;

    // Scale health bar based on intensity
    this.healthBarFg.scale.x = Math.max(0.01, this.intensity);
    if (this.intensity < 0.3) {
      this.healthBarFgMat.color.setHex(0x00e676);
    } else if (this.intensity < 0.6) {
      this.healthBarFgMat.color.setHex(0xffaa00);
    } else {
      this.healthBarFgMat.color.setHex(0xff334b);
    }

    // Update particle dynamics
    const posAttr = this.pointCloud.geometry.attributes.position;
    const colAttr = this.pointCloud.geometry.attributes.color;

    for (let i = 0; i < this.particleCount; i++) {
      const p = this.particleData[i];
      p.life += dt;

      if (p.life >= p.maxLife) {
        // Reset particle
        p.life = 0;
        p.x = (Math.random() - 0.5) * 0.2 * this.intensity;
        p.y = 0.05;
        p.z = (Math.random() - 0.5) * 0.2 * this.intensity;
        p.vy = (0.4 + Math.random() * 0.5) * (0.5 + 0.5 * this.intensity);
      } else {
        p.x += p.vx * dt;
        p.y += p.vy * dt;
        p.z += p.vz * dt;
      }

      posAttr.setXYZ(i, p.x, p.y, p.z);

      // Color fades from hot yellow to flame orange to dark smoke
      const progress = p.life / p.maxLife;
      if (progress < 0.3) {
        colAttr.setXYZ(i, 1.0, 0.9 * (1 - progress), 0.2);
      } else if (progress < 0.7) {
        colAttr.setXYZ(i, 1.0, 0.4, 0.05);
      } else {
        const fade = (1.0 - progress) * this.intensity;
        colAttr.setXYZ(i, 0.4 * fade, 0.1 * fade, 0.05 * fade);
      }
    }

    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
  }

  extinguishStep(amount) {
    if (this.isExtinguished) return true;

    this.intensity -= amount;
    if (this.intensity <= 0.0) {
      this.intensity = 0.0;
      this.isExtinguished = true;
      this.group.remove(this.pointCloud);
      this.group.remove(this.light);
      this.group.remove(this.healthBarBg);
      this.group.remove(this.healthBarFg);
      this.baseDisc.material.color.setHex(0x111111);
      return true; // Newly extinguished
    }
    return false;
  }

  destroy() {
    this.scene.remove(this.group);
  }
}

class FireSystem {
  constructor(scene) {
    this.scene = scene;
    this.fires = new Map();
    this.nextId = 1;
  }

  spawnFire(x, z, intensity = 1.0) {
    const id = this.nextId++;
    const fire = new FireEntity(id, this.scene, x, z, intensity);
    this.fires.set(id, fire);
    return fire;
  }

  removeFire(id) {
    const fire = this.fires.get(id);
    if (fire) {
      fire.destroy();
      this.fires.delete(id);
    }
  }

  clearAllFires() {
    for (const [id, fire] of this.fires.entries()) {
      fire.destroy();
    }
    this.fires.clear();
  }

  update(dt, camera) {
    for (const [id, fire] of this.fires.entries()) {
      fire.update(dt, camera);
    }
  }

  getActiveFires() {
    const active = [];
    for (const [id, fire] of this.fires.entries()) {
      if (!fire.isExtinguished && fire.intensity > 0.01) {
        active.push({
          id: fire.id,
          position: fire.position.clone(),
          intensity: fire.intensity
        });
      }
    }
    return active;
  }

  /**
   * Applies extinguisher spray check.
   * If nozzle pos + spray vector intersects a fire within range and angle, dampens it.
   */
  applyExtinguisher(nozzlePos, sprayDir, maxRange = 1.2, coneAngleRad = 0.45, rate = 0.45, dt = 0.016) {
    let anyHit = false;
    const hitFires = [];

    for (const [id, fire] of this.fires.entries()) {
      if (fire.isExtinguished) continue;

      const fireCenter = fire.position.clone().add(new THREE.Vector3(0, 0.25, 0));
      const toFire = new THREE.Vector3().subVectors(fireCenter, nozzlePos);
      const dist = toFire.length();

      if (dist <= maxRange) {
        toFire.normalize();
        const dot = sprayDir.dot(toFire);
        const angle = Math.acos(Math.min(1.0, Math.max(-1.0, dot)));

        if (angle <= coneAngleRad) {
          const extinguished = fire.extinguishStep(rate * dt);
          anyHit = true;
          hitFires.push({ id, intensity: fire.intensity, extinguished });
        }
      }
    }

    return { hit: anyHit, hitFires };
  }
}
