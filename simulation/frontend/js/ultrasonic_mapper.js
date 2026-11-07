/**
 * BANTAY-SUNOG 2D Ultrasonic Radar & SLAM Projection Mapper
 * Visualizes 3-beam ultrasonic field of view (HC-SR04 Front, Left, Right),
 * ego-centric point cloud projection of scanned obstacles, detected fire target,
 * standoff suppression perimeter, real-time extinguisher spray cone,
 * and PREDICTS A STABLE, COMMITTED, COLLISION-FREE AVOIDANCE PATH toward the best destination.
 *
 * Anti-Hallucination Features:
 * - Corridor Commitment & Direction Lock (1.5s hysteresis hold prevents rapid Left/Right flipping)
 * - Destination-Directed Sector Scoring (steers toward fire or open room waypoint)
 * - Exponential Moving Average (EMA) Heading Smoothing (eliminates jitter and twitching)
 */

/**
 * BANTAY-SUNOG 2D Exploration & Occupancy Grid
 * Discrete 2D occupancy and coverage map covering 9.0m x 9.0m at 0.20m resolution.
 * Tracks discovered free space, scanned obstacle boundaries, cell visit frequency,
 * and frontier boundaries leading into undiscovered areas.
 */
class ExplorationGrid {
  constructor(sizeMeters = 9.0, resolution = 0.20) {
    this.sizeMeters = sizeMeters;
    this.resolution = resolution;
    this.halfSize = sizeMeters / 2;
    this.arenaRadius = this.halfSize;
    this.gridWidth = Math.round(sizeMeters / resolution);  // 45
    this.gridHeight = Math.round(sizeMeters / resolution); // 45
    this.gridDim = this.gridWidth;
    this.totalCells = this.gridWidth * this.gridHeight;    // 2025

    // Cell states: 0 = UNDISCOVERED, 1 = FREE, 2 = OCCUPIED
    this.cells = new Uint8Array(this.totalCells);
    // Visit count: increments when robot moves through cell
    this.visitCounts = new Uint16Array(this.totalCells);

    this.discoveredCount = 0;
  }

  reset() {
    this.cells.fill(0);
    this.visitCounts.fill(0);
    this.discoveredCount = 0;
  }

  worldToGrid(x, z) {
    const gx = Math.floor((x + this.halfSize) / this.resolution);
    const gz = Math.floor((z + this.halfSize) / this.resolution);
    if (gx < 0 || gx >= this.gridWidth || gz < 0 || gz >= this.gridHeight) return null;
    return { gx, gz, idx: gz * this.gridWidth + gx };
  }

  gridToWorld(gx, gz) {
    const x = (gx + 0.5) * this.resolution - this.halfSize;
    const z = (gz + 0.5) * this.resolution - this.halfSize;
    return { x, z };
  }

  getCell(gx, gz) {
    if (gx < 0 || gx >= this.gridWidth || gz < 0 || gz >= this.gridHeight) return 0;
    return this.cells[gz * this.gridWidth + gx];
  }

  setCell(gx, gz, state) {
    if (gx < 0 || gx >= this.gridWidth || gz < 0 || gz >= this.gridHeight) return;
    const idx = gz * this.gridWidth + gx;
    const prev = this.cells[idx];
    if (prev === 0 && state !== 0) {
      this.discoveredCount++;
    }
    // OCCUPIED (2) overrides FREE (1); once marked OCCUPIED, stays OCCUPIED
    if (prev !== 2 || state === 2) {
      this.cells[idx] = state;
    }
  }

  recordRobotPose(x, z) {
    const g = this.worldToGrid(x, z);
    if (g) {
      this.setCell(g.gx, g.gz, 1);
      this.visitCounts[g.idx]++;
      // Mark immediate footprint of differential drive chassis (~0.22m)
      for (let dx = -1; dx <= 1; dx++) {
        for (let dz = -1; dz <= 1; dz++) {
          const nx = g.gx + dx;
          const nz = g.gz + dz;
          if (nx >= 0 && nx < this.gridWidth && nz >= 0 && nz < this.gridHeight) {
            this.setCell(nx, nz, 1);
          }
        }
      }
    }
  }

  updateBeam(robotX, robotZ, beamAngle, distanceM) {
    const g0 = this.worldToGrid(robotX, robotZ);
    if (!g0) return;

    const hitX = robotX + Math.sin(beamAngle) * distanceM;
    const hitZ = robotZ + Math.cos(beamAngle) * distanceM;
    const g1 = this.worldToGrid(hitX, hitZ);
    if (!g1) return;

    // Bresenham line raycasting
    let x0 = g0.gx, y0 = g0.gz;
    const x1 = g1.gx, y1 = g1.gz;
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = (x0 < x1) ? 1 : -1;
    const sy = (y0 < y1) ? 1 : -1;
    let err = dx - dy;

    while (true) {
      if (x0 === x1 && y0 === y1) {
        if (distanceM < 2.85) {
          this.setCell(x0, y0, 2); // OCCUPIED obstacle boundary
        } else {
          this.setCell(x0, y0, 1); // FREE at maximum range
        }
        break;
      }
      this.setCell(x0, y0, 1); // Free space carved by optical beam

      const e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx) { err += dx; y0 += sy; }
    }
  }

  /**
   * Fast 2D Bresenham line-of-sight raycaster.
   * Checks if an unobstructed line exists between world (x0, z0) and (x1, z1).
   * Returns false if any intermediate grid cell is OCCUPIED (state === 2).
   */
  hasLineOfSight(x0, z0, x1, z1) {
    const g0 = this.worldToGrid(x0, z0);
    const g1 = this.worldToGrid(x1, z1);
    if (!g0 || !g1) return false;

    let cx = g0.gx, cy = g0.gz;
    const x1g = g1.gx, y1g = g1.gz;
    const dx = Math.abs(x1g - cx);
    const dy = Math.abs(y1g - cy);
    const sx = (cx < x1g) ? 1 : -1;
    const sy = (cy < y1g) ? 1 : -1;
    let err = dx - dy;

    let isStart = true;
    while (true) {
      if (cx === x1g && cy === y1g) {
        break;
      }
      if (!isStart) {
        const cellState = this.getCell(cx, cy);
        if (cellState === 2) {
          return false; // Line-of-sight occluded by mapped wall/obstacle
        }
      }
      isStart = false;

      const e2 = 2 * err;
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 < dx) { err += dx; cy += sy; }
    }
    return true;
  }

  getCoveragePercentage() {
    return Math.min(100, Math.round((this.discoveredCount / (this.totalCells * 0.75)) * 100));
  }

  findFrontierGoals(robotPos, exclusionCentroid = null, exclusionRadius = 1.8) {
    const frontiers = [];
    const minWallDist = 1; // clearance from known walls

    for (let gz = 2; gz < this.gridHeight - 2; gz++) {
      for (let gx = 2; gx < this.gridWidth - 2; gx++) {
        const idx = gz * this.gridWidth + gx;
        if (this.cells[idx] !== 0) continue; // Must be UNDISCOVERED

        let hasFreeNeighbor = false;
        let nearOccupied = false;

        for (let dz = -1; dz <= 1; dz++) {
          for (let dx = -1; dx <= 1; dx++) {
            if (dx === 0 && dz === 0) continue;
            const nState = this.getCell(gx + dx, gz + dz);
            if (nState === 1) hasFreeNeighbor = true;
            if (nState === 2) nearOccupied = true;
          }
        }

        if (hasFreeNeighbor && !nearOccupied) {
          let clearOfWalls = true;
          for (let dz = -minWallDist; dz <= minWallDist; dz++) {
            for (let dx = -minWallDist; dx <= minWallDist; dx++) {
              if (this.getCell(gx + dx, gz + dz) === 2) {
                clearOfWalls = false;
                break;
              }
            }
            if (!clearOfWalls) break;
          }

          if (clearOfWalls) {
            const worldPt = this.gridToWorld(gx, gz);
            frontiers.push({ gx, gz, x: worldPt.x, z: worldPt.z });
          }
        }
      }
    }

    if (frontiers.length === 0) return null;

    // Cluster frontiers into candidate destination zones
    const clusters = [];
    frontiers.forEach(f => {
      let found = false;
      for (const c of clusters) {
        if (Math.hypot(c.x - f.x, c.z - f.z) < 0.9) {
          c.points.push(f);
          c.x = (c.x * (c.points.length - 1) + f.x) / c.points.length;
          c.z = (c.z * (c.points.length - 1) + f.z) / c.points.length;
          found = true;
          break;
        }
      }
      if (!found) {
        clusters.push({ x: f.x, z: f.z, points: [f] });
      }
    });

    let bestGoal = null;
    let maxScore = -999999;

    clusters.forEach(c => {
      const dist = robotPos ? Math.hypot(c.x - robotPos.x, c.z - robotPos.z) : 1.0;
      const infoGain = c.points.length;

      let score = infoGain * 14.0 - dist * 6.5;

      // Line-of-sight check: if frontier is directly occluded behind a solid wall, heavily penalize it
      // so robot routes toward open doorways instead of driving directly into walls!
      if (robotPos) {
        const hasLoS = this.hasLineOfSight(robotPos.x, robotPos.z, c.x, c.z);
        if (!hasLoS) {
          score -= 900.0; // Blocked behind a wall: cannot drive straight into it
        } else {
          score += 250.0; // Clear open corridor directly to frontier
        }
      }

      // Penalize frontiers inside or near the loop exclusion zone
      if (exclusionCentroid) {
        const distToExcl = Math.hypot(c.x - exclusionCentroid.x, c.z - exclusionCentroid.z);
        if (distToExcl < exclusionRadius) {
          score -= 400.0; // Avoid returning into the circulating area
        } else {
          score += distToExcl * 15.0; // Strong bonus for frontiers far from the loop
        }
      }

      if (score > maxScore) {
        maxScore = score;
        bestGoal = { x: c.x, z: c.z, clusterSize: infoGain, dist };
      }
    });

    return bestGoal;
  }
}

/**
 * BANTAY-SUNOG Path Memory & Circulation Loop Detector
 * Maintains a rolling breadcrumb trajectory history. Detects when the robot is
 * trapped circulating or looping in a localized area, and manages escape state.
 */
class CirculationDetector {
  constructor() {
    this.pathHistory = []; // Array of { x, z, yaw, time }
    this.maxHistoryMs = 18000; // Rolling 18-second history window
    this.sampleIntervalMs = 250;
    this.lastSampleTime = 0;

    this.isCirculating = false;
    this.circulationCentroid = null; // { x, z, radius }
    this.circulationStartTime = 0;
    this.escapeLockDurationMs = 6000;
  }

  reset() {
    this.pathHistory = [];
    this.isCirculating = false;
    this.circulationCentroid = null;
  }

  update(robotPos, robotYaw, now) {
    if (!robotPos) return this.isCirculating;

    if (now - this.lastSampleTime >= this.sampleIntervalMs) {
      this.lastSampleTime = now;
      this.pathHistory.push({
        x: robotPos.x,
        z: robotPos.z,
        yaw: robotYaw,
        time: now
      });
    }

    // Prune stale breadcrumbs
    this.pathHistory = this.pathHistory.filter(p => now - p.time <= this.maxHistoryMs);

    // If currently in escape mode, check if loop has been successfully broken
    if (this.isCirculating) {
      if (this.circulationCentroid) {
        const distFromCentroid = Math.hypot(robotPos.x - this.circulationCentroid.x, robotPos.z - this.circulationCentroid.z);
        const elapsed = now - this.circulationStartTime;

        // Escape success: robot moved > 1.8m away from loop center OR timeout reached
        if (distFromCentroid >= 1.75 || elapsed > this.escapeLockDurationMs) {
          this.isCirculating = false;
          this.circulationCentroid = null;
          this.pathHistory = this.pathHistory.filter(p => now - p.time <= 3000);
          console.log('[CirculationDetector] Loop broken! Successfully escaped to new area.');
          return false;
        }
      }
      return true;
    }

    // Evaluate circulation only with at least 5 seconds of movement (~20 samples)
    if (this.pathHistory.length < 20) return false;

    // 1. Total path distance traveled
    let totalPathDist = 0;
    for (let i = 1; i < this.pathHistory.length; i++) {
      totalPathDist += Math.hypot(
        this.pathHistory[i].x - this.pathHistory[i - 1].x,
        this.pathHistory[i].z - this.pathHistory[i - 1].z
      );
    }

    if (totalPathDist < 3.8) return false; // Must be driving continuously

    // 2. Centroid and dispersion radius
    let sumX = 0, sumZ = 0;
    this.pathHistory.forEach(p => { sumX += p.x; sumZ += p.z; });
    const cx = sumX / this.pathHistory.length;
    const cz = sumZ / this.pathHistory.length;

    let maxRadius = 0;
    this.pathHistory.forEach(p => {
      const r = Math.hypot(p.x - cx, p.z - cz);
      if (r > maxRadius) maxRadius = r;
    });

    // 3. Cumulative angular rotation
    let totalYawDelta = 0;
    for (let i = 1; i < this.pathHistory.length; i++) {
      let dy = this.pathHistory[i].yaw - this.pathHistory[i - 1].yaw;
      while (dy > Math.PI) dy -= Math.PI * 2;
      while (dy < -Math.PI) dy += Math.PI * 2;
      totalYawDelta += Math.abs(dy);
    }

    // Trapped Loop Trigger:
    // Traveled >= 3.8m within a tight radius (<= 1.25m) OR accumulated > 3.5pi rotation in tight area
    const isConfinedLoop = (maxRadius <= 1.25 && totalPathDist >= 3.8);
    const isSpinningLoop = (maxRadius <= 1.35 && totalYawDelta >= 3.5 * Math.PI && totalPathDist >= 2.8);

    if (isConfinedLoop || isSpinningLoop) {
      this.isCirculating = true;
      this.circulationCentroid = { x: cx, z: cz, radius: Math.max(1.3, maxRadius + 0.3) };
      this.circulationStartTime = now;
      console.warn(`[CirculationDetector] LOOP DETECTED! Path: ${totalPathDist.toFixed(1)}m, Radius: ${maxRadius.toFixed(2)}m. Engaging Escape Protocol!`);
      return true;
    }

    return false;
  }
}

// Physical inch-to-meter constants & tight navigation collision distances
const INCH_TO_M = 0.0254;
const M_TO_INCH = 1.0 / INCH_TO_M;
const COLLISION_TRIGGER_INCHES = 10.0;     // ~0.254m / 25.4cm (obstacle avoidance trigger)
const COLLISION_CLEAR_INCHES = 14.0;       // ~0.356m / 35.6cm (hysteresis clear threshold)
const COLLISION_EMERGENCY_INCHES = 6.0;    // ~0.152m / 15.2cm (emergency backout)
const CORRIDOR_HALF_WIDTH_INCHES = 5.5;    // ~0.140m / 14.0cm (allows 0.8m doorway transit)

class UltrasonicMapper {
  constructor(canvasId = 'map-canvas') {
    this.canvas = document.getElementById(canvasId);
    if (!this.canvas) {
      console.warn(`[UltrasonicMapper] Canvas #${canvasId} not found`);
      return;
    }
    this.ctx = this.canvas.getContext('2d');

    // Scanned obstacle hit points in world coordinates { worldX, worldZ, time }
    this.pointCloud = [];
    this.maxPointAgeMs = 6000; // 6-second persistence for point cloud
    this.lastPointAdd = 0;

    // Display constants
    this.maxRangeMeters = 3.2; // Radar display range in meters
    this.pulseAnim = 0;

    // Cached latest avoidance prediction
    this.latestAvoidance = null;

    // Stable Corridor Management (Eliminates twitching / rapid switching)
    this.activeCorridor = 'NONE'; // 'LEFT', 'RIGHT', 'NONE'
    this.corridorCommitTime = 0;
    this.minCommitDurationMs = 2000; // Lock into corridor for at least 2.0s
    this.isObstacleAheadState = false; // Schmitt-trigger hysteresis state
    this.smoothedClearAngle = 0.0;

    // Latched World-Anchored Path (Eliminates tail-chasing and circular looping)
    this.latchedWorldPath = null;
    this.latchedWaypointIdx = 0;
    this.latchedTimestamp = 0;
    this.clearCorridorFrames = 0;

    // Persistent Exploration Grid & Circulation Detection
    this.explorationGrid = new ExplorationGrid(9.0, 0.20);
    this.circulationDetector = new CirculationDetector();
    this.currentFrontierGoal = null;
    this.lastFrontierCompute = 0;
  }

  resetMap() {
    this.pointCloud = [];
    this.explorationGrid.reset();
    this.circulationDetector.reset();
    this.currentFrontierGoal = null;
    this.latchedWorldPath = null;
  }

  /**
   * Geometric Wall vs. Discrete Obstacle Classifier.
   * Distinguishes continuous walls/perimeters from localized obstacles (crates/barrels)
   * using point-cloud lateral span and 3-beam HC-SR04 flank clearance.
   *
   * @param {Array} localObstacles - Array of { x, z } in local robot frame
   * @param {number} mF - front distance in meters
   * @param {number} mL - left distance in meters
   * @param {number} mR - right distance in meters
   * @returns {Object} { type: 'CLEAR'|'OBSTACLE'|'WALL', spanX: number, flankClearSide: 'LEFT'|'RIGHT' }
   */
  classifyObject(localObstacles, mF, mL, mR) {
    const triggerM = COLLISION_TRIGGER_INCHES * INCH_TO_M; // ~0.254m (10 inches)
    const clearM = COLLISION_CLEAR_INCHES * INCH_TO_M;     // ~0.356m (14 inches)

    if (mF >= clearM && mL >= triggerM && mR >= triggerM) {
      return { type: 'CLEAR', spanX: 0, flankClearSide: (mR >= mL) ? 'RIGHT' : 'LEFT' };
    }

    // Filter obstacles located in the forward approach zone (Z in [0.04, 0.85m], |X| < 1.6m)
    const forwardObs = localObstacles.filter(o => o.z > 0.04 && o.z < 0.85 && Math.abs(o.x) < 1.6);
    let spanX = 0;
    if (forwardObs.length > 1) {
      const xs = forwardObs.map(o => o.x);
      spanX = Math.max(...xs) - Math.min(...xs);
    } else if (forwardObs.length === 1) {
      spanX = 0.35;
    }

    const flankClearSide = (mR >= mL) ? 'RIGHT' : 'LEFT';
    const isFlankClear = (mL > 0.45 || mR > 0.45);
    const flankRatio = Math.abs(mL - mR) / Math.max(0.1, Math.max(mL, mR));

    // Discrete Obstacle (Crate / Barrel): Narrow lateral span (<0.65m) and open flank
    if (spanX < 0.65 && (isFlankClear || flankRatio > 0.28)) {
      return { type: 'OBSTACLE', spanX, flankClearSide };
    }

    // Continuous surface, wide span, or tightly constrained corridor => WALL
    return { type: 'WALL', spanX, flankClearSide };
  }

  /**
   * Evaluates the mapping point cloud and 3-beam sensors to predict a stable, collision-free
   * avoidance path toward the best destination without rapid switching or oscillation.
   *
   * @param {Object} robotPos - { x, z } robot position in world coordinates
   * @param {number} robotYaw - heading angle in radians
   * @param {number} distF - front distance in cm
   * @param {number} distL - left distance in cm
   * @param {number} distR - right distance in cm
   * @param {Object|null} target - optional fire target from vision
   * @param {Object|null} goalPos - optional best world destination { x, z }
   * @returns {Object} Predicted avoidance path details
   */
  findClearAvoidancePath(robotPos, robotYaw, distF = 300, distL = 300, distR = 300, target = null, goalPos = null, hasFire = false) {
    const isFireActive = Boolean(hasFire || (target && target.confidence > 0.4));
    const now = Date.now();
    const activePoints = this.pointCloud.filter(p => now - p.time <= this.maxPointAgeMs);

    const cosY = Math.cos(robotYaw);
    const sinY = Math.sin(robotYaw);

    // Transform points to local ego-centric coordinates (meters)
    const localObstacles = [];

    // 1. Add point-cloud obstacles from map
    if (robotPos) {
      activePoints.forEach(p => {
        const dx = p.worldX - robotPos.x;
        const dz = p.worldZ - robotPos.z;
        const lx = dx * cosY - dz * sinY; // Lateral (+X = right, -X = left)
        const lz = dx * sinY + dz * cosY; // Longitudinal (+Z = forward)
        // Include flank obstacles alongside chassis (down to -0.22m)
        if (lz >= -0.22 && lz < 2.8 && Math.abs(lx) < 2.2) {
          localObstacles.push({ x: lx, z: lz, weight: 1.0 });
        }
      });
    }

    // 1.5 Add persistent exploration grid occupied cells to local obstacles
    // Adapts path prediction to all previously mapped walls, rooms, and partitions
    if (robotPos && this.explorationGrid) {
      const grid = this.explorationGrid;
      const res = grid.resolution;
      const searchRadiusM = 3.2;
      const minX = Math.max(-grid.arenaRadius, robotPos.x - searchRadiusM);
      const maxX = Math.min(grid.arenaRadius, robotPos.x + searchRadiusM);
      const minZ = Math.max(-grid.arenaRadius, robotPos.z - searchRadiusM);
      const maxZ = Math.min(grid.arenaRadius, robotPos.z + searchRadiusM);

      const iMin = Math.max(0, Math.floor((minX + grid.arenaRadius) / res));
      const iMax = Math.min(grid.gridDim - 1, Math.floor((maxX + grid.arenaRadius) / res));
      const jMin = Math.max(0, Math.floor((minZ + grid.arenaRadius) / res));
      const jMax = Math.min(grid.gridDim - 1, Math.floor((maxZ + grid.arenaRadius) / res));

      for (let j = jMin; j <= jMax; j++) {
        for (let i = iMin; i <= iMax; i++) {
          const idx = j * grid.gridDim + i;
          if (grid.cells[idx] === 2) { // 2 = OCCUPIED
            const wx = (i + 0.5) * res - grid.arenaRadius;
            const wz = (j + 0.5) * res - grid.arenaRadius;
            const dx = wx - robotPos.x;
            const dz = wz - robotPos.z;
            const dist = Math.hypot(dx, dz);
            if (dist > 0.05 && dist <= searchRadiusM) {
              const lx = dx * cosY - dz * sinY;
              const lz = dx * sinY + dz * cosY;
              // Include mapped obstacles down to chassis flank (-0.22m)
              if (lz >= -0.22 && lz < 2.8 && Math.abs(lx) < 2.2) {
                localObstacles.push({ x: lx, z: lz, weight: 2.0 });
              }
            }
          }
        }
      }
    }

    // 2. Add direct ultrasonic beam hits for immediate proximity protection
    const mF = (distF || 300) / 100.0;
    const mL = (distL || 300) / 100.0;
    const mR = (distR || 300) / 100.0;

    if (mF < 2.5) localObstacles.push({ x: 0, z: mF, weight: 1.5 });
    if (mL < 2.5) localObstacles.push({ x: Math.sin(-Math.PI / 4) * mL, z: Math.cos(-Math.PI / 4) * mL, weight: 1.5 });
    if (mR < 2.5) localObstacles.push({ x: Math.sin(Math.PI / 4) * mR, z: Math.cos(Math.PI / 4) * mR, weight: 1.5 });

    // 3. Add virtual arena wall samples to prevent wall collisions in all orientations
    if (robotPos) {
      const wallLimit = 4.22; // meters from arena center to inner wall face
      // East wall (X = +wallLimit)
      if (wallLimit - robotPos.x < 2.8) {
        for (let wz = Math.max(-wallLimit, robotPos.z - 2.2); wz <= Math.min(wallLimit, robotPos.z + 2.2); wz += 0.30) {
          const dx = wallLimit - robotPos.x;
          const dz = wz - robotPos.z;
          const lx = dx * cosY - dz * sinY;
          const lz = dx * sinY + dz * cosY;
          if (lz >= -0.22 && lz < 2.8 && Math.abs(lx) < 2.4) {
            localObstacles.push({ x: lx, z: lz, weight: 3.0 });
          }
        }
      }
      // West wall (X = -wallLimit)
      if (robotPos.x - (-wallLimit) < 2.8) {
        for (let wz = Math.max(-wallLimit, robotPos.z - 2.2); wz <= Math.min(wallLimit, robotPos.z + 2.2); wz += 0.30) {
          const dx = -wallLimit - robotPos.x;
          const dz = wz - robotPos.z;
          const lx = dx * cosY - dz * sinY;
          const lz = dx * sinY + dz * cosY;
          if (lz >= -0.22 && lz < 2.8 && Math.abs(lx) < 2.4) {
            localObstacles.push({ x: lx, z: lz, weight: 3.0 });
          }
        }
      }
      // North wall (Z = +wallLimit)
      if (wallLimit - robotPos.z < 2.8) {
        for (let wx = Math.max(-wallLimit, robotPos.x - 2.2); wx <= Math.min(wallLimit, robotPos.x + 2.2); wx += 0.30) {
          const dx = wx - robotPos.x;
          const dz = wallLimit - robotPos.z;
          const lx = dx * cosY - dz * sinY;
          const lz = dx * sinY + dz * cosY;
          if (lz >= -0.22 && lz < 2.8 && Math.abs(lx) < 2.4) {
            localObstacles.push({ x: lx, z: lz, weight: 3.0 });
          }
        }
      }
      // South wall (Z = -wallLimit)
      if (robotPos.z - (-wallLimit) < 2.8) {
        for (let wx = Math.max(-wallLimit, robotPos.x - 2.2); wx <= Math.min(wallLimit, robotPos.x + 2.2); wx += 0.30) {
          const dx = wx - robotPos.x;
          const dz = -wallLimit - robotPos.z;
          const lx = dx * cosY - dz * sinY;
          const lz = dx * sinY + dz * cosY;
          if (lz >= -0.22 && lz < 2.8 && Math.abs(lx) < 2.4) {
            localObstacles.push({ x: lx, z: lz, weight: 3.0 });
          }
        }
      }
    }

    // Compute flank obstacle clearances along robot body (Z in [-0.22, +0.35])
    let minLeftFlankDist = 9.9;
    let minRightFlankDist = 9.9;
    for (const obs of localObstacles) {
      if (obs.z >= -0.22 && obs.z <= 0.35) {
        if (obs.x < -0.05 && -obs.x < minLeftFlankDist) {
          minLeftFlankDist = -obs.x;
        } else if (obs.x > 0.05 && obs.x < minRightFlankDist) {
          minRightFlankDist = obs.x;
        }
      }
    }

    // 3. Check if forward collision corridor is blocked (with Schmitt-trigger hysteresis)
    // TIGHT INCH-BASED CORRIDOR: 5.5 inches half-width (11" total), 10" trigger distance
    const forwardCorridorHalfW = CORRIDOR_HALF_WIDTH_INCHES * INCH_TO_M; // ~0.140m (5.5 in)
    const collisionTriggerM = COLLISION_TRIGGER_INCHES * INCH_TO_M;     // ~0.254m (10.0 in)
    const collisionClearM = COLLISION_CLEAR_INCHES * INCH_TO_M;         // ~0.356m (14.0 in)
    let minForwardDist = mF;

    for (const obs of localObstacles) {
      if (obs.z > 0.04 && Math.abs(obs.x) < forwardCorridorHalfW) {
        if (obs.z < minForwardDist) minForwardDist = obs.z;
      }
    }

    // Schmitt-trigger hysteresis:
    // Turn ON obstacle avoidance at < 10.0 inches (0.254m); Turn OFF ONLY when path ahead is clear > 14.0 inches (0.356m)!
    if (!this.isObstacleAheadState) {
      if (minForwardDist < collisionTriggerM) {
        this.isObstacleAheadState = true;
      }
    } else {
      if (minForwardDist > collisionClearM) {
        this.isObstacleAheadState = false;
      }
    }
    const isObstacleAhead = this.isObstacleAheadState;

    // 4. Determine Destination Goal Heading (Towards best destination)
    let desiredGoalAngle = 0.0;
    if (goalPos && robotPos) {
      const dx = goalPos.x - robotPos.x;
      const dz = goalPos.z - robotPos.z;
      const worldGoalBearing = Math.atan2(dx, dz);
      let diff = worldGoalBearing - robotYaw;
      while (diff > Math.PI) diff -= Math.PI * 2;
      while (diff < -Math.PI) diff += Math.PI * 2;
      desiredGoalAngle = Math.max(-1.4, Math.min(1.4, diff));
    } else if (target && target.x !== undefined) {
      desiredGoalAngle = (target.x - 0.50) * 1.13;
    }

    // 5. Direction Lock & Corridor Commitment (Prevents Hallucination / Rapid Flipping)
    const isCommitted = (this.activeCorridor !== 'NONE' && (now - this.corridorCommitTime < this.minCommitDurationMs));

    if (isObstacleAhead) {
      if (!isCommitted || this.activeCorridor === 'NONE') {
        // Choose initial corridor side: destination alignment + ultrasonic clearance
        let chosenSide = 'RIGHT';
        if (Math.abs(desiredGoalAngle) > 0.20) {
          // If goal destination is distinctly to the right or left, bypass on that side
          chosenSide = desiredGoalAngle > 0 ? 'RIGHT' : 'LEFT';
          // Fail-safe: if the destination side is cramped (<0.35m), switch to clearer side
          if (chosenSide === 'RIGHT' && mR < 0.35 && mL > 0.55) chosenSide = 'LEFT';
          if (chosenSide === 'LEFT' && mL < 0.35 && mR > 0.55) chosenSide = 'RIGHT';
        } else {
          chosenSide = (mR >= mL) ? 'RIGHT' : 'LEFT';
        }
        this.activeCorridor = chosenSide;
        this.corridorCommitTime = now;
      } else {
        // Active corridor check: if committed side becomes blocked (<0.22m), emergency switch
        if (this.activeCorridor === 'RIGHT' && mR < 0.22 && mL > 0.60) {
          this.activeCorridor = 'LEFT';
          this.corridorCommitTime = now;
        } else if (this.activeCorridor === 'LEFT' && mL < 0.22 && mR > 0.60) {
          this.activeCorridor = 'RIGHT';
          this.corridorCommitTime = now;
        }
      }
    } else {
      // Clear ahead: release corridor lock only if completely free
      if (minForwardDist >= 1.15 && !isCommitted) {
        this.activeCorridor = 'NONE';
      }
    }

    // 6. Multi-Sector Polar Clearance Scan (-70 deg to +70 deg, 15 sectors)
    const sectorAngles = [];
    const numSectors = 15;
    for (let i = 0; i < numSectors; i++) {
      const a = -1.22 + (2.44 / (numSectors - 1)) * i; // ~ -70° to +70°
      sectorAngles.push(a);
    }

    let bestAngle = 0.0;
    let maxScore = -9999;
    const sectorClearances = [];
    const sectorCorridorHalfW = 0.26; // ~10.2 inches (expanded safety envelope to prevent corner grazing)

    sectorAngles.forEach(angle => {
      const dirX = Math.sin(angle);
      const dirZ = Math.cos(angle);

      // Raycast along (dirX, dirZ) out to 2.5m
      let clearance = 2.5;

      for (const obs of localObstacles) {
        const proj = obs.x * dirX + obs.z * dirZ;
        if (proj > 0.10 && proj < clearance) {
          const perpDist = Math.abs(obs.x * dirZ - obs.z * dirX);
          if (perpDist < sectorCorridorHalfW) {
            clearance = Math.min(clearance, proj);
          }
        }
      }

      sectorClearances.push({ angle, clearance });

      // Score components
      let score = clearance * 14.0;
      if (clearance > 1.2) score += 10.0;
      if (clearance > 1.8) score += 15.0;

      // Reward alignment with destination goal
      const angleDev = Math.abs(angle - desiredGoalAngle);
      score -= angleDev * 9.0;

      // Flank clearance safety: strictly prevent turning inward into an obstacle alongside chassis
      if (minLeftFlankDist < 0.32 && angle < -0.08) {
        score -= (0.32 - minLeftFlankDist) * 220.0; // Heavy penalty for turning into left flank obstacle
      }
      if (minRightFlankDist < 0.32 && angle > 0.08) {
        score -= (0.32 - minRightFlankDist) * 220.0; // Heavy penalty for turning into right flank obstacle
      }

      // Strong Corridor Commitment Hysteresis (prevents rapid left/right oscillation)
      if (this.activeCorridor === 'RIGHT') {
        if (angle > 0.15) {
          score += 35.0; // Strong bonus for maintaining committed rightward corridor
        } else if (angle < -0.15) {
          score -= 30.0; // Penalty for switching to opposite side
        }
      } else if (this.activeCorridor === 'LEFT') {
        if (angle < -0.15) {
          score += 35.0; // Strong bonus for maintaining committed leftward corridor
        } else if (angle > 0.15) {
          score -= 30.0; // Penalty for switching to opposite side
        }
      }

      // Heavily penalize heading directly into obstacle
      if (isObstacleAhead && Math.abs(angle) < 0.22) {
        score -= 40.0;
      }

      // Circulation Escape: heavily penalize steering towards the loop centroid
      if (this.circulationDetector.isCirculating && this.circulationDetector.circulationCentroid && robotPos) {
        const toCentroidX = this.circulationDetector.circulationCentroid.x - robotPos.x;
        const toCentroidZ = this.circulationDetector.circulationCentroid.z - robotPos.z;
        const centroidBearing = Math.atan2(toCentroidX, toCentroidZ);
        let centroidDiff = centroidBearing - robotYaw;
        while (centroidDiff > Math.PI) centroidDiff -= Math.PI * 2;
        while (centroidDiff < -Math.PI) centroidDiff += Math.PI * 2;
        const devFromCentroid = Math.abs(angle - centroidDiff);
        if (devFromCentroid < 0.70) {
          score -= 75.0; // Prevent steering towards the trapped center
        }
      }

      if (score > maxScore) {
        maxScore = score;
        bestAngle = angle;
      }
    });

    // 7. Exponential Moving Average (EMA) Heading Smoothing (removes jitter)
    const alpha = isObstacleAhead ? 0.25 : 0.40;
    this.smoothedClearAngle = this.smoothedClearAngle * (1.0 - alpha) + bestAngle * alpha;
    const finalAngle = (Math.abs(this.smoothedClearAngle) < 0.03 && !isObstacleAhead) ? 0.0 : this.smoothedClearAngle;

    // 7.5 Geometric Classification of Obstruction: Wall vs Discrete Obstacle
    const classification = this.classifyObject(localObstacles, mF, mL, mR);

    // 8. World-Anchored Trajectory Latching (Eliminates tail-chasing and endless circulating)
    let localPath = [];
    let worldPath = [];
    let isPathLatched = false;

    // Check if previously latched path should be released
    if (this.latchedWorldPath && robotPos) {
      const elapsed = now - this.latchedTimestamp;
      if (minForwardDist > 1.25) {
        this.clearCorridorFrames++;
      } else {
        this.clearCorridorFrames = 0;
      }

      const reachedEnd = this.latchedWaypointIdx >= this.latchedWorldPath.length - 1;
      const corridorClear = (this.clearCorridorFrames > 15 && elapsed > 600);
      const timedOut = (elapsed > 4500);
      const blockedAhead = (minForwardDist < COLLISION_EMERGENCY_INCHES * INCH_TO_M);

      if (reachedEnd || corridorClear || timedOut || blockedAhead) {
        this.latchedWorldPath = null;
        this.clearCorridorFrames = 0;
      }
    }

    if (!this.latchedWorldPath) {
      if (classification.type === 'WALL' && minForwardDist < collisionTriggerM + 0.05) {
        // Wall Encountered: Generate 90deg lane-turn arc into adjacent sweep lane
        let turnSign = (classification.flankClearSide === 'RIGHT') ? 1 : -1;
        if (Math.abs(desiredGoalAngle) > 0.25) {
          turnSign = (desiredGoalAngle > 0) ? 1 : -1;
        }
        const turnAngle = turnSign * (Math.PI / 2);
        localPath.push({ x: 0, y: 0 });
        if (robotPos) worldPath.push({ x: robotPos.x, z: robotPos.z });

        const numWp = 8;
        for (let i = 1; i <= numWp; i++) {
          const a = turnAngle * (i / numWp);
          const s = (i / numWp) * 0.90;
          const lx = Math.sin(a) * s;
          const lz = Math.cos(a) * s;
          localPath.push({ x: lx, y: lz });
          if (robotPos) {
            worldPath.push({
              x: robotPos.x + lx * cosY + lz * sinY,
              z: robotPos.z - lx * sinY + lz * cosY
            });
          }
        }
      } else if (isObstacleAhead && robotPos) {
        // Discrete Obstacle Encountered: Plan fresh evasive trajectory and LATCH IT in World Space!
        worldPath.push({ x: robotPos.x, z: robotPos.z });
        localPath.push({ x: 0, y: 0 });

        const pathLength = 1.4;
        const numWaypoints = 10;
        for (let i = 1; i <= numWaypoints; i++) {
          const frac = i / numWaypoints;
          const s = frac * pathLength;

          let currentYaw;
          if (frac <= 0.65) {
            currentYaw = finalAngle;
          } else {
            const blend = (frac - 0.65) / 0.35;
            currentYaw = finalAngle * (1.0 - blend) + desiredGoalAngle * blend;
          }

          const lx = Math.sin(currentYaw) * s;
          const lz = Math.cos(currentYaw) * s;
          localPath.push({ x: lx, y: lz });

          const wx = robotPos.x + lx * cosY + lz * sinY;
          const wz = robotPos.z - lx * sinY + lz * cosY;
          worldPath.push({ x: wx, z: wz });
        }

        // Anchor this trajectory to the arena floor!
        this.latchedWorldPath = worldPath;
        this.latchedWaypointIdx = 1;
        this.latchedTimestamp = now;
        isPathLatched = true;
      } else if (robotPos) {
        // Map-Adapted Cruise & Frontier Exploration Path (Adapts directly to mapped environment)
        localPath.push({ x: 0, y: 0 });
        worldPath.push({ x: robotPos.x, z: robotPos.z });

        let targetWorldBearing = robotYaw;
        if (isFireActive && target && target.x !== undefined) {
          targetWorldBearing = robotYaw + (target.x - 0.50) * 1.13;
        } else if (goalPos) {
          // Orient path toward undiscovered frontier goal from the map!
          const dx = goalPos.x - robotPos.x;
          const dz = goalPos.z - robotPos.z;
          targetWorldBearing = Math.atan2(dx, dz);
        }

        let bearingDiff = targetWorldBearing - robotYaw;
        while (bearingDiff > Math.PI) bearingDiff -= Math.PI * 2;
        while (bearingDiff < -Math.PI) bearingDiff += Math.PI * 2;
        bearingDiff = Math.max(-1.25, Math.min(1.25, bearingDiff));

        const cruiseLength = 1.8;
        const numWaypoints = 10;
        for (let i = 1; i <= numWaypoints; i++) {
          const frac = i / numWaypoints;
          const s = frac * cruiseLength;
          // Smooth curved trajectory adapting to mapped frontier heading
          const blendedAngle = bearingDiff * (frac * 0.70);
          let lx = Math.sin(blendedAngle) * s;
          let lz = Math.cos(blendedAngle) * s;

          let wx = robotPos.x + lx * cosY + lz * sinY;
          let wz = robotPos.z - lx * sinY + lz * cosY;

          // Check against ExplorationGrid: if waypoint is inside an occupied mapped cell, deflect away
          if (this.explorationGrid) {
            const gi = Math.floor((wx + this.explorationGrid.arenaRadius) / this.explorationGrid.resolution);
            const gj = Math.floor((wz + this.explorationGrid.arenaRadius) / this.explorationGrid.resolution);
            if (gi >= 0 && gi < this.explorationGrid.gridDim && gj >= 0 && gj < this.explorationGrid.gridDim) {
              if (this.explorationGrid.cells[gj * this.explorationGrid.gridDim + gi] === 2) {
                const deflectSign = (classification.flankClearSide === 'RIGHT') ? 1 : -1;
                wx += Math.cos(robotYaw) * 0.25 * deflectSign;
                wz += -Math.sin(robotYaw) * 0.25 * deflectSign;
                lx = (wx - robotPos.x) * cosY - (wz - robotPos.z) * sinY;
                lz = (wx - robotPos.x) * sinY + (wz - robotPos.z) * cosY;
              }
            }
          }

          worldPath.push({ x: wx, z: wz });
          localPath.push({ x: lx, y: lz });
        }
      }
    } else if (robotPos) {
      // Latched World Path is active!
      worldPath = this.latchedWorldPath;
      isPathLatched = true;

      // Transform the fixed world waypoints into current robot local coordinates
      localPath.push({ x: 0, y: 0 });
      for (let i = 1; i < worldPath.length; i++) {
        const wp = worldPath[i];
        const dx = wp.x - robotPos.x;
        const dz = wp.z - robotPos.z;
        const lx = dx * cosY - dz * sinY;
        const lz = dx * sinY + dz * cosY;
        localPath.push({ x: lx, y: lz });
      }

      // Check distance to current target waypoint to advance along the path
      const currentWp = worldPath[this.latchedWaypointIdx];
      if (currentWp) {
        const distToWp = Math.hypot(currentWp.x - robotPos.x, currentWp.z - robotPos.z);
        if (distToWp < 0.35 && this.latchedWaypointIdx < worldPath.length - 1) {
          this.latchedWaypointIdx++;
        }
      }
    }

    // 8.5 Waypoint Obstacle Inflation Pusher:
    // Guarantee that no waypoint cuts through or passes within 0.28m (11.0 in) of any obstacle vertex
    const minWaypointStandoff = 0.28;
    for (let i = 1; i < localPath.length; i++) {
      let p = localPath[i];
      for (const obs of localObstacles) {
        const dObsX = p.x - obs.x;
        const dObsZ = p.y - obs.z;
        const dObs = Math.hypot(dObsX, dObsZ);
        if (dObs < minWaypointStandoff && dObs > 0.001) {
          const pushDist = minWaypointStandoff - dObs;
          p.x += (dObsX / dObs) * pushDist;
          p.y += (dObsZ / dObs) * pushDist;
          if (robotPos && worldPath[i]) {
            worldPath[i].x = robotPos.x + p.x * cosY + p.y * sinY;
            worldPath[i].z = robotPos.z - p.x * sinY + p.y * cosY;
          }
        }
      }
    }

    // 9. Pure Pursuit Path Tracking & Heading Convergence:
    let recLeft = 145;
    let recRight = 145;

    // Pick lookahead point along localPath:
    let lookaheadIdx = 3;
    if (isPathLatched) {
      lookaheadIdx = Math.min(localPath.length - 1, Math.max(1, this.latchedWaypointIdx));
    }
    const lookaheadWp = localPath[lookaheadIdx] || { x: 0, y: 1.0 };
    // Bearing to lookahead point in robot's local body frame:
    const pathYaw = Math.atan2(lookaheadWp.x, lookaheadWp.y);

    if (minForwardDist < 0.28) {
      // Emergency reverse: immediately back out away from obstacle / wall!
      recLeft = -160;
      recRight = -160;
    } else if (classification.type === 'WALL' && minForwardDist < 0.70) {
      // Decisive 90deg pivot turn away from wall
      const turnSign = (classification.flankClearSide === 'RIGHT') ? 1 : -1;
      recLeft = turnSign * 160;
      recRight = -turnSign * 160;
    } else if (isObstacleAhead || (isPathLatched && minForwardDist < 0.70)) {
      // Decelerate proportional to proximity to wall/obstacle
      const proximityFactor = Math.max(0.0, Math.min(1.0, (minForwardDist - 0.28) / 0.55));
      const baseFwd = Math.round(50 + proximityFactor * 90); // 50 to 140 PWM

      if (minForwardDist < 0.42 && Math.abs(pathYaw) > 0.45) {
        // High proximity and large angular error: spin pivot in place toward the waypoint!
        const turnDir = (pathYaw >= 0) ? 1 : -1;
        recLeft = turnDir * 165;
        recRight = -turnDir * 165;
      } else {
        // Controlled pure pursuit curve along fixed world path:
        const turnRate = Math.min(130, Math.abs(pathYaw) * 150);
        if (pathYaw > 0.06) {
          recLeft = Math.min(220, baseFwd + turnRate);
          recRight = Math.max(-90, baseFwd - turnRate);
        } else if (pathYaw < -0.06) {
          recRight = Math.min(220, baseFwd + turnRate);
          recLeft = Math.max(-90, baseFwd - turnRate);
        } else {
          // Aligned with waypoint: drive straight forward!
          recLeft = baseFwd + 20;
          recRight = baseFwd + 20;
        }
      }
    } else {
      // Clear path cruising: follow predicted path towards destination
      const trim = Math.min(65, pathYaw * 80);
      recLeft = Math.round(145 + trim);
      recRight = Math.round(145 - trim);
    }

    // Flank clearance safety guard: prevent inward turns into obstacles alongside chassis
    if (minLeftFlankDist < 0.26 && recLeft < recRight) {
      recLeft = Math.max(recLeft, 110);
      recRight = Math.min(recRight, 85);
    } else if (minRightFlankDist < 0.26 && recRight < recLeft) {
      recRight = Math.max(recRight, 110);
      recLeft = Math.min(recLeft, 85);
    }

    const angleDeg = Math.round((finalAngle * 180) / Math.PI);
    let pathStatus = 'LANE SWEEP';
    if (this.circulationDetector.isCirculating) {
      pathStatus = 'LOOP ESCAPE (AVOID CORRIDOR)';
    } else if (classification.type === 'WALL' && minForwardDist < 0.85) {
      pathStatus = `WALL (ZIGZAG 90° ${classification.flankClearSide})`;
    } else if (classification.type === 'OBSTACLE' && (isPathLatched || isObstacleAhead)) {
      pathStatus = `OBSTACLE (BYPASS ${classification.flankClearSide})`;
    } else if (goalPos) {
      pathStatus = 'FRONTIER (EXPLORE NEW AREA)';
    }

    return {
      isObstacleAhead,
      isPathLatched,
      classification,
      clearAngle: pathYaw, // Heading error to the lookahead waypoint
      pathYaw: pathYaw,
      recommendedPwm: { left: recLeft, right: recRight },
      pathStatus,
      localPath,
      worldPath,
      sectorClearances,
      activeCorridor: this.activeCorridor,
      isCirculating: this.circulationDetector.isCirculating,
      circulationCentroid: this.circulationDetector.circulationCentroid,
      frontierGoal: this.currentFrontierGoal,
      coveragePercent: this.explorationGrid.getCoveragePercentage()
    };
  }

  /**
   * Main render call for the 2D Ultrasonic Radar window.
   * @param {Object} data - Robot telemetry and tracking state
   * @returns {Object} Avoidance path details
   */
  update(data) {
    if (!this.canvas || !this.ctx) return null;

    const {
      robotPos,
      robotYaw = 0,
      distF = 300,
      distL = 300,
      distR = 300,
      hasFire = false,
      target = null,
      isSpraying = false,
      robotState = 'SEARCHING',
      goalPos = null
    } = data;

    const width = this.canvas.width;
    const height = this.canvas.height;
    const ctx = this.ctx;

    // Origin: bottom-center of the radar screen
    const cx = width / 2;
    const cy = height - 22;
    const meterToPx = (height - 40) / this.maxRangeMeters;

    // Convert cm to meters
    const mF = Math.min(this.maxRangeMeters, (distF || 300) / 100);
    const mL = Math.min(this.maxRangeMeters, (distL || 300) / 100);
    const mR = Math.min(this.maxRangeMeters, (distR || 300) / 100);

    const now = Date.now();
    this.pulseAnim = (this.pulseAnim + 0.05) % (Math.PI * 2);

    // -------------------------------------------------------------------------
    // 1. Maintain Exploration Grid & Circulation Loop Detector
    // -------------------------------------------------------------------------
    if (robotPos) {
      // 1.1 Update robot footprint in persistent exploration grid
      this.explorationGrid.recordRobotPose(robotPos.x, robotPos.z);

      const beams = [
        { dist: (distF || 300) / 100, angle: 0 },
        { dist: (distL || 300) / 100, angle: -Math.PI / 4 },
        { dist: (distR || 300) / 100, angle: Math.PI / 4 }
      ];

      // 1.2 Raycast ultrasonic beams to carve free space & mark obstacles
      beams.forEach(b => {
        if (b.dist > 0.05 && b.dist < 3.2) {
          const beamWorldAngle = robotYaw + b.angle;
          this.explorationGrid.updateBeam(robotPos.x, robotPos.z, beamWorldAngle, b.dist);
        }
      });

      // 1.3 Localized point cloud for reactive obstacle bypass
      if (now - this.lastPointAdd > 80) {
        this.lastPointAdd = now;
        beams.forEach(b => {
          if (b.dist > 0.05 && b.dist < 3.0) {
            const beamWorldAngle = robotYaw + b.angle;
            const hitX = robotPos.x + Math.sin(beamWorldAngle) * b.dist;
            const hitZ = robotPos.z + Math.cos(beamWorldAngle) * b.dist;
            this.pointCloud.push({ worldX: hitX, worldZ: hitZ, time: now });
          }
        });
      }

      // 1.4 Path History & Loop / Circulation Detection
      this.circulationDetector.update(robotPos, robotYaw, now);

      // 1.5 Update Frontier Goal (every 750ms or if none active)
      if (now - this.lastFrontierCompute > 750 || !this.currentFrontierGoal) {
        this.lastFrontierCompute = now;
        if (this.circulationDetector.isCirculating) {
          // Select frontier outside the trapped loop exclusion zone!
          this.currentFrontierGoal = this.explorationGrid.findFrontierGoals(
            robotPos,
            this.circulationDetector.circulationCentroid,
            this.circulationDetector.circulationCentroid ? this.circulationDetector.circulationCentroid.radius : 1.8
          );
        } else {
          this.currentFrontierGoal = this.explorationGrid.findFrontierGoals(robotPos);
        }
      }
    }

    // Prune stale point cloud points
    this.pointCloud = this.pointCloud.filter(p => now - p.time <= this.maxPointAgeMs);

    // If searching (no fire) and a frontier goal exists, route towards it
    const effectiveGoalPos = (hasFire || (target && target.confidence > 0.4))
      ? null
      : (goalPos || this.currentFrontierGoal);

    // Predict stable, destination-directed avoidance path from mapping
    const avoidance = this.findClearAvoidancePath(robotPos, robotYaw, distF, distL, distR, target, effectiveGoalPos, hasFire);
    this.latestAvoidance = avoidance;

    // -------------------------------------------------------------------------
    // 2. Clear & Draw Tactical Radar Background
    // -------------------------------------------------------------------------
    ctx.fillStyle = '#060a14';
    ctx.fillRect(0, 0, width, height);

    // Subtle grid lines
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.04)';
    ctx.lineWidth = 1;
    for (let x = 0; x < width; x += 25) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
    }
    for (let y = 0; y < height; y += 25) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }

    // -------------------------------------------------------------------------
    // 3. Concentric Range Rings (0.5m, 1.0m, 2.0m, 3.0m)
    // -------------------------------------------------------------------------
    const rings = [0.5, 1.0, 2.0, 3.0];
    ctx.lineWidth = 1;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.font = '9px "JetBrains Mono", monospace';

    rings.forEach(r => {
      const radiusPx = r * meterToPx;
      ctx.beginPath();
      ctx.arc(cx, cy, radiusPx, Math.PI, 0, false);
      if (r === 0.5) {
        // Red standoff suppression ring
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.35)';
      } else {
        ctx.strokeStyle = 'rgba(0, 240, 255, 0.15)';
      }
      ctx.stroke();

      // Ring label
      ctx.fillStyle = r === 0.5 ? 'rgba(239, 68, 68, 0.7)' : 'rgba(0, 240, 255, 0.45)';
      ctx.fillText(`${r.toFixed(1)}m`, cx, cy - radiusPx - 3);
    });

    // Radial angle guidelines (-45°, 0°, +45°)
    const angles = [-Math.PI / 4, 0, Math.PI / 4];
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.12)';
    angles.forEach(a => {
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      const endX = cx + Math.sin(a) * (3.0 * meterToPx);
      const endY = cy - Math.cos(a) * (3.0 * meterToPx);
      ctx.lineTo(endX, endY);
      ctx.stroke();
    });

    // -------------------------------------------------------------------------
    // 4. Draw Persistent Explored Free Space & SLAM Map
    // -------------------------------------------------------------------------
    if (robotPos) {
      const cosY = Math.cos(robotYaw);
      const sinY = Math.sin(robotYaw);

      // 4.1 Draw explored free space cells in radar view
      const grid = this.explorationGrid;
      const robotG = grid.worldToGrid(robotPos.x, robotPos.z);
      if (robotG) {
        const checkRad = Math.ceil(this.maxRangeMeters / grid.resolution);
        const minGx = Math.max(0, robotG.gx - checkRad);
        const maxGx = Math.min(grid.gridWidth - 1, robotG.gx + checkRad);
        const minGz = Math.max(0, robotG.gz - checkRad);
        const maxGz = Math.min(grid.gridHeight - 1, robotG.gz + checkRad);

        for (let gz = minGz; gz <= maxGz; gz++) {
          for (let gx = minGx; gx <= maxGx; gx++) {
            const state = grid.getCell(gx, gz);
            if (state === 0) continue; // Undiscovered

            const wpt = grid.gridToWorld(gx, gz);
            const dx = wpt.x - robotPos.x;
            const dz = wpt.z - robotPos.z;
            const localX = dx * cosY - dz * sinY;
            const localForward = dx * sinY + dz * cosY;

            if (localForward > -0.2 && localForward <= this.maxRangeMeters && Math.abs(localX) <= 2.2) {
              const px = cx + localX * meterToPx;
              const py = cy - localForward * meterToPx;
              const cellPx = grid.resolution * meterToPx;

              if (state === 1) {
                // Free space tile
                ctx.fillStyle = 'rgba(56, 189, 248, 0.08)';
                ctx.fillRect(px - cellPx / 2, py - cellPx / 2, cellPx, cellPx);
              } else if (state === 2) {
                // Occupied obstacle cell
                ctx.fillStyle = 'rgba(0, 240, 255, 0.40)';
                ctx.fillRect(px - cellPx / 2, py - cellPx / 2, cellPx, cellPx);
              }
            }
          }
        }
      }

      // 4.2 Draw Breadcrumb Trajectory History Trail
      const history = this.circulationDetector.pathHistory;
      if (history.length > 1) {
        ctx.strokeStyle = 'rgba(0, 210, 255, 0.30)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([2, 3]);
        ctx.beginPath();
        let first = true;
        for (let i = 0; i < history.length; i++) {
          const pt = history[i];
          const dx = pt.x - robotPos.x;
          const dz = pt.z - robotPos.z;
          const lx = dx * cosY - dz * sinY;
          const lz = dx * sinY + dz * cosY;
          const px = cx + lx * meterToPx;
          const py = cy - lz * meterToPx;
          if (first) { ctx.moveTo(px, py); first = false; }
          else { ctx.lineTo(px, py); }
        }
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 4.3 Draw Obstacle Point Cloud
      this.pointCloud.forEach(p => {
        const age = now - p.time;
        const alpha = Math.max(0, 1 - age / this.maxPointAgeMs) * 0.75;
        const dx = p.worldX - robotPos.x;
        const dz = p.worldZ - robotPos.z;
        const localX = dx * cosY - dz * sinY;
        const localForward = dx * sinY + dz * cosY;

        if (localForward > 0 && localForward <= this.maxRangeMeters && Math.abs(localX) <= 2.2) {
          const px = cx + localX * meterToPx;
          const py = cy - localForward * meterToPx;
          ctx.fillStyle = `rgba(0, 240, 255, ${alpha})`;
          ctx.beginPath();
          ctx.arc(px, py, 2.2, 0, Math.PI * 2);
          ctx.fill();
        }
      });

      // 4.4 Draw Loop Exclusion Zone (if circulating)
      if (this.circulationDetector.isCirculating && this.circulationDetector.circulationCentroid) {
        const c = this.circulationDetector.circulationCentroid;
        const dx = c.x - robotPos.x;
        const dz = c.z - robotPos.z;
        const lx = dx * cosY - dz * sinY;
        const lz = dx * sinY + dz * cosY;
        const px = cx + lx * meterToPx;
        const py = cy - lz * meterToPx;
        const rPx = c.radius * meterToPx;

        ctx.strokeStyle = 'rgba(239, 68, 68, 0.75)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.arc(px, py, rPx, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = 'rgba(239, 68, 68, 0.85)';
        ctx.font = 'bold 8px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText('⚠️ LOOP ZONE', px, py);
      }

      // 4.5 Draw Frontier Goal Beacon
      if (this.currentFrontierGoal && !hasFire) {
        const fg = this.currentFrontierGoal;
        const dx = fg.x - robotPos.x;
        const dz = fg.z - robotPos.z;
        const lx = dx * cosY - dz * sinY;
        const lz = dx * sinY + dz * cosY;
        const dist = Math.hypot(lx, lz);

        const dispDist = Math.min(this.maxRangeMeters - 0.2, dist);
        const angle = Math.atan2(lx, lz);
        const px = cx + Math.sin(angle) * dispDist * meterToPx;
        const py = cy - Math.cos(angle) * dispDist * meterToPx;

        const pulse = 4 + Math.sin(this.pulseAnim * 4) * 1.5;
        ctx.fillStyle = '#f59e0b';
        ctx.beginPath();
        ctx.moveTo(px, py - pulse);
        ctx.lineTo(px + pulse, py);
        ctx.lineTo(px, py + pulse);
        ctx.lineTo(px - pulse, py);
        ctx.closePath();
        ctx.fill();

        ctx.fillStyle = '#fef08a';
        ctx.font = 'bold 8px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`📍 ${Math.round(dist * 100)}cm`, px, py - pulse - 3);
      }
    }

    // -------------------------------------------------------------------------
    // 5. Draw 3 Ultrasonic Sensor Cones (Front, Left, Right)
    // -------------------------------------------------------------------------
    const beamConfigs = [
      { name: 'F', dist: mF, angle: 0, rawCm: distF },
      { name: 'L', dist: mL, angle: -Math.PI / 4, rawCm: distL },
      { name: 'R', dist: mR, angle: Math.PI / 4, rawCm: distR }
    ];

    const coneHalfAngle = 0.13; // ~7.5 degrees half-width (15 deg cone)

    beamConfigs.forEach(b => {
      const distPx = b.dist * meterToPx;
      const startAngle = -Math.PI / 2 + b.angle - coneHalfAngle;
      const endAngle = -Math.PI / 2 + b.angle + coneHalfAngle;

      // Color based on proximity
      let beamColor = 'rgba(0, 240, 255, 0.18)';
      let arcColor = 'rgba(0, 240, 255, 0.85)';
      if (b.rawCm < 50) {
        beamColor = 'rgba(239, 68, 68, 0.28)';
        arcColor = 'rgba(239, 68, 68, 0.95)';
      } else if (b.rawCm < 90) {
        beamColor = 'rgba(245, 158, 11, 0.22)';
        arcColor = 'rgba(245, 158, 11, 0.90)';
      }

      // Draw cone area
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, distPx, startAngle, endAngle);
      ctx.closePath();
      ctx.fillStyle = beamColor;
      ctx.fill();

      // Draw wavefront arc at detected distance
      ctx.beginPath();
      ctx.arc(cx, cy, distPx, startAngle, endAngle);
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = arcColor;
      ctx.stroke();

      // Hit point blip
      const hitX = cx + Math.sin(b.angle) * distPx;
      const hitY = cy - Math.cos(b.angle) * distPx;
      ctx.fillStyle = arcColor;
      ctx.beginPath();
      ctx.arc(hitX, hitY, 3.5, 0, Math.PI * 2);
      ctx.fill();
    });

    // -------------------------------------------------------------------------
    // 6. Draw Extinguisher Spray Arc (Active Solenoid)
    // -------------------------------------------------------------------------
    if (isSpraying) {
      const sprayRange = 1.35 * meterToPx;
      const sprayAngle = 0.28; // ~32 deg total spread
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, sprayRange, -Math.PI / 2 - sprayAngle, -Math.PI / 2 + sprayAngle);
      ctx.closePath();

      const sprayGrad = ctx.createRadialGradient(cx, cy, 5, cx, cy, sprayRange);
      sprayGrad.addColorStop(0, 'rgba(0, 240, 255, 0.55)');
      sprayGrad.addColorStop(0.7, 'rgba(147, 197, 253, 0.35)');
      sprayGrad.addColorStop(1, 'rgba(255, 255, 255, 0.05)');

      ctx.fillStyle = sprayGrad;
      ctx.fill();

      // Animated droplets
      ctx.fillStyle = '#ffffff';
      for (let i = 0; i < 6; i++) {
        const randDist = (0.3 + Math.random() * 1.0) * meterToPx;
        const randA = (Math.random() - 0.5) * sprayAngle * 1.8;
        const dx = cx + Math.sin(randA) * randDist;
        const dy = cy - Math.cos(randA) * randDist;
        ctx.fillRect(dx - 1, dy - 1, 2, 2);
      }
    }

    // -------------------------------------------------------------------------
    // 7. Draw Detected Fire Target & Standoff Ring
    // -------------------------------------------------------------------------
    if (hasFire && target) {
      const relYaw = (target.x - 0.50) * 1.13;
      const targetDistM = (target.distanceCm && target.distanceCm > 0)
        ? (target.distanceCm / 100)
        : Math.min(2.5, mF);

      const tDistPx = Math.min(this.maxRangeMeters, targetDistM) * meterToPx;
      const firePx = cx + Math.sin(relYaw) * tDistPx;
      const firePy = cy - Math.cos(relYaw) * tDistPx;

      // Pulsing flame marker
      const pulseSize = 6 + Math.sin(this.pulseAnim * 3) * 2;
      ctx.fillStyle = 'rgba(239, 68, 68, 0.35)';
      ctx.beginPath();
      ctx.arc(firePx, firePy, pulseSize + 4, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#f97316';
      ctx.beginPath();
      ctx.arc(firePx, firePy, pulseSize, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#fef08a';
      ctx.beginPath();
      ctx.arc(firePx, firePy, pulseSize * 0.5, 0, Math.PI * 2);
      ctx.fill();

      // Flame Text & Distance
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 9px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      const distStr = `${Math.round(targetDistM * 100)}cm`;
      ctx.fillText(`🔥 ${distStr}`, firePx, firePy - pulseSize - 4);
    }

    // -------------------------------------------------------------------------
    // 8. Draw PREDICTED AVOIDANCE PATH TRAJECTORY from Mapping
    // -------------------------------------------------------------------------
    if (avoidance && avoidance.localPath && avoidance.localPath.length > 1) {
      ctx.save();
      ctx.lineWidth = 2.5;

      const isAvoid = avoidance.isObstacleAhead;
      const pathColor = isAvoid ? '#10b981' : (hasFire ? '#f59e0b' : '#00f0ff');
      ctx.strokeStyle = pathColor;

      // Draw dashed trajectory line
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      avoidance.localPath.forEach((pt, idx) => {
        const px = cx + pt.x * meterToPx;
        const py = cy - pt.y * meterToPx;
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.setLineDash([]); // Reset line dash

      // Waypoint beads along trajectory
      avoidance.localPath.forEach((pt, idx) => {
        if (idx === 0) return;
        const px = cx + pt.x * meterToPx;
        const py = cy - pt.y * meterToPx;

        ctx.fillStyle = pathColor;
        ctx.beginPath();
        ctx.arc(px, py, 2.5, 0, Math.PI * 2);
        ctx.fill();
      });

      // Arrowhead at trajectory tip
      const lastPt = avoidance.localPath[avoidance.localPath.length - 1];
      const prevPt = avoidance.localPath[avoidance.localPath.length - 2];
      const tipPx = cx + lastPt.x * meterToPx;
      const tipPy = cy - lastPt.y * meterToPx;

      const pathAngle = Math.atan2(lastPt.x - prevPt.x, lastPt.y - prevPt.y);
      ctx.save();
      ctx.translate(tipPx, tipPy);
      ctx.rotate(-pathAngle);
      ctx.fillStyle = pathColor;
      ctx.beginPath();
      ctx.moveTo(0, -6);
      ctx.lineTo(-4, 2);
      ctx.lineTo(4, 2);
      ctx.closePath();
      ctx.fill();
      ctx.restore();

      ctx.restore();
    }

    // -------------------------------------------------------------------------
    // 9. Draw Robot Footprint at Origin
    // -------------------------------------------------------------------------
    ctx.save();
    ctx.translate(cx, cy);

    // Chassis body (0.30m length x 0.22m width)
    const bodyW = 0.22 * meterToPx;
    const bodyH = 0.30 * meterToPx;

    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.roundRect(-bodyW / 2, -bodyH / 2, bodyW, bodyH, 3);
    ctx.fill();
    ctx.stroke();

    // Front Direction Pointer (Arrow)
    ctx.fillStyle = '#00f0ff';
    ctx.beginPath();
    ctx.moveTo(0, -bodyH / 2 - 4);
    ctx.lineTo(-4, -bodyH / 2 + 3);
    ctx.lineTo(4, -bodyH / 2 + 3);
    ctx.closePath();
    ctx.fill();

    // 4 Wheels
    ctx.fillStyle = '#475569';
    const wheelW = 4;
    const wheelH = 8;
    ctx.fillRect(-bodyW / 2 - wheelW + 1, -bodyH / 2 + 2, wheelW, wheelH); // FL
    ctx.fillRect(bodyW / 2 - 1, -bodyH / 2 + 2, wheelW, wheelH);           // FR
    ctx.fillRect(-bodyW / 2 - wheelW + 1, bodyH / 2 - 10, wheelW, wheelH); // RL
    ctx.fillRect(bodyW / 2 - 1, bodyH / 2 - 10, wheelW, wheelH);          // RR

    ctx.restore();

    // -------------------------------------------------------------------------
    // 10. Top Telemetry Metrics Header
    // -------------------------------------------------------------------------
    ctx.fillStyle = 'rgba(6, 10, 20, 0.85)';
    ctx.fillRect(0, 0, width, 18);
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.2)';
    ctx.beginPath();
    ctx.moveTo(0, 18);
    ctx.lineTo(width, 18);
    ctx.stroke();

    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.textAlign = 'left';

    const getDistColor = (cm) => {
      const inVal = cm * 0.393701;
      if (inVal < COLLISION_EMERGENCY_INCHES) return '#ef4444';
      if (inVal < COLLISION_TRIGGER_INCHES) return '#f59e0b';
      return '#00f0ff';
    };

    const toIn = (cm) => `${Math.round(cm * 0.393701)}"`;

    ctx.fillStyle = '#94a3b8';
    ctx.fillText('L:', 4, 12);
    ctx.fillStyle = getDistColor(distL);
    ctx.fillText(`${toIn(distL)}`, 16, 12);

    ctx.fillStyle = '#94a3b8';
    ctx.fillText('F:', 46, 12);
    ctx.fillStyle = getDistColor(distF);
    ctx.fillText(`${toIn(distF)}`, 58, 12);

    ctx.fillStyle = '#94a3b8';
    ctx.fillText('R:', 88, 12);
    ctx.fillStyle = getDistColor(distR);
    ctx.fillText(`${toIn(distR)}`, 100, 12);

    // Collision limit badge in inches
    ctx.fillStyle = '#f59e0b';
    ctx.fillText(`COL:10"`, 134, 12);

    // Map Coverage Percentage
    const cov = this.explorationGrid.getCoveragePercentage();
    ctx.fillStyle = '#38bdf8';
    ctx.fillText(`MAP:${cov}%`, 182, 12);

    // State badge on top right
    ctx.textAlign = 'right';
    if (isSpraying || robotState === 'EXTINGUISHING') {
      ctx.fillStyle = '#ef4444';
      ctx.fillText('EXTINGUISH', width - 4, 12);
    } else if (this.circulationDetector.isCirculating) {
      ctx.fillStyle = '#f59e0b';
      ctx.fillText('⚠️ LOOP ESCAPE', width - 4, 12);
    } else if (avoidance && avoidance.isObstacleAhead) {
      ctx.fillStyle = '#10b981';
      ctx.fillText(avoidance.pathStatus.replace('AVOID ', 'BYPASS '), width - 4, 12);
    } else if (hasFire) {
      ctx.fillStyle = '#f59e0b';
      ctx.fillText('FIRE LOCK', width - 4, 12);
    } else if (this.currentFrontierGoal) {
      ctx.fillStyle = '#00f0ff';
      ctx.fillText('FRONTIER', width - 4, 12);
    } else {
      ctx.fillStyle = '#00f0ff';
      ctx.fillText('CLEAR', width - 4, 12);
    }

    return avoidance;
  }
}

if (typeof window !== 'undefined') {
  window.UltrasonicMapper = UltrasonicMapper;
}
