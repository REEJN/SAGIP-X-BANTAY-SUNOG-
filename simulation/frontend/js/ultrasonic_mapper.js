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
    if (mF >= 1.05 && mL >= 0.85 && mR >= 0.85) {
      return { type: 'CLEAR', spanX: 0, flankClearSide: (mR >= mL) ? 'RIGHT' : 'LEFT' };
    }

    // Filter obstacles located in the forward approach zone (Z in [0.05, 1.35m], |X| < 2.0m)
    const forwardObs = localObstacles.filter(o => o.z > 0.05 && o.z < 1.35 && Math.abs(o.x) < 2.0);
    let spanX = 0;
    if (forwardObs.length > 1) {
      const xs = forwardObs.map(o => o.x);
      spanX = Math.max(...xs) - Math.min(...xs);
    } else if (forwardObs.length === 1) {
      spanX = 0.40;
    }

    const flankClearSide = (mR >= mL) ? 'RIGHT' : 'LEFT';
    const isFlankClear = (mL > 1.10 || mR > 1.10);
    const flankRatio = Math.abs(mL - mR) / Math.max(0.1, Math.max(mL, mR));

    // Discrete Obstacle (Crate / Barrel): Narrow lateral span (<0.85m) and open flank
    if (spanX < 0.85 && (isFlankClear || flankRatio > 0.35)) {
      return { type: 'OBSTACLE', spanX, flankClearSide };
    }

    // Continuous surface, wide span (>=1.0m), or tightly constrained corridor => WALL
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
        if (lz > 0.05 && lz < 2.8 && Math.abs(lx) < 2.2) {
          localObstacles.push({ x: lx, z: lz, weight: 1.0 });
        }
      });
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
          if (lz > 0.05 && lz < 2.8 && Math.abs(lx) < 2.4) {
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
          if (lz > 0.05 && lz < 2.8 && Math.abs(lx) < 2.4) {
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
          if (lz > 0.05 && lz < 2.8 && Math.abs(lx) < 2.4) {
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
          if (lz > 0.05 && lz < 2.8 && Math.abs(lx) < 2.4) {
            localObstacles.push({ x: lx, z: lz, weight: 3.0 });
          }
        }
      }
    }

    // 3. Check if forward collision corridor is blocked (with Schmitt-trigger hysteresis)
    const forwardCorridorHalfW = 0.28;
    let minForwardDist = mF;

    for (const obs of localObstacles) {
      if (obs.z > 0.08 && Math.abs(obs.x) < forwardCorridorHalfW) {
        if (obs.z < minForwardDist) minForwardDist = obs.z;
      }
    }

    // Schmitt-trigger hysteresis:
    // Turn ON obstacle avoidance at < 0.85m; Turn OFF ONLY when path ahead is clear > 1.15m!
    if (!this.isObstacleAheadState) {
      if (minForwardDist < 0.85) {
        this.isObstacleAheadState = true;
      }
    } else {
      if (minForwardDist > 1.15) {
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

    sectorAngles.forEach(angle => {
      const dirX = Math.sin(angle);
      const dirZ = Math.cos(angle);

      // Raycast along (dirX, dirZ) out to 2.5m
      let clearance = 2.5;

      for (const obs of localObstacles) {
        const proj = obs.x * dirX + obs.z * dirZ;
        if (proj > 0.10 && proj < clearance) {
          const perpDist = Math.abs(obs.x * dirZ - obs.z * dirX);
          if (perpDist < forwardCorridorHalfW) {
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
      const corridorClear = (this.clearCorridorFrames > 25 && elapsed > 800);
      const timedOut = (elapsed > 4500);
      const blockedAhead = (minForwardDist < 0.25);

      if (reachedEnd || corridorClear || timedOut || blockedAhead) {
        this.latchedWorldPath = null;
        this.clearCorridorFrames = 0;
      }
    }

    if (!this.latchedWorldPath) {
      if (classification.type === 'WALL' && minForwardDist < 0.75) {
        // Wall Encountered: Generate 90deg lane-turn arc into adjacent sweep lane
        const turnSign = (classification.flankClearSide === 'RIGHT') ? 1 : -1;
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

        const pathLength = 1.6;
        const numWaypoints = 10;
        for (let i = 1; i <= numWaypoints; i++) {
          const frac = i / numWaypoints;
          const s = frac * pathLength;

          let currentYaw;
          if (frac <= 0.5) {
            currentYaw = finalAngle * (frac / 0.5);
          } else {
            const blend = (frac - 0.5) / 0.5;
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
        // Clear cruise: Straight-line lane sweep path
        localPath.push({ x: 0, y: 0 });
        worldPath.push({ x: robotPos.x, z: robotPos.z });

        const targetWorldBearing = (isFireActive && target && target.x !== undefined)
          ? robotYaw + (target.x - 0.50) * 1.13
          : robotYaw;

        const cruiseLength = 1.8;
        const numWaypoints = 10;
        for (let i = 1; i <= numWaypoints; i++) {
          const s = (i / numWaypoints) * cruiseLength;
          const wx = robotPos.x + Math.sin(targetWorldBearing) * s;
          const wz = robotPos.z + Math.cos(targetWorldBearing) * s;
          worldPath.push({ x: wx, z: wz });

          const dx = wx - robotPos.x;
          const dz = wz - robotPos.z;
          const lx = dx * cosY - dz * sinY;
          const lz = dx * sinY + dz * cosY;
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

    const angleDeg = Math.round((finalAngle * 180) / Math.PI);
    let pathStatus = 'LANE SWEEP';
    if (classification.type === 'WALL' && minForwardDist < 0.85) {
      pathStatus = `WALL (ZIGZAG 90° ${classification.flankClearSide})`;
    } else if (classification.type === 'OBSTACLE' && (isPathLatched || isObstacleAhead)) {
      pathStatus = `OBSTACLE (BYPASS ${classification.flankClearSide})`;
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
      activeCorridor: this.activeCorridor
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
    // 1. Maintain World-Relative Point Cloud
    // -------------------------------------------------------------------------
    if (robotPos && now - this.lastPointAdd > 80) {
      this.lastPointAdd = now;
      const beams = [
        { dist: (distF || 300) / 100, angle: 0 },
        { dist: (distL || 300) / 100, angle: -Math.PI / 4 },
        { dist: (distR || 300) / 100, angle: Math.PI / 4 }
      ];

      beams.forEach(b => {
        if (b.dist > 0.05 && b.dist < 3.0) {
          const beamWorldAngle = robotYaw + b.angle;
          const hitX = robotPos.x + Math.sin(beamWorldAngle) * b.dist;
          const hitZ = robotPos.z + Math.cos(beamWorldAngle) * b.dist;
          this.pointCloud.push({ worldX: hitX, worldZ: hitZ, time: now });
        }
      });
    }

    // Prune stale points
    this.pointCloud = this.pointCloud.filter(p => now - p.time <= this.maxPointAgeMs);

    // Predict stable, destination-directed avoidance path from mapping
    const avoidance = this.findClearAvoidancePath(robotPos, robotYaw, distF, distL, distR, target, goalPos, hasFire);
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
    // 4. Draw Projected Obstacle Point Cloud (SLAM Memory)
    // -------------------------------------------------------------------------
    if (robotPos && this.pointCloud.length > 0) {
      const cosY = Math.cos(robotYaw);
      const sinY = Math.sin(robotYaw);

      this.pointCloud.forEach(p => {
        const age = now - p.time;
        const alpha = Math.max(0, 1 - age / this.maxPointAgeMs) * 0.75;

        // Transform world coordinates into local ego-centric coordinates
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
      if (cm < 50) return '#ef4444';
      if (cm < 90) return '#f59e0b';
      return '#00f0ff';
    };

    ctx.fillStyle = '#94a3b8';
    ctx.fillText('L:', 6, 12);
    ctx.fillStyle = getDistColor(distL);
    ctx.fillText(`${Math.round(distL)}cm`, 18, 12);

    ctx.fillStyle = '#94a3b8';
    ctx.fillText('F:', 85, 12);
    ctx.fillStyle = getDistColor(distF);
    ctx.fillText(`${Math.round(distF)}cm`, 97, 12);

    ctx.fillStyle = '#94a3b8';
    ctx.fillText('R:', 160, 12);
    ctx.fillStyle = getDistColor(distR);
    ctx.fillText(`${Math.round(distR)}cm`, 172, 12);

    // State badge on top right
    ctx.textAlign = 'right';
    if (isSpraying || robotState === 'EXTINGUISHING') {
      ctx.fillStyle = '#ef4444';
      ctx.fillText('EXTINGUISH', width - 6, 12);
    } else if (avoidance && avoidance.isObstacleAhead) {
      ctx.fillStyle = '#10b981';
      ctx.fillText(avoidance.pathStatus.replace('AVOID ', 'BYPASS '), width - 6, 12);
    } else if (hasFire) {
      ctx.fillStyle = '#f59e0b';
      ctx.fillText('FIRE LOCK', width - 6, 12);
    } else {
      ctx.fillStyle = '#00f0ff';
      ctx.fillText('PATH: CLEAR', width - 6, 12);
    }

    return avoidance;
  }
}

if (typeof window !== 'undefined') {
  window.UltrasonicMapper = UltrasonicMapper;
}
