"""
BANTAY-SUNOG Kinematic Arena Simulation Environment
Mirrors Three.js simulation world (robot_model.js, sensor_simulator.js, obstacle_manager.js)
Provides 2D/3D physics, raycasting, and obstacle/fire tracking.
"""

import math
import random
from typing import Dict, List, Optional, Tuple
import numpy as np


class Obstacle:
    """Base class for arena obstacles."""
    def __init__(self, x: float, z: float, obs_type: str):
        self.x = x
        self.z = z
        self.obs_type = obs_type


class BoxObstacle(Obstacle):
    def __init__(self, x: float, z: float, width: float, depth: float, yaw: float = 0.0):
        super().__init__(x, z, "box")
        self.width = width
        self.depth = depth
        self.yaw = yaw


class CylinderObstacle(Obstacle):
    def __init__(self, x: float, z: float, radius: float):
        super().__init__(x, z, "cylinder")
        self.radius = radius


def ray_intersect_segment(
    ro_x: float, ro_z: float, rd_x: float, rd_z: float,
    p1_x: float, p1_z: float, p2_x: float, p2_z: float
) -> Optional[float]:
    """Ray vs 2D line segment intersection. Returns distance along ray or None."""
    v1_x = ro_x - p1_x
    v1_z = ro_z - p1_z
    v2_x = p2_x - p1_x
    v2_z = p2_z - p1_z
    v3_x = -rd_z
    v3_z = rd_x

    dot = v2_x * v3_x + v2_z * v3_z
    if abs(dot) < 1e-8:
        return None

    t1 = (v2_x * v1_z - v2_z * v1_x) / dot
    t2 = (v1_x * v3_x + v1_z * v3_z) / dot

    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return t1
    return None


def ray_intersect_circle(
    ro_x: float, ro_z: float, rd_x: float, rd_z: float,
    cx: float, cz: float, radius: float
) -> Optional[float]:
    """Ray vs 2D circle intersection. Returns distance to closest intersection or None."""
    ox = ro_x - cx
    oz = ro_z - cz

    a = rd_x * rd_x + rd_z * rd_z
    b = 2.0 * (ox * rd_x + oz * rd_z)
    c = ox * ox + oz * oz - radius * radius

    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None

    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)

    if t1 >= 0.0:
        return t1
    if t2 >= 0.0:
        return t2
    return None


def ray_intersect_obb(
    ro_x: float, ro_z: float, rd_x: float, rd_z: float,
    cx: float, cz: float, width: float, depth: float, yaw: float
) -> Optional[float]:
    """Ray vs Oriented Bounding Box intersection using slab method in local space."""
    # Transform ray to local box coordinates
    cos_a = math.cos(-yaw)
    sin_a = math.sin(-yaw)

    dx = ro_x - cx
    dz = ro_z - cz
    loc_ro_x = dx * cos_a - dz * sin_a
    loc_ro_z = dx * sin_a + dz * cos_a

    loc_rd_x = rd_x * cos_a - rd_z * sin_a
    loc_rd_z = rd_x * sin_a + rd_z * cos_a

    half_w = width / 2.0
    half_d = depth / 2.0

    tmin = -1e9
    tmax = 1e9

    # X slab
    if abs(loc_rd_x) > 1e-8:
        tx1 = (-half_w - loc_ro_x) / loc_rd_x
        tx2 = (half_w - loc_ro_x) / loc_rd_x
        tmin = max(tmin, min(tx1, tx2))
        tmax = min(tmax, max(tx1, tx2))
    elif abs(loc_ro_x) > half_w:
        return None

    # Z slab
    if abs(loc_rd_z) > 1e-8:
        tz1 = (-half_d - loc_ro_z) / loc_rd_z
        tz2 = (half_d - loc_ro_z) / loc_rd_z
        tmin = max(tmin, min(tz1, tz2))
        tmax = min(tmax, max(tz1, tz2))
    elif abs(loc_ro_z) > half_d:
        return None

    if tmax >= tmin and tmax >= 0.0:
        return max(0.0, tmin)
    return None


class ArenaSim:
    """
    2D/3D kinematic simulation of the BANTAY-SUNOG robot in an arena.
    """
    def __init__(self, arena_size: float = 9.0):
        self.arena_size = arena_size
        self.half_arena = arena_size / 2.0
        self.wall_limit = 4.24  # 4.5m - 0.075m wall - 0.18m robot radius

        # Robot kinematic specs (matches robot_model.js)
        self.track_width = 0.24      # m
        self.wheel_radius = 0.042    # m
        self.max_speed = 0.65        # m/s
        self.max_turn_rate = 2.8     # rad/s
        self.min_pwm = 70.0          # pwm threshold
        self.max_linear_acc = 2.6    # m/s^2
        self.max_angular_acc = 10.5  # rad/s^2
        self.robot_radius = 0.18     # m

        # State
        self.x = 0.0
        self.z = 0.0
        self.yaw = 0.0               # radians, 0 = forward +Z
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.left_pwm = 0.0
        self.right_pwm = 0.0

        # Obstacles and fire
        self.obstacles: List[Obstacle] = []
        self.fire_x: Optional[float] = None
        self.fire_z: Optional[float] = None
        self.has_fire: bool = False
        self.collision_count = 0
        self.is_colliding = False

    def reset_robot(self, x: float = 0.0, z: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.z = z
        self.yaw = yaw
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.left_pwm = 0.0
        self.right_pwm = 0.0
        self.collision_count = 0
        self.is_colliding = False

    def clear_obstacles(self):
        self.obstacles.clear()

    def add_box(self, x: float, z: float, width: float = 0.7, depth: float = 0.7, yaw: float = 0.0):
        self.obstacles.append(BoxObstacle(x, z, width, depth, yaw))

    def add_cylinder(self, x: float, z: float, radius: float = 0.3):
        self.obstacles.append(CylinderObstacle(x, z, radius))

    def set_fire(self, x: float, z: float):
        self.fire_x = x
        self.fire_z = z
        self.has_fire = True

    def clear_fire(self):
        self.fire_x = None
        self.fire_z = None
        self.has_fire = False

    def randomize_arena(self, num_crates: int = 4, num_barrels: int = 3, add_fire: bool = True):
        """Randomizes layout with non-overlapping obstacles and a fire target."""
        self.clear_obstacles()
        self.clear_fire()

        # Place robot near center with random orientation
        self.reset_robot(
            x=random.uniform(-1.5, 1.5),
            z=random.uniform(-1.5, 1.5),
            yaw=random.uniform(-math.pi, math.pi)
        )

        # Helper to check distance from existing objects and robot
        def is_safe_pos(px: float, pz: float, safe_r: float = 0.8) -> bool:
            if math.hypot(px - self.x, pz - self.z) < safe_r + 0.3:
                return False
            for obs in self.obstacles:
                if math.hypot(px - obs.x, pz - obs.z) < safe_r + 0.6:
                    return False
            return True

        # Crates
        for _ in range(num_crates):
            for _ in range(25):
                ox = random.uniform(-3.2, 3.2)
                oz = random.uniform(-3.2, 3.2)
                if is_safe_pos(ox, oz, 0.7):
                    w = random.uniform(0.5, 0.8)
                    d = random.uniform(0.5, 0.8)
                    yaw = random.uniform(0, math.pi)
                    self.add_box(ox, oz, w, d, yaw)
                    break

        # Barrels
        for _ in range(num_barrels):
            for _ in range(25):
                ox = random.uniform(-3.2, 3.2)
                oz = random.uniform(-3.2, 3.2)
                if is_safe_pos(ox, oz, 0.5):
                    self.add_cylinder(ox, oz, radius=random.uniform(0.25, 0.35))
                    break

        # Fire
        if add_fire:
            for _ in range(30):
                fx = random.uniform(-3.5, 3.5)
                fz = random.uniform(-3.5, 3.5)
                # Fire must be at least 1.5m away from robot
                if math.hypot(fx - self.x, fz - self.z) > 1.5 and is_safe_pos(fx, fz, 0.6):
                    self.set_fire(fx, fz)
                    break

    def step(self, left_pwm: float, right_pwm: float, dt: float = 0.05) -> Tuple[np.ndarray, bool]:
        """
        Simulate one physics & kinematics time-step.
        
        Args:
            left_pwm: Motor command [-255, 255]
            right_pwm: Motor command [-255, 255]
            dt: Time step in seconds (default 0.05s / 20 Hz)
            
        Returns:
            (sensor_inputs_array, collision_occurred)
        """
        self.left_pwm = float(np.clip(left_pwm, -255.0, 255.0))
        self.right_pwm = float(np.clip(right_pwm, -255.0, 255.0))

        # Differential drive speed targets
        v_left = (self.left_pwm / 255.0) * self.max_speed if abs(self.left_pwm) >= self.min_pwm else 0.0
        v_right = (self.right_pwm / 255.0) * self.max_speed if abs(self.right_pwm) >= self.min_pwm else 0.0

        target_lin_speed = (v_left + v_right) / 2.0
        target_ang_speed = (v_left - v_right) / self.track_width

        # Linear acceleration / braking inertia
        lin_diff = target_lin_speed - self.current_linear_speed
        max_lin_step = self.max_linear_acc * dt
        if abs(lin_diff) <= max_lin_step:
            self.current_linear_speed = target_lin_speed
        else:
            self.current_linear_speed += math.copysign(max_lin_step, lin_diff)

        # Rotational acceleration / inertia
        ang_diff = target_ang_speed - self.current_angular_speed
        max_ang_step = self.max_angular_acc * dt
        if abs(ang_diff) <= max_ang_step:
            self.current_angular_speed = target_ang_speed
        else:
            self.current_angular_speed += math.copysign(max_ang_step, ang_diff)

        # Update heading
        self.yaw += self.current_angular_speed * dt
        # Normalize yaw to [-PI, +PI]
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # Desired forward translation
        fwd_x = math.sin(self.yaw)
        fwd_z = math.cos(self.yaw)

        next_x = self.x + fwd_x * self.current_linear_speed * dt
        next_z = self.z + fwd_z * self.current_linear_speed * dt

        # Collision resolution matching robot_model.js
        resolved_x = next_x
        resolved_z = next_z
        collision_detected = False

        # Check arena boundaries
        if resolved_x < -self.wall_limit or resolved_x > self.wall_limit or \
           resolved_z < -self.wall_limit or resolved_z > self.wall_limit:
            collision_detected = True
            resolved_x = max(-self.wall_limit, min(self.wall_limit, resolved_x))
            resolved_z = max(-self.wall_limit, min(self.wall_limit, resolved_z))

        # Check obstacles
        for obs in self.obstacles:
            if obs.obs_type == "cylinder":
                obs_cyl: CylinderObstacle = obs
                min_sep = self.robot_radius + obs_cyl.radius
                dx = resolved_x - obs_cyl.x
                dz = resolved_z - obs_cyl.z
                dist = math.hypot(dx, dz)
                if dist < min_sep:
                    collision_detected = True
                    push = min_sep - dist + 0.002
                    if dist > 1e-4:
                        resolved_x += (dx / dist) * push
                        resolved_z += (dz / dist) * push
                    else:
                        resolved_x += push

            elif obs.obs_type == "box":
                obs_box: BoxObstacle = obs
                half_w = obs_box.width / 2.0
                half_d = obs_box.depth / 2.0
                rot_y = obs_box.yaw

                dx = resolved_x - obs_box.x
                dz = resolved_z - obs_box.z

                # Rotate into box local frame
                cos_a = math.cos(-rot_y)
                sin_a = math.sin(-rot_y)
                loc_x = dx * cos_a - dz * sin_a
                loc_z = dx * sin_a + dz * cos_a

                # Closest point on box
                cl_x = max(-half_w, min(half_w, loc_x))
                cl_z = max(-half_d, min(half_d, loc_z))

                diff_x = loc_x - cl_x
                diff_z = loc_z - cl_z
                loc_dist = math.hypot(diff_x, diff_z)

                if loc_dist < self.robot_radius:
                    collision_detected = True
                    push_dist = self.robot_radius - loc_dist + 0.002
                    if loc_dist > 1e-4:
                        push_loc_x = (diff_x / loc_dist) * push_dist
                        push_loc_z = (diff_z / loc_dist) * push_dist
                    else:
                        push_loc_x = push_dist
                        push_loc_z = 0.0

                    # Rotate back to world frame
                    cos_b = math.cos(rot_y)
                    sin_b = math.sin(rot_y)
                    push_world_x = push_loc_x * cos_b - push_loc_z * sin_b
                    push_world_z = push_loc_x * sin_b + push_loc_z * cos_b

                    resolved_x += push_world_x
                    resolved_z += push_world_z

        # Re-clamp arena after push
        resolved_x = max(-self.wall_limit, min(self.wall_limit, resolved_x))
        resolved_z = max(-self.wall_limit, min(self.wall_limit, resolved_z))

        self.x = resolved_x
        self.z = resolved_z

        if collision_detected and not self.is_colliding:
            self.collision_count += 1
        self.is_colliding = collision_detected

        sensor_readings = self.get_sensor_readings()
        return sensor_readings, collision_detected

    def cast_ray(self, origin_x: float, origin_z: float, dir_x: float, dir_z: float, max_dist: float = 3.0) -> float:
        """Cast a 2D ray against all walls and obstacles. Returns closest hit distance."""
        min_hit = max_dist

        # 4 Arena walls
        walls = [
            (-self.half_arena, -self.half_arena, self.half_arena, -self.half_arena),  # South (Z = -4.5)
            (self.half_arena, -self.half_arena, self.half_arena, self.half_arena),    # East  (X = +4.5)
            (self.half_arena, self.half_arena, -self.half_arena, self.half_arena),    # North (Z = +4.5)
            (-self.half_arena, self.half_arena, -self.half_arena, -self.half_arena)   # West  (X = -4.5)
        ]
        for p1x, p1z, p2x, p2z in walls:
            hit = ray_intersect_segment(origin_x, origin_z, dir_x, dir_z, p1x, p1z, p2x, p2z)
            if hit is not None and hit < min_hit:
                min_hit = hit

        # Obstacles
        for obs in self.obstacles:
            if obs.obs_type == "cylinder":
                obs_cyl: CylinderObstacle = obs
                hit = ray_intersect_circle(origin_x, origin_z, dir_x, dir_z, obs_cyl.x, obs_cyl.z, obs_cyl.radius)
                if hit is not None and hit < min_hit:
                    min_hit = hit
            elif obs.obs_type == "box":
                obs_box: BoxObstacle = obs
                hit = ray_intersect_obb(origin_x, origin_z, dir_x, dir_z, obs_box.x, obs_box.z, obs_box.width, obs_box.depth, obs_box.yaw)
                if hit is not None and hit < min_hit:
                    min_hit = hit

        return min_hit

    def get_sensor_readings(self) -> Dict[str, float]:
        """
        Calculates the 7 sensor readings:
        - dist_front (m [0, 3.0])
        - dist_left (m [0, 3.0])
        - dist_right (m [0, 3.0])
        - target_rel_yaw (rad [-pi, pi])
        - target_distance (m)
        - has_fire_target (0.0 or 1.0)
        - heading_rate (rad/s)
        """
        # Sensor rays matching sensor_simulator.js
        # Front: 0 rad offset
        dir_f_x = math.sin(self.yaw)
        dir_f_z = math.cos(self.yaw)
        dist_f = self.cast_ray(self.x, self.z, dir_f_x, dir_f_z)

        # Left: -PI/4 offset (-45 deg toward -X)
        yaw_l = self.yaw - math.pi / 4.0
        dir_l_x = math.sin(yaw_l)
        dir_l_z = math.cos(yaw_l)
        dist_l = self.cast_ray(self.x, self.z, dir_l_x, dir_l_z)

        # Right: +PI/4 offset (+45 deg toward +X)
        yaw_r = self.yaw + math.pi / 4.0
        dir_r_x = math.sin(yaw_r)
        dir_r_z = math.cos(yaw_r)
        dist_r = self.cast_ray(self.x, self.z, dir_r_x, dir_r_z)

        # Fire sensing
        has_fire = 0.0
        target_rel_yaw = 0.0
        target_dist = 5.0

        if self.has_fire and self.fire_x is not None and self.fire_z is not None:
            dx = self.fire_x - self.x
            dz = self.fire_z - self.z
            dist_to_fire = math.hypot(dx, dz)
            fire_angle = math.atan2(dx, dz)
            rel_yaw = (fire_angle - self.yaw + math.pi) % (2.0 * math.pi) - math.pi

            # Camera vision cone: range <= 3.5m, FOV cone +/- 37 degrees (~0.65 rad)
            if dist_to_fire <= 3.5 and abs(rel_yaw) <= 0.65:
                # Line of sight check against obstacles
                dir_fire_x = dx / dist_to_fire
                dir_fire_z = dz / dist_to_fire
                hit_dist = self.cast_ray(self.x, self.z, dir_fire_x, dir_fire_z, max_dist=dist_to_fire)
                if hit_dist >= dist_to_fire - 0.2:  # Not occluded
                    has_fire = 1.0
                    target_rel_yaw = rel_yaw
                    target_dist = dist_to_fire

        return {
            "dist_front": dist_f,
            "dist_left": dist_l,
            "dist_right": dist_r,
            "target_rel_yaw": target_rel_yaw,
            "target_distance": target_dist,
            "has_fire_target": has_fire,
            "heading_rate": self.current_angular_speed
        }
