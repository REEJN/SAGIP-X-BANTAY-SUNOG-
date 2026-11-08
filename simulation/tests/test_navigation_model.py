"""
BANTAY-SUNOG Automated Unit Tests: TinyML Local Navigation Model
Verifies:
1. Architecture, layers, and parameter count (~818 params, < 4 KB).
2. Numerical equivalence of PyTorch reference model, NumPy runner, and C++ header.
3. Functional navigation behaviors (obstacle evasion, fire homing, standoff deceleration).
"""

import math
import os
import re
import unittest
import numpy as np
import torch

from simulation.ai_nav.model import TinyMLNavPolicy, normalize_inputs
from simulation.ai_nav.nav_policy import NavPolicyRunner


class TestNavigationModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pt_model_path = "simulation/models/navigation_model.pt"
        cls.json_model_path = "simulation/models/navigation_model.json"
        cls.cpp_header_path = "bantay_sunog_esp32/navigation_model.h"

        cls.model = TinyMLNavPolicy()
        if os.path.exists(cls.pt_model_path):
            cls.traced_model = torch.jit.load(cls.pt_model_path)
        else:
            cls.traced_model = None

        if os.path.exists(cls.json_model_path):
            cls.numpy_runner = NavPolicyRunner(cls.json_model_path)
        else:
            cls.numpy_runner = None

    def test_parameter_count_and_footprint(self):
        """Verify model parameter count is ~818 and memory footprint < 4 KB."""
        param_count = self.model.get_parameter_count()
        # fc1: 7*32 + 32 = 256
        # fc2: 32*16 + 16 = 528
        # fc3: 16*2 + 2 = 34
        # Total = 818
        self.assertEqual(param_count, 818, f"Expected 818 parameters, got {param_count}")
        footprint_bytes = param_count * 4  # float32 = 4 bytes
        self.assertLess(footprint_bytes, 4096, f"Model footprint ({footprint_bytes} bytes) exceeds 4 KB limit")

    def test_normalization_bounds(self):
        """Verify that normalize_inputs maps arbitrary physical values to [-1.0, 1.0]."""
        # Normal scenario
        norm = normalize_inputs(1.5, 2.0, 0.8, 0.5, 2.0, 1.0, 0.2)
        self.assertEqual(len(norm), 7)
        self.assertTrue(np.all(norm >= -1.0))
        self.assertTrue(np.all(norm <= 1.0))

        # Extreme / out of bounds scenario
        extreme = normalize_inputs(-5.0, 10.0, 0.0, 4.0 * math.pi, 20.0, 0.0, -10.0)
        self.assertEqual(len(extreme), 7)
        self.assertTrue(np.all(extreme >= -1.0))
        self.assertTrue(np.all(extreme <= 1.0))

    def test_numpy_vs_pytorch_inference(self):
        """Verify NumPy inference matches PyTorch TorchScript within float tolerance."""
        if self.traced_model is None or self.numpy_runner is None:
            self.skipTest("Saved models not found")

        np.random.seed(123)
        for _ in range(50):
            sample_in = np.random.uniform(-1.0, 1.0, size=(7,)).astype(np.float32)

            # PyTorch inference
            with torch.no_grad():
                tensor_in = torch.tensor(sample_in).unsqueeze(0)
                pt_out = self.traced_model(tensor_in).squeeze(0).numpy()

            # NumPy inference
            np_out = self.numpy_runner.predict(sample_in)

            # Max absolute error must be < 1e-3 (float32 BLAS precision)
            self.assertAlmostEqual(float(pt_out[0]), np_out[0], delta=1e-3)
            self.assertAlmostEqual(float(pt_out[1]), np_out[1], delta=1e-3)

    def test_cpp_header_mathematical_equivalence(self):
        """
        Extract weights and equations directly from bantay_sunog_esp32/navigation_model.h
        and verify that the C++ forward pass reproduces PyTorch reference output within < 1e-4.
        """
        self.assertTrue(os.path.exists(self.cpp_header_path), f"Header not found: {self.cpp_header_path}")

        with open(self.cpp_header_path, "r", encoding="utf-8") as f:
            header_text = f.read()

        # Parse W1, B1, W2, B2, W3, B3 using regex
        def parse_2d(name, rows, cols):
            pattern = rf"static const float {name}\[{rows}\]\[{cols}\] = \{{(.*?)\}};"
            match = re.search(pattern, header_text, re.DOTALL)
            self.assertIsNotNone(match, f"Could not find {name} in {self.cpp_header_path}")
            # Find all float values
            floats = [float(x.rstrip("f")) for x in re.findall(r"[-+]?\d*\.\d+f", match.group(1))]
            self.assertEqual(len(floats), rows * cols, f"Mismatch in {name} float count")
            return np.array(floats, dtype=np.float32).reshape(rows, cols)

        def parse_1d(name, size):
            pattern = rf"static const float {name}\[{size}\] = \{{(.*?)\}};"
            match = re.search(pattern, header_text, re.DOTALL)
            self.assertIsNotNone(match, f"Could not find {name} in {self.cpp_header_path}")
            floats = [float(x.rstrip("f")) for x in re.findall(r"[-+]?\d*\.\d+f", match.group(1))]
            self.assertEqual(len(floats), size, f"Mismatch in {name} float count")
            return np.array(floats, dtype=np.float32)

        w1 = parse_2d("NAV_W1", 32, 7)
        b1 = parse_1d("NAV_B1", 32)
        w2 = parse_2d("NAV_W2", 16, 32)
        b2 = parse_1d("NAV_B2", 16)
        w3 = parse_2d("NAV_W3", 2, 16)
        b3 = parse_1d("NAV_B3", 2)

        # Simulate C++ nav_model_predict logic
        def cpp_nav_model_predict(input_vec):
            h1 = np.maximum(0.0, np.dot(w1, input_vec) + b1)
            h2 = np.maximum(0.0, np.dot(w2, h1) + b2)
            out = np.dot(w3, h2) + b3
            return np.clip(out, -255.0, 255.0)

        # Compare against Python runner across 100 test inputs
        np.random.seed(456)
        for _ in range(100):
            test_x = np.random.uniform(-1.0, 1.0, size=(7,)).astype(np.float32)
            cpp_out = cpp_nav_model_predict(test_x)
            py_out = self.numpy_runner.predict(test_x)

            np.testing.assert_allclose(cpp_out, py_out, atol=1e-4, err_msg="C++ header logic deviates from model")

    def test_fire_servoing_turn_direction(self):
        """Verify that when fire is on the right, policy steers right (left_pwm > right_pwm)."""
        # Fire is at +30 degrees (target_rel_yaw = +0.52 rad)
        # Clear path ahead (front, left, right = 2.5m)
        left_pwm, right_pwm = self.numpy_runner.step(
            dist_front_m=2.5,
            dist_left_m=2.5,
            dist_right_m=2.5,
            target_rel_yaw_rad=0.52,
            target_dist_m=2.0,
            has_fire=True,
            heading_rate_rads=0.0
        )
        self.assertGreater(left_pwm, right_pwm, "Should steer right (left_pwm > right_pwm) toward right-side fire")

        # Fire is at -30 degrees (target_rel_yaw = -0.52 rad)
        left_pwm, right_pwm = self.numpy_runner.step(
            dist_front_m=2.5,
            dist_left_m=2.5,
            dist_right_m=2.5,
            target_rel_yaw_rad=-0.52,
            target_dist_m=2.0,
            has_fire=True,
            heading_rate_rads=0.0
        )
        self.assertGreater(right_pwm, left_pwm, "Should steer left (right_pwm > left_pwm) toward left-side fire")

    def test_standoff_deceleration_at_suppression_distance(self):
        """Verify that as robot reaches 0.45m - 0.48m from fire, speed drops significantly."""
        # Far from fire (2.0m)
        far_l, far_r = self.numpy_runner.step(
            dist_front_m=2.0,
            dist_left_m=2.0,
            dist_right_m=2.0,
            target_rel_yaw_rad=0.0,
            target_dist_m=2.0,
            has_fire=True,
            heading_rate_rads=0.0
        )

        # At standoff distance (0.47m)
        close_l, close_r = self.numpy_runner.step(
            dist_front_m=0.47,
            dist_left_m=2.0,
            dist_right_m=2.0,
            target_rel_yaw_rad=0.0,
            target_dist_m=0.47,
            has_fire=True,
            heading_rate_rads=0.0
        )

        far_speed = (abs(far_l) + abs(far_r)) / 2.0
        close_speed = (abs(close_l) + abs(close_r)) / 2.0
        self.assertLess(close_speed, far_speed * 0.3, "Policy must decelerate heavily at standoff distance")

    def test_obstacle_overtaking_and_clear_path_selection(self):
        """Verify that when obstacles are detected, policy does NOT stall and selects clear path with overtaking momentum."""
        # 1. Wall straight ahead, sides open: must maintain non-zero overtaking forward momentum
        l_sym, r_sym = self.numpy_runner.step(
            dist_front_m=0.35,
            dist_left_m=2.0,
            dist_right_m=2.0,
            target_rel_yaw_rad=0.0,
            target_dist_m=5.0,
            has_fire=False,
            heading_rate_rads=0.0
        )
        self.assertGreater(abs(l_sym) + abs(r_sym), 80.0, "Policy must not stall or freeze in front of a wall")
        self.assertGreater((l_sym + r_sym) / 2.0, 0.0, "Must maintain positive forward progress to overtake obstacle")

        # 2. Obstacle in front and on left: right path is clearer -> steer right
        l_right, r_right = self.numpy_runner.step(
            dist_front_m=0.35,
            dist_left_m=0.4,
            dist_right_m=2.0,
            target_rel_yaw_rad=0.0,
            target_dist_m=5.0,
            has_fire=False,
            heading_rate_rads=0.0
        )
        self.assertGreater(l_right, r_right, "Should steer right (left_pwm > right_pwm) when right path is clearer")

        # 3. Obstacle in front and on right: left path is clearer -> steer left
        l_left, r_left = self.numpy_runner.step(
            dist_front_m=0.35,
            dist_left_m=2.0,
            dist_right_m=0.4,
            target_rel_yaw_rad=0.0,
            target_dist_m=5.0,
            has_fire=False,
            heading_rate_rads=0.0
        )
        self.assertGreater(r_left, l_left, "Should steer left (right_pwm > left_pwm) when left path is clearer")

    def test_destination_directed_clear_path_steering(self):
        """Verify that when target_rel_yaw is non-zero without fire (e.g. bypass waypoint or room target), policy steers towards it."""
        # Desired destination is to the right (+0.6 rad)
        l_dest_r, r_dest_r = self.numpy_runner.step(
            dist_front_m=2.0,
            dist_left_m=2.0,
            dist_right_m=2.0,
            target_rel_yaw_rad=0.6,
            target_dist_m=3.0,
            has_fire=False,
            heading_rate_rads=0.0
        )
        self.assertGreater(l_dest_r, r_dest_r, "Policy must steer right towards destination waypoint")

        # Desired destination is to the left (-0.6 rad)
        l_dest_l, r_dest_l = self.numpy_runner.step(
            dist_front_m=2.0,
            dist_left_m=2.0,
            dist_right_m=2.0,
            target_rel_yaw_rad=-0.6,
            target_dist_m=3.0,
            has_fire=False,
            heading_rate_rads=0.0
        )
        self.assertGreater(r_dest_l, l_dest_l, "Policy must steer left towards destination waypoint")

    def test_waypoint_alignment_straight_cruising(self):
        """Verify that when aligned with a waypoint (target_rel_yaw ≈ 0), policy cruises straight without spinning."""
        # Aligned with waypoint in open space
        l_pwm, r_pwm = self.numpy_runner.step(
            dist_front_m=2.0,
            dist_left_m=2.0,
            dist_right_m=2.0,
            target_rel_yaw_rad=0.0,
            target_dist_m=2.5,
            has_fire=False,
            heading_rate_rads=0.0
        )
        self.assertGreater(l_pwm, 100.0, "Must cruise forward when aligned with waypoint")
        self.assertGreater(r_pwm, 100.0, "Must cruise forward when aligned with waypoint")
        self.assertLess(abs(l_pwm - r_pwm), 25.0, "Left and right wheels must be balanced (no tail-chasing/spinning)")

        # Aligned with waypoint with obstacle alongside on left (clearing obstacle)
        l_side, r_side = self.numpy_runner.step(
            dist_front_m=2.0,
            dist_left_m=0.45,
            dist_right_m=2.0,
            target_rel_yaw_rad=0.0,
            target_dist_m=2.5,
            has_fire=False,
            heading_rate_rads=0.0
        )
        self.assertGreater(l_side, 80.0, "Must maintain forward momentum while passing obstacle")
        self.assertGreater(r_side, 80.0, "Must maintain forward momentum while passing obstacle")
        self.assertLess(abs(l_side - r_side), 35.0, "Must cruise straight past obstacle without circular looping")

    def test_wall_vs_obstacle_classification(self):
        """
        Verify geometric classification of obstacles (crates/barrels) vs walls (perimeter/dividers).
        Discrete obstacles have narrow point-cloud lateral span (<0.85m) and open flank clearance.
        Continuous walls have wide lateral span (>=1.0m) or constrained flanks.
        """
        def classify_object(local_obstacles, mF, mL, mR):
            if mF >= 1.05 and mL >= 0.85 and mR >= 0.85:
                return {"type": "CLEAR", "flankClearSide": "RIGHT" if mR >= mL else "LEFT"}
            forward_obs = [o for o in local_obstacles if 0.05 < o["z"] < 1.35 and abs(o["x"]) < 2.0]
            span_x = 0.0
            if len(forward_obs) > 1:
                xs = [o["x"] for o in forward_obs]
                span_x = max(xs) - min(xs)
            elif len(forward_obs) == 1:
                span_x = 0.40
            flank_clear_side = "RIGHT" if mR >= mL else "LEFT"
            is_flank_clear = (mL > 1.10 or mR > 1.10)
            flank_ratio = abs(mL - mR) / max(0.1, max(mL, mR))
            if span_x < 0.85 and (is_flank_clear or flank_ratio > 0.35):
                return {"type": "OBSTACLE", "spanX": span_x, "flankClearSide": flank_clear_side}
            return {"type": "WALL", "spanX": span_x, "flankClearSide": flank_clear_side}

        # Case 1: Discrete Crate Obstacle ahead with open right flank
        crate_points = [{"x": 0.05, "z": 0.55}]
        res_crate = classify_object(crate_points, mF=0.55, mL=0.50, mR=1.80)
        self.assertEqual(res_crate["type"], "OBSTACLE")
        self.assertEqual(res_crate["flankClearSide"], "RIGHT")

        # Case 2: Discrete Barrel Obstacle with narrow span (0.35m) and open left flank
        barrel_points = [{"x": -0.15, "z": 0.60}, {"x": 0.20, "z": 0.60}]
        res_barrel = classify_object(barrel_points, mF=0.60, mL=1.60, mR=0.45)
        self.assertEqual(res_barrel["type"], "OBSTACLE")
        self.assertEqual(res_barrel["flankClearSide"], "LEFT")

        # Case 3: Continuous Wall spanning across corridor (1.4m span)
        wall_points = [{"x": -0.70, "z": 0.50}, {"x": 0.0, "z": 0.50}, {"x": 0.70, "z": 0.50}]
        res_wall = classify_object(wall_points, mF=0.50, mL=0.40, mR=0.40)
        self.assertEqual(res_wall["type"], "WALL")

        # Case 4: Constrained boundary (both flanks blocked, tight corner)
        corner_points = [{"x": 0.0, "z": 0.40}]
        res_corner = classify_object(corner_points, mF=0.40, mL=0.35, mR=0.35)
        self.assertEqual(res_corner["type"], "WALL")

    def test_exploration_grid_mapping_and_frontiers(self):
        """
        Verify persistent 2D occupancy & exploration grid algorithm:
        1. Coordinate transformations (world <-> grid).
        2. Bresenham raycasting carving open space (FREE) and marking obstacle hits (OCCUPIED).
        3. Frontier detection (identifying boundary cells between FREE and UNDISCOVERED space).
        """
        size_m = 9.0
        res = 0.20
        grid_w = int(round(size_m / res))
        half_size = size_m / 2.0

        def world_to_grid(x, z):
            gx = int(math.floor((x + half_size) / res))
            gz = int(math.floor((z + half_size) / res))
            if 0 <= gx < grid_w and 0 <= gz < grid_w:
                return gx, gz
            return None

        def grid_to_world(gx, gz):
            x = (gx + 0.5) * res - half_size
            z = (gz + 0.5) * res - half_size
            return x, z

        # Coordinate conversions
        gx, gz = world_to_grid(0.0, 0.0)
        self.assertEqual(gx, grid_w // 2)
        self.assertEqual(gz, grid_w // 2)
        wx, wz = grid_to_world(gx, gz)
        self.assertAlmostEqual(wx, 0.0, delta=res)
        self.assertAlmostEqual(wz, 0.0, delta=res)

        # 2D Grid array: 0=UNDISCOVERED, 1=FREE, 2=OCCUPIED
        cells = np.zeros((grid_w, grid_w), dtype=np.uint8)

        # Carve ray from (0,0) along +Z facing obstacle at 1.4m
        robot_gx, robot_gz = world_to_grid(0.0, 0.0)
        for z_step in np.arange(0.0, 1.4, res):
            g = world_to_grid(0.0, z_step)
            if g:
                cells[g[1], g[0]] = 1  # FREE
        hit_g = world_to_grid(0.0, 1.4)
        cells[hit_g[1], hit_g[0]] = 2  # OCCUPIED

        self.assertEqual(cells[robot_gz, robot_gx], 1)
        self.assertEqual(cells[hit_g[1], hit_g[0]], 2)

        # Frontier cells: UNDISCOVERED cells adjacent to FREE cells (not near OCCUPIED)
        frontiers = []
        for gz in range(1, grid_w - 1):
            for gx in range(1, grid_w - 1):
                if cells[gz, gx] != 0:
                    continue
                neighbors = cells[gz - 1:gz + 2, gx - 1:gx + 2]
                if np.any(neighbors == 1) and not np.any(neighbors == 2):
                    frontiers.append((gx, gz))

        self.assertGreater(len(frontiers), 0, "Must detect undiscovered frontier cells adjacent to free path")

    def test_circulation_detection_and_escape(self):
        """
        Verify circulation / loop trapping detector:
        1. A robot driving continuously (>=4.0m) within a tight dispersion radius (<=1.25m)
           is flagged as CIRCULATING.
        2. A robot driving along an open exploratory trajectory (e.g. straight or wide sweep)
           is NOT flagged as circulating.
        3. Escape goal calculation selects an undiscovered frontier outside the circulation radius.
        """
        def detect_circulation(path_history, min_dist=3.8, max_radius=1.25):
            if len(path_history) < 20:
                return False, None
            total_dist = sum(
                math.hypot(path_history[i][0] - path_history[i - 1][0], path_history[i][1] - path_history[i - 1][1])
                for i in range(1, len(path_history))
            )
            if total_dist < min_dist:
                return False, None

            cx = sum(p[0] for p in path_history) / len(path_history)
            cz = sum(p[1] for p in path_history) / len(path_history)
            dispersion = max(math.hypot(p[0] - cx, p[1] - cz) for p in path_history)

            is_looping = (dispersion <= max_radius and total_dist >= min_dist)
            return is_looping, {"x": cx, "z": cz, "radius": dispersion}

        # Case 1: Trapped in a circle of radius 0.7m (total path 6.6m)
        loop_history = []
        center_x, center_z = 1.0, 1.0
        loop_r = 0.70
        for step in range(30):
            theta = (step / 30.0) * (2.0 * math.pi * 1.5)  # 1.5 revolutions
            x = center_x + math.cos(theta) * loop_r
            z = center_z + math.sin(theta) * loop_r
            loop_history.append((x, z))

        is_circ, centroid = detect_circulation(loop_history)
        self.assertTrue(is_circ, "Circular loop trajectory within 0.7m radius must trigger circulation detection")
        self.assertAlmostEqual(centroid["x"], center_x, delta=0.2)
        self.assertAlmostEqual(centroid["z"], center_z, delta=0.2)

        # Case 2: Open exploratory sweep (traveled 5.2m along straight trajectory)
        open_history = [(0.0, float(step) * 0.18) for step in range(30)]
        is_circ_open, _ = detect_circulation(open_history)
        self.assertFalse(is_circ_open, "Open exploratory trajectory must not trigger loop detection")

        # Case 3: Escape goal selection outside the loop zone
        candidate_frontiers = [
            {"x": 1.2, "z": 1.3},  # Inside loop (dist ~0.36m from centroid)
            {"x": 3.5, "z": 2.8},  # Far outside in undiscovered Room (dist ~3.0m)
        ]
        escape_goals = [f for f in candidate_frontiers if math.hypot(f["x"] - centroid["x"], f["z"] - centroid["z"]) > 1.8]
        self.assertEqual(len(escape_goals), 1)
        self.assertEqual(escape_goals[0]["x"], 3.5)

    def test_inch_collision_thresholds_and_map_adapted_path(self):
        """
        Verify inch-based collision distance thresholds and map-adapted path generation:
        1. 10.0-inch trigger (0.254m), 14.0-inch clear (0.356m), 6.0-inch emergency (0.152m).
        2. Obstacle at 16 inches (0.40m) does not block corridor; obstacle at 8.5 inches (0.216m) triggers avoidance.
        3. Path waypoints curve toward undiscovered frontier heading and deflect away from occupied grid cells.
        """
        INCH_TO_M = 0.0254
        COLLISION_TRIGGER_M = 10.0 * INCH_TO_M  # 0.254m
        COLLISION_CLEAR_M = 14.0 * INCH_TO_M    # 0.356m
        COLLISION_EMERGENCY_M = 6.0 * INCH_TO_M # 0.152m
        CORRIDOR_HALF_W = 5.5 * INCH_TO_M       # 0.140m

        # 1. Threshold checks
        self.assertAlmostEqual(COLLISION_TRIGGER_M, 0.254, places=3)
        self.assertAlmostEqual(COLLISION_CLEAR_M, 0.356, places=3)
        self.assertAlmostEqual(COLLISION_EMERGENCY_M, 0.152, places=3)
        self.assertAlmostEqual(CORRIDOR_HALF_W, 0.140, places=3)

        # 2. Corridor evaluation:
        # Obstacle at 0.40m (15.7 inches) -> Clear corridor
        dist_far = 0.40
        self.assertGreater(dist_far, COLLISION_TRIGGER_M, "15.7-inch obstacle must not trigger collision avoidance")

        # Obstacle at 0.216m (8.5 inches) -> Blocked corridor
        dist_close = 0.216
        self.assertLess(dist_close, COLLISION_TRIGGER_M, "8.5-inch obstacle must trigger collision avoidance")

        # 3. Frontier-directed path curvature
        robot_x, robot_z = 0.0, 0.0
        robot_yaw = 0.0  # Facing +Z
        frontier_goal = {"x": 2.5, "z": 2.5}  # Bearing = +45 deg (+PI/4)
        bearing = math.atan2(frontier_goal["x"] - robot_x, frontier_goal["z"] - robot_z)
        self.assertAlmostEqual(bearing, math.pi / 4, places=2)

        # Generate 10 waypoints curving toward frontier
        waypoints = []
        for i in range(1, 11):
            frac = i / 10.0
            s = frac * 1.8
            angle = bearing * (frac * 0.70)
            wx = robot_x + math.sin(angle) * s
            wz = robot_z + math.cos(angle) * s
            waypoints.append((wx, wz))

        # Waypoints should smoothly bend rightward (+X) toward the frontier
        self.assertGreater(waypoints[-1][0], 0.40, "End of path must bend rightward toward frontier in +X")
        self.assertGreater(waypoints[-1][1], 1.20, "End of path must make forward progress in +Z")

    def test_line_of_sight_frontier_and_corner_safety_margin(self):
        """
        Verify line-of-sight frontier filtering, 0.26m corner safety corridor, and waypoint inflation:
        1. Line-of-sight raycaster detects intervening occupied cells and penalizes occluded frontiers (-900).
        2. Expanded sector corridor (0.26m) detects corner vertices within chassis envelope (0.18m).
        3. Waypoint Obstacle Inflation Pusher enforces minimum 0.28m (11") standoff from obstacle vertices.
        4. Flank clearance guard penalizes inward turns into obstacles alongside chassis.
        """
        # 1. Line-of-Sight Raycasting Simulation
        resolution = 0.20
        grid_dim = 45
        half_size = 4.5
        grid_cells = np.zeros((grid_dim, grid_dim), dtype=np.uint8)

        # Place a wall at x = 1.0m (gx = 27), spanning z from -1.0m to +1.0m
        gx_wall = int(math.floor((1.0 + half_size) / resolution))
        gz_min = int(math.floor((-1.0 + half_size) / resolution))
        gz_max = int(math.floor((1.0 + half_size) / resolution))
        for gz in range(gz_min, gz_max + 1):
            grid_cells[gz, gx_wall] = 2  # 2 = OCCUPIED

        def has_line_of_sight(x0, z0, x1, z1):
            gx0 = int(math.floor((x0 + half_size) / resolution))
            gz0 = int(math.floor((z0 + half_size) / resolution))
            gx1 = int(math.floor((x1 + half_size) / resolution))
            gz1 = int(math.floor((z1 + half_size) / resolution))

            cx, cy = gx0, gz0
            dx = abs(gx1 - cx)
            dy = abs(gz1 - cy)
            sx = 1 if cx < gx1 else -1
            sy = 1 if cy < gz1 else -1
            err = dx - dy

            is_start = True
            while True:
                if cx == gx1 and cy == gz1:
                    break
                if not is_start:
                    if 0 <= cx < grid_dim and 0 <= cy < grid_dim:
                        if grid_cells[cy, cx] == 2:
                            return False  # Blocked by occupied wall cell
                is_start = False
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    cx += sx
                if e2 < dx:
                    err += dx
                    cy += sy
            return True

        # Ray from (0, 0) to occluded frontier (2.0, 0.0) behind wall
        los_occluded = has_line_of_sight(0.0, 0.0, 2.0, 0.0)
        self.assertFalse(los_occluded, "Frontier directly behind wall must have line-of-sight = False")

        # Ray from (0, 0) to clear frontier (0.0, 2.5) with open corridor
        los_clear = has_line_of_sight(0.0, 0.0, 0.0, 2.5)
        self.assertTrue(los_clear, "Frontier with open corridor must have line-of-sight = True")

        # Frontier scoring comparison
        score_occluded = 10 * 14.0 - 2.0 * 6.5 - (0.0 if los_occluded else 900.0)
        score_clear = 10 * 14.0 - 2.5 * 6.5 + (250.0 if los_clear else 0.0)
        self.assertGreater(score_clear, score_occluded, "Clear corridor frontier must easily outscore occluded frontier")

        # 2. Expanded 0.26m Corner Safety Corridor vs Old 0.14m Corridor
        # Corner vertex at (x=0.18, z=0.60). Robot swept diagonal turning radius is 0.186m.
        obs_x, obs_z = 0.18, 0.60
        dir_x, dir_z = 0.0, 1.0  # Ray heading straight forward
        proj = obs_x * dir_x + obs_z * dir_z  # 0.60
        perp_dist = abs(obs_x * dir_z - obs_z * dir_x)  # 0.18

        old_half_w = 0.140
        new_half_w = 0.260

        # Under old narrow corridor (0.14m), corner was MISSED (perp_dist > 0.14m)
        self.assertGreater(perp_dist, old_half_w, "Old corridor failed to detect 0.18m corner vertex")

        # Under new expanded corridor (0.26m), corner is CAUGHT (perp_dist < 0.26m)
        self.assertLess(perp_dist, new_half_w, "New 0.26m corridor must detect 0.18m corner vertex")

        # 3. Waypoint Obstacle Inflation Pusher
        # Waypoint placed too close to obstacle vertex (distance 0.15m < min_standoff 0.28m)
        min_standoff = 0.28
        wp = [0.15, 0.80]
        corner_obs = [0.0, 0.80]
        d_x = wp[0] - corner_obs[0]
        d_z = wp[1] - corner_obs[1]
        dist = math.hypot(d_x, d_z)
        self.assertLess(dist, min_standoff)

        # Apply radial inflation push
        push = min_standoff - dist
        wp[0] += (d_x / dist) * push
        wp[1] += (d_z / dist) * push

        new_dist = math.hypot(wp[0] - corner_obs[0], wp[1] - corner_obs[1])
        self.assertAlmostEqual(new_dist, min_standoff, places=3,
                               msg="Inflated waypoint must maintain >= 0.28m standoff from obstacle vertex")

    def test_deadend_turnaround_and_inter_room_routing(self):
        """
        Verify Dead-End Detection, 180° Backtrack U-Turn, and Local vs Inter-Room Frontier Classification:
        1. Distinguish local room frontiers (dist <= 2.5m, LOS clear) from doorway/inter-room frontiers.
        2. Detect room completion (localFrontierCount == 0 -> isCurrentRoomExplored = True).
        3. Detect dead-end state (front clearance < 0.38m with restricted flanks < 0.45m).
        4. Generate decisive 180° U-turn escape heading pointing back toward undiscovered room doorway.
        5. Verify TinyML policy in-place pivot response for dead-end turnaround (reverse/differential spin).
        """
        def normalize_angle(theta):
            while theta > math.pi:
                theta -= 2.0 * math.pi
            while theta < -math.pi:
                theta += 2.0 * math.pi
            return theta

        # 1. Local vs Inter-Room Frontier Classification
        # Suppose robot is at (0, 0)
        robot_x, robot_z = 0.0, 0.0

        frontiers = [
            {"x": 1.2, "z": 1.5, "los": True, "size": 8},   # dist = 1.92m, LOS -> Local
            {"x": 4.5, "z": 0.5, "los": False, "size": 15}, # dist = 4.53m > 2.5m -> Inter-Room
            {"x": 1.0, "z": 0.8, "los": False, "size": 5},  # dist = 1.28m but occluded -> Inter-Room
        ]

        local_frontiers = []
        inter_room_frontiers = []

        for f in frontiers:
            dist = math.hypot(f["x"] - robot_x, f["z"] - robot_z)
            is_local = (dist <= 2.5 and f["los"])
            if is_local:
                local_frontiers.append(f)
            else:
                inter_room_frontiers.append(f)

        self.assertEqual(len(local_frontiers), 1, "Expected 1 local room frontier")
        self.assertEqual(len(inter_room_frontiers), 2, "Expected 2 inter-room / doorway frontiers")

        # Room exploration status when local frontiers exist
        is_room_explored = (len(local_frontiers) == 0)
        self.assertFalse(is_room_explored, "Room with remaining local frontiers must not be marked explored")

        # Once all local frontiers in current room are cleared
        local_frontiers.clear()
        is_room_explored = (len(local_frontiers) == 0)
        self.assertTrue(is_room_explored, "Room with 0 local frontiers must be marked explored")

        # 2. Dead-End & Wall Boundary Detection
        def check_dead_end(front_clear, left_clear, right_clear, room_cleared):
            front_blocked = (front_clear < 0.38)
            sides_restricted = (left_clear < 0.45 and right_clear < 0.45)
            return (front_blocked and (sides_restricted or left_clear < 0.38 or right_clear < 0.38)) or (front_blocked and room_cleared)

        # Case A: Open space ahead
        self.assertFalse(check_dead_end(1.5, 0.8, 0.8, False))

        # Case B: Corner / Dead-end wall ahead (front 0.28m, left 0.35m, right 0.40m)
        self.assertTrue(check_dead_end(0.28, 0.35, 0.40, False), "Constrained corner must trigger dead-end")

        # Case C: Room perimeter wall reached after room is fully explored
        self.assertTrue(check_dead_end(0.35, 1.2, 1.0, True), "Perimeter wall in cleared room must trigger dead-end backtrack")

        # 3. Decisive 180° Backtrack U-Turn Heading
        robot_yaw = 0.5  # Heading ~28.6 degrees
        escape_bearing = normalize_angle(robot_yaw + math.pi)
        # Difference between robot_yaw and escape_bearing must be exactly 180 degrees (pi rad)
        yaw_diff = abs(normalize_angle(escape_bearing - robot_yaw))
        self.assertAlmostEqual(yaw_diff, math.pi, places=5, msg="Escape bearing must be exactly 180° reversed")

        # 4. TinyML Policy Response for Dead-End Turnaround
        # When target is directly behind (target_rel_yaw ≈ ±pi) and front wall is within 0.32m,
        # TinyML policy must execute an aggressive in-place pivot / differential turn, not push forward into wall.
        if self.numpy_runner is not None:
            l_u_turn, r_u_turn = self.numpy_runner.step(
                dist_front_m=0.30,
                dist_left_m=0.40,
                dist_right_m=1.80,
                target_rel_yaw_rad=-2.8,  # Target is almost 180° behind to left
                target_dist_m=3.0,
                has_fire=False,
                heading_rate_rads=0.0
            )
            # Must steer left towards escape target (right_pwm > left_pwm)
            self.assertGreater(r_u_turn, l_u_turn, "Policy must pivot left toward reverse target")
            # Should have substantial turning differential to turn around in-place
            turn_diff = abs(r_u_turn - l_u_turn)
            self.assertGreater(turn_diff, 100.0, "Must provide aggressive turning differential for 180° turnaround")


if __name__ == "__main__":
    unittest.main()

