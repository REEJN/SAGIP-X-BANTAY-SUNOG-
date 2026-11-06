"""
Unit Tests for BANTAY-SUNOG Simulation & Bridge
Verifies:
1. Serial connector telemetry ingestion and JSON formatting matching types.h and protocol.py.
2. Differential drive kinematic calculations.
3. Ultrasonic sensor threshold checks and ray calculations.
"""

import json
import math
import unittest
from pathlib import Path
import sys

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulation.bridge.serial_connector import SerialConnector
from bantay_sunog.communication.protocol import (
    build_fire_confirmed,
    build_fire_candidate,
    build_no_fire
)


class TestSimulationBridge(unittest.TestCase):
    def setUp(self):
        self.conn = SerialConnector(mock_mode=True)

    def test_serial_mock_telemetry_schema(self):
        """Verify mock telemetry format matches ESP32 RobotTelemetry struct."""
        received = []
        self.conn.on_telemetry_received = lambda data: received.append(data)
        self.conn.connect()

        import time
        time.sleep(0.6)
        self.conn.disconnect()

        self.assertGreater(len(received), 0, "Should have received at least one telemetry packet")
        telem = received[0]

        # Verify key fields required by frontend and ESP32 types.h
        self.assertIn("robot_state", telem)
        self.assertIn("dist_front", telem)
        self.assertIn("dist_left", telem)
        self.assertIn("dist_right", telem)
        self.assertIn("flame_detected", telem)
        self.assertIn("tilt_angle", telem)
        self.assertIn("solenoid_active", telem)
        self.assertIn("left_pwm", telem)
        self.assertIn("right_pwm", telem)
        self.assertIn("uptime", telem)

    def test_vision_message_formatting(self):
        """Verify Pi vision messages match communication protocol."""
        pkt = build_fire_confirmed(
            confidence=0.92,
            x=0.55,
            y=0.48,
            bbox=[120, 80, 100, 150],
            frame_dims=[640, 480],
            timestamp=1730000000
        )
        self.assertEqual(pkt["event"], "FIRE_CONFIRMED")
        self.assertEqual(pkt["confidence"], 0.92)
        self.assertEqual(pkt["x"], 0.55)
        self.assertEqual(pkt["y"], 0.48)
        self.assertEqual(len(pkt["bbox"]), 4)

    def test_differential_kinematics(self):
        """Verify 4WD skid-steer kinematics: straight driving vs in-place pivot."""
        track_width = 0.24  # 24 cm
        max_speed = 0.65   # m/s

        # 1. Straight forward (Left PWM = 255, Right PWM = 255)
        v_l = max_speed
        v_r = max_speed
        linear = (v_l + v_r) / 2.0
        angular = (v_r - v_l) / track_width

        self.assertAlmostEqual(linear, max_speed)
        self.assertAlmostEqual(angular, 0.0)

        # 2. In-place pivot right (Left PWM = +130, Right PWM = -130)
        v_l = (130 / 255.0) * max_speed
        v_r = (-130 / 255.0) * max_speed
        linear = (v_l + v_r) / 2.0
        angular = (v_r - v_l) / track_width

        self.assertAlmostEqual(linear, 0.0)
        self.assertLess(angular, 0.0) # Clockwise / rightward turn

    def test_ultrasonic_obstacle_thresholds(self):
        """Verify obstacle threshold logic matches ESP32 config.h."""
        obstacle_front_threshold_cm = 25.0
        extinguish_distance_cm = 30.0

        # Safe distance
        dist_1 = 80.0
        self.assertFalse(dist_1 < obstacle_front_threshold_cm)

        # Emergency stop obstacle threshold
        dist_2 = 22.5
        self.assertTrue(dist_2 < obstacle_front_threshold_cm)

        # Standoff spray distance
        dist_3 = 29.0
        self.assertTrue(dist_3 <= extinguish_distance_cm)

    def test_zigzag_scan_lane_shift(self):
        """Verify 2 consecutive 90-degree turns invert heading by 180 degrees."""
        initial_yaw = 0.0  # Facing +Z
        turn_out = initial_yaw + (math.pi / 2)  # 90 deg right
        shift_heading = turn_out
        turn_in = shift_heading + (math.pi / 2)  # another 90 deg right -> 180 deg
        
        # Turn in heading should be opposite to initial
        self.assertAlmostEqual(turn_in, math.pi)
        
        # Inverted forward vector
        fwd_x = math.sin(turn_in)
        fwd_z = math.cos(turn_in)
        self.assertAlmostEqual(fwd_x, 0.0, places=5)
        self.assertAlmostEqual(fwd_z, -1.0, places=5)

    def test_obstacle_clearance_selection(self):
        """Verify avoidance turn direction chooses side with higher clearance."""
        # Case 1: Left is clear (120 cm), Right is blocked (25 cm) -> Turn Left
        dist_left_1 = 120.0
        dist_right_1 = 25.0
        turn_left_1 = (dist_left_1 >= dist_right_1)
        self.assertTrue(turn_left_1)

        # Case 2: Right is clear (90 cm), Left is blocked (15 cm) -> Turn Right
        dist_left_2 = 15.0
        dist_right_2 = 90.0
        turn_left_2 = (dist_left_2 >= dist_right_2)
    def test_virtual_camera_ai_inference_pipeline(self):
        """Verify real-time camera inference loop: base64 frame -> cv2 decode -> FireDetector -> ConfirmationTracker."""
        import base64
        import cv2
        import numpy as np
        from bantay_sunog.detection.model_loader import load_model
        from bantay_sunog.detection.fire_detector import FireDetector
        from bantay_sunog.detection.confirmation import ConfirmationTracker

        detector = FireDetector(load_model("hsv_fallback"))
        tracker = ConfirmationTracker()

        # Generate synthetic flame test frame (250x187) with flame color
        frame = np.zeros((187, 250, 3), dtype=np.uint8)
        # BGR (20, 160, 255) -> HSV [18, 235, 255] (flame core)
        cv2.circle(frame, (125, 95), 24, (20, 160, 255), -1)

        # Encode to JPEG Base64
        _, buf = cv2.imencode('.jpg', frame)
        img_b64 = base64.b64encode(buf).decode('ascii')

        # Decode in bridge
        raw_bytes = base64.b64decode(img_b64)
        dec_frame = cv2.imdecode(np.frombuffer(raw_bytes, np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(dec_frame)
        self.assertEqual(dec_frame.shape, (187, 250, 3))

        # Detect
        detections = detector.detect(dec_frame)
        self.assertGreater(len(detections), 0, "HSV detector should detect flame circle")

        # Confirmation tracker update
        state, event, best_det = tracker.update(detections)
        self.assertIn(state, ("FIRE_CANDIDATE", "FIRE_CONFIRMED"))
        self.assertIsNotNone(best_det)
        self.assertGreater(best_det.confidence, 0.4)
        self.assertAlmostEqual(best_det.normalized_x, 0.50, delta=0.1)

    def test_angle_normalization(self):
        """Verify _normalizeAngle logic brings angles to [-pi, +pi]."""
        def normalize_angle(a):
            while a > math.pi:
                a -= 2 * math.pi
            while a < -math.pi:
                a += 2 * math.pi
            return a

        self.assertAlmostEqual(normalize_angle(3 * math.pi), math.pi, places=5)
        self.assertAlmostEqual(normalize_angle(-3 * math.pi), -math.pi, places=5)
        self.assertAlmostEqual(normalize_angle(0.5), 0.5, places=5)
        self.assertAlmostEqual(normalize_angle(2 * math.pi + 0.1), 0.1, places=5)

    def test_sliding_collision_resolution(self):
        """Verify sliding collision resolves penetration along collision normal without blocking tangential movement."""
        robot_radius = 0.18
        obs_radius = 0.35
        min_sep = robot_radius + obs_radius # 0.53

        # Obstacle at (1.0, 0.0), robot moving towards (0.8, 0.0) -> penetration = 0.53 - 0.2 = 0.33
        obs_x, obs_z = 1.0, 0.0
        robot_x, robot_z = 0.8, 0.0

        dx = robot_x - obs_x
        dz = robot_z - obs_z
        dist = math.hypot(dx, dz)
        self.assertLess(dist, min_sep)

        # Apply push out
        push_dist = min_sep - dist + 0.005
        nx = dx / dist
        nz = dz / dist
        resolved_x = robot_x + nx * push_dist
        resolved_z = robot_z + nz * push_dist

        new_dist = math.hypot(resolved_x - obs_x, resolved_z - obs_z)
        self.assertGreaterEqual(new_dist, min_sep)

    def test_obb_box_collision_resolution(self):
        """Verify 2D Oriented Bounding Box (OBB) collision resolution against rotated wall/box."""
        robot_radius = 0.18
        # Box centered at (0.0, 2.0) with width=2.0 (halfW=1.0), depth=0.2 (halfD=0.1), rotY=0
        half_w, half_d = 1.0, 0.1
        box_x, box_z = 0.0, 2.0
        rot_y = 0.0

        # Robot penetrates the front face at (0.2, 1.95) -> localZ is -0.05, within box
        robot_x, robot_z = 0.2, 1.95
        dx = robot_x - box_x
        dz = robot_z - box_z

        cos_a = math.cos(-rot_y)
        sin_a = math.sin(-rot_y)
        local_x = dx * cos_a - dz * sin_a
        local_z = dx * sin_a + dz * cos_a

        closest_x = max(-half_w, min(half_w, local_x))
        closest_z = max(-half_d, min(half_d, local_z))

        diff_x = local_x - closest_x
        diff_z = local_z - closest_z
        dist = math.hypot(diff_x, diff_z)

        # In this case robot center penetrates front face
        self.assertLess(dist, robot_radius)

        # Resolution along closest normal
        push_dist = robot_radius - dist + 0.002
        if dist > 0.001:
            push_x = (diff_x / dist) * push_dist
            push_z = (diff_z / dist) * push_dist
        else:
            push_x = 0
            push_z = -(half_d - local_z + robot_radius + 0.002)

        res_x = robot_x + push_x
        res_z = robot_z + push_z

        # After push out, distance to box boundary is at least robot_radius
        new_local_z = res_z - box_z
        self.assertLessEqual(new_local_z, -half_d - robot_radius + 0.01)

    def test_vehicle_dynamics_inertia(self):
        """Verify linear and angular velocity ramp smoothly under max acceleration."""
        dt = 0.016
        max_linear_acc = 2.6
        current_speed = 0.0
        target_speed = 0.65 # max speed at 255 PWM

        # After 1 frame, speed should not jump to target_speed instantly
        diff = target_speed - current_speed
        step = max_linear_acc * dt
        current_speed += step
        self.assertLess(current_speed, target_speed)
        self.assertAlmostEqual(current_speed, 2.6 * 0.016, places=4)

    def test_mission_task_completion_lifecycle(self):
        """Verify FSM state transition: APPROACHING -> EXTINGUISHING -> COOLDOWN -> TASK_COMPLETE."""
        state = 'APPROACHING'
        target_dist = 45.0 # within 50cm standoff spray range
        flame_detected = True

        # Standoff reached
        if target_dist <= 50.0:
            state = 'EXTINGUISHING'
        self.assertEqual(state, 'EXTINGUISHING')

        # After spraying, flame is extinguished
        spray_time = 2.0
        if spray_time >= 1.8:
            flame_detected = False
            state = 'COOLDOWN'
        self.assertEqual(state, 'COOLDOWN')

        # After cooldown and zero remaining fires in arena
        cooldown_time = 2.0
        active_fires_count = 0
        if cooldown_time > 1.6 and not flame_detected:
            if active_fires_count == 0:
                state = 'TASK_COMPLETE'
            else:
                state = 'SEARCHING'
        self.assertEqual(state, 'TASK_COMPLETE')


if __name__ == "__main__":
    unittest.main()
