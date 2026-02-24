#!/usr/bin/env python3
"""
Integration Tests for Vision-Robot System

Run all tests:
    python test_integration.py

Run specific test:
    python test_integration.py TestCoordinates
"""

import unittest
import numpy as np
from typing import Tuple

# Import modules to test
from config import BOARD, WARP, WORKSPACE, MOTION, ARM
from coordinates import CoordinateConverter, is_reachable, compute_workspace_bounds
from vision import mirror_box_across_vertical_line, choose_target, Detection
from robot_serial import MockRobotSerial, CommandStatus
from motion import MotionController, PickPlaceTask, MotionResult


class TestConfig(unittest.TestCase):
    """Test configuration values."""

    def test_board_dimensions(self):
        """Board dimensions should be positive and reasonable."""
        self.assertGreater(BOARD.inner_width_mm, 0)
        self.assertGreater(BOARD.inner_height_mm, 0)
        self.assertGreater(BOARD.border_mm, 0)
        self.assertEqual(
            BOARD.outer_width_mm,
            BOARD.inner_width_mm + 2 * BOARD.border_mm
        )

    def test_warp_settings(self):
        """Warp settings should be consistent."""
        self.assertGreater(WARP.px_per_mm, 0)
        expected_width = int(BOARD.outer_width_mm * WARP.px_per_mm)
        self.assertEqual(WARP.warp_width, expected_width)

    def test_motion_heights(self):
        """Motion heights should be in correct order."""
        self.assertGreater(MOTION.z_safe, MOTION.z_pick)
        self.assertGreater(MOTION.z_safe, MOTION.z_place)


class TestCoordinates(unittest.TestCase):
    """Test coordinate conversions."""

    def setUp(self):
        self.converter = CoordinateConverter()

    def test_inner_px_to_board_mm(self):
        """Test pixel to mm conversion."""
        # Origin should stay at origin
        bx, by = self.converter.inner_px_to_board_mm(0, 0)
        self.assertAlmostEqual(bx, 0)
        self.assertAlmostEqual(by, 0)

        # Check scaling
        px_per_mm = WARP.px_per_mm
        bx, by = self.converter.inner_px_to_board_mm(px_per_mm, px_per_mm)
        self.assertAlmostEqual(bx, 1.0)
        self.assertAlmostEqual(by, 1.0)

    def test_board_mm_to_robot(self):
        """Test board to robot conversion."""
        # Board origin should map to workspace origin
        rx, ry = self.converter.board_mm_to_robot_mm(0, 0)
        self.assertAlmostEqual(rx, WORKSPACE.board_origin_x)
        self.assertAlmostEqual(ry, WORKSPACE.board_origin_y)

    def test_roundtrip_inner_to_robot(self):
        """Test full conversion chain."""
        # Convert a point and verify it's in reasonable range
        rx, ry = self.converter.inner_px_to_robot(100, 100)

        # Should be positive X (forward of robot base)
        self.assertGreater(rx, 0)

    def test_is_reachable_origin(self):
        """Test reachability check."""
        # A point at the robot origin should not be reachable (too close)
        self.assertFalse(is_reachable(0, 0, 50))

    def test_is_reachable_typical(self):
        """Typical working position should be reachable."""
        # A point at typical pick position
        self.assertTrue(is_reachable(150, 0, 50))


class TestMirrorLogic(unittest.TestCase):
    """Test ghost/mirror computation."""

    def test_mirror_symmetric(self):
        """Object at divider should stay at divider."""
        divider_x = 200
        box = (190, 100, 20, 20)  # Centered at x=200

        ghost = mirror_box_across_vertical_line(box, divider_x)

        # Center should be at same X
        ghost_cx = ghost[0] + ghost[2] / 2.0
        self.assertAlmostEqual(ghost_cx, divider_x, places=0)

    def test_mirror_left_to_right(self):
        """Object left of divider should move right."""
        divider_x = 200
        box = (50, 100, 20, 20)  # cx = 60, left of divider

        ghost = mirror_box_across_vertical_line(box, divider_x)

        # Ghost center should be at 2*200 - 60 = 340
        ghost_cx = ghost[0] + ghost[2] / 2.0
        self.assertAlmostEqual(ghost_cx, 340, places=0)

    def test_mirror_preserves_y(self):
        """Y coordinate should not change."""
        divider_x = 200
        box = (50, 150, 20, 30)

        ghost = mirror_box_across_vertical_line(box, divider_x)

        # Y should be preserved
        original_cy = box[1] + box[3] / 2.0
        ghost_cy = ghost[1] + ghost[3] / 2.0
        self.assertAlmostEqual(ghost_cy, original_cy, places=0)

    def test_mirror_preserves_size(self):
        """Width and height should be preserved."""
        divider_x = 200
        box = (50, 100, 25, 35)

        ghost = mirror_box_across_vertical_line(box, divider_x)

        self.assertEqual(ghost[2], box[2])  # Width
        self.assertEqual(ghost[3], box[3])  # Height


class TestTargetSelection(unittest.TestCase):
    """Test target selection logic."""

    def test_choose_leftmost(self):
        """Should choose leftmost object when left of divider."""
        divider_x = 200

        detections = [
            Detection("RED", 100, 50, 150, 50, 20, 20, 0.9),  # cx=150
            Detection("BLUE", 10, 50, 50, 50, 20, 20, 0.9),   # cx=50, leftmost
            Detection("GREEN", 80, 50, 120, 50, 20, 20, 0.9), # cx=120
        ]

        target = choose_target(detections, divider_x)

        self.assertIsNotNone(target)
        self.assertEqual(target.color, "BLUE")

    def test_no_target_right_of_divider(self):
        """Objects only on right side should return None."""
        divider_x = 100

        detections = [
            Detection("RED", 150, 50, 150, 50, 20, 20, 0.9),  # cx=150, right of divider
            Detection("BLUE", 200, 50, 200, 50, 20, 20, 0.9), # cx=200, right of divider
        ]

        target = choose_target(detections, divider_x)

        self.assertIsNone(target)

    def test_empty_detections(self):
        """Empty detection list should return None."""
        target = choose_target([], 200)
        self.assertIsNone(target)


class TestMockRobot(unittest.TestCase):
    """Test mock robot for simulation."""

    def setUp(self):
        self.robot = MockRobotSerial()

    def test_connect(self):
        """Mock should always connect."""
        self.assertTrue(self.robot.connect())
        self.assertTrue(self.robot.connected)

    def test_move_to(self):
        """Move commands should succeed."""
        response = self.robot.move_to(100, 50, 80)
        self.assertEqual(response.status, CommandStatus.OK)

        pos = self.robot.get_position()
        self.assertAlmostEqual(pos.x, 100)
        self.assertAlmostEqual(pos.y, 50)
        self.assertAlmostEqual(pos.z, 80)

    def test_gripper(self):
        """Gripper commands should work."""
        response = self.robot.open_gripper()
        self.assertEqual(response.status, CommandStatus.OK)

        joints = self.robot.get_joints()
        self.assertEqual(joints.gripper, 90)

        response = self.robot.close_gripper()
        self.assertEqual(response.status, CommandStatus.OK)

        joints = self.robot.get_joints()
        self.assertEqual(joints.gripper, 0)

    def test_command_history(self):
        """Commands should be recorded."""
        self.robot.move_to(100, 0, 50)
        self.robot.open_gripper()
        self.robot.home()

        self.assertEqual(len(self.robot.command_history), 3)
        self.assertTrue(self.robot.command_history[0].startswith("G"))
        self.assertEqual(self.robot.command_history[1], "O")
        self.assertEqual(self.robot.command_history[2], "H")


class TestMotionController(unittest.TestCase):
    """Test motion controller with mock robot."""

    def setUp(self):
        self.robot = MockRobotSerial()
        self.motion = MotionController(self.robot)

    def test_pick_place_success(self):
        """Complete pick-and-place should succeed with mock."""
        task = PickPlaceTask(
            pick_x=150,
            pick_y=50,
            place_x=150,
            place_y=-50,
            object_color="RED"
        )

        result = self.motion.execute_pick_place(task)

        self.assertEqual(result, MotionResult.SUCCESS)

    def test_go_home(self):
        """Home command should work."""
        result = self.motion.go_home()
        self.assertTrue(result)


class TestWorkspaceBounds(unittest.TestCase):
    """Test workspace boundary computation."""

    def test_bounds_computed(self):
        """Should compute reasonable workspace bounds."""
        bounds = compute_workspace_bounds()

        # Should have positive X range (forward)
        self.assertGreater(bounds.x_max, bounds.x_min)

        # Should have symmetric Y range (left/right)
        # Note: may not be exactly symmetric depending on board placement

        # Z should start at 0 (table) and go up
        self.assertGreaterEqual(bounds.z_min, 0)
        self.assertGreater(bounds.z_max, bounds.z_min)


if __name__ == "__main__":
    print("=" * 60)
    print("VISION-ROBOT INTEGRATION TESTS")
    print("=" * 60)
    print()

    unittest.main(verbosity=2)
