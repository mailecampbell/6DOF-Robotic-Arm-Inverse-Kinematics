#!/usr/bin/env python3
"""
Core Logic Tests (no OpenCV required)

Tests the coordinate conversion and mirror logic without camera/vision dependencies.

Run:
    python test_core.py
"""

import unittest
import numpy as np
import sys

# Test config module
print("Testing config module...")
from config import BOARD, WARP, WORKSPACE, MOTION, ARM

print(f"  Board: {BOARD.inner_width_mm}x{BOARD.inner_height_mm} mm")
print(f"  Warp: {WARP.warp_width}x{WARP.warp_height} px at {WARP.px_per_mm} px/mm")
print(f"  Workspace origin: ({WORKSPACE.board_origin_x}, {WORKSPACE.board_origin_y}) mm")


class TestBoardConfig(unittest.TestCase):
    """Test board configuration."""

    def test_dimensions(self):
        self.assertEqual(BOARD.inner_width_mm, 252.0)
        self.assertEqual(BOARD.inner_height_mm, 195.0)
        self.assertAlmostEqual(BOARD.outer_width_mm, 272.0)
        self.assertAlmostEqual(BOARD.outer_height_mm, 215.0)


class TestWarpConfig(unittest.TestCase):
    """Test warp configuration."""

    def test_pixel_dimensions(self):
        expected_width = int(BOARD.outer_width_mm * WARP.px_per_mm)
        expected_height = int(BOARD.outer_height_mm * WARP.px_per_mm)
        self.assertEqual(WARP.warp_width, expected_width)
        self.assertEqual(WARP.warp_height, expected_height)

    def test_margin(self):
        expected_margin = int(BOARD.border_mm * WARP.px_per_mm)
        self.assertEqual(WARP.margin_px, expected_margin)


class TestWorkspaceTransform(unittest.TestCase):
    """Test coordinate transforms."""

    def test_origin_transform(self):
        """Board origin should map to workspace origin."""
        rx, ry = WORKSPACE.board_to_robot(0, 0)
        self.assertAlmostEqual(rx, WORKSPACE.board_origin_x)
        self.assertAlmostEqual(ry, WORKSPACE.board_origin_y)

    def test_x_direction(self):
        """Moving right on board should change robot X (forward)."""
        rx0, ry0 = WORKSPACE.board_to_robot(0, 0)
        rx1, ry1 = WORKSPACE.board_to_robot(100, 0)

        # X should increase (moving forward)
        self.assertGreater(rx1, rx0)
        # Y should stay same (no lateral movement)
        self.assertAlmostEqual(ry1, ry0)

    def test_y_direction(self):
        """Moving down on board should decrease robot Y (moving right)."""
        rx0, ry0 = WORKSPACE.board_to_robot(0, 0)
        rx1, ry1 = WORKSPACE.board_to_robot(0, 100)

        # X should stay same
        self.assertAlmostEqual(rx1, rx0)
        # Y should decrease (board Y down = robot Y right = negative)
        self.assertLess(ry1, ry0)


class TestMirrorLogic(unittest.TestCase):
    """Test ghost/mirror computation."""

    def mirror_box(self, box_xywh, x_div_px):
        """Mirror logic extracted for testing."""
        x, y, w, h = box_xywh
        cx = x + w / 2.0
        cy = y + h / 2.0
        cx_ghost = 2.0 * x_div_px - cx
        xg = int(round(cx_ghost - w / 2.0))
        yg = int(round(cy - h / 2.0))
        return (xg, yg, w, h)

    def test_mirror_at_divider(self):
        """Object centered at divider should stay there."""
        divider = 200
        # Box centered at x=200
        box = (190, 100, 20, 20)  # cx = 200

        ghost = self.mirror_box(box, divider)
        ghost_cx = ghost[0] + ghost[2] / 2.0

        self.assertAlmostEqual(ghost_cx, divider, places=0)

    def test_mirror_left_to_right(self):
        """Object left of divider mirrors to right."""
        divider = 200
        # Box centered at x=60
        box = (50, 100, 20, 20)  # cx = 60

        ghost = self.mirror_box(box, divider)
        ghost_cx = ghost[0] + ghost[2] / 2.0

        # Expected: 2*200 - 60 = 340
        self.assertAlmostEqual(ghost_cx, 340, places=0)

    def test_mirror_right_to_left(self):
        """Object right of divider mirrors to left."""
        divider = 200
        # Box centered at x=300
        box = (290, 100, 20, 20)  # cx = 300

        ghost = self.mirror_box(box, divider)
        ghost_cx = ghost[0] + ghost[2] / 2.0

        # Expected: 2*200 - 300 = 100
        self.assertAlmostEqual(ghost_cx, 100, places=0)

    def test_mirror_preserves_y(self):
        """Y coordinate unchanged by mirror."""
        divider = 200
        box = (50, 150, 20, 30)  # cy = 165

        ghost = self.mirror_box(box, divider)

        original_cy = box[1] + box[3] / 2.0
        ghost_cy = ghost[1] + ghost[3] / 2.0

        self.assertAlmostEqual(ghost_cy, original_cy)

    def test_mirror_preserves_size(self):
        """Width and height unchanged by mirror."""
        divider = 200
        box = (50, 100, 25, 35)

        ghost = self.mirror_box(box, divider)

        self.assertEqual(ghost[2], box[2])
        self.assertEqual(ghost[3], box[3])


class TestCoordinateChain(unittest.TestCase):
    """Test full coordinate conversion chain."""

    def inner_px_to_board_mm(self, ix, iy):
        bx = ix / WARP.px_per_mm
        by = iy / WARP.px_per_mm
        return (bx, by)

    def board_mm_to_robot_mm(self, bx, by):
        return WORKSPACE.board_to_robot(bx, by)

    def inner_px_to_robot(self, ix, iy):
        bx, by = self.inner_px_to_board_mm(ix, iy)
        return self.board_mm_to_robot_mm(bx, by)

    def test_origin_conversion(self):
        """Inner pixel origin should map correctly."""
        rx, ry = self.inner_px_to_robot(0, 0)

        self.assertAlmostEqual(rx, WORKSPACE.board_origin_x)
        self.assertAlmostEqual(ry, WORKSPACE.board_origin_y)

    def test_board_corner(self):
        """Bottom-right corner of board."""
        # Convert board dimensions to inner pixels
        width_px = BOARD.inner_width_mm * WARP.px_per_mm
        height_px = BOARD.inner_height_mm * WARP.px_per_mm

        rx, ry = self.inner_px_to_robot(width_px, height_px)

        # Should be forward and to the right of origin
        self.assertGreater(rx, WORKSPACE.board_origin_x)
        self.assertLess(ry, WORKSPACE.board_origin_y)

    def test_center_of_board(self):
        """Center of board."""
        center_px_x = (BOARD.inner_width_mm * WARP.px_per_mm) / 2
        center_px_y = (BOARD.inner_height_mm * WARP.px_per_mm) / 2

        rx, ry = self.inner_px_to_robot(center_px_x, center_px_y)

        # Should be reachable (positive X, reasonable Y)
        self.assertGreater(rx, 0)


class TestArmGeometry(unittest.TestCase):
    """Test arm geometry calculations."""

    def test_total_reach(self):
        expected = (ARM.upper_arm_length + ARM.forearm_length +
                   ARM.wrist_length + ARM.tool_length)
        self.assertAlmostEqual(ARM.total_reach, expected)

    def test_min_reach(self):
        expected = (abs(ARM.upper_arm_length - ARM.forearm_length) +
                   ARM.wrist_length + ARM.tool_length)
        self.assertAlmostEqual(ARM.min_reach, expected)


class TestMotionParams(unittest.TestCase):
    """Test motion parameter sanity."""

    def test_height_ordering(self):
        """Safe height should be above pick/place."""
        self.assertGreater(MOTION.z_safe, MOTION.z_pick)
        self.assertGreater(MOTION.z_safe, MOTION.z_place)

    def test_gripper_range(self):
        """Gripper angles should be valid."""
        self.assertGreaterEqual(MOTION.gripper_open, 0)
        self.assertLessEqual(MOTION.gripper_open, 180)
        self.assertGreaterEqual(MOTION.gripper_closed, 0)
        self.assertLessEqual(MOTION.gripper_closed, 180)


def run_tests():
    """Run all tests and report results."""
    print("\n" + "=" * 60)
    print("CORE LOGIC TESTS")
    print("=" * 60 + "\n")

    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestBoardConfig))
    suite.addTests(loader.loadTestsFromTestCase(TestWarpConfig))
    suite.addTests(loader.loadTestsFromTestCase(TestWorkspaceTransform))
    suite.addTests(loader.loadTestsFromTestCase(TestMirrorLogic))
    suite.addTests(loader.loadTestsFromTestCase(TestCoordinateChain))
    suite.addTests(loader.loadTestsFromTestCase(TestArmGeometry))
    suite.addTests(loader.loadTestsFromTestCase(TestMotionParams))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Summary
    print("\n" + "=" * 60)
    if result.wasSuccessful():
        print("ALL TESTS PASSED!")
    else:
        print(f"FAILURES: {len(result.failures)}")
        print(f"ERRORS: {len(result.errors)}")
    print("=" * 60)

    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(run_tests())
