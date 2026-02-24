"""
Coordinate Transformation Module

Handles conversion between coordinate systems:
1. Camera pixel coordinates
2. Warped/rectified board pixel coordinates
3. Board millimeter coordinates
4. Robot base frame coordinates

The transformation chain:
  Camera pixels -> (homography) -> Warped pixels -> (scale) -> Board mm -> (transform) -> Robot mm
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass

from config import WARP, WORKSPACE, BOARD


# =============================================================================
# COORDINATE SYSTEMS DOCUMENTATION
# =============================================================================
"""
COORDINATE SYSTEMS:

1. CAMERA PIXELS (u, v)
   - Origin: top-left of camera image
   - u: right (0 to image_width)
   - v: down (0 to image_height)
   - Unit: pixels

2. WARPED PIXELS (wx, wy)
   - Origin: top-left of warped OUTER board image
   - wx: right (0 to WARP.warp_width)
   - wy: down (0 to WARP.warp_height)
   - Unit: pixels
   - Related to camera by homography H_img2warp

3. INNER PIXELS (ix, iy)
   - Origin: top-left of INNER board area (inside black tape)
   - ix: right (0 to inner_width)
   - iy: down (0 to inner_height)
   - Unit: pixels
   - Related to warped by: (ix, iy) = (wx - margin, wy - margin)

4. BOARD MILLIMETERS (bx, by)
   - Origin: top-left of INNER board area
   - bx: right (0 to BOARD.inner_width_mm)
   - by: down (0 to BOARD.inner_height_mm)
   - Unit: millimeters
   - Related to inner pixels by: (bx, by) = (ix / PX_PER_MM, iy / PX_PER_MM)

5. ROBOT BASE FRAME (rx, ry, rz)
   - Origin: center of base rotation axis at table level
   - rx: forward (arm points +X when base=0)
   - ry: left (following right-hand rule)
   - rz: up
   - Unit: millimeters
   - Related to board by: WORKSPACE.board_to_robot(bx, by)
"""


# =============================================================================
# COORDINATE CONVERTER CLASS
# =============================================================================

class CoordinateConverter:
    """
    Converts between all coordinate systems.

    Usage:
        converter = CoordinateConverter()
        converter.set_homography(H_warp2img)  # Set when board detected

        # Convert detection to robot coordinates
        robot_xy = converter.inner_px_to_robot(cx_inner, cy_inner)
    """

    def __init__(self):
        self.H_warp2img: Optional[np.ndarray] = None
        self.H_img2warp: Optional[np.ndarray] = None

    def set_homography(self, H_img2warp: Optional[np.ndarray] = None,
                       H_warp2img: Optional[np.ndarray] = None):
        """Set homography matrices (provide at least one)"""
        if H_img2warp is not None:
            self.H_img2warp = H_img2warp
            self.H_warp2img = np.linalg.inv(H_img2warp)
        elif H_warp2img is not None:
            self.H_warp2img = H_warp2img
            self.H_img2warp = np.linalg.inv(H_warp2img)

    # -------------------------------------------------------------------------
    # INNER PIXELS <-> BOARD MM
    # -------------------------------------------------------------------------

    def inner_px_to_board_mm(self, ix: float, iy: float) -> Tuple[float, float]:
        """Convert inner pixel coordinates to board millimeters"""
        bx = ix / WARP.px_per_mm
        by = iy / WARP.px_per_mm
        return (bx, by)

    def board_mm_to_inner_px(self, bx: float, by: float) -> Tuple[float, float]:
        """Convert board millimeters to inner pixel coordinates"""
        ix = bx * WARP.px_per_mm
        iy = by * WARP.px_per_mm
        return (ix, iy)

    # -------------------------------------------------------------------------
    # INNER PIXELS <-> WARPED PIXELS
    # -------------------------------------------------------------------------

    def inner_px_to_warped_px(self, ix: float, iy: float) -> Tuple[float, float]:
        """Convert inner pixel coordinates to warped pixel coordinates"""
        margin = WARP.margin_px
        wx = ix + margin
        wy = iy + margin
        return (wx, wy)

    def warped_px_to_inner_px(self, wx: float, wy: float) -> Tuple[float, float]:
        """Convert warped pixel coordinates to inner pixel coordinates"""
        margin = WARP.margin_px
        ix = wx - margin
        iy = wy - margin
        return (ix, iy)

    # -------------------------------------------------------------------------
    # WARPED PIXELS <-> CAMERA PIXELS
    # -------------------------------------------------------------------------

    def warped_px_to_camera_px(self, wx: float, wy: float) -> Optional[Tuple[float, float]]:
        """Convert warped pixel coordinates to camera pixel coordinates"""
        if self.H_warp2img is None:
            return None

        pt = np.array([[[wx, wy]]], dtype=np.float32)
        out = cv2.perspectiveTransform(pt, self.H_warp2img)
        return (float(out[0, 0, 0]), float(out[0, 0, 1]))

    def camera_px_to_warped_px(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """Convert camera pixel coordinates to warped pixel coordinates"""
        if self.H_img2warp is None:
            return None

        pt = np.array([[[u, v]]], dtype=np.float32)
        out = cv2.perspectiveTransform(pt, self.H_img2warp)
        return (float(out[0, 0, 0]), float(out[0, 0, 1]))

    # -------------------------------------------------------------------------
    # BOARD MM <-> ROBOT MM
    # -------------------------------------------------------------------------

    def board_mm_to_robot_mm(self, bx: float, by: float) -> Tuple[float, float]:
        """
        Convert board millimeter coordinates to robot base frame.

        Args:
            bx: X position on board (mm from left edge of inner area)
            by: Y position on board (mm from top edge of inner area)

        Returns:
            (rx, ry): Position in robot base frame (mm)
        """
        return WORKSPACE.board_to_robot(bx, by)

    # -------------------------------------------------------------------------
    # CONVENIENCE: DIRECT CONVERSIONS
    # -------------------------------------------------------------------------

    def inner_px_to_robot(self, ix: float, iy: float) -> Tuple[float, float]:
        """Convert inner pixel coordinates directly to robot coordinates"""
        bx, by = self.inner_px_to_board_mm(ix, iy)
        return self.board_mm_to_robot_mm(bx, by)

    def board_mm_to_camera_px(self, bx: float, by: float) -> Optional[Tuple[float, float]]:
        """Convert board mm directly to camera pixels"""
        ix, iy = self.board_mm_to_inner_px(bx, by)
        wx, wy = self.inner_px_to_warped_px(ix, iy)
        return self.warped_px_to_camera_px(wx, wy)


# Need cv2 for perspectiveTransform
import cv2


# =============================================================================
# WORKSPACE BOUNDS CHECKING
# =============================================================================

@dataclass
class WorkspaceBounds:
    """Workspace boundaries in robot coordinates"""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def contains(self, x: float, y: float, z: float = 0) -> bool:
        """Check if point is within bounds"""
        return (self.x_min <= x <= self.x_max and
                self.y_min <= y <= self.y_max and
                self.z_min <= z <= self.z_max)

    def clamp(self, x: float, y: float, z: float = 0) -> Tuple[float, float, float]:
        """Clamp point to bounds"""
        x = max(self.x_min, min(self.x_max, x))
        y = max(self.y_min, min(self.y_max, y))
        z = max(self.z_min, min(self.z_max, z))
        return (x, y, z)


def compute_workspace_bounds() -> WorkspaceBounds:
    """
    Compute robot workspace bounds based on arm geometry and board placement.

    Returns bounds that encompass the board area reachable by the robot.
    """
    from config import ARM

    # Robot arm reach limits
    max_reach = ARM.total_reach
    min_reach = ARM.min_reach

    # Board corners in robot coordinates
    corners_board = [
        (0, 0),  # Top-left of inner board
        (BOARD.inner_width_mm, 0),  # Top-right
        (BOARD.inner_width_mm, BOARD.inner_height_mm),  # Bottom-right
        (0, BOARD.inner_height_mm),  # Bottom-left
    ]

    corners_robot = [WORKSPACE.board_to_robot(bx, by) for (bx, by) in corners_board]

    # Find bounding box of board in robot coords
    xs = [c[0] for c in corners_robot]
    ys = [c[1] for c in corners_robot]

    # Intersect with arm reach
    # The arm can reach points where sqrt(x² + y²) is between min_reach and max_reach
    # For simplicity, use rectangular bounds
    x_min = max(min(xs), -max_reach)
    x_max = min(max(xs), max_reach)
    y_min = max(min(ys), -max_reach)
    y_max = min(max(ys), max_reach)

    # Z bounds
    z_min = 0.0  # Table surface
    z_max = ARM.base_height + ARM.total_reach  # Max height

    return WorkspaceBounds(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        z_min=z_min,
        z_max=z_max
    )


def is_reachable(x: float, y: float, z: float = 0) -> bool:
    """
    Check if a point is reachable by the robot arm.

    This is a simplified check based on radial distance.
    The actual IK may still fail for some points within these bounds.
    """
    from config import ARM

    # Radial distance in XY plane
    r = np.sqrt(x * x + y * y)

    # Height relative to shoulder
    h = z - ARM.base_height

    # Distance from shoulder to target
    d = np.sqrt(r * r + h * h)

    # Check against arm reach
    max_reach = ARM.upper_arm_length + ARM.forearm_length + ARM.wrist_length + ARM.tool_length
    min_reach = abs(ARM.upper_arm_length - ARM.forearm_length)

    return min_reach <= d <= max_reach


# =============================================================================
# CALIBRATION HELPERS
# =============================================================================

def calibrate_board_to_robot_transform(
    board_points_mm: list,
    robot_points_mm: list
) -> Tuple[float, float, float, float]:
    """
    Calibrate the board-to-robot transform using measured correspondences.

    Args:
        board_points_mm: List of (bx, by) points in board mm
        robot_points_mm: List of (rx, ry) points in robot mm (measured)

    Returns:
        (origin_x, origin_y, origin_z, rotation_deg): Transform parameters
    """
    if len(board_points_mm) < 2 or len(board_points_mm) != len(robot_points_mm):
        raise ValueError("Need at least 2 corresponding points")

    board_pts = np.array(board_points_mm, dtype=np.float64)
    robot_pts = np.array(robot_points_mm, dtype=np.float64)

    # Compute centroid offset
    board_center = board_pts.mean(axis=0)
    robot_center = robot_pts.mean(axis=0)

    # Estimate rotation using first two points
    if len(board_points_mm) >= 2:
        db = board_pts[1] - board_pts[0]
        dr = robot_pts[1] - robot_pts[0]

        angle_board = np.arctan2(db[1], db[0])
        angle_robot = np.arctan2(dr[1], dr[0])
        rotation_deg = np.degrees(angle_robot - angle_board)
    else:
        rotation_deg = 0.0

    # Compute origin
    # robot = origin + R @ board
    # origin = robot_center - R @ board_center
    angle_rad = np.radians(rotation_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    rotated_board_center = np.array([
        board_center[0] * cos_a - board_center[1] * sin_a,
        board_center[0] * sin_a + board_center[1] * cos_a
    ])

    origin = robot_center - rotated_board_center

    return (origin[0], origin[1], 0.0, rotation_deg)
