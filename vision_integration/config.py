"""
Configuration for Vision-Robot Integration System

All physical parameters, calibration values, and tuning constants
are defined here. NO hard-coded values in other modules.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional, List
from pathlib import Path

# =============================================================================
# PATHS
# =============================================================================

BASE_DIR = Path(__file__).parent
CALIBRATION_DIR = BASE_DIR / "calibration"

# =============================================================================
# BOARD GEOMETRY (mm) - Physical board dimensions
# =============================================================================

@dataclass
class BoardGeometry:
    """Physical dimensions of the detection board"""
    inner_width_mm: float = 252.0      # Inner playing area width (25.2cm)
    inner_height_mm: float = 195.0     # Inner playing area height (19.5cm)
    border_mm: float = 10.0            # Black tape border width

    @property
    def outer_width_mm(self) -> float:
        return self.inner_width_mm + 2 * self.border_mm

    @property
    def outer_height_mm(self) -> float:
        return self.inner_height_mm + 2 * self.border_mm


BOARD = BoardGeometry()

# =============================================================================
# WARP/RECTIFICATION SETTINGS
# =============================================================================

@dataclass
class WarpSettings:
    """Settings for perspective warp/rectification"""
    px_per_mm: float = 4.0   # Resolution of warped image

    @property
    def warp_width(self) -> int:
        return int(BOARD.outer_width_mm * self.px_per_mm)

    @property
    def warp_height(self) -> int:
        return int(BOARD.outer_height_mm * self.px_per_mm)

    @property
    def margin_px(self) -> int:
        """Border margin in warped image pixels"""
        return int(BOARD.border_mm * self.px_per_mm)


WARP = WarpSettings()

# =============================================================================
# COLOR DETECTION (HSV ranges)
# =============================================================================

@dataclass
class ColorRange:
    """HSV color range for detection"""
    name: str
    lower: Tuple[int, int, int]
    upper: Tuple[int, int, int]
    bgr_draw: Tuple[int, int, int]  # Color for drawing (BGR)

    # Optional second range (for colors like red that wrap around)
    lower2: Optional[Tuple[int, int, int]] = None
    upper2: Optional[Tuple[int, int, int]] = None


# Color definitions (HSV ranges, OpenCV uses H:0-179, S:0-255, V:0-255)
COLORS = [
    ColorRange("BLUE", (105, 80, 25), (125, 255, 120), (255, 0, 0)),
    ColorRange("RED", (0, 120, 50), (6, 255, 255), (0, 0, 255),
               lower2=(173, 120, 50), upper2=(179, 255, 255)),
    ColorRange("PINK", (135, 30, 120), (175, 200, 255), (255, 0, 255)),
    ColorRange("ORANGE", (8, 150, 80), (18, 255, 255), (0, 165, 255)),
    ColorRange("YELLOW", (20, 110, 80), (38, 255, 255), (0, 255, 255)),
    ColorRange("PURPLE", (125, 60, 40), (155, 255, 255), (255, 0, 128)),
    ColorRange("CYAN", (80, 80, 60), (100, 255, 255), (255, 255, 0)),
]

# Detection parameters
MIN_CUBE_AREA_PX = 300      # Minimum contour area in pixels
CUBE_ASPECT_MIN = 0.6       # Minimum aspect ratio (w/h)
CUBE_ASPECT_MAX = 1.6       # Maximum aspect ratio

# =============================================================================
# ROBOT ARM GEOMETRY (mm) - From your calibrated measurements
# =============================================================================

@dataclass
class ArmGeometry:
    """Physical dimensions of the robot arm"""
    base_height: float = 66.76      # Height from table to shoulder axis
    shoulder_offset_x: float = 0.0  # Horizontal offset (usually 0)
    upper_arm_length: float = 31.1  # Shoulder to elbow
    forearm_length: float = 33.0    # Elbow to wrist
    wrist_length: float = 60.0      # Wrist pitch to wrist roll axis
    tool_length: float = 16.4       # Wrist roll to gripper tip

    @property
    def total_reach(self) -> float:
        """Maximum horizontal reach from base"""
        return (self.upper_arm_length + self.forearm_length +
                self.wrist_length + self.tool_length)

    @property
    def min_reach(self) -> float:
        """Minimum reach (arm folded)"""
        return abs(self.upper_arm_length - self.forearm_length) + self.wrist_length + self.tool_length


ARM = ArmGeometry()

# =============================================================================
# ROBOT ARM WORKSPACE TRANSFORM
# =============================================================================

@dataclass
class WorkspaceTransform:
    """
    Transform from board coordinates (mm) to robot base coordinates (mm).

    Board coordinate system (as seen by camera, looking down):
      - Origin: top-left corner of inner board
      - X: right (along board width)
      - Y: down (along board height)

    Robot coordinate system:
      - Origin: center of base rotation axis at table level
      - X: forward (arm pointing forward)
      - Y: left
      - Z: up

    The transform includes:
      - Translation: board origin relative to robot base
      - Rotation: alignment between board and robot axes
    """
    # Position of board origin (top-left of inner area) in robot coordinates
    board_origin_x: float = 100.0   # mm forward from robot base
    board_origin_y: float = 126.0   # mm left from robot base (half board width centered)
    board_origin_z: float = 0.0     # mm (table surface = Z=0 for robot)

    # Rotation: angle from board X-axis to robot X-axis (degrees)
    # 0 = board X points same as robot X (forward)
    # 90 = board X points toward robot Y (left)
    rotation_deg: float = 0.0

    def board_to_robot(self, board_x_mm: float, board_y_mm: float) -> Tuple[float, float]:
        """
        Convert board coordinates to robot XY coordinates.

        Args:
            board_x_mm: X position on board (mm from left edge of inner area)
            board_y_mm: Y position on board (mm from top edge of inner area)

        Returns:
            (robot_x, robot_y): Position in robot base frame (mm)
        """
        # Apply rotation
        angle_rad = np.radians(self.rotation_deg)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)

        # Rotate board coordinates
        rotated_x = board_x_mm * cos_a - board_y_mm * sin_a
        rotated_y = board_x_mm * sin_a + board_y_mm * cos_a

        # Translate to robot frame
        robot_x = self.board_origin_x + rotated_x
        robot_y = self.board_origin_y - rotated_y  # Y is flipped (board Y down, robot Y left)

        return (robot_x, robot_y)


WORKSPACE = WorkspaceTransform()

# =============================================================================
# MOTION PARAMETERS
# =============================================================================

@dataclass
class MotionParams:
    """Motion heights and speeds"""
    # Z heights (mm above table)
    z_safe: float = 60.0        # Safe travel height
    z_approach: float = 40.0    # Approach height before final descent
    z_pick: float = 20.0        # Pick height (gripper closes here)
    z_place: float = 25.0       # Place height (gripper opens here)

    # Gripper
    gripper_open: float = 90.0   # Open angle (degrees)
    gripper_closed: float = 30.0 # Closed angle for gripping (degrees)

    # Timing (ms)
    move_duration: int = 1500    # Duration for moves
    gripper_delay: int = 500     # Delay after gripper action
    settle_delay: int = 300      # Delay after reaching position

    # Tool pitch for picking (degrees from horizontal)
    pick_pitch: float = -45.0    # Angled down for picking (was -60, too steep)
    place_pitch: float = -45.0   # Angled down for placing
    travel_pitch: float = -30.0  # Slight angle for travel


MOTION = MotionParams()

# =============================================================================
# HOME POSITION
# =============================================================================

@dataclass
class HomePosition:
    """Home/park position for the arm"""
    base: float = 0.0
    shoulder: float = 45.0
    elbow: float = -45.0
    wrist_pitch: float = 0.0
    wrist_roll: float = 0.0
    gripper: float = 0.0  # Closed


HOME = HomePosition()

# =============================================================================
# SERIAL COMMUNICATION
# =============================================================================

@dataclass
class SerialConfig:
    """Serial port configuration for Arduino"""
    port: str = "/dev/cu.usbmodem11301"   # Mac USB port
    baudrate: int = 115200
    timeout: float = 2.0

    # Alternative ports to try
    alt_ports: List[str] = field(default_factory=lambda: [
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/cu.usbserial-*",
        "/dev/cu.usbmodem*",
    ])


SERIAL = SerialConfig()

# =============================================================================
# CAMERA SETTINGS
# =============================================================================

@dataclass
class CameraConfig:
    """Camera configuration"""
    index: int = 0              # Camera index to try first
    width: int = 1920
    height: int = 1080

    # Calibration file paths (relative to calibration dir)
    matrix_file: str = "cameraMatrix.pkl"
    dist_file: str = "dist.pkl"


CAMERA = CameraConfig()

# =============================================================================
# SMOOTHING PARAMETERS
# =============================================================================

@dataclass
class SmoothingParams:
    """Smoothing parameters for detection stability"""
    corner_alpha: float = 0.25      # Corner detection smoothing (0-1, lower = smoother)
    divider_beta: float = 0.2       # Divider position smoothing
    position_filter_frames: int = 5  # Number of frames to average for position


SMOOTHING = SmoothingParams()

# =============================================================================
# DEBUG / VISUALIZATION
# =============================================================================

@dataclass
class DebugConfig:
    """Debug and visualization settings"""
    show_windows: bool = True       # Show OpenCV debug windows
    print_detections: bool = True   # Print detection info
    print_interval: int = 10        # Print every N frames
    dry_run: bool = False           # If True, don't send commands to robot
    save_frames: bool = False       # Save debug frames to disk


DEBUG = DebugConfig()
