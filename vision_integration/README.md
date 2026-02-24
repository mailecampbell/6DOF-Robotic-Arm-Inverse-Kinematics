# Vision-Robot Integration for Pick and Place

This module integrates OpenCV-based object detection with a 6-DOF robotic arm for autonomous pick-and-place operations.

## Overview

The system detects colored cubes on a white board, selects a target, computes a "ghost" (mirror) drop location across a vertical divider, converts coordinates from camera pixels to robot millimeters, and executes the pick-and-place motion.

### Key Features

- **Board Detection**: Automatically detects the white playing area bounded by black tape
- **Object Detection**: HSV-based color detection for multiple cube colors (BLUE, RED, PINK, YELLOW, ORANGE, PURPLE, CYAN)
- **Ghost/Mirror Logic**: Computes drop location by mirroring across a vertical divider line
- **Coordinate Transformation**: Full pipeline from camera pixels to robot base frame coordinates
- **Motion Control**: State machine for reliable 10-step pick-and-place sequence
- **Multiple Modes**: Autonomous, manual (keyboard), and vision-only modes

## File Structure

```
vision_integration/
├── main.py           # Main orchestrator and entry point
├── config.py         # All configuration parameters
├── vision.py         # Vision pipeline (camera, detection, ghost computation)
├── coordinates.py    # Coordinate system conversions
├── robot_serial.py   # Serial communication with Arduino
├── motion.py         # Motion controller and pick-place state machine
├── calibration/      # Camera calibration files (optional)
│   ├── cameraMatrix.pkl
│   └── dist.pkl
└── README.md         # This file
```

## Coordinate Systems

The system uses four coordinate frames:

### 1. Camera Pixels (u, v)
- Origin: Top-left of camera image
- u: Right (0 to image_width)
- v: Down (0 to image_height)
- Unit: Pixels

### 2. Warped/Inner Pixels (ix, iy)
- Origin: Top-left of inner board (inside black tape border)
- ix: Right (0 to inner_width_px)
- iy: Down (0 to inner_height_px)
- Unit: Pixels (after perspective rectification)

### 3. Board Millimeters (bx, by)
- Origin: Top-left of inner board
- bx: Right (0 to 252 mm)
- by: Down (0 to 195 mm)
- Unit: Millimeters

### 4. Robot Base Frame (rx, ry, rz)
- Origin: Center of base rotation axis at table level
- rx: Forward (arm points +X when base=0)
- ry: Left (right-hand rule)
- rz: Up
- Unit: Millimeters

## Configuration

All parameters are defined in `config.py`. Key settings:

### Board Geometry
```python
@dataclass
class BoardGeometry:
    inner_width_mm: float = 252.0   # Inner playing area width
    inner_height_mm: float = 195.0  # Inner playing area height
    border_mm: float = 10.0         # Black tape border width
```

### Workspace Transform
```python
@dataclass
class WorkspaceTransform:
    board_origin_x: float = 100.0   # Board origin X in robot frame (mm)
    board_origin_y: float = 126.0   # Board origin Y in robot frame (mm)
    board_origin_z: float = 0.0     # Table surface
    rotation_deg: float = 0.0       # Rotation between frames
```

### Motion Parameters
```python
@dataclass
class MotionParams:
    z_safe: float = 80.0        # Safe travel height (mm)
    z_pick: float = 15.0        # Pick height (mm)
    z_place: float = 20.0       # Place height (mm)
    pick_pitch: float = -60.0   # Tool angle for picking (degrees)
```

### Color Detection (HSV)
```python
COLORS = [
    ColorRange("BLUE", (105, 80, 25), (125, 255, 120), (255, 0, 0)),
    ColorRange("RED", (0, 120, 50), (6, 255, 255), (0, 0, 255),
               lower2=(173, 120, 50), upper2=(179, 255, 255)),
    # ... more colors
]
```

## Installation

### Requirements
```bash
pip install opencv-python numpy pyserial
```

### Camera Calibration (Optional)
If you have camera calibration data:
1. Place `cameraMatrix.pkl` and `dist.pkl` in `calibration/`
2. The system will automatically apply distortion correction

## Usage

### Autonomous Mode (Default)
```bash
python main.py
```
Continuously detects objects and performs pick-and-place operations.

**Controls:**
- `q`: Quit
- `p`: Pause/Resume
- `h`: Return to home position

### Manual Mode
```bash
python main.py --manual
```
Keyboard control with visual feedback.

**Controls:**
- Arrow keys: Move X/Y
- `+`/`-`: Move Z
- `o`: Open gripper
- `l`: Close gripper
- `h`: Home position
- `t`: Test pick at current detection
- `Space`: Execute pick-and-place
- `q`: Quit

### Vision-Only Mode
```bash
python main.py --vision-only
```
Test detection without robot commands.

### Dry Run
```bash
python main.py --dry-run
```
Shows robot commands without sending them.

### Other Options
```bash
python main.py --help
```

## Ghost/Mirror Logic

The ghost point is computed by mirroring the object's center across a vertical divider line:

```python
def mirror_box_across_vertical_line(box_xywh, x_div_px):
    """
    Mirror a bounding box across a vertical line.

    Args:
        box_xywh: (x, y, width, height) of bounding box in inner pixels
        x_div_px: X coordinate of vertical divider in inner pixels

    Returns:
        (x_ghost, y_ghost, width, height) of mirrored box
    """
    x, y, w, h = box_xywh
    cx = x + w / 2.0        # Object center X
    cy = y + h / 2.0        # Object center Y

    # Mirror X across divider: x_ghost = 2 * divider - x
    cx_ghost = 2.0 * x_div_px - cx

    # Y stays the same
    xg = int(round(cx_ghost - w / 2.0))
    yg = int(round(cy - h / 2.0))

    return (xg, yg, w, h)
```

## Pick-and-Place Motion Sequence

The motion controller executes a 10-step state machine:

1. **MOVING_TO_PICK_ABOVE**: Move to safe height above pick position
2. **DESCENDING_TO_PICK**: Lower to pick height
3. **GRIPPING**: Close gripper
4. **ASCENDING_WITH_OBJECT**: Lift to safe height
5. **MOVING_TO_PLACE_ABOVE**: Travel to above ghost/drop position
6. **DESCENDING_TO_PLACE**: Lower to place height
7. **RELEASING**: Open gripper
8. **ASCENDING_AFTER_PLACE**: Retract to safe height
9. **RETURNING_HOME**: Return to home position
10. **COMPLETED**: Operation finished

Each step checks for errors and abort requests. If a failure occurs, the system attempts to open the gripper and return home safely.

## Arduino Serial Commands

The robot communicates via serial using these commands:

| Command | Format | Description |
|---------|--------|-------------|
| G | `G X Y Z [pitch] [roll]` | Move to XYZ position |
| J | `J b s e wp wr g` | Set joint angles directly |
| H | `H` | Move to home position |
| K | `K` | Move to park position |
| P | `P` | Query current position |
| C | `C` | Query current joint angles |
| W | `W` | Query workspace bounds |
| O | `O` | Open gripper |
| L | `L` | Close gripper |

## Calibration

### Board-to-Robot Transform Calibration

1. Place a marker at a known position on the board (e.g., top-left corner)
2. Command the robot to touch that position
3. Record both board and robot coordinates
4. Repeat for 3-4 points
5. Update `WORKSPACE` in `config.py`

Example calibration session:
```
Board Point 1: (0, 0) mm -> Robot: (100, 126) mm
Board Point 2: (252, 0) mm -> Robot: (100, -126) mm
Board Point 3: (0, 195) mm -> Robot: (295, 126) mm
```

### Camera Calibration

Use OpenCV's calibration with a checkerboard pattern:
```python
import cv2
import pickle

# After calibration...
pickle.dump(camera_matrix, open('calibration/cameraMatrix.pkl', 'wb'))
pickle.dump(dist_coeffs, open('calibration/dist.pkl', 'wb'))
```

## Troubleshooting

### Robot Not Connecting
1. Check USB cable connection
2. Verify serial port in `config.py` (default: `/dev/ttyUSB0`)
3. Check permissions: `sudo chmod 666 /dev/ttyUSB0`
4. Test with Arduino Serial Monitor

### Camera Not Opening
1. Check camera index in `config.py` (try 0, 1, 2)
2. Verify camera is not in use by another application
3. Check permissions for camera access

### Board Not Detected
1. Ensure black tape border is clearly visible
2. Adjust lighting to reduce glare
3. Check `MIN_BOARD_AREA` and contour detection thresholds in `vision.py`

### Objects Not Detected
1. Print detection debug info: `python main.py -v`
2. Adjust HSV ranges in `config.py` for your lighting
3. Check `MIN_CUBE_AREA_PX` threshold

### Inaccurate Positioning
1. Re-calibrate workspace transform
2. Check robot arm calibration (servo offsets)
3. Verify board dimensions match `config.py`

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                         main.py                              │
│                    VisionRobotApp                            │
│  ┌──────────────────────────────────────────────────────┐   │
│  │                 Main Control Loop                     │   │
│  │  1. Process frame -> detect objects                   │   │
│  │  2. Choose target -> compute ghost point              │   │
│  │  3. Convert coordinates -> robot frame                │   │
│  │  4. Execute pick-and-place -> motion controller       │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│   vision.py   │    │coordinates.py │    │   motion.py   │
│───────────────│    │───────────────│    │───────────────│
│ VisionPipeline│    │ Coordinate-   │    │ Motion-       │
│ - detect_board│    │   Converter   │    │   Controller  │
│ - detect_cubes│    │───────────────│    │ PickPlace-    │
│ - find_divider│    │ inner_px →    │    │   Coordinator │
│ - mirror_box  │    │   board_mm →  │    │───────────────│
│ - choose_tgt  │    │   robot_mm    │    │ 10-step state │
└───────────────┘    └───────────────┘    │   machine     │
                                          └───────────────┘
                                                  │
                                                  ▼
                                          ┌───────────────┐
                                          │robot_serial.py│
                                          │───────────────│
                                          │ RobotSerial   │
                                          │ - connect     │
                                          │ - move_to     │
                                          │ - gripper     │
                                          └───────────────┘
                                                  │
                                                  ▼
                                          ┌───────────────┐
                                          │   Arduino     │
                                          │ RobotArmIK    │
                                          │───────────────│
                                          │ IK solver     │
                                          │ Servo control │
                                          └───────────────┘
```

## API Reference

### VisionPipeline

```python
class VisionPipeline:
    def start() -> bool
        """Initialize camera. Returns True on success."""

    def stop()
        """Release camera resources."""

    def process_frame() -> Optional[BoardState]
        """Capture and process one frame. Returns BoardState with all detections."""
```

### CoordinateConverter

```python
class CoordinateConverter:
    def inner_px_to_board_mm(ix, iy) -> Tuple[float, float]
        """Convert inner pixel coords to board mm."""

    def board_mm_to_robot_mm(bx, by) -> Tuple[float, float]
        """Convert board mm to robot base frame mm."""

    def inner_px_to_robot(ix, iy) -> Tuple[float, float]
        """Direct conversion from inner pixels to robot mm."""
```

### MotionController

```python
class MotionController:
    def execute_pick_place(task: PickPlaceTask) -> MotionResult
        """Execute complete pick-and-place operation."""

    def go_home() -> bool
        """Move to home position."""

    def abort()
        """Request abort of current operation."""
```

### RobotSerial

```python
class RobotSerial:
    def connect(port=None) -> bool
        """Connect to Arduino. Auto-detects port if not specified."""

    def move_to(x, y, z, pitch=0, roll=0) -> CommandResponse
        """Move to XYZ position with optional orientation."""

    def open_gripper() -> CommandResponse
    def close_gripper() -> CommandResponse

    def get_position() -> Optional[Position]
        """Query current position via forward kinematics."""
```

## License

This project is provided as-is for educational and hobby use.
