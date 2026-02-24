"""
Motion Pipeline for Pick and Place Operations

This module implements the complete motion sequence:
1. Detect object
2. Compute pick point
3. Compute ghost (mirror) drop point
4. Execute pick-and-place motion

The motion sequence follows these steps:
1. Move above pick position (Z_SAFE)
2. Descend to approach height (Z_APPROACH)
3. Descend to pick height (Z_PICK)
4. Close gripper
5. Ascend to safe height (Z_SAFE)
6. Move above ghost/drop position (Z_SAFE)
7. Descend to place height (Z_PLACE)
8. Open gripper
9. Ascend to safe height (Z_SAFE)
10. Return to home position
"""

import time
from typing import Optional, Tuple, List, Callable
from dataclasses import dataclass, field
from enum import Enum

from config import MOTION, HOME, DEBUG, WORKSPACE
from robot_serial import RobotSerial, CommandStatus, CommandResponse
from coordinates import CoordinateConverter, is_reachable


# =============================================================================
# MOTION STATE
# =============================================================================

class MotionState(Enum):
    """States in the pick-and-place state machine"""
    IDLE = "IDLE"
    MOVING_TO_PICK_ABOVE = "MOVING_TO_PICK_ABOVE"
    DESCENDING_TO_PICK = "DESCENDING_TO_PICK"
    GRIPPING = "GRIPPING"
    ASCENDING_WITH_OBJECT = "ASCENDING_WITH_OBJECT"
    MOVING_TO_PLACE_ABOVE = "MOVING_TO_PLACE_ABOVE"
    DESCENDING_TO_PLACE = "DESCENDING_TO_PLACE"
    RELEASING = "RELEASING"
    ASCENDING_AFTER_PLACE = "ASCENDING_AFTER_PLACE"
    RETURNING_HOME = "RETURNING_HOME"
    COMPLETED = "COMPLETED"
    ABORTED = "ABORTED"
    ERROR = "ERROR"


class MotionResult(Enum):
    """Result of a motion operation"""
    SUCCESS = "SUCCESS"
    UNREACHABLE = "UNREACHABLE"
    ROBOT_ERROR = "ROBOT_ERROR"
    TIMEOUT = "TIMEOUT"
    ABORTED = "ABORTED"


@dataclass
class PickPlaceTask:
    """A pick and place task"""
    pick_x: float           # Pick position X (robot mm)
    pick_y: float           # Pick position Y (robot mm)
    place_x: float          # Place position X (robot mm)
    place_y: float          # Place position Y (robot mm)
    object_color: str = ""  # Color of object being picked
    z_pick: float = MOTION.z_pick
    z_place: float = MOTION.z_place
    z_safe: float = MOTION.z_safe
    pick_pitch: float = MOTION.pick_pitch
    place_pitch: float = MOTION.place_pitch


@dataclass
class MotionStatus:
    """Current status of motion system"""
    state: MotionState = MotionState.IDLE
    current_task: Optional[PickPlaceTask] = None
    error_message: str = ""
    last_command_response: Optional[CommandResponse] = None


# =============================================================================
# MOTION CONTROLLER
# =============================================================================

class MotionController:
    """
    High-level motion controller for pick and place operations.

    Handles:
    - Pick and place sequence execution
    - State machine management
    - Error handling and recovery
    - Motion callbacks for UI updates
    """

    def __init__(self, robot: RobotSerial):
        self.robot = robot
        self.status = MotionStatus()
        self.coordinator = CoordinateConverter()

        # Callbacks for UI integration
        self.on_state_change: Optional[Callable[[MotionState], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None
        self.on_progress: Optional[Callable[[str], None]] = None

        # Abort flag
        self._abort_requested = False

    def _set_state(self, state: MotionState, message: str = ""):
        """Update state and notify callbacks"""
        self.status.state = state
        if self.on_state_change:
            self.on_state_change(state)
        if message and self.on_progress:
            self.on_progress(message)
        if DEBUG.print_detections:
            print(f"[MOTION] State: {state.value} - {message}")

    def _check_abort(self) -> bool:
        """Check if abort was requested"""
        if self._abort_requested:
            self._set_state(MotionState.ABORTED, "Operation aborted")
            return True
        return False

    def abort(self):
        """Request abort of current operation"""
        self._abort_requested = True

    def reset(self):
        """Reset state machine"""
        self._abort_requested = False
        self.status = MotionStatus()

    # -------------------------------------------------------------------------
    # MOTION PRIMITIVES
    # -------------------------------------------------------------------------

    def _move_to(self, x: float, y: float, z: float,
                 pitch: float = 0, roll: float = 0,
                 description: str = "") -> bool:
        """
        Execute a move and wait for completion.

        Returns True if successful, False otherwise.
        """
        if self._check_abort():
            return False

        # Check reachability
        if not is_reachable(x, y, z):
            self.status.error_message = f"Position unreachable: ({x:.1f}, {y:.1f}, {z:.1f})"
            if self.on_error:
                self.on_error(self.status.error_message)
            return False

        if self.on_progress:
            self.on_progress(f"Moving to {description or f'({x:.1f}, {y:.1f}, {z:.1f})'}")

        response = self.robot.move_to(x, y, z, pitch, roll)
        self.status.last_command_response = response

        if response.status == CommandStatus.ERR:
            self.status.error_message = response.message
            if self.on_error:
                self.on_error(response.message)
            return False

        # Wait for move to complete
        time.sleep(MOTION.move_duration / 1000.0)
        time.sleep(MOTION.settle_delay / 1000.0)

        return True

    def _grip(self, close: bool, description: str = "") -> bool:
        """
        Execute gripper action.

        Args:
            close: True to close, False to open
        """
        if self._check_abort():
            return False

        if self.on_progress:
            action = "Closing" if close else "Opening"
            self.on_progress(f"{action} gripper - {description}")

        if close:
            response = self.robot.close_gripper()
        else:
            response = self.robot.open_gripper()

        self.status.last_command_response = response

        if response.status == CommandStatus.ERR:
            self.status.error_message = response.message
            return False

        time.sleep(MOTION.gripper_delay / 1000.0)
        return True

    # -------------------------------------------------------------------------
    # PICK AND PLACE SEQUENCE
    # -------------------------------------------------------------------------

    def execute_pick_place(self, task: PickPlaceTask) -> MotionResult:
        """
        Execute a complete pick and place operation.

        Args:
            task: PickPlaceTask with pick and place coordinates

        Returns:
            MotionResult indicating success or failure type
        """
        self.reset()
        self.status.current_task = task

        print(f"\n{'='*60}")
        print(f"PICK AND PLACE: {task.object_color}")
        print(f"  Pick:  ({task.pick_x:.1f}, {task.pick_y:.1f})")
        print(f"  Place: ({task.place_x:.1f}, {task.place_y:.1f})")
        print(f"{'='*60}\n")

        # Step 1: Move above pick position
        self._set_state(MotionState.MOVING_TO_PICK_ABOVE, "Moving above pick position")
        if not self._move_to(task.pick_x, task.pick_y, task.z_safe,
                             pitch=MOTION.travel_pitch, description="above pick"):
            return self._handle_failure()

        # Step 2: Descend to pick position
        self._set_state(MotionState.DESCENDING_TO_PICK, "Descending to pick")
        if not self._move_to(task.pick_x, task.pick_y, task.z_pick,
                             pitch=task.pick_pitch, description="pick height"):
            return self._handle_failure()

        # Step 3: Close gripper
        self._set_state(MotionState.GRIPPING, "Gripping object")
        if not self._grip(close=True, description=f"gripping {task.object_color}"):
            return self._handle_failure()

        # Step 4: Ascend with object
        self._set_state(MotionState.ASCENDING_WITH_OBJECT, "Lifting object")
        if not self._move_to(task.pick_x, task.pick_y, task.z_safe,
                             pitch=task.pick_pitch, description="lifting"):
            return self._handle_failure()

        # Step 5: Move above place position
        self._set_state(MotionState.MOVING_TO_PLACE_ABOVE, "Moving to place position")
        if not self._move_to(task.place_x, task.place_y, task.z_safe,
                             pitch=MOTION.travel_pitch, description="above place"):
            return self._handle_failure()

        # Step 6: Descend to place position
        self._set_state(MotionState.DESCENDING_TO_PLACE, "Descending to place")
        if not self._move_to(task.place_x, task.place_y, task.z_place,
                             pitch=task.place_pitch, description="place height"):
            return self._handle_failure()

        # Step 7: Open gripper
        self._set_state(MotionState.RELEASING, "Releasing object")
        if not self._grip(close=False, description="releasing"):
            return self._handle_failure()

        # Step 8: Ascend after place
        self._set_state(MotionState.ASCENDING_AFTER_PLACE, "Retracting")
        if not self._move_to(task.place_x, task.place_y, task.z_safe,
                             pitch=MOTION.travel_pitch, description="retracting"):
            return self._handle_failure()

        # Step 9: Return home
        self._set_state(MotionState.RETURNING_HOME, "Returning to home")
        self.robot.home()
        time.sleep(MOTION.move_duration / 1000.0)

        # Success!
        self._set_state(MotionState.COMPLETED, "Pick and place completed!")
        return MotionResult.SUCCESS

    def _handle_failure(self) -> MotionResult:
        """Handle a motion failure"""
        if self._abort_requested:
            self._set_state(MotionState.ABORTED)
            return MotionResult.ABORTED

        self._set_state(MotionState.ERROR, self.status.error_message)

        # Try to return to safe position
        try:
            self.robot.open_gripper()
            time.sleep(0.5)
            self.robot.home()
        except:
            pass

        if "unreachable" in self.status.error_message.lower():
            return MotionResult.UNREACHABLE
        return MotionResult.ROBOT_ERROR

    # -------------------------------------------------------------------------
    # CONVENIENCE METHODS
    # -------------------------------------------------------------------------

    def go_home(self) -> bool:
        """Move to home position"""
        response = self.robot.home()
        return response.status != CommandStatus.ERR

    def go_to_position(self, x: float, y: float, z: float,
                       pitch: float = 0) -> bool:
        """Move to a specific position"""
        return self._move_to(x, y, z, pitch, description=f"({x}, {y}, {z})")

    def test_pick_position(self, x: float, y: float) -> bool:
        """
        Test picking at a position without actually gripping.

        Useful for calibration.
        """
        # Move above
        if not self._move_to(x, y, MOTION.z_safe, MOTION.travel_pitch):
            return False

        # Descend
        if not self._move_to(x, y, MOTION.z_pick, MOTION.pick_pitch):
            return False

        # Pause to verify position
        time.sleep(2.0)

        # Ascend
        if not self._move_to(x, y, MOTION.z_safe, MOTION.travel_pitch):
            return False

        return True


# =============================================================================
# PICK AND PLACE COORDINATOR
# =============================================================================

class PickPlaceCoordinator:
    """
    Coordinates vision and motion for autonomous pick and place.

    This is the top-level controller that:
    - Gets detections from vision
    - Computes pick and ghost points
    - Converts to robot coordinates
    - Executes motion
    """

    def __init__(self, robot: RobotSerial, coord_converter: CoordinateConverter):
        self.robot = robot
        self.coord_converter = coord_converter
        self.motion = MotionController(robot)

        # Statistics
        self.picks_attempted = 0
        self.picks_succeeded = 0

    def execute_pick_place_for_detection(
        self,
        pick_board_mm: Tuple[float, float],
        place_board_mm: Tuple[float, float],
        object_color: str = ""
    ) -> MotionResult:
        """
        Execute pick and place given board coordinates.

        Args:
            pick_board_mm: (x, y) pick position in board mm
            place_board_mm: (x, y) place position in board mm
            object_color: Color name for logging

        Returns:
            MotionResult
        """
        # Convert to robot coordinates
        pick_robot = self.coord_converter.board_mm_to_robot_mm(*pick_board_mm)
        place_robot = self.coord_converter.board_mm_to_robot_mm(*place_board_mm)

        print(f"\nCoordinate conversion:")
        print(f"  Pick board:  ({pick_board_mm[0]:.1f}, {pick_board_mm[1]:.1f}) mm")
        print(f"  Pick robot:  ({pick_robot[0]:.1f}, {pick_robot[1]:.1f}) mm")
        print(f"  Place board: ({place_board_mm[0]:.1f}, {place_board_mm[1]:.1f}) mm")
        print(f"  Place robot: ({place_robot[0]:.1f}, {place_robot[1]:.1f}) mm")

        # Create task
        task = PickPlaceTask(
            pick_x=pick_robot[0],
            pick_y=pick_robot[1],
            place_x=place_robot[0],
            place_y=place_robot[1],
            object_color=object_color
        )

        # Execute
        self.picks_attempted += 1
        result = self.motion.execute_pick_place(task)

        if result == MotionResult.SUCCESS:
            self.picks_succeeded += 1

        return result

    def get_stats(self) -> dict:
        """Get pick/place statistics"""
        return {
            "attempted": self.picks_attempted,
            "succeeded": self.picks_succeeded,
            "success_rate": (
                self.picks_succeeded / self.picks_attempted * 100
                if self.picks_attempted > 0 else 0
            )
        }
