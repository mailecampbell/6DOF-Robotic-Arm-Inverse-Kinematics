#!/usr/bin/env python3
"""
Vision-Robot Integration Main Controller

This is the main orchestrator that ties together:
1. Vision detection (camera, board detection, object detection)
2. Ghost/mirror computation for drop locations
3. Coordinate conversion (pixel -> board mm -> robot mm)
4. Robot motion control (serial communication with Arduino)

Usage:
    # Full autonomous operation:
    python main.py

    # Manual mode (keyboard control):
    python main.py --manual

    # Test vision only (no robot):
    python main.py --vision-only

    # Dry run (show commands without sending):
    python main.py --dry-run
"""

import argparse
import sys
import time
import cv2
import numpy as np
from typing import Optional, Tuple, List

from config import (
    DEBUG, MOTION, WORKSPACE, BOARD, WARP, COLORS,
    CameraConfig, MotionParams
)
from vision import (
    VisionPipeline, Detection, BoardState,
    mirror_box_across_vertical_line, choose_target
)
from coordinates import CoordinateConverter, is_reachable
from robot_serial import RobotSerial, MockRobotSerial, CommandStatus
from motion import (
    MotionController, PickPlaceCoordinator, PickPlaceTask,
    MotionResult, MotionState
)


# =============================================================================
# MAIN APPLICATION
# =============================================================================

class VisionRobotApp:
    """
    Main application that integrates vision and robot control.

    This class orchestrates the complete pick-and-place workflow:
    1. Initialize camera and robot connection
    2. Detect board and establish coordinate transform
    3. Detect colored objects on the board
    4. Choose target object to pick
    5. Compute ghost/mirror drop location
    6. Convert coordinates to robot frame
    7. Execute pick-and-place motion
    8. Repeat
    """

    def __init__(self, args):
        self.args = args

        # Configure debug mode
        DEBUG.dry_run = args.dry_run
        DEBUG.show_windows = not args.headless
        DEBUG.print_detections = args.verbose

        # Components
        self.vision: Optional[VisionPipeline] = None
        self.robot: Optional[RobotSerial] = None
        self.coordinator: Optional[PickPlaceCoordinator] = None
        self.converter = CoordinateConverter()

        # State
        self.running = False
        self.paused = False
        self.board_state: Optional[BoardState] = None

        # Statistics
        self.total_picks = 0
        self.successful_picks = 0
        self.failed_picks = 0

    def initialize(self) -> bool:
        """Initialize all components. Returns True if successful."""

        print("=" * 60)
        print("VISION-ROBOT INTEGRATION SYSTEM")
        print("=" * 60)

        # Initialize vision
        print("\n[1/3] Initializing vision system...")
        self.vision = VisionPipeline()

        if not self.vision.start():
            print("ERROR: Failed to initialize camera")
            return False

        print(f"  Camera initialized: {self.vision.frame_width}x{self.vision.frame_height}")

        # Initialize robot
        print("\n[2/3] Connecting to robot...")

        if self.args.vision_only or self.args.dry_run:
            print("  Using mock robot (vision-only or dry-run mode)")
            self.robot = MockRobotSerial()
        else:
            self.robot = RobotSerial()
            if not self.robot.connect():
                print("ERROR: Failed to connect to robot")
                print("  Check USB connection and port settings in config.py")
                # Fall back to mock
                if self.args.allow_mock:
                    print("  Falling back to mock robot...")
                    self.robot = MockRobotSerial()
                else:
                    return False

        # Initialize motion coordinator
        print("\n[3/3] Initializing motion coordinator...")
        self.coordinator = PickPlaceCoordinator(self.robot, self.converter)

        # Register callbacks
        self.coordinator.motion.on_state_change = self._on_motion_state_change
        self.coordinator.motion.on_error = self._on_motion_error
        self.coordinator.motion.on_progress = self._on_motion_progress

        print("\n" + "=" * 60)
        print("INITIALIZATION COMPLETE")
        print("=" * 60)

        return True

    def shutdown(self):
        """Clean shutdown of all components."""
        print("\nShutting down...")

        if self.vision:
            self.vision.stop()

        if self.robot:
            # Return to home and disconnect
            try:
                self.robot.open_gripper()
                self.robot.home()
                self.robot.disconnect()
            except:
                pass

        cv2.destroyAllWindows()

        # Print final statistics
        self._print_statistics()

    def _on_motion_state_change(self, state: MotionState):
        """Callback when motion state changes."""
        if DEBUG.print_detections:
            print(f"  Motion state: {state.value}")

    def _on_motion_error(self, message: str):
        """Callback when motion error occurs."""
        print(f"  MOTION ERROR: {message}")

    def _on_motion_progress(self, message: str):
        """Callback for motion progress updates."""
        if DEBUG.print_detections:
            print(f"  {message}")

    def _print_statistics(self):
        """Print final pick-and-place statistics."""
        print("\n" + "=" * 60)
        print("SESSION STATISTICS")
        print("=" * 60)
        print(f"  Total picks attempted: {self.total_picks}")
        print(f"  Successful picks: {self.successful_picks}")
        print(f"  Failed picks: {self.failed_picks}")
        if self.total_picks > 0:
            rate = self.successful_picks / self.total_picks * 100
            print(f"  Success rate: {rate:.1f}%")

    # -------------------------------------------------------------------------
    # VISION PROCESSING
    # -------------------------------------------------------------------------

    def process_frame(self) -> Optional[Tuple[Detection, Tuple[float, float]]]:
        """
        Process one frame from the camera.

        Returns:
            Tuple of (target_detection, ghost_point_inner_px) if target found,
            None otherwise.
        """
        if not self.vision:
            return None

        # Get frame and run detection
        self.board_state = self.vision.process_frame()

        if self.board_state is None:
            return None

        # Need valid board detection
        if not self.board_state.board_detected:
            if DEBUG.print_detections:
                print("  No board detected")
            return None

        # Need divider for ghost computation
        if self.board_state.divider_x_inner is None:
            if DEBUG.print_detections:
                print("  No divider detected")
            return None

        # Need objects to pick
        if not self.board_state.detections:
            return None

        # Choose target (leftmost, smallest)
        target = choose_target(
            self.board_state.detections,
            self.board_state.divider_x_inner
        )

        if target is None:
            return None

        # Compute ghost point
        ghost_box = mirror_box_across_vertical_line(
            (target.cx_inner, target.cy_inner, target.width_px, target.height_px),
            self.board_state.divider_x_inner
        )

        ghost_center_inner = (
            ghost_box[0] + ghost_box[2] / 2.0,
            ghost_box[1] + ghost_box[3] / 2.0
        )

        return (target, ghost_center_inner)

    def convert_to_robot_coords(
        self,
        pick_inner_px: Tuple[float, float],
        place_inner_px: Tuple[float, float]
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Convert pick and place points from inner pixels to robot mm.

        Returns:
            (pick_robot_mm, place_robot_mm)
        """
        # Inner pixels -> board mm
        pick_board = self.converter.inner_px_to_board_mm(*pick_inner_px)
        place_board = self.converter.inner_px_to_board_mm(*place_inner_px)

        # Board mm -> robot mm
        pick_robot = self.converter.board_mm_to_robot_mm(*pick_board)
        place_robot = self.converter.board_mm_to_robot_mm(*place_board)

        return (pick_robot, place_robot)

    # -------------------------------------------------------------------------
    # MAIN LOOPS
    # -------------------------------------------------------------------------

    def run_autonomous(self):
        """
        Run in autonomous mode.

        Continuously:
        1. Look for objects
        2. Pick and place to ghost location
        3. Repeat
        """
        print("\n" + "=" * 60)
        print("AUTONOMOUS MODE")
        print("  Press 'q' to quit")
        print("  Press 'p' to pause/resume")
        print("  Press 'h' to return to home")
        print("=" * 60 + "\n")

        self.running = True
        frame_count = 0
        stable_frames = 0
        last_target = None
        REQUIRED_STABLE_FRAMES = 10  # Require stable detection before picking

        while self.running:
            # Process frame
            result = self.process_frame()

            # Display
            if self.board_state and self.board_state.display_frame is not None:
                self._draw_overlay(self.board_state, result)
                cv2.imshow("Vision-Robot", self.board_state.display_frame)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.running = False
                break
            elif key == ord('p'):
                self.paused = not self.paused
                print(f"{'PAUSED' if self.paused else 'RESUMED'}")
            elif key == ord('h'):
                print("Returning to home...")
                self.robot.home()

            if self.paused:
                continue

            # Check for stable target
            if result is not None:
                target, ghost_pt = result

                # Check if same target as last frame
                if last_target is not None:
                    dist = np.sqrt(
                        (target.cx_inner - last_target.cx_inner) ** 2 +
                        (target.cy_inner - last_target.cy_inner) ** 2
                    )
                    if dist < 10:  # Within 10 pixels
                        stable_frames += 1
                    else:
                        stable_frames = 0
                else:
                    stable_frames = 1

                last_target = target

                # If stable enough, execute pick-and-place
                if stable_frames >= REQUIRED_STABLE_FRAMES:
                    print(f"\n{'='*50}")
                    print(f"STABLE TARGET DETECTED: {target.color}")
                    print(f"  Location: ({target.cx_inner:.1f}, {target.cy_inner:.1f}) inner px")
                    print(f"  Ghost: ({ghost_pt[0]:.1f}, {ghost_pt[1]:.1f}) inner px")

                    # Convert coordinates
                    pick_inner = (target.cx_inner, target.cy_inner)
                    pick_robot, place_robot = self.convert_to_robot_coords(
                        pick_inner, ghost_pt
                    )

                    print(f"  Pick robot:  ({pick_robot[0]:.1f}, {pick_robot[1]:.1f}) mm")
                    print(f"  Place robot: ({place_robot[0]:.1f}, {place_robot[1]:.1f}) mm")

                    # Check reachability
                    if not is_reachable(pick_robot[0], pick_robot[1], MOTION.z_pick):
                        print("  WARNING: Pick position may be unreachable")
                    if not is_reachable(place_robot[0], place_robot[1], MOTION.z_place):
                        print("  WARNING: Place position may be unreachable")

                    # Execute pick-and-place
                    self.total_picks += 1
                    result = self.coordinator.execute_pick_place_for_detection(
                        pick_board_mm=self.converter.inner_px_to_board_mm(*pick_inner),
                        place_board_mm=self.converter.inner_px_to_board_mm(*ghost_pt),
                        object_color=target.color
                    )

                    if result == MotionResult.SUCCESS:
                        self.successful_picks += 1
                        print("  PICK-AND-PLACE SUCCESSFUL!")
                    else:
                        self.failed_picks += 1
                        print(f"  PICK-AND-PLACE FAILED: {result.value}")

                    # Reset for next target
                    stable_frames = 0
                    last_target = None

                    # Small delay before looking for next target
                    time.sleep(1.0)

            else:
                # No target found
                stable_frames = 0
                last_target = None

            frame_count += 1

    def run_manual(self):
        """
        Run in manual mode with keyboard control.

        Keyboard controls:
        - Arrow keys: Move XY
        - +/-: Move Z
        - o/l: Open/close gripper
        - h: Home
        - t: Test pick at current position
        - Space: Pick-and-place at detected target
        - q: Quit
        """
        print("\n" + "=" * 60)
        print("MANUAL MODE")
        print("  Arrow keys: Move X/Y")
        print("  +/-: Move Z")
        print("  o: Open gripper")
        print("  l: Close gripper")
        print("  h: Home position")
        print("  t: Test pick at current detection")
        print("  Space: Execute pick-and-place")
        print("  q: Quit")
        print("=" * 60 + "\n")

        self.running = True
        current_x, current_y, current_z = 100.0, 0.0, MOTION.z_safe
        step_xy = 10.0  # mm
        step_z = 5.0    # mm

        while self.running:
            # Process frame
            result = self.process_frame()

            # Display
            if self.board_state and self.board_state.display_frame is not None:
                self._draw_overlay(self.board_state, result)

                # Show current position
                pos_text = f"Robot: ({current_x:.0f}, {current_y:.0f}, {current_z:.0f})"
                cv2.putText(
                    self.board_state.display_frame,
                    pos_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
                )

                cv2.imshow("Vision-Robot", self.board_state.display_frame)

            # Handle keyboard
            key = cv2.waitKey(30) & 0xFF

            if key == ord('q'):
                self.running = False
            elif key == ord('h'):
                print("Homing...")
                self.robot.home()
                current_x, current_y, current_z = 100.0, 0.0, MOTION.z_safe
            elif key == ord('o'):
                self.robot.open_gripper()
            elif key == ord('l'):
                self.robot.close_gripper()
            elif key == ord('+') or key == ord('='):
                current_z += step_z
                self.robot.move_to(current_x, current_y, current_z)
            elif key == ord('-'):
                current_z -= step_z
                self.robot.move_to(current_x, current_y, current_z)
            elif key == 82:  # Up arrow
                current_x += step_xy
                self.robot.move_to(current_x, current_y, current_z)
            elif key == 84:  # Down arrow
                current_x -= step_xy
                self.robot.move_to(current_x, current_y, current_z)
            elif key == 81:  # Left arrow
                current_y += step_xy
                self.robot.move_to(current_x, current_y, current_z)
            elif key == 83:  # Right arrow
                current_y -= step_xy
                self.robot.move_to(current_x, current_y, current_z)
            elif key == ord('t'):
                # Test pick at detected position
                if result:
                    target, _ = result
                    print(f"Testing pick at {target.color}...")
                    pick_inner = (target.cx_inner, target.cy_inner)
                    pick_robot, _ = self.convert_to_robot_coords(
                        pick_inner, pick_inner
                    )
                    self.coordinator.motion.test_pick_position(
                        pick_robot[0], pick_robot[1]
                    )
            elif key == ord(' '):
                # Execute pick-and-place
                if result:
                    target, ghost_pt = result
                    print(f"Picking {target.color}...")
                    pick_inner = (target.cx_inner, target.cy_inner)
                    self.total_picks += 1
                    motion_result = self.coordinator.execute_pick_place_for_detection(
                        pick_board_mm=self.converter.inner_px_to_board_mm(*pick_inner),
                        place_board_mm=self.converter.inner_px_to_board_mm(*ghost_pt),
                        object_color=target.color
                    )
                    if motion_result == MotionResult.SUCCESS:
                        self.successful_picks += 1
                    else:
                        self.failed_picks += 1

    def run_vision_only(self):
        """
        Run vision-only mode (no robot commands).

        Displays detection overlay and prints coordinates.
        """
        print("\n" + "=" * 60)
        print("VISION-ONLY MODE")
        print("  Press 'q' to quit")
        print("  Press 's' to save frame")
        print("=" * 60 + "\n")

        self.running = True
        frame_count = 0

        while self.running:
            # Process frame
            result = self.process_frame()

            # Display
            if self.board_state and self.board_state.display_frame is not None:
                self._draw_overlay(self.board_state, result)
                cv2.imshow("Vision", self.board_state.display_frame)

                # Also show warped view if available
                if self.board_state.warped_frame is not None:
                    cv2.imshow("Warped", self.board_state.warped_frame)

            # Print detection info periodically
            if frame_count % DEBUG.print_interval == 0 and result is not None:
                target, ghost_pt = result
                pick_robot, place_robot = self.convert_to_robot_coords(
                    (target.cx_inner, target.cy_inner), ghost_pt
                )
                print(f"[Frame {frame_count}] {target.color}: "
                      f"pick=({pick_robot[0]:.1f}, {pick_robot[1]:.1f}) "
                      f"place=({place_robot[0]:.1f}, {place_robot[1]:.1f})")

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.running = False
            elif key == ord('s'):
                filename = f"frame_{frame_count:06d}.png"
                cv2.imwrite(filename, self.board_state.display_frame)
                print(f"Saved {filename}")

            frame_count += 1

    def _draw_overlay(self, board_state: BoardState,
                      result: Optional[Tuple[Detection, Tuple[float, float]]]):
        """Draw status overlay on display frame."""
        if board_state.display_frame is None:
            return

        frame = board_state.display_frame
        h, w = frame.shape[:2]

        # Status bar at bottom
        status_h = 40
        cv2.rectangle(frame, (0, h - status_h), (w, h), (0, 0, 0), -1)

        # Status text
        status_parts = []

        if board_state.board_detected:
            status_parts.append("Board: OK")
        else:
            status_parts.append("Board: NOT FOUND")

        if board_state.divider_x_inner is not None:
            status_parts.append(f"Divider: {board_state.divider_x_inner:.0f}px")
        else:
            status_parts.append("Divider: --")

        status_parts.append(f"Objects: {len(board_state.detections)}")

        if result:
            target, ghost_pt = result
            status_parts.append(f"Target: {target.color}")

        status_text = " | ".join(status_parts)
        cv2.putText(
            frame, status_text, (10, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )

        # Draw ghost point if target selected
        if result and board_state.H_warp2img is not None:
            target, ghost_pt = result

            # Convert ghost point from inner pixels to camera pixels for display
            wx = ghost_pt[0] + WARP.margin_px
            wy = ghost_pt[1] + WARP.margin_px

            pt = np.array([[[wx, wy]]], dtype=np.float32)
            out = cv2.perspectiveTransform(pt, board_state.H_warp2img)
            gx, gy = int(out[0, 0, 0]), int(out[0, 0, 1])

            # Draw ghost marker
            cv2.drawMarker(
                frame, (gx, gy),
                (0, 255, 255), cv2.MARKER_STAR, 20, 2
            )
            cv2.putText(
                frame, "GHOST", (gx + 10, gy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
            )


# =============================================================================
# ENTRY POINT
# =============================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Vision-Robot Integration for Pick and Place"
    )

    parser.add_argument(
        "--manual", action="store_true",
        help="Run in manual keyboard control mode"
    )
    parser.add_argument(
        "--vision-only", action="store_true",
        help="Run vision only (no robot commands)"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print commands without sending to robot"
    )
    parser.add_argument(
        "--allow-mock", action="store_true",
        help="Fall back to mock robot if connection fails"
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without display windows"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Print verbose detection info"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    app = VisionRobotApp(args)

    if not app.initialize():
        print("\nFailed to initialize. Exiting.")
        sys.exit(1)

    try:
        if args.vision_only:
            app.run_vision_only()
        elif args.manual:
            app.run_manual()
        else:
            app.run_autonomous()

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        app.shutdown()


if __name__ == "__main__":
    main()
