"""
Serial Communication with Arduino Robot Arm

Provides a clean interface to send commands to the Arduino
running the RobotArmIK sketch.

Commands supported:
  G X Y Z [pitch] [roll]  - Move to position
  J b s e wp wr g         - Set joints directly
  H                       - Home position
  K                       - Park position
  P                       - Get current position
  C                       - Get current joints
  W                       - Get workspace bounds
  O                       - Open gripper
  L                       - Close gripper
"""

import serial
import time
import glob
from typing import Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

from config import SERIAL, DEBUG


# =============================================================================
# RESPONSE TYPES
# =============================================================================

class CommandStatus(Enum):
    """Status of a command execution"""
    OK = "OK"
    WARN = "WARN"
    ERR = "ERR"
    TIMEOUT = "TIMEOUT"
    NOT_CONNECTED = "NOT_CONNECTED"


@dataclass
class CommandResponse:
    """Response from a command"""
    status: CommandStatus
    message: str
    data: Optional[dict] = None


@dataclass
class Position:
    """Current position from FK"""
    x: float
    y: float
    z: float
    pitch: float


@dataclass
class JointAngles:
    """Current joint angles"""
    base: float
    shoulder: float
    elbow: float
    wrist_pitch: float
    wrist_roll: float
    gripper: float


# =============================================================================
# ROBOT SERIAL INTERFACE
# =============================================================================

class RobotSerial:
    """
    Serial interface to the Arduino robot arm.

    Usage:
        robot = RobotSerial()
        robot.connect()

        # Move to position
        robot.move_to(100, 0, 80, pitch=-45)

        # Direct joint control
        robot.set_joints(0, 45, -45, 0, 0, 0)

        # Gripper
        robot.open_gripper()
        robot.close_gripper()

        robot.disconnect()
    """

    def __init__(self, port: Optional[str] = None, baudrate: int = SERIAL.baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.connected = False

    def connect(self, port: Optional[str] = None) -> bool:
        """
        Connect to the Arduino.

        Args:
            port: Serial port (auto-detect if None)

        Returns:
            True if connected successfully
        """
        if port is not None:
            self.port = port

        # Try specified port or auto-detect
        ports_to_try = [self.port] if self.port else []
        ports_to_try.extend(SERIAL.alt_ports)

        for port_pattern in ports_to_try:
            if port_pattern is None:
                continue

            # Handle glob patterns
            if '*' in port_pattern:
                matching = glob.glob(port_pattern)
                actual_ports = matching
            else:
                actual_ports = [port_pattern]

            for actual_port in actual_ports:
                try:
                    self.ser = serial.Serial(
                        actual_port,
                        self.baudrate,
                        timeout=SERIAL.timeout
                    )
                    time.sleep(2)  # Wait for Arduino reset

                    # Clear any startup messages
                    self.ser.reset_input_buffer()

                    # Test communication
                    self.ser.write(b"?\n")
                    time.sleep(0.5)
                    response = self._read_all()

                    if response and "Commands:" in response:
                        self.connected = True
                        self.port = actual_port
                        print(f"Connected to robot on {actual_port}")
                        return True

                    self.ser.close()
                except (serial.SerialException, OSError) as e:
                    if self.ser:
                        try:
                            self.ser.close()
                        except:
                            pass
                    continue

        print("Could not connect to robot on any port")
        return False

    def disconnect(self):
        """Disconnect from Arduino"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        print("Disconnected from robot")

    def _read_all(self, timeout: float = 1.0) -> str:
        """Read all available data from serial"""
        if not self.ser:
            return ""

        result = []
        start = time.time()

        while (time.time() - start) < timeout:
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        result.append(line)
                except:
                    pass
            else:
                time.sleep(0.05)

        return '\n'.join(result)

    def _send_command(self, cmd: str, wait_for_ok: bool = True) -> CommandResponse:
        """
        Send a command and wait for response.

        Args:
            cmd: Command string
            wait_for_ok: If True, wait for OK/WARN/ERR response

        Returns:
            CommandResponse with status and message
        """
        if DEBUG.dry_run:
            print(f"[DRY RUN] Would send: {cmd}")
            return CommandResponse(CommandStatus.OK, "Dry run - not sent")

        if not self.connected or not self.ser:
            return CommandResponse(CommandStatus.NOT_CONNECTED, "Not connected")

        try:
            # Send command
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()

            if not wait_for_ok:
                return CommandResponse(CommandStatus.OK, "Sent")

            # Wait for response
            response_lines = []
            start = time.time()

            while (time.time() - start) < SERIAL.timeout:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response_lines.append(line)

                        # Check for status prefix
                        if line.startswith("OK"):
                            return CommandResponse(
                                CommandStatus.OK,
                                '\n'.join(response_lines)
                            )
                        elif line.startswith("WARN"):
                            return CommandResponse(
                                CommandStatus.WARN,
                                '\n'.join(response_lines)
                            )
                        elif line.startswith("ERR"):
                            return CommandResponse(
                                CommandStatus.ERR,
                                '\n'.join(response_lines)
                            )
                else:
                    time.sleep(0.01)

            return CommandResponse(
                CommandStatus.TIMEOUT,
                '\n'.join(response_lines) if response_lines else "No response"
            )

        except Exception as e:
            return CommandResponse(CommandStatus.ERR, str(e))

    # -------------------------------------------------------------------------
    # MOTION COMMANDS
    # -------------------------------------------------------------------------

    def move_to(self, x: float, y: float, z: float,
                pitch: float = 0, roll: float = 0) -> CommandResponse:
        """
        Move to XYZ position with optional pitch and roll.

        Args:
            x, y, z: Position in mm
            pitch: Tool pitch in degrees (0 = horizontal)
            roll: Tool roll in degrees

        Returns:
            CommandResponse
        """
        cmd = f"G {x:.1f} {y:.1f} {z:.1f} {pitch:.1f} {roll:.1f}"
        return self._send_command(cmd)

    def set_joints(self, base: float, shoulder: float, elbow: float,
                   wrist_pitch: float, wrist_roll: float, gripper: float
                   ) -> CommandResponse:
        """
        Set joint angles directly.

        Args:
            base, shoulder, elbow, wrist_pitch, wrist_roll, gripper: Angles in degrees

        Returns:
            CommandResponse
        """
        cmd = f"J {base:.1f} {shoulder:.1f} {elbow:.1f} {wrist_pitch:.1f} {wrist_roll:.1f} {gripper:.1f}"
        return self._send_command(cmd)

    def home(self) -> CommandResponse:
        """Move to home position"""
        return self._send_command("H")

    def park(self) -> CommandResponse:
        """Move to park position (folded)"""
        return self._send_command("K")

    # -------------------------------------------------------------------------
    # GRIPPER COMMANDS
    # -------------------------------------------------------------------------

    def open_gripper(self) -> CommandResponse:
        """Open the gripper"""
        return self._send_command("O")

    def close_gripper(self) -> CommandResponse:
        """Close the gripper"""
        return self._send_command("L")

    def set_gripper(self, angle: float) -> CommandResponse:
        """
        Set gripper to specific angle.

        Args:
            angle: Gripper angle (0 = closed, 90 = open)
        """
        # Get current joints, modify gripper, set joints
        joints = self.get_joints()
        if joints is None:
            return CommandResponse(CommandStatus.ERR, "Could not get current joints")

        return self.set_joints(
            joints.base, joints.shoulder, joints.elbow,
            joints.wrist_pitch, joints.wrist_roll, angle
        )

    # -------------------------------------------------------------------------
    # QUERY COMMANDS
    # -------------------------------------------------------------------------

    def get_position(self) -> Optional[Position]:
        """
        Get current position from forward kinematics.

        Returns:
            Position object or None if failed
        """
        response = self._send_command("P", wait_for_ok=False)

        # Parse response like: "Position: X=100.5 Y=0.0 Z=82.3 Pitch=15.0"
        for line in response.message.split('\n'):
            if "Position:" in line:
                try:
                    parts = line.split()
                    values = {}
                    for part in parts:
                        if '=' in part:
                            key, val = part.split('=')
                            values[key] = float(val)

                    return Position(
                        x=values.get('X', 0),
                        y=values.get('Y', 0),
                        z=values.get('Z', 0),
                        pitch=values.get('Pitch', 0)
                    )
                except:
                    pass

        return None

    def get_joints(self) -> Optional[JointAngles]:
        """
        Get current joint angles.

        Returns:
            JointAngles object or None if failed
        """
        response = self._send_command("C", wait_for_ok=False)

        # Parse response like: "Joints: Base=0.0 Shoulder=45.0 ..."
        for line in response.message.split('\n'):
            if "Joints:" in line:
                try:
                    parts = line.split()
                    values = {}
                    for part in parts:
                        if '=' in part:
                            key, val = part.split('=')
                            values[key] = float(val)

                    return JointAngles(
                        base=values.get('Base', 0),
                        shoulder=values.get('Shoulder', 0),
                        elbow=values.get('Elbow', 0),
                        wrist_pitch=values.get('WristPitch', 0),
                        wrist_roll=values.get('WristRoll', 0),
                        gripper=values.get('Gripper', 0)
                    )
                except:
                    pass

        return None

    def get_workspace(self) -> Optional[dict]:
        """
        Get workspace bounds.

        Returns:
            Dict with MaxRadius, MinRadius, MaxZ, MinZ or None
        """
        response = self._send_command("W", wait_for_ok=False)

        for line in response.message.split('\n'):
            if "Workspace:" in line:
                try:
                    parts = line.split()
                    values = {}
                    for part in parts:
                        if '=' in part:
                            key, val = part.split('=')
                            values[key] = float(val)
                    return values
                except:
                    pass

        return None

    # -------------------------------------------------------------------------
    # UTILITY METHODS
    # -------------------------------------------------------------------------

    def wait_for_move(self, timeout: float = 10.0) -> bool:
        """
        Wait for current move to complete.

        Returns:
            True if move completed, False if timeout
        """
        start = time.time()

        while (time.time() - start) < timeout:
            if self.ser and self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "Move complete" in line:
                    return True
            time.sleep(0.05)

        return False

    def send_raw(self, cmd: str) -> str:
        """
        Send raw command and return raw response.

        Useful for debugging and custom commands.
        """
        if DEBUG.dry_run:
            return f"[DRY RUN] {cmd}"

        if not self.connected or not self.ser:
            return "Not connected"

        self.ser.write((cmd + '\n').encode())
        time.sleep(0.5)
        return self._read_all()


# =============================================================================
# MOCK ROBOT FOR TESTING
# =============================================================================

class MockRobotSerial(RobotSerial):
    """
    Mock robot for testing without hardware.

    Simulates responses and tracks commanded positions.
    """

    def __init__(self):
        super().__init__()
        self.connected = True
        self.current_position = Position(x=100, y=0, z=80, pitch=0)
        self.current_joints = JointAngles(
            base=0, shoulder=45, elbow=-45,
            wrist_pitch=0, wrist_roll=0, gripper=0
        )
        self.command_history: List[str] = []

    def connect(self, port: Optional[str] = None) -> bool:
        print("[MOCK] Connected to simulated robot")
        return True

    def disconnect(self):
        print("[MOCK] Disconnected from simulated robot")

    def _send_command(self, cmd: str, wait_for_ok: bool = True) -> CommandResponse:
        self.command_history.append(cmd)
        print(f"[MOCK] Command: {cmd}")

        # Simulate response based on command
        if cmd.startswith("G"):
            parts = cmd.split()
            if len(parts) >= 4:
                self.current_position = Position(
                    x=float(parts[1]),
                    y=float(parts[2]),
                    z=float(parts[3]),
                    pitch=float(parts[4]) if len(parts) > 4 else 0
                )
            return CommandResponse(CommandStatus.OK, "OK Moving to position")

        elif cmd.startswith("J"):
            parts = cmd.split()
            if len(parts) >= 7:
                self.current_joints = JointAngles(
                    base=float(parts[1]),
                    shoulder=float(parts[2]),
                    elbow=float(parts[3]),
                    wrist_pitch=float(parts[4]),
                    wrist_roll=float(parts[5]),
                    gripper=float(parts[6])
                )
            return CommandResponse(CommandStatus.OK, "OK Moving to joint position")

        elif cmd == "H":
            self.current_joints = JointAngles(0, 0, 0, 0, 0, 0)
            return CommandResponse(CommandStatus.OK, "OK Moving to home")

        elif cmd == "O":
            self.current_joints.gripper = 90
            return CommandResponse(CommandStatus.OK, "OK Opening gripper")

        elif cmd == "L":
            self.current_joints.gripper = 0
            return CommandResponse(CommandStatus.OK, "OK Closing gripper")

        return CommandResponse(CommandStatus.OK, f"OK {cmd}")

    def get_position(self) -> Optional[Position]:
        return self.current_position

    def get_joints(self) -> Optional[JointAngles]:
        return self.current_joints

    def wait_for_move(self, timeout: float = 10.0) -> bool:
        time.sleep(0.1)  # Simulate short delay
        return True
