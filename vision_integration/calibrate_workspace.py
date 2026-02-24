#!/usr/bin/env python3
"""
Workspace Calibration Helper

This script helps calibrate the board-to-robot coordinate transform.
It moves the robot to specific positions so you can measure the correspondence.
"""

import serial
import time
import sys

PORT = '/dev/cu.usbmodem11301'
BAUD = 115200

print("=" * 60)
print("WORKSPACE CALIBRATION")
print("=" * 60)

# Connect to Arduino
print("\nConnecting to Arduino...")
ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2)
ser.reset_input_buffer()

def send_cmd(cmd):
    """Send command and wait for response"""
    ser.write((cmd + '\n').encode())
    time.sleep(0.5)
    response = ""
    while ser.in_waiting:
        response += ser.readline().decode('utf-8', errors='ignore')
    return response.strip()

def get_position():
    """Get current position"""
    resp = send_cmd("P")
    # Parse: Position: X=208.6 Y=0.0 Z=136.8 Pitch=0.0
    for line in resp.split('\n'):
        if 'Position:' in line:
            parts = line.split()
            pos = {}
            for p in parts:
                if '=' in p:
                    k, v = p.split('=')
                    pos[k] = float(v)
            return pos
    return None

def move_to(x, y, z, pitch=0):
    """Move to position"""
    cmd = f"G {x:.1f} {y:.1f} {z:.1f} {pitch:.1f} 0"
    print(f"  Moving to ({x}, {y}, {z})...")
    resp = send_cmd(cmd)
    time.sleep(1.5)  # Wait for move
    return "OK" in resp or "Move" in resp

# Home first
print("\n1. Homing robot...")
send_cmd("H")
time.sleep(2)

pos = get_position()
print(f"   Current position: X={pos['X']:.1f}, Y={pos['Y']:.1f}, Z={pos['Z']:.1f}")

print("""
CALIBRATION PROCEDURE:
======================
We'll move the robot to several positions. For each position:
1. Note where the gripper tip is relative to the board
2. Record the (board_x_mm, board_y_mm) position

Board coordinates:
  - Origin (0,0) = top-left corner of INNER white area
  - X increases going RIGHT
  - Y increases going DOWN

Press Enter after each move to continue, or 'q' to quit.
""")

# Test positions - adjust these based on your setup
# These are robot coordinates (X forward, Y left)
test_positions = [
    (120, 80, 50, "Front-left area"),
    (120, -80, 50, "Front-right area"),
    (180, 0, 50, "Center-forward"),
    (150, 50, 30, "Pick height test - left"),
    (150, -50, 30, "Pick height test - right"),
]

calibration_data = []

for i, (rx, ry, rz, desc) in enumerate(test_positions):
    print(f"\n--- Position {i+1}/{len(test_positions)}: {desc} ---")

    if move_to(rx, ry, rz, pitch=-45):
        pos = get_position()
        if pos:
            print(f"   Robot position: X={pos['X']:.1f}, Y={pos['Y']:.1f}, Z={pos['Z']:.1f}")

        response = input("\n   Where is gripper on board? Enter 'bx,by' in mm (or 'skip' or 'q'): ").strip()

        if response.lower() == 'q':
            break
        elif response.lower() != 'skip' and ',' in response:
            try:
                bx, by = map(float, response.split(','))
                calibration_data.append({
                    'robot': (rx, ry),
                    'board': (bx, by)
                })
                print(f"   Recorded: board ({bx}, {by}) -> robot ({rx}, {ry})")
            except:
                print("   Invalid input, skipping")
    else:
        print("   Move failed!")

# Return home
print("\nReturning home...")
send_cmd("H")
time.sleep(2)

ser.close()

# Calculate transform if we have data
if len(calibration_data) >= 2:
    print("\n" + "=" * 60)
    print("CALIBRATION RESULTS")
    print("=" * 60)

    print("\nRecorded points:")
    for d in calibration_data:
        print(f"  Board ({d['board'][0]:.1f}, {d['board'][1]:.1f}) -> Robot ({d['robot'][0]:.1f}, {d['robot'][1]:.1f})")

    # Simple estimation: assume no rotation, compute origin offset
    # robot_x = board_origin_x + board_x
    # robot_y = board_origin_y - board_y (Y is flipped)

    # From first point: origin_x = robot_x - board_x
    #                   origin_y = robot_y + board_y

    offsets_x = []
    offsets_y = []
    for d in calibration_data:
        bx, by = d['board']
        rx, ry = d['robot']
        offsets_x.append(rx - bx)
        offsets_y.append(ry + by)

    origin_x = sum(offsets_x) / len(offsets_x)
    origin_y = sum(offsets_y) / len(offsets_y)

    print(f"\nEstimated workspace transform:")
    print(f"  board_origin_x = {origin_x:.1f}")
    print(f"  board_origin_y = {origin_y:.1f}")

    print(f"\nUpdate config.py with these values:")
    print(f"""
@dataclass
class WorkspaceTransform:
    board_origin_x: float = {origin_x:.1f}
    board_origin_y: float = {origin_y:.1f}
    board_origin_z: float = 0.0
    rotation_deg: float = 0.0
""")
else:
    print("\nNot enough calibration points recorded.")
    print("Run again and enter board coordinates for at least 2 positions.")
