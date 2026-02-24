#!/usr/bin/env python3
"""
Hardware Test Script

Tests camera and Arduino connection.
"""

import sys
import time

print("=" * 60)
print("HARDWARE TEST")
print("=" * 60)

# =============================================================================
# TEST 1: Camera
# =============================================================================
print("\n[1/2] TESTING CAMERA...")

try:
    import cv2

    # Try different camera indices
    camera = None
    for idx in [0, 1, 2]:
        print(f"  Trying camera index {idx}...", end=" ")
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                print(f"OK! Resolution: {w}x{h}")
                camera = cap
                camera_idx = idx
                break
            else:
                print("opened but no frame")
                cap.release()
        else:
            print("not found")

    if camera is None:
        print("  ERROR: No working camera found!")
        camera_ok = False
    else:
        camera_ok = True
        # Show a test frame
        print(f"  Camera {camera_idx} working. Capturing test frame...")
        ret, frame = camera.read()
        if ret:
            cv2.imwrite("/tmp/camera_test.jpg", frame)
            print(f"  Saved test frame to /tmp/camera_test.jpg")
        camera.release()

except Exception as e:
    print(f"  ERROR: {e}")
    camera_ok = False

# =============================================================================
# TEST 2: Arduino
# =============================================================================
print("\n[2/2] TESTING ARDUINO...")

try:
    import serial
    import glob

    # Find Arduino port
    ports = glob.glob('/dev/cu.usbmodem*') + glob.glob('/dev/cu.usbserial*')

    if not ports:
        print("  ERROR: No Arduino found!")
        arduino_ok = False
    else:
        port = ports[0]
        print(f"  Found Arduino at {port}")
        print(f"  Connecting at 115200 baud...")

        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(2)  # Wait for Arduino reset

        # Clear buffer
        ser.reset_input_buffer()

        # Send help command
        print("  Sending '?' command...")
        ser.write(b"?\n")
        time.sleep(0.5)

        # Read response
        response = ""
        while ser.in_waiting:
            response += ser.readline().decode('utf-8', errors='ignore')

        if "Commands:" in response:
            print("  Arduino responded! Commands available:")
            for line in response.strip().split('\n'):
                print(f"    {line}")
            arduino_ok = True
        else:
            print(f"  Unexpected response: {response[:100]}")
            arduino_ok = False

        # Try to get current position
        print("\n  Querying position (P command)...")
        ser.write(b"P\n")
        time.sleep(0.5)

        response = ""
        while ser.in_waiting:
            response += ser.readline().decode('utf-8', errors='ignore')

        if response:
            print(f"  Position response: {response.strip()}")

        # Try to get joints
        print("\n  Querying joints (C command)...")
        ser.write(b"C\n")
        time.sleep(0.5)

        response = ""
        while ser.in_waiting:
            response += ser.readline().decode('utf-8', errors='ignore')

        if response:
            print(f"  Joints response: {response.strip()}")

        ser.close()
        print("\n  Arduino connection closed.")

except Exception as e:
    print(f"  ERROR: {e}")
    arduino_ok = False

# =============================================================================
# SUMMARY
# =============================================================================
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)
print(f"  Camera:  {'OK' if camera_ok else 'FAILED'}")
print(f"  Arduino: {'OK' if arduino_ok else 'FAILED'}")

if camera_ok and arduino_ok:
    print("\n  All hardware ready! You can run:")
    print("    python3 main.py --vision-only  # Test vision")
    print("    python3 main.py --manual       # Manual control")
    print("    python3 main.py                # Autonomous mode")
else:
    print("\n  Fix hardware issues before running main.py")

print("=" * 60)
