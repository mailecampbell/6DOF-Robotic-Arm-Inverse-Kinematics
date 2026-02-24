# Inverse Kinematics for 3D Printed Arduino Robotic Arm

A complete inverse kinematics (IK) solution for the **3D Printed Arduino Based Robotic Arm** (BasementMaker Instructables build) and similar 6-DOF servo arms.

## Features

- **Geometric/Analytical IK**: Fast, deterministic solution using closed-form equations
- **Forward Kinematics**: Compute end-effector pose from joint angles
- **Dual Solutions**: Returns both elbow-up and elbow-down configurations
- **Workspace Checking**: Reachability validation with closest-point fallback
- **Servo Integration**: Direct support for Arduino Servo library with calibration
- **Smooth Motion**: Cubic interpolation for smooth, jerk-free movements
- **Comprehensive Testing**: Desktop test harness for verification

## Coordinate System

```
        Z (up)
        |
        |
        +------ X (forward)
       /
      /
     Y (left)

   Origin: Center of base rotation axis, at table level
```

### Joint Definitions

| Joint | Name | Axis | Zero Position | Positive Direction |
|-------|------|------|--------------|-------------------|
| J0 | Base | Z (yaw) | Arm points forward (+X) | CCW when viewed from above |
| J1 | Shoulder | Y (pitch) | Upper arm horizontal | Raises arm up |
| J2 | Elbow | Y (pitch) | Forearm aligned with upper arm | Bends inward/up |
| J3 | Wrist Pitch | Y (pitch) | Gripper aligned with forearm | Tilts gripper up |
| J4 | Wrist Roll | X (roll) | Gripper vertical | Rotates gripper |
| J5 | Gripper | - | Closed | Opens |

## Hardware Setup

### Default Pin Mapping

| Joint | Arduino Pin |
|-------|-------------|
| Base | D3 |
| Shoulder | D5 |
| Elbow | D6 |
| Wrist Pitch | D11 |
| Wrist Roll | D10 |
| Gripper | D9 |

### Wiring Diagram

```
Arduino Uno/Nano
    +5V ────────┬───┬───┬───┬───┬───┐
    GND ────────┼───┼───┼───┼───┼───┤
                │   │   │   │   │   │
             ┌──┴┐┌─┴─┐┌┴─┐┌┴─┐┌┴─┐┌┴─┐
             │S0││S1 ││S2││S3││S4││S5│
             └──┘└───┘└──┘└──┘└──┘└──┘
              ↑   ↑    ↑   ↑   ↑   ↑
             D3  D5   D6  D11 D10  D9

Note: Use external 5V supply for servos (4-6A recommended)
      Do NOT power servos from Arduino 5V pin!
```

## Measuring Link Lengths

**Critical**: Accurate link lengths are essential for correct IK. Measure YOUR arm!

### Method 1: From Physical Arm

1. **Base Height (baseHeight)**: Measure from table surface to center of shoulder joint axis
   - Typical: 65-80mm

2. **Upper Arm (upperArmLength)**: Measure from shoulder joint axis to elbow joint axis
   - Typical: 100-110mm

3. **Forearm (forearmLength)**: Measure from elbow joint axis to wrist pitch joint axis
   - Typical: 95-105mm

4. **Wrist (wristLength)**: Measure from wrist pitch axis to wrist roll axis
   - Typical: 20-30mm

5. **Tool (toolLength)**: Measure from wrist roll axis to gripper fingertips
   - Typical: 60-80mm

### Method 2: From STL Files

1. Open STL files in a slicer (Cura, PrusaSlicer) or CAD viewer
2. Use measurement tools to find distances between joint centers
3. Add any assembly gaps/spacing

### Where to Enter Measurements

**Arduino Sketch** (`RobotArmIK.ino`):
```cpp
const float BASE_HEIGHT = 70.0;        // Your measurement
const float UPPER_ARM_LENGTH = 105.0;  // Your measurement
const float FOREARM_LENGTH = 98.0;     // Your measurement
const float WRIST_LENGTH = 25.0;       // Your measurement
const float TOOL_LENGTH = 70.0;        // Your measurement
```

**C++ Library** (`include/arm_config.h`):
```cpp
constexpr ArmGeometry MY_GEOMETRY = {
    .baseHeight = 70.0f,
    .shoulderOffsetX = 0.0f,
    .upperArmLength = 105.0f,
    .forearmLength = 98.0f,
    .wristLength = 25.0f,
    .toolLength = 70.0f
};
```

## Servo Calibration

### Finding Calibration Offsets

The goal is to find the servo angle that corresponds to joint angle = 0°.

1. **Upload calibration sketch** or use manual positioning
2. **For each joint**:
   - Send servo to 90° (middle position)
   - Observe physical joint position
   - If joint is at 0° when servo is at 90°, offset = 90
   - If joint is at 15° when servo is at 90°, offset = 75 (90 - 15)

### Calibration Values

In `RobotArmIK.ino`:
```cpp
float offsetBase = 90;        // Adjust so joint 0° = servo 90°
float offsetShoulder = 90;
float offsetElbow = 90;
float offsetWristPitch = 90;
float offsetWristRoll = 90;
float offsetGripper = 0;

// Set to true if servo moves opposite to expected
const bool invertBase = false;
const bool invertShoulder = false;
const bool invertElbow = true;   // Often true due to mounting
const bool invertWristPitch = false;
const bool invertWristRoll = false;
const bool invertGripper = false;
```

### Calibration Procedure

1. Power on arm (servos at center position)
2. Open Serial Monitor (115200 baud)
3. Type `J 0 0 0 0 0 0` to command all joints to zero
4. Observe each joint:
   - If joint isn't at expected position, adjust offset
   - If joint moves wrong direction, set `invert = true`
5. Repeat until all joints behave correctly

## Usage

### Arduino Serial Commands

Connect at 115200 baud:

```
G X Y Z [pitch] [roll]  - Move to position (mm), optional pitch/roll (deg)
J b s e wp wr g         - Set joint angles directly (degrees)
H                       - Go to home position
K                       - Park (fold arm)
P                       - Print current position (FK result)
C                       - Print current joint angles
W                       - Print workspace bounds
O                       - Open gripper
L                       - Close gripper
?                       - Help
```

### Examples

```
G 150 0 100             Move to X=150mm, Y=0mm, Z=100mm (horizontal tool)
G 150 0 100 -45         Same but with tool pitched 45° down
G 100 100 80 0 90       Move with 90° wrist roll
J 0 45 -45 0 0 0        Set joints directly
H                       Home position
```

### C++ Library Usage

```cpp
#include "arm_kinematics.h"

// Define your arm geometry
ArmGeometry geometry = {
    .baseHeight = 70.0f,
    .shoulderOffsetX = 0.0f,
    .upperArmLength = 105.0f,
    .forearmLength = 98.0f,
    .wristLength = 25.0f,
    .toolLength = 70.0f
};

// Forward Kinematics
JointAngles joints = {0, 45, -45, 0, 0, 0};
Pose pose = forwardKinematics(joints, geometry);

// Inverse Kinematics
Pose targetPose;
targetPose.position = Vec3(150, 0, 100);
targetPose.orientation = Mat3::rotY(-30);  // 30° pitch down

IKOptions options = DEFAULT_IK_OPTIONS;
options.elbowConfig = ELBOW_UP;

JointSolution solution = inverseKinematics(
    targetPose, geometry, DEFAULT_SERVO_CONFIG, options);

if (solution.valid) {
    // Use solution.angles
}
```

## Building and Testing

### Desktop Test Harness

```bash
cd arduino_arm_ik
make test
```

This compiles and runs 500 random FK→IK→FK round-trip tests.

### Arduino Sketch

1. Open `arduino/RobotArmIK/RobotArmIK.ino` in Arduino IDE
2. Adjust link lengths and calibration values
3. Upload to Arduino
4. Open Serial Monitor at 115200 baud

## Geometric IK Algorithm

The inverse kinematics uses a closed-form geometric solution:

### Step 1: Wrist Center Calculation
```
wrist_center = target_position - tool_length * tool_z_axis
```

### Step 2: Base Angle
```
base = atan2(wrist_y, wrist_x)
```

### Step 3: Planar 2-Link IK
Transform to shoulder frame and solve in the vertical plane:
```
r = horizontal_distance_to_wrist - shoulder_offset
h = wrist_z - base_height
d = sqrt(r² + h²)

# Law of cosines for elbow
cos(elbow) = (d² - L1² - L2²) / (2·L1·L2)
elbow = ±acos(cos_elbow)  # ± for elbow up/down

# Shoulder angle
α = atan2(h, r)
β = acos((L1² + d² - L2²) / (2·L1·d))
shoulder = α + β  (elbow up) or α - β (elbow down)
```

### Step 4: Wrist Angles
```
wrist_pitch = desired_pitch - shoulder - elbow
wrist_roll = extracted from orientation about tool axis
```

## Status Codes

| Code | Meaning |
|------|---------|
| IK_OK | Solution found, within all limits |
| IK_UNREACHABLE | Target outside workspace |
| IK_SINGULAR | Singular configuration (e.g., above base) |
| IK_LIMIT_CLAMPED | Solution found but clamped to limits |
| IK_NO_SOLUTION | No valid solution exists |
| IK_ORIENTATION_UNREACHABLE | Position OK but orientation impossible |

## Project Structure

```
arduino_arm_ik/
├── include/
│   ├── arm_config.h      # Configuration, geometry, joint limits
│   ├── arm_math.h        # Math utilities, Vec3, Mat3, angle functions
│   ├── arm_kinematics.h  # FK/IK function declarations
│   └── servo_controller.h # Servo control class
├── src/
│   ├── arm_kinematics.cpp    # FK/IK implementation
│   └── servo_controller.cpp  # Servo control implementation
├── arduino/
│   └── RobotArmIK/
│       └── RobotArmIK.ino    # Self-contained Arduino sketch
├── test/
│   └── test_kinematics.cpp   # Desktop test harness
├── Makefile
└── README.md
```

## Troubleshooting

### Arm moves to wrong position
- Verify link length measurements
- Check calibration offsets
- Verify invert flags for each servo

### Servo jitters or doesn't hold position
- Use external 5V power supply (not Arduino 5V)
- Check for loose connections
- Reduce max servo speed if needed

### IK returns unreachable for valid positions
- Check that target is within workspace (`W` command)
- Verify orientation is achievable (some pitch angles impossible)
- Check joint limits match physical limits

### Jerky movement
- Increase move duration in `startMove()` calls
- Verify loop runs fast enough for smooth interpolation

## License

MIT License - Use freely for personal and commercial projects.

## Acknowledgments

- BasementMaker's "3D Printed Arduino Based Robotic Arm" Instructables project
- Classic robotics texts for the geometric IK derivation
