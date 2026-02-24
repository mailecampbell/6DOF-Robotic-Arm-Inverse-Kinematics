/**
 * Arm Configuration for 3D Printed Arduino Robotic Arm (BasementMaker)
 *
 * COORDINATE FRAME CONVENTION:
 * ============================
 * - World frame origin at center of base rotation axis, at table level
 * - X-axis: forward (arm pointing forward when base=0)
 * - Y-axis: left (following right-hand rule)
 * - Z-axis: up (vertical)
 *
 * JOINT CONVENTIONS:
 * ==================
 * Joint 0 (Base):         Rotation about Z-axis (yaw). 0° = forward (+X)
 * Joint 1 (Shoulder):     Rotation about Y-axis (pitch). 0° = upper arm horizontal forward
 * Joint 2 (Elbow):        Rotation about Y-axis (pitch). 0° = forearm aligned with upper arm
 * Joint 3 (Wrist Pitch):  Rotation about Y-axis (pitch). 0° = gripper aligned with forearm
 * Joint 4 (Wrist Roll):   Rotation about local X-axis (roll). 0° = gripper vertical
 * Joint 5 (Gripper):      Linear actuator mapped to angle. 0° = closed, 90° = open
 *
 * POSITIVE ANGLE DIRECTIONS:
 * - Base: CCW when viewed from above (toward +Y)
 * - Shoulder: CCW when viewed from +Y (raises arm up)
 * - Elbow: CCW when viewed from +Y (bends arm inward/up)
 * - Wrist Pitch: CCW when viewed from +Y
 * - Wrist Roll: CW when viewed from +X (gripper rotates)
 */

#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#include <stdint.h>

// ============================================================================
// LINK LENGTHS (in millimeters)
// ============================================================================
// Measure these from your actual printed arm or the STL files
// See README for measurement instructions

struct ArmGeometry {
    // Vertical offset from world origin (table) to shoulder joint axis
    float baseHeight;           // ~70mm typical

    // Horizontal offset from base rotation axis to shoulder joint
    float shoulderOffsetX;      // ~0mm (usually centered)

    // Upper arm length: shoulder joint axis to elbow joint axis
    float upperArmLength;       // ~105mm typical (L1)

    // Forearm length: elbow joint axis to wrist pitch joint axis
    float forearmLength;        // ~98mm typical (L2)

    // Wrist length: wrist pitch axis to wrist roll axis
    float wristLength;          // ~25mm typical

    // Tool/gripper length: wrist roll axis to gripper tip (end effector)
    float toolLength;           // ~70mm typical (including gripper)

    // Combined wrist-to-tool for wrist center calculation
    float wristToTool() const { return wristLength + toolLength; }
};

// Default geometry for the BasementMaker arm (adjust to your build)
constexpr ArmGeometry DEFAULT_GEOMETRY = {
    .baseHeight = 66.76f,
    .shoulderOffsetX = 0.0f,
    .upperArmLength = 31.1f,
    .forearmLength = 33.0f,
    .wristLength = 60.0f,
    .toolLength = 16.4f
};

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

struct ServoConfig {
    uint8_t pin;                // Arduino pin number
    float minAngle;             // Minimum mechanical angle (degrees)
    float maxAngle;             // Maximum mechanical angle (degrees)
    float zeroOffset;           // Offset to align servo "90" with joint "0"
    bool inverted;              // True if servo direction is reversed
    uint16_t minPulseUs;        // Minimum pulse width (microseconds)
    uint16_t maxPulseUs;        // Maximum pulse width (microseconds)
};

// Joint indices
enum JointIndex {
    JOINT_BASE = 0,
    JOINT_SHOULDER = 1,
    JOINT_ELBOW = 2,
    JOINT_WRIST_PITCH = 3,
    JOINT_WRIST_ROLL = 4,
    JOINT_GRIPPER = 5,
    NUM_JOINTS = 6
};

// Default servo configuration for BasementMaker arm
// Pin mapping: Base=D3, Shoulder=D5, Elbow=D6, WristPitch=D11, WristRoll=D10, Gripper=D9
constexpr ServoConfig DEFAULT_SERVO_CONFIG[NUM_JOINTS] = {
    // Base (J0): yaw rotation
    {
        .pin = 3,
        .minAngle = -90.0f,     // Leftmost position
        .maxAngle = 90.0f,      // Rightmost position
        .zeroOffset = 90.0f,    // Servo 90° = joint 0°
        .inverted = false,
        .minPulseUs = 544,
        .maxPulseUs = 2400
    },
    // Shoulder (J1): pitch
    {
        .pin = 5,
        .minAngle = -30.0f,     // Below horizontal (limited by structure)
        .maxAngle = 135.0f,     // Up and back
        .zeroOffset = 90.0f,
        .inverted = false,
        .minPulseUs = 544,
        .maxPulseUs = 2400
    },
    // Elbow (J2): pitch
    {
        .pin = 6,
        .minAngle = -135.0f,    // Fully folded
        .maxAngle = 0.0f,       // Straight (aligned with upper arm)
        .zeroOffset = 90.0f,
        .inverted = true,       // Often inverted due to mounting
        .minPulseUs = 544,
        .maxPulseUs = 2400
    },
    // Wrist Pitch (J3)
    {
        .pin = 11,
        .minAngle = -90.0f,
        .maxAngle = 90.0f,
        .zeroOffset = 90.0f,
        .inverted = false,
        .minPulseUs = 544,
        .maxPulseUs = 2400
    },
    // Wrist Roll (J4)
    {
        .pin = 10,
        .minAngle = -90.0f,
        .maxAngle = 90.0f,
        .zeroOffset = 90.0f,
        .inverted = false,
        .minPulseUs = 544,
        .maxPulseUs = 2400
    },
    // Gripper (J5)
    {
        .pin = 9,
        .minAngle = 0.0f,       // Closed
        .maxAngle = 90.0f,      // Open
        .zeroOffset = 0.0f,     // No offset needed
        .inverted = false,
        .minPulseUs = 544,
        .maxPulseUs = 2400
    }
};

// ============================================================================
// IK SOLVER OPTIONS
// ============================================================================

enum IKStatus {
    IK_OK = 0,                      // Solution found within limits
    IK_UNREACHABLE = 1,             // Target outside workspace
    IK_SINGULAR = 2,                // Singular configuration encountered
    IK_LIMIT_CLAMPED = 3,           // Solution found but clamped to limits
    IK_NO_SOLUTION = 4,             // No valid solution exists
    IK_NUMERICAL_FALLBACK = 5,      // Used numerical solver
    IK_ORIENTATION_UNREACHABLE = 6  // Position OK but orientation impossible
};

enum ElbowConfig {
    ELBOW_UP = 0,       // Elbow points upward (preferred for most tasks)
    ELBOW_DOWN = 1,     // Elbow points downward
    ELBOW_AUTO = 2      // Choose based on proximity to current config
};

struct IKOptions {
    ElbowConfig elbowConfig;        // Preferred elbow configuration
    bool returnClosest;             // If unreachable, return closest valid pose
    bool clampToLimits;             // Clamp joint angles to servo limits
    float positionTolerance;        // Position tolerance in mm
    float orientationTolerance;     // Orientation tolerance in degrees
    float* currentJoints;           // Current joint angles for ELBOW_AUTO (can be null)
};

constexpr IKOptions DEFAULT_IK_OPTIONS = {
    .elbowConfig = ELBOW_UP,
    .returnClosest = true,
    .clampToLimits = true,
    .positionTolerance = 1.0f,      // 1mm
    .orientationTolerance = 1.0f,   // 1 degree
    .currentJoints = nullptr
};

// ============================================================================
// JOINT SOLUTION STRUCTURE
// ============================================================================

struct JointAngles {
    float base;         // Joint 0: base yaw (degrees)
    float shoulder;     // Joint 1: shoulder pitch (degrees)
    float elbow;        // Joint 2: elbow pitch (degrees)
    float wristPitch;   // Joint 3: wrist pitch (degrees)
    float wristRoll;    // Joint 4: wrist roll (degrees)
    float gripper;      // Joint 5: gripper (degrees)

    // Array-style access
    float& operator[](int i) {
        switch(i) {
            case 0: return base;
            case 1: return shoulder;
            case 2: return elbow;
            case 3: return wristPitch;
            case 4: return wristRoll;
            case 5: return gripper;
            default: return base;
        }
    }

    const float& operator[](int i) const {
        switch(i) {
            case 0: return base;
            case 1: return shoulder;
            case 2: return elbow;
            case 3: return wristPitch;
            case 4: return wristRoll;
            case 5: return gripper;
            default: return base;
        }
    }
};

struct JointSolution {
    JointAngles angles;     // Joint angles in degrees
    bool valid;             // True if solution is valid
    IKStatus status;        // Status code
    float positionError;    // Position error in mm (if applicable)
    float orientationError; // Orientation error in degrees (if applicable)
};

// ============================================================================
// POSE REPRESENTATION
// ============================================================================

struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }
    float dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    float length() const;
    Vec3 normalized() const;
    Vec3 cross(const Vec3& v) const;
};

// Roll-Pitch-Yaw orientation (in degrees)
// Convention: Rz(yaw) * Ry(pitch) * Rx(roll) applied to base frame
struct RPY {
    float roll;     // Rotation about X-axis
    float pitch;    // Rotation about Y-axis
    float yaw;      // Rotation about Z-axis

    RPY() : roll(0), pitch(0), yaw(0) {}
    RPY(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

// 3x3 Rotation matrix (row-major)
struct Mat3 {
    float m[3][3];

    Mat3();  // Identity
    static Mat3 rotX(float angleDeg);
    static Mat3 rotY(float angleDeg);
    static Mat3 rotZ(float angleDeg);
    static Mat3 fromRPY(const RPY& rpy);

    Mat3 operator*(const Mat3& other) const;
    Vec3 operator*(const Vec3& v) const;
    Mat3 transpose() const;

    Vec3 col(int i) const;  // Get column as vector
    RPY toRPY() const;      // Extract RPY angles
};

// Complete pose: position + orientation
struct Pose {
    Vec3 position;      // Position in mm
    Mat3 orientation;   // Orientation as rotation matrix

    Pose() {}
    Pose(const Vec3& pos, const Mat3& rot) : position(pos), orientation(rot) {}
    Pose(const Vec3& pos, const RPY& rpy) : position(pos), orientation(Mat3::fromRPY(rpy)) {}
};

#endif // ARM_CONFIG_H
