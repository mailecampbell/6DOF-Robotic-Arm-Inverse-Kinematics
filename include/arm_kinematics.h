/**
 * Forward and Inverse Kinematics for 6-DOF Robotic Arm
 *
 * This implements geometric/analytical IK for the BasementMaker-style arm:
 * - Base (yaw about Z)
 * - Shoulder (pitch about Y)
 * - Elbow (pitch about Y)
 * - Wrist Pitch (pitch about Y)
 * - Wrist Roll (roll about local X/tool axis)
 * - Gripper (not part of spatial IK)
 *
 * GEOMETRIC IK APPROACH:
 * ======================
 * 1. Compute wrist center by subtracting tool length along tool Z-axis from target
 * 2. Base angle = atan2(wrist_y, wrist_x)
 * 3. Project to shoulder plane and solve 2-link planar IK for shoulder+elbow
 * 4. Compute wrist pitch to achieve desired end-effector pitch
 * 5. Compute wrist roll from desired orientation about tool axis
 */

#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include "arm_config.h"
#include "arm_math.h"

// ============================================================================
// FORWARD KINEMATICS
// ============================================================================

/**
 * Compute end-effector pose from joint angles.
 *
 * @param joints Joint angles in degrees
 * @param geometry Arm link lengths and offsets
 * @return End-effector pose (position in mm, orientation as rotation matrix)
 *
 * Transform chain:
 *   T_base = Rz(base)
 *   T_shoulder = Trans(0, 0, baseHeight) * Ry(shoulder)
 *   T_elbow = Trans(upperArm, 0, 0) * Ry(elbow)
 *   T_wristPitch = Trans(forearm, 0, 0) * Ry(wristPitch)
 *   T_wristRoll = Trans(wrist, 0, 0) * Rx(wristRoll)
 *   T_tool = Trans(tool, 0, 0)
 *
 *   T_total = T_base * T_shoulder * T_elbow * T_wristPitch * T_wristRoll * T_tool
 */
Pose forwardKinematics(const JointAngles& joints, const ArmGeometry& geometry);

/**
 * Compute pose at each joint frame (for visualization/debugging).
 *
 * @param joints Joint angles in degrees
 * @param geometry Arm link lengths
 * @param poses Output array of 7 poses (base, shoulder, elbow, wristPitch, wristRoll, tool, endEffector)
 */
void forwardKinematicsAllFrames(const JointAngles& joints, const ArmGeometry& geometry, Pose poses[7]);

// ============================================================================
// INVERSE KINEMATICS
// ============================================================================

/**
 * Compute joint angles from desired end-effector pose using geometric IK.
 *
 * @param targetPose Desired end-effector pose
 * @param geometry Arm link lengths
 * @param servoConfigs Servo limit configurations
 * @param options IK solver options
 * @return Joint solution with angles, validity flag, and status
 *
 * Algorithm:
 * 1. Wrist center = target_position - tool_length * target_z_axis
 * 2. Base = atan2(wrist_y, wrist_x)
 * 3. Project wrist center to sagittal plane (XZ plane after base rotation)
 * 4. Solve 2-link IK for shoulder and elbow:
 *    - r = sqrt(wx^2 + wy^2) - shoulder_offset_x
 *    - h = wz - base_height
 *    - d = sqrt(r^2 + h^2)  (distance from shoulder to wrist center)
 *    - Use law of cosines for elbow angle
 *    - Compute shoulder angle using geometry
 * 5. Wrist pitch = desired_pitch - shoulder - elbow
 * 6. Wrist roll = extracted from orientation about tool axis
 */
JointSolution inverseKinematics(
    const Pose& targetPose,
    const ArmGeometry& geometry,
    const ServoConfig servoConfigs[NUM_JOINTS],
    const IKOptions& options = DEFAULT_IK_OPTIONS
);

/**
 * Compute both elbow-up and elbow-down solutions.
 *
 * @param targetPose Desired end-effector pose
 * @param geometry Arm link lengths
 * @param servoConfigs Servo limit configurations
 * @param options IK solver options
 * @param solutionUp Output: elbow-up solution
 * @param solutionDown Output: elbow-down solution
 */
void inverseKinematicsBothConfigs(
    const Pose& targetPose,
    const ArmGeometry& geometry,
    const ServoConfig servoConfigs[NUM_JOINTS],
    const IKOptions& options,
    JointSolution& solutionUp,
    JointSolution& solutionDown
);

// ============================================================================
// WORKSPACE AND REACHABILITY
// ============================================================================

/**
 * Check if a position is reachable (ignoring orientation).
 *
 * @param position Target position in mm
 * @param geometry Arm geometry
 * @return true if position is within workspace
 */
bool isPositionReachable(const Vec3& position, const ArmGeometry& geometry);

/**
 * Compute closest reachable position if target is outside workspace.
 *
 * @param position Target position in mm
 * @param geometry Arm geometry
 * @return Closest point on workspace boundary
 */
Vec3 closestReachablePosition(const Vec3& position, const ArmGeometry& geometry);

/**
 * Get workspace boundaries.
 *
 * @param geometry Arm geometry
 * @param minRadius Output: minimum reach radius in XY plane
 * @param maxRadius Output: maximum reach radius in XY plane
 * @param minHeight Output: minimum reachable height
 * @param maxHeight Output: maximum reachable height
 */
void getWorkspaceBounds(
    const ArmGeometry& geometry,
    float& minRadius,
    float& maxRadius,
    float& minHeight,
    float& maxHeight
);

// ============================================================================
// JOINT LIMIT UTILITIES
// ============================================================================

/**
 * Clamp joint angles to servo limits.
 *
 * @param joints Input joint angles
 * @param servoConfigs Servo configurations with limits
 * @param wasClamped Output: set to true if any joint was clamped
 * @return Clamped joint angles
 */
JointAngles clampToLimits(
    const JointAngles& joints,
    const ServoConfig servoConfigs[NUM_JOINTS],
    bool& wasClamped
);

/**
 * Check if all joint angles are within limits.
 *
 * @param joints Joint angles to check
 * @param servoConfigs Servo configurations with limits
 * @return true if all joints are within limits
 */
bool isWithinLimits(
    const JointAngles& joints,
    const ServoConfig servoConfigs[NUM_JOINTS]
);

// ============================================================================
// GRIPPER UTILITIES
// ============================================================================

/**
 * Map gripper opening distance to servo angle.
 *
 * @param openingMm Gripper opening in mm (0 = closed)
 * @param maxOpeningMm Maximum gripper opening
 * @param config Gripper servo configuration
 * @return Servo angle in degrees
 */
float gripperDistanceToAngle(float openingMm, float maxOpeningMm, const ServoConfig& config);

/**
 * Map gripper percentage to servo angle.
 *
 * @param percent Gripper opening percentage (0 = closed, 100 = fully open)
 * @param config Gripper servo configuration
 * @return Servo angle in degrees
 */
float gripperPercentToAngle(float percent, const ServoConfig& config);

#endif // ARM_KINEMATICS_H
