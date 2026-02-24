/**
 * Forward and Inverse Kinematics Implementation
 *
 * Geometric IK derivation for planar 2-link arm:
 *
 *      (shoulder)----L1----(elbow)----L2----(wrist)
 *           |                                  ^
 *           |                                  |
 *           h = wrist height - base height     |
 *           |                                  |
 *           +-------------- r -----------------+
 *                (horizontal distance)
 *
 * Let d = sqrt(r² + h²) = distance from shoulder to wrist
 *
 * Using law of cosines for elbow angle:
 *   d² = L1² + L2² - 2·L1·L2·cos(π - elbow)
 *   d² = L1² + L2² + 2·L1·L2·cos(elbow)
 *   cos(elbow) = (d² - L1² - L2²) / (2·L1·L2)
 *
 * For shoulder angle:
 *   α = atan2(h, r)  (angle to wrist from horizontal)
 *   β = acos((L1² + d² - L2²) / (2·L1·d))  (angle at shoulder in triangle)
 *   shoulder = α + β  (elbow up) or α - β (elbow down)
 */

#include "arm_kinematics.h"
#include <cmath>
#include <algorithm>

// ============================================================================
// FORWARD KINEMATICS IMPLEMENTATION
// ============================================================================

Pose forwardKinematics(const JointAngles& joints, const ArmGeometry& geometry) {
    // All angles in radians for computation
    float base = degToRad(joints.base);
    float shoulder = degToRad(joints.shoulder);
    float elbow = degToRad(joints.elbow);
    float wristPitch = degToRad(joints.wristPitch);
    float wristRoll = degToRad(joints.wristRoll);

    // Cumulative pitch angle of arm segments (in base-rotated plane)
    // Each pitch joint adds to the cumulative angle from horizontal
    float theta1 = shoulder;                          // Upper arm angle from horizontal
    float theta2 = shoulder + elbow;                  // Forearm angle from horizontal
    float theta3 = shoulder + elbow + wristPitch;     // Tool angle from horizontal

    // Position calculations in arm plane (before base rotation)
    // Starting from shoulder joint at (0, 0, baseHeight) in base-rotated frame

    // Shoulder position (in base-rotated frame, X' is forward in arm plane)
    float sx = geometry.shoulderOffsetX;
    float sz = geometry.baseHeight;

    // Elbow position
    float ex = sx + geometry.upperArmLength * std::cos(theta1);
    float ez = sz + geometry.upperArmLength * std::sin(theta1);

    // Wrist pitch joint position
    float wpx = ex + geometry.forearmLength * std::cos(theta2);
    float wpz = ez + geometry.forearmLength * std::sin(theta2);

    // Wrist roll joint position
    float wrx = wpx + geometry.wristLength * std::cos(theta3);
    float wrz = wpz + geometry.wristLength * std::sin(theta3);

    // End effector position (tool tip)
    float toolx = wrx + geometry.toolLength * std::cos(theta3);
    float toolz = wrz + geometry.toolLength * std::sin(theta3);

    // Apply base rotation to get world coordinates
    float cosBase = std::cos(base);
    float sinBase = std::sin(base);

    Vec3 position(
        toolx * cosBase,    // x = planar_x * cos(base)
        toolx * sinBase,    // y = planar_x * sin(base)
        toolz               // z = planar_z (unchanged by base rotation)
    );

    // Orientation calculation
    // Start with identity, then apply rotations in order:
    // 1. Rz(base) - rotate entire arm about vertical axis
    // 2. Ry(shoulder + elbow + wristPitch) - cumulative pitch in arm plane
    // 3. Rx(wristRoll) - roll about tool axis

    // Build rotation matrix:
    // R = Rz(base) * Ry(pitch_total) * Rx(wristRoll)
    // where pitch_total = shoulder + elbow + wristPitch

    Mat3 Rbase = Mat3::rotZ(joints.base);
    Mat3 Rpitch = Mat3::rotY(joints.shoulder + joints.elbow + joints.wristPitch);
    Mat3 Rroll = Mat3::rotX(joints.wristRoll);

    Mat3 orientation = Rbase * Rpitch * Rroll;

    return Pose(position, orientation);
}

void forwardKinematicsAllFrames(const JointAngles& joints, const ArmGeometry& geometry, Pose poses[7]) {
    float base = degToRad(joints.base);
    float shoulder = degToRad(joints.shoulder);
    float elbow = degToRad(joints.elbow);
    float wristPitch = degToRad(joints.wristPitch);
    float wristRoll = degToRad(joints.wristRoll);

    float cosBase = std::cos(base);
    float sinBase = std::sin(base);

    // Frame 0: Base (at origin, only rotated)
    poses[0].position = Vec3(0, 0, 0);
    poses[0].orientation = Mat3::rotZ(joints.base);

    // Frame 1: Shoulder
    float sx = geometry.shoulderOffsetX;
    float sz = geometry.baseHeight;
    poses[1].position = Vec3(sx * cosBase, sx * sinBase, sz);
    poses[1].orientation = Mat3::rotZ(joints.base) * Mat3::rotY(joints.shoulder);

    // Frame 2: Elbow
    float theta1 = shoulder;
    float ex = sx + geometry.upperArmLength * std::cos(theta1);
    float ez = sz + geometry.upperArmLength * std::sin(theta1);
    poses[2].position = Vec3(ex * cosBase, ex * sinBase, ez);
    poses[2].orientation = Mat3::rotZ(joints.base) * Mat3::rotY(joints.shoulder + joints.elbow);

    // Frame 3: Wrist Pitch
    float theta2 = shoulder + elbow;
    float wpx = ex + geometry.forearmLength * std::cos(theta2);
    float wpz = ez + geometry.forearmLength * std::sin(theta2);
    poses[3].position = Vec3(wpx * cosBase, wpx * sinBase, wpz);
    poses[3].orientation = Mat3::rotZ(joints.base) *
                           Mat3::rotY(joints.shoulder + joints.elbow + joints.wristPitch);

    // Frame 4: Wrist Roll
    float theta3 = theta2 + degToRad(joints.wristPitch);
    float wrx = wpx + geometry.wristLength * std::cos(theta3);
    float wrz = wpz + geometry.wristLength * std::sin(theta3);
    poses[4].position = Vec3(wrx * cosBase, wrx * sinBase, wrz);
    poses[4].orientation = Mat3::rotZ(joints.base) *
                           Mat3::rotY(joints.shoulder + joints.elbow + joints.wristPitch) *
                           Mat3::rotX(joints.wristRoll);

    // Frame 5 & 6: Tool tip (end effector)
    float toolx = wrx + geometry.toolLength * std::cos(theta3);
    float toolz = wrz + geometry.toolLength * std::sin(theta3);
    poses[5].position = Vec3(toolx * cosBase, toolx * sinBase, toolz);
    poses[5].orientation = poses[4].orientation;
    poses[6] = poses[5];  // Same as tool frame
}

// ============================================================================
// INVERSE KINEMATICS IMPLEMENTATION
// ============================================================================

JointSolution inverseKinematics(
    const Pose& targetPose,
    const ArmGeometry& geometry,
    const ServoConfig servoConfigs[NUM_JOINTS],
    const IKOptions& options
) {
    JointSolution solution;
    solution.valid = false;
    solution.status = IK_OK;
    solution.positionError = 0;
    solution.orientationError = 0;

    // ========================================================================
    // STEP 1: Compute wrist center position
    // ========================================================================
    // The wrist center is located at (wristLength + toolLength) back from
    // the end effector along the tool's forward direction.
    //
    // The tool forward direction in the arm plane is [cos(pitch), 0, sin(pitch)]
    // where pitch = shoulder + elbow + wristPitch.
    //
    // After Rz(base), this becomes [cos(base)*cos(pitch), sin(base)*cos(pitch), sin(pitch)].
    //
    // However, the standard rotY matrix gives col(0) = [cos(pitch), 0, -sin(pitch)],
    // so we need to negate the Z component to get the physical direction.

    // Get tool axis from orientation, with corrected Z sign
    Vec3 toolAxis(
        targetPose.orientation.m[0][0],
        targetPose.orientation.m[1][0],
        -targetPose.orientation.m[2][0]  // Negate to match physical convention
    );

    float wristToTool = geometry.wristLength + geometry.toolLength;
    Vec3 wristCenter = targetPose.position - toolAxis * wristToTool;

    // ========================================================================
    // STEP 2: Compute base angle
    // ========================================================================
    // Base rotates about Z axis, angle is atan2(y, x) of wrist center

    float baseAngle;

    // Check for singular case: wrist center directly above/below base axis
    float wristXY = std::sqrt(wristCenter.x * wristCenter.x + wristCenter.y * wristCenter.y);
    if (wristXY < EPSILON) {
        // Wrist is on the Z axis - base angle is undefined
        // Use current base angle if available, otherwise use 0
        if (options.currentJoints != nullptr) {
            baseAngle = options.currentJoints[JOINT_BASE];
        } else {
            baseAngle = 0;
        }
        solution.status = IK_SINGULAR;
    } else {
        baseAngle = radToDeg(std::atan2(wristCenter.y, wristCenter.x));
    }

    solution.angles.base = baseAngle;

    // ========================================================================
    // STEP 3: Transform wrist center to shoulder frame (arm plane)
    // ========================================================================
    // After base rotation, the arm operates in a vertical plane
    // We need the radial distance (r) and height (h) of wrist center
    // relative to the shoulder joint

    float r = wristXY - geometry.shoulderOffsetX;  // Radial distance in arm plane
    float h = wristCenter.z - geometry.baseHeight;  // Height relative to shoulder

    // Distance from shoulder to wrist center
    float d = std::sqrt(r * r + h * h);

    // ========================================================================
    // STEP 4: Check reachability
    // ========================================================================
    float L1 = geometry.upperArmLength;
    float L2 = geometry.forearmLength;

    float maxReach = L1 + L2;
    float minReach = std::abs(L1 - L2);

    if (d > maxReach) {
        // Target too far - arm fully extended won't reach
        if (!options.returnClosest) {
            solution.status = IK_UNREACHABLE;
            return solution;
        }

        // Scale down to max reach
        float scale = maxReach / d;
        r *= scale;
        h *= scale;
        d = maxReach;
        solution.status = IK_UNREACHABLE;
    } else if (d < minReach) {
        // Target too close - arm can't fold enough
        if (!options.returnClosest) {
            solution.status = IK_UNREACHABLE;
            return solution;
        }

        // Scale up to min reach
        float scale = minReach / d;
        r *= scale;
        h *= scale;
        d = minReach;
        solution.status = IK_UNREACHABLE;
    }

    // ========================================================================
    // STEP 5: Solve 2-link planar IK for shoulder and elbow
    // ========================================================================
    // Using law of cosines:
    // d² = L1² + L2² - 2·L1·L2·cos(π - θ_elbow)
    // d² = L1² + L2² + 2·L1·L2·cos(θ_elbow)
    // cos(θ_elbow) = (d² - L1² - L2²) / (2·L1·L2)

    float cosElbow = (d * d - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cosElbow = clamp(cosElbow, -1.0f, 1.0f);  // Clamp for numerical stability

    // Two solutions: elbow up (negative angle) and elbow down (positive angle)
    // Our convention: elbow angle of 0 means straight, negative means bent
    float elbowAngle;
    if (options.elbowConfig == ELBOW_UP) {
        elbowAngle = -std::acos(cosElbow);  // Negative for elbow-up
    } else if (options.elbowConfig == ELBOW_DOWN) {
        elbowAngle = std::acos(cosElbow);   // Positive for elbow-down
    } else {
        // ELBOW_AUTO: choose based on current configuration
        float elbowUp = -std::acos(cosElbow);
        float elbowDown = std::acos(cosElbow);

        if (options.currentJoints != nullptr) {
            float currentElbow = degToRad(options.currentJoints[JOINT_ELBOW]);
            elbowAngle = (std::abs(elbowUp - currentElbow) < std::abs(elbowDown - currentElbow))
                         ? elbowUp : elbowDown;
        } else {
            elbowAngle = elbowUp;  // Default to elbow-up
        }
    }

    // Shoulder angle calculation:
    // α = atan2(h, r)  -- angle to wrist center from horizontal
    // β = acos((L1² + d² - L2²) / (2·L1·d))  -- angle at shoulder in triangle
    // shoulder = α + β (elbow up) or α - β (elbow down)

    float alpha = std::atan2(h, r);
    float cosBeta = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
    cosBeta = clamp(cosBeta, -1.0f, 1.0f);
    float beta = std::acos(cosBeta);

    float shoulderAngle;
    if (elbowAngle < 0) {
        // Elbow up configuration
        shoulderAngle = alpha + beta;
    } else {
        // Elbow down configuration
        shoulderAngle = alpha - beta;
    }

    solution.angles.shoulder = radToDeg(shoulderAngle);
    solution.angles.elbow = radToDeg(elbowAngle);

    // ========================================================================
    // STEP 6: Compute wrist pitch to achieve desired end-effector orientation
    // ========================================================================
    // The cumulative pitch of the tool relative to horizontal is:
    //   toolPitch = shoulder + elbow + wristPitch
    //
    // We need to extract the desired pitch from the target orientation.
    // The tool X-axis (forward) direction gives us the pitch.

    // Get tool pitch from target orientation
    // Tool X-axis (forward) = col(0) of orientation matrix
    // In the base-rotated frame, pitch = atan2(toolZ, toolXY)

    float baseRad = degToRad(baseAngle);
    float cosBaseR = std::cos(baseRad);
    float sinBaseR = std::sin(baseRad);

    // Rotate tool axis back to arm plane frame
    Vec3 toolAxisLocal(
        toolAxis.x * cosBaseR + toolAxis.y * sinBaseR,  // Forward component
        -toolAxis.x * sinBaseR + toolAxis.y * cosBaseR, // Sideways (should be ~0)
        toolAxis.z                                       // Vertical component
    );

    float desiredPitch = std::atan2(toolAxisLocal.z, toolAxisLocal.x);
    float cumulativePitch = shoulderAngle + elbowAngle;
    float wristPitchAngle = desiredPitch - cumulativePitch;

    solution.angles.wristPitch = radToDeg(wristPitchAngle);

    // ========================================================================
    // STEP 7: Compute wrist roll
    // ========================================================================
    // Wrist roll rotates about the tool axis
    // We need to find the rotation about the tool X-axis that aligns the
    // gripper Y/Z axes with the desired orientation

    // After base + shoulder + elbow + wristPitch, the tool frame has:
    //   X-axis = tool forward direction
    //   Y-axis = left (perpendicular to arm plane initially)
    //   Z-axis = up in arm plane

    // The wrist roll rotates around the tool X-axis
    // We can extract this by looking at the Y-axis of the desired orientation
    // relative to where it "should" be without roll

    // Compute what the Y-axis would be with roll = 0
    Mat3 Rbase = Mat3::rotZ(baseAngle);
    Mat3 Rpitch = Mat3::rotY(solution.angles.shoulder + solution.angles.elbow + solution.angles.wristPitch);
    Mat3 noRollOrientation = Rbase * Rpitch;

    // Y-axis without roll
    Vec3 yNoRoll = noRollOrientation.col(1);
    // Z-axis without roll
    Vec3 zNoRoll = noRollOrientation.col(2);

    // Desired Y and Z axes
    Vec3 yDesired = targetPose.orientation.col(1);
    Vec3 zDesired = targetPose.orientation.col(2);

    // Wrist roll is the angle between yNoRoll and yDesired
    // projected onto the plane perpendicular to the tool axis
    float wristRollAngle = std::atan2(
        yDesired.dot(zNoRoll),
        yDesired.dot(yNoRoll)
    );

    solution.angles.wristRoll = radToDeg(wristRollAngle);

    // ========================================================================
    // STEP 8: Set gripper (pass through or default)
    // ========================================================================
    solution.angles.gripper = 0;  // Default closed

    // ========================================================================
    // STEP 9: Apply joint limits if requested
    // ========================================================================
    if (options.clampToLimits) {
        bool wasClamped = false;
        solution.angles = clampToLimits(solution.angles, servoConfigs, wasClamped);
        if (wasClamped && solution.status == IK_OK) {
            solution.status = IK_LIMIT_CLAMPED;
        }
    }

    // ========================================================================
    // STEP 10: Verify solution by running FK
    // ========================================================================
    Pose verifyPose = forwardKinematics(solution.angles, geometry);

    solution.positionError = (verifyPose.position - targetPose.position).length();

    // Orientation error: compute angle between orientations
    Mat3 Rerr = targetPose.orientation.transpose() * verifyPose.orientation;
    // Trace of rotation matrix is 1 + 2*cos(angle)
    float trace = Rerr.m[0][0] + Rerr.m[1][1] + Rerr.m[2][2];
    float cosAngle = clamp((trace - 1.0f) / 2.0f, -1.0f, 1.0f);
    solution.orientationError = radToDeg(std::acos(cosAngle));

    // Mark as valid if errors are within tolerance
    if (solution.positionError <= options.positionTolerance &&
        solution.orientationError <= options.orientationTolerance) {
        solution.valid = true;
    } else {
        solution.valid = (solution.status != IK_UNREACHABLE);
        if (solution.status == IK_OK) {
            solution.status = IK_ORIENTATION_UNREACHABLE;
        }
    }

    return solution;
}

void inverseKinematicsBothConfigs(
    const Pose& targetPose,
    const ArmGeometry& geometry,
    const ServoConfig servoConfigs[NUM_JOINTS],
    const IKOptions& options,
    JointSolution& solutionUp,
    JointSolution& solutionDown
) {
    IKOptions optUp = options;
    optUp.elbowConfig = ELBOW_UP;
    solutionUp = inverseKinematics(targetPose, geometry, servoConfigs, optUp);

    IKOptions optDown = options;
    optDown.elbowConfig = ELBOW_DOWN;
    solutionDown = inverseKinematics(targetPose, geometry, servoConfigs, optDown);
}

// ============================================================================
// WORKSPACE AND REACHABILITY
// ============================================================================

bool isPositionReachable(const Vec3& position, const ArmGeometry& geometry) {
    // Compute distance from shoulder to target in arm plane
    float wristXY = std::sqrt(position.x * position.x + position.y * position.y);
    float r = wristXY - geometry.shoulderOffsetX;
    float h = position.z - geometry.baseHeight;
    float d = std::sqrt(r * r + h * h);

    // Account for wrist-to-tool offset (rough approximation)
    float wristToTool = geometry.wristLength + geometry.toolLength;
    float effectiveD = d - wristToTool;  // Approximate

    float L1 = geometry.upperArmLength;
    float L2 = geometry.forearmLength;
    float maxReach = L1 + L2;
    float minReach = std::abs(L1 - L2);

    // Check if the effective distance is within reach
    // This is a simplified check - actual reachability depends on orientation
    return (effectiveD >= 0) && (effectiveD <= maxReach) && (effectiveD >= minReach * 0.5f);
}

Vec3 closestReachablePosition(const Vec3& position, const ArmGeometry& geometry) {
    // Project to arm plane
    float wristXY = std::sqrt(position.x * position.x + position.y * position.y);
    float baseAngle = std::atan2(position.y, position.x);

    float r = wristXY - geometry.shoulderOffsetX;
    float h = position.z - geometry.baseHeight;
    float d = std::sqrt(r * r + h * h);

    float L1 = geometry.upperArmLength;
    float L2 = geometry.forearmLength;
    float maxReach = L1 + L2;
    float minReach = std::abs(L1 - L2);

    // Clamp distance to valid range
    float clampedD = clamp(d, minReach, maxReach);

    if (d > EPSILON) {
        // Scale r and h proportionally
        float scale = clampedD / d;
        r *= scale;
        h *= scale;
    }

    // Convert back to world coordinates
    float newXY = r + geometry.shoulderOffsetX;
    return Vec3(
        newXY * std::cos(baseAngle),
        newXY * std::sin(baseAngle),
        h + geometry.baseHeight
    );
}

void getWorkspaceBounds(
    const ArmGeometry& geometry,
    float& minRadius,
    float& maxRadius,
    float& minHeight,
    float& maxHeight
) {
    float L1 = geometry.upperArmLength;
    float L2 = geometry.forearmLength;
    float wristToTool = geometry.wristLength + geometry.toolLength;

    maxRadius = L1 + L2 + wristToTool + geometry.shoulderOffsetX;
    float minR = std::abs(L1 - L2) - wristToTool + geometry.shoulderOffsetX;
    minRadius = (minR > 0.0f) ? minR : 0.0f;

    maxHeight = geometry.baseHeight + L1 + L2 + wristToTool;
    float minH = geometry.baseHeight - L1 - L2 - wristToTool;
    minHeight = (minH > 0.0f) ? minH : 0.0f;
}

// ============================================================================
// JOINT LIMIT UTILITIES
// ============================================================================

JointAngles clampToLimits(
    const JointAngles& joints,
    const ServoConfig servoConfigs[NUM_JOINTS],
    bool& wasClamped
) {
    wasClamped = false;
    JointAngles clamped = joints;

    for (int i = 0; i < NUM_JOINTS; i++) {
        float orig = clamped[i];
        float minA = servoConfigs[i].minAngle;
        float maxA = servoConfigs[i].maxAngle;

        if (orig < minA) {
            clamped[i] = minA;
            wasClamped = true;
        } else if (orig > maxA) {
            clamped[i] = maxA;
            wasClamped = true;
        }
    }

    return clamped;
}

bool isWithinLimits(
    const JointAngles& joints,
    const ServoConfig servoConfigs[NUM_JOINTS]
) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        float angle = joints[i];
        if (angle < servoConfigs[i].minAngle || angle > servoConfigs[i].maxAngle) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// GRIPPER UTILITIES
// ============================================================================

float gripperDistanceToAngle(float openingMm, float maxOpeningMm, const ServoConfig& config) {
    float percent = clamp(openingMm / maxOpeningMm * 100.0f, 0.0f, 100.0f);
    return gripperPercentToAngle(percent, config);
}

float gripperPercentToAngle(float percent, const ServoConfig& config) {
    percent = clamp(percent, 0.0f, 100.0f);
    float range = config.maxAngle - config.minAngle;
    return config.minAngle + (percent / 100.0f) * range;
}
