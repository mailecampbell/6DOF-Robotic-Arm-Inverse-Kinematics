/**
 * Desktop Test Harness for Robot Arm Kinematics
 *
 * This program tests the forward and inverse kinematics by:
 * 1. Randomly sampling joint angles within limits
 * 2. Running FK to get a pose
 * 3. Running IK on that pose
 * 4. Running FK on the IK result
 * 5. Comparing original and recovered poses
 *
 * Compile:
 *   g++ -std=c++11 -I../include -o test_kinematics test_kinematics.cpp ../src/arm_kinematics.cpp
 *
 * Run:
 *   ./test_kinematics [num_tests]
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>

#include "arm_config.h"
#include "arm_math.h"
#include "arm_kinematics.h"

// =============================================================================
// TEST CONFIGURATION
// =============================================================================

const int DEFAULT_NUM_TESTS = 1000;
const float POSITION_TOLERANCE = 2.0f;       // mm
const float ORIENTATION_TOLERANCE = 3.0f;    // degrees

// Test geometry (same as default)
ArmGeometry testGeometry = DEFAULT_GEOMETRY;
ServoConfig testServoConfigs[NUM_JOINTS];

// =============================================================================
// UTILITIES
// =============================================================================

float randomFloat(float min, float max) {
    return min + static_cast<float>(rand()) / RAND_MAX * (max - min);
}

void initServoConfigs() {
    // Copy default configs
    for (int i = 0; i < NUM_JOINTS; i++) {
        testServoConfigs[i] = DEFAULT_SERVO_CONFIG[i];
    }
}

void printJoints(const JointAngles& j, const char* prefix = "") {
    std::cout << prefix << "Joints: ["
              << std::fixed << std::setprecision(2)
              << j.base << ", " << j.shoulder << ", " << j.elbow << ", "
              << j.wristPitch << ", " << j.wristRoll << ", " << j.gripper << "]"
              << std::endl;
}

void printPose(const Pose& p, const char* prefix = "") {
    RPY rpy = p.orientation.toRPY();
    std::cout << prefix << "Pose: pos=["
              << std::fixed << std::setprecision(2)
              << p.position.x << ", " << p.position.y << ", " << p.position.z << "] "
              << "rpy=[" << rpy.roll << ", " << rpy.pitch << ", " << rpy.yaw << "]"
              << std::endl;
}

void printStatus(IKStatus status) {
    switch (status) {
        case IK_OK: std::cout << "OK"; break;
        case IK_UNREACHABLE: std::cout << "UNREACHABLE"; break;
        case IK_SINGULAR: std::cout << "SINGULAR"; break;
        case IK_LIMIT_CLAMPED: std::cout << "LIMIT_CLAMPED"; break;
        case IK_NO_SOLUTION: std::cout << "NO_SOLUTION"; break;
        case IK_NUMERICAL_FALLBACK: std::cout << "NUMERICAL_FALLBACK"; break;
        case IK_ORIENTATION_UNREACHABLE: std::cout << "ORIENTATION_UNREACHABLE"; break;
        default: std::cout << "UNKNOWN(" << status << ")"; break;
    }
}

// =============================================================================
// TEST CASES
// =============================================================================

struct TestResult {
    bool passed;
    float positionError;
    float orientationError;
    IKStatus status;
    JointAngles originalJoints;
    JointAngles recoveredJoints;
    Pose originalPose;
    Pose recoveredPose;
};

TestResult runSingleTest(const JointAngles& joints) {
    TestResult result;
    result.originalJoints = joints;

    // Step 1: FK to get pose
    result.originalPose = forwardKinematics(joints, testGeometry);

    // Step 2: IK to recover joints
    IKOptions options = DEFAULT_IK_OPTIONS;
    options.elbowConfig = ELBOW_UP;  // Match typical config
    options.clampToLimits = true;
    options.returnClosest = true;

    JointSolution solution = inverseKinematics(
        result.originalPose, testGeometry, testServoConfigs, options);

    result.recoveredJoints = solution.angles;
    result.status = solution.status;

    // Step 3: FK on recovered joints
    result.recoveredPose = forwardKinematics(solution.angles, testGeometry);

    // Step 4: Compare poses
    result.positionError = (result.originalPose.position - result.recoveredPose.position).length();

    // Orientation error
    Mat3 Rerr = result.originalPose.orientation.transpose() * result.recoveredPose.orientation;
    float trace = Rerr.m[0][0] + Rerr.m[1][1] + Rerr.m[2][2];
    float cosAngle = clamp((trace - 1.0f) / 2.0f, -1.0f, 1.0f);
    result.orientationError = radToDeg(std::acos(cosAngle));

    result.passed = (result.positionError < POSITION_TOLERANCE &&
                     result.orientationError < ORIENTATION_TOLERANCE);

    return result;
}

JointAngles randomJointsWithinLimits() {
    JointAngles j;
    j.base = randomFloat(testServoConfigs[0].minAngle, testServoConfigs[0].maxAngle);
    j.shoulder = randomFloat(testServoConfigs[1].minAngle, testServoConfigs[1].maxAngle);
    j.elbow = randomFloat(testServoConfigs[2].minAngle, testServoConfigs[2].maxAngle);
    j.wristPitch = randomFloat(testServoConfigs[3].minAngle, testServoConfigs[3].maxAngle);
    j.wristRoll = randomFloat(testServoConfigs[4].minAngle, testServoConfigs[4].maxAngle);
    j.gripper = 0;  // Gripper doesn't affect IK
    return j;
}

// Sample more conservatively to avoid extreme configurations
JointAngles randomJointsConservative() {
    JointAngles j;
    // Use narrower ranges that produce more achievable configurations
    // Key insight: wristPitch = desiredPitch - shoulder - elbow
    // So we need configurations where shoulder + elbow + wristPitch stays reasonable
    j.base = randomFloat(-60.0f, 60.0f);
    j.shoulder = randomFloat(20.0f, 80.0f);
    j.elbow = randomFloat(-80.0f, -30.0f);

    // Constrain wrist pitch so total pitch stays achievable
    float minWP = -40.0f - j.shoulder - j.elbow;
    float maxWP = 40.0f - j.shoulder - j.elbow;
    minWP = clamp(minWP, -90.0f, 90.0f);
    maxWP = clamp(maxWP, -90.0f, 90.0f);
    if (minWP > maxWP) { float t = minWP; minWP = maxWP; maxWP = t; }
    j.wristPitch = randomFloat(minWP, maxWP);

    j.wristRoll = randomFloat(-45.0f, 45.0f);
    j.gripper = 0;
    return j;
}

// Sample IK-friendly configurations: position above table, reasonable pitch
JointAngles randomJointsIKFriendly() {
    JointAngles j;
    // Configurations that guarantee end effector above table
    j.base = randomFloat(-80.0f, 80.0f);
    j.shoulder = randomFloat(30.0f, 100.0f);   // Upper arm raised
    j.elbow = randomFloat(-90.0f, -20.0f);      // Some bend

    // Keep wrist pitch small relative to cumulative angle
    float cumPitch = j.shoulder + j.elbow;
    // Target a reasonable tool pitch (e.g., -30 to +60 degrees from horizontal)
    float targetToolPitch = randomFloat(-30.0f, 60.0f);
    j.wristPitch = clamp(targetToolPitch - cumPitch, -70.0f, 70.0f);

    j.wristRoll = randomFloat(-60.0f, 60.0f);
    j.gripper = 0;
    return j;
}

// =============================================================================
// SPECIFIC TEST CASES
// =============================================================================

void testSpecificPoses() {
    std::cout << "\n=== Testing Specific Poses ===" << std::endl;

    // Test case 1: Home position (all zeros except for reachable config)
    {
        std::cout << "\nTest: Home position" << std::endl;
        JointAngles home = {0, 45, -45, 0, 0, 0};
        TestResult result = runSingleTest(home);
        std::cout << "  Status: "; printStatus(result.status); std::cout << std::endl;
        std::cout << "  Position error: " << result.positionError << " mm" << std::endl;
        std::cout << "  Orientation error: " << result.orientationError << " deg" << std::endl;
        std::cout << "  " << (result.passed ? "PASSED" : "FAILED") << std::endl;
    }

    // Test case 2: Arm extended forward
    {
        std::cout << "\nTest: Extended forward" << std::endl;
        JointAngles extended = {0, 0, 0, 0, 0, 0};
        TestResult result = runSingleTest(extended);
        std::cout << "  Status: "; printStatus(result.status); std::cout << std::endl;
        std::cout << "  Position error: " << result.positionError << " mm" << std::endl;
        std::cout << "  Orientation error: " << result.orientationError << " deg" << std::endl;
        std::cout << "  " << (result.passed ? "PASSED" : "FAILED") << std::endl;
        printPose(result.originalPose, "  Original: ");
        printPose(result.recoveredPose, "  Recovered: ");
    }

    // Test case 3: Arm to the side
    {
        std::cout << "\nTest: Arm rotated 45 degrees" << std::endl;
        JointAngles side = {45, 30, -30, 0, 0, 0};
        TestResult result = runSingleTest(side);
        std::cout << "  Status: "; printStatus(result.status); std::cout << std::endl;
        std::cout << "  Position error: " << result.positionError << " mm" << std::endl;
        std::cout << "  Orientation error: " << result.orientationError << " deg" << std::endl;
        std::cout << "  " << (result.passed ? "PASSED" : "FAILED") << std::endl;
    }

    // Test case 4: Elbow bent significantly
    {
        std::cout << "\nTest: Elbow bent -90 degrees" << std::endl;
        JointAngles bent = {0, 60, -90, 30, 0, 0};
        TestResult result = runSingleTest(bent);
        std::cout << "  Status: "; printStatus(result.status); std::cout << std::endl;
        std::cout << "  Position error: " << result.positionError << " mm" << std::endl;
        std::cout << "  Orientation error: " << result.orientationError << " deg" << std::endl;
        std::cout << "  " << (result.passed ? "PASSED" : "FAILED") << std::endl;
    }

    // Test case 5: With wrist roll
    {
        std::cout << "\nTest: With wrist roll 45 degrees" << std::endl;
        JointAngles rolled = {0, 45, -45, 0, 45, 0};
        TestResult result = runSingleTest(rolled);
        std::cout << "  Status: "; printStatus(result.status); std::cout << std::endl;
        std::cout << "  Position error: " << result.positionError << " mm" << std::endl;
        std::cout << "  Orientation error: " << result.orientationError << " deg" << std::endl;
        std::cout << "  " << (result.passed ? "PASSED" : "FAILED") << std::endl;
    }
}

void testSingularities() {
    std::cout << "\n=== Testing Singularities ===" << std::endl;

    // Test: Target directly above base (base angle undefined)
    {
        std::cout << "\nTest: Target above base axis" << std::endl;
        // Create a pose directly above the base
        Pose targetPose;
        targetPose.position = Vec3(0, 0, testGeometry.baseHeight + 100);
        targetPose.orientation = Mat3::rotY(-90);  // Pointing up

        IKOptions options = DEFAULT_IK_OPTIONS;
        JointSolution solution = inverseKinematics(targetPose, testGeometry, testServoConfigs, options);

        std::cout << "  Status: "; printStatus(solution.status); std::cout << std::endl;
        std::cout << "  Valid: " << (solution.valid ? "yes" : "no") << std::endl;
        printJoints(solution.angles, "  Solution: ");
    }

    // Test: Near fully extended
    {
        std::cout << "\nTest: Near full extension" << std::endl;
        JointAngles nearExtended = {0, 10, -5, 0, 0, 0};  // Almost straight
        TestResult result = runSingleTest(nearExtended);
        std::cout << "  Status: "; printStatus(result.status); std::cout << std::endl;
        std::cout << "  Position error: " << result.positionError << " mm" << std::endl;
        std::cout << "  " << (result.passed ? "PASSED" : "FAILED") << std::endl;
    }
}

void testWorkspaceBoundary() {
    std::cout << "\n=== Testing Workspace Boundary ===" << std::endl;

    float minR, maxR, minH, maxH;
    getWorkspaceBounds(testGeometry, minR, maxR, minH, maxH);

    std::cout << "Workspace bounds:" << std::endl;
    std::cout << "  Radius: " << minR << " to " << maxR << " mm" << std::endl;
    std::cout << "  Height: " << minH << " to " << maxH << " mm" << std::endl;

    // Test: Just within reach
    {
        std::cout << "\nTest: Position just within max reach" << std::endl;
        float r = maxR * 0.9f;
        Pose targetPose;
        targetPose.position = Vec3(r, 0, testGeometry.baseHeight);
        targetPose.orientation = Mat3();  // Identity

        IKOptions options = DEFAULT_IK_OPTIONS;
        JointSolution solution = inverseKinematics(targetPose, testGeometry, testServoConfigs, options);

        std::cout << "  Status: "; printStatus(solution.status); std::cout << std::endl;
        std::cout << "  Valid: " << (solution.valid ? "yes" : "no") << std::endl;
    }

    // Test: Just outside reach
    {
        std::cout << "\nTest: Position outside max reach" << std::endl;
        float r = maxR * 1.2f;
        Pose targetPose;
        targetPose.position = Vec3(r, 0, testGeometry.baseHeight);
        targetPose.orientation = Mat3();

        IKOptions options = DEFAULT_IK_OPTIONS;
        options.returnClosest = true;
        JointSolution solution = inverseKinematics(targetPose, testGeometry, testServoConfigs, options);

        std::cout << "  Status: "; printStatus(solution.status); std::cout << std::endl;
        std::cout << "  Valid (closest returned): " << (solution.valid ? "yes" : "no") << std::endl;
        std::cout << "  Position error: " << solution.positionError << " mm" << std::endl;
    }
}

void testBothElbowConfigs() {
    std::cout << "\n=== Testing Both Elbow Configurations ===" << std::endl;

    // Create a reachable pose
    JointAngles testJoints = {0, 45, -60, 15, 0, 0};
    Pose targetPose = forwardKinematics(testJoints, testGeometry);

    printPose(targetPose, "Target pose: ");

    JointSolution solutionUp, solutionDown;
    IKOptions options = DEFAULT_IK_OPTIONS;
    inverseKinematicsBothConfigs(targetPose, testGeometry, testServoConfigs, options,
                                  solutionUp, solutionDown);

    std::cout << "\nElbow-up solution:" << std::endl;
    std::cout << "  Status: "; printStatus(solutionUp.status); std::cout << std::endl;
    std::cout << "  Valid: " << (solutionUp.valid ? "yes" : "no") << std::endl;
    printJoints(solutionUp.angles, "  ");
    std::cout << "  Position error: " << solutionUp.positionError << " mm" << std::endl;
    std::cout << "  Orientation error: " << solutionUp.orientationError << " deg" << std::endl;

    std::cout << "\nElbow-down solution:" << std::endl;
    std::cout << "  Status: "; printStatus(solutionDown.status); std::cout << std::endl;
    std::cout << "  Valid: " << (solutionDown.valid ? "yes" : "no") << std::endl;
    printJoints(solutionDown.angles, "  ");
    std::cout << "  Position error: " << solutionDown.positionError << " mm" << std::endl;
    std::cout << "  Orientation error: " << solutionDown.orientationError << " deg" << std::endl;
}

// =============================================================================
// RANDOM TEST SUITE
// =============================================================================

void runRandomTests(int numTests) {
    std::cout << "\n=== Running " << numTests << " Random Tests (Full Range) ===" << std::endl;

    int passed = 0;
    int failed = 0;
    int singular = 0;
    int unreachable = 0;
    int clamped = 0;
    int okStatus = 0;

    float maxPosErr = 0;
    float maxOriErr = 0;

    for (int i = 0; i < numTests; i++) {
        JointAngles joints = randomJointsWithinLimits();
        TestResult result = runSingleTest(joints);

        if (result.status == IK_OK) okStatus++;
        if (result.status == IK_SINGULAR) singular++;
        if (result.status == IK_UNREACHABLE) unreachable++;
        if (result.status == IK_LIMIT_CLAMPED) clamped++;

        if (result.passed) {
            passed++;
        } else {
            failed++;
        }

        // Only track errors for non-unreachable cases
        if (result.status != IK_UNREACHABLE) {
            if (result.positionError > maxPosErr) maxPosErr = result.positionError;
            if (result.orientationError > maxOriErr) maxOriErr = result.orientationError;
        }
    }

    std::cout << "\nResults (full joint range - includes impossible poses):" << std::endl;
    std::cout << "  Passed (within tolerance): " << passed << " / " << numTests
              << " (" << std::fixed << std::setprecision(1) << (100.0 * passed / numTests) << "%)" << std::endl;
    std::cout << "  IK_OK status: " << okStatus << std::endl;
    std::cout << "  Unreachable (expected): " << unreachable << std::endl;
    std::cout << "  Limit clamped: " << clamped << std::endl;
    std::cout << "  Singular: " << singular << std::endl;
    std::cout << "  Max pos error (reachable): " << maxPosErr << " mm" << std::endl;
    std::cout << "  Max ori error (reachable): " << maxOriErr << " deg" << std::endl;

    // Run IK-friendly tests (configurations that should roundtrip)
    std::cout << "\n=== Running " << numTests << " IK-Friendly Tests ===" << std::endl;

    int passedIK = 0;
    int failedIK = 0;
    int okIK = 0;
    int clampedIK = 0;
    int oriUnreach = 0;
    float maxPosErrIK = 0;
    float maxOriErrIK = 0;
    std::vector<TestResult> failures;

    for (int i = 0; i < numTests; i++) {
        JointAngles joints = randomJointsIKFriendly();
        TestResult result = runSingleTest(joints);

        if (result.status == IK_OK) okIK++;
        if (result.status == IK_LIMIT_CLAMPED) clampedIK++;
        if (result.status == IK_ORIENTATION_UNREACHABLE) oriUnreach++;

        if (result.passed) {
            passedIK++;
        } else {
            failedIK++;
            if (failures.size() < 5) {
                failures.push_back(result);
            }
        }

        if (result.positionError > maxPosErrIK) maxPosErrIK = result.positionError;
        if (result.orientationError > maxOriErrIK) maxOriErrIK = result.orientationError;
    }

    std::cout << "\nResults (IK-friendly configurations):" << std::endl;
    std::cout << "  Passed: " << passedIK << " / " << numTests
              << " (" << std::fixed << std::setprecision(1) << (100.0 * passedIK / numTests) << "%)" << std::endl;
    std::cout << "  IK_OK status: " << okIK << std::endl;
    std::cout << "  Limit clamped: " << clampedIK << std::endl;
    std::cout << "  Orientation unreachable: " << oriUnreach << std::endl;
    std::cout << "  Max position error: " << maxPosErrIK << " mm" << std::endl;
    std::cout << "  Max orientation error: " << maxOriErrIK << " deg" << std::endl;

    if (!failures.empty()) {
        std::cout << "\nIK-friendly test failures:" << std::endl;
        for (size_t i = 0; i < failures.size(); i++) {
            std::cout << "\n  Failure " << (i+1) << ":" << std::endl;
            printJoints(failures[i].originalJoints, "    Original: ");
            printJoints(failures[i].recoveredJoints, "    Recovered: ");
            printPose(failures[i].originalPose, "    Target pose: ");
            std::cout << "    Position error: " << failures[i].positionError << " mm" << std::endl;
            std::cout << "    Orientation error: " << failures[i].orientationError << " deg" << std::endl;
            std::cout << "    Status: "; printStatus(failures[i].status); std::cout << std::endl;
        }
    }
}

// =============================================================================
// PICK AND PLACE EXAMPLE
// =============================================================================

void testPickAndPlace() {
    std::cout << "\n=== Pick and Place Example ===" << std::endl;

    IKOptions options = DEFAULT_IK_OPTIONS;

    // Define waypoints for a pick-and-place operation
    struct Waypoint {
        Vec3 pos;
        float pitch;  // Tool pitch in degrees (0 = horizontal, -90 = pointing down)
        const char* name;
    };

    Waypoint waypoints[] = {
        {{150, 0, 150}, 0, "Approach (high)"},
        {{150, 0, 50}, -45, "Pick position"},
        {{150, 0, 150}, -45, "Lift"},
        {{0, 150, 150}, -45, "Move to place"},
        {{0, 150, 50}, -45, "Place position"},
        {{0, 150, 150}, 0, "Retract"},
    };

    int numWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);

    std::cout << "\nWaypoint IK solutions:" << std::endl;

    for (int i = 0; i < numWaypoints; i++) {
        const Waypoint& wp = waypoints[i];

        // Create pose with desired pitch
        Pose targetPose;
        targetPose.position = wp.pos;

        // Orientation: rotate base toward target, then apply pitch
        float baseAngle = radToDeg(std::atan2(wp.pos.y, wp.pos.x));
        targetPose.orientation = Mat3::rotZ(baseAngle) * Mat3::rotY(wp.pitch);

        JointSolution solution = inverseKinematics(targetPose, testGeometry, testServoConfigs, options);

        std::cout << "\n  " << wp.name << ":" << std::endl;
        std::cout << "    Target: [" << wp.pos.x << ", " << wp.pos.y << ", " << wp.pos.z
                  << "] pitch=" << wp.pitch << std::endl;
        std::cout << "    Status: "; printStatus(solution.status); std::cout << std::endl;
        std::cout << "    Valid: " << (solution.valid ? "yes" : "no") << std::endl;

        if (solution.valid || solution.status == IK_LIMIT_CLAMPED) {
            printJoints(solution.angles, "    Solution: ");

            // Verify with FK
            Pose verify = forwardKinematics(solution.angles, testGeometry);
            std::cout << "    Position error: " << (verify.position - wp.pos).length() << " mm" << std::endl;
        }
    }
}

// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "Robot Arm Kinematics Test Harness" << std::endl;
    std::cout << "========================================" << std::endl;

    // Initialize random seed
    srand(static_cast<unsigned>(time(nullptr)));

    // Initialize servo configs
    initServoConfigs();

    // Print geometry
    std::cout << "\nArm Geometry:" << std::endl;
    std::cout << "  Base height: " << testGeometry.baseHeight << " mm" << std::endl;
    std::cout << "  Upper arm: " << testGeometry.upperArmLength << " mm" << std::endl;
    std::cout << "  Forearm: " << testGeometry.forearmLength << " mm" << std::endl;
    std::cout << "  Wrist: " << testGeometry.wristLength << " mm" << std::endl;
    std::cout << "  Tool: " << testGeometry.toolLength << " mm" << std::endl;

    // Print joint limits
    std::cout << "\nJoint Limits (degrees):" << std::endl;
    const char* jointNames[] = {"Base", "Shoulder", "Elbow", "WristPitch", "WristRoll", "Gripper"};
    for (int i = 0; i < NUM_JOINTS; i++) {
        std::cout << "  " << jointNames[i] << ": ["
                  << testServoConfigs[i].minAngle << ", "
                  << testServoConfigs[i].maxAngle << "]" << std::endl;
    }

    // Run test suites
    testSpecificPoses();
    testSingularities();
    testWorkspaceBoundary();
    testBothElbowConfigs();
    testPickAndPlace();

    // Random tests
    int numTests = DEFAULT_NUM_TESTS;
    if (argc > 1) {
        numTests = atoi(argv[1]);
    }
    runRandomTests(numTests);

    std::cout << "\n========================================" << std::endl;
    std::cout << "Tests complete" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
