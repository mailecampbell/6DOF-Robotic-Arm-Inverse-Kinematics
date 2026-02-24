/**
 * Servo Controller Implementation
 */

#include "servo_controller.h"
#include "arm_math.h"
#include "arm_kinematics.h"

#ifdef ARDUINO
#include <Arduino.h>
#else
// Stub for desktop compilation
unsigned long millis() {
    static unsigned long t = 0;
    return t++;
}
#endif

// ============================================================================
// INTERPOLATION FUNCTIONS
// ============================================================================

float interpolate(float t, InterpolationType type) {
    t = clamp(t, 0.0f, 1.0f);

    switch (type) {
        case INTERP_LINEAR:
            return t;

        case INTERP_CUBIC:
            // Smoothstep: 3t² - 2t³
            return t * t * (3.0f - 2.0f * t);

        case INTERP_QUINTIC:
            // Smootherstep: 6t⁵ - 15t⁴ + 10t³
            return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);

        default:
            return t;
    }
}

JointAngles lerpJoints(const JointAngles& a, const JointAngles& b, float t) {
    JointAngles result;
    result.base = lerp(a.base, b.base, t);
    result.shoulder = lerp(a.shoulder, b.shoulder, t);
    result.elbow = lerp(a.elbow, b.elbow, t);
    result.wristPitch = lerp(a.wristPitch, b.wristPitch, t);
    result.wristRoll = lerp(a.wristRoll, b.wristRoll, t);
    result.gripper = lerp(a.gripper, b.gripper, t);
    return result;
}

// ============================================================================
// SERVO CONTROLLER IMPLEMENTATION
// ============================================================================

void ServoController::begin(const ServoConfig configs[NUM_JOINTS]) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        m_configs[i] = configs[i];
    }
    m_moving = false;

    // Initialize current joints to middle of range
    m_currentJoints.base = 0;
    m_currentJoints.shoulder = 45;
    m_currentJoints.elbow = -45;
    m_currentJoints.wristPitch = 0;
    m_currentJoints.wristRoll = 0;
    m_currentJoints.gripper = 0;
}

void ServoController::attachAll() {
#ifdef ARDUINO
    for (int i = 0; i < NUM_JOINTS; i++) {
        m_servos[i].attach(
            m_configs[i].pin,
            m_configs[i].minPulseUs,
            m_configs[i].maxPulseUs
        );
    }
#endif
}

void ServoController::detachAll() {
#ifdef ARDUINO
    for (int i = 0; i < NUM_JOINTS; i++) {
        m_servos[i].detach();
    }
#endif
}

void ServoController::setJoints(const JointAngles& joints) {
    m_currentJoints = joints;
    for (int i = 0; i < NUM_JOINTS; i++) {
        writeServoAngle(static_cast<JointIndex>(i), joints[i]);
    }
}

void ServoController::moveToJoints(const JointAngles& joints, unsigned int durationMs) {
    startMove(joints, durationMs);
    while (!updateMove()) {
#ifdef ARDUINO
        delay(10);  // Small delay between updates
#endif
    }
}

void ServoController::startMove(const JointAngles& joints, unsigned int durationMs) {
    m_startJoints = m_currentJoints;
    m_targetJoints = joints;
    m_moveDuration = durationMs;
    m_moveStartTime = millis();
    m_moving = true;
}

bool ServoController::updateMove() {
    if (!m_moving) {
        return true;
    }

    unsigned long elapsed = millis() - m_moveStartTime;
    float t = (float)elapsed / (float)m_moveDuration;

    if (t >= 1.0f) {
        // Move complete
        setJoints(m_targetJoints);
        m_moving = false;
        return true;
    }

    // Apply cubic interpolation for smooth motion
    float tSmooth = interpolate(t, INTERP_CUBIC);
    JointAngles interpolated = lerpJoints(m_startJoints, m_targetJoints, tSmooth);
    setJoints(interpolated);

    return false;
}

void ServoController::setJoint(JointIndex joint, float angleDeg) {
    m_currentJoints[joint] = angleDeg;
    writeServoAngle(joint, angleDeg);
}

void ServoController::setGripper(float percent) {
    float angle = gripperPercentToAngle(percent, m_configs[JOINT_GRIPPER]);
    setJoint(JOINT_GRIPPER, angle);
}

float ServoController::jointToServo(JointIndex joint, float jointAngleDeg) const {
    const ServoConfig& cfg = m_configs[joint];

    // Apply inversion first
    float angle = cfg.inverted ? -jointAngleDeg : jointAngleDeg;

    // Apply offset (joint 0° corresponds to servo zeroOffset°)
    angle += cfg.zeroOffset;

    // Clamp to servo range
    return clamp(angle, cfg.minAngle + cfg.zeroOffset, cfg.maxAngle + cfg.zeroOffset);
}

float ServoController::servoToJoint(JointIndex joint, float servoAngleDeg) const {
    const ServoConfig& cfg = m_configs[joint];

    // Remove offset
    float angle = servoAngleDeg - cfg.zeroOffset;

    // Apply inversion
    if (cfg.inverted) {
        angle = -angle;
    }

    return angle;
}

void ServoController::setCalibrationOffset(JointIndex joint, float offsetDeg) {
    m_configs[joint].zeroOffset = offsetDeg;
}

float ServoController::getCalibrationOffset(JointIndex joint) const {
    return m_configs[joint].zeroOffset;
}

void ServoController::setInverted(JointIndex joint, bool inverted) {
    m_configs[joint].inverted = inverted;
}

void ServoController::goHome(unsigned int durationMs) {
    JointAngles home;
    home.base = 0;
    home.shoulder = 0;
    home.elbow = 0;
    home.wristPitch = 0;
    home.wristRoll = 0;
    home.gripper = 0;
    moveToJoints(home, durationMs);
}

void ServoController::park(unsigned int durationMs) {
    // Park position: arm folded, gripper closed
    JointAngles park;
    park.base = 0;
    park.shoulder = 90;      // Shoulder up
    park.elbow = -90;        // Elbow folded
    park.wristPitch = 0;
    park.wristRoll = 0;
    park.gripper = 0;        // Closed
    moveToJoints(park, durationMs);
}

void ServoController::writeServoAngle(JointIndex joint, float jointAngleDeg) {
#ifdef ARDUINO
    // Convert joint angle to servo angle
    float servoAngle = jointToServo(joint, jointAngleDeg);

    // Write to servo
    m_servos[joint].write((int)servoAngle);
#else
    // Desktop: no-op or could log
    (void)joint;
    (void)jointAngleDeg;
#endif
}
