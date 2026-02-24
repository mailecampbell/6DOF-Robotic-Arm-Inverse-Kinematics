/**
 * Servo Controller for Arduino Robot Arm
 *
 * This module provides:
 * - Servo initialization and calibration
 * - Joint-to-servo angle conversion (with offsets and inversion)
 * - Smooth interpolated movements
 * - Direct servo pulse control for precision
 *
 * Compatible with Arduino Servo library and similar PWM servo controllers.
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "arm_config.h"

#ifdef ARDUINO
#include <Arduino.h>
#include <Servo.h>
#endif

// ============================================================================
// SERVO CONTROLLER CLASS
// ============================================================================

class ServoController {
public:
    /**
     * Initialize servo controller with configuration.
     * @param configs Array of servo configurations
     */
    void begin(const ServoConfig configs[NUM_JOINTS]);

    /**
     * Attach all servos to their configured pins.
     * Call this in Arduino setup().
     */
    void attachAll();

    /**
     * Detach all servos (releases PWM control).
     */
    void detachAll();

    /**
     * Move all joints to specified angles immediately.
     * Angles are in joint space (kinematics convention).
     * @param joints Joint angles in degrees
     */
    void setJoints(const JointAngles& joints);

    /**
     * Move all joints to specified angles with smooth interpolation.
     * Blocking call - returns when movement is complete.
     * @param joints Target joint angles in degrees
     * @param durationMs Movement duration in milliseconds
     */
    void moveToJoints(const JointAngles& joints, unsigned int durationMs);

    /**
     * Start an interpolated move (non-blocking).
     * Call updateMove() repeatedly until it returns true.
     * @param joints Target joint angles in degrees
     * @param durationMs Movement duration in milliseconds
     */
    void startMove(const JointAngles& joints, unsigned int durationMs);

    /**
     * Update ongoing interpolated move.
     * @return true when move is complete
     */
    bool updateMove();

    /**
     * Check if a move is in progress.
     */
    bool isMoving() const { return m_moving; }

    /**
     * Stop any ongoing movement immediately.
     */
    void stopMove() { m_moving = false; }

    /**
     * Get current joint angles (last commanded values).
     */
    JointAngles getCurrentJoints() const { return m_currentJoints; }

    /**
     * Set a single joint angle.
     * @param joint Joint index (JOINT_BASE, etc.)
     * @param angleDeg Joint angle in degrees
     */
    void setJoint(JointIndex joint, float angleDeg);

    /**
     * Set gripper opening percentage.
     * @param percent 0 = closed, 100 = fully open
     */
    void setGripper(float percent);

    /**
     * Convert joint angle to servo angle.
     * Applies offset and inversion.
     */
    float jointToServo(JointIndex joint, float jointAngleDeg) const;

    /**
     * Convert servo angle to joint angle.
     * Inverse of jointToServo.
     */
    float servoToJoint(JointIndex joint, float servoAngleDeg) const;

    /**
     * Set calibration offset for a joint.
     * Use this during calibration to adjust servo zero position.
     */
    void setCalibrationOffset(JointIndex joint, float offsetDeg);

    /**
     * Get calibration offset for a joint.
     */
    float getCalibrationOffset(JointIndex joint) const;

    /**
     * Set whether a joint servo direction is inverted.
     */
    void setInverted(JointIndex joint, bool inverted);

    /**
     * Move to home position (all joints at 0 degrees).
     */
    void goHome(unsigned int durationMs = 1000);

    /**
     * Move to a safe "parked" position for power-off.
     */
    void park(unsigned int durationMs = 1000);

private:
    ServoConfig m_configs[NUM_JOINTS];
    JointAngles m_currentJoints;
    JointAngles m_startJoints;
    JointAngles m_targetJoints;

    bool m_moving;
    unsigned long m_moveStartTime;
    unsigned int m_moveDuration;

#ifdef ARDUINO
    Servo m_servos[NUM_JOINTS];
#endif

    void writeServoAngle(JointIndex joint, float jointAngleDeg);
};

// ============================================================================
// TRAJECTORY INTERPOLATION
// ============================================================================

enum InterpolationType {
    INTERP_LINEAR,      // Linear interpolation (constant velocity)
    INTERP_CUBIC,       // Cubic spline (smooth acceleration)
    INTERP_QUINTIC      // Quintic spline (smooth jerk)
};

/**
 * Compute interpolation factor based on type.
 * @param t Normalized time [0, 1]
 * @param type Interpolation type
 * @return Interpolated factor [0, 1]
 */
float interpolate(float t, InterpolationType type);

/**
 * Linear interpolation between two values.
 */
inline float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

/**
 * Interpolate between two joint angle sets.
 */
JointAngles lerpJoints(const JointAngles& a, const JointAngles& b, float t);

#endif // SERVO_CONTROLLER_H
