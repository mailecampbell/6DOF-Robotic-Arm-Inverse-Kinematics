/**
 * Math utilities for robot arm kinematics
 */

#ifndef ARM_MATH_H
#define ARM_MATH_H

#include "arm_config.h"
#include <cmath>

// ============================================================================
// CONSTANTS
// ============================================================================

constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float EPSILON = 1e-6f;

// ============================================================================
// ANGLE UTILITIES
// ============================================================================

inline float degToRad(float deg) { return deg * DEG_TO_RAD; }
inline float radToDeg(float rad) { return rad * RAD_TO_DEG; }

// Normalize angle to [-180, 180] degrees
inline float normalizeAngleDeg(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// Normalize angle to [-PI, PI] radians
inline float normalizeAngleRad(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// Clamp value to range
inline float clamp(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// Safe atan2 that handles zero case
inline float safeAtan2(float y, float x) {
    if (std::abs(x) < EPSILON && std::abs(y) < EPSILON) {
        return 0.0f;  // Return 0 for origin (ambiguous case)
    }
    return std::atan2(y, x);
}

// Safe acos that clamps input to [-1, 1]
inline float safeAcos(float x) {
    return std::acos(clamp(x, -1.0f, 1.0f));
}

// Safe asin that clamps input to [-1, 1]
inline float safeAsin(float x) {
    return std::asin(clamp(x, -1.0f, 1.0f));
}

// Check if two floats are approximately equal
inline bool approxEqual(float a, float b, float tolerance = EPSILON) {
    return std::abs(a - b) < tolerance;
}

// ============================================================================
// VEC3 IMPLEMENTATION
// ============================================================================

inline float Vec3::length() const {
    return std::sqrt(x*x + y*y + z*z);
}

inline Vec3 Vec3::normalized() const {
    float len = length();
    if (len < EPSILON) return Vec3(0, 0, 0);
    return Vec3(x/len, y/len, z/len);
}

inline Vec3 Vec3::cross(const Vec3& v) const {
    return Vec3(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    );
}

// ============================================================================
// MAT3 IMPLEMENTATION
// ============================================================================

inline Mat3::Mat3() {
    // Initialize as identity matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

inline Mat3 Mat3::rotX(float angleDeg) {
    float c = std::cos(degToRad(angleDeg));
    float s = std::sin(degToRad(angleDeg));
    Mat3 r;
    r.m[0][0] = 1; r.m[0][1] = 0; r.m[0][2] = 0;
    r.m[1][0] = 0; r.m[1][1] = c; r.m[1][2] = -s;
    r.m[2][0] = 0; r.m[2][1] = s; r.m[2][2] = c;
    return r;
}

inline Mat3 Mat3::rotY(float angleDeg) {
    float c = std::cos(degToRad(angleDeg));
    float s = std::sin(degToRad(angleDeg));
    Mat3 r;
    r.m[0][0] = c;  r.m[0][1] = 0; r.m[0][2] = s;
    r.m[1][0] = 0;  r.m[1][1] = 1; r.m[1][2] = 0;
    r.m[2][0] = -s; r.m[2][1] = 0; r.m[2][2] = c;
    return r;
}

inline Mat3 Mat3::rotZ(float angleDeg) {
    float c = std::cos(degToRad(angleDeg));
    float s = std::sin(degToRad(angleDeg));
    Mat3 r;
    r.m[0][0] = c; r.m[0][1] = -s; r.m[0][2] = 0;
    r.m[1][0] = s; r.m[1][1] = c;  r.m[1][2] = 0;
    r.m[2][0] = 0; r.m[2][1] = 0;  r.m[2][2] = 1;
    return r;
}

inline Mat3 Mat3::fromRPY(const RPY& rpy) {
    // Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // This applies roll first, then pitch, then yaw
    return rotZ(rpy.yaw) * rotY(rpy.pitch) * rotX(rpy.roll);
}

inline Mat3 Mat3::operator*(const Mat3& other) const {
    Mat3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                result.m[i][j] += m[i][k] * other.m[k][j];
            }
        }
    }
    return result;
}

inline Vec3 Mat3::operator*(const Vec3& v) const {
    return Vec3(
        m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
        m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
        m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
    );
}

inline Mat3 Mat3::transpose() const {
    Mat3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = m[j][i];
        }
    }
    return result;
}

inline Vec3 Mat3::col(int i) const {
    return Vec3(m[0][i], m[1][i], m[2][i]);
}

inline RPY Mat3::toRPY() const {
    // Extract RPY from rotation matrix
    // Assuming R = Rz(yaw) * Ry(pitch) * Rx(roll)
    RPY rpy;

    // Check for gimbal lock (pitch = +/- 90 degrees)
    if (std::abs(m[2][0]) > 1.0f - EPSILON) {
        // Gimbal lock case
        rpy.yaw = 0;  // Set yaw to 0 arbitrarily
        if (m[2][0] < 0) {
            // pitch = 90 degrees
            rpy.pitch = 90.0f;
            rpy.roll = radToDeg(std::atan2(m[0][1], m[0][2]));
        } else {
            // pitch = -90 degrees
            rpy.pitch = -90.0f;
            rpy.roll = radToDeg(std::atan2(-m[0][1], -m[0][2]));
        }
    } else {
        // Normal case
        rpy.pitch = radToDeg(-std::asin(m[2][0]));
        rpy.roll = radToDeg(std::atan2(m[2][1], m[2][2]));
        rpy.yaw = radToDeg(std::atan2(m[1][0], m[0][0]));
    }

    return rpy;
}

#endif // ARM_MATH_H
