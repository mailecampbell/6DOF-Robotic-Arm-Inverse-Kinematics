/**
 * Robot Arm Inverse Kinematics - Arduino Sketch
 *
 * For the "3D Printed Arduino Based Robotic Arm" (BasementMaker)
 *
 * Features:
 * - Inverse kinematics to compute joint angles from XYZ position
 * - Forward kinematics to verify positions
 * - Smooth interpolated servo movements
 * - Serial command interface for testing
 *
 * Pin Configuration (default):
 *   Base:        D3
 *   Shoulder:    D5
 *   Elbow:       D6
 *   Wrist Pitch: D11
 *   Wrist Roll:  D10
 *   Gripper:     D9
 *
 * Serial Commands (115200 baud):
 *   G X Y Z [P R] - Go to position X,Y,Z with optional pitch P and roll R
 *   J B S E P R G - Set joints directly (base, shoulder, elbow, wristPitch, wristRoll, gripper)
 *   H             - Go to home position
 *   K             - Go to park position
 *   W             - Report workspace bounds
 *   P             - Report current position (FK)
 *   C             - Report current joint angles
 *   ?             - Help
 *
 * Example:
 *   G 150 0 100     -> Move to X=150mm, Y=0mm, Z=100mm (horizontal tool)
 *   G 150 0 100 -45 -> Move with pitch=-45 degrees (tool pointing down)
 *   J 0 45 -45 0 0 0 -> Set joints directly
 *   H               -> Go home
 */

#include <Servo.h>

// =============================================================================
// HELPER: Parse space-separated floats from a String
// =============================================================================

// Parse up to maxCount floats from a space-separated string
// Returns the number of floats successfully parsed
int parseFloats(const String& str, float* values, int maxCount) {
  int count = 0;
  int start = 0;
  int len = str.length();

  // Skip leading whitespace
  while (start < len && (str.charAt(start) == ' ' || str.charAt(start) == '\t')) {
    start++;
  }

  while (start < len && count < maxCount) {
    // Find end of this number
    int end = start;
    while (end < len && str.charAt(end) != ' ' && str.charAt(end) != '\t') {
      end++;
    }

    if (end > start) {
      // Extract substring and convert to float
      String numStr = str.substring(start, end);
      values[count] = numStr.toFloat();
      count++;
    }

    // Skip whitespace to next number
    start = end;
    while (start < len && (str.charAt(start) == ' ' || str.charAt(start) == '\t')) {
      start++;
    }
  }

  return count;
}

// =============================================================================
// CONFIGURATION - ADJUST THESE FOR YOUR ARM
// =============================================================================

// Link lengths in millimeters - MEASURE YOUR ARM!
// See README for measurement instructions
const float BASE_HEIGHT = 66.76;       // Height from table to shoulder axis
const float SHOULDER_OFFSET_X = 0.0;   // Horizontal offset (usually 0)
const float UPPER_ARM_LENGTH = 31.1;   // Shoulder to elbow
const float FOREARM_LENGTH = 33.0;     // Elbow to wrist
const float WRIST_LENGTH = 60.0;       // Wrist pitch to wrist roll axis
const float TOOL_LENGTH = 16.4;        // Wrist roll to gripper tip

// Servo pin assignments
const int PIN_BASE = 3;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW = 6;
const int PIN_WRIST_PITCH = 11;
const int PIN_WRIST_ROLL = 10;
const int PIN_GRIPPER = 9;

// Joint limits in degrees (joint space, not servo space)
const float BASE_MIN = -90, BASE_MAX = 90;
const float SHOULDER_MIN = 0, SHOULDER_MAX = 120;  // Tighter limits to prevent base collision
const float ELBOW_MIN = -120, ELBOW_MAX = 0;       // Tighter to prevent over-extension
const float WRIST_PITCH_MIN = -90, WRIST_PITCH_MAX = 90;
const float WRIST_ROLL_MIN = -90, WRIST_ROLL_MAX = 90;
const float GRIPPER_MIN = 0, GRIPPER_MAX = 180;

// Maximum joint speed (degrees per second) - slower = safer
const float MAX_JOINT_SPEED = 30.0;  // Max 30 degrees per second per joint

// Servo calibration offsets (joint 0° = servo offset°)
// Adjust these during calibration
float offsetBase = 90;
float offsetShoulder = 90;
float offsetElbow = 90;
float offsetWristPitch = 90;
float offsetWristRoll = 90;
float offsetGripper = 0;

// Servo direction (true if servo moves opposite to expected)
const bool invertBase = false;
const bool invertShoulder = false;
const bool invertElbow = true;    // Often inverted due to mounting
const bool invertWristPitch = false;
const bool invertWristRoll = false;
const bool invertGripper = false;

// =============================================================================
// CONSTANTS
// =============================================================================

const float PI_F = 3.14159265;
const float D2R = PI_F / 180.0;  // Degrees to radians
const float R2D = 180.0 / PI_F;  // Radians to degrees
const float EPSILON = 0.0001;

// =============================================================================
// GLOBALS
// =============================================================================

Servo servoBase, servoShoulder, servoElbow;
Servo servoWristPitch, servoWristRoll, servoGripper;

// Current joint angles (in degrees)
float jointBase = 0, jointShoulder = 45, jointElbow = -45;
float jointWristPitch = 0, jointWristRoll = 0, jointGripper = 0;

// Move interpolation
bool moving = false;
float startJoints[6], targetJoints[6];
unsigned long moveStart, moveDuration;

// =============================================================================
// MATH UTILITIES
// =============================================================================

float clampF(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float safeAcos(float x) {
  return acos(clampF(x, -1.0, 1.0));
}

// Cubic smoothstep interpolation
float smoothstep(float t) {
  t = clampF(t, 0, 1);
  return t * t * (3.0 - 2.0 * t);
}

// =============================================================================
// SERVO CONTROL
// =============================================================================

float jointToServo(int joint, float angleDeg) {
  float offset, angle;
  bool invert;

  switch (joint) {
    case 0: offset = offsetBase; invert = invertBase; break;
    case 1: offset = offsetShoulder; invert = invertShoulder; break;
    case 2: offset = offsetElbow; invert = invertElbow; break;
    case 3: offset = offsetWristPitch; invert = invertWristPitch; break;
    case 4: offset = offsetWristRoll; invert = invertWristRoll; break;
    case 5: offset = offsetGripper; invert = invertGripper; break;
    default: return 90;
  }

  angle = invert ? -angleDeg : angleDeg;
  return angle + offset;
}

void writeJoint(int joint, float angleDeg) {
  int servoAngle = (int)jointToServo(joint, angleDeg);
  servoAngle = constrain(servoAngle, 0, 180);

  switch (joint) {
    case 0: servoBase.write(servoAngle); jointBase = angleDeg; break;
    case 1: servoShoulder.write(servoAngle); jointShoulder = angleDeg; break;
    case 2: servoElbow.write(servoAngle); jointElbow = angleDeg; break;
    case 3: servoWristPitch.write(servoAngle); jointWristPitch = angleDeg; break;
    case 4: servoWristRoll.write(servoAngle); jointWristRoll = angleDeg; break;
    case 5: servoGripper.write(servoAngle); jointGripper = angleDeg; break;
  }
}

void writeAllJoints(float b, float s, float e, float wp, float wr, float g) {
  writeJoint(0, b);
  writeJoint(1, s);
  writeJoint(2, e);
  writeJoint(3, wp);
  writeJoint(4, wr);
  writeJoint(5, g);
}

void clampJoints(float* j) {
  j[0] = clampF(j[0], BASE_MIN, BASE_MAX);
  j[1] = clampF(j[1], SHOULDER_MIN, SHOULDER_MAX);
  j[2] = clampF(j[2], ELBOW_MIN, ELBOW_MAX);
  j[3] = clampF(j[3], WRIST_PITCH_MIN, WRIST_PITCH_MAX);
  j[4] = clampF(j[4], WRIST_ROLL_MIN, WRIST_ROLL_MAX);
  j[5] = clampF(j[5], GRIPPER_MIN, GRIPPER_MAX);
}

// =============================================================================
// FORWARD KINEMATICS
// =============================================================================

void forwardKinematics(float b, float s, float e, float wp, float wr,
                       float* x, float* y, float* z, float* pitch) {
  float baseRad = b * D2R;
  float shoulderRad = s * D2R;
  float elbowRad = e * D2R;
  float wristPitchRad = wp * D2R;

  // Cumulative pitch angles
  float theta1 = shoulderRad;
  float theta2 = shoulderRad + elbowRad;
  float theta3 = shoulderRad + elbowRad + wristPitchRad;

  // Position in arm plane (before base rotation)
  float sx = SHOULDER_OFFSET_X;
  float sz = BASE_HEIGHT;

  // Elbow position
  float ex = sx + UPPER_ARM_LENGTH * cos(theta1);
  float ez = sz + UPPER_ARM_LENGTH * sin(theta1);

  // Wrist position
  float wpx = ex + FOREARM_LENGTH * cos(theta2);
  float wpz = ez + FOREARM_LENGTH * sin(theta2);

  // Tool tip position
  float wristToTool = WRIST_LENGTH + TOOL_LENGTH;
  float toolx = wpx + wristToTool * cos(theta3);
  float toolz = wpz + wristToTool * sin(theta3);

  // Apply base rotation
  *x = toolx * cos(baseRad);
  *y = toolx * sin(baseRad);
  *z = toolz;
  *pitch = (s + e + wp);  // Total pitch in degrees
}

// =============================================================================
// INVERSE KINEMATICS
// =============================================================================

// Status codes
#define IK_OK 0
#define IK_UNREACHABLE 1
#define IK_SINGULAR 2
#define IK_LIMIT_CLAMPED 3

int inverseKinematics(float targetX, float targetY, float targetZ, float targetPitch,
                      float* outBase, float* outShoulder, float* outElbow, float* outWristPitch) {
  int status = IK_OK;

  // Convert target pitch to radians
  float pitchRad = targetPitch * D2R;

  // Step 1: Compute wrist center by subtracting tool length along tool direction
  float wristToTool = WRIST_LENGTH + TOOL_LENGTH;
  float wcX = targetX - wristToTool * cos(pitchRad) * cos(atan2(targetY, targetX));
  float wcY = targetY - wristToTool * cos(pitchRad) * sin(atan2(targetY, targetX));
  float wcZ = targetZ - wristToTool * sin(pitchRad);

  // Step 2: Compute base angle
  float wristXY = sqrt(wcX * wcX + wcY * wcY);
  float baseAngle;

  if (wristXY < EPSILON) {
    // Singular case: wrist directly above base
    baseAngle = jointBase;  // Keep current base angle
    status = IK_SINGULAR;
  } else {
    baseAngle = atan2(wcY, wcX) * R2D;
  }

  // Step 3: Solve 2-link IK in arm plane
  float r = wristXY - SHOULDER_OFFSET_X;  // Radial distance
  float h = wcZ - BASE_HEIGHT;            // Height relative to shoulder
  float d = sqrt(r * r + h * h);          // Distance from shoulder to wrist

  float L1 = UPPER_ARM_LENGTH;
  float L2 = FOREARM_LENGTH;
  float maxReach = L1 + L2;
  float minReach = abs(L1 - L2);

  // Check reachability
  if (d > maxReach) {
    // Scale to max reach
    float scale = maxReach / d;
    r *= scale;
    h *= scale;
    d = maxReach;
    status = IK_UNREACHABLE;
  } else if (d < minReach) {
    float scale = minReach / d;
    r *= scale;
    h *= scale;
    d = minReach;
    status = IK_UNREACHABLE;
  }

  // Law of cosines for elbow angle
  float cosElbow = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  cosElbow = clampF(cosElbow, -1.0, 1.0);
  float elbowAngle = -safeAcos(cosElbow);  // Negative for elbow-up

  // Shoulder angle
  float alpha = atan2(h, r);
  float cosBeta = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d);
  cosBeta = clampF(cosBeta, -1.0, 1.0);
  float beta = safeAcos(cosBeta);

  float shoulderAngle = (alpha + beta) * R2D;  // Elbow-up config
  elbowAngle = elbowAngle * R2D;

  // Wrist pitch to achieve desired tool pitch
  float wristPitchAngle = targetPitch - shoulderAngle - elbowAngle;

  // Output
  *outBase = baseAngle;
  *outShoulder = shoulderAngle;
  *outElbow = elbowAngle;
  *outWristPitch = wristPitchAngle;

  return status;
}

// =============================================================================
// SMOOTH MOVEMENT
// =============================================================================

void startMove(float b, float s, float e, float wp, float wr, float g, unsigned long minDuration) {
  startJoints[0] = jointBase;
  startJoints[1] = jointShoulder;
  startJoints[2] = jointElbow;
  startJoints[3] = jointWristPitch;
  startJoints[4] = jointWristRoll;
  startJoints[5] = jointGripper;

  targetJoints[0] = b;
  targetJoints[1] = s;
  targetJoints[2] = e;
  targetJoints[3] = wp;
  targetJoints[4] = wr;
  targetJoints[5] = g;

  clampJoints(targetJoints);

  // Calculate duration based on largest joint change and max speed
  float maxChange = 0;
  for (int i = 0; i < 6; i++) {
    float change = abs(targetJoints[i] - startJoints[i]);
    if (change > maxChange) maxChange = change;
  }

  // Duration = degrees / (degrees per second) * 1000 for milliseconds
  unsigned long speedBasedDuration = (unsigned long)(maxChange / MAX_JOINT_SPEED * 1000.0);

  // Use the longer of minimum duration or speed-based duration
  moveDuration = speedBasedDuration > minDuration ? speedBasedDuration : minDuration;

  // Minimum 500ms for any move
  if (moveDuration < 500) moveDuration = 500;

  moveStart = millis();
  moving = true;
}

void updateMove() {
  if (!moving) return;

  unsigned long elapsed = millis() - moveStart;
  float t = (float)elapsed / (float)moveDuration;

  if (t >= 1.0) {
    // Move complete
    writeAllJoints(targetJoints[0], targetJoints[1], targetJoints[2],
                   targetJoints[3], targetJoints[4], targetJoints[5]);
    moving = false;
    Serial.println("OK Move complete");
    return;
  }

  // Smooth interpolation
  float tSmooth = smoothstep(t);
  float joints[6];
  for (int i = 0; i < 6; i++) {
    joints[i] = startJoints[i] + (targetJoints[i] - startJoints[i]) * tSmooth;
  }

  writeAllJoints(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
}

// =============================================================================
// SERIAL COMMANDS
// =============================================================================

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = cmd.charAt(0);
  String args = cmd.substring(1);
  args.trim();

  switch (c) {
    case 'G':
    case 'g': {
      // Go to XYZ position
      float vals[5] = {0, 0, 0, 0, 0};  // x, y, z, pitch, roll
      int n = parseFloats(args, vals, 5);

      if (n < 3) {
        Serial.println("ERR Usage: G X Y Z [pitch] [roll]");
        return;
      }

      float x = vals[0], y = vals[1], z = vals[2];
      float pitch = vals[3], roll = vals[4];

      float b, s, e, wp;
      int status = inverseKinematics(x, y, z, pitch, &b, &s, &e, &wp);

      if (status == IK_UNREACHABLE) {
        Serial.println("WARN Position unreachable, moving to closest");
      } else if (status == IK_SINGULAR) {
        Serial.println("WARN Singular configuration");
      }

      startMove(b, s, e, wp, roll, jointGripper, 1000);
      Serial.print("OK Moving to ");
      Serial.print(x); Serial.print(" ");
      Serial.print(y); Serial.print(" ");
      Serial.println(z);
      break;
    }

    case 'J':
    case 'j': {
      // Set joints directly
      float vals[6] = {0, 0, 0, 0, 0, 0};
      int n = parseFloats(args, vals, 6);

      if (n < 6) {
        Serial.println("ERR Usage: J base shoulder elbow wristPitch wristRoll gripper");
        return;
      }

      float b = vals[0], s = vals[1], e = vals[2];
      float wp = vals[3], wr = vals[4], g = vals[5];

      startMove(b, s, e, wp, wr, g, 1000);
      Serial.println("OK Moving to joint position");
      break;
    }

    case 'H':
    case 'h':
      // Home position
      startMove(0, 0, 0, 0, 0, 0, 1500);
      Serial.println("OK Moving to home");
      break;

    case 'K':
    case 'k':
      // Park position (folded)
      startMove(0, 90, -90, 0, 0, 0, 1500);
      Serial.println("OK Parking");
      break;

    case 'P':
    case 'p': {
      // Print current position (FK)
      float x, y, z, pitch;
      forwardKinematics(jointBase, jointShoulder, jointElbow, jointWristPitch, jointWristRoll,
                        &x, &y, &z, &pitch);
      Serial.print("Position: X=");
      Serial.print(x, 1);
      Serial.print(" Y=");
      Serial.print(y, 1);
      Serial.print(" Z=");
      Serial.print(z, 1);
      Serial.print(" Pitch=");
      Serial.println(pitch, 1);
      break;
    }

    case 'C':
    case 'c':
      // Print current joints
      Serial.print("Joints: Base=");
      Serial.print(jointBase, 1);
      Serial.print(" Shoulder=");
      Serial.print(jointShoulder, 1);
      Serial.print(" Elbow=");
      Serial.print(jointElbow, 1);
      Serial.print(" WristPitch=");
      Serial.print(jointWristPitch, 1);
      Serial.print(" WristRoll=");
      Serial.print(jointWristRoll, 1);
      Serial.print(" Gripper=");
      Serial.println(jointGripper, 1);
      break;

    case 'W':
    case 'w': {
      // Workspace bounds
      float maxR = UPPER_ARM_LENGTH + FOREARM_LENGTH + WRIST_LENGTH + TOOL_LENGTH;
      float minR = abs(UPPER_ARM_LENGTH - FOREARM_LENGTH);
      float maxZ = BASE_HEIGHT + maxR;
      float minZ = BASE_HEIGHT - maxR;
      if (minZ < 0) minZ = 0;

      Serial.print("Workspace: MaxRadius=");
      Serial.print(maxR, 0);
      Serial.print(" MinRadius=");
      Serial.print(minR, 0);
      Serial.print(" MaxZ=");
      Serial.print(maxZ, 0);
      Serial.print(" MinZ=");
      Serial.println(minZ, 0);
      break;
    }

    case 'O':
    case 'o': {
      // Open gripper
      startMove(jointBase, jointShoulder, jointElbow, jointWristPitch, jointWristRoll, 90, 500);
      Serial.println("OK Opening gripper");
      break;
    }

    case 'L':
    case 'l': {
      // Close gripper
      startMove(jointBase, jointShoulder, jointElbow, jointWristPitch, jointWristRoll, 0, 500);
      Serial.println("OK Closing gripper");
      break;
    }

    case '?':
      Serial.println("Commands:");
      Serial.println("  G X Y Z [pitch] [roll] - Move to position");
      Serial.println("  J b s e wp wr g - Set joints");
      Serial.println("  H - Home position");
      Serial.println("  K - Park (fold)");
      Serial.println("  P - Show position (FK)");
      Serial.println("  C - Show joint angles");
      Serial.println("  W - Show workspace");
      Serial.println("  O - Open gripper");
      Serial.println("  L - Close gripper");
      Serial.println("  ? - This help");
      break;

    default:
      Serial.print("ERR Unknown command: ");
      Serial.println(c);
  }
}

// =============================================================================
// SETUP AND LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Robot Arm IK Controller");
  Serial.println("Type ? for help");

  // Attach servos
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);
  servoWristPitch.attach(PIN_WRIST_PITCH);
  servoWristRoll.attach(PIN_WRIST_ROLL);
  servoGripper.attach(PIN_GRIPPER);

  // Move to initial position slowly
  delay(500);
  startMove(0, 45, -45, 0, 0, 0, 2000);
}

void loop() {
  // Update any ongoing movement
  updateMove();

  // Process serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}
