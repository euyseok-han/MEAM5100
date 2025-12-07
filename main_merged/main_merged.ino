/*
 * MEAM5100 Unified Control - main_merged.ino
 *
 * Modes:
 *  - MODE_MANUAL: speed + steering with PID speed control (web /setspeed)
 *  - MODE_WALL:   wall-following with ToF sensors (currently commented out)
 *  - MODE_VIVE:   Vive-based navigation + queued BFS route following
 *
 * Notes on pins (ESP32-S3 defaults used in this project):
 *  - I2C remains on SDA=11, SCL=12
 *  - Vive pins: 5 (left) and 4 (right)
 *  - RIGHT_ENCODER_B kept at 16
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <vector>
#include <queue>
#include <algorithm>
#include <math.h>

#include "website.h"      // Dashboard UI
#include "vive510.h"      // Vive tracker driver

// ==================== PIN DEFINITIONS (ESP32-S3) ====================

#define LEFT_MOTOR         0   // Left motor identifier
#define RIGHT_MOTOR        1   // Right motor identifier

// Motor 1 (Left Wheel)
#define RIGHT_ENCODER_A          1   // Left encoder channel A
#define RIGHT_ENCODER_B          2   // Left encoder channel B
#define RIGHT_MOTOR_RPWM         18  // Left motor RPWM = Forward direction
#define RIGHT_MOTOR_LPWM         17  // Left motor LPWM = Reverse direction

// Motor 2 (Right Wheel)
#define ENCODER_A    15  // Right encoder channel A
#define ENCODER_B    16  // Right encoder channel B
#define MOTOR_RPWM   41  // Right motor RPWM = Forward direction
#define MOTOR_LPWM   42  // Right motor LPWM = Reverse direction

// I2C Sensors (TOF + MPU6050)
#define I2C_SDA            11
#define I2C_SCL            12

// ToF XSHUT pins (for power/reset control)
#define TOF_XSHUT_FRONT     14  // VL53L1X
#define TOF_XSHUT_RIGHT     13  // VL53L0X (front-right)
#define TOF_XSHUT_RIGHT2    21  // VL53L0X (back-right)

// Vive tracker pins
// Now used as LEFT and RIGHT Vive trackers (Option B)
#define VIVE_LEFT_PIN     4    // left Vive
#define VIVE_RIGHT_PIN      5    // right Vive

// ==================== WIFI ====================
const char* ssid = "TP-Link_8A8C";
const char* password = "12488674";

WebServer server(80);

// ==================== CONTROL CONSTANTS ====================
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;        // 8-bit resolution (0-255)
const float GOAL_REACHED_THRESHOLD = 400.0f;  // mm radius to consider waypoint reached

// ==================== MOTOR & ENCODER VARIABLES ====================
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long rightLastEncoderCount = 0;

// Speed calculation (RPM)
float currentSpeed = 0;        // Left RPM
float rightCurrentSpeed = 0;   // Right RPM
float targetSpeed = 0;         // Left target RPM
float rightTargetSpeed = 0;    // Right target RPM

// Speed + steering inputs (manual)
float baseSpeed = 0;           // Base forward/backward speed
float steeringValue = 0;       // Steering amount (negative=left, positive=right)

// ==================== PID PARAMETERS ====================
struct PIDController {
  float Kp = 0.3;
  float Ki = 1.5;
  float Kd = 0.0;

  float error = 0;
  float lastError = 0;
  float integral = 0;
  float derivative = 0;
  float output = 0;

  float integralMax = 100;
  float integralMin = -100;
};

PIDController leftPID;
PIDController rightPID;

// ==================== TOF SENSORS (CURRENTLY UNUSED) ====================
Adafruit_VL53L1X frontTOF = Adafruit_VL53L1X();
Adafruit_VL53L0X rightTOF = Adafruit_VL53L0X();
Adafruit_VL53L0X right2TOF = Adafruit_VL53L0X();

int frontDistance = 0;          // mm
int rightDistance1 = 0;         // mm (front-right)
int rightDistance2 = 0;         // mm (back-right)

// ==================== MPU6050 ====================
Adafruit_MPU6050 mpu;
float currentAngle = 0;         // degrees
float gyroZOffset = 0;          // rad/s offset converted later
unsigned long lastGyroTime = 0;

// ==================== WALL FOLLOWING (COMMENTED FEATURE) ====================
bool wallFollowMode = false;
int frontGoalDistance = 150;    // mm
int rightGoalDistance1 = 100;   // mm
int rightGoalDistance2 = 100;   // mm
float wallFollowSpeed = 40;     // RPM base forward

float lastDistError = 0;
float wallFollowKp = 1.5;
float wallFollowKd = 0.8;

enum RobotState {
  STATE_IDLE,
  STATE_WALL_FOLLOW
};

RobotState currentState = STATE_IDLE;

// ==================== CONTROL MODE ====================
enum ControlMode {
  MODE_MANUAL = 0,
  MODE_WALL   = 1,
  MODE_VIVE   = 2
};

ControlMode controlMode = MODE_MANUAL;

// ==================== VIVE (dual sensors, used as LEFT + RIGHT) ====================
Vive510 viveLeft(VIVE_LEFT_PIN);   // right Vive tracker
Vive510 viveRight(VIVE_RIGHT_PIN);     // left Vive tracker

uint16_t lx, ly, rx, ry;
uint16_t lx0, ly0, lx1, ly1, lx2, ly2;
uint16_t rx0, ry0, rx1, ry1, rx2, ry2;

bool leftValid = false;
bool rightValid  = false;

float robotX = 0, robotY = 0;
float robotHeading = 0;         // radians

int viveTargetX = 0;
int viveTargetY = 0;
int viveDirectionMode = 0;      // kept but not used in new logic

// ==================== TIMING ====================
unsigned long lastControlUpdate = 0;
unsigned long lastSpeedCalc = 0;
unsigned long lastTOFRead = 0;
unsigned long lastPrint = 0;
unsigned long lastVive = 0;


const unsigned long CONTROL_PERIOD     = 50;   // ms
const unsigned long SPEED_CALC_PERIOD  = 100;  // ms
const unsigned long TOF_READ_PERIOD    = 50;   // ms
const uint16_t      Print_PERIOD       = 500;  // ms
const uint16_t      Vive_PERIOD = 150; // ms

// ==================== GRAPH + BFS ROUTE FINDING ====================
class Node {
public:
  int x, y;               // Vive coordinates (mm, or Vive units)
  std::vector<int> neighbors;

  Node() : x(0), y(0) {}
  Node(int xCoord, int yCoord, std::vector<int> neigh)
      : x(xCoord), y(yCoord), neighbors(neigh) {}
};

class Graph {
public:
  std::vector<Node> nodes;

  void addNode(int x, int y, std::vector<int> neigh) {
    nodes.push_back(Node(x, y, neigh));
  }

  std::vector<int> bfs(int start, int goal) {
    int N = nodes.size();
    if (start < 0 || start >= N || goal < 0 || goal >= N) {
      return std::vector<int>();
    }

    std::vector<bool> visited(N, false);
    std::vector<int> parent(N, -1);
    std::queue<int> q;

    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
      int cur = q.front(); q.pop();

      if (cur == goal) {
        return reconstructPath(parent, start, goal);
      }

      for (int nb : nodes[cur].neighbors) {
        if (nb < 0 || nb >= N) continue;
        if (!visited[nb]) {
          visited[nb] = true;
          parent[nb] = cur;
          q.push(nb);
        }
      }
    }

    return std::vector<int>(); // no route
  }

private:
  std::vector<int> reconstructPath(std::vector<int>& parent, int start, int goal) {
    std::vector<int> path;
    int cur = goal;

    while (cur != -1) {
      path.push_back(cur);
      if (cur == start) break;
      cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
  }
};

Graph graph;

// Queue of *node indices* to visit in order
std::vector<int> nodeQueue;   // front = nodeQueue[0]
bool queuePaused = false;     // pause following without clearing queue

// ==================== UTILS ====================
float normalizeAngle(float a) {
    while (a >  M_PI)  a -= 2*M_PI;
    while (a < -M_PI)  a += 2*M_PI;
    return a;
}

uint16_t filterVive(uint16_t raw, uint16_t last1, uint16_t last2, uint16_t lastGood) {

    // voting window of previous good values
    // last1 = most recent, last2 = older
    // lastGood = global stable value
    if (robotX == 0 || robotY == 0) return raw;
    // If any value is clearly invalid (0 or tiny), reject immediately
    if (raw < 100 || raw > 9000)
        return lastGood;
    
    // Compute expected next value (smooth motion)
    // Weighted average vote of history
    float predicted = 0.5f * last1 + 0.3f * last2 + 0.2f * lastGood;

    // Max allowed change per frame
    const int MAX_STEP = 250;    // Vive rarely moves >200 in one frame
                                 // tune 150~300 depending on speed

    // If raw diverges too far from predicted → it's noise → reject
    if (abs((int)raw - (int)predicted) > MAX_STEP) {
        return lastGood;   // ignore outlier
    }

    // Otherwise accept raw and update stable value
    return raw;
}

// Find nearest graph node to current Vive pose
int findNearestNode(float x, float y) {
  int best = -1;
  float bestDist = 1e12;

  for (int i = 0; i < (int)graph.nodes.size(); i++) {
    float dx = x - graph.nodes[i].x;
    float dy = y - graph.nodes[i].y;
    float d2 = dx * dx + dy * dy;
    if (d2 < bestDist) {
      bestDist = d2;
      best = i;
    }
  }
  return best;
}

// ==================== ENCODER ISR ====================
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) encoderCount++;
  else encoderCount--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) rightEncoderCount++;
  else rightEncoderCount--;
}

// ==================== MOTOR CONTROL ====================
static inline int pwmFromRPM(float rpm) {
  int pwm = (int)(rpm * 255.0f / 120.0f); // map ±120 RPM to ±255 PWM
  return constrain(pwm, -255, 255);
}

void setMotorPWM(int pwmValue, int motorSide) {
  int scaled = pwmFromRPM(pwmValue);

  if (motorSide == RIGHT_MOTOR) {
    // Invert for right motor wiring
    scaled = -scaled;

    if (scaled > 0) {
      ledcWrite(RIGHT_MOTOR_RPWM, scaled);
      ledcWrite(RIGHT_MOTOR_LPWM, 0);
    } else if (scaled < 0) {
      ledcWrite(RIGHT_MOTOR_RPWM, 0);
      ledcWrite(RIGHT_MOTOR_LPWM, -scaled);
    } else {
      ledcWrite(RIGHT_MOTOR_RPWM, 0);
      ledcWrite(RIGHT_MOTOR_LPWM, 0);
    }
    return;
  }

  // Left motor
  if (scaled > 0) {
    ledcWrite(MOTOR_RPWM, scaled);
    ledcWrite(MOTOR_LPWM, 0);
  } else if (scaled < 0) {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, -scaled);
  } else {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, 0);
  }
}

void stopMotor() {
  targetSpeed = 0;
  rightTargetSpeed = 0;
  baseSpeed = 0;
  steeringValue = 0;

  leftPID.integral = 0; leftPID.lastError = 0;
  rightPID.integral = 0; rightPID.lastError = 0;
}

// ==================== PID CONTROL ====================
float calculatePID(PIDController &pid, float target, float current, float dt) {
  pid.error = target - current;

  pid.integral += pid.error * dt;
  pid.integral = constrain(pid.integral, pid.integralMin, pid.integralMax);

  if (dt > 0) {
    pid.derivative = (pid.error - pid.lastError) / dt;
  } else {
    pid.derivative = 0;
  }

  pid.output = pid.Kp * pid.error + pid.Ki * pid.integral + pid.Kd * pid.derivative;
  pid.lastError = pid.error;

  return pid.output;
}

void updateMotorControl() {
  unsigned long now = millis();
  float dt = (now - lastControlUpdate) / 1000.0f;
  if (dt <= 0) {
    dt = CONTROL_PERIOD / 1000.0f;
  }

  float leftOut  = calculatePID(leftPID,  targetSpeed,      currentSpeed,      dt);
  float rightOut = calculatePID(rightPID, rightTargetSpeed, rightCurrentSpeed, dt);

  // NOTE: You had this form (current + PID); kept as-is.
  setMotorPWM(currentSpeed + leftOut,       LEFT_MOTOR);
  setMotorPWM(rightCurrentSpeed + rightOut, RIGHT_MOTOR);
}

// ==================== SPEED CALCULATION ====================
void calculateSpeed() {
  unsigned long now = millis();
  float dt = now - lastSpeedCalc;

  if (dt > SPEED_CALC_PERIOD) {
    long dl = encoderCount      - lastEncoderCount;
    long dr = rightEncoderCount - rightLastEncoderCount;

    float revLeft  = dl / 1400.0f;  // 1400 counts per rev
    float revRight = dr / 1400.0f;

    currentSpeed      = - revLeft  / dt * 1000.0f * 60.0f; // RPM
    rightCurrentSpeed =   revRight / dt * 1000.0f * 60.0f;

    lastEncoderCount      = encoderCount;
    rightLastEncoderCount = rightEncoderCount;
    lastSpeedCalc         = now;
  }
}

// ==================== GYRO (OPTIONAL, CURRENTLY UNUSED) ====================
float readGyroZdeg() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return (g.gyro.z - gyroZOffset) * 57.2958f; // rad/s -> deg/s
}

void updateGyroIntegration() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0f;
  if (dt <= 0) return;
  lastGyroTime = now;

  currentAngle += readGyroZdeg() * dt;
  while (currentAngle >  180) currentAngle -= 360;
  while (currentAngle < -180) currentAngle += 360;
}

void resetYaw() {
  currentAngle = 0;
  lastGyroTime = millis();
}

// ==================== TOF READ (COMMENTED OUT FEATURE) ====================
void readTOFSensors() {
  unsigned long now = millis();
  if (now - lastTOFRead < TOF_READ_PERIOD) return;

  // FRONT VL53L1X & right VL53L0X sensors are currently disabled
  /*
  if (frontTOF.dataReady()) {
    int16_t d = frontTOF.distance();
    frontDistance = (d > 0) ? d : 8190;
    frontTOF.clearInterrupt();
  }

  VL53L0X_RangingMeasurementData_t m1;
  rightTOF.rangingTest(&m1, false);
  rightDistance1 = (m1.RangeStatus != 4) ? m1.RangeMilliMeter : 8190;

  VL53L0X_RangingMeasurementData_t m2;
  right2TOF.rangingTest(&m2, false);
  rightDistance2 = (m2.RangeStatus != 4) ? m2.RangeMilliMeter : 8190;
  */

  lastTOFRead = now;
}

// ==================== WALL FOLLOW PD (COMMENTED OUT FEATURE) ====================
void wallFollowPD() {
  // Placeholder for future wall-follow PD (disabled for now)
}

// ==================== VIVE READ + NAV ====================
void readDualVive() {
  leftValid = false;
  rightValid  = false;

  // LEFT (was FRONT) VIVE
  if (viveLeft.status() != VIVE_RECEIVING) {
    viveLeft.sync(5);
  } else {
    // shift history of filtered values
    lx2 = lx1; ly2 = ly1;
    lx1 = lx0; ly1 = ly0;

    // new raw samples
    lx0 = viveLeft.xCoord();
    ly0 = viveLeft.yCoord();

    // continuity-based filter using lastGood = lx/ly
    uint16_t lxf = filterVive(lx0, lx1, lx2, lx);
    uint16_t lyf = filterVive(ly0, ly1, ly2, ly);

    if (lxf > 0 && lyf > 0) {
      lx = lxf;
      ly = lyf;
      leftValid = true;
    }
  }

  // RIGHT (was BACK) VIVE
  if (viveRight.status() != VIVE_RECEIVING) {
    viveRight.sync(5);
  } else {
    rx2 = rx1; ry2 = ry1;
    rx1 = rx0; ry1 = ry0;

    rx0 = viveRight.xCoord();
    ry0 = viveRight.yCoord();

    uint16_t rxf = filterVive(rx0, rx1, rx2, rx);
    uint16_t ryf = filterVive(ry0, ry1, ry2, ry);

    if (rxf > 0 && ryf > 0) {
      rx = rxf;
      ry = ryf;
      rightValid = true;
    }
  }
}

// Pose + heading from left/right Vive pair (Option B)
void computeVivePose() {
  if (leftValid && rightValid) {
    // Midpoint of the two trackers
    robotX = (lx + rx) / 2.0f;
    robotY = (ly + ry) / 2.0f;

    // Heading from RIGHT -> LEFT tracker
    // (sign depends on how you mounted them; this is consistent with lx=left, rx=right)
    robotHeading = atan2f(ly - ry, lx - rx) - 3.14159265f/2;
    normalizeAngle(robotHeading);
  } else if (leftValid && !rightValid) {
    robotX = lx;
    robotY = ly;
  } else if (!leftValid && rightValid) {
    robotX = rx;
    robotY = ry;
  } else {
    // Explicitly mark pose invalid
    robotX = 0;
    robotY = 0;
  }
}

bool vivePoseValid() {
  return !(robotX == 0 && robotY == 0);
}

// ==================== NEW VIVE GO-TO-POINT LOGIC ====================
//  - Choose forward/backward direction based on which heading is closer
//  - If |angle error| > 5° → turn in place
//  - If |angle error| ≤ 5° → drive toward target with steering correction
bool viveGoToPointStep() {
  if (!leftValid || !rightValid) {
    stopMotor();
    return false;
  }

  if (!vivePoseValid()) {
    Serial.println("Skipping goto: Vive invalid (0,0)");
    stopMotor();
    return false;
  }

  float dx = (float)viveTargetX - robotX;
  float dy = (float)viveTargetY - robotY;

  float dist = sqrtf(dx * dx + dy * dy);

  // ---------------------------
  // GOAL REACHED
  // ---------------------------
  if (dist < GOAL_REACHED_THRESHOLD) {
    stopMotor();
    return true;
  }

  // ---------------------------
  // FORWARD & BACKWARD HEADINGS
  // ---------------------------
  float desiredForward  = atan2f(dy, dx);
  float desiredBackward = desiredForward + 3.14159265f;  // 180 deg
  if (desiredBackward > 3.14159265f)
      desiredBackward -= 6.2831853f;

  float errF = normalizeAngle(desiredForward  - robotHeading);
  float errB = normalizeAngle(desiredBackward - robotHeading);

  // Choose the direction with smaller heading error
  bool backward = (fabs(errB) < fabs(errF));
  float err     = backward ? errB : errF;

  // ---------------------------
  // TURN-IN-PLACE vs DRIVE+STEER
  // ---------------------------
  const float DEG2RAD        = 3.14159265f / 180.0f;
  const float TURN_THRESHOLD = 35.0f * DEG2RAD;      // 5 degrees
  const float TURN_GAIN      = 3.0f;              // spin aggressiveness
  const int   TURN_LIMIT     = 20;                  // max spin speed

  if (fabs(err) > TURN_THRESHOLD) {
    // TURN IN PLACE (no forward motion)
    float turnRaw = err * TURN_GAIN;
    float turn    = constrain((int)turnRaw, -TURN_LIMIT, TURN_LIMIT);

    // Spin: left and right opposite
    targetSpeed      = -turn;
    rightTargetSpeed =  turn;

    return false;   // still rotating toward target
  }

  // ---------------------------
  // DRIVE TOWARD TARGET WITH STEERING
  // ---------------------------
  float speed = 10; // dist * 0.05f;              // distance-based gain
  speed = constrain((int)speed, 25, 80);   // mm→RPM scaling

  if (backward) {
    speed = -speed;
  }

  // Steering correction while moving
  const float STEER_GAIN  = 5.0f;
  const int   STEER_LIMIT = 15;
  float steerRaw = err * STEER_GAIN;
  float steer    = constrain((int)steerRaw, -STEER_LIMIT, STEER_LIMIT);

  // Differential drive mapping
  float leftCmd  = speed - steer;
  float rightCmd = speed + steer;

  targetSpeed      = leftCmd;
  rightTargetSpeed = rightCmd;

  return false;
}

// Queue-based BFS navigation: follow first node in nodeQueue
void followQueueStep() {
  if (!vivePoseValid()) {
    stopMotor();
    return;
  }

  if (nodeQueue.empty()) {
    stopMotor();
    return;
  }

  int currentNode = nodeQueue.front();

  // Set current target to this node's coordinate
  viveTargetX = graph.nodes[currentNode].x;
  viveTargetY = graph.nodes[currentNode].y;

  bool reached = viveGoToPointStep();

  if (reached) {
    int removed = nodeQueue.front();
    nodeQueue.erase(nodeQueue.begin());
    Serial.printf("Reached node %d. Next queue:\n", removed);

    // Print new queue state
    Serial.print("QUEUE: [");
    for (int i = 0; i < (int)nodeQueue.size(); i++) {
      Serial.print(nodeQueue[i]);
      if (i < (int)nodeQueue.size() - 1) Serial.print(",");
    }
    Serial.println("]");
  }
}

// ==================== WEB SERVER HANDLERS ====================

// Root – dashboard HTML
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// Manual control (PID car manipulation)
void handleSetSpeed() {
  if (server.hasArg("speed") && server.hasArg("steering")) {
    baseSpeed     = server.arg("speed").toFloat();
    steeringValue = server.arg("steering").toFloat();

    if (fabs(steeringValue) <= 5) steeringValue *= 1.4f;
    if (fabs(baseSpeed)    <= 10) baseSpeed     *= 1.4f;

    if (baseSpeed == 0.0f) {
      Serial.println("Spin in place (manual)");
    }

    // Differential mapping: steering positive → turn right (left faster)
    targetSpeed      =  baseSpeed + steeringValue;
    rightTargetSpeed =  baseSpeed - steeringValue;

    targetSpeed      = constrain(targetSpeed,      -120, 120);
    rightTargetSpeed = constrain(rightTargetSpeed, -120, 120);

    controlMode = MODE_MANUAL;

    server.send(200, "text/plain",
      "Control set: Speed=" + String(baseSpeed) +
      " Steering=" + String(steeringValue));
    Serial.printf("Control - Base: %.1f, Steering: %.1f -> Left: %.1f, Right: %.1f\n",
                  baseSpeed, steeringValue, targetSpeed, rightTargetSpeed);
  } else {
    server.send(400, "text/plain", "Missing speed or steering parameter");
  }
}

// legacy alias if some old UI hits /control
void handleControl() {
  if (server.hasArg("speed") && server.hasArg("steering")) {
    baseSpeed     = server.arg("speed").toFloat();
    steeringValue = server.arg("steering").toFloat();

    targetSpeed      = -baseSpeed - steeringValue;
    rightTargetSpeed =  baseSpeed - steeringValue;

    targetSpeed      = constrain(targetSpeed,      -120, 120);
    rightTargetSpeed = constrain(rightTargetSpeed, -120, 120);

    controlMode = MODE_MANUAL;
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing speed or steering");
  }
}

void handleStop() {
  stopMotor();
  controlMode = MODE_MANUAL;
  server.send(200, "text/plain", "Motor stopped");
  Serial.println("Motor stopped");
}

void handleSetPID() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    leftPID.Kp = server.arg("kp").toFloat();
    leftPID.Ki = server.arg("ki").toFloat();
    leftPID.Kd = server.arg("kd").toFloat();

    rightPID.Kp = leftPID.Kp;
    rightPID.Ki = leftPID.Ki;
    rightPID.Kd = leftPID.Kd;

    leftPID.integral  = 0;
    rightPID.integral = 0;

    server.send(200, "text/plain", "PID updated");
    Serial.printf("PID updated: Kp=%.2f Ki=%.2f Kd=%.3f\n",
                  leftPID.Kp, leftPID.Ki, leftPID.Kd);
  } else {
    server.send(400, "text/plain", "Missing PID parameters");
  }
}

// legacy alias
void handlePID() {
  handleSetPID();
}

// Status JSON for dashboard (includes queue of node indices)
void handleStatus() {
  String json = "{";
  json += "\"baseSpeed\":" + String(baseSpeed, 1) + ",";
  json += "\"steering\":"  + String(steeringValue, 1) + ",";

  json += "\"leftTarget\":"   + String(targetSpeed, 1) + ",";
  json += "\"leftCurrent\":"  + String(currentSpeed, 1) + ",";
  json += "\"leftError\":"    + String(leftPID.error, 1) + ",";
  json += "\"leftPWM\":"      + String((int)leftPID.output) + ",";
  json += "\"leftEncoder\":"  + String(encoderCount) + ",";

  json += "\"rightTarget\":"   + String(rightTargetSpeed, 1) + ",";
  json += "\"rightCurrent\":"  + String(rightCurrentSpeed, 1) + ",";
  json += "\"rightError\":"    + String(rightPID.error, 1) + ",";
  json += "\"rightPWM\":"      + String((int)rightPID.output) + ",";
  json += "\"rightEncoder\":"  + String(rightEncoderCount) + ",";

  json += "\"front\":"   + String(frontDistance) + ",";
  json += "\"right1\":"  + String(rightDistance1) + ",";
  json += "\"right2\":"  + String(rightDistance2) + ",";
  json += "\"yaw\":"     + String(currentAngle, 1) + ",";
  json += "\"mode\":"    + String((int)controlMode) + ",";
  json += "\"vx\":"      + String((int)robotX) + ",";
  json += "\"vy\":"      + String((int)robotY) + ",";

  // paused + queue of node indices
  json += "\"paused\":" + String(queuePaused ? 1 : 0) + ",";
  json += "\"queue\":[";
  for (size_t i = 0; i < nodeQueue.size(); ++i) {
    json += String(nodeQueue[i]);
    if (i + 1 < nodeQueue.size()) json += ",";
  }
  json += "]";

  json += "}";
  server.send(200, "application/json", json);
}

// Wall / mode / Vive APIs (wall is effectively disabled but endpoints kept)
void handleWallEnable() {
  if (server.hasArg("enable")) {
    bool enable = (server.arg("enable") == "1" || server.arg("enable") == "true");
    wallFollowMode = enable;
    controlMode    = enable ? MODE_WALL : MODE_MANUAL;
    if (enable) {
      lastDistError = 0;
      resetYaw();
      currentState = STATE_WALL_FOLLOW;
    } else {
      stopMotor();
      currentState = STATE_IDLE;
    }
    server.send(200, "text/plain", enable ? "WALL ON" : "WALL OFF");
  } else {
    server.send(400, "text/plain", "Missing enable");
  }
}

void handleWallGoals() {
  if (server.hasArg("frontGoal"))  frontGoalDistance   = server.arg("frontGoal").toInt();
  if (server.hasArg("rightGoal1")) rightGoalDistance1  = server.arg("rightGoal1").toInt();
  if (server.hasArg("rightGoal2")) rightGoalDistance2  = server.arg("rightGoal2").toInt();
  server.send(200, "text/plain", "Wall goals updated");
}

void handleWallPD() {
  if (server.hasArg("kp")) wallFollowKp = server.arg("kp").toFloat();
  if (server.hasArg("kd")) wallFollowKd = server.arg("kd").toFloat();
  server.send(200, "text/plain", "Wall PD updated");
}

// Mode selection: "manual", "wall", "vive"
void handleMode() {
  if (!server.hasArg("m")) {
    server.send(400, "text/plain", "Missing m");
    return;
  }
  String m = server.arg("m");
  if (m == "manual") {
    controlMode = MODE_MANUAL;
    wallFollowMode = false;
  } else if (m == "wall") {
    controlMode = MODE_WALL;
    wallFollowMode = true;   // logic still commented out
  } else if (m == "vive") {
    controlMode = MODE_VIVE;
  }
  server.send(200, "text/plain", "OK");
}

// Single point go-to (no queue), still available for debugging
// /gotopoint?x=XXXX&y=YYYY[&dir=0/1]
void handleGoToPoint() {
  if (server.hasArg("x") && server.hasArg("y")) {
    viveTargetX = server.arg("x").toInt();
    viveTargetY = server.arg("y").toInt();

    if (server.hasArg("dir")) {
      viveDirectionMode = server.arg("dir").toInt();
    } else {
      viveDirectionMode = 0; // default forward
    }

    controlMode = MODE_VIVE;

    String msg = "Moving ";
    msg += (viveDirectionMode == 0 ? "(FORWARD/BACK AUTO)" : "(LEGACY DIR MODE)");
    server.send(200, "text/plain", msg);
  } else {
    server.send(400, "text/plain", "Missing x or y");
  }
}

// BFS route API: /route?goal=Y
// - If queue not empty: start from last node in queue
// - If queue empty: start from nearest node to current Vive pose
// - Concatenate new route to the queue (without duplicating start node)
// - Start following queue immediately in MODE_VIVE
void handleQueueClear(){
    nodeQueue.clear();        // Remove all planned nodes
    stopMotor();
    server.send(200, "text/plain", "Queue cleared");

    Serial.println("[QUEUE] Cleared");

    // If queue cleared while in VIVE mode, stop motors
    if (controlMode == MODE_VIVE) {
        stopMotor();
    }
}

void handleQueueSkip() {
    if (!nodeQueue.empty()) {
        int skipped = nodeQueue.front();
        nodeQueue.erase(nodeQueue.begin());

        String msg = "Skipped node ";
        msg += skipped;
        server.send(200, "text/plain", msg);

        Serial.printf("[QUEUE] Skipped node %d\n", skipped);
    }
}

void handleRoute() {
  if (!server.hasArg("goal")) {
    server.send(400, "text/plain", "Missing goal");
    return;
  }

  int start = -1;
  int goal = server.arg("goal").toInt();

  // --- VALIDATE VIVE POSE (NEW: robotX > 0 and robotY > 0) ---
  if ((robotX <= 0 || robotY <= 0) && nodeQueue.empty()) {
    server.send(200, "text/plain", "Invalid pose: robotX/robotY <= 0. Cannot compute start node.");
    return;
  }

  // If Vive is invalid and no queue exists → cannot route
  if (!vivePoseValid() && nodeQueue.empty()) {
    server.send(200, "text/plain", "Vive invalid (0,0). Cannot compute start node.");
    return;
  }

  // =========================
  // Determine START NODE
  // =========================

  // 1. If queue not empty, resume from last queued node
  if (!nodeQueue.empty()) {
    start = nodeQueue.back();
  }
  else {
    // 2. If start parameter provided, use it
    if (server.hasArg("start")) {
      start = server.arg("start").toInt();
    }
    // 3. Otherwise infer from pose → but only if pose >0
    else {
      if (!vivePoseValid()) {
        server.send(200, "text/plain", "Vive invalid. Provide explicit start.");
        return;
      }

      if (robotX <= 0 || robotY <= 0) {
        server.send(200, "text/plain", "Invalid pose: robotX/robotY <= 0");
        return;
      }

      start = findNearestNode(robotX, robotY);
    }
  }

  if (start < 0) {
    server.send(200, "text/plain", "No valid start node");
    return;
  }

  // =========================
  // BFS ROUTE COMPUTATION
  // =========================
  std::vector<int> route = graph.bfs(start, goal);
  if (route.empty()) {
    server.send(200, "text/plain", "NO ROUTE FOUND");
    return;
  }

  // =========================
  // Append new route to queue
  // =========================
  size_t beginIndex = 0;
  if (!nodeQueue.empty() && nodeQueue.back() == route[0]) {
    beginIndex = 1;
  }

  for (size_t i = beginIndex; i < route.size(); ++i) {
    nodeQueue.push_back(route[i]);
  }

  // =========================
  // Start following the route
  // =========================
  controlMode = MODE_VIVE;
  viveDirectionMode = 0;   // kept for compatibility, but motion uses auto dir

  // =========================
  // Send back full queue as JSON
  // =========================
  String s = "[";
  for (size_t i = 0; i < nodeQueue.size(); ++i) {
    s += String(nodeQueue[i]);
    if (i + 1 < nodeQueue.size()) s += ",";
  }
  s += "]";

  server.send(200, "application/json", s);

  Serial.println("QUEUE UPDATED:");
  Serial.println(s);
  Serial.printf("BFS from start=%d to goal=%d\n", start, goal);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("MEAM5100 Unified Control - Startup");

  // PWM pins
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_LPWM, OUTPUT);

  ledcAttach(MOTOR_RPWM,       PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_LPWM,       PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_LPWM, PWM_FREQ, PWM_RESOLUTION);

  ledcWrite(MOTOR_RPWM, 0);
  ledcWrite(MOTOR_LPWM, 0);
  ledcWrite(RIGHT_MOTOR_RPWM, 0);
  ledcWrite(RIGHT_MOTOR_LPWM, 0);

  // Encoders
  pinMode(ENCODER_A,       INPUT_PULLUP);
  pinMode(ENCODER_B,       INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A),       encoderISR,      RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ");
  Serial.println(ssid);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts++ < 100) {
    delay(200);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed");
  }

  // HTTP routes
  server.on("/",            handleRoot);

  // Manual control APIs
  server.on("/setspeed",    handleSetSpeed);
  server.on("/stop",        handleStop);
  server.on("/setpid",      handleSetPID);
  server.on("/status",      handleStatus);

  // legacy
  server.on("/control",     handleControl);
  server.on("/pid",         handlePID);

  // wall / vive / mode / BFS
  server.on("/wall/enable", handleWallEnable);
  server.on("/wall/goals",  handleWallGoals);
  server.on("/wall/pd",     handleWallPD);
  server.on("/mode",        handleMode);
  server.on("/gotopoint",   handleGoToPoint);
  server.on("/route",       handleRoute);
  server.on("/queue/clear", handleQueueClear);
  server.on("/queue/skip",  handleQueueSkip);
  server.begin();

  // Vive sensors
  Serial.println("Vive begin");
  viveLeft.begin();
  viveRight.begin();

  lastSpeedCalc     = millis();
  lastControlUpdate = millis();
  lastPrint         = millis();

  // ==================== GRAPH NODES (YOUR MAP) ====================
  // Vive coords
  graph.addNode(5000,6320, {1,5});          // Node 0
  graph.addNode(4500,6230, {0,2,8});        // Node 1
  graph.addNode(3900,6250, {1,3,7});        // Node 2
  graph.addNode(3900,6300, {2,4});          // Node 3
  graph.addNode(3530,6350, {3,11});         // Node 4
  graph.addNode(5000,5630, {0,6,7});        // Node 5
  graph.addNode(5000,5250, {5,7,8,9});      // Node 6
  graph.addNode(4320,5100, {2,5,6,8,10});   // Node 7
  graph.addNode(4640,4770, {6,7,5,1,2});    // Node 8
  graph.addNode(5100,4150, {6,12});         // Node 9
  graph.addNode(4000,4150, {7});            // Node 10
  graph.addNode(3050,4200, {4,15});         // Node 11

  // coords 12~15 are suspicious. not accurate at the edge
  graph.addNode(5000,2100, {9,13});         // Node 12
  graph.addNode(4400,2100, {12,14});        // Node 13
  graph.addNode(4000,2000, {13,15});        // Node 14
  graph.addNode(2000,1900, {13,15});        // Node 15
}

// ==================== LOOP ====================
void loop() {
  server.handleClient();

  // Sensors (gyro + ToF): currently disabled
  // updateGyroIntegration();
  // readTOFSensors();

  // Vive pose
  

  // Mode behaviors
  switch (controlMode) {
    case MODE_MANUAL:
      // targetSpeed & rightTargetSpeed set ry /setspeed
      robotX = 0;
      robotY = 0;
      break;

    case MODE_WALL:
      if (wallFollowMode) {
        robotX = 0;
        robotY = 0;
        // wallFollowPD();  // commented out feature
      }
      break;

    case MODE_VIVE:
      if (millis() - lastVive >= Vive_PERIOD){
      readDualVive();
      computeVivePose();
      lastVive = millis();
      }
      if (!nodeQueue.empty()) {
        followQueueStep();
      } else {
        // No queued route – if you used /gotopoint, you can still call step once:
        // (optional) viveGoToPointStep();
      }
      if (millis() - lastPrint >= Print_PERIOD) {
        // Existing debug print
        Serial.print("X, Y: ");
        Serial.print(robotX);
        Serial.print(", ");
        Serial.println(robotY);
        Serial.print("Heading: ");
        Serial.println(robotHeading);
        Serial.print("Desired Heading: ");
        Serial.println(robotHeading);

        // NEW: PRINT QUEUE
        Serial.print("QUEUE: [");
        for (int i = 0; i < (int)nodeQueue.size(); i++) {
          Serial.print(nodeQueue[i]);
          if (i < (int)nodeQueue.size() - 1) Serial.print(",");
        }
        Serial.println("]");

        lastPrint = millis();
      }
      break;
  }

  if (millis() - lastSpeedCalc >= SPEED_CALC_PERIOD) {
    calculateSpeed();
  }

  if (millis() - lastControlUpdate >= CONTROL_PERIOD) {
    updateMotorControl();
    lastControlUpdate = millis();
  }
}
