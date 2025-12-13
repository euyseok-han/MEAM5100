/*
 * MEAM5100 Unified Control
 * - MODE_MANUAL: web manual drive + PID speed control
 * - MODE_WALL:   wall-follow with TOF + IMU (via TCA9548A)
 * - MODE_VIVE:   Vive-based BFS route following
 */

// ToDo: remove stopmotor() between vive points

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <math.h>

#include "website.h"
#include "vive510.h"
#include "LobotSerialServoControl.h"

// ========== PIN DEFINITIONS (ESP32-S3) ==========
#define LEFT_MOTOR         0
#define RIGHT_MOTOR        1

// LEFT Motor & Encoder (as in your Code A)
#define ENCODER_A          15
#define ENCODER_B          16
#define MOTOR_RPWM         6
#define MOTOR_LPWM         7

// RIGHT  Motor & Encoder (as in your Code A)
#define RIGHT_ENCODER_A    1
#define RIGHT_ENCODER_B    2
#define RIGHT_MOTOR_RPWM   41
#define RIGHT_MOTOR_LPWM   42

// I2C + TCA9548A (Code B)
#define I2C_SDA            47
#define I2C_SCL            48
#define TOF_FRONT_BUS      0
#define TOF_SIDE_FRONT_BUS 1
#define TOF_SIDE_BACK_BUS  2
#define IMU_BUS            3
#define TOP_HAT_BUS        4

// Robot Arm Servo
#define SERVO_SERIAL_RX    18
#define SERVO_SERIAL_TX    17
#define receiveEnablePin   13
#define transmitEnablePin  14

// Vive trackers (Code A style: left/right)
#define VIVE_LEFT_PIN      4
#define VIVE_RIGHT_PIN     5

uint8_t health = 1;
float Faster = 1.0; // Speed multiplier
// ========== WIFI ==========
const char* ssid = "Hphone";
const char* password = "qqqq1234";
WebServer server(80);
volatile uint32_t commandCount = 0;

// ========== CONTROL CONSTANTS ==========
const int   PWM_FREQ           = 5000;
const int   PWM_RESOLUTION     = 8;
const float GOAL_REACHED_THRESHOLD = 173.0f; // mm radius for BFS nodes

// ========== MOTOR & ENCODER ==========
volatile long encoderCount      = 0;
volatile long lastEncoderCount  = 0;
volatile long rightEncoderCount = 0;
volatile long rightLastEncoderCount = 0;
uint8_t correctTime = 0;
float currentSpeed       = 0;
float rightCurrentSpeed  = 0;
float targetSpeed        = 0;
float rightTargetSpeed   = 0;

float baseSpeed          = 0;
float steeringValue      = 0;

// ========== SERVO ==========
int lastServoMove = 0;
int servoPos = 0;
bool armOut = false;
bool armAttacking = false;
bool wallDone = false;
bool unloaded = true;
unsigned long servoReturn = 0;
HardwareSerial HardwareSerial(2);
LobotSerialServoControl BusServo(HardwareSerial,receiveEnablePin,transmitEnablePin);

// ========== PID ==========
struct PIDController {
  float Kp = 0.4;
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

// ========== TOF (Code B) ==========
Adafruit_VL53L0X frontTOF;
Adafruit_VL53L0X rightTOF;
Adafruit_VL53L0X right2TOF;

int frontDistance  = 0;
int rightDistance1 = 0;
int rightDistance2 = 0;

// ========== IMU (Code B) ==========
Adafruit_MPU6050 mpu;
float currentAngle   = 0;
float gyroZOffset    = 0;
unsigned long lastGyroTime = 0;

const float GYRO_ALPHA    = 0.8;
const float GYRO_DEADBAND = 0.5;
float       filteredGyroZ = 0.0;

// ========== WALL FOLLOW (Code B) ==========
bool  wallFollowMode      = false;
int   frontGoalDistance   = 100;
int   rightGoalDistance1  = 60;
int   rightGoalDistance2  = 60;
float wallFollowSpeed     = 45;

float lastDistError       = 0;
float wallFollowKp        = 0.05;
float wallFollowKd        = 0.8;
float wallAngleKp         = 0.1;
unsigned long lastWallFollowUpdate = 0;

enum RobotState {
  STATE_IDLE,
  STATE_WALL_FOLLOW,
  STATE_INNER_CORNER,
  STATE_OUTER_CORNER,
  STATE_BLIND_FORWARD,
  STATE_SEEK_WALL
};
RobotState currentState = STATE_IDLE;

const int  WALL_LOST_THRESHOLD    = 800;
const int  BLIND_FORWARD_DURATION = 800;
float      targetTurnAngle        = 0;
unsigned long stateStartTime      = 0;
unsigned long lastPrint           = 0;

enum ControlMode {
  MODE_MANUAL = 0,
  MODE_WALL   = 1,
  MODE_VIVE   = 2,
  MODE_LOW_TOWER = 3,
  MODE_HIGH_TOWER = 4,
  MODE_NEXUS = 5,
  LAST_TASK = 6
};
ControlMode controlMode = MODE_MANUAL;

// ========== AUTONOMOUS TASKS ==========
bool autoWall = true;
bool nexusStraight = false;
uint16_t certainCount = 0;
bool hitNexus = false;
unsigned long driveForward = 0;
unsigned long wallFollowTime = 0;



// coords!!!!!


const int LOW_TOWER_Y_THRESHOLD = 4600;
const int PRE_LOW_TOWER_X = 4600;
const int PRE_LOW_TOWER_Y = 4600;
const int LOW_TOWER_X = 4700;
const int LOW_TOWER_Y = 4400;

const int HIGH_TOWER_Y_THRESHOLD = 4100;
const int HIGH_TOWER_X = 2700;
const int HIGH_TOWER_Y = 3550;
// Pre-approach waypoint for high tower (fallback: same as target to avoid compile errors)
const int PRE_HIGH_TOWER_X = 3140;
const int PRE_HIGH_TOWER_Y = 3550;

const int NEXUS_Y_THRESHOLD = 4540;
const int PRE_NEXUS_X = 4720;
const int PRE_NEXUS_Y = 6090;
const int NEXUS_X = 4760;
const int NEXUS_Y = 6500;






// ========== VIVE ==========
Vive510 viveLeft(VIVE_LEFT_PIN);
Vive510 viveRight(VIVE_RIGHT_PIN);

uint16_t lx, ly, rx, ry;
uint16_t lx0, ly0, lx1, ly1, lx2, ly2;
uint16_t rx0, ry0, rx1, ry1, rx2, ry2;

struct ViveBuffer {
  uint16_t buf[7];
  int count = 0;
};

ViveBuffer initLX, initLY, initRX, initRY;
bool leftValid  = false;
bool rightValid = false;
bool viveTargetDead = false;
bool wasBackward = false;
float robotX         = 0;
float robotY         = 0;
float robotHeading   = 0;
float desiredHeading = 0;

int viveTargetX      = 0;
int viveTargetY      = 0;

// ========== TIMING ==========
unsigned long lastControlUpdate = 0;
unsigned long lastSpeedCalc     = 0;
unsigned long lastTOFRead       = 0;
unsigned long lastVive          = 0;
unsigned long lastViveMove      = 0;
unsigned long topHatUpdate      = 0;

const unsigned long CONTROL_PERIOD    = 2;
const unsigned long SPEED_CALC_PERIOD = 2;
const unsigned long TOF_READ_PERIOD   = 50;
const unsigned long IMU_READ_PERIOD   = 50;
const unsigned long PRINT_PERIOD      = 2000;
const unsigned long VIVE_READ_PERIOD       = 8;
const unsigned long VIVE_MOVE_PERIOD       = 30;
bool coordViveMode = false;
bool isPush = false;
// ========== GRAPH + BFS (Code A) ==========
class Node {
public:
  int x, y;
  bool dead;
  bool push;
  std::vector<int> neighbors;

  Node(int xCoord, int yCoord, std::vector<int> neigh, bool isDead = false, bool doPush=false)
      : x(xCoord), y(yCoord), neighbors(neigh), dead(isDead), push(doPush) {}
};

class Graph {
public:
  std::vector<Node> nodes;
  void addNode(int x, int y, std::vector<int> neigh, bool isDead = false, bool push=false) {
    nodes.push_back(Node(x, y, neigh, isDead, push));
  }
  std::vector<int> bfs(int start, int goal) {
    int N = nodes.size();
    if (start < 0 || start >= N || goal < 0 || goal >= N) return {};
    std::vector<bool> visited(N, false);
    std::vector<int> parent(N, -1);
    std::queue<int> q;
    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
      int cur = q.front(); q.pop();
      if (cur == goal) return reconstructPath(parent, start, goal);
      for (int nb : nodes[cur].neighbors) {
        if (nb < 0 || nb >= N) continue;
        if (!visited[nb]) {
          visited[nb] = true;
          parent[nb] = cur;
          q.push(nb);
        }
      }
    }
    return {};
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
std::vector<int> nodeQueue;
bool queuePaused = false;

// ========== XY GOTO QUEUE ==========
struct XYTarget {
  int x;
  int y;
  bool isDead;
  bool push;
};

std::deque<XYTarget> xyQueue; // FIFO queue for coordinate targets

// ========== UTILS ==========
float normalizeAngle(float a) {
  while (a >  M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

uint16_t median7(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint16_t e, uint16_t f, uint16_t g) {
  uint16_t arr[7] = {a, b, c, d, e, f, g};
  for (int i = 0; i < 6; i++) {
    for (int j = i + 1; j < 7; j++) {
      if (arr[j] < arr[i]) {
        uint16_t t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
      }
    }
  }
  return arr[3];
}
uint16_t filterVive(uint16_t raw, ViveBuffer &v) {
    if (v.count < 7) {
        v.buf[v.count++] = raw;

        if (v.count == 7) {
            uint16_t m = median7(v.buf[0], v.buf[1], v.buf[2], v.buf[3], v.buf[4], v.buf[5], v.buf[6]);
            return m;
        }
        return raw;
    }
    for (int i = 0; i < 6; i++)
        v.buf[i] = v.buf[i+1];
    v.buf[6] = raw;
    uint16_t m = median7(v.buf[0], v.buf[1], v.buf[2], v.buf[3], v.buf[4], v.buf[5], v.buf[6]);

    return m;
}


int findNearestNode(float x, float y) {
  int best = -1;
  float bestDist = 1e12;
  for (int i = 0; i < (int)graph.nodes.size(); i++) {
    if (graph.nodes[i].dead) continue;
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

void resetYaw() {
  currentAngle = 0;
  filteredGyroZ = 0.0;  // Reset low-pass filter
  lastGyroTime = millis();
}
bool viveDone = false;
bool vivePoseValid() {
  return !(robotX == 0 && robotY == 0);
}

// ========== TCA9548A ==========
void setMultiplexerBus(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
  delay(2);
}

// ========== ENCODERS ==========
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) encoderCount++;
  else encoderCount--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) rightEncoderCount++;
  else rightEncoderCount--;
}

// ========== MOTOR CONTROL ==========
static inline int pwmFromRPM(float rpm) {
  int pwm = (int)(rpm * 255.0f / 330.0f);
  return constrain(pwm, -255, 255);
}

void setMotorPWM(int rpmCmd, int motorSide) {
  int scaled = pwmFromRPM(rpmCmd);
  if (motorSide == RIGHT_MOTOR) {
    // scaled = -scaled;
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
  if (scaled > 0) {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, scaled);
  } else if (scaled < 0) {
    ledcWrite(MOTOR_RPWM, -scaled);
    ledcWrite(MOTOR_LPWM, 0);
  } else {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, 0);
  }
}

// Direct PWM command without RPM mapping (helper for short pulses)
void rawSetMotorPWM(int pwm, int motorSide) {
  int scaled = constrain(pwm, -255, 255);
  scaled = scaled * Faster;
  if (motorSide == RIGHT_MOTOR) {
    // scaled = -scaled;
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
  if (scaled > 0) {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, scaled);
  } else if (scaled < 0) {
    ledcWrite(MOTOR_RPWM, -scaled);
    ledcWrite(MOTOR_LPWM, 0);
  } else {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, 0);
  }
}

void stopMotor() {
  autoWall = true;
  targetSpeed        = 0;
  rightTargetSpeed   = 0;
  baseSpeed          = 0;
  steeringValue      = 0;
  leftPID.integral   = 0;
  leftPID.lastError  = 0;
  rightPID.integral  = 0;
  rightPID.lastError = 0;
}

void rawStopMotor() {
  rawSetMotorPWM(0, LEFT_MOTOR);
  rawSetMotorPWM(0, RIGHT_MOTOR);
  targetSpeed        = 0;
  rightTargetSpeed   = 0;
  baseSpeed          = 0;
  steeringValue      = 0;
  leftPID.integral   = 0;
  leftPID.lastError  = 0;
  rightPID.integral  = 0;
  rightPID.lastError = 0;
}

// ========== PID ==========
float calculatePID(PIDController &pid, float target, float current, float dt) {
  pid.error = target - current;
  pid.integral += pid.error * dt;
  pid.integral = constrain(pid.integral, pid.integralMin, pid.integralMax);
  if (dt > 0) pid.derivative = (pid.error - pid.lastError) / dt;
  else        pid.derivative = 0;
  pid.output = pid.Kp * pid.error + pid.Ki * pid.integral + pid.Kd * pid.derivative;
  pid.lastError = pid.error;
  return pid.output;
}

void updateMotorControl() {
  float dt = (millis() - lastControlUpdate) / 1000.0f;
  if (dt <= 0) dt = CONTROL_PERIOD / 1000.0f;

  float leftOut  = calculatePID(leftPID,  targetSpeed,      currentSpeed,      dt);
  float rightOut = calculatePID(rightPID, rightTargetSpeed, rightCurrentSpeed, dt);

  setMotorPWM(leftOut,  LEFT_MOTOR);
  setMotorPWM(rightOut, RIGHT_MOTOR);
}

// ========== SPEED ==========
void calculateSpeed() {
  unsigned long now = millis();
  float dt = now - lastSpeedCalc;
  if (dt > SPEED_CALC_PERIOD) {
    long dl = encoderCount      - lastEncoderCount;
    long dr = rightEncoderCount - rightLastEncoderCount;

    float revLeft  = dl / 480.0f;
    float revRight = dr / 480.0f;

    currentSpeed      =   revLeft  / dt * 1000.0f * 60.0f;
    rightCurrentSpeed = - revRight / dt * 1000.0f * 60.0f;

    lastEncoderCount      = encoderCount;
    rightLastEncoderCount = rightEncoderCount;
    lastSpeedCalc         = now;
  }
}

// ========== SERVO ==========
void returnArm() {
  BusServo.LobotSerialServoMove(1,0,1500);
  unloaded = false;
  servoReturn = millis();
}

void attackArm() {
  if(!armOut){
    BusServo.LobotSerialServoMove(1,700,1500);
    armOut = true;
    servoPos = 1000;
    lastServoMove = millis();
  } else {
    if(millis() - lastServoMove > 2000){
      if(servoPos == 400){
        servoPos = 1000;
      } else {
        servoPos = 400;
      }
      BusServo.LobotSerialServoMove(1,servoPos,1200);
      lastServoMove = millis();
    }
  }
}

// ========== GYRO / IMU ==========
float readGyroZdeg() {
  setMultiplexerBus(IMU_BUS);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyroZ = (g.gyro.z - gyroZOffset) * 57.2958f;
  filteredGyroZ = GYRO_ALPHA * filteredGyroZ + (1.0 - GYRO_ALPHA) * gyroZ;
  if (fabs(filteredGyroZ) < GYRO_DEADBAND) return 0.0;
  return filteredGyroZ;
}

void reSetup(){

  setMultiplexerBus(TOF_FRONT_BUS);
  if (!frontTOF.begin()) Serial.println("Front TOF init failed");
  else                   Serial.println("Front TOF OK");

  setMultiplexerBus(TOF_SIDE_FRONT_BUS);
  if (!rightTOF.begin()) Serial.println("Side front TOF init failed");
  else                   Serial.println("Side front TOF OK");

  setMultiplexerBus(TOF_SIDE_BACK_BUS);
  if (!right2TOF.begin()) Serial.println("Side back TOF init failed");
  else                    Serial.println("Side back TOF OK");

  setMultiplexerBus(IMU_BUS);
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sensors_event_t a, g, temp;
    float sum = 0;
    for (int i = 0; i < 50; i++) {
      mpu.getEvent(&a, &g, &temp);
      sum += g.gyro.z;
      delay(10);
    }
    gyroZOffset = sum / 50.0f;
    filteredGyroZ = 0.0;
    lastGyroTime = millis();
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 not found");
  }

  viveLeft.begin();
  viveRight.begin();

  BusServo.OnInit();
  HardwareSerial.begin(115200,SERIAL_8N1,SERVO_SERIAL_RX,SERVO_SERIAL_TX);
  delay(500);

  lastSpeedCalc     = millis();
  lastControlUpdate = millis();
  lastTOFRead       = millis();
  lastPrint         = millis();
  lastVive          = millis();
  topHatUpdate      = millis();
  lastServoMove     = millis();

}
void updateGyroIntegration() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0f;
  if (dt * 1000 <= IMU_READ_PERIOD) return;
  lastGyroTime = now;

  currentAngle += readGyroZdeg() * dt;
  while (currentAngle >  180) currentAngle -= 360;
  while (currentAngle < -180) currentAngle += 360;
}

// ========== TURN BY ANGLE (WALL) ==========
bool turnByAngle(float targetAngle) {
  float angleError = targetAngle - currentAngle;
  while (angleError >  180) angleError -= 360;
  while (angleError < -180) angleError += 360;

  const float angleTolerance = 5.0;
  const float Kp_turn        = 0.5;
  const float minTurnSpeed   = 15;
  const float maxTurnSpeed   = 50;

  if (fabs(angleError) < angleTolerance) {
    stopMotor();
    return true;
  }

  float turnSpeed = Kp_turn * fabs(angleError);
  turnSpeed = constrain(turnSpeed, minTurnSpeed, maxTurnSpeed);

  if (angleError > 0) {
    targetSpeed      = -turnSpeed;
    rightTargetSpeed =  turnSpeed;
  } else {
    targetSpeed      =  turnSpeed;
    rightTargetSpeed = -turnSpeed;
  }
  return false;
}

// ========== TOF READ ==========
void readTOFSensors() {
  unsigned long now = millis();
  if (now - lastTOFRead < TOF_READ_PERIOD) return;

  // Front
  setMultiplexerBus(TOF_FRONT_BUS);
  VL53L0X_RangingMeasurementData_t measureFront;
  frontTOF.rangingTest(&measureFront, false);
  if (measureFront.RangeStatus != 4) {
    frontDistance = measureFront.RangeMilliMeter;
  } else {
    frontDistance = 8190;
  }

  // Side front
  setMultiplexerBus(TOF_SIDE_FRONT_BUS);
  VL53L0X_RangingMeasurementData_t measureRight1;
  rightTOF.rangingTest(&measureRight1, false);
  if (measureRight1.RangeStatus != 4) {
    rightDistance1 = measureRight1.RangeMilliMeter;
  } else {
    rightDistance1 = 8190;
  }

  // Side back
  setMultiplexerBus(TOF_SIDE_BACK_BUS);
  VL53L0X_RangingMeasurementData_t measureRight2;
  right2TOF.rangingTest(&measureRight2, false);
  if (measureRight2.RangeStatus != 4) {
    rightDistance2 = measureRight2.RangeMilliMeter;
  } else {
    rightDistance2 = 8190;
  }

  lastTOFRead = now;
}

// ========== WALL FOLLOW PD ==========
void wallFollowPD() {
  unsigned long currentTime = millis();
  if (rightDistance1 > WALL_LOST_THRESHOLD || rightDistance2 > WALL_LOST_THRESHOLD) return;

  float dt = (currentTime - lastWallFollowUpdate) / 1000.0f;
  if (lastWallFollowUpdate == 0) dt = TOF_READ_PERIOD / 1000.0f;
  lastWallFollowUpdate = currentTime;

  // float avgRight  = (rightDistance1 + rightDistance2) / 2.0f;
  // float distError = avgRight - rightGoalDistance1;
  // float deri      = (dt > 0) ? (distError - lastDistError) / dt : 0;
  // lastDistError   = distError;

  float distOneError = rightDistance1 - rightGoalDistance1;

  float angleError = rightDistance1 - rightDistance2;

  // float steer = wallFollowKp * distError + wallFollowKd * deri;
  // float steer = wallFollowKp * distError - wallAngleKp * angleRight;
  float steer = wallFollowKp * distOneError + wallAngleKp * angleError;
  if (millis() - lastPrint > PRINT_PERIOD) {
  Serial.print("wallAngleKp: ");
  Serial.print(wallAngleKp);
  Serial.print("  Steer: ");
  Serial.print(steer);
  Serial.print("  Distance Error: ");
  Serial.print(distOneError);
  Serial.print("  Angle Error: ");
  Serial.println(angleError);
  Serial.println(autoWall);
  Serial.println(commandCount);
  }
  steer = constrain(steer, -20, 20);

  targetSpeed      = (wallFollowSpeed + steer) * Faster;
  rightTargetSpeed = (wallFollowSpeed - steer) * Faster;


}

// ========== WALL STATE MACHINE ==========
void updateStateMachine() {
  switch (currentState) {
    case STATE_WALL_FOLLOW:
      wallFollowPD();
      if (frontDistance < frontGoalDistance) {
        currentState = STATE_INNER_CORNER;
        stateStartTime = millis();
        stopMotor();
        updateGyroIntegration();
        resetYaw();
        targetTurnAngle = 70;
      } else if (rightDistance2 > WALL_LOST_THRESHOLD) {
        currentState = STATE_BLIND_FORWARD;
        stateStartTime = millis();
        targetSpeed      = wallFollowSpeed * 0.6;
        rightTargetSpeed = wallFollowSpeed * 0.6;
      }
      break;

    case STATE_INNER_CORNER:
      if (turnByAngle(targetTurnAngle)) {
        currentState = STATE_WALL_FOLLOW;
        resetYaw();
        lastDistError = 0;
        lastWallFollowUpdate = 0;
      }
      break;

    case STATE_BLIND_FORWARD:
      targetSpeed      = wallFollowSpeed * 0.6;
      rightTargetSpeed = wallFollowSpeed * 0.6;
      if (millis() - stateStartTime > BLIND_FORWARD_DURATION) {
        currentState = STATE_OUTER_CORNER;
        stopMotor();
        resetYaw();
        targetTurnAngle = -70;
      }
      break;

    case STATE_OUTER_CORNER:
      if (turnByAngle(targetTurnAngle)) {
        currentState = STATE_SEEK_WALL;
      }
      break;

    case STATE_SEEK_WALL:
      targetSpeed      = wallFollowSpeed * 0.5;
      rightTargetSpeed = wallFollowSpeed * 0.5;
      if (rightDistance1 < WALL_LOST_THRESHOLD) {
        currentState = STATE_WALL_FOLLOW;
        lastDistError = 0;
        lastWallFollowUpdate = 0;
      }
      break;

    case STATE_IDLE:
      stopMotor();
      break;
  }

  if (millis() - lastPrint > PRINT_PERIOD) {
    Serial.printf("STATE=%d Front=%d RF=%d RB=%d Yaw=%.1f\n",
                  currentState, frontDistance, rightDistance1, rightDistance2, currentAngle);
    lastPrint = millis();
  }
}

// ========== VIVE READ + POSE (Code A) ==========
void readDualVive() {
  leftValid  = false;
  rightValid = false;
  // last good sample for pair-based spike rejection
static uint16_t lastLX = 0, lastLY = 0;

if (viveLeft.status() != VIVE_RECEIVING) {
    viveLeft.sync(5);
} else {
    // shift history
    lx2 = lx1; ly2 = ly1;
    lx1 = lx0; ly1 = ly0;

    // new raw sample
    uint16_t rawX = viveLeft.xCoord();
    uint16_t rawY = viveLeft.yCoord();

    // now push through the individual filters
    uint16_t fx = filterVive(rawX, initLX);
    uint16_t fy = filterVive(rawY, initLY);

    // update outputs only after filtering
    if (fx > 0 && fy > 0) {
        lx = fx;
        ly = fy;
        leftValid = true;
        lastLX = fx;
        lastLY = fy;
    }
}

static uint16_t lastRX = 0, lastRY = 0;

if (viveRight.status() != VIVE_RECEIVING) {
    viveRight.sync(5);
} else {
    rx2 = rx1; ry2 = ry1;
    rx1 = rx0; ry1 = ry0;

    uint16_t rawX = viveRight.xCoord();
    uint16_t rawY = viveRight.yCoord();

    uint16_t fx = filterVive(rawX, initRX);
    uint16_t fy = filterVive(rawY, initRY);

    if (fx > 0 && fy > 0) {
        rx = fx;
        ry = fy;
        rightValid = true;
        lastRX = fx;
        lastRY = fy;
    }
}
}


void computeVivePose() {
  if (leftValid && rightValid) {
    robotX = (lx + rx) / 2.0f;
    robotY = (ly + ry) / 2.0f;
    robotHeading = atan2f(ly - ry, lx - rx) - (float)M_PI / 2.0f;
    robotHeading = normalizeAngle(robotHeading);
  } else if (leftValid && !rightValid) {
    robotX = lx;
    robotY = ly;
    robotHeading = 0;
  } else if (!leftValid && rightValid) {
    robotX = rx;
    robotY = ry;
    robotHeading = 0;
  } else {
    robotHeading = 0;
    robotX = 0;
    robotY = 0;
  }
}

void pushTower(){
  int hitSpeed = wasBackward ? -30 * Faster : 30 * Faster;
  rawSetMotorPWM( hitSpeed, LEFT_MOTOR);
  rawSetMotorPWM( hitSpeed, RIGHT_MOTOR);
  
  if (controlMode == LAST_TASK) {hitSpeed = hitSpeed * Faster; delay(1000);}
  else delay(4000);
  rawSetMotorPWM(-hitSpeed, LEFT_MOTOR);
  rawSetMotorPWM(-hitSpeed, RIGHT_MOTOR);
  delay(600);
  rawStopMotor();
}

void spin(int t=500){
  rawSetMotorPWM(26, RIGHT_MOTOR);
  rawSetMotorPWM(-26, LEFT_MOTOR);
  delay(t);
}
void hitTower(){
  int hitSpeed = wasBackward ? -40 : 40;

  uint8_t hitTimes = 6;

  for (int k = 0; k < hitTimes; k++) {
    rawSetMotorPWM( hitSpeed, LEFT_MOTOR);
    rawSetMotorPWM( hitSpeed, RIGHT_MOTOR);
    if (k==0) delay(5000);
    else delay(800);
    rawSetMotorPWM(-hitSpeed, LEFT_MOTOR);
    rawSetMotorPWM(-hitSpeed, RIGHT_MOTOR);
    delay(100);
  }
  rawSetMotorPWM(0, LEFT_MOTOR);
  rawSetMotorPWM(0, RIGHT_MOTOR);

  hitNexus = true;
  rawStopMotor();
}
bool viveGoToPointStep() {
  if (!leftValid || !rightValid) {
    rawStopMotor();
    return false;
  }
  if (!vivePoseValid()) {
    rawStopMotor();
    return false;
  }

  // ---------------------------
  // Compute vector to target
  // ---------------------------
  float dx = (float)viveTargetX - robotX;
  float dy = (float)viveTargetY - robotY;
  float dist = sqrtf(dx * dx + dy * dy);

  // If target is NOT dead point, stop when close enough
  if (!viveTargetDead && dist < GOAL_REACHED_THRESHOLD) {
    rawStopMotor();
    return true;
  }
  float desiredForward  = atan2f(dy, dx);
  float desiredBackward = desiredForward + (float)M_PI;
  if (desiredBackward > (float)M_PI) desiredBackward -= 2.0f * (float)M_PI;
  float errF = normalizeAngle(desiredForward  - robotHeading);
  float errB = normalizeAngle(desiredBackward - robotHeading);

  wasBackward = (fabs(errB) < fabs(errF));
  float err = wasBackward ? errB : errF;
  desiredHeading = wasBackward ? desiredBackward : desiredForward;
  // ---------------------------
  // ALIGNMENT
  // ---------------------------
  const float DEG2RAD        = (float)M_PI / 180.0f;
  const float TURN_THRESHOLD = viveTargetDead ? (8.0f * DEG2RAD) : (25.0f * DEG2RAD);
  const float TURN_GAIN      = viveTargetDead ? 50.0f : 50.0f;
  const float   TURN_LIMIT     = viveTargetDead ? 23.0 : 25.5;
  int turnErrorTime = 0;
  if (fabs(err) > TURN_THRESHOLD) {
    turnErrorTime ++;
    float turn;
    if (turnErrorTime > 50){
      turn = 50;
    }
    else {
    float turnRaw = err * TURN_GAIN;
    if (23> turnRaw && turnRaw >= 0) turnRaw = 23;
    if (-23 < turnRaw && turnRaw < 0) turnRaw = -23;
    turn    = constrain((int)turnRaw, -TURN_LIMIT, TURN_LIMIT);
    }

    rawSetMotorPWM( -turn/Faster*1.2, LEFT_MOTOR);
    rawSetMotorPWM( turn/Faster, RIGHT_MOTOR);
    return false;  // still turning
  }
  turnErrorTime = 0;
  if (viveTargetDead) {
    rawStopMotor();
    return true;
  }

  // ---------------------------
  // FORWARD DRIVE TOWARD TARGET
  // ---------------------------
  const float SPEED_GAIN = 20.0f;   // dist / 25 gives speed
  float bfsSpeed = dist / SPEED_GAIN;
  bfsSpeed = constrain((int)bfsSpeed, 30, 49);

  const float STEER_GAIN  = 40.0f;
  const int   STEER_LIMIT = 20.0;
  float steerRaw = err * STEER_GAIN;
  float steer    = constrain((int)steerRaw, -STEER_LIMIT, STEER_LIMIT);

  if (wasBackward) bfsSpeed = -bfsSpeed;  

  // Compute wheel commands
  float leftCmd  = bfsSpeed - steer;
  float rightCmd = bfsSpeed + steer;
  rawSetMotorPWM(leftCmd, LEFT_MOTOR);
  rawSetMotorPWM( rightCmd * 0.95, RIGHT_MOTOR);
  return false;
}

void followQueueStep() {
  if (!vivePoseValid()) {
    rawStopMotor();
    return;
  }
  if (queuePaused) {
    rawStopMotor();
    return;
  }
  if ( nodeQueue.empty()) {
    rawStopMotor();
    coordViveMode = true;
    return;
  }

  int currentNode = nodeQueue.front();
  viveTargetX = graph.nodes[currentNode].x;
  viveTargetY = graph.nodes[currentNode].y;
  viveTargetDead = graph.nodes[currentNode].dead;
  isPush = graph.nodes[currentNode].push;
  if (viveGoToPointStep()) correctTime ++;
  else correctTime = 0;
  uint8_t correctThreshold = viveTargetDead ? 4 : 0;
  if (correctTime > correctThreshold) {
    if (viveTargetDead) {
      if (isPush) pushTower();
      else hitTower();
    }
    int removed = nodeQueue.front();
    nodeQueue.erase(nodeQueue.begin());
  }
}

void followXYQueueStep() {
  if (!vivePoseValid()) {
    rawStopMotor();
    return;
  }
  if (xyQueue.empty()) {
    rawStopMotor();
    coordViveMode = false;
    return;
  }

  XYTarget &t = xyQueue.front();
  viveTargetX = t.x;
  viveTargetY = t.y;
  viveTargetDead = t.isDead;
  isPush = t.push;

  if (viveGoToPointStep()) correctTime ++;
  else correctTime = 0;
  uint8_t correctThreshold = viveTargetDead ? 6 : 0;
  if (correctTime > correctThreshold) {
    if (viveTargetDead) {
      if (isPush) pushTower();
      else hitTower();
    }
    xyQueue.pop_front();
  }
}

// ========== WEB HANDLERS ==========
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}


void handleSetSpeed() {
  commandCount++;
  if (server.hasArg("speed") && server.hasArg("steering")) {
   
    baseSpeed     = server.arg("speed").toFloat();
    steeringValue = server.arg("steering").toFloat();

    if (fabs(steeringValue) <= 5) steeringValue *= 1.4f;
    if (fabs(baseSpeed)    <= 10) baseSpeed     *= 1.4f;

    targetSpeed      =  baseSpeed + steeringValue;
    rightTargetSpeed =  baseSpeed - steeringValue;

    targetSpeed      = constrain(targetSpeed,      -120, 120);
    rightTargetSpeed = constrain(rightTargetSpeed, -120, 120);

    controlMode = MODE_MANUAL;

    server.send(200, "text/plain",
      "Control set: Speed=" + String(baseSpeed) +
      "   Steering=" + String(steeringValue));
  } else {
    server.send(400, "text/plain", "Missing speed or steering");
  }
}

// legacy alias
void handleControl() {
  commandCount++;
  handleSetSpeed();
}

void handleStop() {
  commandCount++;
  stopMotor();
  controlMode = MODE_MANUAL;
  
  server.send(200, "text/plain", "Motor stopped");
}

void handleStopTask() {
  commandCount++;
  reSetup();
  stopMotor();
  wallFollowTime = millis();
  rawStopMotor();
  controlMode = MODE_MANUAL;
  autoWall = true;

  server.send(200, "text/plain", "Motor stopped");
}

void handleSetPID() {
  commandCount++;
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    leftPID.Kp = server.arg("kp").toFloat();
    leftPID.Ki = server.arg("ki").toFloat();
    leftPID.Kd = server.arg("kd").toFloat();
    rightPID = leftPID;
    leftPID.integral  = 0;
    rightPID.integral = 0;
    server.send(200, "text/plain", "PID updated");
  } else {
    server.send(400, "text/plain", "Missing PID parameters");
  }
}

// legacy alias
void handlePID() {
  commandCount++;
  handleSetPID();
}

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
  json += "\"desiredYaw\":" + String(desiredHeading * 180.0f / (float)M_PI, 1) + ",";

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

void handleWallEnable() {
  commandCount++;
  if (server.hasArg("enable")) {
    bool enable = (server.arg("enable") == "1" || server.arg("enable") == "true");
    wallFollowMode = enable;
    controlMode    = enable ? MODE_WALL : MODE_MANUAL;
    if (enable) {
      lastDistError = 0;
      lastWallFollowUpdate = 0;
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
  commandCount++;
  if (server.hasArg("frontGoal"))  frontGoalDistance   = server.arg("frontGoal").toInt();
  if (server.hasArg("rightGoal1")) rightGoalDistance1  = server.arg("rightGoal1").toInt();
  if (server.hasArg("rightGoal2")) rightGoalDistance2  = server.arg("rightGoal2").toInt();
  server.send(200, "text/plain", "Wall goals updated");
}

void handleWallPD() {
  commandCount++;
  if (server.hasArg("kp")) wallFollowKp = server.arg("kp").toFloat();
  if (server.hasArg("kd")) wallFollowKd = server.arg("kd").toFloat();
  if (server.hasArg("kpa"))wallAngleKp  = server.arg("kpa").toFloat();
  server.send(200, "text/plain", "Wall PD updated");
}

void xyQueueClear() {
  while (!xyQueue.empty()) xyQueue.pop_front();
  viveTargetX = 0 ;
  viveTargetY = 0 ;
}
void nodeQueueClear() {
  while (!nodeQueue.empty()) nodeQueue.erase(nodeQueue.begin());
  viveTargetX = 0 ;
  viveTargetY = 0 ;   
}
void handleMode() {
  commandCount++;
  if (!server.hasArg("m")) {
    server.send(400, "text/plain", "Missing m");
    return;
  }
  String m = server.arg("m");
  if (m == "manual") {
    controlMode   = MODE_MANUAL;
    wallFollowMode = false;
    currentState  = STATE_IDLE;
    autoWall = true;
    rawStopMotor();
    robotX = 0; robotY = 0;
    xyQueueClear();
    nodeQueueClear();
  } else if (m == "wall") {
    controlMode    = MODE_WALL;
    autoWall = true;
    wallFollowMode = true;
    lastDistError  = 0;
    lastWallFollowUpdate = 0;
    currentState = STATE_WALL_FOLLOW;
  } else if (m == "vive") {
    controlMode    = MODE_VIVE;
    wallFollowMode = false;
    currentState   = STATE_IDLE;
  }
  server.send(200, "text/plain", "OK");
}

// ======================= ATTACK HANDLER ============================
void handleAttack() {
  commandCount++;

  if (!server.hasArg("target")) {
    server.send(400, "text/plain", "Missing target");
    return;
  }

  String t = server.arg("target");

  // Clear queues so attack mode takes over immediately
  nodeQueueClear();
  xyQueueClear();
  queuePaused = false;

  // Reset motion
  stopMotor();
  wallDone = false;
  viveDone = false;
  autoWall = true;
  coordViveMode = true;   // attacks always use coordinate navigation
  wasBackward = false;
  nexusStraight = false;
  certainCount = 0;
  wallFollowTime = millis();
  driveForward = 0;  // clear any prior forward timers
  
  if (t == "lowtower") {
    Serial.println("Attacking low tower");
    controlMode = MODE_LOW_TOWER;
    xyQueue.push_back({PRE_LOW_TOWER_X, PRE_LOW_TOWER_Y, false, false});
    xyQueue.push_back({LOW_TOWER_X, LOW_TOWER_Y, true, true});
    server.send(200, "text/plain", "Mode set: LOW TOWER");
  }
  else if (t == "hightower") {
    Serial.println("Attacking high tower");
    controlMode = MODE_HIGH_TOWER;
    // xyQueue.push_back({PRE_HIGH_TOWER_X, PRE_HIGH_TOWER_Y, false, false});
    xyQueue.push_back({HIGH_TOWER_X, HIGH_TOWER_Y, true, true});
    server.send(200, "text/plain", "Mode set: HIGH TOWER");
  }
  else if (t == "nexus") {
    Serial.println("Attacking nexus");
    controlMode = MODE_NEXUS;
    hitNexus = false;
    xyQueue.push_back({PRE_NEXUS_X, PRE_NEXUS_Y, false, false});
    xyQueue.push_back({NEXUS_X, NEXUS_Y, true, false});
    server.send(200, "text/plain", "Mode set: NEXUS");
  }
  else if (t == "lastTask"){
    controlMode = LAST_TASK;
    Faster = 1.4;
    xyQueue.push_back({PRE_LOW_TOWER_X, PRE_LOW_TOWER_Y, false, false});
    xyQueue.push_back({LOW_TOWER_X, LOW_TOWER_Y, true, true});
    xyQueue.push_back({PRE_NEXUS_X, PRE_NEXUS_Y, false, false});
    xyQueue.push_back({NEXUS_X, NEXUS_Y, true, true});
    server.send(200, "text/plain", "Mode set: Last Task");
  }
  else {
    server.send(400, "text/plain", "Unknown attack target");
  }
}

// ======================= ARM CONTROL HANDLER ============================
void handleArm() {
  commandCount++;

  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing cmd");
    return;
  }

  String cmd = server.arg("cmd");

  // You can replace these with actual motor/servo commands later
  if (cmd == "return") {
    armAttacking = false;
    returnArm();
    Serial.println("ARM: Return arm command received");
    server.send(200, "text/plain", "Arm returning");
  }
  else if (cmd == "attack") {
    armAttacking = true;
    armOut = false;
    Serial.println("ARM: Attack arm command received");
    server.send(200, "text/plain", "Arm attacking");
  }
  else {
    server.send(400, "text/plain", "Unknown cmd");
  }
}

int gotoY;

void handleGoToPoint() {
  commandCount++;
  autoWall = true;
  gotoY = server.arg("y").toInt();
  controlMode = MODE_VIVE;
  readDualVive();
  if (!server.hasArg("x") || !server.hasArg("y")) {
    server.send(400, "text/plain", "Missing x or y");
    return;
  }
  
  bool isNearest = server.arg("nearest") == "1" || server.arg("nearest") == "true";

  // Push into XY queue (FIFO)
  xyQueue.push_back({server.arg("x").toInt(), server.arg("y").toInt(), false, false});
  coordViveMode = true;
  if (gotoY<4000 && gotoY > 3350) gotoY = 2050;
  wallFollowTime = millis();
}

// BFS route API: /route?goal=Y[&start=X]
void handleRoute() {
  commandCount++;
  controlMode = MODE_VIVE;
  rawStopMotor();
  coordViveMode = false;
  xyQueueClear();
  if (!server.hasArg("goal")) {
    server.send(400, "text/plain", "Missing goal");
    return;
  }
  int start = -1;
  int goal  = server.arg("goal").toInt();

  if ((robotX <= 0 || robotY <= 0) && nodeQueue.empty()) {
    server.send(200, "text/plain", "Invalid pose: robotX/robotY <= 0");
    return;
  }
  if (!vivePoseValid() && nodeQueue.empty()) {
    server.send(200, "text/plain", "Vive invalid (0,0). Cannot compute start node.");
    return;
  }

  if (!nodeQueue.empty()) {
    start = nodeQueue.back();
  } else {
    if (server.hasArg("start")) {
      start = server.arg("start").toInt();
    } else {
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

  std::vector<int> route = graph.bfs(start, goal);
  if (route.empty()) {
    server.send(200, "text/plain", "NO ROUTE FOUND");
    return;
  }

  

  size_t beginIndex = 0;
  if (!nodeQueue.empty() && nodeQueue.back() == route[0]) {
    beginIndex = 1;
  }
  for (size_t i = beginIndex; i < route.size(); ++i) {
    nodeQueue.push_back(route[i]);
  }

  

  String s = "[";
  for (size_t i = 0; i < nodeQueue.size(); ++i) {
    s += String(nodeQueue[i]);
    if (i + 1 < nodeQueue.size()) s += ",";
  }
  s += "]";
  server.send(200, "application/json", s);
}

void handleQueueClear() {
  autoWall = false;
  commandCount++;
  controlMode = MODE_MANUAL;
  nodeQueueClear();
  xyQueueClear();
  rawStopMotor();
  server.send(200, "text/plain", "Queue cleared");
}

void handleQueuePause() {
  commandCount++;
  if (!server.hasArg("enable")) {
    server.send(400, "text/plain", "Missing enable");
    return;
  }
  bool enable = (server.arg("enable") == "1" || server.arg("enable") == "true");
  queuePaused = enable;
  server.send(200, "text/plain", enable ? "QUEUE PAUSED" : "QUEUE RESUMED");
}

void printViveState() {
    unsigned long now = millis();
    if (now - lastPrint < PRINT_PERIOD) return;
    lastPrint = now;

    // ----- POSE -----
    Serial.printf("POSE X,Y = %.1f, %.1f\n", robotX, robotY);

    // ----- Heading -----
    Serial.printf("Heading: %.4f\n", robotHeading);

    // ----- Desired Heading -----
    Serial.printf("Desired: %.4f\n", desiredHeading);

    // ===============================
    // NODE QUEUE (BFS)
    // ===============================
    if (!coordViveMode) {
        Serial.print("QUEUE: [");

        for (int i = 0; i < (int)nodeQueue.size(); i++) {
            if (i < (int)nodeQueue.size() - 1)
                Serial.printf("%d,", nodeQueue[i]);
            else
                Serial.printf("%d", nodeQueue[i]);
        }

        Serial.println("]");
    }

    // ===============================
    // XY QUEUE (Go-To-Point)
    // ===============================
    else {
        Serial.print("XYQ: [");

        for (int i = 0; i < (int)xyQueue.size(); i++) {
            Serial.printf("(%d,%d,%c)", 
                  xyQueue[i].x,
                  xyQueue[i].y,
                  xyQueue[i].isDead ? 'D' : 'N');

            if (i < (int)xyQueue.size() - 1)
                Serial.print(" - ");
        }

        Serial.println("]");
    }

}



// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

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

  pinMode(ENCODER_A,       INPUT_PULLUP);
  pinMode(ENCODER_B,       INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A),       encoderISR,      RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

  // // WiFi with static IP
  // IPAddress local_IP(192, 168, 1, 111);
  // IPAddress gateway(192, 168, 1, 1);
  // IPAddress subnet(255, 255, 255, 0);

  // if (!WiFi.config(local_IP, gateway, subnet)) {
  //   Serial.println("Failed to configure static IP");
  // }

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
  server.on("/",            handleRoot);
  server.on("/setspeed",    handleSetSpeed);
  server.on("/control",     handleControl);
  server.on("/stop",        handleStop);
  server.on("/stop_task",       handleStopTask);
  server.on("/setpid",      handleSetPID);
  server.on("/pid",         handlePID);
  server.on("/status",      handleStatus);
  server.on("/wall/enable", handleWallEnable);
  server.on("/wall/goals",  handleWallGoals);
  server.on("/wall/pd",     handleWallPD);
  server.on("/mode",        handleMode);
  server.on("/gotopoint",   handleGoToPoint);
  server.on("/route",       handleRoute);
  server.on("/queue/clear", handleQueueClear);
  server.on("/queue/pause", handleQueuePause);
  server.on("/attack",      handleAttack);
  server.on("/arm", handleArm);
  server.begin();

  Wire.begin(I2C_SDA, I2C_SCL, 40000);

  setMultiplexerBus(TOF_FRONT_BUS);
  if (!frontTOF.begin()) Serial.println("Front TOF init failed");
  else                   Serial.println("Front TOF OK");

  setMultiplexerBus(TOF_SIDE_FRONT_BUS);
  if (!rightTOF.begin()) Serial.println("Side front TOF init failed");
  else                   Serial.println("Side front TOF OK");

  setMultiplexerBus(TOF_SIDE_BACK_BUS);
  if (!right2TOF.begin()) Serial.println("Side back TOF init failed");
  else                    Serial.println("Side back TOF OK");

  setMultiplexerBus(IMU_BUS);
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sensors_event_t a, g, temp;
    float sum = 0;
    for (int i = 0; i < 50; i++) {
      mpu.getEvent(&a, &g, &temp);
      sum += g.gyro.z;
      delay(10);
    }
    gyroZOffset = sum / 50.0f;
    filteredGyroZ = 0.0;
    lastGyroTime = millis();
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 not found");
  }

  viveLeft.begin();
  viveRight.begin();

  BusServo.OnInit();
  HardwareSerial.begin(115200,SERIAL_8N1,SERVO_SERIAL_RX,SERVO_SERIAL_TX);
  delay(500);

  lastSpeedCalc     = millis();
  lastControlUpdate = millis();
  lastTOFRead       = millis();
  lastPrint         = millis();
  lastVive          = millis();
  topHatUpdate      = millis();
  lastServoMove     = millis();

  // Graph nodes (from Code A)
    // Graph nodes (from Code A)
  graph.addNode(4550, 1800, {1}, true); //0
  graph.addNode(4550,2110,{0,2,3}); //1
  graph.addNode(4475,3275, {1,3,9}); //2
  graph.addNode(5020,3200,{1,2,4}); //3
  graph.addNode(5090,4120, {3,6}); //4
  graph.addNode(4490,4900,{6,7,10}); //5
  graph.addNode(5030,4990,{4,5,7}); //6
  graph.addNode(4670,6250,{5,6,8}); //7
  graph.addNode(4670, 6600,{7}, true); //8
  graph.addNode(4575, 3720,{2}, true); //9(2-2)
  graph.addNode(4600, 4600,{5}, true, true); //10(5-2) // Lower Tower
  
}

// ========== LOOP ==========
void loop() {
  server.handleClient();

  if(millis() - topHatUpdate > 500){
    setMultiplexerBus(TOP_HAT_BUS);
    Wire.beginTransmission(0x28);
    Wire.write(commandCount);
    Wire.endTransmission();
    commandCount = 0;

    uint8_t cnt = Wire.requestFrom(0x28, 1);
    if (cnt > 0) {
      while(Wire.available()){
        health = Wire.read();
        Serial.print("Health: ");
        Serial.println(health);
      }
    }
    
    // Robot is dead
    if(health == 0){
      stopMotor();
    }
    topHatUpdate = millis();
  }

  readTOFSensors();
  updateGyroIntegration();

  switch (controlMode) {
    case MODE_MANUAL:
    if (millis() - lastPrint >= PRINT_PERIOD) {
        Serial.printf("Control - target left: %.1f, terget right: %.1f -> Left: %.1f, Right: %.1f\n",
                  targetSpeed, rightTargetSpeed, currentSpeed, rightCurrentSpeed);
        lastPrint = millis();
      }
      if(armAttacking){
        attackArm();
      } else if(!unloaded && millis() - servoReturn > 2000){
        BusServo.LobotSerialServoUnload(1);
        unloaded = true;
      }
      robotX = 0;
      robotY = 0;
      break;

    case MODE_WALL:
      if (wallFollowMode) {
        updateStateMachine();
      } else {
        stopMotor();
        currentState = STATE_IDLE;
      }
      break;

    case MODE_VIVE:
      if (millis() - lastVive >= VIVE_READ_PERIOD) {
        readDualVive();
        computeVivePose();
        lastVive = millis();
      }
      if (autoWall){
        if (!(millis() - wallFollowTime > 1100 && robotY > gotoY + 30)) {wallFollowPD();}
        else {
          rawStopMotor(); 
          autoWall = false;

          // if (goToY)
          // int start = findNearestNode(robotX, robotY);
          // int goal = findNearestNode(gotoX, gotoY);
          // std::vector<int> route = graph.bfs(start, goal);
          // if (route.empty()) {
          //   server.send(200, "text/plain", "NO ROUTE FOUND, but coord is added to XYqueue");
          //   return;
          // }

          // size_t beginIndex = 0;
          // if (!nodeQueue.empty() && nodeQueue.back() == route[0]) {
          //   beginIndex = 1;
          // }
          // for (size_t i = beginIndex; i < route.size(); ++i) {
          //   nodeQueue.push_back(route[i]);
          // }
        }
      }
      else {
        if (millis() - lastViveMove >= VIVE_MOVE_PERIOD) {
          if (coordViveMode)
            followXYQueueStep();
          else
            followQueueStep();
        
        lastViveMove = millis();
        }
        printViveState();
      }
      break;

    case MODE_LOW_TOWER:
  if (millis() - lastVive >= VIVE_READ_PERIOD) {
    readDualVive();
    computeVivePose();
    lastVive = millis();
  }

  // stage 1: wall-follow until threshold (only once)
  if (!wallDone && millis() - wallFollowTime > 2000 && robotY > LOW_TOWER_Y_THRESHOLD) {
    autoWall = false;
    wallDone = true;
    targetSpeed = 0;
    rightTargetSpeed = 0;
    Serial.println("[LOW] Wall done -> switching to Vive");
  }

  // stage 1 motion
  if (autoWall && !viveDone) {
    wallFollowPD();
  }
  // stage 2: Vive XY queue
  else if (!viveDone) {
    if (millis() - lastViveMove >= VIVE_MOVE_PERIOD) {
      followXYQueueStep();

      if (xyQueue.empty()) {
        Serial.println("[LOW] Vive done -> forward escape");
        viveDone = true;
        autoWall = true;
        driveForward = millis();
      }

      lastViveMove = millis();
    }
  }
  // stage 3: forward escape for N seconds then stop
  else {
    if (millis() - driveForward < 30000) {
      if (wasBackward) {
        targetSpeed = -25;
        rightTargetSpeed = -25;
      } else {
        targetSpeed = 25;
        rightTargetSpeed = 25;
      }
    } else {
      controlMode   = MODE_MANUAL;
      wallFollowMode = false;
      currentState  = STATE_IDLE;
      stopMotor();
      robotX = 0; robotY = 0;
    }
  }
  break;

    case MODE_NEXUS:
      if (millis() - lastVive >= VIVE_READ_PERIOD) {
        readDualVive();
        computeVivePose();
        lastVive = millis();
      }
      if(autoWall && millis() - wallFollowTime > 2000 && robotY > NEXUS_Y_THRESHOLD){
        autoWall = false;
        targetSpeed = 0;
        rightTargetSpeed = 0;
      }
      if(autoWall && !viveDone){
        wallFollowPD();
      } else if(!viveDone){
        if (millis() - lastViveMove >= VIVE_MOVE_PERIOD) {
          followXYQueueStep();
          if (xyQueue.empty()) {
            viveDone = true;
            autoWall = true;
            driveForward = millis();
            if(wasBackward){
              targetSpeed = -20;
              rightTargetSpeed = -20;
            } else {
              targetSpeed = 20;
              rightTargetSpeed = 20;
            }
          }
          lastViveMove = millis();
        }
      } else{
        if(!nexusStraight && millis() - driveForward > 2000){
          targetSpeed = 0;
          rightTargetSpeed = 0;
          autoWall = false;
          nexusStraight = true;
        }
        if(nexusStraight && !hitNexus){
          hitTower();
        } else if (hitNexus) {
          controlMode   = MODE_MANUAL;
          wallFollowMode = false;
          autoWall = true;
          currentState  = STATE_IDLE;
          stopMotor();
          robotX = 0; robotY = 0;
        }
      }
      break;

    
    case MODE_HIGH_TOWER:
      if (millis() - lastVive >= VIVE_READ_PERIOD) {
        readDualVive();
        computeVivePose();
        lastVive = millis();
      }
      if(millis() - wallFollowTime > 3000 && robotY > 4800 && robotY < 5200){
        certainCount++;
      }
      if(!wallDone && certainCount > 1 && robotY < HIGH_TOWER_Y_THRESHOLD){
        Serial.println(certainCount);
        autoWall = false;
        wallDone = true;
        targetSpeed = 0;
        rightTargetSpeed = 0;
      }
      if(autoWall && !viveDone){
        wallFollowPD();
      } else if(!viveDone){
        if (millis() - lastViveMove >= VIVE_MOVE_PERIOD) {
          followXYQueueStep();
          if (xyQueue.empty()) {
            viveDone = true;
            autoWall = true;
            driveForward = millis();
          }
          lastViveMove = millis();
        }
      } else{
        if(millis() - driveForward < 18000){
          if(wasBackward){
            targetSpeed = -20;
            rightTargetSpeed = -20;
          } else {
            targetSpeed = 20;
            rightTargetSpeed = 20;
          }
        } else {
          controlMode   = MODE_MANUAL;
          wallFollowMode = false;
          currentState  = STATE_IDLE;
          stopMotor();
          robotX = 0; robotY = 0;
        }
      }
      break;

    case LAST_TASK:
      if (millis() - lastVive >= VIVE_READ_PERIOD) {
        readDualVive();
        computeVivePose();
        lastVive = millis();
      }
      if (autoWall && millis() - wallFollowTime > 2000 && robotY > LOW_TOWER_Y_THRESHOLD) {
        autoWall = false;
        rawSetMotorPWM(0, LEFT_MOTOR);
        rawSetMotorPWM(0, RIGHT_MOTOR);
      }

      if (autoWall) {
        wallFollowPD();
      } 
      else {
        if (millis() - lastViveMove >= VIVE_MOVE_PERIOD) {
          followXYQueueStep();
          lastViveMove = millis();
          if (xyQueue.empty()) {
            spin(500);
            xyQueue.push_back({PRE_HIGH_TOWER_X, PRE_HIGH_TOWER_Y, false, false});
            xyQueue.push_back({HIGH_TOWER_X, HIGH_TOWER_Y, true, true});
            controlMode = MODE_HIGH_TOWER;
            autoWall = true;
            driveForward = millis();
            wallFollowTime = millis();
          }
          }
        }

      break;}


  if (millis() - lastSpeedCalc >= SPEED_CALC_PERIOD) {
    calculateSpeed();
    lastSpeedCalc = millis();
  }
  if (controlMode == MODE_VIVE && autoWall){
    updateMotorControl();
    lastControlUpdate = millis();
  }

  if (autoWall && millis() - lastControlUpdate >= CONTROL_PERIOD && controlMode != MODE_VIVE) {
    updateMotorControl();
    lastControlUpdate = millis();
  }
}
