/*
 * MEAM5100 Unified Control - main_merged.ino
 *
 * Combines three modes into one sketch:
 *  - Manual speed + steering with PID speed control
 *  - Wall-following with ToF sensors + simple PD steering
 *  - Vive dual-marker navigation to a target (X,Y)
 *
 * Notes on pins (ESP32-S3 defaults used in this project):
 *  - I2C remains on SDA=10, SCL=18
 *  - Vive pins moved to 11 (front) and 14 (back) to avoid I2C conflict
 *  - RIGHT_ENCODER_B kept at 16 (consistent with manual_and_wall_follow)
 *
 * If your hardware uses different pins, update the defines below.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "website.h"      // Root website with all modes
#include "vive510.h"  // Vive tracker driver (kept in subdir)

// ==================== PIN DEFINITIONS (ESP32-S3) ====================


#define LEFT_MOTOR         0   // Left motor identifier
#define RIGHT_MOTOR        1   // Right motor identifier

// Motor 1 (Left Wheel)
#define ENCODER_A          1   // Left encoder channel A
#define ENCODER_B          2   // Left encoder channel B
#define MOTOR_RPWM         18   // Left motor RPWM = Forward direction
#define MOTOR_LPWM         17   // Left motor LPWM = Reverse direction

// Motor 2 (Right Wheel)
#define RIGHT_ENCODER_A   15   // Right encoder channel A
#define RIGHT_ENCODER_B   16   // Right encoder channel B
#define RIGHT_MOTOR_RPWM   41   // Right motor RPWM = Forward direction
#define RIGHT_MOTOR_LPWM   42   // Right motor LPWM = Reverse direction

// I2C Sensors (TOF + MPU6050)
#define I2C_SDA             10   // GPIO 10 (SDA)
#define I2C_SCL             18   // GPIO 18 (SCL)

// ToF XSHUT pins (for power/reset control)
#define TOF_XSHUT_FRONT      1   // VL53L1X
#define TOF_XSHUT_RIGHT      2   // VL53L0X (front-right)
#define TOF_XSHUT_RIGHT2     3   // VL53L0X (back-right)

// Vive tracker pins (moved off I2C pins)
#define VIVE_FRONT_PIN      11
#define VIVE_BACK_PIN       14

// ==================== WIFI ====================
const char* ssid = "TP-Link_8A8C";
const char* password = "12488674";

WebServer server(80);

// ==================== MOTOR & ENCODER ====================
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long rightLastEncoderCount = 0;

float currentSpeed = 0;        // Left RPM
float rightCurrentSpeed = 0;   // Right RPM
float targetSpeed = 0;         // Left target RPM
float rightTargetSpeed = 0;    // Right target RPM

int lastLeftPWM = 0;           // Last applied PWM (0..255)
int lastRightPWM = 0;          // Last applied PWM (0..255)

float baseSpeed = 0;           // UI input
float steeringValue = 0;       // UI input

const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;  // 0..255

struct PIDController {
  float Kp = 0.3;
  float Ki = 1.1;
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

// ==================== TOF SENSORS ====================
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

// ==================== WALL FOLLOWING ====================
bool wallFollowMode = false;
int frontGoalDistance = 150;    // mm
int rightGoalDistance1 = 100;   // mm
int rightGoalDistance2 = 100;   // mm
float wallFollowSpeed = 40;     // RPM base forward

float lastDistError = 0;        // For derivative
float wallFollowKp = 1.5;       // PD gains
float wallFollowKd = 0.8;

// Simple state enumeration for display (kept minimal)
enum RobotState {
  STATE_IDLE,
  STATE_WALL_FOLLOW
};

RobotState currentState = STATE_IDLE;

// ==================== CONTROL MODE ====================
enum ControlMode {
  MODE_MANUAL = 0,
  MODE_WALL = 1,
  MODE_VIVE = 2
};

ControlMode controlMode = MODE_MANUAL;

// ==================== VIVE (dual sensors) ====================
Vive510 viveFront(VIVE_FRONT_PIN);
Vive510 viveBack(VIVE_BACK_PIN);

uint16_t fx, fy, bx, by;        // filtered front/back positions
uint16_t fx0, fy0, fx1, fy1, fx2, fy2;
uint16_t bx0, by0, bx1, by1, bx2, by2;

float robotX = 0, robotY = 0;   // Vive-derived pose
float robotHeading = 0;         // radians
bool viveNavigationMode = false;
int viveTargetX = 0;
int viveTargetY = 0;

// ==================== TIMING ====================
unsigned long lastControlUpdate = 0;
unsigned long lastSpeedCalc = 0;
unsigned long lastTOFRead = 0;

const unsigned long CONTROL_PERIOD = 50;      // ms
const unsigned long SPEED_CALC_PERIOD = 100;  // ms
const unsigned long TOF_READ_PERIOD = 50;     // ms

// ==================== UTILS ====================
uint32_t med3(uint32_t a, uint32_t b, uint32_t c) {
  if ((a <= b) && (a <= c)) return (b <= c) ? b : c;
  else if ((b <= a) && (b <= c)) return (a <= c) ? a : c;
  else return (a <= b) ? a : b;
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
  int pwm = (int)(rpm * 255.0f / 120.0f); // scale ±120 RPM to ±255 PWM
  return constrain(pwm, -255, 255);
}

void setMotorPWM(int pwmValue, int motorSide) {
  pwmValue = constrain(pwmValue, -255, 255);

  if (motorSide == RIGHT_MOTOR) {
    // Invert for right motor wiring
    pwmValue = -pwmValue;
    if (pwmValue >= 0) {
      ledcWrite(RIGHT_MOTOR_RPWM, pwmValue);
      ledcWrite(RIGHT_MOTOR_LPWM, 0);
    } else {
      ledcWrite(RIGHT_MOTOR_RPWM, 0);
      ledcWrite(RIGHT_MOTOR_LPWM, -pwmValue);
    }
    lastRightPWM = abs(pwmValue);
    return;
  }

  // Left motor
  if (pwmValue >= 0) {
    ledcWrite(MOTOR_RPWM, pwmValue);
    ledcWrite(MOTOR_LPWM, 0);
  } else {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, -pwmValue);
  }
  lastLeftPWM = abs(pwmValue);
}

void stopMotor() {
  targetSpeed = 0;
  rightTargetSpeed = 0;
  leftPID.integral = 0; leftPID.lastError = 0;
  rightPID.integral = 0; rightPID.lastError = 0;
}

// ==================== PID ====================
float calculatePID(PIDController &pid, float target, float current, float dt) {
  pid.error = target - current;
  pid.integral += pid.error * dt;
  pid.integral = constrain(pid.integral, pid.integralMin, pid.integralMax);
  pid.derivative = (dt > 0) ? (pid.error - pid.lastError) / dt : 0;
  pid.output = pid.Kp * pid.error + pid.Ki * pid.integral + pid.Kd * pid.derivative;
  pid.lastError = pid.error;
  return pid.output;
}

void updateMotorControl() {
  unsigned long now = millis();
  float dt = (now - lastControlUpdate) / 1000.0f;
  if (dt <= 0) dt = CONTROL_PERIOD / 1000.0f;

  float leftOut = calculatePID(leftPID, targetSpeed, currentSpeed, dt);
  float rightOut = calculatePID(rightPID, rightTargetSpeed, rightCurrentSpeed, dt);

  setMotorPWM(pwmFromRPM(leftOut), LEFT_MOTOR);
  setMotorPWM(pwmFromRPM(rightOut), RIGHT_MOTOR);
}

// ==================== SPEED CALC ====================
void calculateSpeed() {
  unsigned long now = millis();
  float dt = now - lastSpeedCalc;
  if (dt > SPEED_CALC_PERIOD) {
    long dl = encoderCount - lastEncoderCount;
    long dr = rightEncoderCount - rightLastEncoderCount;

    currentSpeed = dl / 1400.0f / dt * 1000.0f * 60.0f;       // rev per ms -> RPM
    rightCurrentSpeed = dr / 1400.0f / dt * 1000.0f * 60.0f;

    lastEncoderCount = encoderCount;
    rightLastEncoderCount = rightEncoderCount;
    lastSpeedCalc = now;
  }
}

// ==================== MPU6050 GYRO ====================
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
  while (currentAngle > 180) currentAngle -= 360;
  while (currentAngle < -180) currentAngle += 360;
}

void resetYaw() {
  currentAngle = 0;
  lastGyroTime = millis();
}

// ==================== TOF READ ====================
void readTOFSensors() {
  unsigned long now = millis();
  if (now - lastTOFRead < TOF_READ_PERIOD) return;

  // VL53L1X front (interrupt-driven API)
  if (frontTOF.dataReady()) {
    int16_t d = frontTOF.distance();
    frontDistance = (d > 0) ? d : 8190;
    frontTOF.clearInterrupt();
  }

  // VL53L0X right front
  VL53L0X_RangingMeasurementData_t m1;
  rightTOF.rangingTest(&m1, false);
  rightDistance1 = (m1.RangeStatus != 4) ? m1.RangeMilliMeter : 8190;

  // VL53L0X right back
  VL53L0X_RangingMeasurementData_t m2;
  right2TOF.rangingTest(&m2, false);
  rightDistance2 = (m2.RangeStatus != 4) ? m2.RangeMilliMeter : 8190;

  lastTOFRead = now;
}

// ==================== WALL FOLLOW (PD) ====================
void wallFollowPD() {
  // average side distance
  float avgRight = (rightDistance1 + rightDistance2) / 2.0f;
  float distError = avgRight - rightGoalDistance1; // goal uses rightGoalDistance1

  float deri = (distError - lastDistError) / (TOF_READ_PERIOD / 1000.0f);
  lastDistError = distError;

  float steer = wallFollowKp * distError + wallFollowKd * deri;
  steer = constrain(steer, -60, 60);

  // Set wheel targets (left faster when turning right => steering positive)
  targetSpeed = wallFollowSpeed - steer;
  rightTargetSpeed = wallFollowSpeed + steer;
}

// ==================== VIVE READ + NAV ====================
void readDualVive() {
  // Front
  if (viveFront.status() != VIVE_RECEIVING) {
    fx = fy = 0; viveFront.sync(5);
  } else {
    fx2 = fx1; fy2 = fy1; fx1 = fx0; fy1 = fy0;
    fx0 = viveFront.xCoord();
    fy0 = viveFront.yCoord();
    fx = med3(fx0, fx1, fx2);
    fy = med3(fy0, fy1, fy2);
    if (fx < 1000 || fy < 1000 || fx > 8000 || fy > 8000) fx = fy = 0;
  }

  // Back
  if (viveBack.status() != VIVE_RECEIVING) {
    bx = by = 0; viveBack.sync(5);
  } else {
    bx2 = bx1; by2 = by1; bx1 = bx0; by1 = by0;
    bx0 = viveBack.xCoord();
    by0 = viveBack.yCoord();
    bx = med3(bx0, bx1, bx2);
    by = med3(by0, by1, by2);
    if (bx < 1000 || by < 1000 || bx > 8000 || by > 8000) bx = by = 0;
  }
}

void computeVivePose() {
  if (fx == 0 || fy == 0 || bx == 0 || by == 0) return;
  robotX = (fx + bx) / 2.0f;
  robotY = (fy + by) / 2.0f;
  robotHeading = atan2f((float)fy - by, (float)fx - bx);
}

void viveGoToPoint() {
  if (!viveNavigationMode) return;
  if (robotX == 0 || robotY == 0) { stopMotor(); return; }

  float dx = (float)viveTargetX - robotX;
  float dy = (float)viveTargetY - robotY;
  float dist = sqrtf(dx*dx + dy*dy);

  if (dist < 80) {
    stopMotor();
    viveNavigationMode = false;
    return;
  }

  float desired = atan2f(dy, dx);
  float err = desired - robotHeading;
  while (err > 3.14159f) err -= 6.28318f;
  while (err < -3.14159f) err += 6.28318f;

  float turn = err * 150.0f;     // turning term
  float speed = dist * 0.05f;    // forward term

  turn = constrain((int)turn, -50, 50);
  speed = constrain((int)speed, 20, 80);

  targetSpeed = speed - turn;
  rightTargetSpeed = speed + turn;
}

// ==================== WEB SERVER HANDLERS ====================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleControl() {
  if (server.hasArg("speed") && server.hasArg("steering")) {
    baseSpeed = server.arg("speed").toFloat();
    steeringValue = server.arg("steering").toFloat();

    // Steering positive: turn right -> left wheel faster
    targetSpeed = -baseSpeed - steeringValue;
    rightTargetSpeed = baseSpeed - steeringValue;

    targetSpeed = constrain((int)targetSpeed, -120, 120);
    rightTargetSpeed = constrain((int)rightTargetSpeed, -120, 120);

    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing speed or steering");
  }
}

void handleStop() {
  stopMotor();
  server.send(200, "text/plain", "Stopped");
}

void handlePID() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    leftPID.Kp = server.arg("kp").toFloat();
    leftPID.Ki = server.arg("ki").toFloat();
    leftPID.Kd = server.arg("kd").toFloat();
    rightPID = leftPID; // copy
    leftPID.integral = rightPID.integral = 0;
    server.send(200, "text/plain", "PID updated");
  } else {
    server.send(400, "text/plain", "Missing kp/ki/kd");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"leftTarget\":" + String(targetSpeed, 1) + ",";
  json += "\"leftCurrent\":" + String(currentSpeed, 1) + ",";
  json += "\"rightTarget\":" + String(rightTargetSpeed, 1) + ",";
  json += "\"rightCurrent\":" + String(rightCurrentSpeed, 1) + ",";
  json += "\"leftPWM\":" + String(lastLeftPWM) + ",";
  json += "\"rightPWM\":" + String(lastRightPWM) + ",";
  json += "\"front\":" + String(frontDistance) + ",";
  json += "\"right1\":" + String(rightDistance1) + ",";
  json += "\"right2\":" + String(rightDistance2) + ",";
  json += "\"yaw\":" + String(currentAngle, 1) + ",";
  json += "\"state\":" + String((controlMode==MODE_WALL && wallFollowMode)?1:0) + ",";
  json += "\"mode\":" + String((int)controlMode) + ",";
  json += "\"vx\":" + String((int)robotX) + ",";
  json += "\"vy\":" + String((int)robotY);
  json += "}";
  server.send(200, "application/json", json);
}

void handleWallEnable() {
  if (server.hasArg("enable")) {
    bool enable = (server.arg("enable") == "1" || server.arg("enable") == "true");
    wallFollowMode = enable;
    controlMode = enable ? MODE_WALL : MODE_MANUAL;
    if (enable) { lastDistError = 0; resetYaw(); currentState = STATE_WALL_FOLLOW; }
    else { stopMotor(); currentState = STATE_IDLE; }
    server.send(200, "text/plain", enable ? "WALL ON" : "WALL OFF");
  } else {
    server.send(400, "text/plain", "Missing enable");
  }
}

void handleWallGoals() {
  if (server.hasArg("frontGoal")) frontGoalDistance = server.arg("frontGoal").toInt();
  if (server.hasArg("rightGoal1")) rightGoalDistance1 = server.arg("rightGoal1").toInt();
  if (server.hasArg("rightGoal2")) rightGoalDistance2 = server.arg("rightGoal2").toInt();
  server.send(200, "text/plain", "Wall goals updated");
}

void handleWallPD() {
  if (server.hasArg("kp")) wallFollowKp = server.arg("kp").toFloat();
  if (server.hasArg("kd")) wallFollowKd = server.arg("kd").toFloat();
  server.send(200, "text/plain", "Wall PD updated");
}

void handleMode() {
  if (!server.hasArg("m")) { server.send(400, "text/plain", "Missing m"); return; }
  String m = server.arg("m");
  if (m == "manual") controlMode = MODE_MANUAL;
  else if (m == "wall") { controlMode = MODE_WALL; wallFollowMode = true; }
  else if (m == "vive") controlMode = MODE_VIVE;
  server.send(200, "text/plain", "OK");
}

void handleGoToPoint() {
  if (server.hasArg("x") && server.hasArg("y")) {
    viveTargetX = server.arg("x").toInt();
    viveTargetY = server.arg("y").toInt();
    viveNavigationMode = true;
    controlMode = MODE_VIVE;
    server.send(200, "text/plain", "Moving");
  } else {
    server.send(400, "text/plain", "Missing x or y");
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(300);

  // PWM
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_LPWM, OUTPUT);
  ledcAttach(MOTOR_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_LPWM, PWM_FREQ, PWM_RESOLUTION);

  // Encoders
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ");
  Serial.println(ssid);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts++ < 100) { delay(200); Serial.print('.'); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed");
  }

  // HTTP routes
  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/stop", handleStop);
  server.on("/pid", handlePID);
  server.on("/status", handleStatus);
  server.on("/wall/enable", handleWallEnable);
  server.on("/wall/goals", handleWallGoals);
  server.on("/wall/pd", handleWallPD);
  server.on("/mode", handleMode);
  server.on("/gotopoint", handleGoToPoint);
  server.begin();

  // I2C + sensors
  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_RIGHT, OUTPUT);
  pinMode(TOF_XSHUT_RIGHT2, OUTPUT);
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  digitalWrite(TOF_XSHUT_RIGHT, HIGH);
  digitalWrite(TOF_XSHUT_RIGHT2, HIGH);
  delay(10);

  if (!frontTOF.begin()) {
    Serial.println("VL53L1X front init failed!");
  } else {
    frontTOF.startRanging();
    Serial.println("VL53L1X front init success!");

  }

  if (!rightTOF.begin()) Serial.println("VL53L0X right1 init failed!");
  if (!right2TOF.begin()) Serial.println("VL53L0X right2 init failed!");

  // MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    // Calibrate Z offset briefly (assume still)
    sensors_event_t a, g, temp;
    float sum = 0;
    for (int i = 0; i < 50; i++) { mpu.getEvent(&a, &g, &temp); sum += g.gyro.z; delay(10); }
    gyroZOffset = sum / 50.0f;
    lastGyroTime = millis();
    Serial.println("MPU6050 init success!");
  } else {
    Serial.println("MPU6050 not found");
  }

  // Vive sensors
  viveFront.begin();
  viveBack.begin();

  lastSpeedCalc = millis();
  lastControlUpdate = millis();
}

// ==================== LOOP ====================
void loop() {
  server.handleClient();

  // Sensors
  updateGyroIntegration();
  readTOFSensors();
  readDualVive();
  computeVivePose();

  // Mode behaviors
  switch (controlMode) {
    case MODE_MANUAL:
      // Targets already set by /control
      break;
    case MODE_WALL:
      if (wallFollowMode) wallFollowPD();
      break;
    case MODE_VIVE:
      viveGoToPoint();
      break;
  }

  if (millis() - lastSpeedCalc >= SPEED_CALC_PERIOD) calculateSpeed();

  if (millis() - lastControlUpdate >= CONTROL_PERIOD) {
    updateMotorControl();
    lastControlUpdate = millis();
  }
}

