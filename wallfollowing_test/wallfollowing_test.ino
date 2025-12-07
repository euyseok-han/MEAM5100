/*
 * MEAM5100 Unified Control - main_merged.ino
 *
 * Combines three modes into one sketch:
 *  - Manual speed + steering with PID speed control
 *  - Wall-following with ToF sensors + simple PD steering
 *  - Vive dual-marker navigation to a target (X,Y)
 *
 * Notes on pins (ESP32-S3 defaults used in this project):
 *  - I2C1 on SDA=11, SCL=12 (Front TOF + Side Front TOF)
 *  - I2C2 on SDA=19, SCL=20 (IMU + Side Back TOF)
 *  - Vive pins: 4 (front) and 47 (back)
 *  - RIGHT_ENCODER_B kept at 16 (consistent with manual_and_wall_follow)
 *
 * If your hardware uses different pins, update the defines below.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "website.h"      // Root website with all modes
#include "vive510.h"  

// ==================== PIN DEFINITIONS (ESP32-S3) ====================
#define LEFT_MOTOR           0   // Motor identifier
#define RIGHT_MOTOR          1   // Motor identifier

// Left Motor & Encoder
#define ENCODER_A            1   // GPIO 1
#define ENCODER_B            2   // GPIO 2
#define MOTOR_RPWM          18   // GPIO 41 - Forward
#define MOTOR_LPWM          17   // GPIO 42 - Reverse

// Right Motor & Encoder
#define RIGHT_ENCODER_A     15   // GPIO 1
#define RIGHT_ENCODER_B     16   // GPIO 2
#define RIGHT_MOTOR_RPWM    41   // GPIO 18 - Forward
#define RIGHT_MOTOR_LPWM    42   // GPIO 17 - Reverse

// I2C Pins
#define I2C_SDA             47   // GPIO 47 (SDA)
#define I2C_SCL             48   // GPIO 48 (SCL)

// Multiplexer Sensor Bus Channels
#define TOF_FRONT_BUS        0   // Multiplexer Bus 0 (Front TOF)
#define TOF_SIDE_FRONT_BUS   1   // Multiplexer Bus 1 (Side Front TOF)
#define TOF_SIDE_BACK_BUS    2   // Multiplexer Bus 2 (Side Back TOF)
#define IMU_BUS              3   // Multiplexer Bus 3 (IMU)

// Vive tracker pins
#define VIVE_FRONT_PIN       4
#define VIVE_BACK_PIN        5

// ==================== WIFI ====================
const char* ssid = "team35_Robot";          // AP name
const char* password = "35353535";         // AP password (min 8 chars)

WebServer server(80);

IPAddress myIP(192, 168, 1, 188);

// ==================== MOTOR & ENCODER ====================
volatile long encoderCount = 0;
long lastEncoderCount = 0;           // Not modified in ISR, doesn't need volatile
volatile long rightEncoderCount = 0;
long rightLastEncoderCount = 0;      // Not modified in ISR, doesn't need volatile

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

// ==================== TOF SENSORS ====================
Adafruit_VL53L0X frontTOF = Adafruit_VL53L0X();   // Front TOF on I2C1
Adafruit_VL53L0X rightTOF = Adafruit_VL53L0X();   // Side front TOF on I2C1
Adafruit_VL53L0X right2TOF = Adafruit_VL53L0X();  // Side back TOF on I2C2

int frontDistance = 0;          // mm
int rightDistance1 = 0;         // mm (front-right)
int rightDistance2 = 0;         // mm (back-right)

// ==================== MPU6050 ====================
Adafruit_MPU6050 mpu;
float currentAngle = 0;         // degrees
float gyroZOffset = 0;          // rad/s offset converted later
unsigned long lastGyroTime = 0;

// Low-pass filter for gyro noise reduction
const float GYRO_ALPHA = 0.8;        // Low-pass filter coefficient (0.8 = smoother, less noise)
const float GYRO_DEADBAND = 0.5;     // Ignore readings below this threshold (deg/s)
float filteredGyroZ = 0.0;           // Filtered gyro Z value

// ==================== WALL FOLLOWING ====================
bool wallFollowMode = false;
int frontGoalDistance = 150;    // mm
int rightGoalDistance1 = 100;   // mm
int rightGoalDistance2 = 100;   // mm
float wallFollowSpeed = 40;     // RPM base forward

float lastDistError = 0;        // For derivative
float wallFollowKp = 1.5;       // PD gains
float wallFollowKd = 0.8;
unsigned long lastWallFollowUpdate = 0;  // Timestamp for derivative calculation

// State machine for wall following with corner detection
enum RobotState {
  STATE_IDLE,
  STATE_WALL_FOLLOW,    // Normal PID wall following
  STATE_INNER_CORNER,   // 90° left turn (obstacle)
  STATE_OUTER_CORNER,   // 90° right turn (wall ends)
  STATE_BLIND_FORWARD,  // Move forward to clear corner
  STATE_SEEK_WALL       // Find wall after turn
};

RobotState currentState = STATE_IDLE;

// Corner detection thresholds
const int WALL_LOST_THRESHOLD = 800;   // mm - trigger outer corner
const int BLIND_FORWARD_DURATION = 800; // ms - duration to move forward after outer corner

float targetTurnAngle = 0;
unsigned long stateStartTime = 0;
unsigned long lastPrint = 0;

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

uint16_t fx = 0, fy = 0, bx = 0, by = 0;  // filtered front/back positions
uint16_t fx0 = 0, fy0 = 0, fx1 = 0, fy1 = 0, fx2 = 0, fy2 = 0;
uint16_t bx0 = 0, by0 = 0, bx1 = 0, by1 = 0, bx2 = 0, by2 = 0;

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
const unsigned long IMU_READ_PERIOD = 50;     // ms

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

  setMotorPWM(leftOut, LEFT_MOTOR);
  setMotorPWM(rightOut, RIGHT_MOTOR);
}

// ==================== SPEED CALC ====================
void calculateSpeed() {
  unsigned long now = millis();
  float dt = now - lastSpeedCalc;
  if (dt > SPEED_CALC_PERIOD) {
    // Read encoder counts atomically (disable interrupts briefly)
    noInterrupts();
    long currentLeft = encoderCount;
    long currentRight = rightEncoderCount;
    interrupts();

    long dl = currentLeft - lastEncoderCount;
    long dr = currentRight - rightLastEncoderCount;

    currentSpeed = dl / 1400.0f / dt * 1000.0f * 60.0f;       // rev per ms -> RPM
    rightCurrentSpeed = dr / 1400.0f / dt * 1000.0f * 60.0f;

    lastEncoderCount = currentLeft;
    rightLastEncoderCount = currentRight;
    lastSpeedCalc = now;
  }
}

// ==================== MULTIPLEXER =====================
void setMultiplexerBus(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  // Serial.print(bus);
  delay(2);
}

// ==================== MPU6050 GYRO ====================
float readGyroZdeg() {
  setMultiplexerBus(IMU_BUS);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert rad/s to deg/s and remove offset
  float gyroZ = (g.gyro.z - gyroZOffset) * 57.2958f;

  // Apply low-pass filter to reduce noise
  filteredGyroZ = GYRO_ALPHA * filteredGyroZ + (1.0 - GYRO_ALPHA) * gyroZ;

  // Apply deadband - ignore small values (noise)
  if (abs(filteredGyroZ) < GYRO_DEADBAND) {
    return 0.0;
  }

  return filteredGyroZ;
}

void updateGyroIntegration() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0f;
  if (dt * 1000 <= IMU_READ_PERIOD) return;
  lastGyroTime = now;

  currentAngle += readGyroZdeg() * dt;
  while (currentAngle > 180) currentAngle -= 360;
  while (currentAngle < -180) currentAngle += 360;
  Serial.print("  Angle: ");
  Serial.println(currentAngle);
}

void resetYaw() {
  currentAngle = 0;
  filteredGyroZ = 0.0;  // Reset low-pass filter
  lastGyroTime = millis();
}

// ==================== TURN BY ANGLE ====================
/*
 * P-Controller for turning:
 * - Motor speed proportional to remaining angle
 * - Slows down as target approached (prevents overshoot)
 * - Returns true when turn complete (within tolerance)
 */
bool turnByAngle(float targetAngle) {
  float angleError = targetAngle - currentAngle;

  // Normalize error to [-180, 180]
  while (angleError > 180) angleError -= 360;
  while (angleError < -180) angleError += 360;

  const float angleTolerance = 2.0;  // degrees
  const float Kp_turn = 0.8;         // Turn P-gain
  const float minTurnSpeed = 15;     // Minimum speed to overcome friction
  const float maxTurnSpeed = 50;     // Maximum turn speed

  if (abs(angleError) < angleTolerance) {
    stopMotor();
    return true;  // Turn complete
  }

  // P-controller: speed proportional to error
  float turnSpeed = Kp_turn * abs(angleError);
  turnSpeed = constrain(turnSpeed, minTurnSpeed, maxTurnSpeed);

  // Differential drive turning
  if (angleError > 0) {
    // Turn left
    targetSpeed = -turnSpeed;
    rightTargetSpeed = turnSpeed;
  } else {
    // Turn right
    targetSpeed = turnSpeed;
    rightTargetSpeed = -turnSpeed;
  }

  return false;  // Still turning
}

// ==================== TOF READ ====================
void readTOFSensors() {
  unsigned long now = millis();
  if (now - lastTOFRead < TOF_READ_PERIOD) return;

  // VL53L0X front
  setMultiplexerBus(TOF_FRONT_BUS);
  VL53L0X_RangingMeasurementData_t measureFront;
  frontTOF.rangingTest(&measureFront, false);
  if (measureFront.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Front: "); Serial.print(measureFront.RangeMilliMeter);
    frontDistance = measureFront.RangeMilliMeter;
  } else {
    Serial.println(" out of range ");
  }

  // VL53L0X right front
  setMultiplexerBus(TOF_SIDE_FRONT_BUS);
  VL53L0X_RangingMeasurementData_t measureRight1;
  rightTOF.rangingTest(&measureRight1, false);
  if (measureRight1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("  Side Front: "); Serial.print(measureRight1.RangeMilliMeter);
    rightDistance1 = measureRight1.RangeMilliMeter;
  } else {
    Serial.println(" out of range ");
  }

  // VL53L0X right back
  setMultiplexerBus(TOF_SIDE_BACK_BUS);
  VL53L0X_RangingMeasurementData_t measureRight2;
  right2TOF.rangingTest(&measureRight2, false);
  if (measureRight2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("  Side Back: "); Serial.print(measureRight2.RangeMilliMeter);
    rightDistance2 = measureRight2.RangeMilliMeter;
  } else {
    Serial.println(" out of range ");
  }

  lastTOFRead = now;
}

// ==================== STATE MACHINE ====================
void updateStateMachine() {
  // Note: gyro is now updated in main loop, not here

  switch (currentState) {

    // ===== STATE: WALL FOLLOWING =====
    case STATE_WALL_FOLLOW:
      wallFollowPD();

      // Transition: Inner corner (obstacle ahead)
      if (frontDistance < frontGoalDistance) {
        Serial.println("Inner corner detected - turning left 90°");
        currentState = STATE_INNER_CORNER;
        stateStartTime = millis();
        stopMotor();
        resetYaw();
        targetTurnAngle = 90;  // Turn left
      }

      // Transition: Outer corner (wall disappeared)
      else if (rightDistance1 > WALL_LOST_THRESHOLD) {
        Serial.println("Outer corner detected - blind forward");
        currentState = STATE_BLIND_FORWARD;
        stateStartTime = millis();
        targetSpeed = wallFollowSpeed * 0.6;
        rightTargetSpeed = wallFollowSpeed * 0.6;
      }
      break;

    // ===== STATE: INNER CORNER (90° LEFT TURN) =====
    case STATE_INNER_CORNER:
      if (turnByAngle(targetTurnAngle)) {
        Serial.println("Inner corner turn complete - resuming wall follow");
        currentState = STATE_WALL_FOLLOW;
        resetYaw();  // Reset angle to prevent accumulated error
        lastDistError = 0;  // Reset derivative
        lastWallFollowUpdate = 0;  // Reset wall follow timer
      }
      break;

    // ===== STATE: OUTER CORNER - BLIND FORWARD =====
    case STATE_BLIND_FORWARD:
      // Maintain forward motion during blind forward
      targetSpeed = wallFollowSpeed * 0.6;
      rightTargetSpeed = wallFollowSpeed * 0.6;

      if (millis() - stateStartTime > BLIND_FORWARD_DURATION) {
        Serial.println("Blind forward complete - turning right 90°");
        currentState = STATE_OUTER_CORNER;
        stopMotor();
        resetYaw();
        targetTurnAngle = -90;  // Turn right
      }
      break;

    // ===== STATE: OUTER CORNER (90° RIGHT TURN) =====
    case STATE_OUTER_CORNER:
      if (turnByAngle(targetTurnAngle)) {
        Serial.println("Outer corner turn complete - seeking wall");
        currentState = STATE_SEEK_WALL;
      }
      break;

    // ===== STATE: SEEK WALL =====
    case STATE_SEEK_WALL:
      // Maintain slow forward motion while seeking wall
      targetSpeed = wallFollowSpeed * 0.5;
      rightTargetSpeed = wallFollowSpeed * 0.5;

      if (rightDistance1 < WALL_LOST_THRESHOLD) {
        Serial.println("Wall found - resuming wall follow");
        currentState = STATE_WALL_FOLLOW;
        lastDistError = 0;  // Reset derivative
        lastWallFollowUpdate = 0;  // Reset wall follow timer
      }
      break;

    // ===== STATE: IDLE =====
    case STATE_IDLE:
      stopMotor();
      break;
  }

  // Debug output
  if (millis() - lastPrint > 500) {
    Serial.printf("State: %d | Front: %d | RF: %d | RB: %d | Yaw: %.1f\n",
                  currentState, frontDistance, rightDistance1, rightDistance2, currentAngle);
    lastPrint = millis();
  }
}

// ==================== WALL FOLLOW (PD) ====================
void wallFollowPD() {
  unsigned long currentTime = millis();

  // Check if sensor readings are valid (not out of range)
  // If sensors read 8190mm, it means no object detected - don't update control
  if (rightDistance1 > WALL_LOST_THRESHOLD || rightDistance2 > WALL_LOST_THRESHOLD) {
    // Keep previous motor speeds, don't update PD control
    return;
  }

  // Calculate actual time delta for derivative
  float dt = (currentTime - lastWallFollowUpdate) / 1000.0f;  // Convert to seconds

  // Initialize on first call
  if (lastWallFollowUpdate == 0) {
    dt = TOF_READ_PERIOD / 1000.0f;  // Use expected period
  }
  lastWallFollowUpdate = currentTime;

  // Average side distance
  float avgRight = (rightDistance1 + rightDistance2) / 2.0f;
  float distError = avgRight - rightGoalDistance1;

  // Calculate derivative with actual time delta
  float deri = (dt > 0) ? (distError - lastDistError) / dt : 0;
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

  turn = constrain(turn, -50.0f, 50.0f);
  speed = constrain(speed, 20.0f, 80.0f);

  targetSpeed = speed - turn;
  rightTargetSpeed = speed + turn;
}

// ==================== WEB SERVER HANDLERS ====================
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

  // // paused + queue of node indices
  // json += "\"paused\":" + String(queuePaused ? 1 : 0) + ",";
  // json += "\"queue\":[";
  // for (size_t i = 0; i < nodeQueue.size(); ++i) {
  //   json += String(nodeQueue[i]);
  //   if (i + 1 < nodeQueue.size()) json += ",";
  // }
  // json += "]";

  // json += "}";
  server.send(200, "application/json", json);
}

void handleWallEnable() {
  if (server.hasArg("enable")) {
    bool enable = (server.arg("enable") == "1" || server.arg("enable") == "true");
    wallFollowMode = enable;
    controlMode    = enable ? MODE_WALL : MODE_MANUAL;
    if (enable) {
      lastDistError = 0;
      lastWallFollowUpdate = 0;  // Reset wall follow timer
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
  if (m == "manual") {
    controlMode = MODE_MANUAL;
    wallFollowMode = false;
    currentState = STATE_IDLE;
    stopMotor();
  }
  else if (m == "wall") {
    controlMode = MODE_WALL;
    wallFollowMode = true;
    lastDistError = 0;
    lastWallFollowUpdate = 0;
    resetYaw();
    currentState = STATE_WALL_FOLLOW;
  }
  else if (m == "vive") {
    controlMode = MODE_VIVE;
    wallFollowMode = false;
    currentState = STATE_IDLE;
  }
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

  // WiFi AP Mode with Fixed IP
  WiFi.softAP(ssid, password);

  Serial.print("AP IP address: HTML//");  
  Serial.println(WiFi.softAPIP());

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

  // I2C bus initialization
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize front sensor (VL53L0X on I2C1)
  setMultiplexerBus(TOF_FRONT_BUS);
  if (!frontTOF.begin()) {
    Serial.println("VL53L0X front init failed!");
  } else {
    Serial.println("VL53L0X front initialized!");
  }

  // Initialize side front sensor (VL53L0X on I2C1)
  setMultiplexerBus(TOF_SIDE_FRONT_BUS);
  if (!rightTOF.begin()) {
    Serial.println("VL53L0X side front init failed!");
  } else {
    Serial.println("VL53L0X side front initialized!");
  }

  // Initialize side back sensor (VL53L0X on I2C2) - no address change needed, it's alone with IMU
  setMultiplexerBus(TOF_SIDE_BACK_BUS);
  if (!right2TOF.begin()) {
    Serial.println("VL53L0X side back init failed!");
  } else {
    Serial.println("VL53L0X side back initialized!");
  }

  // MPU6050 (IMU)
  setMultiplexerBus(IMU_BUS);
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    // Calibrate Z offset briefly (assume still)
    sensors_event_t a, g, temp;
    float sum = 0;
    for (int i = 0; i < 50; i++) { mpu.getEvent(&a, &g, &temp); sum += g.gyro.z; delay(10); }
    gyroZOffset = sum / 50.0f;
    // Initialize low-pass filter with calibrated zero value
    filteredGyroZ = 0.0;
    lastGyroTime = millis();
    Serial.println("MPU6050 initialized on I2C2");
  } else {
    Serial.println("MPU6050 not found on I2C2");
  }

  // // // Vive sensors
  // // viveFront.begin();
  // // viveBack.begin();

  lastSpeedCalc = millis();
  lastControlUpdate = millis();
  lastTOFRead = millis();
  lastPrint = millis();
}

// ==================== LOOP ====================
void loop() {
  server.handleClient();

  // Sensors
  readTOFSensors();
  // // readDualVive();
  // // computeVivePose();

  // Always update gyro for all modes (needed for mode switching)
  updateGyroIntegration();

  // Mode behaviors
  switch (controlMode) {
    case MODE_MANUAL:
      // Targets already set by /control
      break;
    case MODE_WALL:
      if (wallFollowMode) {
        // State machine also calls updateGyroIntegration(), but it's idempotent
        updateStateMachine();
      }
      break;
  //   case MODE_VIVE:
  //     viveGoToPoint();
  //     break;
  }

  if (millis() - lastSpeedCalc >= SPEED_CALC_PERIOD) calculateSpeed();

  if (millis() - lastControlUpdate >= CONTROL_PERIOD) {
    updateMotorControl();
    lastControlUpdate = millis();
  }
}

