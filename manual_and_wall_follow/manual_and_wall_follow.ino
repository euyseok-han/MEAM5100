/*
 * MEAM 5100 Lab 4.2 - Dual Motor Test with PID
 * ESP32-S3 with Encoder Feedback and PID Control
 *
 * LPWM = Reverse/Backward
 * RPWM = Forward
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "website.h"

// ==================== PIN DEFINITIONS (ESP32-S3) ====================
#define LEFT_MOTOR         0   // Motor identifier
#define RIGHT_MOTOR        1   // Motor identifier

// Left Motor & Encoder
#define ENCODER_A          4   // GPIO 4
#define ENCODER_B          5   // GPIO 5
#define MOTOR_RPWM         6   // GPIO 6 - Forward
#define MOTOR_LPWM         7   // GPIO 7 - Reverse

// Right Motor & Encoder
#define RIGHT_ENCODER_A   15   // GPIO 15
#define RIGHT_ENCODER_B   16   // GPIO 16
#define RIGHT_MOTOR_RPWM   9   // GPIO 9 - Forward
#define RIGHT_MOTOR_LPWM  10   // GPIO 10 - Reverse

// I2C Sensors (TOF + MPU6050)
#define I2C_SDA            8   // GPIO 8 (SDA)
#define I2C_SCL           18   // GPIO 18 (SCL)
#define TOF_XSHUT_FRONT    1   // GPIO 1 - VL53L1X
#define TOF_XSHUT_RIGHT    2   // GPIO 2 - VL53L0X
#define TOF_XSHUT_RIGHT2   3   // GPIO 3 - VL53L0X (second right sensor)
// MPU6050 uses same I2C bus (default address 0x68)

const char* ssid = "TP-Link_8A8C";
const char* password = "12488674";

WebServer server(80);

// ==================== MOTOR & ENCODER VARIABLES ====================
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long rightLastEncoderCount = 0;

float currentSpeed = 0;       // RPM
float targetSpeed = 0;        // RPM
float rightCurrentSpeed = 0;  // RPM
float rightTargetSpeed = 0;   // RPM

float baseSpeed = 0;          // User input
float steeringValue = 0;      // User input

const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8; // 0-255

// ==================== PID PARAMETERS ====================
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
Adafruit_VL53L0X right2TOF = Adafruit_VL53L0X();  // Second right sensor

int frontDistance = 0;         // mm
int rightDistance1 = 0;        // mm (front-right sensor)
int rightDistance2 = 0;        // mm (back-right sensor)

// ==================== MPU6050 ====================
Adafruit_MPU6050 mpu;

float currentAngle = 0;        // Current orientation angle (degrees)
float gyroZOffset = 0;         // Gyro Z-axis calibration offset
unsigned long lastGyroTime = 0;

// Low-pass filter for gyro noise reduction
const float GYRO_ALPHA = 0.8;        // Low-pass filter coefficient (0.8 = smoother, less noise)
const float GYRO_DEADBAND = 0.5;     // Ignore readings below this threshold (deg/s)
float filteredGyroZ = 0.0;           // Filtered gyro Z value

bool wallFollowMode = false;
int frontGoalDistance = 150;   // mm
int rightGoalDistance1 = 100;  // mm
int rightGoalDistance2 = 100;  // mm
float wallFollowSpeed = 40;   // RPM

float lastRightError = 0;     // For derivative
float Kp_dist = 1.5;          // PD gain for distance error
float Kd_dist = 0.8;          // PD derivative for distance
float Kp_angle = 2.0;         // P gain for angle error
float lastDistError = 0;      // For derivative
unsigned long lastWallFollowUpdate = 0;  // Timestamp for derivative calculation

// ==================== STATE MACHINE ====================
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
const int FRONT_STOP_DISTANCE = 200;   // mm
const int WALL_LOST_THRESHOLD = 800;   // mm
const int BLIND_FORWARD_DURATION = 800; // ms

float targetTurnAngle = 0;
unsigned long stateStartTime = 0;

// ==================== TIMING VARIABLES ====================
unsigned long lastControlUpdate = 0;
unsigned long lastSpeedCalc = 0;
unsigned long lastPrint = 0;
unsigned long lastTOFRead = 0;

const unsigned long CONTROL_PERIOD = 50;      // ms
const unsigned long SPEED_CALC_PERIOD = 100;  // ms
const unsigned long TOF_READ_PERIOD = 50;     // ms

// ==================== ENCODER ISR ====================
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void IRAM_ATTR rightEncoderISR_R() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}


void setMotorPWM(int pwmValue, int motorSide) {
  // pwmValue can be positive (forward) or negative (reverse)
  pwmValue = pwmValue * 255 / 120; // Scaling rpm speed to pwm duty cycle
  pwmValue = constrain(pwmValue, -255, 255);

  if (motorSide == RIGHT_MOTOR) { // Right motor
    pwmValue = pwmValue * -1; // Invert for right motor
    if (pwmValue > 0) {
    // Forward: RPWM active, LPWM off
    ledcWrite(RIGHT_MOTOR_RPWM, pwmValue);
    ledcWrite(RIGHT_MOTOR_LPWM, 0);
  } 
  else if (pwmValue < 0) {
    // Reverse: LPWM active, RPWM off
    ledcWrite(RIGHT_MOTOR_RPWM, 0);
    ledcWrite(RIGHT_MOTOR_LPWM, abs(pwmValue));
  } 
  else {
    // Stop: Both off
    ledcWrite(RIGHT_MOTOR_RPWM, 0);
    ledcWrite(RIGHT_MOTOR_LPWM, 0);
  }
  return;
  }
  // LEFT MOTOR
  if (pwmValue > 0) { 
    // Forward: RPWM active, LPWM off
    ledcWrite(MOTOR_RPWM, pwmValue);
    ledcWrite(MOTOR_LPWM, 0);
  } 
  else if (pwmValue < 0) {
    // Reverse: LPWM active, RPWM off
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, abs(pwmValue));
  } 
  else {
    // Stop: Both off
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, 0);
  }
}

void stopMotor() {
  targetSpeed = 0;
  rightTargetSpeed = 0;

  // Reset PID for both motors
  leftPID.integral = 0;
  leftPID.lastError = 0;
  rightPID.integral = 0;
  rightPID.lastError = 0;
}

// ==================== PID CONTROL ====================
float calculatePID(PIDController &pid, float target, float current, float dt) {
  // Calculate error
  pid.error = target - current;

  // Integral with anti-windup (time-normalized)
  pid.integral += pid.error * dt;
  pid.integral = constrain(pid.integral, pid.integralMin, pid.integralMax);

  // Derivative (time-normalized)
  if (dt > 0) {
    pid.derivative = (pid.error - pid.lastError) / dt;
  } else {
    pid.derivative = 0;
  }

  // PID output
  pid.output = (pid.Kp * pid.error) +
               (pid.Ki * pid.integral) +
               (pid.Kd * pid.derivative);

  // Update last error
  pid.lastError = pid.error;

  return pid.output;
}

void updateMotorControl() {
  // Calculate time delta in seconds for PID normalization
  unsigned long currentTime = millis();
  float dt = (currentTime - lastControlUpdate) / 1000.0; // Convert ms to seconds

  // Prevent division by zero or negative dt
  if (dt <= 0) {
    dt = CONTROL_PERIOD / 1000.0; // Use expected period as fallback
  }

  // Calculate PID output (can be positive or negative)
  float leftPidOutput = calculatePID(leftPID, targetSpeed, currentSpeed, dt);
  float rightPidOutput = calculatePID(rightPID, rightTargetSpeed, rightCurrentSpeed, dt);

  // Apply to motor (automatically handles direction)
  setMotorPWM(leftPidOutput, LEFT_MOTOR);
  setMotorPWM(rightPidOutput, RIGHT_MOTOR);
}

// ==================== MPU6050 GYRO INTEGRATION ====================
/*
 * Gyro Integration Math:
 * - MPU6050 returns angular velocity in rad/s
 * - Convert to deg/s: multiply by 57.2958
 * - Integration: angle(t) = angle(t-1) + (gyro_z * dt)
 * - Calibration offset removes drift when stationary
 */
float readGyroZ() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert rad/s to deg/s and remove offset
  float gyroZ = (g.gyro.z - gyroZOffset) * 57.2958;

  // Apply low-pass filter to reduce noise
  filteredGyroZ = GYRO_ALPHA * filteredGyroZ + (1.0 - GYRO_ALPHA) * gyroZ;

  // Apply deadband - ignore small values (noise)
  if (abs(filteredGyroZ) < GYRO_DEADBAND) {
    return 0.0;
  }

  return filteredGyroZ;
}

void updateGyroIntegration() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastGyroTime) / 1000.0;

  if (dt <= 0) return;

  lastGyroTime = currentTime;

  float gyroZ = readGyroZ();
  currentAngle += gyroZ * dt; // Integration: angle = velocity * time

  // Normalize angle to [-180, 180]
  while (currentAngle > 180) currentAngle -= 360;
  while (currentAngle < -180) currentAngle += 360;
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

// ==================== TOF SENSOR READING ====================
void readTOFSensors() {
  unsigned long currentTime = millis();

  if (currentTime - lastTOFRead >= TOF_READ_PERIOD) {
    // VL53L1X front sensor
    if (frontTOF.dataReady()) {
      int16_t distance = frontTOF.distance();
      frontDistance = (distance > 0) ? distance : 8190;
      frontTOF.clearInterrupt();
    }

    // VL53L0X right front sensor
    VL53L0X_RangingMeasurementData_t measure1;
    rightTOF.rangingTest(&measure1, false);
    rightDistance1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : 8190;

    // VL53L0X right back sensor
    VL53L0X_RangingMeasurementData_t measure2;
    right2TOF.rangingTest(&measure2, false);
    rightDistance2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : 8190;

    lastTOFRead = currentTime;
  }
}

// ==================== WALL FOLLOWING PD CONTROL ====================
/*
 * Two-error PD Control:
 * Error_Distance: Average side distance from target
 * Error_Angle: Difference between front and back sensors (parallelism)
 */
void wallFollowPD() {
  // Rate limiting - only update at TOF_READ_PERIOD intervals
  unsigned long currentTime = millis();
  if (lastWallFollowUpdate != 0 && (currentTime - lastWallFollowUpdate) < TOF_READ_PERIOD) {
    // Too soon since last update, skip this call
    return;
  }

  // Check if sensor readings are valid (not out of range)
  // If sensors read 8190mm, it means no object detected - don't update control
  if (rightDistance1 > WALL_LOST_THRESHOLD || rightDistance2 > WALL_LOST_THRESHOLD) {
    // Keep previous motor speeds, don't update PD control
    return;
  }

  // Calculate actual time delta for derivative
  float dt = (currentTime - lastWallFollowUpdate) / 1000.0;  // Convert to seconds

  // Initialize on first call
  if (lastWallFollowUpdate == 0) {
    dt = TOF_READ_PERIOD / 1000.0;  // Use expected period
  }
  lastWallFollowUpdate = currentTime;

  // Calculate average distance to wall
  float avgRightDist = (rightDistance1 + rightDistance2) / 2.0;

  // Distance error: how far from target distance
  float distError = avgRightDist - rightGoalDistance1;

  // Angle error: are we parallel to the wall?
  float angleError = rightDistance1 - rightDistance2;

  // PD control for distance (time-normalized derivative)
  float distDerivative = 0;
  if (dt > 0) {
    distDerivative = (distError - lastDistError) / dt;
  }
  float distCorrection = (Kp_dist * distError) + (Kd_dist * distDerivative);

  // P control for angle
  float angleCorrection = Kp_angle * angleError;

  // Total steering correction
  float steeringCorrection = distCorrection + angleCorrection;

  // Apply differential steering
  targetSpeed = -(wallFollowSpeed - steeringCorrection);
  rightTargetSpeed = wallFollowSpeed + steeringCorrection;

  lastDistError = distError;
}

// ==================== STATE MACHINE ====================
void updateStateMachine() {
  // Always update gyro
  updateGyroIntegration();

  switch (currentState) {

    // ===== STATE: WALL FOLLOWING =====
    case STATE_WALL_FOLLOW:
      wallFollowPD();

      // Transition: Inner corner (obstacle ahead)
      if (frontDistance < FRONT_STOP_DISTANCE) {
        Serial.println("Inner corner detected - turning left 90°");
        currentState = STATE_INNER_CORNER;
        stateStartTime = millis();
        stopMotor();
        resetYaw();
        targetTurnAngle = 90;  // Turn left
      }

      // Transition: Outer corner (wall disappeared - front sensor loses wall first or both lose it)
      else if (rightDistance1 > WALL_LOST_THRESHOLD) {
        Serial.println("Outer corner detected - blind forward");
        currentState = STATE_BLIND_FORWARD;
        stateStartTime = millis();
        targetSpeed = -wallFollowSpeed * 0.6;
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
      targetSpeed = -wallFollowSpeed * 0.6;
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
      targetSpeed = -wallFollowSpeed * 0.5;
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

// ==================== SPEED CALCULATION ====================
void calculateSpeed() { 
  unsigned long currentTime = millis();
  float dt = currentTime - lastSpeedCalc; // Units (ms)

  if (dt > 100) {
    // Calculate speed from encoder counts
    long delta = encoderCount - lastEncoderCount;
    long delta_right = rightEncoderCount - rightLastEncoderCount;

    // Left wheel
    currentSpeed = delta / 1400.0 / dt; // 1400 counts per rev for 1:90 Motor
    float rpm = currentSpeed * 1000 * 60; // Convert to RPM

    // Right wheel
    rightCurrentSpeed = delta_right / 1400.0 / dt; // deleted -1
    float right_rpm = rightCurrentSpeed * 1000 * 60; // Convert to RPM

    // Update current speeds with RPM values
    currentSpeed = rpm;
    rightCurrentSpeed = right_rpm;

    // Update last values
    lastEncoderCount = encoderCount;
    rightLastEncoderCount = rightEncoderCount;
    
    lastSpeedCalc = currentTime;
  }
}

// ==================== WEB SERVER HANDLERS ====================
void handleRoot() {
  // Serve the HTML page stored in flash
  server.send_P(200, "text/html", INDEX_HTML);
}

//debugging

void handleSetSpeed() {
  if (server.hasArg("speed") && server.hasArg("steering")) {
    baseSpeed = server.arg("speed").toFloat();
    steeringValue = server.arg("steering").toFloat();

    // Calculate left and right wheel speeds
    // steering > 0 means turn right (left wheel faster, right wheel slower)
    // steering < 0 means turn left (right wheel faster, left wheel slower)
    targetSpeed = -baseSpeed - steeringValue;
    rightTargetSpeed = baseSpeed - steeringValue;

    // Constrain each wheel's target speed to ±120 RPM to prevent PWM saturation
    targetSpeed = constrain(targetSpeed, -120, 120);
    rightTargetSpeed = constrain(rightTargetSpeed, -120, 120);

    server.send(200, "text/plain", "Control set: Speed=" + String(baseSpeed) + " Steering=" + String(steeringValue));
    Serial.printf("Control - Base: %.1f, Steering: %.1f -> Left: %.1f, Right: %.1f\n",
                  baseSpeed, steeringValue, targetSpeed, rightTargetSpeed);
  }
  else {
    server.send(400, "text/plain", "Missing speed or steering parameter");
  }
}

void handleStop() {
  stopMotor();
  server.send(200, "text/plain", "Motor stopped");
  Serial.println("Motor stopped");
}

void handleSetPID() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    // Update both PID controllers with the same parameters
    leftPID.Kp = server.arg("kp").toFloat();
    leftPID.Ki = server.arg("ki").toFloat();
    leftPID.Kd = server.arg("kd").toFloat();

    rightPID.Kp = leftPID.Kp;
    rightPID.Ki = leftPID.Ki;
    rightPID.Kd = leftPID.Kd;

    // Reset integral when changing parameters
    leftPID.integral = 0;
    rightPID.integral = 0;

    server.send(200, "text/plain", "PID updated");
    Serial.printf("PID updated: Kp=%.2f Ki=%.2f Kd=%.3f\n", leftPID.Kp, leftPID.Ki, leftPID.Kd);
  } else {
    server.send(400, "text/plain", "Missing PID parameters");
  }
}

void handleSetWallFollow() {
  if (server.hasArg("frontGoal") && server.hasArg("rightGoal1") && server.hasArg("rightGoal2")) {
    frontGoalDistance = server.arg("frontGoal").toInt();
    rightGoalDistance1 = server.arg("rightGoal1").toInt();
    rightGoalDistance2 = server.arg("rightGoal2").toInt();

    server.send(200, "text/plain", "Wall-following goals updated: Front=" + String(frontGoalDistance) + "mm, Right1=" + String(rightGoalDistance1) + "mm, Right2=" + String(rightGoalDistance2) + "mm");
    Serial.printf("Wall goals updated: Front=%d mm, Right1=%d mm, Right2=%d mm\n", frontGoalDistance, rightGoalDistance1, rightGoalDistance2);
  } else {
    server.send(400, "text/plain", "Missing goal parameters");
  }
}

void handleSetWallPID() {
  if (server.hasArg("kp") && server.hasArg("kd")) {
    Kp_dist = server.arg("kp").toFloat();
    Kd_dist = server.arg("kd").toFloat();

    server.send(200, "text/plain", "Wall-following PD updated: Kp=" + String(Kp_dist, 2) + ", Kd=" + String(Kd_dist, 2));
    Serial.printf("Wall PD updated: Kp=%.2f, Kd=%.2f\n", Kp_dist, Kd_dist);
  } else {
    server.send(400, "text/plain", "Missing kp or kd parameter");
  }
}

void handleWallFollowMode() {
  if (server.hasArg("enable")) {
    bool enable = (server.arg("enable") == "true" || server.arg("enable") == "1");

    if (enable) {
      currentState = STATE_WALL_FOLLOW;
      lastDistError = 0;
      lastWallFollowUpdate = 0;  // Reset wall follow timer
      resetYaw();
      Serial.println("Wall-following started - entering STATE_WALL_FOLLOW");
      server.send(200, "text/plain", "Wall-following started");
    } else {
      currentState = STATE_IDLE;
      stopMotor();
      Serial.println("Wall-following stopped - entering STATE_IDLE");
      server.send(200, "text/plain", "Wall-following stopped");
    }
  } else {
    server.send(400, "text/plain", "Missing enable parameter");
  }
}

void handleStatus() {
  String json = "{";
  // Base control values
  json += "\"baseSpeed\":" + String(baseSpeed, 1) + ",";
  json += "\"steering\":" + String(steeringValue, 1) + ",";

  // Left wheel
  json += "\"leftTarget\":" + String(targetSpeed, 1) + ",";
  json += "\"leftCurrent\":" + String(currentSpeed, 1) + ",";
  json += "\"leftError\":" + String(leftPID.error, 1) + ",";
  json += "\"leftPWM\":" + String((int)leftPID.output) + ",";
  json += "\"leftEncoder\":" + String(encoderCount) + ",";

  // Right wheel
  json += "\"rightTarget\":" + String(rightTargetSpeed, 1) + ",";
  json += "\"rightCurrent\":" + String(rightCurrentSpeed, 1) + ",";
  json += "\"rightError\":" + String(rightPID.error, 1) + ",";
  json += "\"rightPWM\":" + String((int)rightPID.output) + ",";
  json += "\"rightEncoder\":" + String(rightEncoderCount) + ",";

  // PID parameters (same for both)
  json += "\"kp\":" + String(leftPID.Kp, 2) + ",";
  json += "\"ki\":" + String(leftPID.Ki, 2) + ",";
  json += "\"kd\":" + String(leftPID.Kd, 3) + ",";

  // TOF sensor data
  json += "\"frontDist\":" + String(frontDistance) + ",";
  json += "\"rightDist1\":" + String(rightDistance1) + ",";
  json += "\"rightDist2\":" + String(rightDistance2) + ",";
  json += "\"frontGoal\":" + String(frontGoalDistance) + ",";
  json += "\"rightGoal1\":" + String(rightGoalDistance1) + ",";
  json += "\"rightGoal2\":" + String(rightGoalDistance2) + ",";

  // State machine and IMU
  json += "\"state\":" + String(currentState) + ",";
  json += "\"yaw\":" + String(currentAngle, 1) + ",";
  json += "\"wallFollowMode\":" + String((currentState != STATE_IDLE) ? "true" : "false") + ",";

  // Wall-following PD parameters
  json += "\"Kp_dist\":" + String(Kp_dist, 2) + ",";
  json += "\"Kd_dist\":" + String(Kd_dist, 2) + ",";
  json += "\"Kp_angle\":" + String(Kp_angle, 2);
  json += "}";

  server.send(200, "application/json", json);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("  Single Motor PID Test - ESP32");
  Serial.println("  Bidirectional PWM Control");
  
  // Configure motor PWM pins
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_LPWM, OUTPUT);

  // Setup PWM for both channels
  ledcAttach(MOTOR_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_LPWM, PWM_FREQ, PWM_RESOLUTION);

  // Initialize both to 0 (motor stopped)
  ledcWrite(MOTOR_RPWM, 0);
  ledcWrite(MOTOR_LPWM, 0);
  ledcWrite(RIGHT_MOTOR_RPWM, 0);
  ledcWrite(RIGHT_MOTOR_LPWM, 0);
  
  Serial.println("PWM configured:");
  Serial.println("  Motor 1 (Left):");
  Serial.println("    GPIO 6 = RPWM (Forward)");
  Serial.println("    GPIO 7 = LPWM (Reverse)");
  Serial.println("  Motor 2 (Right):");
  Serial.println("    GPIO 9 = RPWM (Forward)");
  Serial.println("    GPIO 10 = LPWM (Reverse)");
  
  // Configure encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR_R, RISING);

  Serial.println("Encoder configured");

  // ==================== TOF SENSOR SETUP ====================
  // Initialize I2C
  Serial.println(F("Initializing TOF sensors..."));
  Wire.begin(I2C_SDA, I2C_SCL);

  // Configure XSHUT pins for all TOF sensors
  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_RIGHT, OUTPUT);
  pinMode(TOF_XSHUT_RIGHT2, OUTPUT);

  // Shutdown all sensors initially
  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_RIGHT, LOW);
  digitalWrite(TOF_XSHUT_RIGHT2, LOW);
  delay(10);

  // Initialize front sensor (VL53L1X) - activate first
  Serial.println(F("Initializing front sensor (VL53L1X)..."));
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(10);
  if (!frontTOF.begin()) {
    Serial.println(F("Failed to boot VL53L1X - Check wiring!"));
    while(1);
  }

  // Start ranging on front sensor
  if (!frontTOF.startRanging()) {
    Serial.println(F("Failed to start ranging on front sensor"));
    while(1);
  }
  Serial.println(F("Front sensor (VL53L1X) ready!"));

  // Initialize right-front sensor (VL53L0X) - activate and set unique address
  Serial.println(F("Initializing right-front sensor (VL53L0X)..."));
  digitalWrite(TOF_XSHUT_RIGHT, HIGH);
  delay(10);
  if (!rightTOF.begin(0x30)) {  // Set unique I2C address
    Serial.println(F("Failed to boot right-front VL53L0X"));
    while(1);
  }
  Serial.println(F("Right-front sensor ready at 0x30!"));

  // Initialize right-back sensor (VL53L0X) - activate and set unique address
  Serial.println(F("Initializing right-back sensor (VL53L0X)..."));
  digitalWrite(TOF_XSHUT_RIGHT2, HIGH);
  delay(10);
  if (!right2TOF.begin(0x31)) {  // Set different I2C address
    Serial.println(F("Failed to boot right-back VL53L0X"));
    while(1);
  }
  Serial.println(F("Right-back sensor ready at 0x31!"));

  Serial.println(F("All TOF sensors initialized successfully!"));

  // ==================== MPU6050 SETUP ====================
  Serial.println(F("Initializing MPU6050..."));
  if (!mpu.begin()) {
    Serial.println(F("Failed to find MPU6050 chip"));
    while (1);
  }
  Serial.println(F("MPU6050 Found!"));

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Enable hardware low-pass filter to reduce high-frequency noise (~43Hz bandwidth)
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Calibrate gyro (read 100 samples to get offset)
  Serial.println(F("Calibrating gyro..."));
  float gyroSum = 0;
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroSum += g.gyro.z;
    delay(10);
  }
  gyroZOffset = gyroSum / 100.0;
  Serial.printf("Gyro Z offset: %.3f rad/s\n", gyroZOffset);

  // Initialize low-pass filter with calibrated zero value
  filteredGyroZ = 0.0;

  lastGyroTime = millis();

  Serial.println("Hardware configured!");

  // Configure static IP address
  IPAddress local_IP(192, 168, 0, 111);      // Static IP
  IPAddress gateway(192, 168, 0, 1);         // Gateway (usually your router)
  IPAddress subnet(255, 255, 255, 0);        // Subnet mask
  IPAddress primaryDNS(8, 8, 8, 8);          // Google DNS (optional)
  IPAddress secondaryDNS(8, 8, 4, 4);        // Google DNS (optional)

  // Configure ESP32 to use static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Static IP configuration failed!");
  }

  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 100) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Open http://192.168.0.111 in your browser");
  } else {
    Serial.println("WiFi connection failed!");
    Serial.println("Check SSID and password");
  }
  
  // Setup web server
  server.on("/", handleRoot);
  server.on("/setspeed", handleSetSpeed);
  server.on("/stop", handleStop);
  server.on("/setpid", handleSetPID);
  server.on("/status", handleStatus);
  server.on("/setwallfollow", handleSetWallFollow);
  server.on("/wallfollowmode", handleWallFollowMode);
  server.on("/setwallpid", handleSetWallPID);
  server.begin();
  
  Serial.println("HTTP server started");
  Serial.println("Ready for testing!");
  
  // Initialize timing
  lastControlUpdate = millis();
  lastSpeedCalc = millis();
  lastPrint = millis();
  lastTOFRead = millis();
}

// ==================== MAIN LOOP ====================
void loop() {
  server.handleClient();
  unsigned long currentTime = millis();

  // Read TOF sensors
  readTOFSensors();

  // Speed calculation at specified interval
  if (currentTime - lastSpeedCalc >= SPEED_CALC_PERIOD) {
    calculateSpeed();
  }

  // State machine (if wall-following mode enabled)
  if (currentState != STATE_IDLE) {
    updateStateMachine();
  }

  // Control loop at specified interval
  if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
    updateMotorControl();
    lastControlUpdate = currentTime;
  }

}
