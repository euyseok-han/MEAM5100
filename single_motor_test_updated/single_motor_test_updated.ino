/*
 * MEAM 5100 Lab 4.2 - Dual Motor Test with PID
 * ESP32-C3 with Encoder Feedback and PID Control
 *
 * LPWM = Reverse/Backward
 * RPWM = Forward
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include "website.h"

// ==================== PIN DEFINITIONS ====================
// Motor Side Identifiers
#define LEFT_MOTOR         0   // Left motor identifier
#define RIGHT_MOTOR        1   // Right motor identifier

// Motor 1 (Left Wheel)
#define ENCODER_A          4   // Left encoder channel A
#define ENCODER_B          5   // Left encoder channel B
#define MOTOR_RPWM         0   // Left motor RPWM = Forward direction
#define MOTOR_LPWM         1   // Left motor LPWM = Reverse direction

// Motor 2 (Right Wheel)
#define RIGHT_ENCODER_A   10   // Right encoder channel A
#define RIGHT_ENCODER_B   19   // Right encoder channel B
#define RIGHT_MOTOR_RPWM   6   // Right motor RPWM = Forward direction
#define RIGHT_MOTOR_LPWM   7   // Right motor LPWM = Reverse direction

// TOF Sensors (I2C)
#define I2C_SDA            8   // I2C Data pin
#define I2C_SCL            9   // I2C Clock pin
#define TOF_XSHUT_FRONT    2   // VL53L0X (front) shutdown pin
#define TOF_XSHUT_RIGHT    3   // VL53L1X (right) shutdown pin

const char* ssid = "TP-Link_8A8C";        // Change this
const char* password = "12488674";        // Change this

WebServer server(80);

// ==================== MOTOR & ENCODER VARIABLES ====================
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long rightLastEncoderCount = 0;
// Speed calculation
float currentSpeed = 0;  // counts per second
float targetSpeed = 0;   // desired speed (positive=forward, negative=reverse)
float rightCurrentSpeed = 0;
float rightTargetSpeed = 0; // desired speed for the right wheel

// Speed + Steering control variables
float baseSpeed = 0;     // Base forward/backward speed
float steeringValue = 0; // Steering amount (negative=left, positive=right)

// PWM settings
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;  // 8-bit resolution (0-255)

// ==================== PID PARAMETERS ====================
struct PIDController {
  float Kp = 0.3;      // Proportional gain
  float Ki = 1.1;      // Integral gain
  float Kd = 0.0;      // Derivative gain
  
  float error = 0;
  float lastError = 0;
  float integral = 0;
  float derivative = 0;
  float output = 0;
  
  // Anti-windup limits
  float integralMax = 100;
  float integralMin = -100;
};

PIDController leftPID;   // PID controller for left motor
PIDController rightPID;  // PID controller for right motor

// ==================== TOF SENSORS ====================
VL53L0X frontTOF;  // Front sensor (VL53L0X)
VL53L1X rightTOF;  // Right sensor (VL53L1X)

int frontDistance = 0;     // Front distance in mm
int rightDistance = 0;     // Right distance in mm

// Wall-following control variables
bool wallFollowMode = false;     // Enable/disable wall-following
int frontGoalDistance = 150;     // Goal distance to front wall (mm)
int rightGoalDistance = 100;     // Goal distance to right wall (mm)
float wallFollowSpeed = 40;      // Base speed for wall following (RPM)

// Wall-following PD control
float lastRightError = 0;        // Previous error for derivative calculation
float wallFollowKp = 0.3;        // Proportional gain
float wallFollowKd = 0.5;        // Derivative gain

// Turn state tracking
enum TurnState { NONE, TURNING_LEFT };
TurnState turnState = NONE;
unsigned long turnStartTime = 0;
const unsigned long TURN_DURATION = 1000; // Time to turn 90° (adjust based on testing)

// ==================== TIMING VARIABLES ====================
unsigned long lastControlUpdate = 0;
unsigned long lastSpeedCalc = 0;
unsigned long lastPrint = 0;
unsigned long lastTOFRead = 0;

const unsigned long CONTROL_PERIOD = 50;      // 50ms = 20Hz control loop
const unsigned long SPEED_CALC_PERIOD = 100;  // 100ms speed calculation
const unsigned long TOF_READ_PERIOD = 50;     // 50ms TOF sensor read interval

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

// ==================== TOF SENSOR READING ====================
void readTOFSensors() {
  unsigned long currentTime = millis();

  if (currentTime - lastTOFRead >= TOF_READ_PERIOD) {
    // Read front sensor (VL53L0X)
    frontDistance = frontTOF.readRangeContinuousMillimeters();
    if (frontTOF.timeoutOccurred()) {
      Serial.println("Front TOF timeout!");
      frontDistance = 8190; // Max range on timeout
    }

    // Read right sensor (VL53L1X)
    rightDistance = rightTOF.read();
    if (rightTOF.timeoutOccurred()) {
      Serial.println("Right TOF timeout!");
      rightDistance = 4000; // Max range on timeout
    }

    lastTOFRead = currentTime;
  }
}

// ==================== WALL FOLLOWING CONTROL ====================
void updateWallFollowing() {
  if (!wallFollowMode) return;

  // Handle ongoing turn
  if (turnState == TURNING_LEFT) {
    unsigned long currentTime = millis();
    if (currentTime - turnStartTime < TURN_DURATION) {
      // Continue turning left (spin in place)
      targetSpeed = wallFollowSpeed;    // Left wheel forward
      rightTargetSpeed = wallFollowSpeed; // Right wheel forward (same direction = turn left)
      Serial.printf("Turning left... %lu ms remaining\n", TURN_DURATION - (currentTime - turnStartTime));
      return;
    } else {
      // Turn complete
      turnState = NONE;
      lastRightError = 0; // Reset derivative term after turn
      Serial.println("Turn complete - resuming wall following");
    }
  }

  // Check if obstacle is too close in front
  if (frontDistance < frontGoalDistance) {
    // Obstacle detected - initiate 90° left turn
    Serial.printf("Front obstacle detected: %d mm (goal: %d mm) - Turning left\n", frontDistance, frontGoalDistance);
    turnState = TURNING_LEFT;
    turnStartTime = millis();
    lastRightError = 0; // Reset derivative term
    return;
  }

  // Normal wall-following control
  // Calculate error from right wall
  float rightError = rightDistance - rightGoalDistance;

  // Calculate derivative (rate of change of error)
  // Positive derivative = distance increasing (moving away from wall)
  // Negative derivative = distance decreasing (approaching wall)
  float errorDerivative = rightError - lastRightError;

  // PD control for steering correction
  // Proportional: corrects based on current distance error
  //   - Positive error (too far) -> steer right (negative correction)
  //   - Negative error (too close) -> steer left (positive correction)
  // Derivative: corrects based on rate of change
  //   - Moving away from wall -> steer right more aggressively
  //   - Approaching wall -> steer left more aggressively
  float steeringCorrection = -(wallFollowKp * rightError + wallFollowKd * errorDerivative);
  steeringCorrection = constrain(steeringCorrection, -60, 60);

  // Set motor speeds: move forward while correcting for wall distance
  targetSpeed = -wallFollowSpeed - steeringCorrection;
  rightTargetSpeed = wallFollowSpeed - steeringCorrection;

  // Constrain to safe limits
  targetSpeed = constrain(targetSpeed, -120, 120);
  rightTargetSpeed = constrain(rightTargetSpeed, -120, 120);

  // Update last error for next iteration
  lastRightError = rightError;

  // Debug output
  Serial.printf("Wall: Dist=%dmm, Goal=%dmm, Err=%.1f, dErr=%.1f, Steer=%.1f\n",
                rightDistance, rightGoalDistance, rightError, errorDerivative, steeringCorrection);
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
  if (server.hasArg("frontGoal") && server.hasArg("rightGoal")) {
    frontGoalDistance = server.arg("frontGoal").toInt();
    rightGoalDistance = server.arg("rightGoal").toInt();

    server.send(200, "text/plain", "Wall-following goals updated: Front=" + String(frontGoalDistance) + "mm, Right=" + String(rightGoalDistance) + "mm");
    Serial.printf("Wall goals updated: Front=%d mm, Right=%d mm\n", frontGoalDistance, rightGoalDistance);
  } else {
    server.send(400, "text/plain", "Missing frontGoal or rightGoal parameter");
  }
}

void handleSetWallPID() {
  if (server.hasArg("kp") && server.hasArg("kd")) {
    wallFollowKp = server.arg("kp").toFloat();
    wallFollowKd = server.arg("kd").toFloat();

    server.send(200, "text/plain", "Wall-following PD updated: Kp=" + String(wallFollowKp, 2) + ", Kd=" + String(wallFollowKd, 2));
    Serial.printf("Wall PD updated: Kp=%.2f, Kd=%.2f\n", wallFollowKp, wallFollowKd);
  } else {
    server.send(400, "text/plain", "Missing kp or kd parameter");
  }
}

void handleWallFollowMode() {
  if (server.hasArg("enable")) {
    wallFollowMode = (server.arg("enable") == "true" || server.arg("enable") == "1");

    if (wallFollowMode) {
      Serial.println("Wall-following mode ENABLED");
      server.send(200, "text/plain", "Wall-following mode enabled");
    } else {
      Serial.println("Wall-following mode DISABLED");
      stopMotor();
      server.send(200, "text/plain", "Wall-following mode disabled");
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
  json += "\"rightDist\":" + String(rightDistance) + ",";
  json += "\"frontGoal\":" + String(frontGoalDistance) + ",";
  json += "\"rightGoal\":" + String(rightGoalDistance) + ",";
  json += "\"wallFollowMode\":" + String(wallFollowMode ? "true" : "false") + ",";

  // Wall-following PD parameters
  json += "\"wallKp\":" + String(wallFollowKp, 2) + ",";
  json += "\"wallKd\":" + String(wallFollowKd, 2);
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
  Serial.println("    GPIO 0 = RPWM (Forward)");
  Serial.println("    GPIO 1 = LPWM (Reverse)");
  Serial.println("  Motor 2 (Right):");
  Serial.println("    GPIO 6 = RPWM (Forward)");
  Serial.println("    GPIO 7 = LPWM (Reverse)");
  
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
  Wire.begin(I2C_SDA, I2C_SCL);

  // Configure shutdown pins
  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_RIGHT, OUTPUT);

  // Reset both sensors
  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_RIGHT, LOW);
  delay(10);

  // Initialize front sensor (VL53L0X) first
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(10);

  frontTOF.setTimeout(500);
  if (!frontTOF.init()) {
    Serial.println("Failed to initialize front TOF sensor (VL53L0X)!");
  } else {
    frontTOF.setAddress(0x30); // Change I2C address to avoid conflict
    frontTOF.startContinuous(50); // Continuous mode, 50ms timing budget
    Serial.println("Front TOF sensor (VL53L0X) configured at address 0x30");
  }

  // Initialize right sensor (VL53L1X)
  digitalWrite(TOF_XSHUT_RIGHT, HIGH);
  delay(10);

  rightTOF.setTimeout(500);
  if (!rightTOF.init()) {
    Serial.println("Failed to initialize right TOF sensor (VL53L1X)!");
  } else {
    rightTOF.setAddress(0x31); // Change I2C address
    rightTOF.setDistanceMode(VL53L1X::Short); // Short range mode for better accuracy
    rightTOF.setMeasurementTimingBudget(50000); // 50ms timing budget
    rightTOF.startContinuous(50);
    Serial.println("Right TOF sensor (VL53L1X) configured at address 0x31");
  }

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

  // Wall-following control (if enabled)
  if (wallFollowMode) {
    updateWallFollowing();
  }

  // Control loop at specified interval
  if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
    updateMotorControl();
    lastControlUpdate = currentTime;
  }

}
