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
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL53L1X.h>
#include "website.h"

// ==================== PIN DEFINITIONS ====================
#define LEFT_MOTOR         0   // Motor identifier
#define RIGHT_MOTOR        1   // Motor identifier

// Left Motor & Encoder
#define ENCODER_A          4   // GPIO 4
#define ENCODER_B          5   // GPIO 5
#define MOTOR_RPWM         0   // GPIO 0 - Forward
#define MOTOR_LPWM         1   // GPIO 1 - Reverse

// Right Motor & Encoder
#define RIGHT_ENCODER_A   10   // GPIO 10
#define RIGHT_ENCODER_B   19   // GPIO 19
#define RIGHT_MOTOR_RPWM   6   // GPIO 6 - Forward
#define RIGHT_MOTOR_LPWM   7   // GPIO 7 - Reverse

// TOF Sensors
#define I2C_SDA            8   // GPIO 8
#define I2C_SCL            9   // GPIO 9
#define TOF_XSHUT_FRONT    2   // GPIO 2 - VL53L1X
#define TOF_XSHUT_RIGHT    3   // GPIO 3 - VL53L0X

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

int frontDistance = 0;        // mm
int rightDistance = 0;        // mm

bool wallFollowMode = false;
int frontGoalDistance = 150;  // mm
int rightGoalDistance = 100;  // mm
float wallFollowSpeed = 40;   // RPM

float lastRightError = 0;     // For derivative
float wallFollowKp = 0.3;     // Proportional gain
float wallFollowKd = 0.5;     // Derivative gain

enum TurnState { NONE, TURNING_LEFT };
TurnState turnState = NONE;
unsigned long turnStartTime = 0;
const unsigned long TURN_DURATION = 1000;

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

    // VL53L0X right sensor
    VL53L0X_RangingMeasurementData_t measure;
    rightTOF.rangingTest(&measure, false);
    rightDistance = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 8190;

    Serial.printf("Front: %d mm | Right: %d mm\n", frontDistance, rightDistance);
    lastTOFRead = currentTime;
  }
}

// ==================== WALL FOLLOWING CONTROL ====================
void updateWallFollowing() {
  if (!wallFollowMode) return;

  // Obstacle detection - turn left
  if (frontDistance < frontGoalDistance) {
    targetSpeed = wallFollowSpeed;
    rightTargetSpeed = wallFollowSpeed;
    Serial.printf("Front obstacle at %d mm - Turning left\n", frontDistance);
    return;
  }

  // PD control for wall following
  float rightError = rightDistance - rightGoalDistance;
  float rightDerivative = rightError - lastRightError;
  float steeringCorrection = (wallFollowKp * rightError) + (wallFollowKd * rightDerivative);

  // Apply steering correction
  targetSpeed = -(wallFollowSpeed - steeringCorrection);
  rightTargetSpeed = wallFollowSpeed + steeringCorrection;

  lastRightError = rightError;

  Serial.printf("Wall-follow: Front=%dmm Right=%dmm Error=%.1f Correction=%.1f\n",
                frontDistance, rightDistance, rightError, steeringCorrection);
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
  Serial.println(F("Initializing TOF sensors..."));
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize front sensor (VL53L1X)
  Serial.println(F("Initializing front sensor (VL53L1X)..."));
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

  // Initialize right sensor (VL53L0X)
  Serial.println(F("Initializing right sensor (VL53L0X)..."));
  if (!rightTOF.begin()) {
    Serial.println(F("Failed to boot VL53L0X - Check wiring!"));
    while(1);
  }
  Serial.println(F("Right sensor (VL53L0X) ready!"));

  Serial.println(F("Both TOF sensors initialized successfully!"));

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
