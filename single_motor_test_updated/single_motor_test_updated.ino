/*
 * MEAM 5100 Lab 4.2 - Dual Motor Test with PID
 * ESP32-C3 with Encoder Feedback and PID Control
 *
 * LPWM = Reverse/Backward
 * RPWM = Forward
 */

#include <WiFi.h>
#include <WebServer.h>
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

const char* ssid = "Hphone";        // Change this
const char* password = "qqqq1234";     // Change this

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
  float Kp = 0.4;      // Proportional gain
  float Ki = 0.005;      // Integral gain
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

// ==================== TIMING VARIABLES ====================
unsigned long lastControlUpdate = 0;
unsigned long lastSpeedCalc = 0;
unsigned long lastPrint = 0;

const unsigned long CONTROL_PERIOD = 50;      // 50ms = 20Hz control loop
const unsigned long SPEED_CALC_PERIOD = 100;  // 100ms speed calculation
const unsigned long PRINT_PERIOD = 1000;      // 1000ms for serial output

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
  setMotorPWM(currentSpeed + leftPidOutput, LEFT_MOTOR);
  setMotorPWM(rightCurrentSpeed + rightPidOutput, RIGHT_MOTOR);
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
    currentSpeed = -1* delta / 1400.0 / dt; // 1400 counts per rev for 1:90 Motor
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
    targetSpeed = baseSpeed + steeringValue;
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
  json += "\"kd\":" + String(leftPID.Kd, 3);
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
  Serial.println("Hardware configured!");

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
    Serial.println("Open this IP in your browser");
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
  server.begin();
  
  Serial.println("HTTP server started");
  Serial.println("Ready for testing!");
  
  // Initialize timing
  lastControlUpdate = millis();
  lastSpeedCalc = millis();
  lastPrint = millis();
}

// ==================== MAIN LOOP ====================
void loop() {
  server.handleClient();
  unsigned long currentTime = millis();
  
  // Speed calculation at specified interval
  if (currentTime - lastSpeedCalc >= SPEED_CALC_PERIOD) {
    calculateSpeed();
  }
  
  // Control loop at specified interval
  if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
    updateMotorControl();
    lastControlUpdate = currentTime;
  }
  
  // // Print debug info to Serial Monitor
  // if (currentTime - lastPrint >= PRINT_PERIOD) {
  //   Serial.println("========== Motor Status ==========");
  //   Serial.printf("Base Speed: %.1f RPM | Steering: %.1f\n", baseSpeed, steeringValue);
  //   Serial.println("----------------------------------");
  //   Serial.printf("LEFT  - Target: %.1f | Current: %.1f | Error: %.1f | PWM: %.0f | Encoder: %ld\n",
  //                 targetSpeed, currentSpeed, leftPID.error, leftPID.output, encoderCount);
  //   Serial.printf("RIGHT - Target: %.1f | Current: %.1f | Error: %.1f | PWM: %.0f | Encoder: %ld\n",
  //                 rightTargetSpeed, rightCurrentSpeed, rightPID.error, rightPID.output, rightEncoderCount);
  //   lastPrint = currentTime;
  // }
}
