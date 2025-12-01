#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "vive510.h"
#include "website.h"

// ==================== PIN DEFINITIONS ====================
#define LEFT_MOTOR         0
#define RIGHT_MOTOR        1

// Left motor
#define ENCODER_A          4
#define ENCODER_B          5
#define MOTOR_RPWM         6
#define MOTOR_LPWM         7

// Right motor
#define RIGHT_ENCODER_A   15
#define RIGHT_ENCODER_B   16
#define RIGHT_MOTOR_RPWM   9
#define RIGHT_MOTOR_LPWM   8

// Vive PINS (front/back)
#define VIVE_FRONT_PIN 10
#define VIVE_BACK_PIN  18

Vive510 viveFront(VIVE_FRONT_PIN);
Vive510 viveBack(VIVE_BACK_PIN);

WebServer server(80);

const char* ssid = "TP-Link_8A8C";
const char* password = "12488674";

// ==================== MOTOR & PID ====================
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long rightLastEncoderCount = 0;

float currentSpeed = 0;
float rightCurrentSpeed = 0;
float targetSpeed = 0;
float rightTargetSpeed = 0;

const int PWM_FREQ = 5000;
const int PWM_RES = 8;

struct PIDController {
  float Kp = 0.3;
  float Ki = 1.1;
  float Kd = 0.0;

  float error, lastError, integral, derivative;
  float output;

  float integralMax = 100;
  float integralMin = -100;
};

PIDController leftPID, rightPID;

// ==================== MPU6050 (removed) ====================

// ==================== VIVE FILTERS (dual sensors) ====================
uint16_t fx, fy, bx, by;        // filtered front/back positions
uint16_t fx0, fy0, fx1, fy1, fx2, fy2;
uint16_t bx0, by0, bx1, by1, bx2, by2;

// Robot pose from Vive
float robotX = 0, robotY = 0;
float robotHeading = 0; // radians

bool viveNavigationMode = false;
int viveTargetX = 0;
int viveTargetY = 0;

// ==================== TIMING ====================
unsigned long lastSpeedCalc = 0;
unsigned long lastControlUpdate = 0;

const unsigned long SPEED_CALC_PERIOD = 100;
const unsigned long CONTROL_PERIOD = 50;

// ==================== UTILS ====================
uint32_t med3(uint32_t a, uint32_t b, uint32_t c) {
  if ((a <= b) && (a <= c)) return (b <= c) ? b : c;
  else if ((b <= a) && (b <= c)) return (a <= c) ? a : c;
  else return (a <= b) ? a : b;
}

// ==================== ENCODER ISR ====================
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B)) encoderCount++;
  else encoderCount--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B)) rightEncoderCount++;
  else rightEncoderCount--;
}

// ==================== MOTOR CONTROL ====================
void setMotorPWM(int pwmValue, int motorSide) {
  pwmValue = pwmValue * 255 / 120;
  pwmValue = constrain(pwmValue, -255, 255);

  if (motorSide == RIGHT_MOTOR) {
    pwmValue = -pwmValue;
    if (pwmValue > 0) {
      ledcWrite(RIGHT_MOTOR_RPWM, pwmValue);
      ledcWrite(RIGHT_MOTOR_LPWM, 0);
    } else {
      ledcWrite(RIGHT_MOTOR_RPWM, 0);
      ledcWrite(RIGHT_MOTOR_LPWM, -pwmValue);
    }
    return;
  }

  if (pwmValue > 0) {
    ledcWrite(MOTOR_RPWM, pwmValue);
    ledcWrite(MOTOR_LPWM, 0);
  } else {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, -pwmValue);
  }
}

void stopMotor() {
  targetSpeed = rightTargetSpeed = 0;
  leftPID.integral = rightPID.integral = 0;
  leftPID.lastError = rightPID.lastError = 0;
}

// ==================== PID ====================
float calculatePID(PIDController &pid, float target, float current, float dt) {
  pid.error = target - current;
  pid.integral += pid.error * dt;
  pid.integral = constrain(pid.integral, pid.integralMin, pid.integralMax);

  pid.derivative = (pid.error - pid.lastError) / dt;
  pid.output = pid.Kp * pid.error + pid.Ki * pid.integral + pid.Kd * pid.derivative;
  pid.lastError = pid.error;

  return pid.output;
}

void updateMotorControl() {
  float dt = (millis() - lastControlUpdate) / 1000.0;
  if (dt <= 0) dt = CONTROL_PERIOD / 1000.0;

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
    long dl = encoderCount - lastEncoderCount;
    long dr = rightEncoderCount - rightLastEncoderCount;

    currentSpeed = dl / 1400.0 / dt * 1000 * 60;
    rightCurrentSpeed = dr / 1400.0 / dt * 1000 * 60;

    lastEncoderCount = encoderCount;
    rightLastEncoderCount = rightEncoderCount;
    lastSpeedCalc = now;
  }
}
// ==================== VIVE POSITION (dual sensors) ====================
void readDualVive() {
  // ======= FRONT SENSOR =======
  if (viveFront.status() != VIVE_RECEIVING) {
    fx = fy = 0;
    viveFront.sync(5);
  } else {
    fx2 = fx1; fy2 = fy1;
    fx1 = fx0; fy1 = fy0;

    fx0 = viveFront.xCoord();
    fy0 = viveFront.yCoord();

    fx = med3(fx0, fx1, fx2);
    fy = med3(fy0, fy1, fy2);

    if (fx < 1000 || fy < 1000 || fx > 8000 || fy > 8000)
      fx = fy = 0;
  }

  // ======= BACK SENSOR =======
  if (viveBack.status() != VIVE_RECEIVING) {
    bx = by = 0;
    viveBack.sync(5);
  } else {
    bx2 = bx1; by2 = by1;
    bx1 = bx0; by1 = by0;

    bx0 = viveBack.xCoord();
    by0 = viveBack.yCoord();

    bx = med3(bx0, bx1, bx2);
    by = med3(by0, by1, by2);

    if (bx < 1000 || by < 1000 || bx > 8000 || by > 8000)
      bx = by = 0;
  }
}

void computeVivePose() {
  if (fx == 0 || fy == 0 || bx == 0 || by == 0) return;

  // Position = midpoint of front/back markers
  robotX = (fx + bx) / 2.0;
  robotY = (fy + by) / 2.0;

  // Heading = angle from BACK → FRONT
  robotHeading = atan2(fy - by, fx - bx);
}

// ==================== VIVE NAVIGATION ====================
void viveGoToPoint() {
  if (!viveNavigationMode) return;
  if (robotX == 0 || robotY == 0) { stopMotor(); return; }

  float dx = viveTargetX - robotX;
  float dy = viveTargetY - robotY;

  float dist = sqrt(dx*dx + dy*dy);
  if (dist < 80) {
    stopMotor();
    viveNavigationMode = false;
    Serial.println("Reached target!");
    return;
  }

  float desired = atan2(dy, dx);
  float err = desired - robotHeading;

  while (err > 3.14) err -= 6.28;
  while (err < -3.14) err += 6.28;

  float turn = err * 150;      // turning
  float speed = dist * 0.05;   // forward speed

  turn = constrain(turn, -50, 50);
  speed = constrain(speed, 20, 80);

  targetSpeed = (speed - turn);
  rightTargetSpeed = (speed + turn);
}

// ==================== HTTP API ====================
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleGoToPoint() {
  if (server.hasArg("x") && server.hasArg("y")) {
    viveTargetX = server.arg("x").toInt();
    viveTargetY = server.arg("y").toInt();
    viveNavigationMode = true;

    server.send(200, "text/plain", "Moving to X=" + String(viveTargetX) +
                                   " Y=" + String(viveTargetY));
    Serial.printf("Vive Target: %d, %d\n", viveTargetX, viveTargetY);
  } else {
    server.send(400, "text/plain", "Missing x or y");
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.print()
  // Start dual Vive sensors
  viveFront.begin();
  viveBack.begin();

  // PWM
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_LPWM, OUTPUT);

  ledcAttach(MOTOR_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(RIGHT_MOTOR_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(RIGHT_MOTOR_LPWM, PWM_FREQ, PWM_RES);

  // encoders
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
  
  server.begin();

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
  server.on("/", handleRoot);
  server.on("/gotopoint", handleGoToPoint);
  lastSpeedCalc = millis();
  lastControlUpdate = millis();
}

// ==================== LOOP ====================
void loop() {
  server.handleClient();

  // Read Vive sensors and compute pose
  readDualVive();
  computeVivePose();

  if (viveNavigationMode) viveGoToPoint();

  if (millis() - lastSpeedCalc >= SPEED_CALC_PERIOD)
    calculateSpeed();

  if (millis() - lastControlUpdate >= CONTROL_PERIOD) {
    updateMotorControl();
    lastControlUpdate = millis();
  }
}
