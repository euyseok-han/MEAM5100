/*
 * ESP32-S3 Dual Motor Robot – VIVE Navigation Version
 * Wall-following code removed entirely.
 * Uses Vive510 XY coordinates + gyro heading + PID wheel control.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
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
#define RIGHT_MOTOR_LPWM  10

// MPU6050
#define I2C_SDA            8
#define I2C_SCL           18

// Vive PIN
#define VIVE_PIN          34
Vive510 vive(VIVE_PIN);

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

// ==================== MPU6050 ====================
Adafruit_MPU6050 mpu;
float currentAngle = 0;
float gyroZOffset = 0;
unsigned long lastGyroTime = 0;

// ==================== VIVE FILTERS ====================
uint16_t vx, vy;
uint16_t vx0, vy0, vx1, vy1, vx2, vy2;

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

// ==================== GYRO ====================
float readGyroZ() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  return (g.gyro.z - gyroZOffset) * 57.2958;
}

void updateGyro() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  if (dt <= 0) return;

  lastGyroTime = now;

  currentAngle += readGyroZ() * dt;
  while (currentAngle > 180) currentAngle -= 360;
  while (currentAngle < -180) currentAngle += 360;
}

void resetYaw() {
  currentAngle = 0;
  lastGyroTime = millis();
}

// ==================== VIVE POSITION ====================
void readVivePosition() {
  if (vive.status() != VIVE_RECEIVING) {
    vx = vy = 0;
    vive.sync(5);
    return;
  }

  vx2 = vx1; vy2 = vy1;
  vx1 = vx0; vy1 = vy0;

  vx0 = vive.xCoord();
  vy0 = vive.yCoord();

  vx = med3(vx0, vx1, vx2);
  vy = med3(vy0, vy1, vy2);

  if (vx > 8000 || vy > 8000 || vx < 1000 || vy < 1000) {
    vx = vy = 0;
  }
}

// ==================== VIVE NAVIGATION ====================
void viveGoToPoint() {
  if (!viveNavigationMode) return;
  if (vx == 0 || vy == 0) { stopMotor(); return; }

  int dx = viveTargetX - vx;
  int dy = viveTargetY - vy;

  float dist = sqrt(dx*dx + dy*dy);

  if (dist < 80) {
    stopMotor();
    viveNavigationMode = false;
    Serial.println("Reached Vive target.");
    return;
  }

  float desired = atan2(dy, dx);
  float heading = currentAngle * 3.14159/180.0;
  float err = desired - heading;

  while (err > 3.14) err -= 6.28;
  while (err < -3.14) err += 6.28;

  float Kp_heading = 180;
  float Kp_dist = 0.06;

  float turn = err * Kp_heading;
  float speed = dist * Kp_dist;

  turn = constrain(turn, -50, 50);
  speed = constrain(speed, 20, 80);

  targetSpeed = -(speed - turn);
  rightTargetSpeed = (speed + turn);
}

// ==================== HTTP API ====================
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleGoToPoint() {
  if (server.hasArg("x") && server.hasArg("y")) {
    viveTargetX = server.arg("x").toInt();
    viveTargetY = server.arg("y").toInt();
    viveNavigationMode = true;

    resetYaw();

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

  vive.begin();

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

  // I2C + MPU
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) while (1) Serial.println("MPU FAIL");

  delay(100);
  float sum=0;
  for (int i=0;i<100;i++){
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    sum+=g.gyro.z;
    delay(5);
  }
  gyroZOffset=sum/100;

  lastGyroTime = millis();

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status()!=WL_CONNECTED) { delay(200); }

  server.on("/", handleRoot);
  server.on("/gotopoint", handleGoToPoint);
  server.begin();

  lastSpeedCalc = millis();
  lastControlUpdate = millis();
}

// ==================== LOOP ====================
void loop() {
  server.handleClient();

  updateGyro();
  readVivePosition();

  if (viveNavigationMode) viveGoToPoint();

  if (millis() - lastSpeedCalc >= SPEED_CALC_PERIOD)
    calculateSpeed();

  if (millis() - lastControlUpdate >= CONTROL_PERIOD) {
    updateMotorControl();
    lastControlUpdate = millis();
  }
}
