#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define I2C1_SDA            11   // GPIO 11 (SDA)
#define I2C1_SCL            12   // GPIO 12 (SCL)

// ==================== I2C BUSES ====================
TwoWire I2C1 = TwoWire(0);

// ==================== MPU6050 ====================
Adafruit_MPU6050 mpu;
float currentAngle = 0;         // degrees
float gyroZOffset = 0;          // rad/s offset converted later
unsigned long lastGyroTime = 0;

// Low-pass filter for gyro noise reduction
const float GYRO_ALPHA = 0.8;        // Low-pass filter coefficient (0.8 = smoother, less noise)
const float GYRO_DEADBAND = 0.5;     // Ignore readings below this threshold (deg/s)
float filteredGyroZ = 0.0;           // Filtered gyro Z value

// ==================== MPU6050 GYRO ====================
float readGyroZdeg() {
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
  if (dt <= 0) return;
  lastGyroTime = now;

  currentAngle += readGyroZdeg() * dt;
  while (currentAngle > 180) currentAngle -= 360;
  while (currentAngle < -180) currentAngle += 360;

  Serial.println(currentAngle);
}

void resetYaw() {
  currentAngle = 0;
  filteredGyroZ = 0.0;  // Reset low-pass filter
  lastGyroTime = millis();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C buses initialization
  I2C1.begin(I2C1_SDA, I2C1_SCL);

  delay(150);

    // MPU6050 on I2C1
  if (mpu.begin(0x68, &I2C1)) {
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
    Serial.println("MPU6050 initialized on I2C1");
  } else {
    Serial.println("MPU6050 not found on I2C1");
  }
}

void loop() {
  updateGyroIntegration();
  delay(10);
}
