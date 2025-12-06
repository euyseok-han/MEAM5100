#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// I2C 1 (Front TOF + Side Front TOF)
#define I2C_SDA            47   // GPIO 47 (SDA)
#define I2C_SCL            48   // GPIO 48 (SCL)

Adafruit_VL53L0X frontTOF = Adafruit_VL53L0X();   // Front TOF on I2C1
Adafruit_VL53L0X sideTOF1 = Adafruit_VL53L0X();   // Front Side TOF on I2C1
Adafruit_VL53L0X sideTOF2 = Adafruit_VL53L0X();   // Front Side TOF on I2C1

int frontDistance = 0;          // mm
int rightDistance1 = 0;         // mm (front-right)
int rightDistance2 = 0;         // mm (back-right)

void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  // Serial.print(bus);
  delay(2);
}

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
  TCA9548A(3);
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

  while (! Serial) {
    delay(1);
  }

  // I2C buses initialization
  Wire.begin(I2C_SDA, I2C_SCL);

  // power 
  Serial.println("Multiplexer Test");

  // Front TOF
  TCA9548A(0);
  if (!frontTOF.begin()) {
    Serial.println("VL53L0X front (I2C1) init failed!");
    while (1);
  } else {
    Serial.println("VL53L0X front initialized!");
  }

  // Side Front TOF
  TCA9548A(1);
  if (!sideTOF1.begin()) {
    Serial.println("VL53L0X side front (I2C1) init failed!");
    while (1);
  } else {
    Serial.println("VL53L0X side front initialized!");
  }

  // Side Back TOF
  TCA9548A(2);
  if (!sideTOF2.begin()) {
    Serial.println("VL53L0X side back (I2C1) init failed!");
    while (1);
  } else {
    Serial.println("VL53L0X side back initialized!");
  }

  // IMU
  TCA9548A(3);
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
    Serial.println("MPU6050 initialized on I2C1");
  } else {
    Serial.println("MPU6050 not found on I2C1");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  VL53L0X_RangingMeasurementData_t measureFront;
  VL53L0X_RangingMeasurementData_t measureRight1;
  VL53L0X_RangingMeasurementData_t measureRight2;

  TCA9548A(0);
  frontTOF.rangingTest(&measureFront, false); // pass in 'true' to get debug data printout!
  if (measureFront.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Front: "); Serial.print(measureFront.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
  delay(50);

  TCA9548A(1);
  sideTOF1.rangingTest(&measureRight1, false);
  if (measureRight1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("  Side Front: "); Serial.print(measureRight1.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
  delay(50);

  TCA9548A(2);
  sideTOF2.rangingTest(&measureRight2, false);
  if (measureRight2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("  Side Back: "); Serial.print(measureRight2.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
  delay(50);

  Serial.print("  Angle: ");
  updateGyroIntegration();
  delay(50);
}
