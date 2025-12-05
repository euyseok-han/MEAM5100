#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// I2C 1 (Front TOF + Side Front TOF)
#define I2C1_SDA            11   // GPIO 11 (SDA)
#define I2C1_SCL            12   // GPIO 12 (SCL)
#define TOF_XSHUT_RIGHT1    13   // VL53L0X (side front) XSHUT
#define TOF_XSHUT_FRONT     14   // VL53L1X (front) XSHUT

// I2C 2 (IMU + Side Back TOF)
#define I2C2_SDA            19   // GPIO 19 (SDA)
#define I2C2_SCL            20   // GPIO 20 (SCL)
#define TOF_XSHUT_RIGHT2    21   // VL53L0X (side back) XSHUT

TwoWire I2C1 = TwoWire(0);  // I2C bus 1 for Front TOF + Side Front TOF

Adafruit_VL53L0X frontTOF = Adafruit_VL53L0X();   // Front TOF on I2C1
Adafruit_VL53L0X sideTOF1 = Adafruit_VL53L0X();   // Front Side TOF on I2C1
// Adafruit_VL53L0X sideTOF2 = Adafruit_VL53L0X();   // Front Side TOF on I2C1

int frontDistance = 0;          // mm
int rightDistance1 = 0;         // mm (front-right)
int rightDistance2 = 0;         // mm (back-right)

// ==================== TIMING ====================
unsigned long lastControlUpdate = 0;
unsigned long lastSpeedCalc = 0;
unsigned long lastTOFRead = 0;

const unsigned long CONTROL_PERIOD = 50;      // ms
const unsigned long SPEED_CALC_PERIOD = 100;  // ms
const unsigned long TOF_READ_PERIOD = 50;     // ms

void setup() {
  Serial.begin(115200);

  while (! Serial) {
    delay(1);
  }

  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_RIGHT1, OUTPUT);
  // pinMode(TOF_XSHUT_RIGHT2, OUTPUT);

  // I2C buses initialization
  I2C1.begin(I2C1_SDA, I2C1_SCL);

  // // Shutdown all sensors initially
  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_RIGHT1, LOW);
  // digitalWrite(TOF_XSHUT_RIGHT2, LOW);
  delay(10);

  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  Serial.println("Adafruit VL53L0X test");
  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(10);
  if (!frontTOF.begin(0x30,false,&I2C1)) {
    Serial.println("VL53L0X front (I2C1) init failed!");
  } else {
    Serial.println("VL53L0X front initialized on I2C1");
  }

  digitalWrite(TOF_XSHUT_RIGHT1, HIGH);
  delay(10);
  if (!sideTOF1.begin(0x31,false,&I2C1)) {
    Serial.println("VL53L0X side front (I2C1) init failed!");
  } else {
    Serial.println("VL53L0X side front initialized on I2C1");
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  VL53L0X_RangingMeasurementData_t measureFront;
  VL53L0X_RangingMeasurementData_t measureRight1;
  // VL53L0X_RangingMeasurementData_t measureRight2;
    
  Serial.print("Reading a measurement... ");
  frontTOF.rangingTest(&measureFront, false); // pass in 'true' to get debug data printout!
  sideTOF1.rangingTest(&measureRight1, false);

  if (measureFront.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Front Distance (mm): "); Serial.println(measureFront.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }

  delay(100);

  if (measureRight1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Side Front Distance (mm): "); Serial.println(measureRight1.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
}
