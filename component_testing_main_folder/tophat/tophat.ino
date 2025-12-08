#include <Wire.h>
#define SDA_PIN 5 // Custom SDA
#define SCL_PIN 4 // Custom SCL
const uint8_t *msg = (const uint8_t*)"Hello from Master";
void setup() {
    Wire.begin(SDA_PIN, SCL_PIN); // Master 2 par
    Serial.begin(115200);
}
void loop() {
    Wire.beginTransmission(8); // Slave address
    Wire.write(msg, sizeof(msg));
    Wire.endTransmission();
    Serial.println("Sent message to slave");
    delay(1000);
}