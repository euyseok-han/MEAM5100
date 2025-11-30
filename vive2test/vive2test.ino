/*
 * Sample Vive code
 */
#include "vive510.h"

#define SIGNALPIN1 34 // pin receiving signal from Vive circuit

Vive510 vive1(SIGNALPIN1);

#define FREQ 1 // in Hz
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);

  vive1.begin();
  Serial.println("Vive trackers= started");
}
                 
uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;  
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else    middle = (a <= b) ? a : b;
  return middle;
}
                               
void loop() {  
  static uint16_t x,y;
  
  if (vive1.status() == VIVE_RECEIVING) {
    static uint16_t x0, y0, oldx1, oldx2, oldy1, oldy2;
    oldx2 = oldx1; oldy2 = oldy1;
    oldx1 = x0;     oldy1 = y0;
    
    x0 = vive1.xCoord();
    y0 = vive1.yCoord();
    x = med3filt(x0, oldx1, oldx2);
    y = med3filt(y0, oldy1, oldy2);
    digitalWrite(LED_BUILTIN,HIGH);
    if (x > 8000 || y > 8000 || x< 1000 || y < 1000) {
      x=0; y=0;
      digitalWrite(LED_BUILTIN,LOW);
    }
  }
  else {
    digitalWrite(LED_BUILTIN,LOW);
    x=0;
    y=0; 
    vive1.sync(5); 
  }
    
  delay(10);
}
