#include "vive510.h"
#include <math.h>

// ================= PIN SETUP ==================
#define SIGNALPIN_REAR   18  
#define SIGNALPIN_FRONT  10  

Vive510 viveRear(SIGNALPIN_REAR); 
Vive510 viveFront(SIGNALPIN_FRONT);

// ======== STRUCT TO HOLD COORDS =========
struct Coord {
  uint16_t x;
  uint16_t y;
};

// ============== MEDIAN FILTER ==============
uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;  
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else
    middle = (a <= b) ? a : b;
  return middle;
}

// ================= SETUP ==================
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  viveRear.begin();
  viveFront.begin();

  Serial.println("Vive trackers started!");
  delay(2000);
}

// ================= LOOP ==================
void loop() {

  // ===== static memory for filtering =====
  static uint16_t rx0, ry0, rx1, ry1, rx2, ry2;   // rear history
  static uint16_t fx0, fy0, fx1, fy1, fx2, fy2;   // front history

  Coord rear  = {0, 0};
  Coord front = {0, 0};

  bool rearValid  = false;
  bool frontValid = false;

  // ---------- REAR TRACKER ----------
  if (viveRear.status() == VIVE_RECEIVING) {
    rx2 = rx1;   ry2 = ry1;
    rx1 = rx0;   ry1 = ry0;

    rx0 = viveRear.xCoord();
    ry0 = viveRear.yCoord();

    rear.x = med3filt(rx0, rx1, rx2);
    rear.y = med3filt(ry0, ry1, ry2);

    if (!(rear.x > 8000 || rear.y > 8000 || rear.x < 1000 || rear.y < 1000)) {
      rearValid = true;
    } else {
      rear = {0, 0};
    }
  } else {
    viveRear.sync(5);           
  }

  // ---------- FRONT TRACKER ----------
  if (viveFront.status() == VIVE_RECEIVING) {
    fx2 = fx1;   fy2 = fy1;
    fx1 = fx0;   fy1 = fy0;

    fx0 = viveFront.xCoord();
    fy0 = viveFront.yCoord();

    front.x = med3filt(fx0, fx1, fx2);
    front.y = med3filt(fy0, fy1, fy2);

    if (!(front.x > 8000 || front.y > 8000 || front.x < 1000 || front.y < 1000)) {
      frontValid = true;
    } else {
      front = {0, 0};
    }
  } else {
    viveFront.sync(5);         
  }

  // ================== PRINT & ANGLE ==================
  if (rearValid && frontValid) {

    float dx = (float)front.x - (float)rear.x;
    float dy = (float)front.y - (float)rear.y;

    float angle_rad = atan2(dy, dx);
    float angle_deg = angle_rad * 180.0f / M_PI + 180;

    Serial.print("Rear: (");
    Serial.print(rear.x); Serial.print(", ");
    Serial.print(rear.y); Serial.print(")  ");

    Serial.print("Front: (");
    Serial.print(front.x); Serial.print(", ");
    Serial.print(front.y); Serial.print(")  ");

    Serial.print("Angle: ");
    Serial.print(angle_deg);
    Serial.println(" deg");

    digitalWrite(LED_BUILTIN, HIGH);

  } else {
    Serial.println("Tracking lost...");
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(300);
}
