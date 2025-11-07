/*
 * MEAM 5100 Lab 4.2 - Single Motor Test with PID
 * ESP32 with Encoder Feedback and PID Control
 * 
 * Updated for bidirectional PWM motor driver (like BTS7960, IBT-2)
 * LPWM = Reverse/Backward
 * RPWM = Forward
 * 
 * YOUR PIN CONFIGURATION:
 * Encoder A Output: GPIO 1
 * Encoder B Output: GPIO 4
 * RPWM (Forward): GPIO 18
 * LPWM (Reverse): GPIO 19
 */

#include <WiFi.h>
#include <WebServer.h>

// ==================== PIN DEFINITIONS ====================
// Single Motor Test Configuration
#define ENCODER_A          1   // Encoder channel A
#define ENCODER_B          4   // Encoder channel B
#define MOTOR_RPWM        18   // Right PWM = Forward direction
#define MOTOR_LPWM        19   // Left PWM = Reverse direction

// ==================== WIFI CONFIGURATION ====================
const char* ssid = "Hphone";        // Change this
const char* password = "qqqq1234";     // Change this

WebServer server(80);

// ==================== MOTOR & ENCODER VARIABLES ====================
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0;

// Speed calculation
float currentSpeed = 0;  // counts per second
float targetSpeed = 0;   // desired speed (positive=forward, negative=reverse)

// PWM settings
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;  // 8-bit resolution (0-255)

// ==================== PID PARAMETERS ====================
struct PIDController {
  float Kp = 2.0;      // Proportional gain
  float Ki = 0.5;      // Integral gain
  float Kd = 0.1;      // Derivative gain
  
  float error = 0;
  float lastError = 0;
  float integral = 0;
  float derivative = 0;
  float output = 0;
  
  // Anti-windup limits
  float integralMax = 100;
  float integralMin = -100;
};

PIDController pid;

// ==================== TIMING VARIABLES ====================
unsigned long lastControlUpdate = 0;
unsigned long lastSpeedCalc = 0;
unsigned long lastPrint = 0;

const unsigned long CONTROL_PERIOD = 20;      // 20ms = 50Hz control loop
const unsigned long SPEED_CALC_PERIOD = 100;  // 100ms speed calculation
const unsigned long PRINT_PERIOD = 200;       // 200ms for serial output

// ==================== ENCODER ISR ====================
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ==================== MOTOR CONTROL ====================
// For bidirectional PWM driver:
// - RPWM active (LPWM=0) = Forward
// - LPWM active (RPWM=0) = Reverse
// - Both 0 = Stop
// - Never activate both at same time!

void setMotorPWM(int pwmValue) {
  // pwmValue can be positive (forward) or negative (reverse)
  pwmValue = constrain(pwmValue, -255, 255);
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
  ledcWrite(MOTOR_RPWM, 0);
  ledcWrite(MOTOR_LPWM, 0);
  targetSpeed = 0;
  
  // Reset PID
  pid.integral = 0;
  pid.lastError = 0;
}

// ==================== PID CONTROL ====================
float calculatePID(float target, float current) {
  // Calculate error
  pid.error = target - current;
  
  // Integral with anti-windup
  pid.integral += pid.error;
  pid.integral = constrain(pid.integral, pid.integralMin, pid.integralMax);
  
  // Derivative
  pid.derivative = pid.error - pid.lastError;
  
  // PID output
  pid.output = (pid.Kp * pid.error) + 
               (pid.Ki * pid.integral) + 
               (pid.Kd * pid.derivative);
  
  // Update last error
  pid.lastError = pid.error;
  
  return pid.output;
}

void updateMotorControl() {
  // Calculate PID output (can be positive or negative)
  float pidOutput = calculatePID(targetSpeed, currentSpeed);
  // Apply to motor (automatically handles direction)
  setMotorPWM(pwm);
}

// ==================== SPEED CALCULATION ====================
void calculateSpeed() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastSpeedCalc) / 1000.0; // Convert to seconds
  
  if (dt > 0) {
    // Calculate speed in encoder counts per second
    long delta = encoderCount - lastEncoderCount;
    currentSpeed = delta / dt;
    
    // Update last values
    lastEncoderCount = encoderCount;
    lastSpeedCalc = currentTime;
  }
}

// ==================== WEB SERVER HANDLERS ====================
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Single Motor PID Test</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { 
      font-family: Arial; 
      text-align: center; 
      margin: 20px; 
      background-color: #f0f0f0; 
    }
    .container { 
      max-width: 500px; 
      margin: auto; 
      background: white; 
      padding: 20px; 
      border-radius: 10px; 
      box-shadow: 0 2px 10px rgba(0,0,0,0.1);
    }
    h1 { color: #333; }
    .control-section { 
      margin: 20px 0; 
      padding: 15px; 
      background: #f9f9f9; 
      border-radius: 5px; 
    }
    button { 
      padding: 15px 30px; 
      margin: 5px;
      font-size: 16px; 
      border: none; 
      border-radius: 5px; 
      cursor: pointer; 
      background-color: #4CAF50; 
      color: white; 
    }
    button:active { background-color: #45a049; }
    .stop-btn { background-color: #f44336; }
    .reverse-btn { background-color: #FF9800; }
    .status { 
      margin: 20px 0; 
      padding: 15px; 
      background: #e7f3ff; 
      border-radius: 5px; 
      font-family: monospace;
    }
    .speed-control { margin: 20px 0; }
    input[type="range"] { width: 100%; }
    .speed-value { 
      font-size: 24px; 
      font-weight: bold; 
      color: #2196F3; 
    }
    .pid-section {
      margin: 20px 0;
      padding: 15px;
      background: #fff3cd;
      border-radius: 5px;
    }
    .pid-input {
      margin: 10px 0;
      text-align: left;
    }
    .pid-input label {
      display: inline-block;
      width: 80px;
    }
    .pid-input input {
      width: 100px;
      padding: 5px;
    }
    .preset-btn {
      background-color: #2196F3;
      padding: 8px 15px;
      font-size: 14px;
      margin: 3px;
    }
    .direction-indicator {
      display: inline-block;
      padding: 5px 15px;
      border-radius: 5px;
      font-weight: bold;
      margin-left: 10px;
    }
    .forward { background-color: #4CAF50; color: white; }
    .reverse { background-color: #FF9800; color: white; }
    .stopped { background-color: #999; color: white; }
  </style>
</head>
<body>
  <div class="container">
    <h1>🔧 Single Motor PID Test</h1>
    <h3>Bidirectional Control (Forward/Reverse)</h3>
    
    <div class="control-section">
      <h3>Speed Control <span id="direction" class="direction-indicator stopped">STOPPED</span></h3>
      <div class="speed-control">
        <label>Target Speed: <span id="speedValue" class="speed-value">0</span> counts/s</label>
        <input type="range" id="speedSlider" min="-150" max="150" value="0" step="10" oninput="setSpeed(this.value)">
        <div style="display: flex; justify-content: space-between; font-size: 12px; color: #666;">
          <span>← Reverse</span>
          <span>Forward →</span>
        </div>
      </div>
      
      <h4>Forward Direction</h4>
      <button onclick="setSpeed(30)">Slow (30)</button>
      <button onclick="setSpeed(60)">Medium (60)</button>
      <button onclick="setSpeed(100)">Fast (100)</button>
      <button onclick="setSpeed(250)">Fast (250)</button>

      
      <h4>Reverse Direction</h4>
      <button class="reverse-btn" onclick="setSpeed(-30)">Slow (-30)</button>
      <button class="reverse-btn" onclick="setSpeed(-60)">Medium (-60)</button>
      <button class="reverse-btn" onclick="setSpeed(-100)">Fast (-100)</button>
      <button class="reverse-btn" onclick="setSpeed(-250)">Fast (-100)</button>
      
      <br><br>
      <button class="stop-btn" onclick="stopMotor()">⏹️ STOP</button>
    </div>
    
    <div class="pid-section">
      <h3>PID Tuning</h3>
      <div class="pid-input">
        <label>Kp:</label>
        <input type="number" id="kp" value="2.0" step="0.1" onchange="updatePID()">
      </div>
      <div class="pid-input">
        <label>Ki:</label>
        <input type="number" id="ki" value="0.5" step="0.1" onchange="updatePID()">
      </div>
      <div class="pid-input">
        <label>Kd:</label>
        <input type="number" id="kd" value="0.1" step="0.01" onchange="updatePID()">
      </div>
      <button class="preset-btn" onclick="setPIDPreset(1)">P-only</button>
      <button class="preset-btn" onclick="setPIDPreset(2)">PI</button>
      <button class="preset-btn" onclick="setPIDPreset(3)">PID</button> b
    </div>
    
    <div class="status">
      <h3>Status</h3>
      <p><strong>Target Speed:</strong> <span id="targetSpeed">0</span> counts/s</p>
      <p><strong>Current Speed:</strong> <span id="currentSpeed">0</span> counts/s</p>
      <p><strong>Error:</strong> <span id="error">0</span> counts/s</p>
      <p><strong>PWM Output:</strong> <span id="pwm">0</span> / 255</p>
      <p><strong>Active Channel:</strong> <span id="activeChannel">None</span></p>
      <p><strong>Encoder Count:</strong> <span id="encoder">0</span></p>
      <p><strong>PID:</strong> Kp=<span id="displayKp">2.0</span> Ki=<span id="displayKi">0.5</span> Kd=<span id="displayKd">0.1</span></p>
    </div>
    
    <div style="margin-top: 20px; padding: 10px; background: #ffe6e6; border-radius: 5px;">
      <strong>📝 Bidirectional Motor Tips:</strong><br>
      1. Positive speed = Forward (RPWM active)<br>
      2. Negative speed = Reverse (LPWM active)<br>
      3. Test both directions work correctly<br>
      4. Tune PID in forward direction first<br>
      5. Verify encoder counts correctly in both directions
    </div>
  </div>
  
  <script>
    let currentTargetSpeed = 0;
                                                                                                                                                                                      
    function updateSpeed(value) {
      console.log("updateSpeed is called");
      currentTargetSpeed = parseInt(value);
      setSpeed(currentTargetSpeed);
      document.getElementById('speedValue').textContent = value;
      updateDirectionIndicator(currentTargetSpeed);
    }
    
    function updateDirectionIndicator(speed) {
      const indicator = document.getElementById('direction');
      if (speed > 0) {
        indicator.textContent = 'FORWARD';
        indicator.className = 'direction-indicator forward';
      } else if (speed < 0) {
        indicator.textContent = 'REVERSE';
        indicator.className = 'direction-indicator reverse';
      } else {
        indicator.textContent = 'STOPPED';
        indicator.className = 'direction-indicator stopped';
      }
    }
    
    function setSpeed(speed) {
      currentTargetSpeed = speed;
      document.getElementById('speedValue').textContent = speed;
      document.getElementById('speedSlider').value = speed;
      updateDirectionIndicator(speed);
      
      fetch('/setspeed?speed=' + speed)
        .then(response => response.text())
        .then(data => console.log(data));
    }
    
    function stopMotor() {
      currentTargetSpeed = 0;
      document.getElementById('speedValue').textContent = 0;
      document.getElementById('speedSlider').value = 0;
      updateDirectionIndicator(0);
      
      fetch('/stop')
        .then(response => response.text())
        .then(data => console.log(data));
    }
    
    function updatePID() {
      let kp = document.getElementById('kp').value;
      let ki = document.getElementById('ki').value;
      let kd = document.getElementById('kd').value;
      
      fetch('/setpid?kp=' + kp + '&ki=' + ki + '&kd=' + kd)
        .then(response => response.text())
        .then(data => console.log(data));
    }
    
    function setPIDPreset(preset) {
      let kp, ki, kd;
      if (preset === 1) {  // P-only
        kp = 2.0; ki = 0.0; kd = 0.0;
      } else if (preset === 2) {  // PI
        kp = 2.0; ki = 0.5; kd = 0.0;
      } else {  // PID
        kp = 2.0; ki = 0.5; kd = 0.1;
      }
      
      document.getElementById('kp').value = kp;
      document.getElementById('ki').value = ki;
      document.getElementById('kd').value = kd;
      updatePID();
    }
    
    // Keyboard controls
    document.addEventListener('keydown', function(e) {
      switch(e.key) {
        case 'ArrowUp': 
        case 'w':
          setSpeed(Math.min(currentTargetSpeed + 10, 150)); 
          break;
        case 'ArrowDown': 
        case 's':
          setSpeed(Math.max(currentTargetSpeed - 10, -150)); 
          break;
        case ' ': 
          stopMotor(); 
          e.preventDefault(); 
          break;
        case '1': setSpeed(30); break;
        case '2': setSpeed(60); break;
        case '3': setSpeed(100); break;
        case '4': setSpeed(-30); break;
        case '5': setSpeed(-60); break;
        case '6': setSpeed(-100); break;
      }
    });
  </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

void handleSetSpeed() {
  if (server.hasArg("speed")) {
    targetSpeed = server.arg("speed").toFloat();
    server.send(200, "text/plain", "Speed set to " + String(targetSpeed));
    Serial.printf("Target speed set to: %.1f (", targetSpeed);
    if (targetSpeed > 0) {
      Serial.print("FORWARD"); // debug point
      ledcWrite(MOTOR_RPWM, targetSpeed);
      ledcWrite(MOTOR_LPWM, 0);
      }
    else if (targetSpeed < 0) {
      Serial.print("REVERSE");
      ledcWrite(MOTOR_RPWM, 0);
      ledcWrite(MOTOR_LPWM, -1 * targetSpeed);
      }
    else Serial.print("STOP");
    Serial.println(")");
  } else {
    server.send(400, "text/plain", "Missing speed parameter");
  }
}

void handleStop() {
  stopMotor();
  server.send(200, "text/plain", "Motor stopped");
  Serial.println("Motor stopped");
}

void handleSetPID() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    pid.Kp = server.arg("kp").toFloat();
    pid.Ki = server.arg("ki").toFloat();
    pid.Kd = server.arg("kd").toFloat();
    
    // Reset integral when changing parameters
    pid.integral = 0;
    
    server.send(200, "text/plain", "PID updated");
    Serial.printf("PID updated: Kp=%.2f Ki=%.2f Kd=%.3f\n", pid.Kp, pid.Ki, pid.Kd);
  } else {
    server.send(400, "text/plain", "Missing PID parameters");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"target\":" + String(targetSpeed, 1) + ",";
  json += "\"current\":" + String(currentSpeed, 1) + ",";
  json += "\"error\":" + String(pid.error, 1) + ",";
  json += "\"pwm\":" + String((int)pid.output) + ",";  // Keep sign to show direction
  json += "\"encoder\":" + String(encoderCount) + ",";
  json += "\"kp\":" + String(pid.Kp, 2) + ",";
  json += "\"ki\":" + String(pid.Ki, 2) + ",";
  json += "\"kd\":" + String(pid.Kd, 3);
  json += "}";
  
  server.send(200, "application/json", json);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("====================================");
  Serial.println("  Single Motor PID Test - ESP32");
  Serial.println("  Bidirectional PWM Control");
  Serial.println("====================================");
  
  // Configure motor PWM pins
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(MOTOR_LPWM, OUTPUT);
  
  // Setup PWM for both channels
  ledcAttach(MOTOR_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_LPWM, PWM_FREQ, PWM_RESOLUTION);
  
  // Initialize both to 0 (motor stopped)
  ledcWrite(MOTOR_RPWM, 0);
  ledcWrite(MOTOR_LPWM, 0);
  
  Serial.println("PWM configured:");
  Serial.println("  GPIO 18 = RPWM (Forward)");
  Serial.println("  GPIO 19 = LPWM (Reverse)");
  
  // Configure encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  
  Serial.println("Encoder configured on GPIO 1 and 4");
  Serial.println("Hardware configured!");
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 500) {
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
  Serial.println("====================================");
  Serial.println("Ready for testing!");
  Serial.println("Positive speed = Forward (RPWM)");
  Serial.println("Negative speed = Reverse (LPWM)");
  Serial.println("====================================\n");
  
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
  // if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
  //   updateMotorControl();
  //   lastControlUpdate = currentTime;
  // }
  
  // Print debug info to Serial Monitor
  if (currentTime - lastPrint >= PRINT_PERIOD) {
    char dir[10];
    if (targetSpeed > 0) strcpy(dir, "FWD");
    else if (targetSpeed < 0) strcpy(dir, "REV");
    else strcpy(dir, "STOP");
    
    // Serial.printf("Target: %6.1f | Current: %6.1f | Error: %6.1f | PWM: %4d | Enc: %6ld | %s | PID(%.1f,%.1f,%.2f)\n",
    // targetSpeed, currentSpeed, pid.error, 
    //               (int)pid.output, encoderCount, dir,
    //               pid.Kp, pid.Ki, pid.Kd)
    lastPrint = currentTime;
  }
}
