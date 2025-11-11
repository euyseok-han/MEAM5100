#ifndef WEBSITE_H
#define WEBSITE_H

// Store page in flash (saves RAM on ESP32)
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
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
    <h1>Dual Motor Differential Drive</h1>
    <h3>Speed + Steering Control</h3>

    <div class="control-section">
      <h3>Speed Control <span id="direction" class="direction-indicator stopped">STOPPED</span></h3>
      <div class="speed-control">
        <label>Base Speed: <span id="speedValue" class="speed-value">0</span> RPM</label>
        <input type="range" id="speedSlider" min="-120" max="120" value="0" step="10" oninput="updateControl()">
        <div style="display: flex; justify-content: space-between; font-size: 12px; color: #666;">
          <span>← Reverse</span>
          <span>Forward →</span>
        </div>
      </div>

      <div class="speed-control">
        <label>Steering: <span id="steeringValue" class="speed-value">0</span></label>
        <input type="range" id="steeringSlider" min="-60" max="60" value="0" step="5" oninput="updateControl()">
        <div style="display: flex; justify-content: space-between; font-size: 12px; color: #666;">
          <span>← Left Turn</span>
          <span>Right Turn →</span>
        </div>
      </div>

      <br>
      <button class="stop-btn" onclick="stopMotor()">STOP</button>
    </div>
    
    <div class="pid-section">
      <h3>PID Tuning</h3>
      <div class="pid-input">
        <label>Kp:</label>
        <input type="number" id="kp" value="0.4" step="0.1" onchange="updatePID()">
      </div>
      <div class="pid-input">
        <label>Ki:</label>
        <input type="number" id="ki" value="0.005" step="0.1" onchange="updatePID()">
      </div>
      <div class="pid-input">
        <label>Kd:</label>
        <input type="number" id="kd" value="0" step="0.01" onchange="updatePID()">
    </div>
    
    <div class="status">
      <p><strong>LEFT Wheel:</strong></p>
      <p style="margin-left: 20px;">Target: <span id="leftTarget">0</span> RPM | Current: <span id="leftCurrent">0</span> RPM</p>
      <p style="margin-left: 20px;">Error: <span id="leftError">0</span> | PWM: <span id="leftPWM">0</span> | Encoder: <span id="leftEncoder">0</span></p>
      <hr>
      <p><strong>RIGHT Wheel:</strong></p>
      <p style="margin-left: 20px;">Target: <span id="rightTarget">0</span> RPM | Current: <span id="rightCurrent">0</span> RPM</p>
      <p style="margin-left: 20px;">Error: <span id="rightError">0</span> | PWM: <span id="rightPWM">0</span> | Encoder: <span id="rightEncoder">0</span></p>
    </div>
 
  </div>
  
  <script>
    let currentSpeed = 0;
    let currentSteering = 0;

    function updateControl() {
      currentSpeed = parseInt(document.getElementById('speedSlider').value);
      currentSteering = parseInt(document.getElementById('steeringSlider').value);

      document.getElementById('speedValue').textContent = currentSpeed;
      document.getElementById('steeringValue').textContent = currentSteering;

      updateDirectionIndicator(currentSpeed, currentSteering);

      // Send to ESP32
      fetch('/setspeed?speed=' + currentSpeed + '&steering=' + currentSteering)
        .then(response => response.text())
        .then(data => console.log(data));
    }

    function setControl(speed, steering) {
      currentSpeed = speed;
      currentSteering = steering;

      document.getElementById('speedSlider').value = speed;
      document.getElementById('steeringSlider').value = steering;
      document.getElementById('speedValue').textContent = speed;
      document.getElementById('steeringValue').textContent = steering;

      updateDirectionIndicator(speed, steering);

      fetch('/setspeed?speed=' + speed + '&steering=' + steering)
        .then(response => response.text())
        .then(data => console.log(data));
    }

    setInterval(updateStatus, 1000);

    function updateStatus(){
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          // Base control
          document.getElementById("baseSpeed").textContent = data.baseSpeed.toFixed(1);
          document.getElementById("steering").textContent = data.steering.toFixed(1);

          // Left wheel
          document.getElementById("leftTarget").textContent = data.leftTarget.toFixed(1);
          document.getElementById("leftCurrent").textContent = data.leftCurrent.toFixed(1);
          document.getElementById("leftError").textContent = data.leftError.toFixed(1);
          document.getElementById("leftPWM").textContent = data.leftPWM;
          document.getElementById("leftEncoder").textContent = data.leftEncoder;

          // Right wheel
          document.getElementById("rightTarget").textContent = data.rightTarget.toFixed(1);
          document.getElementById("rightCurrent").textContent = data.rightCurrent.toFixed(1);
          document.getElementById("rightError").textContent = data.rightError.toFixed(1);
          document.getElementById("rightPWM").textContent = data.rightPWM;
          document.getElementById("rightEncoder").textContent = data.rightEncoder;

          // PID
          document.getElementById("displayKp").textContent = data.kp.toFixed(2);
          document.getElementById("displayKi").textContent = data.ki.toFixed(2);
          document.getElementById("displayKd").textContent = data.kd.toFixed(3);
        }).catch(err => console.error("Error fetching /status:", err));
    }

    function updateDirectionIndicator(speed, steering) {
      const indicator = document.getElementById('direction');
      if (speed > 0 && Math.abs(steering) < 10) {
        indicator.textContent = 'FORWARD';
        indicator.className = 'direction-indicator forward';
      } else if (speed > 0 && steering < -10) {
        indicator.textContent = 'FORWARD LEFT';
        indicator.className = 'direction-indicator forward';
      } else if (speed > 0 && steering > 10) {
        indicator.textContent = 'FORWARD RIGHT';
        indicator.className = 'direction-indicator forward';
      } else if (speed < 0 && Math.abs(steering) < 10) {
        indicator.textContent = 'REVERSE';
        indicator.className = 'direction-indicator reverse';
      } else if (speed < 0 && steering < -10) {
        indicator.textContent = 'REVERSE LEFT';
        indicator.className = 'direction-indicator reverse';
      } else if (speed < 0 && steering > 10) {
        indicator.textContent = 'REVERSE RIGHT';
        indicator.className = 'direction-indicator reverse';
      } else if (Math.abs(speed) < 10 && Math.abs(steering) > 10) {
        indicator.textContent = 'SPINNING';
        indicator.className = 'direction-indicator forward';
      } else {
        indicator.textContent = 'STOPPED';
        indicator.className = 'direction-indicator stopped';
      }
    }


    function stopMotor() {
      currentSpeed = 0;
      currentSteering = 0;
      document.getElementById('speedValue').textContent = 0;
      document.getElementById('speedSlider').value = 0;
      document.getElementById('steeringValue').textContent = 0;
      document.getElementById('steeringSlider').value = 0;
      updateDirectionIndicator(0, 0);

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
      
  </script>
</body>
</html>
)rawliteral";

#endif

