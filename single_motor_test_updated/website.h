#ifndef WEBSITE_H
#define WEBSITE_H

// Store page in flash (saves RAM on ESP32)
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Dual Motor Differential Drive</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { 
      font-family: Arial; 
      text-align: center; 
      margin: 20px; 
      background-color: #f0f0f0; 
    }
    .container { 
      max-width: 1100px; /* wider for 2-column layout */
      margin: auto; 
      background: white; 
      padding: 20px; 
      border-radius: 10px; 
      box-shadow: 0 2px 10px rgba(0,0,0,0.1);
      text-align: left;
    }
    h1, h3 {
      text-align: center;
      color: #333;
      margin-top: 0;
    }

    /* === GRID LAYOUT FOR PANELS === */
    .layout-grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 16px;
      margin-top: 20px;
    }
    @media (min-width: 768px) {
      .layout-grid {
        grid-template-columns: repeat(2, minmax(0, 1fr)); /* 2 columns */
      }
    }

    .panel {
      padding: 15px;
      border-radius: 8px;
      background: #f9f9f9;
      box-shadow: 0 1px 4px rgba(0,0,0,0.05);
    }

    .control-section { 
      background: #f9f9f9;
    }
    .pid-section {
      background: #fff3cd;
    }
    .status { 
      background: #e7f3ff; 
      font-family: monospace;
    }
    .graph-section {
      background: #ffffff;
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

    .speed-control { 
      margin: 15px 0; 
    }
    input[type="range"] { 
      width: 100%; 
    }
    .speed-value { 
      font-size: 20px; 
      font-weight: bold; 
      color: #2196F3; 
    }

    .pid-input {
      margin: 8px 0;
      text-align: left;
    }
    .pid-input label {
      display: inline-block;
      width: 60px;
    }
    .pid-input input {
      width: 90px;
      padding: 4px;
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

    /* Graph */
    #speedChart {
      width: 100%;
      display: block;
      margin-top: 10px;
      border-radius: 4px;
      background: #fafafa;
    }

    .legend {
      margin-top: 8px;
      font-size: 12px;
      color: #555;
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }

    .legend-box {
      display: inline-block;
      width: 14px;
      height: 4px;
      margin-right: 4px;
      border-radius: 2px;
    }

    .legend-box.lt { background: #2196F3; }  /* Left Target */
    .legend-box.lc { background: #0D47A1; }  /* Left Current */
    .legend-box.rt { background: #FF9800; }  /* Right Target */
    .legend-box.rc { background: #F44336; }  /* Right Current */
  </style>
</head>
<body>
  <div class="container">
    <h1>Dual Motor Differential Drive</h1>
    <h3>Speed + Steering Control</h3>
    <div id="gamepadStatus" style="text-align: center; padding: 10px; margin-bottom: 15px; border-radius: 5px; background-color: #ffebee; color: #c62828; font-weight: bold;">
      Xbox Controller: Disconnected
    </div>

    <div class="layout-grid">
      <!-- Control -->
      <div class="control-section panel">
        <h3>Speed Control <span id="direction" class="direction-indicator stopped">STOPPED</span></h3>
        <div class="speed-control">
          <label>Base Speed: <span id="speedValue" class="speed-value">0</span> RPM</label>
          <input type="range" id="speedSlider" min="-120" max="120" value="0" step="10" oninput="updateControl()">
          <div style="display: flex; justify-content: space-between; font-size: 12px; color: #666;">
            <span> Reverse</span>
            <span>Forward </span>
          </div>
        </div>

        <div class="speed-control">
          <label>Steering: <span id="steeringValue" class="speed-value">0</span></label>
          <input type="range" id="steeringSlider" min="-60" max="60" value="0" step="5" oninput="updateControl()">
          <div style="display: flex; justify-content: space-between; font-size: 12px; color: #666;">
            <span> Left Turn</span>
            <span>Right Turn </span>
          </div>
        </div>

        <button class="stop-btn" onclick="stopMotor()">STOP</button>
      </div>

      <!-- PID -->
      <div class="pid-section panel">
        <h3>PID Tuning</h3>
        <div class="pid-input">
          <label>Kp:</label>
          <input type="number" id="kp" value="0.3" step="0.1" onchange="updatePID()">
        </div>
        <div class="pid-input">
          <label>Ki:</label>
          <input type="number" id="ki" value="1.5" step="0.1" onchange="updatePID()">
        </div>
        <div class="pid-input">
          <label>Kd:</label>
          <input type="number" id="kd" value="0" step="0.01" onchange="updatePID()">
        </div>
      </div>

      <!-- Status -->
      <div class="status panel">
        <p><strong>LEFT Wheel:</strong></p>
        <p style="margin-left: 20px;">
          Target: <span id="leftTarget">0</span> RPM |
          Current: <span id="leftCurrent">0</span> RPM
        </p>
        <p style="margin-left: 20px;">
          Error: <span id="leftError">0</span> |
          PWM: <span id="leftPWM">0</span> |
          Encoder: <span id="leftEncoder">0</span>
        </p>
        <hr>
        <p><strong>RIGHT Wheel:</strong></p>
        <p style="margin-left: 20px;">
          Target: <span id="rightTarget">0</span> RPM |
          Current: <span id="rightCurrent">0</span> RPM
        </p>
        <p style="margin-left: 20px;">
          Error: <span id="rightError">0</span> |
          PWM: <span id="rightPWM">0</span> |
          Encoder: <span id="rightEncoder">0</span>
        </p>
      </div>

      <!-- Graph -->
      <div class="graph-section panel">
        <h3>Wheel Speed History</h3>
        <canvas id="speedChart" width="500" height="220"></canvas>
        <div class="legend">
          <span class="legend-box lt"></span> Left Target
          <span class="legend-box lc"></span> Left Current
          <span class="legend-box rt"></span> Right Target
          <span class="legend-box rc"></span> Right Current
        </div>
      </div>
    </div>
  </div>
  
  <script>
    let currentSpeed = 0;
    let currentSteering = 0;
    const MAX_POINTS = 60;
    const speedHistory = {
      leftTarget: [],
      leftCurrent: [],
      rightTarget: [],
      rightCurrent: []
    };

    // Xbox gamepad support
    let gamepadConnected = false;
    let gamepadIndex = null;
    const DEADZONE = 0.15;
    const MAX_SPEED = 120;

    function pushHistory(arr, value) {
      if (typeof value !== 'number' || isNaN(value)) return;
      arr.push(value);
      if (arr.length > MAX_POINTS) arr.shift();
    }

    function updateControl() {
      currentSpeed = parseInt(document.getElementById('speedSlider').value);
      currentSteering = parseInt(document.getElementById('steeringSlider').value);

      document.getElementById('speedValue').textContent = currentSpeed;
      document.getElementById('steeringValue').textContent = currentSteering;

      updateDirectionIndicator(currentSpeed, currentSteering);

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

    // Gamepad event handlers
    window.addEventListener("gamepadconnected", (e) => {
      console.log("Gamepad connected:", e.gamepad.id);
      gamepadConnected = true;
      gamepadIndex = e.gamepad.index;
      updateGamepadStatus(true, e.gamepad.id);
    });

    window.addEventListener("gamepaddisconnected", (e) => {
      console.log("Gamepad disconnected");
      gamepadConnected = false;
      gamepadIndex = null;
      updateGamepadStatus(false, "");
      stopMotor();
    });

    function updateGamepadStatus(connected, name) {
      const statusDiv = document.getElementById('gamepadStatus');
      if (connected) {
        statusDiv.style.backgroundColor = '#e8f5e9';
        statusDiv.style.color = '#2e7d32';
        statusDiv.textContent = 'Xbox Controller: Connected - ' + name;
      } else {
        statusDiv.style.backgroundColor = '#ffebee';
        statusDiv.style.color = '#c62828';
        statusDiv.textContent = 'Xbox Controller: Disconnected';
      }
    }

    function applyDeadzone(value) {
      return Math.abs(value) < DEADZONE ? 0 : value;
    }

    function pollGamepad() {
      if (!gamepadConnected) return;

      const gamepads = navigator.getGamepads();
      const gamepad = gamepads[gamepadIndex];

      if (!gamepad) return;

      // RT (Right Trigger) = Forward (button 7 or axes 5)
      // LT (Left Trigger) = Reverse (button 6 or axes 4)
      // Left stick X-axis = Steering (axes 0)

      let rtValue = 0;
      let ltValue = 0;

      // Try to get trigger values (different browsers may use different mappings)
      if (gamepad.buttons[7]) {
        rtValue = gamepad.buttons[7].value;
      }
      if (gamepad.buttons[6]) {
        ltValue = gamepad.buttons[6].value;
      }

      // Left stick horizontal axis for steering
      let stickX = applyDeadzone(gamepad.axes[0]);

      // Calculate speed: RT gives forward, LT gives reverse
      let speed = (rtValue - ltValue) * MAX_SPEED;

      // Calculate steering: -60 to 60 range
      let steering = stickX * 60;

      // Update controls
      if (Math.abs(speed - currentSpeed) > 2 || Math.abs(steering - currentSteering) > 2) {
        setControl(Math.round(speed), Math.round(steering));
      }
    }

    // Poll gamepad at 50ms intervals
    setInterval(pollGamepad, 50);

    // Check for already connected gamepads on load
    window.addEventListener('load', () => {
      const gamepads = navigator.getGamepads();
      for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
          gamepadConnected = true;
          gamepadIndex = i;
          updateGamepadStatus(true, gamepads[i].id);
          console.log("Gamepad already connected:", gamepads[i].id);
          break;
        }
      }
    });

    // --- Added keyboard control section ---
    document.addEventListener('keydown', function (e) {
      let handled = false;

      switch (e.key) {
        case 'ArrowUp':
          currentSpeed = Math.min(120, currentSpeed + 10);
          setControl(currentSpeed, currentSteering);
          handled = true;
          break;

        case 'ArrowDown':
          currentSpeed = Math.max(-120, currentSpeed - 10);
          setControl(currentSpeed, currentSteering);
          handled = true;
          break;

        case 'ArrowLeft':
          currentSteering = Math.max(-60, currentSteering - 5);
          setControl(currentSpeed, currentSteering);
          handled = true;
          break;

        case 'ArrowRight':
          currentSteering = Math.min(60, currentSteering + 5);
          setControl(currentSpeed, currentSteering);
          handled = true;
          break;

        case 's':
        case 'S':
          stopMotor();
          handled = true;
          break;
      }

      if (handled) e.preventDefault();
    });
  </script>
</body>
</html>
)rawliteral";

#endif
