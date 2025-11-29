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

    <!-- Mode Selection -->
    <div style="text-align: center; margin-bottom: 15px; padding: 15px; background: #fff9c4; border-radius: 8px;">
      <label style="font-weight: bold; margin-right: 15px;">Control Mode:</label>
      <label style="margin-right: 20px;">
        <input type="radio" name="mode" value="1" checked onchange="changeMode()"> Mode 1 (Real-time)
      </label>
      <label>
        <input type="radio" name="mode" value="2" onchange="changeMode()"> Mode 2 (Batch - Lower WiFi Usage)
      </label>
      <div id="mode2Hint" style="display: none; margin-top: 8px; font-size: 13px; color: #666;">
        Press <strong>SEND COMMAND</strong>, <strong>Enter</strong> key, or <strong>A button</strong> (Xbox) to send
      </div>
      <div style="margin-top: 8px; font-size: 12px; color: #555; text-align: center;">
    </div>

    <div id="gamepadStatus" style="text-align: center; padding: 10px; margin-bottom: 15px; border-radius: 5px; background-color: #ffebee; color: #c62828; font-weight: bold;">
      Xbox Controller: Disconnected
    </div>

    <div class="layout-grid">
      <!-- Wall Following Control -->
      <div class="panel" style="background: #e8f5e9; grid-column: 1 / -1;">
        <h3>Wall-Following Mode</h3>
        <div style="display: flex; align-items: center; gap: 15px; flex-wrap: wrap;">
          <label style="font-weight: bold;">
            <input type="checkbox" id="wallFollowCheckbox" onchange="toggleWallFollow()">
            Enable Wall-Following
          </label>
          <div style="display: flex; align-items: center; gap: 8px;">
            <label>Front Goal:</label>
            <input type="number" id="frontGoalInput" value="150" min="50" max="500" step="10" style="width: 80px; padding: 4px;" onchange="updateWallGoals()">
            <span>mm</span>
          </div>
          <div style="display: flex; align-items: center; gap: 8px;">
            <label>Right Goal:</label>
            <input type="number" id="rightGoalInput" value="100" min="30" max="300" step="10" style="width: 80px; padding: 4px;" onchange="updateWallGoals()">
            <span>mm</span>
          </div>
        </div>
        <div style="margin-top: 10px; display: flex; gap: 15px; flex-wrap: wrap; align-items: center;">
          <div style="font-family: monospace; display: flex; gap: 15px; flex-wrap: wrap;">
            <span>Front: <strong id="frontDistValue">--</strong> mm</span>
            <span>Right-F: <strong id="rightDist1Value">--</strong> mm</span>
            <span>Right-B: <strong id="rightDist2Value">--</strong> mm</span>
          </div>
          <span id="wallFollowStatus" style="padding: 5px 10px; border-radius: 5px; background: #ccc; font-weight: bold;">INACTIVE</span>
        </div>
        <div style="margin-top: 10px; padding: 10px; background: white; border-radius: 5px; font-family: monospace;">
          <strong>State:</strong> <span id="stateDisplay" style="color: #2196F3; font-weight: bold;">IDLE</span> |
          <strong>Yaw:</strong> <span id="yawDisplay">0.0</span>°
        </div>
        <div style="margin-top: 10px; padding-top: 10px; border-top: 1px solid #ccc; display: flex; gap: 15px; flex-wrap: wrap; align-items: center;">
          <strong>Wall PD Tuning:</strong>
          <div style="display: flex; align-items: center; gap: 8px;">
            <label>Kp:</label>
            <input type="number" id="wallKp" value="0.3" min="0" max="2" step="0.1" style="width: 70px; padding: 4px;" onchange="updateWallPID()">
          </div>
          <div style="display: flex; align-items: center; gap: 8px;">
            <label>Kd:</label>
            <input type="number" id="wallKd" value="0.5" min="0" max="2" step="0.1" style="width: 70px; padding: 4px;" onchange="updateWallPID()">
          </div>
        </div>
      </div>

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
          <input type="range" id="steeringSlider" min="-60" max="60" value="0" step="10" oninput="updateControl()">
          <div style="display: flex; justify-content: space-between; font-size: 12px; color: #666;">
            <span> Left Turn</span>
            <span>Right Turn </span>
          </div>
        </div>

        <div style="display: flex; align-items: center; gap: 10px;">
          <button class="stop-btn" onclick="stopMotor()">STOP</button>
          <div id="sendStatus" style="display:none; flex: 1; padding: 8px; border-radius: 5px; text-align: center; font-weight: bold; font-size: 14px; transition: opacity 0.3s;"></div>
        </div>
        <button id="sendBtn" style="display:none; background-color: #2196F3; margin-top: 10px; width: 100%;" onclick="sendBatchControl()">SEND COMMAND</button>
        <div id="pendingIndicator" style="display:none; margin-top: 10px; padding: 8px; background: #fff3cd; border-radius: 5px; text-align: center; font-weight: bold; color: #856404;">
          Pending: Speed=<span id="pendingSpeed">0</span> RPM, Steering=<span id="pendingSteering">0</span>
        </div>
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
    let controlMode = 1; // 1 = Real-time, 2 = Batch
    let pendingSpeed = 0;
    let pendingSteering = 0;
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

    function changeMode() {
      const modeRadios = document.getElementsByName('mode');
      for (let radio of modeRadios) {
        if (radio.checked) {
          controlMode = parseInt(radio.value);
          break;
        }
      }

      const sendBtn = document.getElementById('sendBtn');
      const pendingIndicator = document.getElementById('pendingIndicator');
      const mode2Hint = document.getElementById('mode2Hint');

      if (controlMode === 2) {
        sendBtn.style.display = 'block';
        pendingIndicator.style.display = 'block';
        mode2Hint.style.display = 'block';
        pendingSpeed = currentSpeed;
        pendingSteering = currentSteering;
        updatePendingDisplay();
      } else {
        sendBtn.style.display = 'none';
        pendingIndicator.style.display = 'none';
        mode2Hint.style.display = 'none';
      }

      console.log('Control mode changed to:', controlMode);
    }

    function updatePendingDisplay() {
      document.getElementById('pendingSpeed').textContent = pendingSpeed;
      document.getElementById('pendingSteering').textContent = pendingSteering;
    }

    function updateControl() {
      const speed = parseInt(document.getElementById('speedSlider').value);
      const steering = parseInt(document.getElementById('steeringSlider').value);

      document.getElementById('speedValue').textContent = speed;
      document.getElementById('steeringValue').textContent = steering;

      updateDirectionIndicator(speed, steering);

      if (controlMode === 1) {
        // Mode 1: Send immediately (real-time)
        currentSpeed = speed;
        currentSteering = steering;
        fetch('/setspeed?speed=' + currentSpeed + '&steering=' + currentSteering)
          .then(response => response.text())
          .then(data => console.log(data));
      } else {
        // Mode 2: Buffer the values, don't send yet
        pendingSpeed = speed;
        pendingSteering = steering;
        updatePendingDisplay();
      }
    }

    function sendBatchControl() {
      // Send the pending values in one batch
      currentSpeed = pendingSpeed;
      currentSteering = pendingSteering;

      const statusDiv = document.getElementById('sendStatus');

      fetch('/setspeed?speed=' + currentSpeed + '&steering=' + currentSteering)
        .then(response => response.text())
        .then(data => {
          console.log('Batch sent:', data);

          // Show success message
          statusDiv.style.display = 'block';
          statusDiv.style.backgroundColor = '#d4edda';
          statusDiv.style.color = '#155724';
          statusDiv.textContent = 'Sent!';

          // Auto-hide after 2 seconds
          setTimeout(() => {
            statusDiv.style.opacity = '0';
            setTimeout(() => {
              statusDiv.style.display = 'none';
              statusDiv.style.opacity = '1';
            }, 300);
          }, 2000);
        })
        .catch(err => {
          console.error('Error sending batch:', err);

          // Show error message
          statusDiv.style.display = 'block';
          statusDiv.style.backgroundColor = '#f8d7da';
          statusDiv.style.color = '#721c24';
          statusDiv.textContent = 'Error sending command';

          // Auto-hide after 3 seconds
          setTimeout(() => {
            statusDiv.style.opacity = '0';
            setTimeout(() => {
              statusDiv.style.display = 'none';
              statusDiv.style.opacity = '1';
            }, 300);
          }, 3000);
        });
    }

    function setControl(speed, steering) {
      document.getElementById('speedSlider').value = speed;
      document.getElementById('steeringSlider').value = steering;
      document.getElementById('speedValue').textContent = speed;
      document.getElementById('steeringValue').textContent = steering;

      updateDirectionIndicator(speed, steering);

      if (controlMode === 1) {
        // Mode 1: Send immediately (real-time)
        currentSpeed = speed;
        currentSteering = steering;
        fetch('/setspeed?speed=' + speed + '&steering=' + steering)
          .then(response => response.text())
          .then(data => console.log(data));
      } else {
        // Mode 2: Buffer the values, don't send yet
        pendingSpeed = speed;
        pendingSteering = steering;
        updatePendingDisplay();
      }
    }

    setInterval(updateStatus, 200);

    function updateWallGoals() {
      const frontGoal = parseInt(document.getElementById('frontGoalInput').value);
      const rightGoal = parseInt(document.getElementById('rightGoalInput').value);

      fetch('/setwallfollow?frontGoal=' + frontGoal + '&rightGoal=' + rightGoal)
        .then(response => response.text())
        .then(data => console.log(data))
        .catch(err => console.error('Error setting wall goals:', err));
    }

    function updateWallPID() {
      const kp = parseFloat(document.getElementById('wallKp').value);
      const kd = parseFloat(document.getElementById('wallKd').value);

      fetch('/setwallpid?kp=' + kp + '&kd=' + kd)
        .then(response => response.text())
        .then(data => console.log(data))
        .catch(err => console.error('Error setting wall PID:', err));
    }

    function toggleWallFollow() {
      const enabled = document.getElementById('wallFollowCheckbox').checked;

      fetch('/wallfollowmode?enable=' + (enabled ? 'true' : 'false'))
        .then(response => response.text())
        .then(data => {
          console.log(data);
          const statusEl = document.getElementById('wallFollowStatus');
          if (enabled) {
            statusEl.textContent = 'ACTIVE';
            statusEl.style.background = '#4CAF50';
            statusEl.style.color = 'white';
          } else {
            statusEl.textContent = 'INACTIVE';
            statusEl.style.background = '#ccc';
            statusEl.style.color = 'black';
          }
        })
        .catch(err => console.error('Error toggling wall-follow mode:', err));
    }

    function updateStatus(){
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          document.getElementById("leftTarget").textContent = data.leftTarget.toFixed(1);
          document.getElementById("leftCurrent").textContent = data.leftCurrent.toFixed(1);
          document.getElementById("leftError").textContent = data.leftError.toFixed(1);
          document.getElementById("leftPWM").textContent = data.leftPWM;
          document.getElementById("leftEncoder").textContent = data.leftEncoder;

          document.getElementById("rightTarget").textContent = data.rightTarget.toFixed(1);
          document.getElementById("rightCurrent").textContent = data.rightCurrent.toFixed(1);
          document.getElementById("rightError").textContent = data.rightError.toFixed(1);
          document.getElementById("rightPWM").textContent = data.rightPWM;
          document.getElementById("rightEncoder").textContent = data.rightEncoder;

          // Update TOF sensor data
          if (data.frontDist !== undefined) {
            document.getElementById("frontDistValue").textContent = data.frontDist;
          }
          if (data.rightDist1 !== undefined) {
            document.getElementById("rightDist1Value").textContent = data.rightDist1;
          }
          if (data.rightDist2 !== undefined) {
            document.getElementById("rightDist2Value").textContent = data.rightDist2;
          }

          // Update state machine
          if (data.state !== undefined) {
            const stateNames = ['IDLE', 'WALL FOLLOW', 'INNER CORNER', 'OUTER CORNER', 'BLIND FORWARD', 'SEEK WALL'];
            document.getElementById("stateDisplay").textContent = stateNames[data.state] || 'UNKNOWN';
          }

          // Update yaw angle
          if (data.yaw !== undefined) {
            document.getElementById("yawDisplay").textContent = data.yaw.toFixed(1);
          }

          // Update wall-follow mode status
          if (data.wallFollowMode !== undefined) {
            document.getElementById('wallFollowCheckbox').checked = data.wallFollowMode;
            const statusEl = document.getElementById('wallFollowStatus');
            if (data.wallFollowMode) {
              statusEl.textContent = 'ACTIVE';
              statusEl.style.background = '#4CAF50';
              statusEl.style.color = 'white';
            } else {
              statusEl.textContent = 'INACTIVE';
              statusEl.style.background = '#ccc';
              statusEl.style.color = 'black';
            }
          }

          // Update goal inputs if needed
          if (data.frontGoal !== undefined) {
            document.getElementById('frontGoalInput').value = data.frontGoal;
          }
          if (data.rightGoal !== undefined) {
            document.getElementById('rightGoalInput').value = data.rightGoal;
          }

          // Update wall PD parameters
          if (data.wallKp !== undefined) {
            document.getElementById('wallKp').value = data.wallKp.toFixed(1);
          }
          if (data.wallKd !== undefined) {
            document.getElementById('wallKd').value = data.wallKd.toFixed(1);
          }

          pushHistory(speedHistory.leftTarget,  Number(data.leftTarget));
          pushHistory(speedHistory.leftCurrent, Number(data.leftCurrent));
          pushHistory(speedHistory.rightTarget, Number(data.rightTarget));
          pushHistory(speedHistory.rightCurrent,Number(data.rightCurrent));

          drawSpeedChart();
        })
        .catch(err => console.error("Error fetching /status:", err));
    }
    
    function drawSpeedChart() {
      const canvas = document.getElementById('speedChart');
      if (!canvas) return;

      const ctx = canvas.getContext('2d');
      const w = canvas.width;
      const h = canvas.height;

      ctx.clearRect(0, 0, w, h);

      const allValues = [
        ...speedHistory.leftTarget,
        ...speedHistory.leftCurrent,
        ...speedHistory.rightTarget,
        ...speedHistory.rightCurrent
      ].filter(v => typeof v === 'number' && !isNaN(v));

      if (allValues.length === 0) return;

      let minVal = Math.min(...allValues);
      let maxVal = Math.max(...allValues);

      if (maxVal === minVal) {
        maxVal += 1;
        minVal -= 1;
      }

      const padding = 25;
      const innerW = w - padding * 2;
      const innerH = h - padding * 2;

      function yFor(v) {
        return padding + innerH * (1 - (v - minVal) / (maxVal - minVal));
      }

      const stepX = innerW / Math.max(MAX_POINTS - 1, 1);

      ctx.strokeStyle = '#e0e0e0';
      ctx.lineWidth = 1;
      ctx.beginPath();
      const midY = yFor((maxVal + minVal) / 2);
      ctx.moveTo(padding, midY);
      ctx.lineTo(w - padding, midY);
      ctx.stroke();

      ctx.fillStyle = '#888';
      ctx.font = '10px Arial';
      ctx.fillText(maxVal.toFixed(0) + ' RPM', 5, yFor(maxVal) + 3);
      ctx.fillText(minVal.toFixed(0) + ' RPM', 5, yFor(minVal) + 3);

      function drawSeries(arr, color) {
        if (!arr.length) return;
        ctx.beginPath();
        const offset = MAX_POINTS - arr.length;
        for (let i = 0; i < arr.length; i++) {
          const x = padding + (offset + i) * stepX;
          const y = yFor(arr[i]);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      drawSeries(speedHistory.leftTarget,  '#2196F3');
      drawSeries(speedHistory.leftCurrent, '#0D47A1');
      drawSeries(speedHistory.rightTarget, '#FF9800');
      drawSeries(speedHistory.rightCurrent,'#F44336');
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
      pendingSpeed = 0;
      pendingSteering = 0;
      document.getElementById('speedValue').textContent = 0;
      document.getElementById('speedSlider').value = 0;
      document.getElementById('steeringValue').textContent = 0;
      document.getElementById('steeringSlider').value = 0;
      updateDirectionIndicator(0, 0);

      if (controlMode === 2) {
        updatePendingDisplay();
      }

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

    let aButtonWasPressed = false; // Track A button state to detect press events

    function pollGamepad() {
      if (!gamepadConnected) return;

      const gamepads = navigator.getGamepads();
      const gamepad = gamepads[gamepadIndex];

      if (!gamepad) return;

      // RT (Right Trigger) = Forward (button 7 or axes 5)
      // LT (Left Trigger) = Reverse (button 6 or axes 4)
      // Left stick X-axis = Steering (axes 0)
      // A Button = Send command in Mode 2 (button 0)

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

      // Check A button (button 0) for sending command in Mode 2
      if (controlMode === 2 && gamepad.buttons[0]) {
        const aButtonPressed = gamepad.buttons[0].pressed;

        // Detect button press (not held) - trigger on rising edge
        if (aButtonPressed && !aButtonWasPressed) {
          console.log('A button pressed - sending batch command');
          sendBatchControl();
        }

        aButtonWasPressed = aButtonPressed;
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

    // Keyboard controls
    const SPEED_STEP = 10;
    const STEERING_STEP = 5;

    document.addEventListener('keydown', (e) => {
      // Prevent default behavior for arrow keys and space
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key)) {
        e.preventDefault();
      }

      let newSpeed = parseInt(document.getElementById('speedSlider').value);
      let newSteering = parseInt(document.getElementById('steeringSlider').value);
      let updated = false;

      switch(e.key) {
        case 'ArrowUp':
          // Increase speed
          newSpeed = Math.min(newSpeed + SPEED_STEP, 120);
          updated = true;
          break;

        case 'ArrowDown':
          // Decrease speed
          newSpeed = Math.max(newSpeed - SPEED_STEP, -120);
          updated = true;
          break;

        case 'ArrowLeft':
          // Steer left (decrease steering value)
          newSteering = Math.max(newSteering - STEERING_STEP, -60);
          updated = true;
          break;

        case 'ArrowRight':
          // Steer right (increase steering value)
          newSteering = Math.min(newSteering + STEERING_STEP, 60);
          updated = true;
          break;

        case ' ':
          // Space bar - stop motor
          stopMotor();
          return;

        case '1':
          // Switch to Mode 1
          document.querySelector('input[name="mode"][value="1"]').checked = true;
          changeMode();
          console.log('Switched to Mode 1 via keyboard');
          return;

        case '2':
          // Switch to Mode 2
          document.querySelector('input[name="mode"][value="2"]').checked = true;
          changeMode();
          console.log('Switched to Mode 2 via keyboard');
          return;

        case 'Enter':
          // Enter key - send batch command in Mode 2
          if (controlMode === 2) {
            sendBatchControl();
          }
          return;
      }

      // Update controls if speed or steering changed
      if (updated) {
        document.getElementById('speedSlider').value = newSpeed;
        document.getElementById('steeringSlider').value = newSteering;
        updateControl();
      }
    });
  </script>
</body>
</html>
)rawliteral";

#endif