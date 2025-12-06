#ifndef WEBSITE_H
#define WEBSITE_H

// Store page in flash (saves RAM on ESP32)
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>MEAM5100 Robot Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { 
      font-family: Arial; 
      text-align: center; 
      margin: 20px; 
      background-color: #f0f0f0; 
    }
    .container { 
      max-width: 1100px;
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

    /* === GRID LAYOUT === */
    .layout-grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 18px;
      margin-top: 20px;
    }
    @media (min-width: 768px) {
      .layout-grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }

    .panel {
      padding: 15px;
      border-radius: 8px;
      background: #fafafa;
      box-shadow: 0 1px 4px rgba(0,0,0,0.05);
    }

    .control-section { background: #f9f9f9; }
    .pid-section     { background: #fff3cd; }
    .status          { background: #e7f3ff; font-family: monospace; }

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

    input[type="range"] { width: 100%; }
    .speed-value { font-size: 20px; font-weight: bold; color: #2196F3; }

    /* Direction indicator */
    .direction-indicator {
      padding: 5px 15px;
      border-radius: 5px;
      font-weight: bold;
      display: inline-block;
      margin-left: 10px;
    }
    .forward { background: #4CAF50; color: white; }
    .reverse { background: #FF9800; color: white; }
    .stopped { background: #999;     color: white; }
    .spin    { background: #673ab7; color: white; }
  </style>
</head>
<body>
  <div class="container">
    <h1>MEAM5100 Robot Control</h1>
    <h3>Manual Drive · PID Tuning · Live Telemetry · BFS Routing</h3>

    <div id="gamepadStatus"
        style="text-align:center; padding:10px; margin-bottom:15px; 
               border-radius:5px; background-color:#ffebee; color:#c62828; font-weight:bold;">
      Xbox Controller: Disconnected
    </div>

    <div class="layout-grid">
      
      <!-- CONTROL PANEL -->
      <div class="panel control-section">
        <h3>Speed Control <span id="direction" class="direction-indicator stopped">STOPPED</span></h3>

        <label>Base Speed: <span id="speedValue" class="speed-value">0</span> RPM</label>
        <input type="range" id="speedSlider" min="-120" max="120" value="0" step="5" oninput="updateControl()">

        <label>Steering: <span id="steeringValue" class="speed-value">0</span></label>
        <input type="range" id="steeringSlider" min="-60" max="60" value="0" step="5" oninput="updateControl()">

        <button class="stop-btn" onclick="stopMotor()">STOP</button>
      </div>

      <!-- PID TUNING -->
      <div class="panel pid-section">
        <h3>PID Tuning</h3>

        <label>Kp:</label><input id="kp" type="number" value="0.3" step="0.1" onchange="updatePID()"><br><br>
        <label>Ki:</label><input id="ki" type="number" value="1.5" step="0.1" onchange="updatePID()"><br><br>
        <label>Kd:</label><input id="kd" type="number" value="0" step="0.01" onchange="updatePID()">
      </div>

      <!-- STATUS PANEL -->
      <div class="panel status">
        <h3>Live Status</h3>
        <pre id="statusBox">Loading...</pre>
      </div>

      <!-- =============== BFS ROUTE FINDER PANEL =============== -->
      <div class="panel" style="background:#e8f4ff;">
        <h3>BFS Route Finder</h3>

        <label>Start Node:</label>
        <input id="routeStart" type="number" value="0" style="width:80px;"><br><br>

        <label>Goal Node:</label>
        <input id="routeGoal" type="number" value="1" style="width:80px;"><br><br>

        <button onclick="callRoute()">Find Route</button>

        <pre id="routeResult" style="margin-top:10px; font-family:monospace;"></pre>
      </div>

    </div>
  </div>

  <script>
    let currentSpeed = 0;
    let currentSteering = 0;

    function updateDirectionIndicator(speed, steering) {
      const d = document.getElementById('direction');

      if (speed > 0 && Math.abs(steering) < 10) {
        d.textContent = 'FORWARD'; d.className = 'direction-indicator forward';
      } else if (speed < 0 && Math.abs(steering) < 10) {
        d.textContent = 'REVERSE'; d.className = 'direction-indicator reverse';
      } else if (Math.abs(speed) < 5 && Math.abs(steering) > 15) {
        d.textContent = 'SPIN'; d.className = 'direction-indicator spin';
      } else if (speed === 0 && steering === 0) {
        d.textContent = 'STOPPED'; d.className = 'direction-indicator stopped';
      } else {
        d.textContent = 'MOVING'; d.className = 'direction-indicator forward';
      }
    }

    function updateControl() {
      currentSpeed = parseInt(document.getElementById('speedSlider').value);
      currentSteering = parseInt(document.getElementById('steeringSlider').value);

      document.getElementById('speedValue').textContent = currentSpeed;
      document.getElementById('steeringValue').textContent = currentSteering;

      updateDirectionIndicator(currentSpeed, currentSteering);

      fetch('/setspeed?speed=' + currentSpeed + '&steering=' + currentSteering);
    }

    function stopMotor() {
      currentSpeed = 0;
      currentSteering = 0;
      document.getElementById('speedSlider').value = 0;
      document.getElementById('steeringSlider').value = 0;
      updateDirectionIndicator(0, 0);

      fetch('/stop');
    }

    function updatePID() {
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;
      fetch('/setpid?kp=' + kp + '&ki=' + ki + '&kd=' + kd);
    }

    // ================== STATUS UPDATES ==================
    function refreshStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          const txt =
`BASE  : speed=${data.baseSpeed}  steering=${data.steering}
LEFT  : target=${data.leftTarget}  current=${data.leftCurrent}  pwm=${data.leftPWM}
RIGHT : target=${data.rightTarget} current=${data.rightCurrent} pwm=${data.rightPWM}
ENC   : L=${data.leftEncoder}  R=${data.rightEncoder}
POSE  : x=${data.vx}  y=${data.vy}  yaw=${data.yaw}
MODE  : ${data.mode}`;
          document.getElementById('statusBox').textContent = txt;
        });
    }
    setInterval(refreshStatus, 150);

    // ================== GAMEPAD SUPPORT ==================
    let gamepadConnected = false;
    let gamepadIndex = null;

    function applyDeadzone(v) {
      return Math.abs(v) < 0.15 ? 0 : v;
    }

    window.addEventListener("gamepadconnected", (e) => {
      gamepadConnected = true;
      gamepadIndex = e.gamepad.index;
      document.getElementById('gamepadStatus').textContent =
        'Xbox Controller: Connected';
      document.getElementById('gamepadStatus').style.backgroundColor = '#e8f5e9';
      document.getElementById('gamepadStatus').style.color = '#2e7d32';
    });

    window.addEventListener("gamepaddisconnected", () => {
      gamepadConnected = false;
      gamepadIndex = null;
      document.getElementById('gamepadStatus').textContent =
        'Xbox Controller: Disconnected';
      stopMotor();
    });

    function pollGamepad() {
      if (!gamepadConnected) return;

      const gp = navigator.getGamepads()[gamepadIndex];
      if (!gp) return;

      const rt = gp.buttons[7]?.value || 0;
      const lt = gp.buttons[6]?.value || 0;
      const speed = (rt - lt) * 120;

      const steer = applyDeadzone(gp.axes[0]) * 60;

      setControl(speed, steer);
    }

    function setControl(speed, steering) {
      currentSpeed = Math.round(speed);
      currentSteering = Math.round(steering);

      document.getElementById('speedSlider').value = currentSpeed;
      document.getElementById('steeringSlider').value = currentSteering;

      updateDirectionIndicator(speed, steering);

      fetch('/setspeed?speed=' + currentSpeed + '&steering=' + currentSteering);
    }

    setInterval(pollGamepad, 50);

    // ================== KEYBOARD CONTROL ==================
    document.addEventListener('keydown', (e) => {
      let handled = false;

      if (e.key === 'ArrowUp') { currentSpeed = Math.min(120, currentSpeed + 10); handled = true; }
      if (e.key === 'ArrowDown') { currentSpeed = Math.max(-120, currentSpeed - 10); handled = true; }
      if (e.key === 'ArrowLeft') { currentSteering = Math.max(-60, currentSteering - 5); handled = true; }
      if (e.key === 'ArrowRight') { currentSteering = Math.min(60, currentSteering + 5); handled = true; }

      if (e.key === 's' || e.key === 'S') { stopMotor(); handled = true; }

      if (handled) {
        e.preventDefault();
        setControl(currentSpeed, currentSteering);
      }
    });

    // ================== BFS ROUTE CALL ==================
    function callRoute() {
      const start = document.getElementById("routeStart").value;
      const goal  = document.getElementById("routeGoal").value;

      fetch(`/route?start=${start}&goal=${goal}`)
        .then(r => r.text())
        .then(txt => {
          document.getElementById("routeResult").textContent =
            "Route: " + txt;
        })
        .catch(err => {
          document.getElementById("routeResult").textContent =
            "Error fetching route.";
        });
    }

  </script>
</body>
</html>
)rawliteral";

#endif
