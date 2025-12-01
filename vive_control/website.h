#ifndef WEBSITE_H
#define WEBSITE_H

// Store page in flash (saves RAM on ESP32)
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Dual Motor Differential Drive + Vive XY Navigation</title>
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

    .layout-grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 16px;
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
      background: #f9f9f9;
      box-shadow: 0 1px 4px rgba(0,0,0,0.05);
    }

    button { 
      padding: 10px 20px; 
      margin: 5px;
      font-size: 16px; 
      border: none; 
      border-radius: 5px; 
      cursor: pointer; 
      background-color: #4CAF50; 
      color: white; 
    }
    .stop-btn { background-color: #f44336; }

    input[type="number"] {
      padding: 6px;
      width: 90px;
      font-size: 16px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Dual Motor Differential Drive</h1>
    <h3>Motor Control + Vive Position Navigation</h3>

    <div class="layout-grid">

      <!-- ============================= -->
      <!-- VIVE XY NAVIGATION PANEL      -->
      <!-- ============================= -->
      <div class="panel" style="background:#e3f2fd;">
        <h3>Vive Coordinate Navigation</h3>

        <div style="display:flex; flex-wrap:wrap; gap:20px; align-items:center;">
          <div>
            <label><strong>X Target:</strong></label>
            <input id="viveX" type="number" value="3000" step="50">
          </div>
          <div>
            <label><strong>Y Target:</strong></label>
            <input id="viveY" type="number" value="1500" step="50">
          </div>
        </div>

        <div style="margin-top:10px;">
          <button onclick="goToVivePoint()">GO TO POINT</button>
          <button class="stop-btn" onclick="stopMotor()">STOP</button>
        </div>

        <div style="margin-top:10px; font-family:monospace;">
          <strong>Current Vive:</strong>
          X=<span id="viveXNow">--</span> |
          Y=<span id="viveYNow">--</span>
        </div>
      </div>

      <!-- ============================= -->
      <!-- SPEED / STEERING PANEL        -->
      <!-- ============================= -->
      <div class="panel" style="background:#f9f9f9;">
        <h3>Speed Control</h3>
        <label>Base Speed: <span id="speedValue">0</span></label>
        <input type="range" id="speedSlider" min="-120" max="120" value="0"
               step="10" oninput="updateControl()">

        <label>Steering: <span id="steeringValue">0</span></label>
        <input type="range" id="steeringSlider" min="-60" max="60" value="0"
               step="10" oninput="updateControl()">

        <button class="stop-btn" onclick="stopMotor()">STOP</button>
      </div>

      <!-- ============================= -->
      <!-- PID PANEL                     -->
      <!-- ============================= -->
      <div class="panel" style="background:#fff3cd;">
        <h3>PID Tuning</h3>
        <label>Kp:</label> <input id="kp" type="number" value="0.3" step="0.1" onchange="updatePID()"><br>
        <label>Ki:</label> <input id="ki" type="number" value="1.5" step="0.1" onchange="updatePID()"><br>
        <label>Kd:</label> <input id="kd" type="number" value="0" step="0.01" onchange="updatePID()">
      </div>

      <!-- ============================= -->
      <!-- STATUS PANEL                  -->
      <!-- ============================= -->
      <div class="panel" style="background:#e7f3ff; font-family:monospace;">
        <h3>Status</h3>
        Left Target: <span id="leftTarget">0</span> |
        Left Current: <span id="leftCurrent">0</span><br>
        Right Target: <span id="rightTarget">0</span> |
        Right Current: <span id="rightCurrent">0</span><br>
        Vive (X,Y): <span id="viveXNow2">--</span>, <span id="viveYNow2">--</span>
      </div>

    </div>
  </div>

  <script>
    // ================================
    // SEND (X,Y) to ESP32
    // ================================
    function goToVivePoint() {
      const x = document.getElementById('viveX').value;
      const y = document.getElementById('viveY').value;

      fetch('/gotopoint?x=' + x + '&y=' + y)
        .then(r => r.text())
        .then(t => alert("Moving to:\n" + t));
    }

    // ================================
    // SPEED + STEERING UPDATE
    // ================================
    function updateControl() {
      const speed = parseInt(document.getElementById('speedSlider').value);
      const steering = parseInt(document.getElementById('steeringSlider').value);

      document.getElementById('speedValue').textContent = speed;
      document.getElementById('steeringValue').textContent = steering;

      fetch(`/control?speed=${speed}&steering=${steering}`);
    }

    // ================================
    // STOP MOTOR
    // ================================
    function stopMotor() {
      fetch('/stop');
    }

    // ================================
    // UPDATE PID VALUES
    // ================================
    function updatePID() {
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;

      fetch(`/pid?kp=${kp}&ki=${ki}&kd=${kd}`);
    }

    // ================================
    // LIVE STATUS POLLING
    // ================================
    setInterval(() => {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('leftTarget').textContent = data.leftTarget;
          document.getElementById('leftCurrent').textContent = data.leftCurrent;
          document.getElementById('rightTarget').textContent = data.rightTarget;
          document.getElementById('rightCurrent').textContent = data.rightCurrent;

          document.getElementById('viveXNow').textContent = data.vx;
          document.getElementById('viveYNow').textContent = data.vy;
          document.getElementById('viveXNow2').textContent = data.vx;
          document.getElementById('viveYNow2').textContent = data.vy;
        });
    }, 200); // update 5 times/s

  </script>

</body>
</html>
)rawliteral";

#endif
