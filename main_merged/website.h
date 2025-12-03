// Unified website for all modes: Manual, Wall-Follow, Vive Nav
#ifndef WEBSITE_H
#define WEBSITE_H

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MEAM5100 Robot Control (Unified)</title>
  <style>
    body { font-family: Arial, sans-serif; background: #f0f0f0; margin: 0; }
    .container { max-width: 1080px; margin: 20px auto; padding: 20px; background: #fff; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
    h1, h2, h3 { margin: 8px 0; color: #333; }
    .grid { display: grid; grid-template-columns: 1fr; gap: 16px; }
    @media (min-width: 900px) { .grid { grid-template-columns: 1fr 1fr; } }
    .panel { background: #fafafa; border-radius: 8px; padding: 14px; box-shadow: 0 1px 3px rgba(0,0,0,0.06); }
    .row { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; }
    label { font-weight: bold; }
    input[type="number"], input[type="text"] { padding: 6px 8px; width: 90px; }
    input[type="range"] { width: 100%; }
    button { padding: 10px 16px; border: 0; border-radius: 6px; cursor: pointer; background: #4CAF50; color: #fff; font-weight: 600; }
    .stop { background: #f44336; }
    .blue { background: #2196F3; }
    .warn { background: #fff3cd; }
    .badge { display: inline-block; padding: 4px 10px; border-radius: 999px; background: #ccc; font-weight: bold; }
    .state { background: #e1f5fe; }
    .mono { font-family: monospace; }
    .kv { display: inline-block; min-width: 90px; }
  </style>
</head>
<body>
  <div class="container">
    <h1>MEAM5100 Robot Control</h1>
    <h3>Manual + Wall-Follow + Vive Navigation</h3>

    <div class="panel warn">
      <div class="row">
        <label>Control Mode:</label>
        <label><input type="radio" name="mode" value="manual" checked onchange="setMode('manual')"> Manual</label>
        <label><input type="radio" name="mode" value="wall" onchange="setMode('wall')"> Wall-Follow</label>
        <label><input type="radio" name="mode" value="vive" onchange="setMode('vive')"> Vive Nav</label>
        <span id="modeBadge" class="badge">MANUAL</span>
      </div>
    </div>

    <div class="grid">
      <div class="panel">
        <h3>Speed + Steering</h3>
        <div>
          <div>Base Speed: <span id="speedValue" class="mono">0</span> RPM</div>
          <input type="range" id="speedSlider" min="-120" max="120" step="10" value="0" oninput="updateControl()">
        </div>
        <div style="margin-top:8px;">
          <div>Steering: <span id="steeringValue" class="mono">0</span></div>
          <input type="range" id="steeringSlider" min="-60" max="60" step="5" value="0" oninput="updateControl()">
        </div>
        <div class="row" style="margin-top:10px;">
          <button class="stop" onclick="stopMotor()">STOP</button>
        </div>
        <div style="margin-top:10px;" class="mono">
          Left Target: <span id="leftTarget">0</span> | Current: <span id="leftCurrent">0</span><br>
          Right Target: <span id="rightTarget">0</span> | Current: <span id="rightCurrent">0</span><br>
          PWM L/R: <span id="pwmL">0</span>/<span id="pwmR">0</span>
        </div>
      </div>

      <div class="panel">
        <h3>PID Tuning</h3>
        <div class="row"><label>Kp</label><input id="kp" type="number" step="0.1" value="0.3" onchange="updatePID()"></div>
        <div class="row"><label>Ki</label><input id="ki" type="number" step="0.1" value="1.1" onchange="updatePID()"></div>
        <div class="row"><label>Kd</label><input id="kd" type="number" step="0.01" value="0" onchange="updatePID()"></div>
      </div>

      <div class="panel" style="grid-column: 1 / -1; background:#e8f5e9;">
        <h3>Wall-Following</h3>
        <div class="row">
          <label><input type="checkbox" id="wallEnable" onchange="toggleWall()"> Enable</label>
          <span id="wallBadge" class="badge">OFF</span>
        </div>
        <div class="row" style="margin-top:8px;">
          <label>Front Goal</label><input id="frontGoal" type="number" value="150" step="10" onchange="updateWallGoals()"> mm
          <label>Right Goal 1</label><input id="rightGoal1" type="number" value="100" step="10" onchange="updateWallGoals()"> mm
          <label>Right Goal 2</label><input id="rightGoal2" type="number" value="100" step="10" onchange="updateWallGoals()"> mm
        </div>
        <div class="row" style="margin-top:8px;">
          <label>Kp</label><input id="wkp" type="number" value="1.5" step="0.1" onchange="updateWallPD()">
          <label>Kd</label><input id="wkd" type="number" value="0.8" step="0.1" onchange="updateWallPD()">
        </div>
        <div class="row" style="margin-top:8px;" class="mono">
          <span class="kv">Front</span><span id="frontDist" class="mono">--</span> mm
          <span class="kv">Right-F</span><span id="right1" class="mono">--</span> mm
          <span class="kv">Right-B</span><span id="right2" class="mono">--</span> mm
          <span class="kv">Yaw</span><span id="yaw" class="mono">0.0</span>°
          <span class="kv">State</span><span id="state" class="badge state">IDLE</span>
        </div>
      </div>

      <div class="panel" style="grid-column: 1 / -1; background:#e3f2fd;">
        <h3>Vive Navigation (Go To XY)</h3>
        <div class="row">
          <label>X</label><input id="vx" type="number" value="3000" step="50">
          <label>Y</label><input id="vy" type="number" value="1500" step="50">
          <button class="blue" onclick="goToVivePoint()">GO</button>
          <button class="stop" onclick="stopMotor()">STOP</button>
        </div>
        <div style="margin-top:8px;" class="mono">
          Vive X=<span id="vxNow">--</span> Y=<span id="vyNow">--</span>
        </div>
      </div>
    </div>

  </div>

  <script>
    function setMode(m) {
      fetch('/mode?m=' + m);
      document.getElementById('modeBadge').textContent = m.toUpperCase();
      if (m === 'wall') document.getElementById('wallEnable').checked = true;
      if (m === 'manual') document.getElementById('wallEnable').checked = false;
    }

    function updateControl() {
      const s = parseInt(document.getElementById('speedSlider').value);
      const t = parseInt(document.getElementById('steeringSlider').value);
      document.getElementById('speedValue').textContent = s;
      document.getElementById('steeringValue').textContent = t;
      fetch('/control?speed=' + s + '&steering=' + t);
    }

    function stopMotor() { fetch('/stop'); }

    function updatePID() {
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;
      fetch(`/pid?kp=${kp}&ki=${ki}&kd=${kd}`);
    }

    function toggleWall() {
      const en = document.getElementById('wallEnable').checked ? '1' : '0';
      fetch('/wall/enable?enable=' + en);
    }

    function updateWallGoals() {
      const fg = document.getElementById('frontGoal').value;
      const r1 = document.getElementById('rightGoal1').value;
      const r2 = document.getElementById('rightGoal2').value;
      fetch(`/wall/goals?frontGoal=${fg}&rightGoal1=${r1}&rightGoal2=${r2}`);
    }

    function updateWallPD() {
      const kp = document.getElementById('wkp').value;
      const kd = document.getElementById('wkd').value;
      fetch(`/wall/pd?kp=${kp}&kd=${kd}`);
    }

    function goToVivePoint() {
      const x = document.getElementById('vx').value;
      const y = document.getElementById('vy').value;
      fetch(`/gotopoint?x=${x}&y=${y}`);
      document.querySelector("input[name='mode'][value='vive']").checked = true;
      document.getElementById('modeBadge').textContent = 'VIVE';
    }

    // Status polling
    setInterval(() => {
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          document.getElementById('leftTarget').textContent = d.leftTarget;
          document.getElementById('leftCurrent').textContent = d.leftCurrent;
          document.getElementById('rightTarget').textContent = d.rightTarget;
          document.getElementById('rightCurrent').textContent = d.rightCurrent;
          document.getElementById('pwmL').textContent = d.leftPWM;
          document.getElementById('pwmR').textContent = d.rightPWM;

          document.getElementById('frontDist').textContent = d.front;
          document.getElementById('right1').textContent = d.right1;
          document.getElementById('right2').textContent = d.right2;
          document.getElementById('yaw').textContent = d.yaw;
          document.getElementById('state').textContent = (d.state ? 'WALL' : 'IDLE');

          document.getElementById('vxNow').textContent = d.vx;
          document.getElementById('vyNow').textContent = d.vy;

          const m = d.mode;
          const names = ['MANUAL','WALL','VIVE'];
          document.getElementById('modeBadge').textContent = names[m] || 'MANUAL';
          const radios = document.querySelectorAll("input[name='mode']");
          if (m === 0) radios[0].checked = true;
          if (m === 1) radios[1].checked = true;
          if (m === 2) radios[2].checked = true;
          document.getElementById('wallEnable').checked = (m === 1);
          document.getElementById('wallBadge').textContent = (m === 1) ? 'ON' : 'OFF';
        });
    }, 250);
  </script>

</body>
</html>
)rawliteral";

#endif

