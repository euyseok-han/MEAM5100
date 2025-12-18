#ifndef WEBSITE_H
#define WEBSITE_H

// Store page in flash (saves RAM on ESP32)
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>RoBa</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { 
      font-family: Arial, sans-serif; 
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
    .bfs-section     { background: #f5f5f5; }

    button { 
      padding: 10px 20px; 
      margin: 5px;
      font-size: 15px; 
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
    .spin    { background: #673ab7;  color: white; }

    .bfs-input-row {
      display: flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 10px;
      flex-wrap: wrap;
    }
    .bfs-input-row input[type="number"] {
      width: 80px;
      padding: 4px;
    }

    #queueBox {
      margin-top: 8px;
      padding: 6px;
      background: #ffffff;
      border-radius: 4px;
      border: 1px solid #ddd;
      font-family: monospace;
      font-size: 14px;
    }

    /* === COMBINED QUEUE PANEL (Option C) === */
    .combined-queue {
      background: #fff7e6;
    }

    .queue-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin-top: 10px;
    }

    .queue-list, .queue-buttons {
      background: #ffffff;
      border-radius: 6px;
      padding: 12px;
      border: 1px solid #ddd;
    }

    .queue-box {
      margin-top: 6px;
      padding: 6px;
      background: #fafafa;
      border: 1px solid #ddd;
      border-radius: 4px;
      font-family: monospace;
      font-size: 14px;
      max-height: 120px;
      overflow-y: auto;
    }

    .queue-btn {
      display: block;
      width: 100%;
      padding: 10px;
      margin-top: 8px;
      border-radius: 5px;
      border: none;
      cursor: pointer;
      background: #4CAF50;
      color: white;
      font-size: 15px;
    }
    .queue-btn.stop-btn {
      background: #f44336;
    }
  </style>
</head>
<body>
  <div class="container">
    <!-- MODE SELECTION PANEL -->
    <div class="panel mode-section" style="margin-bottom:15px; text-align:left;">
      <label>
        <input type="radio" name="mode" value="manual" onclick="setMode('manual')">
        Manual Control
      </label>
      <label>
        <input type="radio" name="mode" value="vive" onclick="setMode('vive')">
        BFS (Vive Navigation)
      </label>
      <label>
        <input type="radio" name="mode" value="wall" onclick="setMode('wall')">
        Wall Follow (disabled)
      </label>

      <div id="modeStatus" style="margin-top:10px; font-weight:bold;">
        Mode: Unknown
      </div>
    </div>

    <div class="layout-grid">
      
      <!-- CONTROL PANEL -->
      <div class="panel control-section">
        <h3>Speed Control <span id="direction" class="direction-indicator stopped">STOPPED</span></h3>

        <label>Base Speed: <span id="speedValue" class="speed-value">0</span> RPM</label>
        <input type="range" id="speedSlider" min="-120" max="120" value="0" step="5" oninput="updateControlLocal()">

        <label>Steering: <span id="steeringValue" class="speed-value">0</span></label>
        <input type="range" id="steeringSlider" min="-60" max="60" value="0" step="5" oninput="updateControlLocal()">

        <button class="stop-btn" onclick="stopMotor()">STOP("S" key)</button>
        <button onclick="setControl(currentSpeed, currentSteering)" style="background:#008CFF;">Go("D" key)</button>
        <button onclick="initSpeed()" style="background:#008CFF;">Init("I" key)</button>
      </div>

      <!-- PID TUNING -->
      <div class="panel pid-section">
        <h3>PID Tuning</h3>

        <label>Kp:</label><input id="kp" type="number" value="0.4" step="0.1" onchange="updatePID()"><br><br>
        <label>Ki:</label><input id="ki" type="number" value="1.5" step="0.1" onchange="updatePID()"><br><br>
        <label>Kd:</label><input id="kd" type="number" value="0" step="0.01" onchange="updatePID()">
      </div>

      <!-- STATUS PANEL -->
      <div class="panel status">
        <h3>Live Status</h3>
        <pre id="statusBox">Loading...</pre>
      </div>

      <!-- BFS ROUTE PANEL -->
      <div class="panel bfs-section">
        <h3>BFS Route Planner</h3>
        <div class="bfs-input-row">
          <span>Start Node (optional):</span>
          <input type="number" id="bfsStart" min="0" max="40" step="1" placeholder="">  
          <span>Goal Node:</span>
          <input type="number" id="bfsGoal" min="0" max="40" step="1" placeholder="">
          <button onclick="callRoute()">BFS("B" key)</button>
        </div>
      </div>

      <!-- WALL-FOLLOW CONTROL PANEL -->
      <div class="panel" style="background:#e8fff5;">
        <h3>Wall-Follow Controls</h3>

        <div>
          <button onclick="enableWall(1)">Enable Wall-Follow</button>
          <button onclick="enableWall(0)" class="stop-btn">Disable</button>
        </div>

        <h4>Goal Distances (mm)</h4>
        <label>Front Goal:</label>
        <input id="frontGoal" type="number" value="100" min="50" max="800" step="10">
        <br>
        <label>Right Goal Front:</label>
        <input id="rightGoal1" type="number" value="60" min="50" max="800" step="10">
        <br>
        <label>Right Goal Back:</label>
        <input id="rightGoal2" type="number" value="60" min="50" max="800" step="10">
        <button onclick="updateWallGoals()">Update Goals</button>

        <h4>Wall-Follow PD</h4>
        <label>Kp:</label>
        <input id="wallKp" type="number" value="0.05" step="0.1">
        <br>
        <label>Kd:</label>
        <input id="wallKd" type="number" value="0.8" step="0.05">
        <br>
        <label>Kpa:</label>
        <input id="wallKpa" type="number" value="0.1" step="0.1">
        <button onclick="updateWallPD()">Update PD</button>
      </div>

      <div class="panel combined-queue">
        <h3>Route Queue & Controls</h3>

        <div class="queue-grid">

          <!-- Left side: Queue List -->
          <div class="queue-list">
            <h4>Queue</h4>
            <div id="queueDisplay" class="queue-box">(empty)</div>
          </div>

          <!-- Right side: Buttons -->
          <div class="queue-buttons">
            <h4>Actions</h4>
            <button id="pauseBtn" onclick="toggleQueuePause()" class="queue-btn">Pause</button>
            <button onclick="queueClear()" class="queue-btn stop-btn">Clear Queue("C" key)</button>
          </div>

            </div>
            <div class="goto-panel" style="
            margin-top:18px;
            background:#eefaff;
            padding:12px;
            border-radius:6px;
            border:1px solid #cce7ff;
      ">
        <h4>Go To Coordinate (Vive)</h4>

        <div class="bfs-input-row" style="margin-top:6px;">
          <span>X:</span>
          <input type="number" id="gotoX" placeholder="x" style="width:80px;">

          <span>Y:</span>
          <input type="number" id="gotoY" placeholder="y" style="width:80px;">

          <label style="margin-left:10px;">
            <input type="checkbox" id="gotoDead"> Dead(point with a Tower/Nexus)
          </label>
          <label style="margin-left:10px;">
            <input type="checkbox" id="gotoNearestNode" checked> Nearest Node?
          </label>
          <button onclick="sendGoToPoint()" class="queue-btn" style="width:auto;padding:6px 14px;background:#007bff;">
            Go("V" key)
          </button>
        </div>
      </div>
      </div>

      <!-- ROW 4: ATTACK BUTTONS -->
      <div class="panel attack-section" style="
          background:#ffe6e6;
          display:flex;
          justify-content:space-between;
          align-items:center;
          gap:12px;
      ">
        <button onclick="attackLowTower()"  style="flex:1; padding:14px;">Attack Low Tower</button>
        <button onclick="attackHighTower()" style="flex:1; padding:14px;">Attack High Tower</button>
        <button onclick="attackNexus()"     style="flex:1; padding:14px;">Attack Nexus</button>
        <button onclick="attackLast()"     style="flex:1; padding:14px;">Last Task</button>
        <button onclick="stopTask()"     style="flex:1; padding:14px;">Purge Tasks(P key)(</button>
      </div>

      <!-- ARM CONTROLS -->
      <div class="panel arm-section" style="
          background:#e6f7ff;
          display:flex;
          justify-content:space-between;
          align-items:center;
          gap:12px;
      ">
        <button onclick="returnArm()" style="flex:1; padding:14px;">Return Arm(Q)</button>
        <button onclick="attackArm()" style="flex:1; padding:14px;">Attack Arm(E)</button>
      </div>
    </div>
  </div>

  <script>
    let currentSpeed = 0;
    let currentSteering = 0;
    let wallPrefilled = false;

    // Press-and-hold driving constants and key state
    const DRIVE_SPEED = 60;   // forward speed when holding Up
    const STEER_SPEED = 30;   // steering magnitude when holding Left/Right
    const keyState = { up: false, down: false, left: false, right: false };

    function recomputeDriveFromKeys() {
      let speed = 0;
      let steer = 0;

      if (keyState.up)   speed = DRIVE_SPEED;
      if (keyState.down) speed = -DRIVE_SPEED;

      if (keyState.left)  {steer = -STEER_SPEED;
      }
      if (keyState.right) steer =  STEER_SPEED;
      if (speed) steer = steer * 2 / 3; 
      setControl(speed, steer);
    }

    // ================== DIRECTION INDICATOR ==================
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

    // ================== MANUAL CONTROL ==================
    // Slider moves: only update UI + local state (no server call)
    function updateControlLocal() {
      currentSpeed = parseInt(document.getElementById('speedSlider').value);
      currentSteering = parseInt(document.getElementById('steeringSlider').value);
      document.getElementById('speedValue').textContent = currentSpeed;
      document.getElementById('steeringValue').textContent = currentSteering;
      updateDirectionIndicator(currentSpeed, currentSteering);
    }

    // Single function that updates UI + sends command to /setspeed
    function setControl(speed, steering) {
      currentSpeed = Math.round(speed);
      currentSteering = Math.round(steering);
      document.getElementById('speedSlider').value = currentSpeed;
      document.getElementById('steeringSlider').value = currentSteering;
      document.getElementById('speedValue').textContent = currentSpeed;
      document.getElementById('steeringValue').textContent = currentSteering;

      updateDirectionIndicator(currentSpeed, currentSteering);

      fetch('/setspeed?speed=' + currentSpeed + '&steering=' + currentSteering)
        .then(() => console.log("Control submitted:", currentSpeed, currentSteering));
    }

    function stopMotor() {
      currentSpeed = 0;
      currentSteering = 0;
      document.getElementById('speedSlider').value = 0;
      document.getElementById('steeringSlider').value = 0;
      document.getElementById('speedValue').textContent = 0;
      document.getElementById('steeringValue').textContent = 0;
      updateDirectionIndicator(0, 0);

      fetch('/stop');
    }

    function initSpeed(){
      currentSpeed = 0;
      currentSteering = 0;
      document.getElementById('speedSlider').value = 0;
      document.getElementById('steeringSlider').value = 0;
      document.getElementById('speedValue').textContent = 0;
      document.getElementById('steeringValue').textContent = 0;
      updateDirectionIndicator(0, 0);
      }
    function stopTask() {
      currentSpeed = 0;
      currentSteering = 0;
      document.getElementById('speedSlider').value = 0;
      document.getElementById('steeringSlider').value = 0;
      document.getElementById('speedValue').textContent = 0;
      document.getElementById('steeringValue').textContent = 0;
      updateDirectionIndicator(0, 0);

      fetch('/stop_task');
    }

    function updatePID() {
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;
      fetch('/setpid?kp=' + kp + '&ki=' + ki + '&kd=' + kd);
    }

    // ================== MODE CONTROL ==================
    function setMode(m) {
      fetch('/mode?m=' + m)
        .then(() => refreshStatus());
    }

    // Sync radio buttons with robot mode coming from /status
    function updateModeUI(modeInt) {
      const modes = document.getElementsByName('mode');
      modes.forEach(r => {
        if (r.value === 'manual' && modeInt === 0) r.checked = true;
        if (r.value === 'wall'   && modeInt === 1) r.checked = true;
        if (r.value === 'vive'   && modeInt === 2) r.checked = true;
      });

      const modeNames = ['Manual', 'Wall Follow', 'BFS (Vive Navigation)'];
      const statusEl = document.getElementById('modeStatus');
      if (statusEl && modeInt >= 0 && modeInt < modeNames.length) {
        statusEl.textContent = 'Mode: ' + modeNames[modeInt];
      }
    }

    // ================== STATUS UPDATES ==================

    function clearBfsInputs(){
      document.getElementById("bfsStart").value = "";
      document.getElementById("bfsGoal").value = "";
    }

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
TOF   : front=${data.front}  r1=${data.right1} r2=${data.right2}
MODE  : ${data.mode}`;
          document.getElementById('statusBox').textContent = txt;

          // Update mode UI according to current firmware mode
          updateModeUI(data.mode);

          // Update pause button from firmware status
          if (typeof data.paused !== 'undefined') {
            queuePaused = !!data.paused;
            updatePauseButton();
          }

          // Update Queue panel
          updateQueueUI(data.queue);

          // Prefill Wall-Follow panel from status (if provided)
          if (!wallPrefilled) {
            const fg  = document.getElementById('frontGoal');
            const rg1 = document.getElementById('rightGoal1');
            const rg2 = document.getElementById('rightGoal2');
            const wkp = document.getElementById('wallKp');
            const wkd = document.getElementById('wallKd');
            const wkpa= document.getElementById('wallKpa');

            // Accept multiple possible key names
            if (typeof data.frontGoalDistance !== 'undefined' && fg) fg.value = data.frontGoalDistance;
            if (typeof data.rightGoalDistance1 !== 'undefined' && rg1) rg1.value = data.rightGoalDistance1;
            if (typeof data.rightGoalDistance2 !== 'undefined' && rg2) rg2.value = data.rightGoalDistance2;

            const kpVal = (typeof data.wallFollowKp !== 'undefined') ? data.wallFollowKp
                          : (typeof data.wallKp !== 'undefined') ? data.wallKp : undefined;
            const kdVal = (typeof data.wallFollowKd !== 'undefined') ? data.wallFollowKd
                          : (typeof data.wallKd !== 'undefined') ? data.wallKd : undefined;
            const kpaVal = (typeof data.wallFollowKpa !== 'undefined') ? data.wallFollowKpa
                          : (typeof data.wallKpa !== 'undefined') ? data.wallKpa : undefined;
            if (typeof kpVal !== 'undefined' && wkp) wkp.value = kpVal;
            if (typeof kdVal !== 'undefined' && wkd) wkd.value = kdVal;
            if (typeof kpaVal!== 'undefined' && wkpa)wkpa.value= kpaVal;

            wallPrefilled = true;
          }
        })
        .catch(err => {
          console.log(err);
        });
    }

    // ================== BFS ROUTE CALL ==================
    function callRoute() {
      const goal = document.getElementById('bfsGoal').value;
      const start = document.getElementById('bfsStart').value;

      fetch('/mode?m=vive')
      .then(() => {
        refreshStatus();  // Update mode immediately
      });
      if (goal === '') {
        console.log("Missing goal node");
        return;
      }
      let url = '/route?goal=' + goal;

      // Append start ONLY if user typed something
      if (start !== null && start !== '') {
        url += '&start=' + start;
      }

      fetch(url)
        .then(r => r.text())
        .then(t => {
          refreshStatus();  // Update queue immediately
          clearBfsInputs();
        })
    }

    // ================== QUEUE DISPLAY ==================
    function updateQueueUI(queueArr) {
      const el = document.getElementById('queueDisplay');
      if (!el) return;
      if (!queueArr || queueArr.length === 0) {
        el.textContent = '(empty)';
        return;
      }
      el.textContent = '[ ' + queueArr.join(' - ') + ' ]';
    }

    // ================== QUEUE CONTROL ACTIONS ==================
    let queuePaused = false;

    function updatePauseButton() {
      const b = document.getElementById('pauseBtn');
      if (!b) return;
      if (queuePaused) {
        b.textContent = 'Resume';
        b.style.backgroundColor = '#ff9800';
      } else {
        b.textContent = 'Pause';
        b.style.backgroundColor = '#4CAF50';
      }
    }

    function toggleQueuePause() {
      const next = !queuePaused;
      fetch('/queue/pause?enable=' + (next ? '1' : '0'))
        .then(r => r.text())
        .then(() => {
          queuePaused = next;
          updatePauseButton();
          refreshStatus();
        });
    }

    function queueClear() {
      fetch('/queue/clear').then(() => refreshStatus());
    }

    function sendGoToPoint() {
      const x = document.getElementById("gotoX").value;
      const y = document.getElementById("gotoY").value;
      const dead = document.getElementById("gotoDead").checked ? 1 : 0;
      const nearestNode = document.getElementById("gotoNearestNode").checked ? 1 : 0;
      fetch('/mode?m=vive')
      .then(() => {
        refreshStatus();  // Update mode immediately
      });
      if (x === "" || y === "") {
        console.log("Missing coordinate");
        return;
      }

      const url = `/gotopoint?x=${x}&y=${y}&dead=${dead}&nearest=${nearestNode}`;

      fetch(url)
        .then(r => r.text())
        .then(t => {
          console.log("GoTo response:", t);

          document.getElementById("gotoX").value = "";
          document.getElementById("gotoY").value = "";
          document.getElementById("gotoDead").checked = false;
          document.getElementById("gotoNearestNode").checked = true;
        });

      // Switch mode to VIVE automatically
      fetch('/mode?m=vive');
    }
    function enableWall(en) {
      fetch('/wall/enable?enable=' + en)
        .then(r => r.text())
        .then(t => {
          refreshStatus();
        });
    }

    function updateWallGoals() {
      const f  = document.getElementById('frontGoal').value;
      const r1 = document.getElementById('rightGoal1').value;
      const r2 = document.getElementById('rightGoal2').value;

      const url = `/wall/goals?frontGoal=${f}&rightGoal1=${r1}&rightGoal2=${r2}`;

      fetch(url).then(r => r.text()).then(t => {
        refreshStatus();
      });
    }

    function updateWallPD() {
      const kp = document.getElementById('wallKp').value;
      const kd = document.getElementById('wallKd').value;
      const kpa= document.getElementById('wallKpa').value;

      fetch(`/wall/pd?kp=${kp}&kd=${kd}&kpa=${kpa}`)
        .then(r => r.text())
        .then(t => {
          refreshStatus();
        });
    }

    function attackLowTower() {
      fetch('/attack?target=lowtower');
    }

    function attackHighTower() {
      fetch('/attack?target=hightower');
    }

    function attackNexus() {
      fetch('/attack?target=nexus');
    }

    function attackLast() {
      fetch('/attack?target=lastTask');
    }

    function returnArm() {
      fetch('/arm?cmd=return');
    }

    function attackArm() {
      fetch('/arm?cmd=attack');
    }

    document.addEventListener('keydown', (e) => {
      if (e.repeat) return; // prevent auto-repeat

      switch (e.key) {
        case 'ArrowUp':    keyState.up = true;    recomputeDriveFromKeys(); break;
        case 'ArrowDown':  keyState.down = true;  recomputeDriveFromKeys(); break;
        case 'ArrowLeft':  keyState.left = true;  recomputeDriveFromKeys(); break;
        case 'ArrowRight': keyState.right = true; recomputeDriveFromKeys(); break;
      }

      if (e.key === 's' || e.key === 'S') {
        stopMotor();
        return;
      }
      if (e.key === 'p' || e.key === 'P') {
        stopTask();
        return;
      }  
      if (e.key === "d" || e.key === "D") {
        setControl(currentSpeed, currentSteering);
        return;
      }
      if (e.key === "i" || e.key === "I") {
        initSpeed();
        return;
      }
      if (e.key === "b" || e.key === "B") {
        callRoute();
        return;
      }
      if (e.key === "v" || e.key === "V") {
        sendGoToPoint();
        return;
      }
      if (e.key === "q" || e.key === "Q") {
        returnArm();
        return;
      }
      if (e.key === "E" || e.key === "e") {
        attackArm();
        return;
}
      if (e.key === "c" || e.key === "C") {
        queueClear();
        return;
      }
      document.getElementById('speedSlider').value = currentSpeed;
      document.getElementById('steeringSlider').value = currentSteering;
      updateControlLocal();
    });

    // Stop or adjust on key release
    document.addEventListener('keyup', (e) => {
      switch (e.key) {
        case 'ArrowUp':    keyState.up = false;    break;
        case 'ArrowDown':  keyState.down = false;  break;
        case 'ArrowLeft':  keyState.left = false;  break;
        case 'ArrowRight': keyState.right = false; break;
        default: return;
      }
      recomputeDriveFromKeys();
    });

    // Enter on BFS inputs = "Add Route"
    document.getElementById("bfsStart").addEventListener("keydown", function(e) {
      if (e.key === "Enter") {
        callRoute();
        clearBfsInputs();
      }
    });

    document.getElementById("bfsGoal").addEventListener("keydown", function(e) {
      if (e.key === "Enter") {
        callRoute();
        clearBfsInputs();
      }
    });

  </script>
</body>
</html>
)rawliteral";

#endif