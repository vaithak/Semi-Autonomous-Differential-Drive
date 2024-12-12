const char WEBPAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>Car Control Panel</title>
  <style>
    :root {
      --primary-color: #00e5ff;
      --secondary-color: #7b2fff;
      --background-color: #0a0b1e;
      --panel-bg: rgba(20, 21, 44, 0.7);
      --text-color: #ffffff;
    }
    
    body {
      min-height: 100vh;
      background: linear-gradient(135deg, var(--background-color), #1a1b3c);
      color: var(--text-color);
      font-family: 'Segoe UI', Arial, sans-serif;
      padding: 20px;
    }

    .control-panel {
      background: var(--panel-bg);
      border-radius: 15px;
      padding: 20px;
      margin: 20px auto;
      max-width: 600px;
    }

    h2, h3 {
      color: var(--primary-color);
      margin-bottom: 15px;
    }

    .mode-switch {
      margin: 20px 0;
      padding: 15px;
      background: rgba(10, 11, 30, 0.8);
      border-radius: 10px;
    }

    .slider-container {
      margin: 15px 0;
    }

    label {
      display: block;
      margin-bottom: 5px;
      color: var(--text-color);
    }

    input[type="range"] {
      width: 100%;
      margin: 10px 0;
    }

    input[type="number"] {
      background: rgba(10, 11, 30, 0.8);
      border: 1px solid var(--primary-color);
      color: var(--text-color);
      padding: 5px;
      border-radius: 5px;
    }

    select {
      background: rgba(10, 11, 30, 0.8);
      border: 1px solid var(--primary-color);
      color: var(--text-color);
      padding: 5px;
      border-radius: 5px;
      width: 100%;
    }

    button {
      background: var(--primary-color);
      color: var(--background-color);
      border: none;
      padding: 10px 20px;
      border-radius: 5px;
      cursor: pointer;
      margin-top: 10px;
    }

    button:hover {
      background: var(--secondary-color);
      color: var(--text-color);
    }

    .motor-control {
      display: none;
    }
  </style>
</head>
<body>
  <div class="control-panel">
    <h2>Operation Mode</h2>
    <div class="mode-switch">
      <label>
        <input type="radio" name="mode" value="autonomous" checked> Autonomous
      </label>
      <label>
        <input type="radio" name="mode" value="manual"> Manual Override
      </label>
    </div>

    <div class="motor-control">
      <h3>Manual Control</h3>
      <div class="slider-container">
        <label>Speed (0-100%):</label>
        <input type="range" id="speedSlider" min="0" max="100" value="0">
        <span id="speedValue">0</span>
      </div>
      <div class="slider-container">
        <label>Direction:</label>
        <select id="direction">
          <option value="Forward">Forward</option>
          <option value="Backward">Backward</option>
        </select>
      </div>
      <div class="slider-container">
        <label>Turn Rate (-50% to 50%):</label>
        <input type="range" id="turnSlider" min="-50" max="50" value="0">
        <span id="turnValue">0</span>
      </div>
    </div>

    <div class="settings-panel">
      <h3>Settings</h3>
      <div class="slider-container">
        <label>Default PWM (0-1023):</label>
        <input type="range" id="pwmSlider" min="0" max="1023" value="500">
        <span id="pwmValue">500</span>
      </div>

      <h3>PID Control</h3>
      <div class="pid-controls">
        <div class="slider-container">
          <label>Kp:</label>
          <input type="number" id="kp" step="0.1" value="0">
        </div>
        <div class="slider-container">
          <label>Ki:</label>
          <input type="number" id="ki" step="0.1" value="0">
        </div>
        <div class="slider-container">
          <label>Kd:</label>
          <input type="number" id="kd" step="0.1" value="0">
        </div>
        <div>
          <label>
            <input type="checkbox" id="pidEnabled"> Enable PID
          </label>
        </div>
        <button onclick="updatePID()">Update PID</button>
      </div>
    </div>
  </div>

  <script>
    // Mode switching
    const modeRadios = document.getElementsByName('mode');
    const motorControl = document.querySelector('.motor-control');

    modeRadios.forEach(radio => {
      radio.addEventListener('change', function() {
        if (this.value === 'manual') {
          motorControl.style.display = 'block';
          fetch('/setMode?mode=manual');
        } else {
          motorControl.style.display = 'none';
          fetch('/setMode?mode=autonomous');
        }
      });
    });

    // Motor controls
    const speedSlider = document.getElementById('speedSlider');
    const speedValue = document.getElementById('speedValue');
    const turnSlider = document.getElementById('turnSlider');
    const turnValue = document.getElementById('turnValue');
    const directionSelect = document.getElementById('direction');

    function updateMotor() {
      const speed = speedSlider.value;
      const direction = directionSelect.value;
      const turn = turnSlider.value * -1;
      fetch(`/setMotor?speed=${speed}&forwardBackward=${direction}&turnRate=${turn}`);
    }

    // Event listeners
    speedSlider.oninput = function() {
      speedValue.textContent = this.value;
      if (motorControl.style.display === 'block') updateMotor();
    }

    turnSlider.oninput = function() {
      turnValue.textContent = this.value;
      if (motorControl.style.display === 'block') updateMotor();
    }

    directionSelect.onchange = function() {
      if (motorControl.style.display === 'block') updateMotor();
    }

    // PWM control
    const pwmSlider = document.getElementById('pwmSlider');
    const pwmValue = document.getElementById('pwmValue');

    pwmSlider.oninput = function() {
      pwmValue.textContent = this.value;
    }

    pwmSlider.onchange = function() {
      fetch('/setPWM?defaultPWM=' + this.value);
    }

    // PID control
    function updatePID() {
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;
      const enabled = document.getElementById('pidEnabled').checked ? 1 : 0;
      fetch(`/setPID?kp=${kp}&ki=${ki}&kd=${kd}&enabled=${enabled}`);
    }
  </script>
</body>
</html>
)=====";