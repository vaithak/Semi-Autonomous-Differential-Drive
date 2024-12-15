const char WEBPAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>Test Toggle Visibility</title>
    <style>
        :root {
            --primary-color: #00e5ff;
            --secondary-color: #7b2fff;
            --background-color: #0a0b1e;
            --panel-bg: rgba(20, 21, 44, 0.7);
            --text-color: #ffffff;
        }

        body {
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
            max-width: 90%;
            width: 100%;
        }

        @media (min-width: 992px) {
            .control-panel {
                max-width: 800px;
            }
        }

        h2, h3 {
            color: var(--primary-color);
            margin-bottom: 15px;
        }

        .toggle-switch {
            position: relative;
            width: 300px;
            height: 60px;
            margin: 20px auto;
            background: rgba(10, 11, 30, 0.9);
            border-radius: 30px;
            border: 2px solid var(--primary-color);
            box-shadow: 0 0 15px rgba(0, 229, 255, 0.3);
        }

        .toggle-switch input {
            display: none;
        }

        .toggle-slider {
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            cursor: pointer;
            transition: 0.4s;
            overflow: hidden;
        }

        .toggle-slider:before {
            position: absolute;
            content: "AUTONOMOUS";
            height: 52px;
            width: 146px;
            left: 4px;
            bottom: 4px;
            background: var(--primary-color);
            border-radius: 26px;
            transition: 0.4s;
            text-align: center;
            line-height: 52px;
            color: var(--background-color);
            font-weight: bold;
            font-size: 1.2em;
            box-shadow: 0 0 10px rgba(0, 229, 255, 0.5);
        }

        input:checked + .toggle-slider:before {
            transform: translateX(146px);
            content: "MANUAL";
            background: var(--secondary-color);
        }

        .motor-control {
            padding: 20px;
            background: var(--panel-bg);
            border-radius: 15px;
            margin-top: 20px;
            border: 1px solid var(--primary-color);
            box-shadow: 0 0 20px rgba(0, 229, 255, 0.2);
        }

        .hidden {
            display: none !important;
        }

        .slider-container {
            margin: 15px 0;
            background: rgba(10, 11, 30, 0.9);
            padding: 20px;
            border-radius: 10px;
            border: 1px solid var(--primary-color);
            box-shadow: inset 0 0 10px rgba(0, 229, 255, 0.2);
        }

        label {
            display: block;
            margin-bottom: 5px;
            color: var(--text-color);
        }

        input[type="range"] {
            -webkit-appearance: none;
            width: 100%;
            height: 15px;
            border-radius: 7.5px;
            background: var(--background-color);
            outline: none;
            border: 1px solid var(--primary-color);
        }

        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: var(--primary-color);
            cursor: pointer;
            box-shadow: 0 0 10px rgba(0, 229, 255, 0.5);
        }

        select {
            background: rgba(10, 11, 30, 0.8);
            border: 1px solid var(--primary-color);
            color: var(--text-color);
            padding: 5px;
            border-radius: 5px;
            width: 100%;
        }

        #grid {
            display: none;
            margin-top: 20px;
            width: 400px;
            height: 400px;
            border: 2px solid var(--primary-color);
            position: relative;
            background: var(--panel-bg);
        }

        .cross {
            position: absolute;
        }

        .cross .horizontal, .cross .vertical {
            position: absolute;
            background: var(--primary-color);
        }

        .cross .horizontal {
            width: 20px;
            height: 2px;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }

        .cross .vertical {
            width: 2px;
            height: 20px;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }

        #coordinates {
            margin-top: 10px;
            color: var(--text-color);
        }

        .auto-control {
            padding: 20px;
            background: var(--panel-bg);
            border-radius: 15px;
            margin-top: 20px;
            border: 1px solid var(--primary-color);
            box-shadow: 0 0 20px rgba(0, 229, 255, 0.2);
        }

        .emergency-stop {
            width: 120px;
            height: 120px;
            border-radius: 50%;
            background: linear-gradient(145deg, #ff3636, #cc0000);
            border: none;
            color: white;
            font-weight: bold;
            font-size: 1.2em;
            cursor: pointer;
            margin: 20px auto;
            display: block;
            box-shadow: 0 0 20px rgba(255, 0, 0, 0.3),
                        inset 0 0 15px rgba(0, 0, 0, 0.2);
            transition: transform 0.1s, box-shadow 0.1s;
        }

        .emergency-stop:hover {
            transform: scale(1.05);
            box-shadow: 0 0 30px rgba(255, 0, 0, 0.5),
                        inset 0 0 15px rgba(0, 0, 0, 0.2);
        }

        .emergency-stop:active {
            transform: scale(0.95);
            box-shadow: 0 0 15px rgba(255, 0, 0, 0.3),
                        inset 0 0 15px rgba(0, 0, 0, 0.4);
        }

        @media (max-width: 768px) and (orientation: landscape) {
            body {
                padding: 10px;
            }

            .control-panel {
                padding: 10px;
                margin: 10px auto;
                max-width: 100%;
            }

            h2, h3 {
                font-size: 1.2em;
            }

            .toggle-switch {
                width: 100%;
                height: 50px;
            }

            .toggle-slider:before {
                width: 50%;
                height: 42px;
                line-height: 42px;
                left: 4px;
                bottom: 4px;
                font-size: 1em;
            }

            input:checked + .toggle-slider:before {
                transform: translateX(calc(100% - 8px));
            }

            .slider-container {
                padding: 10px;
            }

            input[type="range"] {
                height: 10px;
            }

            input[type="range"]::-webkit-slider-thumb {
                width: 20px;
                height: 20px;
            }

            .emergency-stop {
                width: 80px;
                height: 80px;
                font-size: 1em;
            }

            select {
                font-size: 0.9em;
            }
        }
    </style>
</head>
<body>
    <div class="control-panel">
        <h2>Operation Mode</h2>
        <div class="toggle-switch">
            <input type="checkbox" id="modeToggle">
            <label class="toggle-slider" for="modeToggle"></label>
        </div>

        <div class="auto-control">
            <h3>Autonomous Control</h3>
            <div class="slider-container">
                <label>Select Mode:</label>
                <select id="modeSelector">
                    <option value="" disabled selected>Select a Mode</option>
                    <option value="leftWallFollow">Left wall follow</option>
                    <option value="rightWallFollow">Right wall follow</option>
                    <option value="attackRampBlueTower">Attack Ramp Blue Tower</option>
                    <option value="attackRampRedTower">Attack Ramp Red Tower</option>
                    <option value="attackGroundNexusRight">Attack Nexus on the right end</option>
                    <option value="attackGroundNexusLeft">Attack Nexus on the left end</option>
                    <option value="attackBlueGroundNexusCenter">Attack Blue Nexus in the center</option>
                    <option value="attackRedGroundNexusCenter">Attack Red Nexus in the center</option>
                    <option value="gridMode">Attack TA Bot (select point to move to)</option>
                </select>
            </div>
            <div id="grid"></div>
            <div id="coordinates"></div>
        </div>

        <div class="motor-control hidden">
            <h3>Manual Control</h3>
            <div class="slider-container">
                <label>Speed (In percent):</label>
                <input type="range" id="speedSlider" min="-100" max="100" value="0">
                <span id="speedValue">0</span>
            </div>
            <div class="slider-container">
                <label>Turn Rate (-50% to 50%):</label>
                <input type="range" id="turnSlider" min="-50" max="50" value="0">
                <span id="turnValue">0</span>
            </div>
            <h3>Servo Control</h3>
            <div class="slider-container">
                <label>Servo (On/Off)</label>
                <select id="servo">
                    <option value="On">On</option>
                    <option value="Off">Off</option>
                </select>
            </div>
            <button class="emergency-stop" id="emergencyStop">BIG RED<br>BUTTON</button>
        </div>
    </div>

    <script>
        const modeToggle = document.getElementById('modeToggle');
        const motorControl = document.querySelector('.motor-control');
        const autoControl = document.querySelector('.auto-control');
        const modeSelector = document.getElementById('modeSelector');
        const grid = document.getElementById('grid');
        const coordinatesDisplay = document.getElementById('coordinates');
        let currentCross = null;

        modeToggle.addEventListener('change', function() {
            if (this.checked) {
                motorControl.classList.remove('hidden');
                autoControl.classList.add('hidden');
                sendModeRequest("manual");
            } else {
                motorControl.classList.add('hidden');
                autoControl.classList.remove('hidden');
                sendModeRequest("autonomous");
            }
        });


        modeSelector.addEventListener('change', (event) => {
            const selectedMode = event.target.value;

            if (selectedMode === "gridMode") {
                grid.style.display = 'block';
                coordinatesDisplay.textContent = "Click on the grid to select a point.";
            } else {
                grid.style.display = 'none';
                coordinatesDisplay.textContent = "";
                sendAutonomousModeRequest(selectedMode);
            }
        });

        grid.addEventListener('click', (event) => {
            const rect = grid.getBoundingClientRect();
            const x = Math.floor((event.clientX - rect.left) / (rect.width / 10));
            const y = Math.floor((event.clientY - rect.top) / (rect.height / 10));

            if (currentCross) {
                currentCross.remove();
            }

            drawCross(x, y);
            coordinatesDisplay.textContent = `Selected Coordinates: x=${x}, y=${y}`;
            sendAutonomousModeRequest(modeSelector.value, x, y);
        });

        function sendModeRequest(mode) {
            const url = `/manual?bool=${mode === "manual"}`;
            const xhr = new XMLHttpRequest();
            xhr.open('GET', url, true);
            xhr.send();
        }

        function sendAutonomousModeRequest(mode, x = null, y = null) {
            const url = x !== null && y !== null 
                ? `/setMode?mode=${mode}&x=${x}&y=${y}`
                : `/setMode?mode=${mode}`;
            
            const xhr = new XMLHttpRequest();
            xhr.open('GET', url, true);
            xhr.send();
        }

        function updateMotor(speed, turnRate) {
            const queryString = `${window.location.origin}/setMotor?` +
                `speed=${Math.abs(speed)}` +
                `&forwardBackward=${speed >= 0 ? "Forward" : "Backward"}` +
                `&turnRate=${turnRate*-1}`;

            fetch(queryString)
                .then(response => response.text())
                .then(data => {
                    console.log('Motor response:', data);
                })
                .catch(error => {
                    console.error('Error setting motor:', error);
                });
        }

        function updateServo() {
            const servo = servoSelect.value;
            fetch(`/setServo?servo=${servo}`);
            // Add speed control maybe later
        }

        function drawCross(x, y) {
            const cross = document.createElement('div');
            cross.classList.add('cross');

            const gridSize = grid.offsetWidth / 10;
            cross.style.left = `${x * gridSize}px`;
            cross.style.top = `${y * gridSize}px`;
            cross.style.width = `${gridSize}px`;
            cross.style.height = `${gridSize}px`;

            const horizontal = document.createElement('div');
            horizontal.classList.add('horizontal');

            const vertical = document.createElement('div');
            vertical.classList.add('vertical');

            cross.appendChild(horizontal);
            cross.appendChild(vertical);

            grid.appendChild(cross);
            currentCross = cross;
        }

        const speedSlider = document.getElementById('speedSlider');
        const speedValue = document.getElementById('speedValue');
        const turnSlider = document.getElementById('turnSlider');
        const turnValue = document.getElementById('turnValue');

        speedSlider.addEventListener('input', function() {
            speedValue.textContent = this.value;
            updateMotor(this.value, turnSlider.value);
        });

        turnSlider.addEventListener('input', function() {
            turnValue.textContent = this.value;
            updateMotor(speedSlider.value, this.value);
        });

        const emergencyStop = document.getElementById('emergencyStop'); // BIG RED BUTTON
        const servoSelect = document.getElementById('servo');


        emergencyStop.addEventListener('click', function() {
            speedSlider.value = 0;
            speedValue.textContent = "0";
            
            turnSlider.value = 0;
            turnValue.textContent = "0";
            
            servoSelect.value = "Off";
            
            updateMotor(0, 0);
            updateServo();
        });

        document.addEventListener('keydown', function(event) {
            if (motorControl.classList.contains('hidden')) {
                return;
            }

            switch(event.key.toLowerCase()) {
                case 'w':
                    speedSlider.value = Math.min(parseInt(speedSlider.value) + 5, 100);
                    speedValue.textContent = speedSlider.value;
                    updateMotor(speedSlider.value, turnSlider.value);
                    break;
                case 's':
                    speedSlider.value = Math.max(parseInt(speedSlider.value) - 5, -100);
                    speedValue.textContent = speedSlider.value;
                    updateMotor(speedSlider.value, turnSlider.value);
                    break;
                case 'a':
                    turnSlider.value = Math.max(parseInt(turnSlider.value) - 5, -50);
                    turnValue.textContent = turnSlider.value;
                    updateMotor(speedSlider.value, turnSlider.value);
                    break;
                case 'd':
                    turnSlider.value = Math.min(parseInt(turnSlider.value) + 5, 50);
                    turnValue.textContent = turnSlider.value;
                    updateMotor(speedSlider.value, turnSlider.value);
                    break;
                case ' ':
                    emergencyStop.click();
                    break;
                case 'x':
                    servoSelect.value = (servoSelect.value === 'On') ? 'Off' : 'On';
                    updateServo();
                    break;
                default:
                    break;
            }
        });
    </script>
</body>
</html>
)=====";