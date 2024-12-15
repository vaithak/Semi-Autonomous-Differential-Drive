const char MODE_SELECT_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Mode Selector</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            height: 100vh;
            background: #f4f4f9;
        }

        select {
            font-size: 16px;
            padding: 10px;
            margin: 20px 0;
            border: 2px solid #007bff;
            border-radius: 5px;
            background: #fff;
            cursor: pointer;
        }

        #grid {
            display: none;
            margin-top: 20px;
            width: 400px;
            height: 400px;
            border: 2px solid #007bff;
            position: relative;
            background: #e7eff6;
        }

        .cross {
            position: absolute;
        }

        .cross .horizontal, .cross .vertical {
            position: absolute;
            background: red;
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
            font-size: 16px;
            color: #333;
        }
    </style>
</head>
<body>
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
    <div id="grid"></div>
    <div id="coordinates"></div>

    <script>
        const modeSelector = document.getElementById('modeSelector');
        const grid = document.getElementById('grid');
        const coordinatesDisplay = document.getElementById('coordinates');
        let currentCross = null; // To keep track of the existing cross

        modeSelector.addEventListener('change', (event) => {
            const selectedMode = event.target.value;

            if (selectedMode === "gridMode") {
                grid.style.display = 'block';
                coordinatesDisplay.textContent = "Click on the grid to select a point.";
            } else {
                grid.style.display = 'none';
                coordinatesDisplay.textContent = "";
                sendModeRequest(selectedMode);
            }
        });

        grid.addEventListener('click', (event) => {
            const rect = grid.getBoundingClientRect();
            const x = Math.floor((event.clientX - rect.left) / (rect.width / 10)); // Example mapping
            const y = Math.floor((event.clientY - rect.top) / (rect.height / 10)); // Example mapping

            // Clear the previous cross
            if (currentCross) {
                currentCross.remove();
            }

            // Draw the new cross
            drawCross(x, y);

            // Update coordinates display
            coordinatesDisplay.textContent = `Selected Coordinates: x=${x}, y=${y}`;

            // Send the mode request with coordinates
            sendModeRequest(modeSelector.value, x, y);
        });

        function sendModeRequest(mode, x = null, y = null) {
            const url = x !== null && y !== null 
                ? `/setMode?mode=${mode}&x=${x}&y=${y}`
                : `/setMode?mode=${mode}`;
            
            const xhr = new XMLHttpRequest();
            xhr.open('GET', url, true);
            xhr.send();
        }

        function drawCross(x, y) {
            const cross = document.createElement('div');
            cross.classList.add('cross');

            const gridSize = grid.offsetWidth / 10; // Assuming 10x10 grid
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
            currentCross = cross; // Keep track of the current cross
        }
    </script>
</body>
</html>
)=====";