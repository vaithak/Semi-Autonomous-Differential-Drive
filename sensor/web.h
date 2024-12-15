const char WEBPAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>Sensor Data Display</title>
    <style>
        :root {
            --primary-color: #00e5ff;
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
        
        .dashboard {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 20px;
            max-width: 1200px;
            margin: 0 auto;
        }
        
        .stat-box {
            background: var(--panel-bg);
            border-radius: 15px;
            padding: 25px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(0, 229, 255, 0.1);
        }
        
        .stat-label {
            font-size: 1.2em;
            color: rgba(255, 255, 255, 0.7);
            margin-bottom: 10px;
        }
        
        .stat-value {
            font-size: 2.5em;
            font-weight: 600;
            color: var(--primary-color);
            text-shadow: 0 0 15px rgba(0, 229, 255, 0.5);
        }
    </style>
</head>
<body>
    <div class="dashboard">
        <div class="stat-box">
            <div class="stat-label">Front Distance</div>
            <div id="frontDistance" class="stat-value">0 mm</div>
        </div>
        <div class="stat-box">
            <div class="stat-label">Left Distance</div>
            <div id="leftDistance" class="stat-value">0 mm</div>
        </div>
        <div class="stat-box">
            <div class="stat-label">Right Distance</div>
            <div id="rightDistance" class="stat-value">0 mm</div>
        </div>
        <div class="stat-box">
            <div class="stat-label">Angle</div>
            <div id="angle" class="stat-value">0</div>
        </div>
        <div class="stat-box">
            <div class="stat-label">Direction</div>
            <div id="direction" class="stat-value">-</div>
        </div>
        <div class="stat-box">
            <div class="stat-label">Speed</div>
            <div id="speed" class="stat-value">0</div>
        </div>
        <div class="stat-box">
            <div class="stat-label">Vive</div>
            <div id="vive" class="stat-value">-</div>
        </div>
        <div class="stat-box">
            <div class="stat-label">Top Hat</div>
            <div id="topHat" class="stat-value">-</div>
        
        </div>
    </div>
    <script>
        // Remove the fetchSensorData function and its setInterval

        // Add WebSocket client script
        var ws = new WebSocket('ws://' + window.location.hostname + ':81/');
        ws.onmessage = function(event) {
            var data = JSON.parse(event.data);
            document.getElementById('frontDistance').textContent = data.front + ' mm';
            document.getElementById('leftDistance').textContent = data.left + ' mm';
            document.getElementById('rightDistance').textContent = data.right + ' mm';
            document.getElementById('angle').textContent = data.angle;
            document.getElementById('direction').textContent = data.direction;
            document.getElementById('speed').textContent = data.speed;
            document.getElementById('vive').textContent = data.x + ', ' + data.y;
            document.getElementById('topHat').textContent = data.HP;
        };

        ws.onerror = function(error) {
            console.error('WebSocket Error:', error);
        };
    </script>
</body>
</html>
)=====";