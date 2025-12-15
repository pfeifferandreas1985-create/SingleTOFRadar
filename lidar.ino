/*
 * TRON Style LIDAR Radar System
 * Hardware: ESP-12F, VL53L1X, Micro Servo
 * 
 * Wiring:
 *  - VL53L1X SDA: GPIO 4 (D2)
 *  - VL53L1X SCL: GPIO 5 (D1)
 *  - Servo Signal: GPIO 14 (D5)
 *  - Status LED:   GPIO 2 (Built-in, Active Low)
 *  
 * Libraries required:
 *  - VL53L1X (Pololu)
 *  - WebSockets (Markus Sattler)
 *  - Servo (Built-in)
 *  - ESP8266WiFi (Built-in)
 */

#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>

// ===================================================================================
// CONFIGURATION
// ===================================================================================
const char* ssid     = "FRITZ!Box 6660 Cable UE";
const char* password = "28370714691864306613";

// STATIC IP CONFIGURATION (System B: TOF)
// Update "gateway" if your router is not 192.168.178.1
IPAddress local_IP(192, 168, 178, 71);
IPAddress gateway(192, 168, 178, 1);   // <--- CHECK THIS (Router IP)
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 178, 1);       // DNS (usually same as gateway)

const int PIN_SDA   = 4;
const int PIN_SCL   = 5;
const int PIN_SERVO = 14;
const int PIN_LED   = 2;

// Servo Settings
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
int servoDelay = 15; // ms between steps (controls sweep speed) - Dynamic

// Sensor Settings
#define SENSOR_BUDGET 20000 // us (20ms)

// ===================================================================================
// GLOBALS
// ===================================================================================
VL53L1X sensor;
Servo scanner;
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer server(80);

int currentAngle = 90;
int angleStep = 1;
unsigned long lastServoMove = 0;
bool longRangeMode = false;

// ===================================================================================
// FRONTEND (HTML/CSS/JS)
// ===================================================================================
const char index_html[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>TRON LIDAR</title>
<style>
  :root {
    --bg-color: #000000;
    --grid-color: #1a1a1a;
    --primary-color: #00FFFF;
    --alert-color: #FF0000;
    --panel-bg: rgba(0, 20, 20, 0.8);
  }
  body { 
    margin: 0; 
    background: var(--bg-color); 
    overflow: hidden; 
    font-family: 'Segoe UI', 'Roboto', monospace; 
    color: var(--primary-color);
    user-select: none;
  }
  #radar-container { 
    position: relative; 
    width: 100vw; 
    height: 100vh; 
    display: flex; 
    justify-content: center; 
    align-items: center; 
  }
  canvas { 
    border-radius: 50%; 
    background: radial-gradient(circle, #001111 0%, #000000 90%); 
    box-shadow: 0 0 30px rgba(0, 255, 255, 0.1);
  }
  .hud { 
    position: absolute; 
    top: 20px; 
    left: 20px; 
    z-index: 10; 
    pointer-events: none; 
    text-shadow: 0 0 5px var(--primary-color);
  }
  .controls {
    position: absolute;
    bottom: 20px;
    left: 50%;
    transform: translateX(-50%);
    z-index: 20;
    background: var(--panel-bg);
    padding: 15px;
    border: 1px solid var(--primary-color);
    border-radius: 10px;
    display: flex;
    gap: 20px;
    align-items: center;
    box-shadow: 0 0 15px rgba(0, 255, 255, 0.2);
    backdrop-filter: blur(5px);
  }
  h1 { margin: 0; font-size: 20px; letter-spacing: 2px; border-bottom: 1px solid var(--primary-color); display: inline-block; padding-bottom: 5px; }
  .data-row { margin-top: 10px; font-size: 14px; font-family: 'Courier New', monospace; }
  .label { opacity: 0.7; margin-right: 10px; }
  .value { font-weight: bold; font-size: 16px; }
  #status { font-weight: bold; color: var(--alert-color); }
  #status.connected { color: #00FF00; text-shadow: 0 0 5px #00FF00; }
  
  /* Inputs */
  button {
    background: transparent;
    border: 1px solid var(--primary-color);
    color: var(--primary-color);
    padding: 8px 16px;
    font-family: inherit;
    font-weight: bold;
    cursor: pointer;
    text-transform: uppercase;
    transition: all 0.2s;
  }
  button:hover { background: rgba(0, 255, 255, 0.2); box-shadow: 0 0 10px var(--primary-color); }
  button:active { transform: scale(0.95); }
  
  input[type=range] {
    -webkit-appearance: none;
    background: transparent;
    width: 150px;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    height: 16px; width: 16px;
    border-radius: 50%;
    background: var(--primary-color);
    cursor: pointer;
    margin-top: -6px;
    box-shadow: 0 0 10px var(--primary-color);
  }
  input[type=range]::-webkit-slider-runnable-track {
    width: 100%; height: 4px;
    background: #333;
    border-radius: 2px;
  }
  
  .control-group { display: flex; flex-direction: column; align-items: center; }
  .control-label { font-size: 10px; margin-bottom: 5px; opacity: 0.8; }

  /* Scan line effect overlay */
  .scan-overlay {
    position: absolute;
    top: 0; left: 0; right: 0; bottom: 0;
    background: linear-gradient(to bottom, rgba(0,255,255,0), rgba(0,255,255,0.05) 50%, rgba(0,255,255,0));
    background-size: 100% 4px;
    pointer-events: none;
    z-index: 5;
  }
</style>
</head>
<body>
<div class="hud">
  <h1>TRON_LIDAR_SYSTEM</h1>
  <div class="data-row"><span class="label">LINK:</span><span id="status">OFFLINE</span></div>
  <div class="data-row"><span class="label">AZIMUTH:</span><span id="angle" class="value">0</span>&deg;</div>
  <div class="data-row"><span class="label">RANGE:</span><span id="dist" class="value">0</span> mm</div>
  <div class="data-row"><span class="label">MODE:</span><span id="modeDisplay" class="value">SHORT</span></div>
</div>

<div class="controls">
  <div class="control-group">
    <span class="control-label">SCAN SPEED</span>
    <input type="range" id="speedSlider" min="0" max="100" value="80">
  </div>
  <div class="control-group">
    <span class="control-label">SENSOR RANGE</span>
    <button id="rangeBtn" onclick="toggleRange()">SHORT (1.3m)</button>
  </div>
</div>

<div class="scan-overlay"></div>
<div id="radar-container">
  <canvas id="radar"></canvas>
</div>
<script>
  const canvas = document.getElementById('radar');
  const ctx = canvas.getContext('2d');
  const statusEl = document.getElementById('status');
  const angleEl = document.getElementById('angle');
  const distEl = document.getElementById('dist');
  const modeDisplay = document.getElementById('modeDisplay');
  const rangeBtn = document.getElementById('rangeBtn');
  const speedSlider = document.getElementById('speedSlider');

  let width, height, centerX, centerY, maxRadius;
  // PERSISTENT MAP: Array of 360 degrees. 0 = No Data.
  const mapData = new Array(360).fill(0);
  let maxDist = 1500; // Default Short
  let ws;

  function resize() {
    width = window.innerWidth;
    height = window.innerHeight;
    canvas.width = width;
    canvas.height = height;
    centerX = width / 2;
    centerY = height / 2;
    maxRadius = Math.min(width, height) / 2 - 40;
  }
  window.addEventListener('resize', resize);
  resize();

  // WebSocket Connection
  function connect() {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen = () => { 
      statusEl.textContent = "ESTABLISHED"; 
      statusEl.classList.add('connected'); 
    };
    ws.onclose = () => { 
      statusEl.textContent = "LOST - RETRYING"; 
      statusEl.classList.remove('connected'); 
      setTimeout(connect, 2000);
    };
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        angleEl.textContent = data.angle;
        distEl.textContent = data.distance;
        
        // UPDATE MAP
        // Only update if valid. If dist > max, we consider it "clear" (set to max)
        let d = data.distance;
        if (d <= 0 || d > 8000) d = maxDist; // Cap at max for "infinity"
        
        // Simple noise filter: Don't update if change is drastic? 
        // For now, direct update:
        mapData[data.angle % 360] = d;

      } catch(e) {}
    };
  }
  connect();

  // Controls
  let isLongRange = false;
  function toggleRange() {
    isLongRange = !isLongRange;
    if (isLongRange) {
      ws.send('R1');
      rangeBtn.textContent = "LONG (4.0m)";
      modeDisplay.textContent = "LONG";
      maxDist = 4000;
    } else {
      ws.send('R0');
      rangeBtn.textContent = "SHORT (1.3m)";
      modeDisplay.textContent = "SHORT";
      maxDist = 1500;
    }
    // Optional: Clear map on mode switch?
    // mapData.fill(0); 
  }

  speedSlider.oninput = function() {
    const val = parseInt(this.value);
    const delay = Math.floor(100 - (val * 0.95)); 
    if(ws && ws.readyState === WebSocket.OPEN) {
      ws.send('S' + delay);
    }
  };

  function drawGrid() {
    ctx.strokeStyle = '#1a1a1a';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let r = 0.25; r <= 1; r += 0.25) {
      ctx.arc(centerX, centerY, maxRadius * r, 0, Math.PI * 2);
    }
    ctx.stroke();
    
    ctx.beginPath();
    ctx.moveTo(centerX - maxRadius, centerY);
    ctx.lineTo(centerX + maxRadius, centerY);
    ctx.moveTo(centerX, centerY - maxRadius);
    ctx.lineTo(centerX, centerY + maxRadius);
    ctx.stroke();
  }

  function draw() {
    ctx.fillStyle = 'rgba(0, 0, 0, 0.1)'; // Slight trail effect
    ctx.fillRect(0, 0, width, height);

    drawGrid();

    // DRAW MAP
    ctx.strokeStyle = '#00FFFF';
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    let firstPoint = true;

    for (let i = 0; i < 360; i++) {
      const dist = mapData[i];
      if (dist === 0) continue; // No data for this angle

      const rad = -(i * Math.PI / 180);
      
      // Clamp for drawing
      let drawDist = dist;
      if (drawDist > maxDist) drawDist = maxDist;
      
      const r = (drawDist / maxDist) * maxRadius;
      const x = centerX + Math.cos(rad) * r;
      const y = centerY + Math.sin(rad) * r;

      if (firstPoint) {
        ctx.moveTo(x, y);
        firstPoint = false;
      } else {
        // Only connect if points are close (avoid lines across the center)
        // Simple check: just lineTo. 
        // For better radar look: draw points or small lines.
        // Let's draw connected lines for "walls"
        ctx.lineTo(x, y);
      }
      
      // Draw Point (Glow)
      ctx.fillStyle = '#00FFFF';
      ctx.fillRect(x-1, y-1, 2, 2);
    }
    ctx.stroke();
    
    // Draw Scanner Line
    const scanRad = -(parseInt(angleEl.textContent) * Math.PI / 180);
    ctx.strokeStyle = 'rgba(0, 255, 0, 0.5)';
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX + Math.cos(scanRad)*maxRadius, centerY + Math.sin(scanRad)*maxRadius);
    ctx.stroke();

    requestAnimationFrame(draw);
  }
  draw();
</script>
</body>
</html>
)=====";

// ===================================================================================
// WEBSOCKET EVENT HANDLER
// ===================================================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    // Command Parsing
    if (payload[0] == 'S') {
      int val = atoi((const char*)&payload[1]);
      if (val < 5) val = 5;
      if (val > 200) val = 200;
      servoDelay = val;
      Serial.print("Servo Delay set to: "); Serial.println(servoDelay);
    }
    else if (payload[0] == 'R') {
      int val = atoi((const char*)&payload[1]);
      bool newMode = (val == 1);
      
      if (newMode != longRangeMode) {
        longRangeMode = newMode;
        Serial.print("Switching to "); Serial.println(longRangeMode ? "LONG RANGE" : "SHORT RANGE");
        
        sensor.stopContinuous();
        delay(10);
        
        if (longRangeMode) {
          sensor.setDistanceMode(VL53L1X::Long);
          sensor.setMeasurementTimingBudget(33000); 
        } else {
          sensor.setDistanceMode(VL53L1X::Short);
          sensor.setMeasurementTimingBudget(20000);
        }
        
        sensor.startContinuous(longRangeMode ? 33 : 20);
      }
    }
  }
}

// ===================================================================================
// SETUP
// ===================================================================================
void setup() {
  // 1. Init Serial
  Serial.begin(115200);
  Serial.println("\n\n--- TRON LIDAR SYSTEM (TOF) STARTING ---");

  // 2. Init Pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); // LED Off

  // 3. Init I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000); // 400kHz I2C
  Serial.println("I2C Initialized");

  // 4. Init Sensor
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("CRITICAL FAILURE: VL53L1X Sensor not found!");
    // FAILURE: Rapid Blink
    while (1) {
      digitalWrite(PIN_LED, LOW);  delay(50);
      digitalWrite(PIN_LED, HIGH); delay(50);
      Serial.print(".");
    }
  }
  Serial.println("VL53L1X Sensor Initialized");
  
  // Default: Short Mode
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(SENSOR_BUDGET);
  sensor.startContinuous(SENSOR_BUDGET / 1000);

  // 5. Init Servo
  scanner.attach(PIN_SERVO);
  scanner.write(currentAngle);
  Serial.println("Servo Attached");

  // 6. Init WiFi (Station Mode with Static IP)
  WiFi.mode(WIFI_STA);
  
  // Config Static IP BEFORE begin
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(PIN_LED, !digitalRead(PIN_LED)); // Blink while connecting
  }
  digitalWrite(PIN_LED, HIGH); // LED Off when connected
  
  Serial.println("");
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  // 7. Init Servers
  server.on("/", []() {
    server.send_P(200, "text/html", index_html);
  });
  server.begin();
  Serial.println("HTTP Server Started");
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket Server Started");
}

// ===================================================================================
// LOOP
// ===================================================================================
void loop() {
  webSocket.loop();
  server.handleClient();

  unsigned long now = millis();

  // Non-blocking Servo Sweep
  if (now - lastServoMove >= servoDelay) {
    lastServoMove = now;
    
    // Update Angle
    currentAngle += angleStep;
    if (currentAngle >= SERVO_MAX_ANGLE || currentAngle <= SERVO_MIN_ANGLE) {
      angleStep = -angleStep; // Reverse direction
    }
    scanner.write(currentAngle);
    
    // Read Sensor
    if (sensor.dataReady()) {
      uint16_t dist = sensor.read(false); // Non-blocking read
      
      // Debug Output
      Serial.print("Angle: "); Serial.print(currentAngle);
      Serial.print("\tDist: "); Serial.println(dist);

      // Broadcast JSON
      String json = "{\"angle\":";
      json += currentAngle;
      json += ",\"distance\":";
      json += dist;
      json += "}";
      webSocket.broadcastTXT(json);
    }
  }
}
