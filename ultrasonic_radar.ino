/*
 * 360° Compound Radar System (Tron Style)
 * Hardware: ESP-12F, 4x HC-SR04, Servo
 * 
 * Wiring:
 *  - Servo: GPIO 2
 *  - Trigger Group A (Front/Back): GPIO 14
 *  - Trigger Group B (Left/Right): GPIO 12
 *  - Echo Front: GPIO 5
 *  - Echo Back:  GPIO 4
 *  - Echo Left:  GPIO 13
 *  - Echo Right: GPIO 15 (Must be LOW at boot!)
 */

#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// ===================================================================================
// CONFIGURATION
// ===================================================================================
const char* ssid     = "FRITZ!Box 6660 Cable UE";
const char* password = "28370714691864306613";

// STATIC IP CONFIGURATION (System A: Ultrasonic)
// Update "gateway" if your router is not 192.168.178.1
IPAddress local_IP(192, 168, 178, 70);
IPAddress gateway(192, 168, 178, 1);   // <--- CHECK THIS (Router IP)
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 178, 1);

// Pins
const int PIN_SERVO     = 2;
const int PIN_TRIG_A    = 14; // Front + Back
const int PIN_TRIG_B    = 12; // Left + Right
const int PIN_ECHO_F    = 5;
const int PIN_ECHO_B    = 4;
const int PIN_ECHO_L    = 13;
const int PIN_ECHO_R    = 15;

// Settings
const int SERVO_MIN     = 40;  // Sector Scan Start (Wider)
const int SERVO_MAX     = 140; // Sector Scan End (Wider)
const int SERVO_STEP    = 3;   // 3 Degree steps (Balanced)
const int MAX_DIST_CM   = 400; 
const int TIMEOUT_US    = 15000; // 15ms

// ===================================================================================
// GLOBALS
// ===================================================================================
Servo scanner;
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer server(80);

int currentAngle = 90;
int angleStep = SERVO_STEP;
int servoDelay = 15; // Default delay (Controlled by Slider)

// ===================================================================================
// FRONTEND (HTML/CSS/JS)
// ===================================================================================
const char index_html[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>COMPOUND RADAR 360</title>
<style>
  :root { --bg: #000; --prim: #00FFFF; }
  body { margin: 0; background: var(--bg); overflow: hidden; font-family: monospace; color: var(--prim); }
  canvas { background: radial-gradient(circle, #001111 0%, #000000 80%); border-radius: 50%; }
  .hud { position: absolute; top: 10px; left: 10px; pointer-events: none; }
  .controls { position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%); display: flex; gap: 10px; pointer-events: auto; background: rgba(0,20,20,0.8); padding: 10px; border-radius: 5px; }
</style>
</head>
<body>
<div class="hud">
  <h1>RADAR SCOPE</h1>
  <div>ANGLE: <span id="angle">0</span></div>
</div>
<div class="controls">
  <span>SPEED</span>
  <input type="range" id="speed" min="0" max="100" value="50">
</div>
<canvas id="radar"></canvas>
<script>
  const cvs = document.getElementById('radar');
  const ctx = cvs.getContext('2d');
  let ws;
  let cx, cy, r;
  // PERSISTENT MAP: 360 degrees
  const mapData = new Array(360).fill(0);

  function resize() {
    cvs.width = window.innerWidth;
    cvs.height = window.innerHeight;
    cx = cvs.width/2; cy = cvs.height/2;
    r = Math.min(cx,cy)-20;
  }
  window.onresize = resize;
  resize();

  function connect() {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onmessage = (e) => {
      const d = JSON.parse(e.data);
      document.getElementById('angle').innerText = d.angle;
      
      // Update 4 sectors
      updateMap(d.angle, d.d_front);
      updateMap(d.angle+90, d.d_left);
      updateMap(d.angle+180, d.d_back);
      updateMap(d.angle+270, d.d_right);
    };
  }
  connect();

  document.getElementById('speed').oninput = function() {
    ws.send('S' + Math.floor(50 - (this.value * 0.45)));
  };

  function updateMap(deg, dist) {
    // Normalize angle to 0-359
    let a = deg % 360;
    if (a < 0) a += 360;
    
    // Filter invalid data
    if(dist <= 0 || dist > 400) {
      // If "out of range", we can either clear it or leave it.
      // User wants "overwrite if object not there".
      // So if we scan and get > 400, we clear the map at this angle.
      mapData[a] = 0; 
    } else {
      mapData[a] = dist;
    }
  }

  function draw() {
    ctx.fillStyle = 'rgba(0,0,0,0.1)'; 
    ctx.fillRect(0,0,cvs.width,cvs.height);
    
    // Grid
    ctx.strokeStyle = '#003333'; ctx.lineWidth=1; ctx.beginPath();
    for(let i=1; i<=4; i++) ctx.arc(cx, cy, r*(i/4), 0, Math.PI*2);
    ctx.stroke();

    // Draw Map
    ctx.strokeStyle = '#00FFFF';
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    let first = true;
    for(let i=0; i<360; i++) {
      if(mapData[i] === 0) continue;
      
      const rad = (i-90)*(Math.PI/180);
      const distPx = (mapData[i]/400)*r;
      const x = cx + Math.cos(rad)*distPx;
      const y = cy + Math.sin(rad)*distPx;
      
      // Draw Point
      ctx.fillStyle = '#00FFFF';
      ctx.fillRect(x-1, y-1, 3, 3);
    }
    
    // Scanner Line
    const currentA = parseInt(document.getElementById('angle').innerText);
    const scanRad = (currentA-90)*(Math.PI/180);
    ctx.strokeStyle = 'rgba(0,255,0,0.5)';
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(scanRad)*r, cy + Math.sin(scanRad)*r);
    ctx.stroke();

    requestAnimationFrame(draw);
  }
  draw();
</script>
</body>
</html>
)=====";

// ===================================================================================
// WEBSOCKET HANDLER
// ===================================================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    if (payload[0] == 'S') {
      int val = atoi((const char*)&payload[1]);
      if (val < 5) val = 5;
      if (val > 100) val = 100;
      servoDelay = val;
    }
  }
}

// ===================================================================================
// SETUP
// ===================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n--- TRON ULTRASONIC RADAR (SYSTEM A) STARTING ---");
  
  pinMode(PIN_TRIG_A, OUTPUT);
  pinMode(PIN_TRIG_B, OUTPUT);
  pinMode(PIN_ECHO_F, INPUT);
  pinMode(PIN_ECHO_B, INPUT);
  pinMode(PIN_ECHO_L, INPUT);
  pinMode(PIN_ECHO_R, INPUT);
  
  digitalWrite(PIN_TRIG_A, LOW);
  digitalWrite(PIN_TRIG_B, LOW);

  scanner.attach(PIN_SERVO);
  scanner.write(currentAngle);

  // 3. Init WiFi (Station Mode with Static IP)
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // KEEPING THIS FIX: Prevents Jitter!
  
  // Config Static IP BEFORE begin
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() { server.send_P(200, "text/html", index_html); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// ===================================================================================
// MEASUREMENT HELPER
// ===================================================================================
long measure(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);
  if (duration == 0) return 0; 
  return duration * 0.034 / 2;
}

// ===================================================================================
// LOOP
// ===================================================================================
void loop() {
  webSocket.loop();
  server.handleClient();

  // Move Servo
  scanner.write(currentAngle);
  delay(servoDelay); // Speed control restored!
  
  // Interleaved Measurements (Ping-Pong)
  // Using the time of one measurement as cooldown for the other trigger group.
  
  // 1. Front (Group A)
  long d_front = measure(PIN_TRIG_A, PIN_ECHO_F);
  delay(2); 

  // 2. Left (Group B) - Acts as cooldown for Group A
  long d_left = measure(PIN_TRIG_B, PIN_ECHO_L);
  delay(2);

  // 3. Back (Group A) - Group A has rested while we measured Left
  long d_back = measure(PIN_TRIG_A, PIN_ECHO_B);
  delay(2);

  // 4. Right (Group B) - Group B has rested while we measured Back
  long d_right = measure(PIN_TRIG_B, PIN_ECHO_R);
  delay(2);
  
  // Broadcast
  String json = "{";
  json += "\"angle\":" + String(currentAngle) + ",";
  json += "\"d_front\":" + String(d_front) + ",";
  json += "\"d_back\":" + String(d_back) + ",";
  json += "\"d_left\":" + String(d_left) + ",";
  json += "\"d_right\":" + String(d_right);
  json += "}";
  webSocket.broadcastTXT(json);

  // Update Angle
  currentAngle += angleStep;
  if (currentAngle >= SERVO_MAX || currentAngle <= SERVO_MIN) {
    angleStep = -angleStep;
  }
  
  yield();
}
