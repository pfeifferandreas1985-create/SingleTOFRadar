/*
 * NERO SENSE - SYSTEM B (FINAL FUSION v6.1 - REPAIRED)
 * ----------------------------------------------------
 * Role: High-Performance Lidar Radar Unit
 * Features: Robust Sensor Logic + Multi-Mode UI + Status Ring
 * Hardware: Wemos D1 Mini (ESP8266)
 *
 * Copyright (c) 2025 Andreas Pfeiffer
 * Seestraße 20, 78234 Engen, Deutschland
 * pfeiffer.andreas1985@gmail.com
 * All Rights Reserved.
 *
 * Display: GC9A01 (240x240 Round) via Hardware SPI
 * Sensor: VL53L1X (TOF) via I2C
 * Servo: SG90 (PWM)
 * Input: TTP223 Touch on D4
 *
 * --- USER_SETUP.h CONFIGURATION (TFT_eSPI) ---
 * #define GC9A01_DRIVER
 * #define TFT_WIDTH  240
 * #define TFT_HEIGHT 240
 * #define TFT_MOSI 13 // D7
 * #define TFT_SCLK 14 // D5
 * #define TFT_CS   15 // D8
 * #define TFT_DC    0 // D3
 * #define TFT_RST  12 // D6 (Changed from D0 to free D0 for Touch)
 * #define LOAD_GLCD
 * #define LOAD_FONT2
 * #define LOAD_FONT4
 * #define LOAD_FONT6
 * #define SMOOTH_FONT
 * #define SPI_FREQUENCY  27000000
 */

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>

// --- HARDWARE PINS (FINAL FIX) ---
#define PIN_SERVO 2  // D4 (Servo)
#define PIN_SDA    4 // D2
#define PIN_SCL    5 // D1
#define PIN_TOUCH  16 // D0 (Touch - Safe for Input, D6 was blocked by SPI)

// --- NETWORK ---
const char* ssid = "FRITZ!Box 6660 Cable UE";
const char* password = "28370714691864306613";
IPAddress local_IP(192, 168, 178, 71); 
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);

// --- DESIGN PALETTE ---
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GRID    0x18E3 // Dim Grey for Grid

// Mode 1: Precision (Orange)
#define C_M1_PRI  0xFDA0
#define C_M1_SEC  0x7800
// Mode 2: Standard (Green)
#define C_M2_PRI  0x07E0
#define C_M2_SEC  0x0320
// Mode 3: Long Range (Cyan)
#define C_M3_PRI  0x07FF
#define C_M3_SEC  0x0010

// Status Ring Colors
#define S_BLUE    0x001F
#define S_GREEN   0x07E0
#define S_RED     0xF800
#define S_WHITE   0xFFFF

// --- OBJECTS ---
TFT_eSPI tft = TFT_eSPI();
VL53L1X sensor;
Servo myServo;
WebSocketsServer webSocket = WebSocketsServer(81);

// --- GLOBAL STATE ---
// Mode Management
enum RadarMode { MODE_PRECISION, MODE_STANDARD, MODE_LONGRANGE };
RadarMode currentMode = MODE_STANDARD;
bool modeChanged = true;

struct ModeSettings {
  uint16_t colorPri;
  uint16_t colorSec;
  uint16_t maxDist;
  uint32_t timingBudget;
  VL53L1X::DistanceMode distMode;
  String label;
};

ModeSettings modes[3] = {
  {C_M1_PRI, C_M1_SEC, 300,  20000,  VL53L1X::Short, "PRECISION"},
  {C_M2_PRI, C_M2_SEC, 1000, 33000,  VL53L1X::Short, "STANDARD"},
  {C_M3_PRI, C_M3_SEC, 4000, 100000, VL53L1X::Long,  "LONG RANGE"}
};

// Servo & Sensor
int currentAngle = 90;
int direction = 1;
unsigned long lastServoMove = 0;
int servoDelay = 15;
uint16_t distHistory[181]; // Memory

// Touch Input
int lastTouchState = LOW;
unsigned long lastDebounceTime = 0;

// UI Geometry
const int CX = 120;
const int CY = 120;
const int R_RADAR = 114; // Reduced to make room for thicker ring
const int R_RING = 115;  // Start of Status Ring (115-119)

// Connectivity State
bool isOnline = false;

// --- PROTOTYPES ---
void playIntro();
void drawRadar(int angle, int dist);
void updateMode();
void resetSensor();
void drawStatusRing(uint16_t color);
void handleTouch();
void drawInterface();

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TOUCH, INPUT);
  
  // 1. INIT DISPLAY
  tft.init();
  tft.setRotation(0); 
  tft.fillScreen(C_BLACK);
  
  // 2. CINEMATIC BOOT
  playIntro();
  
  // 3. ROBUST SENSOR INIT
  resetSensor();
  
  // 4. SERVO
  myServo.attach(PIN_SERVO, 500, 2500);
  myServo.write(90);
  
  // 5. NETWORK
  tft.fillScreen(C_BLACK);
  drawStatusRing(S_BLUE); // Indicate connecting
  drawInterface(); // Draw UI
  
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  
  // Try connecting for 5 seconds, non-blocking UI
  unsigned long startAttempt = millis();
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.drawString("CONNECTING...", CX, CY, 4);
  
  while (millis() - startAttempt < 5000) {
    if (WiFi.status() == WL_CONNECTED) {
      isOnline = true;
      webSocket.begin();
      break;
    }
    delay(10);
  }
  
  // Clear Text
  tft.fillRect(0, 100, 240, 40, C_BLACK);
  
  // Initial Mode Setup
  updateMode();
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  if (isOnline) webSocket.loop();
  
  // Handle Status Ring
  if (isOnline) drawStatusRing(S_BLUE);
  else drawStatusRing(S_GREEN); // Green = Standalone OK
  
  // Handle Touch Input
  handleTouch();
  
  // Mode Change Logic
  if (modeChanged) {
    updateMode();
    modeChanged = false;
  }
  
  unsigned long now = millis();
  
  // 1. SERVO & SENSOR LOOP
  if (now - lastServoMove >= servoDelay) {
    lastServoMove = now;
    
    // Move Servo
    myServo.write(currentAngle);
    
    // WATCHDOG: Check if sensor is frozen (No data for > 2000ms)
    static unsigned long lastSensorUpdate = 0;
    if (now - lastSensorUpdate > 2000) {
       // SENSOR FREEZE DETECTED!
       tft.fillRect(60, 160, 120, 60, C_BLACK);
       tft.setTextColor(S_RED, C_BLACK);
       tft.drawString("RESETTING...", 120, 180, 2);
       drawStatusRing(S_RED);
       
       // Force Full Reset
       resetSensor();
       
       lastSensorUpdate = now; // Reset timer
       updateMode(); // Restore mode settings
       return; // Skip this loop
    }
    
    // Read Sensor (Non-Blocking Check)
    if (sensor.dataReady()) {
      lastSensorUpdate = now; // Pet the watchdog
      
      sensor.read(false);
      uint16_t dist = sensor.ranging_data.range_mm;
      
      // Handle Invalid Readings
      if (sensor.ranging_data.range_status != VL53L1X::RangeValid) {
        dist = modes[currentMode].maxDist; // Default to max (clear)
      }
      
      // Update History
      distHistory[currentAngle] = dist;
      
      // Draw Radar
      drawRadar(currentAngle, dist);
      
      // Broadcast
      if (isOnline) {
        String json = "{\"angle\":" + String(currentAngle) + 
                      ",\"distance\":" + String(dist) + 
                      ",\"mode\":" + String(currentMode) + "}";
        webSocket.broadcastTXT(json);
      }
    }
    
    // Advance Angle
    currentAngle += direction;
    if (currentAngle >= 180 || currentAngle <= 0) {
      direction *= -1;
    }
  }
}

// =========================================================================
// VISUALIZATION ENGINE
// =========================================================================
int lastScannerAngle = 0;

void drawRadar(int angle, int dist) {
  ModeSettings cfg = modes[currentMode];
  
  // 1. SECTOR CLEARING (The "Wiper")
  // Clear wedge between current and next angle
  int nextAngle = angle + direction; // Step is 1
  if (nextAngle < 0) nextAngle = 0;
  if (nextAngle > 180) nextAngle = 180;
  
  if (nextAngle != angle) {
      float rad1 = angle * DEG_TO_RAD;
      float rad2 = nextAngle * DEG_TO_RAD;
      
      int x1 = CX + cos(rad1) * R_RADAR;
      int y1 = CY - sin(rad1) * R_RADAR;
      int x2 = CX + cos(rad2) * R_RADAR;
      int y2 = CY - sin(rad2) * R_RADAR;
      
      // Wipe the sector ahead black
      tft.fillTriangle(CX, CY, x1, y1, x2, y2, C_BLACK);
  }
  
  // 2. RESTORE MAP BEHIND SCANNER
  if (lastScannerAngle != angle) {
      float lastRad = lastScannerAngle * DEG_TO_RAD;
      int lastEdgeX = CX + cos(lastRad) * R_RADAR;
      int lastEdgeY = CY - sin(lastRad) * R_RADAR;
      
      // A. Erase the Red Line
      tft.drawLine(CX, CY, lastEdgeX, lastEdgeY, C_BLACK);
      
      // B. Restore Grid
      for (int r = 40; r < R_RADAR; r += 40) {
        int gx = CX + cos(lastRad) * r;
        int gy = CY - sin(lastRad) * r;
        tft.drawPixel(gx, gy, C_GRID);
      }
      
      // C. Restore Map Data
      int storedDist = distHistory[lastScannerAngle];
      if (storedDist > 0 && storedDist < cfg.maxDist) {
          float scale = (float)R_RADAR / cfg.maxDist;
          int dPx = storedDist * scale;
          int ox = CX + cos(lastRad) * dPx;
          int oy = CY - sin(lastRad) * dPx;
          
          // Shadow Zone (Secondary Color)
          tft.drawLine(ox, oy, lastEdgeX, lastEdgeY, cfg.colorSec);
          
          // Contour Line (Primary Color)
          int neighborIdx = lastScannerAngle - direction;
          bool connected = false;
          
          if (neighborIdx >= 0 && neighborIdx <= 180) {
             int prevDist = distHistory[neighborIdx];
             if (prevDist > 0 && prevDist < cfg.maxDist && abs(prevDist - storedDist) < (cfg.maxDist/10)) {
                 float prevRad = neighborIdx * DEG_TO_RAD;
                 int pdPx = prevDist * scale;
                 int pox = CX + cos(prevRad) * pdPx;
                 int poy = CY - sin(prevRad) * pdPx;
                 
                 tft.drawLine(pox, poy, ox, oy, cfg.colorPri);
                 connected = true;
             }
          }
          
          if (!connected) {
             tft.drawPixel(ox, oy, cfg.colorPri);
          }
      }
  }
  
  // 3. DRAW NEW SCANNER LINE
  float rad = angle * DEG_TO_RAD;
  int edgeX = CX + cos(rad) * R_RADAR;
  int edgeY = CY - sin(rad) * R_RADAR;
  
  tft.drawLine(CX, CY, edgeX, edgeY, C_RED);
  
  lastScannerAngle = angle;
  
  // 4. UPDATE DASHBOARD (Every 5 degrees)
  if (angle % 5 == 0) {
    // Clear Text Area
    tft.fillRect(60, 155, 120, 60, C_BLACK);
    
    tft.setTextDatum(MC_DATUM); 
    tft.setTextColor(C_WHITE, C_BLACK);
    
    String distStr = (dist >= cfg.maxDist) ? "> " + String(cfg.maxDist) : String(dist) + " mm";
    tft.drawString(distStr, 120, 165, 4); // Moved Up
    
    tft.setTextColor(cfg.colorPri, C_BLACK);
    String angleStr = "ANG: " + String(angle);
    tft.drawString(angleStr, 120, 190, 2); // Moved Up
  }
}

// =========================================================================
// HELPER FUNCTIONS
// =========================================================================

void handleTouch() {
  int reading = digitalRead(PIN_TOUCH);
  if (reading != lastTouchState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > 50) {
    static int touchState = LOW;
    if (reading != touchState) {
      touchState = reading;
      if (touchState == HIGH) {
        // Touch Detected!
        currentMode = (RadarMode)((currentMode + 1) % 3);
        modeChanged = true;
        drawStatusRing(S_WHITE); // Flash White
        delay(100); 
      }
    }
  }
  lastTouchState = reading;
}

void updateMode() {
  ModeSettings cfg = modes[currentMode];
  
  // Re-Config Sensor
  sensor.stopContinuous();
  delay(10);
  sensor.setDistanceMode(cfg.distMode);
  sensor.setMeasurementTimingBudget(cfg.timingBudget);
  sensor.startContinuous(cfg.timingBudget / 1000 + 5);
  
  // Update Speed
  servoDelay = (cfg.timingBudget / 1000); 
  if (servoDelay < 15) servoDelay = 15;
  
  // Clear Radar Area (Keep Status Ring)
  tft.fillCircle(CX, CY, R_RADAR, C_BLACK);
  
  // Redraw Static Grid
  drawInterface();
  
  // Clear History
  for(int i=0; i<=180; i++) distHistory[i] = 0;
  
  // Draw Labels
  tft.setTextDatum(MC_DATUM);
  tft.fillRect(0, 150, 240, 90, C_BLACK);
  
  tft.setTextColor(C_GRID, C_BLACK);
  tft.drawString("MODE", CX, 206, 2); // Moved Up 4px (was 210)
  
  tft.setTextColor(cfg.colorPri, C_BLACK);
  tft.drawString(cfg.label, CX, 221, 2); // Moved Up 4px (was 225)
}

void drawInterface() {
  tft.drawCircle(CX, CY, 40, C_GRID);
  tft.drawCircle(CX, CY, 80, C_GRID);
  tft.drawCircle(CX, CY, R_RADAR, C_GRID); // Outer Rim
  
  tft.drawLine(CX - R_RADAR, CY, CX + R_RADAR, CY, C_GRID);
  tft.drawLine(CX, CY, CX, CY - R_RADAR, C_GRID);
}

void drawStatusRing(uint16_t color) {
  // Thicker Ring (5 pixels)
  tft.drawCircle(CX, CY, 119, color);
  tft.drawCircle(CX, CY, 118, color);
  tft.drawCircle(CX, CY, 117, color);
  tft.drawCircle(CX, CY, 116, color);
  tft.drawCircle(CX, CY, 115, color);
}

void resetSensor() {
  // 1. Force I2C Bus Clear
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT);
  
  for(int i=0; i<16; i++) { 
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(10);
  }
  pinMode(PIN_SCL, INPUT_PULLUP);
  
  // 2. Re-Init Wire
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000); 
  Wire.setClockStretchLimit(1000);
  
  // 3. Init Sensor with Retry
  bool sensorFound = false;
  tft.setTextDatum(MC_DATUM);
  
  for(int attempt=1; attempt<=5; attempt++) {
     tft.fillRect(0, 170, 240, 40, C_BLACK);
     tft.setTextColor(C_WHITE, C_BLACK);
     tft.drawString("Init Sensor: " + String(attempt) + "/5", 120, 180);
     
     sensor.setTimeout(500);
     if (sensor.init()) {
       sensorFound = true;
       tft.setTextColor(S_GREEN, C_BLACK);
       tft.drawString("Sensor OK!", 120, 180);
       delay(500);
       break;
     }
     delay(500);
  }
  
  if (!sensorFound) {
    tft.setTextColor(S_RED, C_BLACK);
    tft.drawString("SENSOR ERROR!", 120, 180);
    drawStatusRing(S_RED);
    delay(2000); 
  } else {
    // Default start
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(33000);
    sensor.startContinuous(33);
  }
}

// --- CINEMATIC BOOT ---
void playIntro() {
  tft.fillScreen(C_BLACK);
  tft.drawPixel(CX, CY, C_WHITE);
  delay(500);
  
  // Pulse
  for (int r = 0; r <= 15; r++) {
    tft.fillCircle(CX, CY, r, C_WHITE); delay(10);
  }
  for (int r = 15; r >= 5; r--) {
    tft.fillCircle(CX, CY, r, C_BLACK); 
    tft.drawCircle(CX, CY, r, C_WHITE); delay(10);
  }
  
  // Ignition
  for (int r = 0; r < 140; r+=4) {
    tft.drawCircle(CX, CY, r, C_M3_PRI);
    tft.drawCircle(CX, CY, r-1, C_M3_PRI); delay(5);
  }
  tft.fillScreen(C_BLACK);
  
  // Identity
  tft.setTextDatum(MC_DATUM);
  
  // Glitch
  for(int i=0; i<3; i++) {
    int ox = random(-2, 3);
    int oy = random(-2, 3);
    tft.setTextColor(C_WHITE, C_BLACK);
    tft.drawString("NERO", CX+ox, CY-20+oy, 4);
    tft.setTextColor(C_M3_PRI, C_BLACK);
    tft.drawString("ROBOTICS", CX-ox, CY+10-oy, 2);
    delay(50);
    tft.fillScreen(C_BLACK);
  }
  
  // Final Text
  tft.setTextColor(C_WHITE, C_BLACK);
  tft.drawString("NERO", CX, CY-20, 4);
  tft.setTextColor(C_M3_PRI, C_BLACK);
  tft.drawString("ROBOTICS", CX, CY+10, 2);
  
  // Lines
  tft.drawLine(60, CY-40, 180, CY-40, C_GRID);
  tft.drawLine(60, CY+30, 180, CY+30, C_GRID);
  
  delay(1000);
  
  // Boot Bar
  tft.drawRect(70, 200, 100, 4, C_GRID);
  for(int i=0; i<100; i+=2) {
    tft.fillRect(72, 202, i, 2, S_GREEN); delay(10);
  }
  
  // Flash
  tft.fillScreen(C_WHITE); delay(50);
  tft.fillScreen(C_BLACK);
}
