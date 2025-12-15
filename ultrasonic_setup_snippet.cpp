/*
 * SYSTEM A: ULTRASONIC RADAR SETUP SNIPPET
 * Copy this entire setup() function into your Ultrasonic Sketch.
 * 
 * Hardware: ESP-12F, 4x HC-SR04, Servo
 * Target IP: 192.168.178.40
 */

const char* ssid     = "FRITZ!Box 6660 Cable UE";
const char* password = "28370714691864306613";

// STATIC IP CONFIGURATION (System A: Ultrasonic)
// Update "gateway" if your router is not 192.168.178.1
IPAddress local_IP(192, 168, 178, 40);
IPAddress gateway(192, 168, 178, 1);   // <--- CHECK THIS (Router IP)
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 178, 1);

void setup() {
  // 1. Init Serial
  Serial.begin(115200);
  Serial.println("\n\n--- TRON ULTRASONIC RADAR STARTING ---");

  // 2. Init Pins (Keep your existing Sensor/Servo pin inits here!)
  // Example:
  // pinMode(TRIG_PIN, OUTPUT);
  // pinMode(ECHO_PIN, INPUT);
  // scanner.attach(SERVO_PIN);

  // 3. Init WiFi (Station Mode with Static IP)
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
  }
  
  Serial.println("");
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  // 4. Init Servers (Keep your existing Server/WebSocket inits here!)
  // server.begin();
  // webSocket.begin();
}
