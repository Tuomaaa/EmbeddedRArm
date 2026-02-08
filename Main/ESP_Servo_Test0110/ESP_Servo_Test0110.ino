#include <WiFi.h>
#include <WebServer.h>

#define SLIDE 1
#define POT 0
// WiFi credentials
const char* ssid = "IfiW 881";
const char* password = "88888888";

// Create web server on port 80
WebServer server(80);

const int potPin = 34;  // GPIO34 (ADC1_CH6) for potentiometer
int currentAngle = 90;
int potAngle = 90;      // Angle read from potentiometer
int mode=SLIDE;
const int servoPin = 13;  // Use GPIO 13 for servo signal

// Handle root path
void handleRoot() {
  String html = "<html><body>";
  html += "<h1>ESP32 Servo Control</h1>";
  html += "<p>Potentiometer Angle: " + String(potAngle) + " degrees</p>";
  html += "<p>Current Angle: " + String(currentAngle) + " degrees</p>";
  html += "<p>Get POT value: http://192.168.1.53/pot</p>";
  html += "<p>Send command: http://192.168.1.53/servo?angle=90</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Handle servo control
void handleServo() {
  if (server.hasArg("angle")) {
    String angleStr = server.arg("angle");
    int angle = angleStr.toInt();
    
    if (angle >= 0 && angle <= 180) {
      currentAngle = angle;
      Serial.print("Received angle: ");
      Serial.println(angle);
      
      setServoAngle(angle);

      
      server.send(200, "text/plain", "OK: Angle set to " + String(angle));
    } else {
      server.send(400, "text/plain", "ERROR: Angle must be 0-180");
    }
  } else {
    server.send(400, "text/plain", "ERROR: Missing angle parameter");
  }
}

// ========== 新增：POT 读取端点 ==========
void handlePot() {
  // Return current potentiometer angle as JSON
  String json = "{\"angle\":" + String(potAngle) + "}";
  server.send(200, "application/json", json);
}

void handleMode(){
  if (server.hasArg("mode")){
    String modeStr = server.arg("mode");
    mode = modeStr.toInt();
    server.send(200, "text/plain", "OK: Mode set to " + String(mode));
  }
}

// Handle 404
void handleNotFound() {
  server.send(404, "text/plain", "404 Not Found");
}
// ========== 新增：读取电位器函数 ==========
void readPotentiometer() {
  // Read ADC value (0-4095 for ESP32 12-bit ADC)
  int adcValue = analogRead(potPin);
  
  // Convert to angle (0-180 degrees)
  potAngle = map(adcValue, 0, 4095, 0, 180);
  
  if(mode==POT){
      Serial.print("Pot angle: ");
      Serial.println(potAngle);
      
      setServoAngle(potAngle);

  }
  // Optional: Print for debugging
  // Serial.print("ADC: ");
  // Serial.print(adcValue);
  // Serial.print(" -> Angle: ");
  // Serial.println(potAngle);
}
// ==========================================

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  Serial.begin(115200);
  delay(1000);
  ledcAttach(servoPin, 50, 16);
  Serial.println("Starting ESP32 HTTP Server");
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  pinMode(potPin, INPUT);
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Setup routes
  server.on("/", handleRoot);
  server.on("/servo", handleServo);
  server.on("/pot", handlePot);
  server.on("/mode", handleMode);
  server.onNotFound(handleNotFound);
  
  // Start server
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Ready to receive commands!");
}

void loop() {
  server.handleClient();
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 100) {  // Read every 100ms
    readPotentiometer();
    lastRead = millis();
  }

}

void setServoAngle(int angle) {
  // Convert angle to duty cycle
  // 0 deg = 2.5% duty = 1638 (at 16-bit resolution)
  // 180 deg = 12.5% duty = 8192
  //int dutyCycle = map(angle, 0, 180, 1638, 8192);
  //ledcWrite(servoPin, dutyCycle);
  Serial2.write(angle);
  Serial.println(angle);
}
