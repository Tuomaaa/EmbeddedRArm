#include <WiFi.h>
#include <WiFiUdp.h>

#define SLIDE 1
#define POT 0

WiFiUDP udp;
const int udpPort = 4210;

const int potPin = 34;
int currentAngle = 90;
int potAngle = 90;
int mode = SLIDE;

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
  pinMode(potPin, INPUT);

  WiFi.softAP("RobotArm", "12345678");
  Serial.println("AP mode started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(udpPort);
  Serial.println("UDP listening on port 4210");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize >= 2) {
    uint8_t buffer[8];
    int len = udp.read(buffer, 8);

    uint8_t cmd = buffer[0];
    uint8_t value = buffer[1];

    if (cmd == 0x00 && value <= 180) {
      // Angle command
      currentAngle = value;
      Serial2.write(value);
      Serial.print("UDP angle: ");
      Serial.println(value);
    } else if (cmd == 0x01) {
      // Mode switch
      mode = value;
      Serial.print("Mode set to: ");
      Serial.println(mode == POT ? "POT" : "SLIDE");
    }
  }

  static unsigned long lastRead = 0;
  if (millis() - lastRead > 100) {
    readPotentiometer();
    lastRead = millis();
  }
}

void readPotentiometer() {
  int adcValue = analogRead(potPin);
  potAngle = map(adcValue, 0, 4095, 0, 180);
  if (mode == POT) {
    Serial2.write(potAngle);
    Serial.print("Pot angle: ");
    Serial.println(potAngle);
  }
}