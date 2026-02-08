void setup() {
    Serial.begin(9600);    // USB调试
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    delay(1000);
    Serial.println("UART test started");
}

void loop() {
    Serial2.write(90);   // 直接发一个角度值
    Serial.println("Sent: 90");
    delay(2000);

    Serial2.write(0);
    Serial.println("Sent: 0");
    delay(2000);
}


void sendToFPGA(uint8_t motor_id, uint8_t angle) {
    Serial2.write(motor_id);
    delay(10);
    Serial2.write(angle);
}