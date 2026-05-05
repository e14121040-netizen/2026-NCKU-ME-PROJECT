#include <WiFi.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  Serial.print("ESP32-C3 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.print("ESP32-C3 MAC Address: ");
    Serial.println(WiFi.macAddress());
    delay(2000);
}
