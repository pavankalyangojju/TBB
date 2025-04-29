#include <WiFi.h>

void setup() {
  Serial.begin(115200);

  // Set ESP32 to Station mode
  WiFi.mode(WIFI_STA);
}

void loop() {
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  delay(2000); // 2 second delay
}
