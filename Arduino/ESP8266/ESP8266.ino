
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>

  const char* ssid = "Ann";            // Ваш SSID
  const char* password = "Asdf2086";      // Ваш пароль
  const char* udpServerIP = "192.168.1.72"; // IP получателя
  const int udpServerPort = 8888;            // Порт получателя

  WiFiUDP udp;
  String deviceName = "ESP8266";  // Имя устройства, которое будет добавлено к отправляемому сообщению

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  udp.begin(2000); // Инициализация UDP для приема данных
  while (WiFi.status() != WL_CONNECTED) {
  Serial.println(".......");
  delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    // Передача данных по UDP
    udp.beginPacket(udpServerIP, udpServerPort);
    //udp.printf("[%s] Привет, это сообщение от %s!", deviceName.c_str(), deviceName.c_str());
    udp.endPacket();

    // Прием данных по UDP
    int packetSize = udp.parsePacket();
    if (packetSize) {
      char incomingPacket[255];
      int len = udp.read(incomingPacket, 255);
      if (len > 0) {
        incomingPacket[len] = 0;
        udp.printf("Получено сообщение: %s\n", incomingPacket);
        Serial.println(incomingPacket);
      }
    }
  }
  delay(100);
}
