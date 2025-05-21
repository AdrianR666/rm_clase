#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "MiRedWifi";
const char* password = "123456789";
const int localUdpPort = 4210;  // Puerto local para recibir mensajes UDP
char incomingPacket[255];       // Buffer para datos entrantes

WiFiUDP udp;

const int ledPin = 12;  // Cambia por el pin donde tengas conectado tu LED

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi. Dirección IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localUdpPort);
  Serial.printf("Escuchando UDP en el puerto %d...\n", localUdpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    Serial.printf("Mensaje recibido: %s\n", incomingPacket);

    if (strcmp(incomingPacket, "ON") == 0) {
      digitalWrite(ledPin, HIGH);
    } else if (strcmp(incomingPacket, "OFF") == 0) {
      digitalWrite(ledPin, LOW);
    }
  }
}
