
#include <WiFi.h>
#include <WiFiUdp.h>

// Configuración de red WiFi
const char* ssid = "MiRedWifi";         // ← Cambia esto
const char* password = "123456789"; // ← Cambia esto

// Configuración de UDP
WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[255];  // Buffer para almacenar los paquetes UDP
String data = "";          // Variable para almacenar los datos recibidos

// Pines de los motores (ajústalos si usas un driver distinto)
const int IN1 = 15;
const int IN2 = 2;
const int IN3 = 13;
const int IN4 = 12;

const int PWM_MAX = 255;   // Valor máximo de PWM
const float maxSpeed = 100.0; // Velocidad máxima esperada desde UDP

void setup() {
  Serial.begin(115200);

  // Configura pines de motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  // Conecta a Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Conectado a WiFi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Inicia UDP
  Udp.begin(localUdpPort);
  Serial.print("Esperando mensajes en el puerto UDP: ");
  Serial.println(localUdpPort);
}

void loop() {
  // Revisa si hay datos entrantes en UDP
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // Lee el paquete UDP
    int len = Udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0; // Asegura que la cadena se termine
    }

    // Muestra el paquete recibido
    Serial.print("Mensaje UDP recibido: ");
    Serial.println(incomingPacket);

    // Procesa los datos (se espera un formato tipo "v1,v2")
    int sepIndex = String(incomingPacket).indexOf(',');
    if (sepIndex > 0) {
      float vel1 = String(incomingPacket).substring(0, sepIndex).toFloat();
      float vel2 = String(incomingPacket).substring(sepIndex + 1).toFloat();
      Serial.println("PWM utilizado: " + moverMotor(vel1, IN3, IN4) + ',' + moverMotor(vel2, IN1, IN2));
    }
  }
}

// Función para mover los motores según las velocidades recibidas
String moverMotor(float vel, int in1, int in2) {
  int pwm = min((int)(abs(vel) / maxSpeed * PWM_MAX), PWM_MAX);

  if (vel > 0) {
    analogWrite(in1, pwm);
    analogWrite(in2, 0);
  } else if (vel < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, pwm);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
  return String(pwm);
}
