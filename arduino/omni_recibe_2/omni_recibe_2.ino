#include <WiFi.h>
#include <WiFiUdp.h>

// Configuración WiFi - cambia por tu red
const char* ssid = "MiRedWifi";
const char* password = "123456789";

// UDP
WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[255];

// Pines según tu configuración
#define ENA1  12
#define IN1   14
#define IN2   27

#define ENB1  33
#define IN3   26
#define IN4   25

#define ENA2  4
#define IN5   16
#define IN6   17

#define ENB2  19
#define IN7   5
#define IN8   18

// Canales PWM para cada motor (4 canales diferentes)
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;  // 8 bits -> 0-255

const int CANAL_PWM_1 = 0;  // Motor 1 (ENA1)
const int CANAL_PWM_2 = 1;  // Motor 2 (ENB1)
const int CANAL_PWM_3 = 2;  // Motor 3 (ENA2)
const int CANAL_PWM_4 = 3;  // Motor 4 (ENB2)

const int PWM_MAX = 255;
const float maxSpeed = 100.0; // Velocidad máxima esperada

void setup() {
  Serial.begin(115200);

  // Pines dirección como salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // Configura PWM canales y pines
  ledcSetup(CANAL_PWM_1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA1, CANAL_PWM_1);

  ledcSetup(CANAL_PWM_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENB1, CANAL_PWM_2);

  ledcSetup(CANAL_PWM_3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA2, CANAL_PWM_3);

  ledcSetup(CANAL_PWM_4, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENB2, CANAL_PWM_4);

  // Conectar WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi conectada");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Iniciar UDP
  Udp.begin(localUdpPort);
  Serial.print("Esperando paquetes UDP en puerto ");
  Serial.println(localUdpPort);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize == 16) {  // Esperamos 16 bytes (4 floats de 4 bytes)
    int len = Udp.read(incomingPacket, 16);
    if (len == 16) {
      float *vels = (float *)incomingPacket;
      moverMotor(vels[0], IN1, IN2, CANAL_PWM_1);
      moverMotor(vels[1], IN3, IN4, CANAL_PWM_2);
      moverMotor(vels[2], IN5, IN6, CANAL_PWM_3);
      moverMotor(vels[3], IN7, IN8, CANAL_PWM_4);

      Serial.printf("Vel UDP recibidas: %.2f %.2f %.2f %.2f\n", vels[0], vels[1], vels[2], vels[3]);
    }
  }
}
// Función para mover motor con dirección y PWM
void moverMotor(float vel, int in1, int in2, int pwmChannel) {
  int pwm = min((int)(abs(vel) / maxSpeed * PWM_MAX), PWM_MAX);

  if (vel > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (vel < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(pwmChannel, pwm);
}
