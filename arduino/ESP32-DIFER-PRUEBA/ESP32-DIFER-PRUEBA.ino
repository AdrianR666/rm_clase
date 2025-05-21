
#define ENA 5
#define IN1 18
#define IN2 19
#define ENB 25
#define IN3 33
#define IN4 32

void setup() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Serial.begin(115200);
    Serial.println("Inicializando motores...");
}

void loop() {
    // Motor 1 hacia adelante
    Serial.println("Motor 1: Adelante");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255); // Velocidad máxima
    delay(2000);

    // Motor 1 hacia atrás
    Serial.println("Motor 1: Atras");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    delay(2000);

    // Detener Motor 1
    Serial.println("Motor 1: Detenido");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    delay(1000);

    // Motor 2 hacia adelante
    Serial.println("Motor 2: Adelante");
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255); // Velocidad máxima
    delay(2000);

    // Motor 2 hacia atrás
    Serial.println("Motor 2: Atras");
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(2000);

    // Detener Motor 2
    Serial.println("Motor 2: Detenido");
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(1000);
}
