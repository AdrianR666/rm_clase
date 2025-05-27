//Motor derecho
#define IN1 15
#define IN2 2

//Motor izquierdo
#define IN3 13
#define IN4 12



void setup() {
    //Definición de pines
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    //Inicializar comunicación
    Serial.begin(115200);
    Serial.println("Inicializando motores...");
}

void loop() {

    // Motor Der hacia adelante
    Serial.println("Motor 1: Adelante");
    analogWrite(IN1, 255);
    analogWrite(IN2, 0);
    delay(2000);

    // Motor Der hacia atrás
    Serial.println("Motor 1: Atras");
    analogWrite(IN1, 0);
    analogWrite(IN2, 255);
    delay(2000);

    // Detener Motor Der
    Serial.println("Motor 1: Detenido");
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    delay(1000);

    // Motor izq hacia adelante
    Serial.println("Motor 2: Adelante");
    analogWrite(IN3, 255);
    analogWrite(IN4, 0);
    delay(2000);

    // Motor izq hacia atrás
    Serial.println("Motor 2: Atras");
    analogWrite(IN3, 0);
    analogWrite(IN4, 255);
    delay(2000);

    // Detener Motor Izq
    Serial.println("Motor 2: Detenido");
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
    delay(1000);

}
