#include <ESP32Encoder.h>

// Pines del ESP32
#define POT_PIN 35   // Entrada analógica del potenciómetro
#define ENCODER_A 4  // Señal A del encoder
#define ENCODER_B 15 // Señal B del encoder
#define MOTOR_IN1 18 // Control IN1 del L298N
#define MOTOR_IN2 5  // Control IN2 del L298N
#define MOTOR_PWM 19 // PWM (ENA) del L298N

// Variables del encoder
ESP32Encoder encoder;
double rpm = 0;
unsigned long lastTime = 0;

// PID Variables
double setpoint, input, output;
double Kp = 1.2, Ki = 2.85, Kd = 0.001;
double CV_Anterior = 0, error1 = 0, error2 = 0;
double Ts = 0.1; // Tiempo de muestreo en segundos

const int dead_zone = 50; // 🔹 Zona muerta (mínima potencia para mover el motor)

void setup() {
    Serial.begin(115200);

    // Configuración del encoder
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(ENCODER_A, ENCODER_B);
    encoder.clearCount();

    // Configuración de pines del motor
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);

    // Configuración PWM en ESP32
    ledcSetup(0, 1000, 8);
    ledcAttachPin(MOTOR_PWM, 0);

    // Estado inicial
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(0, 0);
}

void loop() {
    // Leer el potenciómetro y escalarlo a un rango de -100 a 100 RPM
    int potValue = analogRead(POT_PIN);
    setpoint = map(potValue, 0, 4095, -200, 200);  

    // Calcular RPM a partir del encoder
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 100) {  // Cada 100 ms
        long count = encoder.getCount();
        rpm = (count / 360.0) * 60.0 / 0.1;  // 360 pulsos por revolución
        encoder.clearCount();
        lastTime = currentTime;
    }

    // Ejecutar control PID
    pidControl();

    // Mostrar datos en Serial Plotter
    imprimirDatos();

    delay(50);
}

void pidControl() {
    double error = setpoint - rpm;
    output = CV_Anterior + (Kp + Kd / Ts) * error + (-Kp + Ki * Ts - 2 * Kd / Ts) * error1 + (Kd / Ts) * error2;

    // Actualizar errores previos
    CV_Anterior = output;
    error2 = error1;
    error1 = error;

    // Compensación de zona muerta
    if (abs(output) > 0 && abs(output) < dead_zone) {
        output = (output > 0) ? dead_zone : -dead_zone;
    }

    // Saturar la señal PWM
    output = constrain(output, -255, 255);

    // Control del motor en ambos sentidos
    if (setpoint == 0) {
        // 🔹 Si el potenciómetro está al centro, detiene el motor
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(0, 0);
    } else if (output > 0) {
        // 🔹 Sentido horario
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(0, abs(output));
    } else {
        // 🔹 Sentido antihorario
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        ledcWrite(0, abs(output));
    }
}

void imprimirDatos() {
    Serial.print("SP:"); Serial.print(setpoint);  // Setpoint (Referencia)
    Serial.print(", PV:"); Serial.print(rpm);    // PV (Variable de Proceso)
    Serial.print(", Min:"); Serial.print(-200);  // Valor mínimo para gráfica
    Serial.print(", Max:"); Serial.print(200);   // Valor máximo para gráfica
    Serial.println();  // Salto de línea para el Serial Plotter
}
