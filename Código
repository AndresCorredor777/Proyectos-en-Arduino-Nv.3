#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PCF8574.h>

// Configuración del teclado con PCF8574
PCF8574 tecladoPCF(0x20); // Dirección del PCF8574 para el teclado (ajusta según tu configuración)
const int filas[4] = {0, 1, 2, 3}; // Pines de las filas en el PCF8574 (P0-P3)
const int columnas[4] = {4, 5, 6, 7}; // Pines de las columnas en el PCF8574 (P4-P7)
char keys[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// Configuración del LCD con PCF8574
LiquidCrystal_I2C lcd(0x27, 16, 2); // Dirección del PCF8574 del LCD

// Pines de los encoders
int pinEncoderA1 = 2; // Encoder Motor 1
int pinEncoderA2 = 3; // Encoder Motor 2
volatile int contadorMotor1 = 0; // Contador de pulsos para Motor 1
volatile int contadorMotor2 = 0; // Contador de pulsos para Motor 2

// Pines del driver L298
int ENA = 6;  // Pin PWM para el motor 1
int ENB = 9;  // Pin PWM para el motor 2
int IN1 = 7;  // Pin de control del motor 1
int IN2 = 8;
int IN3 = 10; // Pin de control del motor 2
int IN4 = 11;

int vueltasObjetivo = 0; // Vueltas objetivo ingresadas por el usuario

// Variables para el control PID
float kp = 2.0;  // Ganancia proporcional
float ki = 6.0;  // Ganancia integral
float kd = 0.01; // Ganancia derivativa
float Tm = 0.1;  // Tiempo de muestreo

// Variables PID para ambos motores
float error1, error2;
float cv1, cv2;
float errorPrev1 = 0, errorPrev2 = 0;
float integral1 = 0, integral2 = 0;

void setup() {
  // Configuración del LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Ingresa vueltas:");

  // Configuración del teclado
  tecladoPCF.begin();

  // Configuración de los encoders
  pinMode(pinEncoderA1, INPUT);
  pinMode(pinEncoderA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEncoderA1), contarPulsosMotor1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderA2), contarPulsosMotor2, RISING);

  // Configuración de los pines del driver L298
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Inicialización del Monitor Serial
  Serial.begin(115200);
  Serial.println("Sistema iniciado. Ingrese el número de vueltas:");
}

void loop() {
  char tecla = leerTeclado(); // Leer la tecla presionada
  if (tecla) {
    if (tecla >= '0' && tecla <= '9') { // Si se presionó un número
      vueltasObjetivo = vueltasObjetivo * 10 + (tecla - '0'); // Construir el número de vueltas
      lcd.setCursor(0, 1);
      lcd.print("Vueltas: ");
      lcd.print(vueltasObjetivo); // Mostrar en la LCD
    } else if (tecla == '#') { // Confirmar vueltas con '#'
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Comenzando...");
      Serial.print("Vueltas objetivo: ");
      Serial.println(vueltasObjetivo);
      iniciarMotores(); // Función que inicia los motores con las vueltas ingresadas
    }
  }

  delay(500); // Actualización cada medio segundo
}

// Función para leer el teclado conectado al PCF8574
char leerTeclado() {
  for (int fila = 0; fila < 4; fila++) {
    tecladoPCF.write(filas[fila], LOW); // Configurar la fila como salida baja
    for (int columna = 0; columna < 4; columna++) {
      if (tecladoPCF.read(columnas[columna]) == LOW) {
        delay(50); // Esperar para evitar rebotes
        while (tecladoPCF.read(columnas[columna]) == LOW);
        return keys[fila][columna]; // Devolver la tecla presionada
      }
    }
    tecladoPCF.write(filas[fila], HIGH); // Restaurar la fila a estado alto
  }
  return 0; // Si no se presionó ninguna tecla
}

// Función para contar los pulsos del encoder del Motor 1
void contarPulsosMotor1() {
  contadorMotor1++;
}

// Función para contar los pulsos del encoder del Motor 2
void contarPulsosMotor2() {
  contadorMotor2++;
}

// Función para iniciar los motores y controlar la velocidad con PID para sincronizar las vueltas
void iniciarMotores() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  while (contadorMotor1 < vueltasObjetivo || contadorMotor2 < vueltasObjetivo) {
    // Control PID para Motor 1
    error1 = vueltasObjetivo - contadorMotor1;
    integral1 += error1 * Tm;
    cv1 = kp * error1 + ki * integral1 + kd * (error1 - errorPrev1) / Tm;
    errorPrev1 = error1;
    
    if (cv1 > 255.0) cv1 = 255.0;
    if (cv1 < 0) cv1 = 0;
    
    // Control PID para Motor 2
    error2 = vueltasObjetivo - contadorMotor2;
    integral2 += error2 * Tm;
    cv2 = kp * error2 + ki * integral2 + kd * (error2 - errorPrev2) / Tm;
    errorPrev2 = error2;
    
    if (cv2 > 255.0) cv2 = 255.0;
    if (cv2 < 0) cv2 = 0;

    // Sincronización de motores
    // Si un motor ha completado más vueltas que el otro, reducir su velocidad proporcionalmente a la diferencia
    int diferenciaVueltas = abs(contadorMotor1 - contadorMotor2);

    if (contadorMotor1 > contadorMotor2) {
      cv1 -= diferenciaVueltas * 7;  // Reducción más fuerte proporcional al exceso de vueltas
      if (cv1 < 50) cv1 = 50;  // Límite mínimo más bajo para el motor 1
    } else if (contadorMotor2 > contadorMotor1) {
      cv2 -= diferenciaVueltas * 7;  // Reducción más fuerte proporcional al exceso de vueltas
      if (cv2 < 50) cv2 = 50;  // Límite mínimo más bajo para el motor 2
    }

    analogWrite(ENA, cv1); // Ajustar PWM del motor 1
    analogWrite(ENB, cv2); // Ajustar PWM del motor 2

    // Mostrar las vueltas en la pantalla LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Vueltas:");
    lcd.setCursor(0, 1);
    lcd.print("M1:");
    lcd.print(contadorMotor1);
    lcd.print(" M2:");
    lcd.print(contadorMotor2);

    // Mostrar información en el Monitor Serial
    Serial.print("Motor 1 vueltas: ");
    Serial.print(contadorMotor1);
    Serial.print("\tMotor 2 vueltas: ");
    Serial.print(contadorMotor2);
    Serial.print("\tPWM M1: ");
    Serial.print(cv1);
    Serial.print("\tPWM M2: ");
    Serial.println(cv2);

    delay(500);  // Actualización cada 500ms
  }

  // Detener los motores una vez alcanzado el objetivo
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vueltas completas");

  // Mostrar mensaje en el Monitor Serial
  Serial.println("Vueltas completas");
}
