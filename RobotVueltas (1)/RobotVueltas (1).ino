/*
  Robot seguidor de pasillo con giro hacia aberturas,
  contador de vueltas y detección de sentido de giro
  (2 sensores ultrasónicos: derecho e izquierdo).

  Lógica de movimiento:
   - Si ambos sensores detectan pared cerca (distancia <= UMBRAL_GIRO)
     el robot sigue recto.
   - Si un sensor detecta una abertura (distancia > UMBRAL_GIRO), el
     robot gira hacia ESE lado (hacia el espacio libre).
   - Si hay abertura en ambos lados a la vez, gira hacia el que tenga
     MÁS espacio.

  Conteo de giros, vueltas y sentido:
   - Cada giro nuevo (empezar a girar, o cambiar de lado sin pasar por
     recto) suma +1 al total Y al contador de su propia dirección
     (derecha o izquierda), con un debounce por tiempo para que el
     ruido del sensor no cuente el mismo giro dos veces.
   - Cada 4 giros = 1 vuelta completa. Al cerrar la vuelta se informa
     cuántos giros fueron a la derecha, cuántos a la izquierda, y el
     sentido aproximado de esa vuelta (HORARIO si predominó la derecha,
     ANTIHORARIO si predominó la izquierda). Es una heurística por
     conteo, no una medición real de rumbo (para eso haría falta un
     giroscopio/IMU o encoders).
   - Al llegar a 3 vueltas completas, el robot se detiene por completo.
*/

#include <Servo.h>
#include <Arduino.h>

// ---------------- Pines de sensores ultrasónicos ----------------
const int TRIG_DER = 2;
const int ECHO_DER = 4;
const int TRIG_IZQ = 12;
const int ECHO_IZQ = 13;

// ---------------- Pines del driver L298N ----------------
const int ENA = 11;
const int IN3 = 10;
const int IN4 = 9;

// ---------------- Servo de dirección ----------------
const int SERVOPIN = 6;
Servo myServo;

// Límites mecánicos seguros del servo
const int SERVO_MIN_SEGURO = 20;
const int SERVO_MAX_SEGURO = 160;

// ---------------- Velocidades (PWM 0-255) ----------------
const int VEL_RECTA = 255; // máxima velocidad en línea recta
const int VEL_CURVA = 255; // máxima velocidad también al girar

// ---------------- Ángulos de dirección ----------------
const int ANGULO_RECTO = 90;
const int ANGULO_IZQ   = 50;
const int ANGULO_DER   = 130;

// ---------------- Umbral de decisión de giro ----------------
// Si la distancia leída supera este valor, ese lado tiene una
// ABERTURA (espacio para girar). Rango pedido: 20-30 cm; se dejó el
// extremo más sensible (20). Subilo hasta 30 si gira muy pronto.
const float UMBRAL_GIRO = 20.0;

// Distancia asumida cuando el sensor NO recibe eco (timeout): no hay
// pared en rango, o sea que HAY espacio -> nunca se trata como 0.
const float DISTANCIA_SIN_ECO = 400.0;

// ---------------- Conteo de giros, vueltas y sentido ----------------
const int GIROS_POR_VUELTA = 4;
const int VUELTAS_PARA_DETENER = 3;
const unsigned long TIEMPO_MINIMO_ENTRE_GIROS = 500; // ms, evita doble conteo por ruido

int contadorGiros = 0;        // giros acumulados en la vuelta actual (0-4)
int vueltasCompletas = 0;

int girosDerechaVuelta = 0;   // giros a la derecha en la vuelta actual
int girosIzquierdaVuelta = 0; // giros a la izquierda en la vuelta actual

int totalGirosDerecha = 0;    // giros a la derecha en todo el recorrido
int totalGirosIzquierda = 0;  // giros a la izquierda en todo el recorrido

bool robotDetenido = false;
unsigned long tiempoUltimoGiro = 0;

enum EstadoMovimiento { RECTO, GIRANDO_IZQ, GIRANDO_DER };
EstadoMovimiento estadoAnterior = RECTO;

float der, izq;

void moverServo(int angulo) {
  angulo = constrain(angulo, SERVO_MIN_SEGURO, SERVO_MAX_SEGURO);
  myServo.write(angulo);
}

// Fuerza la polaridad de avance en el puente H. Se llama en setup() y en
// cada vuelta del loop() para que el sentido de marcha quede reafirmado
// todo el tiempo, y no dependa de que "nadie más lo toque".
void asegurarAdelante() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_DER, OUTPUT);
  pinMode(ECHO_DER, INPUT);
  pinMode(TRIG_IZQ, OUTPUT);
  pinMode(ECHO_IZQ, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  myServo.attach(SERVOPIN);
  moverServo(ANGULO_RECTO);

  asegurarAdelante();
  analogWrite(ENA, VEL_RECTA);
}

float leerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH, 25000); // timeout 25 ms (~4.2 m)
  if (duracion == 0) return DISTANCIA_SIN_ECO;    // sin eco = espacio libre
  return duracion * 0.034 / 2;
}

void irRecto() {
  moverServo(ANGULO_RECTO);
  analogWrite(ENA, VEL_RECTA);
}

void girarIzquierda() {
  moverServo(ANGULO_IZQ);
  analogWrite(ENA, VEL_CURVA);
}

void girarDerecha() {
  moverServo(ANGULO_DER);
  analogWrite(ENA, VEL_CURVA);
}

void detenerRobot() {
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  moverServo(ANGULO_RECTO);
}

void imprimirResumenFinal() {
  Serial.println(F(">>> 3 vueltas completas. Robot detenido."));
  Serial.print(F("Total giros derecha: "));
  Serial.println(totalGirosDerecha);
  Serial.print(F("Total giros izquierda: "));
  Serial.println(totalGirosIzquierda);
}

// Registra un giro nuevo indicando su dirección (GIRANDO_DER o GIRANDO_IZQ)
void registrarGiro(EstadoMovimiento direccion) {
  contadorGiros++;

  if (direccion == GIRANDO_DER) {
    girosDerechaVuelta++;
    totalGirosDerecha++;
    Serial.print(F("Giro #")); Serial.print(contadorGiros); Serial.println(F(" -> DERECHA"));
  } else {
    girosIzquierdaVuelta++;
    totalGirosIzquierda++;
    Serial.print(F("Giro #")); Serial.print(contadorGiros); Serial.println(F(" -> IZQUIERDA"));
  }

  if (contadorGiros >= GIROS_POR_VUELTA) {
    contadorGiros = 0;
    vueltasCompletas++;

    Serial.print(F(">>> Vuelta completa #")); Serial.print(vueltasCompletas);
    Serial.print(F(" | Derecha: ")); Serial.print(girosDerechaVuelta);
    Serial.print(F(" Izquierda: ")); Serial.print(girosIzquierdaVuelta);
    Serial.print(F(" | Sentido: "));
    if (girosDerechaVuelta > girosIzquierdaVuelta) {
      Serial.println(F("HORARIO"));
    } else if (girosIzquierdaVuelta > girosDerechaVuelta) {
      Serial.println(F("ANTIHORARIO"));
    } else {
      Serial.println(F("INDEFINIDO (empate)"));
    }

    girosDerechaVuelta = 0;
    girosIzquierdaVuelta = 0;

    if (vueltasCompletas >= VUELTAS_PARA_DETENER) {
      robotDetenido = true;
      detenerRobot();
      imprimirResumenFinal();
    }
  }
}

void loop() {
  if (robotDetenido) return;

  asegurarAdelante(); // refuerzo: nunca reversa, siempre avanzando

  der = leerDistancia(TRIG_DER, ECHO_DER);
  izq = leerDistancia(TRIG_IZQ, ECHO_IZQ);

  Serial.print(F("Der: ")); Serial.print(der);
  Serial.print(F(" cm | Izq: ")); Serial.print(izq);
  Serial.println(F(" cm"));

  bool derAbrio = der > UMBRAL_GIRO;
  bool izqAbrio = izq > UMBRAL_GIRO;

  EstadoMovimiento estadoNuevo;
  if (derAbrio && izqAbrio) {
    estadoNuevo = (der > izq) ? GIRANDO_DER : GIRANDO_IZQ; // gira hacia el lado con más espacio
  } else if (derAbrio) {
    estadoNuevo = GIRANDO_DER;
  } else if (izqAbrio) {
    estadoNuevo = GIRANDO_IZQ;
  } else {
    estadoNuevo = RECTO; // pared cerca en ambos lados -> recto
  }

  if (estadoNuevo == RECTO) {
    irRecto();
  } else if (estadoNuevo == GIRANDO_DER) {
    girarDerecha();
  } else {
    girarIzquierda();
  }

  // Cuenta un giro nuevo al empezar a girar, o al cambiar de lado sin pasar por recto
  if (estadoNuevo != RECTO && estadoNuevo != estadoAnterior) {
    unsigned long ahora = millis();
    if (ahora - tiempoUltimoGiro >= TIEMPO_MINIMO_ENTRE_GIROS) {
      registrarGiro(estadoNuevo);
      tiempoUltimoGiro = ahora;
    }
  }
  estadoAnterior = estadoNuevo;

  delay(60);
}
