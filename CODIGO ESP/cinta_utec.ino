#include <Arduino.h>
#include <BluetoothSerial.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================
// CINTA TRANSPORTADORA UTEC - ESP32 + TMC2208
//
// Cambios respecto a la version anterior (buscar las etiquetas):
//  [CAMBIO 1] Encoder: metodo M/T -> pulsos / tiempo real entre pulsos
//             (elimina el error de cuantizacion de la ventana fija)
//  [CAMBIO 2] Divergencia: se evalua desde que se alcanza el regimen,
//             no desde que llega la orden
//  [CAMBIO 3] Microstepping: solo se permite cambiar con motor detenido
//  [CAMBIO 4] I2C a 400 kHz para que la OLED no bloquee el loop
// =====================================================

// =====================================================
// CONFIGURACION GENERAL
// =====================================================

// --------------------
// MECANICA
// --------------------

#define MOTOR_STEPS             200

#define MOTOR_GEAR_TEETH        19.0f
#define IDLER_GEAR_TEETH        19.0f   // informativo
#define ROLLER_GEAR_TEETH       40.0f

#define ROLLER_DIAMETER_MM      29.5f

// Disco encoder
#define ENCODER_PPR             20.0f

// Largo util de la cinta
#define BELT_LENGTH_CM          45.0f

// Offset fisico HC-SR04
// CAMBIAR ACA DESPUES DE CALIBRAR
float HC_OFFSET_CM = 4.0f;

// --------------------
// MOTOR
// --------------------

// V100 = 150 RPM del NEMA
#define MAX_MOTOR_RPM           150.0f

// Aceleracion
#define ACCEL_RPM_PER_SEC       120.0f

// Frenado mas rapido
#define DECEL_RPM_PER_SEC       300.0f

// --------------------
// DIVERGENCIA
// --------------------

#define SPEED_ERROR_LIMIT       15.0f

// [CAMBIO 2] Tiempo extra en regimen antes de evaluar divergencia.
// Se suma a ENCODER_SAMPLE_MS para asegurar que la ventana del
// encoder no contenga ninguna parte de la rampa.
#define DIVERGENCE_MARGIN_MS    500

// Sin pulsos durante este tiempo -> rodillo detenido
#define ENCODER_STOP_TIMEOUT_US 2000000UL

// =====================================================
// PINES
// =====================================================

// TMC2208
#define STEP_PIN        25
#define DIR_PIN         26
#define EN_PIN          27
#define MS1_PIN         32
#define MS2_PIN         33

// Encoder
#define ENCODER_PIN     23

// HC-SR04
#define TRIG_PIN        18
#define ECHO_PIN        19

// OLED
#define SDA_PIN         21
#define SCL_PIN         22

// =====================================================
// OLED
// =====================================================

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDRESS    0x3C

// [CAMBIO 4]
#define I2C_CLOCK_HZ    400000UL

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =====================================================
// BLUETOOTH
// =====================================================

BluetoothSerial SerialBT;

// =====================================================
// DIRECCION
// =====================================================

enum Direccion {
  STOPPED = 0,
  FORWARD = 1,
  REVERSE = -1
};

Direccion direccionSolicitada = STOPPED;
Direccion direccionActual = STOPPED;

// =====================================================
// MOTOR
// =====================================================

int velocidadPercent = 50;
int microsteps = 8;

// RPM objetivo final
float rpmMotorObjetivo = 0.0f;

// RPM que estamos enviando actualmente
float rpmMotorComandada = 0.0f;

// RPM teorica del rodillo
float rpmRodilloTeorica = 0.0f;

// STEP/s
float stepRate = 0.0f;

// Relacion total
const float GEAR_RATIO = MOTOR_GEAR_TEETH / ROLLER_GEAR_TEETH;

// Circunferencia rodillo en cm
const float ROLLER_CIRCUMFERENCE_CM = PI * (ROLLER_DIAMETER_MM / 10.0f);

// =====================================================
// TIMER HARDWARE PARA STEP
// =====================================================

// Timer a 10 MHz: cada tick = 0.1 us
#define STEP_TIMER_HZ 10000000UL

hw_timer_t *stepTimer = nullptr;

volatile bool stepPinState = false;
bool stepTimerRunning = false;
float ultimoStepRateAplicado = -1.0f;

// =====================================================
// ENCODER
// =====================================================

volatile uint32_t encoderPulses = 0;
volatile uint32_t lastEncoderPulseUs = 0;

// Evita pulsos espurios absurdamente rapidos
#define ENCODER_MIN_GAP_US 500

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

float rpmRodilloReal = 0.0f;

float velocidadRealCmS = 0.0f;
float velocidadTeoricaCmS = 0.0f;

float errorRPM = 0.0f;

unsigned long ultimoEncoderMs = 0;

#define ENCODER_SAMPLE_MS 1000

// =====================================================
// HC-SR04
// =====================================================

float distanciaRawCm = 0.0f;
float posicionObjetoCm = 0.0f;

bool hcValido = false;
bool primeraLecturaHC = true;

unsigned long ultimoHC = 0;

#define HC_SAMPLE_MS   100

// Aproximadamente 1.7 m max. Nuestra cinta usa < 50 cm.
#define HC_TIMEOUT_US  10000UL

// Filtro exponencial
#define HC_ALPHA       0.35f

// =====================================================
// DIVERGENCIAS
// =====================================================

bool divergenciaVelocidad = false;
bool motorEnRegimen = false;

unsigned long ultimaOrdenMovimientoMs = 0;

// [CAMBIO 2] Momento en que se alcanzo el regimen
unsigned long regimenDesdeMs = 0;

// =====================================================
// OLED
// =====================================================

unsigned long ultimoOLED = 0;

#define OLED_REFRESH_MS 200

// =====================================================
// TELEMETRIA
// =====================================================

unsigned long ultimaTelemetria = 0;

#define TELEMETRY_MS 1000

// =====================================================
// COMANDOS SERIAL / BLUETOOTH
// =====================================================

String comandoUSB = "";
String comandoBT = "";

unsigned long ultimoCharUSB = 0;
unsigned long ultimoCharBT = 0;

#define COMMAND_TIMEOUT_MS 120

// =====================================================
// INTERRUPCION STEP
// =====================================================

void ARDUINO_ISR_ATTR stepISR() {
  stepPinState = !stepPinState;
  digitalWrite(STEP_PIN, stepPinState);
}

// =====================================================
// INTERRUPCION ENCODER
// [CAMBIO 1] Ademas de contar, guarda el instante exacto
// del ultimo pulso valido (dentro de la seccion critica).
// =====================================================

void ARDUINO_ISR_ATTR encoderISR() {

  uint32_t ahora = micros();

  if ((uint32_t)(ahora - lastEncoderPulseUs) >= ENCODER_MIN_GAP_US) {

    portENTER_CRITICAL_ISR(&encoderMux);

    lastEncoderPulseUs = ahora;
    encoderPulses++;

    portEXIT_CRITICAL_ISR(&encoderMux);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // ===================================================
  // BLUETOOTH
  // ===================================================

  SerialBT.begin("CINTA_UTEC");

  // ===================================================
  // OLED
  // ===================================================

  Wire.begin(SDA_PIN, SCL_PIN);

  // [CAMBIO 4] 400 kHz: display() pasa de ~90 ms a ~25 ms
  Wire.setClock(I2C_CLOCK_HZ);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("ERROR OLED");
    while (true);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 8);
  display.println("CINTA TRANSPORTADORA");

  display.setCursor(0, 25);
  display.println("ESP32 + TMC2208");

  display.setCursor(0, 42);
  display.println("Iniciando...");

  display.display();

  // ===================================================
  // TMC2208
  // ===================================================

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);

  // ENABLE activo en LOW
  digitalWrite(EN_PIN, LOW);

  configurarMicrosteps(8);

  // ===================================================
  // TIMER STEP
  // ===================================================

  stepTimer = timerBegin(STEP_TIMER_HZ);

  timerStop(stepTimer);

  timerAttachInterrupt(stepTimer, &stepISR);

  // Valor inicial cualquiera. Timer queda detenido.
  timerAlarm(stepTimer, 10000, true, 0);

  // ===================================================
  // ENCODER
  // ===================================================

  pinMode(ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  // ===================================================
  // HC-SR04
  // ===================================================

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  // ===================================================
  // INICIO
  // ===================================================

  delay(800);

  Serial.println();
  Serial.println("============================");
  Serial.println(" CINTA TRANSPORTADORA UTEC");
  Serial.println("============================");
  Serial.println();
  Serial.println("Bluetooth: CINTA_UTEC");
  Serial.println();
  Serial.println("COMANDOS:");
  Serial.println("F       Adelante");
  Serial.println("R       Reversa");
  Serial.println("S       Stop");
  Serial.println("E       Stop inmediato");
  Serial.println();
  Serial.println("V50     Velocidad 50%");
  Serial.println("V100    Velocidad 100%");
  Serial.println();
  Serial.println("M2      1/2   (solo con motor detenido)");
  Serial.println("M4      1/4");
  Serial.println("M8      1/8");
  Serial.println("M16     1/16");
  Serial.println();
  Serial.println("STATUS  Estado actual");
  Serial.println();
  Serial.println("O4.0    Cambiar offset HC");
  Serial.println();

  ultimoEncoderMs = millis();
  ultimaOrdenMovimientoMs = millis();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  leerUSB();
  leerBluetooth();
  comprobarTimeoutComandos();

  actualizarMotor();

  calcularEncoder();

  medirHCSR04();

  calcularDivergencias();

  actualizarOLED();

  enviarTelemetria();
}

// =====================================================
// USB
// =====================================================

void leerUSB() {

  while (Serial.available()) {

    char c = Serial.read();
    ultimoCharUSB = millis();

    if (c == '\n' || c == '\r') {
      if (comandoUSB.length() > 0) {
        procesarComando(comandoUSB);
        comandoUSB = "";
      }
    } else {
      comandoUSB += c;
    }
  }
}

// =====================================================
// BLUETOOTH
// =====================================================

void leerBluetooth() {

  while (SerialBT.available()) {

    char c = SerialBT.read();
    ultimoCharBT = millis();

    if (c == '\n' || c == '\r') {
      if (comandoBT.length() > 0) {
        procesarComando(comandoBT);
        comandoBT = "";
      }
    } else {
      comandoBT += c;
    }
  }
}

// =====================================================
// PROCESAR COMANDOS AUN SIN ENTER
// =====================================================

void comprobarTimeoutComandos() {

  if (comandoUSB.length() > 0 && millis() - ultimoCharUSB > COMMAND_TIMEOUT_MS) {
    procesarComando(comandoUSB);
    comandoUSB = "";
  }

  if (comandoBT.length() > 0 && millis() - ultimoCharBT > COMMAND_TIMEOUT_MS) {
    procesarComando(comandoBT);
    comandoBT = "";
  }
}

// =====================================================
// PROCESAR COMANDO
// =====================================================

void procesarComando(String cmd) {

  cmd.trim();
  cmd.toUpperCase();

  // ADELANTE
  if (cmd == "F") {

    direccionSolicitada = FORWARD;
    ultimaOrdenMovimientoMs = millis();
    imprimirAmbos("Direccion: ADELANTE");
  }

  // REVERSA
  else if (cmd == "R") {

    direccionSolicitada = REVERSE;
    ultimaOrdenMovimientoMs = millis();
    imprimirAmbos("Direccion: REVERSA");
  }

  // STOP SUAVE
  else if (cmd == "S") {

    direccionSolicitada = STOPPED;
    ultimaOrdenMovimientoMs = millis();
    imprimirAmbos("STOP");
  }

  // STOP INMEDIATO
  else if (cmd == "E") {

    direccionSolicitada = STOPPED;
    direccionActual = STOPPED;
    rpmMotorComandada = 0;

    detenerSteps();

    ultimaOrdenMovimientoMs = millis();
    imprimirAmbos("STOP INMEDIATO");
  }

  // VELOCIDAD
  else if (cmd.startsWith("V")) {

    int nueva = cmd.substring(1).toInt();

    if (nueva >= 0 && nueva <= 100) {

      velocidadPercent = nueva;

      if (nueva == 0) {
        direccionSolicitada = STOPPED;
      }

      ultimaOrdenMovimientoMs = millis();
      imprimirAmbos("Velocidad: " + String(velocidadPercent) + "%");

    } else {
      imprimirAmbos("V debe ser 0-100");
    }
  }

  // MICROSTEPPING
  else if (cmd.startsWith("M")) {

    int nuevo = cmd.substring(1).toInt();

    if (nuevo == 2 || nuevo == 4 || nuevo == 8 || nuevo == 16) {
      cambiarMicrostepsSeguro(nuevo);
    } else {
      imprimirAmbos("Use M2 M4 M8 M16");
    }
  }

  // OFFSET HC  (ejemplo O4.5)
  else if (cmd.startsWith("O")) {

    float nuevoOffset = cmd.substring(1).toFloat();

    if (nuevoOffset >= 0 && nuevoOffset < 20) {
      HC_OFFSET_CM = nuevoOffset;
      imprimirAmbos("Offset HC: " + String(HC_OFFSET_CM, 2));
    } else {
      imprimirAmbos("Offset invalido");
    }
  }

  // STATUS
  else if (cmd == "STATUS") {
    enviarEstado();
  }

  else {
    imprimirAmbos("Comando invalido");
  }
}

// =====================================================
// MICROSTEPPING
// Tabla TMC2208 standalone:
//   MS1 MS2
//   LOW LOW  -> 1/8
//   HIGH LOW -> 1/2
//   LOW HIGH -> 1/4
//   HIGH HIGH-> 1/16
// =====================================================

void configurarMicrosteps(int valor) {

  switch (valor) {

    case 2:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, LOW);
      break;

    case 4:
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, HIGH);
      break;

    case 8:
      digitalWrite(MS1_PIN, LOW);
      digitalWrite(MS2_PIN, LOW);
      break;

    case 16:
      digitalWrite(MS1_PIN, HIGH);
      digitalWrite(MS2_PIN, HIGH);
      break;
  }

  microsteps = valor;
}

// =====================================================
// CAMBIO SEGURO MICROSTEPS
// [CAMBIO 3] Solo con el motor completamente detenido.
// Evita desenergizar el motor con la cinta en movimiento y
// evita aplicar un stepRate calculado con el microstep anterior.
// =====================================================

void cambiarMicrostepsSeguro(int valor) {

  if (direccionActual != STOPPED || rpmMotorComandada > 0.0f) {
    imprimirAmbos("Detener (S) antes de cambiar M");
    return;
  }

  detenerSteps();

  digitalWrite(EN_PIN, HIGH);
  delay(5);

  configurarMicrosteps(valor);

  delay(5);
  digitalWrite(EN_PIN, LOW);

  ultimaOrdenMovimientoMs = millis();

  imprimirAmbos("Microstep: 1/" + String(microsteps));
}

// =====================================================
// RAMPA MOTOR
// =====================================================

void actualizarMotor() {

  static unsigned long ultimoControl = millis();

  unsigned long ahora = millis();

  if (ahora - ultimoControl < 20) {
    return;
  }

  float dt = (ahora - ultimoControl) / 1000.0f;
  ultimoControl = ahora;

  // ---------------------------------------------------
  // SI ESTA DETENIDO Y SE PIDE UNA DIRECCION
  // ---------------------------------------------------

  if (direccionActual == STOPPED &&
      direccionSolicitada != STOPPED &&
      rpmMotorComandada < 0.1f) {

    direccionActual = direccionSolicitada;
    aplicarDireccion();
  }

  // ---------------------------------------------------
  // RPM OBJETIVO FINAL
  // ---------------------------------------------------

  if (direccionSolicitada == STOPPED) {
    rpmMotorObjetivo = 0;
  } else {
    rpmMotorObjetivo = MAX_MOTOR_RPM * (velocidadPercent / 100.0f);
  }

  // ---------------------------------------------------
  // CAMBIO DE SENTIDO: primero llevar RPM a cero
  // ---------------------------------------------------

  float objetivoTemporal = rpmMotorObjetivo;

  if (direccionActual != direccionSolicitada && direccionActual != STOPPED) {
    objetivoTemporal = 0;
  }

  // ---------------------------------------------------
  // ACELERACION / FRENADO
  // ---------------------------------------------------

  if (rpmMotorComandada < objetivoTemporal) {

    rpmMotorComandada += ACCEL_RPM_PER_SEC * dt;

    if (rpmMotorComandada > objetivoTemporal) {
      rpmMotorComandada = objetivoTemporal;
    }

  } else if (rpmMotorComandada > objetivoTemporal) {

    rpmMotorComandada -= DECEL_RPM_PER_SEC * dt;

    if (rpmMotorComandada < objetivoTemporal) {
      rpmMotorComandada = objetivoTemporal;
    }
  }

  // ---------------------------------------------------
  // CAMBIO EFECTIVO DE SENTIDO
  // ---------------------------------------------------

  if (rpmMotorComandada < 0.1f && direccionActual != direccionSolicitada) {

    rpmMotorComandada = 0;
    direccionActual = direccionSolicitada;

    if (direccionActual != STOPPED) {
      aplicarDireccion();
    }
  }

  // ---------------------------------------------------
  // RPM RODILLO Y VELOCIDAD TEORICA
  // ---------------------------------------------------

  rpmRodilloTeorica = rpmMotorComandada * GEAR_RATIO;

  velocidadTeoricaCmS = (rpmRodilloTeorica * ROLLER_CIRCUMFERENCE_CM) / 60.0f;

  // ---------------------------------------------------
  // [CAMBIO 2] DETECTAR REGIMEN Y GUARDAR DESDE CUANDO
  // Solo hay regimen si hay un objetivo de movimiento
  // y la rampa ya llego a el.
  // ---------------------------------------------------

  bool enRegimenAhora =
    rpmMotorObjetivo > 0.0f &&
    fabs(rpmMotorComandada - rpmMotorObjetivo) < 2.0f;

  if (enRegimenAhora && !motorEnRegimen) {
    regimenDesdeMs = ahora;
  }

  motorEnRegimen = enRegimenAhora;

  // ---------------------------------------------------
  // STEP RATE
  // ---------------------------------------------------

  stepRate = (rpmMotorComandada * MOTOR_STEPS * microsteps) / 60.0f;

  aplicarFrecuenciaStep();
}

// =====================================================
// DIRECCION TMC
// =====================================================

void aplicarDireccion() {

  if (direccionActual == FORWARD) {
    digitalWrite(DIR_PIN, HIGH);
  } else if (direccionActual == REVERSE) {
    digitalWrite(DIR_PIN, LOW);
  }
}

// =====================================================
// APLICAR FRECUENCIA STEP
// =====================================================

void aplicarFrecuenciaStep() {

  if (stepRate < 0.5f) {
    detenerSteps();
    return;
  }

  // Evitamos reprogramar el timer si casi no cambio la frecuencia
  if (fabs(stepRate - ultimoStepRateAplicado) < 1.0f) {
    return;
  }

  ultimoStepRateAplicado = stepRate;

  uint64_t medioPeriodoTicks = (uint64_t)(STEP_TIMER_HZ / (2.0 * stepRate));

  if (medioPeriodoTicks < 2) {
    medioPeriodoTicks = 2;
  }

  timerAlarm(stepTimer, medioPeriodoTicks, true, 0);

  if (!stepTimerRunning) {
    timerWrite(stepTimer, 0);
    timerStart(stepTimer);
    stepTimerRunning = true;
  }
}

// =====================================================
// DETENER PULSOS
// =====================================================

void detenerSteps() {

  if (stepTimerRunning) {
    timerStop(stepTimer);
    stepTimerRunning = false;
  }

  stepPinState = false;
  digitalWrite(STEP_PIN, LOW);

  ultimoStepRateAplicado = -1;
}

// =====================================================
// ENCODER
// [CAMBIO 1] Metodo M/T:
//   RPM = (pulsos / PPR) * 60 / (t_ultimoPulso - t_ultimoPulsoVentanaAnterior)
// El tiempo se mide en microsegundos entre flancos reales,
// no con la duracion de la ventana, asi que desaparece el
// error de +-1 pulso en los bordes.
// =====================================================

void calcularEncoder() {

  unsigned long ahora = millis();

  if (ahora - ultimoEncoderMs < ENCODER_SAMPLE_MS) {
    return;
  }

  ultimoEncoderMs = ahora;

  // Instante del ultimo pulso de la ventana anterior.
  // 0 = no hay referencia valida (arranque o rodillo detenido).
  static uint32_t refUs = 0;

  uint32_t pulsos;
  uint32_t ultimoUs;

  portENTER_CRITICAL(&encoderMux);

  pulsos = encoderPulses;
  encoderPulses = 0;
  ultimoUs = lastEncoderPulseUs;

  portEXIT_CRITICAL(&encoderMux);

  if (pulsos > 0) {

    if (refUs != 0) {

      uint32_t dtUs = ultimoUs - refUs;

      if (dtUs > 0) {
        rpmRodilloReal = (pulsos / ENCODER_PPR) * 60.0e6f / (float)dtUs;
      }
    }
    // Si refUs == 0 es la primera ventana tras arrancar:
    // solo se toma la referencia y se calcula en la siguiente.

    refUs = ultimoUs;

  } else if ((uint32_t)(micros() - ultimoUs) > ENCODER_STOP_TIMEOUT_US) {

    // Sin pulsos durante un buen rato -> detenido
    rpmRodilloReal = 0;
    refUs = 0;
  }

  // ---------------------------------------------------
  // VELOCIDAD LINEAL REAL
  // ---------------------------------------------------

  velocidadRealCmS = (rpmRodilloReal * ROLLER_CIRCUMFERENCE_CM) / 60.0f;

  // ---------------------------------------------------
  // ERROR
  // ---------------------------------------------------

  if (rpmRodilloTeorica > 1.0f) {
    errorRPM = fabs(rpmRodilloReal - rpmRodilloTeorica) / rpmRodilloTeorica * 100.0f;
  } else {
    errorRPM = 0;
  }
}

// =====================================================
// HC-SR04
// =====================================================

void medirHCSR04() {

  unsigned long ahora = millis();

  if (ahora - ultimoHC < HC_SAMPLE_MS) {
    return;
  }

  ultimoHC = ahora;

  // Pulso Trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duracion = pulseIn(ECHO_PIN, HIGH, HC_TIMEOUT_US);

  if (duracion == 0) {
    hcValido = false;
    return;
  }

  // Velocidad sonido aprox
  float distancia = (duracion * 0.0343f) / 2.0f;

  distanciaRawCm = distancia;

  // Ventana valida para nuestra cinta
  float distanciaMin = HC_OFFSET_CM - 1.0f;
  float distanciaMax = HC_OFFSET_CM + BELT_LENGTH_CM + 2.0f;

  if (distancia >= distanciaMin && distancia <= distanciaMax) {

    float posicion = distancia - HC_OFFSET_CM;
    posicion = constrain(posicion, 0.0f, BELT_LENGTH_CM);

    if (primeraLecturaHC) {
      posicionObjetoCm = posicion;
      primeraLecturaHC = false;
    } else {
      posicionObjetoCm = (HC_ALPHA * posicion) + ((1.0f - HC_ALPHA) * posicionObjetoCm);
    }

    hcValido = true;

  } else {
    hcValido = false;
  }
}

// =====================================================
// DIVERGENCIAS
// [CAMBIO 2] Se exige que el motor lleve en regimen al menos
// una ventana completa del encoder + margen, para que el
// calculo de RPM real no incluya nada de la rampa.
// =====================================================

void calcularDivergencias() {

  divergenciaVelocidad = false;

  if (rpmRodilloTeorica > 5.0f &&
      motorEnRegimen &&
      millis() - regimenDesdeMs > (ENCODER_SAMPLE_MS + DIVERGENCE_MARGIN_MS)) {

    if (errorRPM > SPEED_ERROR_LIMIT) {
      divergenciaVelocidad = true;
    }
  }
}

// =====================================================
// OLED
// =====================================================

void actualizarOLED() {

  unsigned long ahora = millis();

  if (ahora - ultimoOLED < OLED_REFRESH_MS) {
    return;
  }

  ultimoOLED = ahora;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // LINEA 1: direccion, velocidad, microstep
  display.setCursor(0, 0);

  if (direccionActual == FORWARD) {
    display.print("FWD");
  } else if (direccionActual == REVERSE) {
    display.print("REV");
  } else {
    display.print("STOP");
  }

  display.print(" V:");
  display.print(velocidadPercent);
  display.print("% M:");
  display.print(microsteps);

  // LINEA 2: RPM
  display.setCursor(0, 11);
  display.print("RPM T:");
  display.print(rpmRodilloTeorica, 1);
  display.print(" R:");
  display.print(rpmRodilloReal, 1);

  // LINEA 3: cm/s
  display.setCursor(0, 22);
  display.print("cm/s T:");
  display.print(velocidadTeoricaCmS, 1);
  display.print(" R:");
  display.print(velocidadRealCmS, 1);

  // LINEA 4: posicion
  display.setCursor(0, 33);
  display.print("Pos:");

  if (hcValido) {
    display.print(posicionObjetoCm, 1);
    display.print("cm");
  } else {
    display.print("--.-cm");
  }

  // LINEA 5: error
  display.setCursor(0, 44);
  display.print("Err:");
  display.print(errorRPM, 1);
  display.print("% ");

  if (divergenciaVelocidad) {
    display.print("DIVERG");
  } else {
    display.print("OK");
  }

  // LINEA 6: HC y BT
  display.setCursor(0, 55);
  display.print("HC:");

  if (hcValido) {
    display.print("OK ");
  } else {
    display.print("--- ");
  }

  display.print("BT:CINTA_UTEC");

  display.display();
}

// =====================================================
// TELEMETRIA
// =====================================================

void enviarTelemetria() {

  unsigned long ahora = millis();

  if (ahora - ultimaTelemetria < TELEMETRY_MS) {
    return;
  }

  ultimaTelemetria = ahora;

  String linea = construirTelemetria();

  Serial.println(linea);
  SerialBT.println(linea);
}

// =====================================================
// TELEMETRIA EN FORMATO FACIL DE PARSEAR
// =====================================================

String construirTelemetria() {

  String s = "";

  s += "DIR=";

  if (direccionActual == FORWARD) {
    s += "FWD";
  } else if (direccionActual == REVERSE) {
    s += "REV";
  } else {
    s += "STOP";
  }

  s += ",V=";
  s += velocidadPercent;

  s += ",M=";
  s += microsteps;

  s += ",RPM_M=";
  s += String(rpmMotorComandada, 1);

  s += ",RPM_T=";
  s += String(rpmRodilloTeorica, 1);

  s += ",RPM_R=";
  s += String(rpmRodilloReal, 1);

  s += ",VEL_T=";
  s += String(velocidadTeoricaCmS, 2);

  s += ",VEL_R=";
  s += String(velocidadRealCmS, 2);

  s += ",DIST=";
  s += String(distanciaRawCm, 1);

  s += ",POS=";

  if (hcValido) {
    s += String(posicionObjetoCm, 1);
  } else {
    s += "NA";
  }

  s += ",ERR=";
  s += String(errorRPM, 1);

  s += ",STATE=";

  if (divergenciaVelocidad) {
    s += "DIVERG";
  } else if (!motorEnRegimen && direccionActual != STOPPED) {
    s += "RAMP";
  } else {
    s += "OK";
  }

  return s;
}

// =====================================================
// STATUS MANUAL
// =====================================================

void enviarEstado() {

  String s = construirTelemetria();

  Serial.println(s);
  SerialBT.println(s);
}

// =====================================================
// USB + BLUETOOTH
// =====================================================

void imprimirAmbos(String texto) {

  Serial.println(texto);
  SerialBT.println(texto);
}
