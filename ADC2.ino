/*
// ADCS single-motor module for ESP32-C3
// - QMC5883L (GY-273) on I2C master (Wire1): SCL=0, SDA=1
// - Module as I2C slave to BC on Wire:  SDA=20, SCL=21, SLAVE_ADDR=0x42
// - Motor: DIR=5, PWM=6, ENABLE/START=7
// Protocol (master->module) in comments above

  ORIGINAL:
  https://chatgpt.com/share/6923c33a-4a94-800c-9286-75e241302edb
*/

#include <Wire.h>
#include <math.h>

///// Pinos /////
const int SCL_SENSOR = 0;   // QMC SCL
const int SDA_SENSOR = 1;   // QMC SDA

const int SDA_SLAVE = 20;   // I2C para BC (SDA)
const int SCL_SLAVE = 21;   // I2C para BC (SCL)
const uint8_t SLAVE_ADDR = 0x42; // endereço I2C do módulo (mude se precisar)

#define MOTOR_DIR_PIN    5
#define MOTOR_PWM_PIN    6
#define MOTOR_ENABLE_PIN 7

///// PWM config /////
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 20000;
const int PWM_RES  = 8; // 0..255

///// QMC5883L (GY-273) /////
const uint8_t QMC_ADDR = 0x0D;
float declinationAngle = 0.22; // default (radianos). Ajuste via comando 'P' se quiser.

///// Controle /////
volatile int last_pwm = 0;
const int MAX_ACCELERATION = 50;
int pwm_output = 0;
int max_pwm = 200; // limite de PWM que pode ser alterado via comando 'M'

const float KP = 2.5; // ganho proporcional (tuning necessário)
const float TOLERANCE_DEG = 1.0; // tolerância para considerar "atingido" em graus
const float DRIFT_TOLERANCE_DEG = 2.0; // em modo IDLE, se desviar disso corrige

unsigned long stabilizing_start = 0;
const unsigned long STABILIZING_MS = 800; // espera após atingir alvo

///// Estado /////
enum ModuleState { IDLE = 0, MOVING = 1, STABILIZING = 2, TRACKING = 3 };
volatile ModuleState state = IDLE;

bool target_active = false;
float target_heading_deg = 0.0; // 0..360

// Wire instances: Wire is the slave bus, Wire1 is used as master for sensor
// (on ESP32 Arduino core Wire1 is available)
String debugPrefix = "[ADCS] ";

///// Helpers /////

// normaliza ângulo para [0,360)
float norm360(float a) {
  while (a < 0) a += 360.0;
  while (a >= 360.0) a -= 360.0;
  return a;
}

// menor diferença angular [-180,180)
float shortestAngle(float fromDeg, float toDeg) {
  float d = toDeg - fromDeg;
  while (d <= -180.0) d += 360.0;
  while (d > 180.0) d -= 360.0;
  return d;
}

// aplica limitador de aceleração entre pwm_output e last_pwm
void applyMotorAccelerationLimit() {
  if (pwm_output - last_pwm > MAX_ACCELERATION) {
    pwm_output = last_pwm + MAX_ACCELERATION;
  } else if (pwm_output - last_pwm < -MAX_ACCELERATION) {
    pwm_output = last_pwm - MAX_ACCELERATION;
  }
}

void motorControl(int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);
  if (pwm_val > 0) {
    digitalWrite(MOTOR_DIR_PIN, HIGH);
  } else if (pwm_val < 0) {
    digitalWrite(MOTOR_DIR_PIN, LOW);
  }
  ledcWrite(PWM_CHANNEL, abs(pwm_val));
  if (pwm_val != 0) {
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
  }
  last_pwm = pwm_val;
}

///// QMC functions using Wire1 (master) /////
bool qmcInit() {
  // Reset period reg recommended by datasheet
  Wire1.beginTransmission(QMC_ADDR);
  Wire1.write(0x0B);
  Wire1.write(0x01);
  if (Wire1.endTransmission() != 0) return false;

  // Mode register 0x09 -> 0x1D as in example (continuous, 200Hz, 8G, OSR512)
  Wire1.beginTransmission(QMC_ADDR);
  Wire1.write(0x09);
  Wire1.write(0x1D);
  if (Wire1.endTransmission() != 0) return false;

  delay(10);
  return true;
}

bool readQmcRaw(int16_t &x, int16_t &y, int16_t &z) {
  Wire1.beginTransmission(QMC_ADDR);
  Wire1.write(0x00);
  if (Wire1.endTransmission(false) != 0) return false; // repeated start
  Wire1.requestFrom(QMC_ADDR, (uint8_t)6);
  if (Wire1.available() < 6) return false;
  uint8_t xLSB = Wire1.read();
  uint8_t xMSB = Wire1.read();
  x = (int16_t)((xMSB << 8) | xLSB);
  uint8_t yLSB = Wire1.read();
  uint8_t yMSB = Wire1.read();
  y = (int16_t)((yMSB << 8) | yLSB);
  uint8_t zLSB = Wire1.read();
  uint8_t zMSB = Wire1.read();
  z = (int16_t)((zMSB << 8) | zLSB);
  return true;
}

float readHeadingDeg() {
  int16_t x,y,z;
  if (!readQmcRaw(x,y,z)) {
    // se falhar, retorna último conhecido (ou 0)
    return 0.0;
  }
  // heading = atan2(y,x)
  float heading = atan2((float)y, (float)x);
  heading += declinationAngle;
  if (heading < 0) heading += 2*PI;
  if (heading > 2*PI) heading -= 2*PI;
  float headingDeg = heading * 180.0 / M_PI;
  return headingDeg;
}

///// I2C Slave callbacks /////
// buffer para enviar no onRequest
uint8_t txBuf[3];

void onRequest() {
  // preenche txBuf com status + heading (centi-graus unsigned)
  float cur = readHeadingDeg();
  uint16_t centi = (uint16_t)round(norm360(cur) * 100.0); // 0..35999
  txBuf[0] = (uint8_t)state;
  txBuf[1] = (uint8_t)((centi >> 8) & 0xFF);
  txBuf[2] = (uint8_t)(centi & 0xFF);
  Wire.write(txBuf, 3);
}

void onReceive(int len) {
  if (len < 1) return;
  uint8_t cmd = Wire.read();
  len--;
  if (cmd == 'G') { // GOTO absolute heading (2 bytes unsigned centi-deg)
    if (len >= 2) {
      uint8_t hi = Wire.read();
      uint8_t lo = Wire.read();
      uint16_t centi = ((uint16_t)hi << 8) | lo;
      float deg = centi / 100.0;
      target_heading_deg = norm360(deg);
      target_active = true;
      state = MOVING;
    }
  } else if (cmd == 'D') { // DELTA relative (int16 centi)
    if (len >= 2) {
      int8_t b1 = Wire.read(); int8_t b2 = Wire.read();
      int16_t s = (int16_t)(((uint8_t)b1 << 8) | (uint8_t)b2);
      float delta = ((float)s) / 100.0;
      // compute new target from current heading
      float cur = readHeadingDeg();
      target_heading_deg = norm360(cur + delta);
      target_active = true;
      state = MOVING;
    }
  } else if (cmd == 'S') { // STOP
    target_active = false;
    pwm_output = 0;
    motorControl(0);
    state = IDLE;
  } else if (cmd == 'P') { // set declination (signed int16 centi-radians)
    if (len >= 2) {
      int8_t b1 = Wire.read(); int8_t b2 = Wire.read();
      int16_t s = (int16_t)(((uint8_t)b1 << 8) | (uint8_t)b2);
      declinationAngle = ((float)s) / 100.0; // in radians
    }
  } else if (cmd == 'M') { // set max PWM (1 byte)
    if (len >= 1) {
      uint8_t m = Wire.read();
      max_pwm = constrain((int)m, 0, 255);
    }
  } else if (cmd == '?') { // pedido de status — nada a fazer, master vai ler
    // do nothing; onRequest responde.
  } else {
    // comando desconhecido — ignore
  }
}

///// Setup e loop /////
void setupMotor() {
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(debugPrefix + "Inicializando ADCS module...");

  // I2C SENSOR (Wire1) - master para QMC
  Wire1.begin(SDA_SENSOR, SCL_SENSOR); // SDA, SCL
  if (!qmcInit()) {
    Serial.println(debugPrefix + "Falha ao configurar QMC5883L (verifique conexões).");
  } else {
    Serial.println(debugPrefix + "QMC5883L inicializado.");
  }

  // I2C SLAVE para BC (Wire) - fica em endereço SLAVE_ADDR
  Wire.begin(SLAVE_ADDR, SDA_SLAVE, SCL_SLAVE);
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);
  Serial.println(debugPrefix + "I2C slave iniciado no endereco 0x" + String(SLAVE_ADDR, HEX));

  setupMotor();

  state = IDLE;
  target_active = false;
  pwm_output = 0;
  last_pwm = 0;
}

unsigned long lastLoop = 0;
const unsigned long LOOP_MS = 120; // taxa de controle ~8.3 Hz

void loop() {
  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  lastLoop = now;

  float heading = readHeadingDeg(); // graus

  // Estado / lógica
  if (target_active) {
    float err = shortestAngle(heading, target_heading_deg); // sinal + para girar "para direita" (convenção)
    float absErr = fabs(err);
    if (absErr <= TOLERANCE_DEG) {
      // atingiu alvo
      pwm_output = 0;
      motorControl(0);
      target_active = false;
      state = STABILIZING;
      stabilizing_start = now;
      Serial.println(debugPrefix + "Alvo atingido: " + String(heading, 2) + " deg");
    } else {
      // controlador proporcional simples
      float p = KP * err; // err em graus
      int pwm = (int)round(p);
      // limita ao max_pwm
      pwm = constrain(pwm, -max_pwm, max_pwm);
      pwm_output = pwm;
      applyMotorAccelerationLimit();
      motorControl(pwm_output);
      state = MOVING;
      // Serial.print debug
      Serial.print(debugPrefix + "Movendo. Heading: ");
      Serial.print(heading); Serial.print(" target: "); Serial.print(target_heading_deg);
      Serial.print(" err: "); Serial.print(err);
      Serial.print(" pwm: "); Serial.println(pwm_output);
    }
  } else {
    // não há alvo. Verifica deriva e corrige (modo TRACKING) se necessário
    // assumimos que "posição desejada em idle" é a última heading mantida; para simplificar:
    static float hold_heading = 0.0;
    static bool hold_initialized = false;
    if (!hold_initialized) {
      hold_heading = heading;
      hold_initialized = true;
    }
    float dErr = shortestAngle(heading, hold_heading);
    if (fabs(dErr) > DRIFT_TOLERANCE_DEG) {
      // corrige
      float p = KP * dErr;
      int pwm = (int)round(p);
      pwm = constrain(pwm, -max_pwm, max_pwm);
      pwm_output = pwm;
      applyMotorAccelerationLimit();
      motorControl(pwm_output);
      state = TRACKING;
      Serial.print(debugPrefix + "Tracking (corrigindo deriva). heading: ");
      Serial.print(heading); Serial.print(" hold: "); Serial.print(hold_heading);
      Serial.print(" derr: "); Serial.print(dErr);
      Serial.print(" pwm: "); Serial.println(pwm_output);
    } else {
      // mantém zero
      pwm_output = 0;
      motorControl(0);
      state = IDLE;
      // atualiza hold_heading para acompanhar pequenas mudanças quando parar
      hold_heading = heading;
    }
  }

  // handle stabilizing timeout
  if (state == STABILIZING) {
    if (millis() - stabilizing_start >= STABILIZING_MS) {
      state = IDLE;
      Serial.println(debugPrefix + "Stabilizing complete -> IDLE");
    }
  }
}
