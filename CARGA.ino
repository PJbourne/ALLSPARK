/*
  Módulo de Carga - ESP32-C3
  - ADC pinos: 0 (VBMS), 1 (V_AUX), 4 (TEMP)
  - Heater / Booster: pin 10 (digital)
  - I2C slave: SDA=20, SCL=21
  - Logging: LittleFS -> /log.csv, append a cada minuto em modo CONTINUOUS
  - Protocolos e comandos documentados na resposta associada.

  ORIGINAL:
  https://chatgpt.com/share/6923c33a-4a94-800c-9286-75e241302edb
*/

#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>

// -------------------- PINOUT --------------------
const int PIN_VBMS = 0;   // entrada ADC (BMS)
const int PIN_VAUX = 1;   // entrada ADC (aux)
const int PIN_TEMP = 4;   // entrada ADC (termistor divider 10k/10k)
const int PIN_HEATER = 10; // saída digital para booster/aquecedor

const int SDA_SLAVE = 20;
const int SCL_SLAVE = 21;
const uint8_t SLAVE_ADDR = 0x43; // endereço I2C do módulo (mude se quiser)

// -------------------- ADC / Conversão --------------------
// Se necessário ajuste ADC_REF_VOLT e ADC_RESOLUTION para sua placa/attenuation
const float ADC_REF_VOLT = 3.3;   // Vref
const int ADC_RESOLUTION = 4095;  // 12-bit
// fator por contagem
const float ADC_LSB = ADC_REF_VOLT / float(ADC_RESOLUTION);

// Divider VBMS: se seu divisor for Rtop / Rbot (ex: 30k cima / 7.5k baixo), a tensão da bateria
// é Vmeas * (Rtop+Rbot)/Rbot
// Ajuste conforme seu divisor real. Exemplo para Rtop=30k, Rbot=7.5k => ratio = (30+7.5)/7.5 = 5.0
const float VBMS_R_TOP_K = 30.0f;  // kOhm
const float VBMS_R_BOT_K = 7.5f;   // kOhm
const float VBMS_DIVIDER_RATIO = (VBMS_R_TOP_K + VBMS_R_BOT_K) / VBMS_R_BOT_K;

// Thermistor (10k divider 10k/10k): we assume NTC 10k, B=3950, R25=10k
const float THERM_R_TOP = 10000.0f; // fixed resistor (ohm) — a parte fixa do divisor
const float NTC_R25 = 10000.0f;
const float NTC_B = 3950.0f;

// -------------------- Thresholds / Defaults --------------------
int16_t temp_threshold_centi = 500;    // 5.00°C (centi°C)
uint16_t vbms_cutoff_mV = 9300;        // 9.3 V (mV)
uint16_t charging_delta_mV = 100;      // 0.1 V difference -> charging
int max_log_lines = 10000;

// Heater mode
// 0 = forced OFF, 1 = forced ON, 2 = AUTO
volatile uint8_t heater_mode = 2;

// MODULE STATE
enum Mode { MODE_IDLE = 0, MODE_CONTINUOUS = 1, MODE_PACKET = 2 };
volatile Mode currentMode = MODE_IDLE;

// Sampling / logging
const unsigned long SAMPLE_INTERVAL_MS = 60UL * 1000UL; // 60 seconds
unsigned long last_sample_ts = 0;

// Log file
const char *LOG_PATH = "/log.csv";
File logFile;
File streamFile;
bool streamingLogs = false;

// I2C send buffer for onRequest
#define I2C_TX_BUF_MAX 32
uint8_t i2c_tx_buf[I2C_TX_BUF_MAX];
size_t i2c_tx_len = 0;

// state flags
bool heating_on = false;
bool charging_flag = false;
bool heater_forced = false;

// -------------------- Helpers --------------------
float adcToVoltage(int raw) {
  return raw * ADC_LSB;
}

uint16_t readVin_mV(int pin) {
  int raw = analogRead(pin);
  float vmeas = adcToVoltage(raw);
  // scale for divider if pin is VBMS or VAUX else return measured
  if (pin == PIN_VBMS || pin == PIN_VAUX) {
    float vbat = vmeas * VBMS_DIVIDER_RATIO;
    return (uint16_t)round(vbat * 1000.0f);
  } else {
    return (uint16_t)round(vmeas * 1000.0f);
  }
}

// converte leitura do termistor (divisor 10k/10k) -> temperatura Celsius
int16_t readTemp_centiC(int pin) {
  int raw = analogRead(pin);
  float vmeas = adcToVoltage(raw);
  // divisor 10k top + 10k bottom => Vout = Vref * (Rbottom / (Rtop+Rbottom)) * (Rt / (Rt + Rbottom)) ???  
  // Simples abordagem: a top resistor é THERM_R_TOP; medimos vout, calculamos Rt:
  // Vout = Vref * (Rt / (Rt + R_fixed))  -> Rt = R_fixed * Vout / (Vref - Vout)
  float vout = vmeas;
  if (vout <= 0.0001f) return (int16_t)(-1000); // out of range
  float rt = THERM_R_TOP * vout / (ADC_REF_VOLT - vout);
  // Steinhart-Hart (B-parameter)
  float tK = 1.0f / ( (1.0f/(25.0f+273.15f)) + (1.0f/NTC_B) * log(rt / NTC_R25) );
  float tC = tK - 273.15f;
  return (int16_t)round(tC * 100.0f);
}

// safe set heater output
void setHeaterOutput(bool on) {
  digitalWrite(PIN_HEATER, on ? HIGH : LOW);
  heating_on = on;
}

// prepare status packet (8 bytes) for onRequest
void prepareStatusPacket() {
  float tmpf = 0.0f;
  int16_t tempCent = readTemp_centiC(PIN_TEMP);
  uint16_t vbms = readVin_mV(PIN_VBMS);
  uint16_t vaux = readVin_mV(PIN_VAUX);
  uint8_t flags = 0;
  if (charging_flag) flags |= 0x01;
  if (heating_on) flags |= 0x02;
  if (heater_mode != 2) flags |= 0x04;

  i2c_tx_len = 8;
  i2c_tx_buf[0] = (uint8_t)currentMode;
  i2c_tx_buf[1] = (uint8_t)((tempCent >> 8) & 0xFF);
  i2c_tx_buf[2] = (uint8_t)(tempCent & 0xFF);
  i2c_tx_buf[3] = (uint8_t)((vbms >> 8) & 0xFF);
  i2c_tx_buf[4] = (uint8_t)(vbms & 0xFF);
  i2c_tx_buf[5] = flags;
  i2c_tx_buf[6] = (uint8_t)((vaux >> 8) & 0xFF);
  i2c_tx_buf[7] = (uint8_t)(vaux & 0xFF);
}

// -------------------- I2C handlers --------------------
void onRequest() {
  // If streaming logs, send next chunk
  if (streamingLogs && streamFile) {
    // read up to I2C_TX_BUF_MAX bytes
    size_t toRead = I2C_TX_BUF_MAX;
    if (toRead > streamFile.available()) toRead = streamFile.available();
    if (toRead == 0) {
      // End of file -> send zero bytes (Wire.write with len=0)
      streamFile.close();
      streamingLogs = false;
      i2c_tx_len = 0;
      return;
    }
    // read chunk
    uint8_t chunk[I2C_TX_BUF_MAX];
    size_t r = streamFile.read(chunk, toRead);
    Wire.write(chunk, r);
    return;
  }

  // Default: send status packet
  prepareStatusPacket();
  Wire.write(i2c_tx_buf, i2c_tx_len);
}

void onReceive(int len) {
  if (len < 1) return;
  uint8_t cmd = Wire.read();
  len--;
  if (cmd == 'm') { // set mode
    if (len >= 1) {
      uint8_t md = Wire.read();
      if (md <= 2) currentMode = (Mode)md;
    }
  } else if (cmd == 'r') {
    // read now -> prepare status to be read by master (nothing more to do here)
    // onRequest will handle and take fresh readings.
  } else if (cmd == 'h') { // heater override
    if (len >= 1) {
      uint8_t hv = Wire.read();
      if (hv == 0) {
        heater_mode = 0; heater_forced = true;
        setHeaterOutput(false);
      } else if (hv == 1) {
        heater_mode = 1; heater_forced = true;
        setHeaterOutput(true);
      } else {
        heater_mode = 2; heater_forced = false; // AUTO
      }
    }
  } else if (cmd == 't') { // set temp threshold (int16 centiC)
    if (len >= 2) {
      uint8_t hi = Wire.read(); uint8_t lo = Wire.read();
      int16_t val = (int16_t)((hi<<8) | lo);
      temp_threshold_centi = val;
    }
  } else if (cmd == 'v') { // set vbms cutoff (uint16 mV)
    if (len >= 2) {
      uint8_t hi = Wire.read(); uint8_t lo = Wire.read();
      uint16_t val = (uint16_t)((hi<<8) | lo);
      vbms_cutoff_mV = val;
    }
  } else if (cmd == 'c') { // set charging delta (uint16 mV)
    if (len >= 2) {
      uint8_t hi = Wire.read(); uint8_t lo = Wire.read();
      uint16_t val = (uint16_t)((hi<<8) | lo);
      charging_delta_mV = val;
    }
  } else if (cmd == 'l') { // start log streaming
    if (LittleFS.exists(LOG_PATH)) {
      streamFile = LittleFS.open(LOG_PATH, "r");
      if (streamFile) {
        streamingLogs = true;
      } else {
        streamingLogs = false;
      }
    } else {
      streamingLogs = false;
    }
  } else if (cmd == 'C') { // clear logs
    if (LittleFS.exists(LOG_PATH)) {
      LittleFS.remove(LOG_PATH);
    }
  } else if (cmd == '?') {
    // status requested; nothing to do here, master will request bytes
  } else {
    // comando desconhecido
  }
}

// -------------------- Logging --------------------
void appendLogLine(uint32_t timestamp_ms, int16_t tempCent, uint16_t vbms_mV, uint16_t vaux_mV, uint8_t flags) {
  File f = LittleFS.open(LOG_PATH, "a");
  if (!f) return;
  // forma: ts,tempCent,vbms_mV,vaux_mV,flags\n
  f.printf("%u,%d,%u,%u,%u\n", timestamp_ms, tempCent, vbms_mV, vaux_mV, flags);
  f.close();
}

// -------------------- Setup & Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("Modulo de Carga - Inicializando...");

  // init LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
  } else {
    Serial.println("LittleFS ok.");
    // create file if not exists
    if (!LittleFS.exists(LOG_PATH)) {
      File f = LittleFS.open(LOG_PATH, "w");
      if (f) { f.println("ts_ms,temp_centi,vbms_mV,vaux_mV,flags"); f.close(); }
    }
  }

  // pins
  pinMode(PIN_HEATER, OUTPUT);
  setHeaterOutput(false);

  // ADC defaults
#if defined(ARDUINO_ARCH_RISCV) || defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(12); // 0..4095
#endif

  // I2C slave
  Wire.begin(SLAVE_ADDR, SDA_SLAVE, SCL_SLAVE);
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);
  Serial.printf("I2C Slave iniciado em 0x%02X\n", SLAVE_ADDR);

  last_sample_ts = millis();
}

void loop() {
  unsigned long now = millis();

  // Amostragem periódica em modo CONTINUOUS
  if (currentMode == MODE_CONTINUOUS && (now - last_sample_ts >= SAMPLE_INTERVAL_MS)) {
    last_sample_ts = now;
    // leitura
    int16_t tmpCent = readTemp_centiC(PIN_TEMP);
    uint16_t vbms = readVin_mV(PIN_VBMS);
    uint16_t vaux = readVin_mV(PIN_VAUX);

    // flags
    uint8_t flags = 0;
    // charging detection
    int16_t diff = (int16_t)vbms - (int16_t)vaux; // VBMS - Vaux
    charging_flag = (abs(diff) >= (int)charging_delta_mV);
    if (charging_flag) flags |= 0x01;

    // heater logic
    bool shouldHeat = false;
    if (heater_mode == 0) {
      shouldHeat = false; heater_forced = true;
    } else if (heater_mode == 1) {
      shouldHeat = true; heater_forced = true;
    } else {
      // AUTO
      heater_forced = false;
      // only heat if temp below threshold AND vbms >= cutoff AND not charging (optional)
      if (tmpCent < temp_threshold_centi && vbms >= vbms_cutoff_mV) {
        shouldHeat = true;
      } else {
        shouldHeat = false;
      }
    }
    setHeaterOutput(shouldHeat);
    if (heating_on) flags |= 0x02;
    if (heater_forced) flags |= 0x04;

    // append to log
    appendLogLine((uint32_t)now, tmpCent, vbms, vaux, flags);

    // debug
    Serial.printf("LOG: t=%d.%02dC vbms=%u mV vaux=%u mV charge=%d heat=%d\n",
                  tmpCent/100, abs(tmpCent%100), vbms, vaux, charging_flag ? 1:0, heating_on ? 1:0);
  }

  // if not continuous, still optionally maintain heater auto behaviour in IDLE and PACKET
  if (currentMode != MODE_CONTINUOUS) {
    // evaluate heater auto (so it still protects module)
    if (heater_mode == 2) {
      int16_t tmpCent = readTemp_centiC(PIN_TEMP);
      uint16_t vbms = readVin_mV(PIN_VBMS);
      bool shouldHeat = (tmpCent < temp_threshold_centi && vbms >= vbms_cutoff_mV);
      setHeaterOutput(shouldHeat);
    } else if (heater_mode == 0) {
      setHeaterOutput(false);
    } else if (heater_mode == 1) {
      setHeaterOutput(true);
    }
    // update charging flag
    uint16_t vbms = readVin_mV(PIN_VBMS);
    uint16_t vaux = readVin_mV(PIN_VAUX);
    int16_t diff = (int16_t)vbms - (int16_t)vaux;
    charging_flag = (abs(diff) >= (int)charging_delta_mV);
  }

  // short delay to avoid busy loop
  delay(10);
}
