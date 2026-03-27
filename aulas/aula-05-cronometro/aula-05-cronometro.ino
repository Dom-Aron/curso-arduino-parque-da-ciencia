struct SensorEvent;
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LCD.h>
#include <stdio.h>
#include <avr/interrupt.h>

// ─────────────────────────────────────────────────────────────────────────────
// 1) LCD (I2C)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t kLcdAddr = 0x27;
static constexpr uint8_t kEn = 2;
static constexpr uint8_t kRw = 1;
static constexpr uint8_t kRs = 0;
static constexpr uint8_t kD4 = 4;
static constexpr uint8_t kD5 = 5;
static constexpr uint8_t kD6 = 6;
static constexpr uint8_t kD7 = 7;
static constexpr uint8_t kBacklightPin = 3;
static constexpr t_backlightPol kBacklightPolarity = POSITIVE;

LiquidCrystal_I2C lcd(
  kLcdAddr, kEn, kRw, kRs, kD4, kD5, kD6, kD7, kBacklightPin, kBacklightPolarity
);

// ─────────────────────────────────────────────────────────────────────────────
// 2) PINOS
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t kPinBtnControl  = 2;

// Sensores: fototransistor + resistor 10k (externo) + fio longo
static constexpr uint8_t kPinSensor0 = 5; // start/resume (também é o 1º gate do experimento)
static constexpr uint8_t kPinSensor1 = 6; // gate 2
static constexpr uint8_t kPinSensor2 = 7; // gate 3
static constexpr uint8_t kPinSensor3 = 8; // gate 4

static constexpr uint8_t kSensorCount = 4; // 0..3

// ─────────────────────────────────────────────────────────────────────────────
// 3) PARÂMETROS
// ─────────────────────────────────────────────────────────────────────────────
// Botão (mecânico)
static constexpr uint16_t kDebounceButtonMs   = 25;
static constexpr uint16_t kLongPressMs        = 1200;
static constexpr uint16_t kMultiClickGapMs    = 350;

// LCD: agora bem mais “leve” (pouca escrita)
static constexpr uint16_t kLcdUpdateIntervalMs = 200; // só verifica ~5 Hz (cache evita escrita)

// Popups pós-medida no LCD
static constexpr uint16_t kFinalPopupCycleMs   = 1500;

// Voltas
static constexpr uint8_t kMaxLaps = 20;

// Sensores: filtro anti-ruído e rearm (em micros)
static constexpr uint16_t kMinActivePulseUs = 150;    // rejeita spikes (ajuste prático)
static constexpr uint32_t kSensorRearmUs    = 25000;  // 25 ms (evita dupla marcação)

// Sensores ativos em LOW (HIGH->LOW inicia o pulso)
static constexpr bool kSensorActiveLow = true;

// Serial
static constexpr uint32_t kSerialBaud = 115200;

// Experimento (queda livre): distância entre sensores (m)
static constexpr float kSensorSpacingM = 0.60f;

// ─────────────────────────────────────────────────────────────────────────────
// 4) DEBOUNCE DO BOTÃO
// ─────────────────────────────────────────────────────────────────────────────
struct DebouncedInput {
  uint8_t pin = 255;
  uint16_t debounceMs = 0;

  bool stableState = true;
  bool lastRawState = true;
  uint32_t lastChangeMs = 0;

  bool fellEvent = false;
  bool roseEvent = false;

  void begin(uint8_t p, uint16_t dbMs) {
    pin = p;
    debounceMs = dbMs;

    const bool initial = (digitalRead(pin) == HIGH);
    stableState = initial;
    lastRawState = initial;
    lastChangeMs = millis();

    fellEvent = false;
    roseEvent = false;
  }

  void update(uint32_t nowMs) {
    const bool raw = (digitalRead(pin) == HIGH);

    if (raw != lastRawState) {
      lastRawState = raw;
      lastChangeMs = nowMs;
    }

    if ((nowMs - lastChangeMs) >= debounceMs && stableState != raw) {
      const bool prev = stableState;
      stableState = raw;

      if (prev == true && stableState == false) fellEvent = true;
      if (prev == false && stableState == true) roseEvent = true;
    }
  }

  bool fell() {
    const bool e = fellEvent;
    fellEvent = false;
    return e;
  }

  bool rose() {
    const bool e = roseEvent;
    roseEvent = false;
    return e;
  }

  bool isPressed() const {
    return (stableState == false); // INPUT_PULLUP => LOW = pressionado
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// 5) TIMER1 TIMEBASE (substitui micros() dentro de ISR)
// ─────────────────────────────────────────────────────────────────────────────
// Arduino Uno / ATmega328P:
// - Timer1 livre com prescaler=8 (2 MHz) => tick = 0.5 us
// - Timestamp retornado em microsegundos (uint32_t), wrap ~71 min
// - Seguro para ser chamado dentro das ISRs de PCINT
//
// NOTA: isso usa o Timer1, então:
// - PWM nos pinos 9 e 10 é afetado
// - Biblioteca Servo (que usa Timer1) entra em conflito
volatile uint32_t gTimer1Overflows = 0;

ISR(TIMER1_OVF_vect) {
  gTimer1Overflows++;
}

static void initTimer1Timebase() {
  noInterrupts();

  // Para o timer
  TCCR1A = 0;
  TCCR1B = 0;

  // Zera contador e contador de overflow
  TCNT1 = 0;
  gTimer1Overflows = 0;

  // Limpa flag pendente de overflow
  TIFR1 |= _BV(TOV1);

  // Habilita interrupção de overflow
  TIMSK1 |= _BV(TOIE1);

  // Inicia com prescaler = 8 (CS11=1)
  // F_CPU=16 MHz -> 2 MHz no timer -> 0.5 us por tick
  TCCR1B |= _BV(CS11);

  interrupts();
}

// Leitura em contexto normal (fora de ISR)
static inline uint32_t timer1Micros() {
  uint16_t t1;
  uint32_t ovf;
  const uint8_t sreg = SREG;

  cli();
  t1 = TCNT1;
  ovf = gTimer1Overflows;

  // Se ocorreu overflow mas a ISR ainda não rodou, contabiliza
  if ((TIFR1 & _BV(TOV1)) && t1 < 0x8000) {
    ovf++;
  }
  SREG = sreg;

  const uint64_t ticks = ((uint64_t)ovf << 16) | (uint64_t)t1; // ticks de 0.5 us
  const uint64_t us = ticks >> 1; // /2 => microsegundos inteiros
  return (uint32_t)us;
}

// Leitura dentro de ISR (interrupções já desabilitadas)
static inline uint32_t timer1MicrosIsr() {
  const uint16_t t1 = TCNT1;
  uint32_t ovf = gTimer1Overflows;

  if ((TIFR1 & _BV(TOV1)) && t1 < 0x8000) {
    ovf++;
  }

  const uint64_t ticks = ((uint64_t)ovf << 16) | (uint64_t)t1;
  const uint64_t us = ticks >> 1;
  return (uint32_t)us;
}

// ─────────────────────────────────────────────────────────────────────────────
// 6) EVENTOS DOS SENSORES via PCINT (ISR) + FILTRO DE PULSO
// ─────────────────────────────────────────────────────────────────────────────
struct SensorEvent {
  uint8_t sensor_id;   // 0..3
  uint32_t t_us;       // timestamp do INÍCIO do pulso (queda)
};

static constexpr uint8_t kEventBufSize = 16;
volatile SensorEvent gEventBuf[kEventBufSize];
volatile uint8_t gEventHead = 0;
volatile uint8_t gEventTail = 0;

volatile uint32_t gFallUs[kSensorCount] = {0, 0, 0, 0};
volatile bool gSawFall[kSensorCount] = {false, false, false, false};
volatile uint32_t gLastAcceptedUs[kSensorCount] = {0, 0, 0, 0};

volatile uint8_t gPrevPIND = 0;
volatile uint8_t gPrevPINB = 0;

static inline uint32_t usDiff(uint32_t a, uint32_t b) {
  return (uint32_t)(a - b);
}

static inline void pushEventIsr(uint8_t sensorId, uint32_t tUs) {
  const uint8_t next = (uint8_t)((gEventHead + 1) % kEventBufSize);
  if (next == gEventTail) {
    // buffer cheio (muito improvável)
    return;
  }
  gEventBuf[gEventHead].sensor_id = sensorId;
  gEventBuf[gEventHead].t_us = tUs;
  gEventHead = next;
}

static void handleSensorEdgeIsr(uint8_t sensorId, bool pinIsHigh, uint32_t nowUs) {
  if (!kSensorActiveLow) return;

  if (!pinIsHigh) {
    // HIGH->LOW: início do pulso
    gFallUs[sensorId] = nowUs;
    gSawFall[sensorId] = true;
    return;
  }

  // LOW->HIGH: fim do pulso
  if (!gSawFall[sensorId]) return;

  const uint32_t fall = gFallUs[sensorId];
  gSawFall[sensorId] = false;

  const uint32_t width = usDiff(nowUs, fall);
  if (width < kMinActivePulseUs) return;

  const uint32_t sinceLast = usDiff(fall, gLastAcceptedUs[sensorId]);
  if (sinceLast < kSensorRearmUs) return;

  gLastAcceptedUs[sensorId] = fall;
  pushEventIsr(sensorId, fall);
}

ISR(PCINT2_vect) { // PORTD -> pinos 5,6,7
  const uint32_t nowUs = timer1MicrosIsr();
  const uint8_t cur = PIND;
  const uint8_t changed = (uint8_t)(cur ^ gPrevPIND);
  gPrevPIND = cur;

  // PD5 (pino 5) -> sensor0
  if (changed & _BV(5)) {
    const bool isHigh = (cur & _BV(5)) != 0;
    handleSensorEdgeIsr(0, isHigh, nowUs);
  }
  // PD6 (pino 6) -> sensor1
  if (changed & _BV(6)) {
    const bool isHigh = (cur & _BV(6)) != 0;
    handleSensorEdgeIsr(1, isHigh, nowUs);
  }
  // PD7 (pino 7) -> sensor2
  if (changed & _BV(7)) {
    const bool isHigh = (cur & _BV(7)) != 0;
    handleSensorEdgeIsr(2, isHigh, nowUs);
  }
}

ISR(PCINT0_vect) { // PORTB -> pino 8 (PB0)
  const uint32_t nowUs = timer1MicrosIsr();
  const uint8_t cur = PINB;
  const uint8_t changed = (uint8_t)(cur ^ gPrevPINB);
  gPrevPINB = cur;

  if (changed & _BV(0)) {
    const bool isHigh = (cur & _BV(0)) != 0;
    handleSensorEdgeIsr(3, isHigh, nowUs);
  }
}

bool popEvent(SensorEvent& out) {
  noInterrupts();
  if (gEventTail == gEventHead) {
    interrupts();
    return false;
  }
  out.sensor_id = gEventBuf[gEventTail].sensor_id;
  out.t_us = gEventBuf[gEventTail].t_us;
  gEventTail = (uint8_t)((gEventTail + 1) % kEventBufSize);
  interrupts();
  return true;
}

static void initPcintForSensors() {
  noInterrupts();

  gPrevPIND = PIND;
  gPrevPINB = PINB;

  PCICR |= (1 << PCIE2) | (1 << PCIE0);

  // PD5, PD6, PD7 -> PCINT21,22,23
  PCMSK2 |= (1 << PCINT21) | (1 << PCINT22) | (1 << PCINT23);

  // PB0 (pino 8) -> PCINT0
  PCMSK0 |= (1 << PCINT0);

  interrupts();
}

// ─────────────────────────────────────────────────────────────────────────────
// 7) CRONÔMETRO (Timer1 micros) + VOLTAS
// ─────────────────────────────────────────────────────────────────────────────
enum class RunState : uint8_t {
  IDLE = 0,
  RUNNING,
  PAUSED,
  FINISHED
};

static RunState gState = RunState::IDLE;

static uint32_t gStartUs = 0;
static uint32_t gElapsedUs = 0;

static uint32_t gLapUs[kMaxLaps];
static uint8_t gLapCount = 0;

static uint8_t gFinalLapIndex = 0;
static uint32_t gNextFinalPopupMs = 0;

static bool gSerialSummaryPrinted = false;

// ─────────────────────────────────────────────────────────────────────────────
// 8) EXPERIMENTO: captura 4 sensores e calcula velocidades + g por regressão
// ─────────────────────────────────────────────────────────────────────────────
static bool gExpSeen[kSensorCount] = {false, false, false, false};
static uint32_t gExpTimeUs[kSensorCount] = {0, 0, 0, 0};
static bool gExpPrinted = false;

static void resetExperimentState() {
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    gExpSeen[i] = false;
    gExpTimeUs[i] = 0;
  }
  gExpPrinted = false;
}

static bool experimentComplete() {
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    if (!gExpSeen[i]) return false;
  }
  return true;
}

static void registerExperimentSensor(uint8_t sensorId, uint32_t tUs) {
  if (sensorId >= kSensorCount) return;
  if (!gExpSeen[sensorId]) {
    gExpSeen[sensorId] = true;
    gExpTimeUs[sensorId] = tUs;
  }
}

static void printExperimentResultsOnceToSerial() {
  if (gExpPrinted) return;
  if (!experimentComplete()) return;
  gExpPrinted = true;

  const uint32_t t0Us = gExpTimeUs[0];

  // Tempos relativos em segundos
  float tS[kSensorCount];
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    const uint32_t dtUs = (uint32_t)(gExpTimeUs[i] - t0Us);
    tS[i] = (float)dtUs * 1e-6f;
  }

  // Checagem simples de monotonicidade (esperado: t0 < t1 < t2 < t3)
  bool monotonic = true;
  for (uint8_t i = 1; i < kSensorCount; ++i) {
    if (gExpTimeUs[i] <= gExpTimeUs[i - 1]) { monotonic = false; break; }
  }

  Serial.println();
  Serial.println("=== EXPERIMENTO QUEDA LIVRE ===");
  Serial.print("SPACING_M,"); Serial.println(kSensorSpacingM, 3);

  Serial.print("S0_US,"); Serial.println(gExpTimeUs[0]);
  Serial.print("S1_US,"); Serial.println(gExpTimeUs[1]);
  Serial.print("S2_US,"); Serial.println(gExpTimeUs[2]);
  Serial.print("S3_US,"); Serial.println(gExpTimeUs[3]);

  Serial.print("T0_S,"); Serial.println(tS[0], 6);
  Serial.print("T1_S,"); Serial.println(tS[1], 6);
  Serial.print("T2_S,"); Serial.println(tS[2], 6);
  Serial.print("T3_S,"); Serial.println(tS[3], 6);

  if (!monotonic) {
    Serial.println("WARN,Non-monotonic sensor timestamps (check wiring/noise/order)");
  }

  // Velocidades médias por trecho (m/s)
  // v01 associada ao gate S1, v12 ao S2, v23 ao S3
  const float dt01 = (float)(gExpTimeUs[1] - gExpTimeUs[0]) * 1e-6f;
  const float dt12 = (float)(gExpTimeUs[2] - gExpTimeUs[1]) * 1e-6f;
  const float dt23 = (float)(gExpTimeUs[3] - gExpTimeUs[2]) * 1e-6f;

  if (dt01 > 0) {
    const float v01 = kSensorSpacingM / dt01;
    Serial.print("DT01_S,"); Serial.print(dt01, 6);
    Serial.print(",V01_MPS,"); Serial.println(v01, 6);
  } else {
    Serial.println("DT01_S,0,V01_MPS,NaN");
  }

  if (dt12 > 0) {
    const float v12 = kSensorSpacingM / dt12;
    Serial.print("DT12_S,"); Serial.print(dt12, 6);
    Serial.print(",V12_MPS,"); Serial.println(v12, 6);
  } else {
    Serial.println("DT12_S,0,V12_MPS,NaN");
  }

  if (dt23 > 0) {
    const float v23 = kSensorSpacingM / dt23;
    Serial.print("DT23_S,"); Serial.print(dt23, 6);
    Serial.print(",V23_MPS,"); Serial.println(v23, 6);
  } else {
    Serial.println("DT23_S,0,V23_MPS,NaN");
  }

  // Regressão linear: y = a + b * (t^2)
  // y em metros: [0, 0.6, 1.2, 1.8], t relativo ao S0
  const uint8_t n = kSensorCount;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumXX = 0.0f;
  float sumXY = 0.0f;

  float x[n];
  float y[n];

  for (uint8_t i = 0; i < n; ++i) {
    x[i] = tS[i] * tS[i];
    y[i] = (float)i * kSensorSpacingM;

    sumX += x[i];
    sumY += y[i];
    sumXX += x[i] * x[i];
    sumXY += x[i] * y[i];
  }

  const float denom = (float)n * sumXX - (sumX * sumX);

  if (denom != 0.0f) {
    const float b = ((float)n * sumXY - (sumX * sumY)) / denom;
    const float a = (sumY - b * sumX) / (float)n;
    const float g = 2.0f * b;

    // R^2
    const float yMean = sumY / (float)n;
    float ssTot = 0.0f;
    float ssRes = 0.0f;

    for (uint8_t i = 0; i < n; ++i) {
      const float yHat = a + b * x[i];
      const float dy = y[i] - yMean;
      const float er = y[i] - yHat;
      ssTot += dy * dy;
      ssRes += er * er;
    }

    float r2 = 0.0f;
    if (ssTot > 0.0f) r2 = 1.0f - (ssRes / ssTot);

    Serial.print("REG_A_M,"); Serial.print(a, 6);
    Serial.print(",REG_B_M_PER_S2,"); Serial.print(b, 6);
    Serial.print(",G_MPS2,"); Serial.print(g, 6);
    Serial.print(",R2,"); Serial.println(r2, 6);
  } else {
    Serial.println("REG_ERROR,Denominator=0 (insufficient variation in t^2)");
  }

  Serial.println("===========================");
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// 9) BOTÃO (1/2/3 cliques + longo)
// ─────────────────────────────────────────────────────────────────────────────
static DebouncedInput inBtnControl;
static uint32_t gBtnPressStartMs = 0;
static bool gLongPressHandled = false;
static uint8_t gPendingClickCount = 0;
static uint32_t gClickSequenceDeadlineMs = 0;

// ─────────────────────────────────────────────────────────────────────────────
// 10) LCD cache (escreve só se mudou)
// ─────────────────────────────────────────────────────────────────────────────
static char gLcdCache0[17] = {0};
static char gLcdCache1[17] = {0};
static uint32_t gLastLcdUpdateMs = 0;

static void pad16(const char* src, char out16[17]) {
  for (uint8_t i = 0; i < 16; ++i) out16[i] = ' ';
  out16[16] = '\0';
  uint8_t i = 0;
  while (i < 16 && src[i] != '\0') { out16[i] = src[i]; ++i; }
}

static void lcdWriteLineCached(uint8_t row, const char* text) {
  char buf[17];
  pad16(text, buf);

  char* cache = (row == 0) ? gLcdCache0 : gLcdCache1;

  bool same = true;
  for (uint8_t i = 0; i < 16; ++i) {
    if (cache[i] != buf[i]) { same = false; break; }
  }
  if (same) return;

  lcd.setCursor(0, row);
  lcd.print(buf);

  for (uint8_t i = 0; i < 16; ++i) cache[i] = buf[i];
  cache[16] = '\0';
}

// ─────────────────────────────────────────────────────────────────────────────
// 11) Formatação de tempo e UI
// ─────────────────────────────────────────────────────────────────────────────
static void formatTimeUsToMsString(uint32_t us, char* out, size_t outLen) {
  // Exibe MM:SS (sem ms) para reduzir mudanças no LCD durante RUNNING
  const uint32_t ms = us / 1000UL;
  const uint16_t minutes = (ms / 60000UL) % 100U;
  const uint8_t seconds  = (ms / 1000UL) % 60U;
  snprintf(out, outLen, "%02u:%02u", minutes, seconds);
}

static void formatTimeUsToFullMsString(uint32_t us, char* out, size_t outLen) {
  // Exibe MM:SS:MMM (us -> ms) para popups finais (uma vez por volta)
  const uint32_t ms = us / 1000UL;
  const uint16_t minutes = (ms / 60000UL) % 100U;
  const uint8_t seconds  = (ms / 1000UL) % 60U;
  const uint16_t millis  = ms % 1000UL;
  snprintf(out, outLen, "%02u:%02u:%03u", minutes, seconds, millis);
}

static uint32_t lapDeltaUs(uint8_t idx) {
  if (idx == 0) return gLapUs[0];
  return gLapUs[idx] - gLapUs[idx - 1];
}

static void renderRunScreenMinimal(uint32_t elapsedUs) {
  // Linha 0: estado + voltas (muda raramente)
  // Linha 1: tempo MM:SS (muda 1x por segundo)
  char line0[17];

  switch (gState) {
    case RunState::IDLE:    snprintf(line0, sizeof(line0), "Crono  V:%02u", (unsigned)gLapCount); break;
    case RunState::RUNNING: snprintf(line0, sizeof(line0), "RodandoV:%02u", (unsigned)gLapCount); break;
    case RunState::PAUSED:  snprintf(line0, sizeof(line0), "PausadoV:%02u", (unsigned)gLapCount); break;
    case RunState::FINISHED:snprintf(line0, sizeof(line0), "Final  V:%02u", (unsigned)gLapCount); break;
  }

  lcdWriteLineCached(0, line0);

  char t[8];
  formatTimeUsToMsString(elapsedUs, t, sizeof(t)); // MM:SS
  lcdWriteLineCached(1, t);
}

static void renderFinalLapPopupAccum(uint8_t lapIndex) {
  // Pós-medida: mostrar apenas acumulado (limpo visualmente)
  // Linha0: "Volta 03/08"
  // Linha1: "MM:SS:MMM"
  char line0[17];
  snprintf(line0, sizeof(line0), "Volta %02u/%02u", (unsigned)(lapIndex + 1), (unsigned)gLapCount);
  lcdWriteLineCached(0, line0);

  char t[12];
  formatTimeUsToFullMsString(gLapUs[lapIndex], t, sizeof(t));
  lcdWriteLineCached(1, t);
}

// ─────────────────────────────────────────────────────────────────────────────
// 12) Serial: impressão de dados
// ─────────────────────────────────────────────────────────────────────────────
static void printLapInstantToSerial(uint8_t sensorId, uint8_t lapNumber1, uint32_t accumUs, uint32_t deltaUs) {
  // Linha curta por evento, fácil de copiar:
  // LAP,<n>,SRC,<S0|S1|S2|S3|BTN>,ACC_US,<...>,DEL_US,<...>
  Serial.print("LAP,");
  Serial.print(lapNumber1);
  Serial.print(",SRC,");

  if (sensorId <= 3) {
    Serial.print('S');
    Serial.print(sensorId);
  } else {
    Serial.print("BTN");
  }

  Serial.print(",ACC_US,");
  Serial.print(accumUs);
  Serial.print(",DEL_US,");
  Serial.println(deltaUs);
}

static void printSummaryOnceToSerial() {
  if (gSerialSummaryPrinted) return;
  gSerialSummaryPrinted = true;

  Serial.println();
  Serial.println("=== RESUMO CRONOMETRO ===");
  Serial.print("TOTAL_LAPS,"); Serial.println(gLapCount);

  // Tempo total: se houver voltas, total pode ser a última volta; se não, usa gElapsedUs
  uint32_t totalUs = gElapsedUs;
  if (gLapCount > 0) totalUs = gLapUs[gLapCount - 1];

  Serial.print("TOTAL_US,"); Serial.println(totalUs);

  Serial.println("IDX,ACC_US,DEL_US,ACC_FMT,DEL_FMT");

  for (uint8_t i = 0; i < gLapCount; ++i) {
    const uint32_t accUs = gLapUs[i];
    const uint32_t delUs = lapDeltaUs(i);

    char accFmt[12];
    char delFmt[12];
    formatTimeUsToFullMsString(accUs, accFmt, sizeof(accFmt));
    formatTimeUsToFullMsString(delUs, delFmt, sizeof(delFmt));

    Serial.print((unsigned)(i + 1));
    Serial.print(',');
    Serial.print(accUs);
    Serial.print(',');
    Serial.print(delUs);
    Serial.print(',');
    Serial.print(accFmt);
    Serial.print(',');
    Serial.println(delFmt);
  }

  Serial.println("=========================");
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// 13) Ações do cronômetro
// ─────────────────────────────────────────────────────────────────────────────
static void resetAll() {
  gState = RunState::IDLE;
  gStartUs = 0;
  gElapsedUs = 0;

  gLapCount = 0;
  for (uint8_t i = 0; i < kMaxLaps; ++i) gLapUs[i] = 0;

  gFinalLapIndex = 0;
  gNextFinalPopupMs = 0;

  gPendingClickCount = 0;
  gClickSequenceDeadlineMs = 0;
  gBtnPressStartMs = 0;
  gLongPressHandled = false;

  gSerialSummaryPrinted = false;

  // Estado do experimento
  resetExperimentState();

  // Limpa cache do LCD (força refresh)
  for (uint8_t i = 0; i < 16; ++i) { gLcdCache0[i] = '\0'; gLcdCache1[i] = '\0'; }
  gLcdCache0[16] = '\0';
  gLcdCache1[16] = '\0';

  // Zera buffer e estados de sensores (com segurança)
  noInterrupts();
  gEventHead = gEventTail = 0;
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    gSawFall[i] = false;
    gFallUs[i] = 0;
    gLastAcceptedUs[i] = 0;
  }
  gPrevPIND = PIND;
  gPrevPINB = PINB;

  // Opcional: zera a base de tempo do Timer1 (deixa logs mais "limpos")
  TCNT1 = 0;
  gTimer1Overflows = 0;
  TIFR1 |= _BV(TOV1);
  interrupts();

  renderRunScreenMinimal(0);
}

static void startOrResumeUs(uint32_t nowUs) {
  if (gState == RunState::FINISHED) return;
  gStartUs = nowUs - gElapsedUs;
  gState = RunState::RUNNING;
}

static void pauseUs(uint32_t nowUs) {
  if (gState != RunState::RUNNING) return;
  gElapsedUs = nowUs - gStartUs;
  gState = RunState::PAUSED;
}

static void togglePauseResumeByDoubleClick(uint32_t nowUs) {
  switch (gState) {
    case RunState::IDLE:    startOrResumeUs(nowUs); break;
    case RunState::RUNNING: pauseUs(nowUs); break;
    case RunState::PAUSED:  startOrResumeUs(nowUs); break;
    case RunState::FINISHED: /* sem ação */ break;
  }
}

static void saveLapUs(uint8_t sourceId, uint32_t eventUs) {
  if (gState != RunState::RUNNING) return;
  if (gLapCount >= kMaxLaps) return;

  const uint32_t accUs = eventUs - gStartUs;
  gLapUs[gLapCount] = accUs;

  const uint32_t delUs = lapDeltaUs(gLapCount);

  // Log instantâneo no Serial (ajuda muito a coletar dados)
  printLapInstantToSerial(sourceId, (uint8_t)(gLapCount + 1), accUs, delUs);

  gLapCount++;
}

static void finishAndShowLaps(uint32_t nowUs, uint32_t nowMs) {
  if (gState == RunState::RUNNING) {
    gElapsedUs = nowUs - gStartUs;
  }

  if (gState == RunState::IDLE && gLapCount == 0 && gElapsedUs == 0) return;

  gState = RunState::FINISHED;
  gFinalLapIndex = 0;
  gNextFinalPopupMs = nowMs;

  // Ao finalizar, imprime resultados do experimento (se completos)
  printExperimentResultsOnceToSerial();

  // Ao finalizar, imprime uma vez o resumo completo no Serial
  printSummaryOnceToSerial();
}

static void processClickSequence(uint8_t clicks, uint32_t nowUs, uint32_t nowMs) {
  switch (clicks) {
    case 1:
      // 1 clique = marca volta (fonte BTN = 255)
      saveLapUs(255, nowUs);
      break;
    case 2:
      togglePauseResumeByDoubleClick(nowUs);
      break;
    case 3:
      finishAndShowLaps(nowUs, nowMs);
      break;
    default:
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 14) Boas-vindas (setup)
// ─────────────────────────────────────────────────────────────────────────────
static void showStartMessage() {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("PNFM");
  delay(900);

  lcd.setCursor(0, 1);
  lcd.print("FORMACAO DE PROF.");
  delay(900);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AT EXPERIMENTAIS");
  delay(900);

  lcd.setCursor(5, 1);
  lcd.print("FISICA");
  delay(900);

  lcd.clear();
}

// ─────────────────────────────────────────────────────────────────────────────
// 15) Setup/Loop
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(kSerialBaud);

  // Inicia base de tempo por hardware (Timer1)
  initTimer1Timebase();

  lcd.begin(16, 2);

  // Botão com pull-up interno
  pinMode(kPinBtnControl, INPUT_PULLUP);
  inBtnControl.begin(kPinBtnControl, kDebounceButtonMs);

  // Sensores com resistor externo 10k -> usar INPUT (sem pull-up interno)
  pinMode(kPinSensor0, INPUT);
  pinMode(kPinSensor1, INPUT);
  pinMode(kPinSensor2, INPUT);
  pinMode(kPinSensor3, INPUT);

  initPcintForSensors();

  showStartMessage();
  resetAll();

  Serial.println("Cronometro pronto. Regras do botao:");
  Serial.println("1 clique = volta | 2 cliques = start/pause/resume | 3 cliques = finalizar | longo = reset");
  Serial.println("Sensores: S0=start/resume (gate 1) | S1,S2,S3 = gates 2..4");
  Serial.print("Experimento: spacing entre sensores = ");
  Serial.print(kSensorSpacingM, 2);
  Serial.println(" m");
}

void loop() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = timer1Micros();

  // A) Processar eventos de sensores (capturados por PCINT)
  SensorEvent ev;
  while (popEvent(ev)) {
    if (ev.sensor_id == 0) {
      // Sensor 0: start/resume e também gate 1 do experimento
      if (gState == RunState::IDLE) {
        resetExperimentState();
      }
      registerExperimentSensor(0, ev.t_us);
      startOrResumeUs(ev.t_us);
    } else {
      // sensor 1..3: registra volta e marca gate do experimento
      saveLapUs(ev.sensor_id, ev.t_us);

      // Só registra gates se o experimento começou de fato (S0 visto e RUNNING)
      if (gState == RunState::RUNNING && gExpSeen[0]) {
        registerExperimentSensor(ev.sensor_id, ev.t_us);

        // Quando os 4 sensores forem acionados, finaliza automaticamente
        if (experimentComplete()) {
          finishAndShowLaps(ev.t_us, nowMs);
        }
      }
    }
  }

  // B) Botão (debounce + multi-clique + longo)
  inBtnControl.update(nowMs);

  if (inBtnControl.fell()) {
    gBtnPressStartMs = nowMs;
    gLongPressHandled = false;
  }

  if (inBtnControl.isPressed() && !gLongPressHandled) {
    if ((uint32_t)(nowMs - gBtnPressStartMs) >= kLongPressMs) {
      resetAll();
      gPendingClickCount = 0;
      gClickSequenceDeadlineMs = 0;
      gLongPressHandled = true;

      Serial.println("RESET");
    }
  }

  if (inBtnControl.rose()) {
    if (!gLongPressHandled) {
      gPendingClickCount++;
      gClickSequenceDeadlineMs = nowMs + kMultiClickGapMs;
    }
  }

  if (gPendingClickCount > 0 && !inBtnControl.isPressed()) {
    if (nowMs >= gClickSequenceDeadlineMs) {
      processClickSequence(gPendingClickCount, nowUs, nowMs);
      gPendingClickCount = 0;
      gClickSequenceDeadlineMs = 0;
    }
  }

  // C) Atualizar tempo em micros
  if (gState == RunState::RUNNING) {
    gElapsedUs = nowUs - gStartUs;
  }

  // D) LCD: atualização reduzida
  if ((uint32_t)(nowMs - gLastLcdUpdateMs) < kLcdUpdateIntervalMs) {
    return;
  }
  gLastLcdUpdateMs = nowMs;

  // FINISHED: popups limpos (apenas acumulado)
  if (gState == RunState::FINISHED) {
    if (gLapCount == 0) {
      renderRunScreenMinimal(gElapsedUs);
      return;
    }

    if (nowMs >= gNextFinalPopupMs) {
      renderFinalLapPopupAccum(gFinalLapIndex);

      gFinalLapIndex++;
      if (gFinalLapIndex >= gLapCount) gFinalLapIndex = 0;

      gNextFinalPopupMs = nowMs + kFinalPopupCycleMs;
    }
    return;
  }

  // IDLE/RUNNING/PAUSED: tela minimalista
  renderRunScreenMinimal(gElapsedUs);
}
