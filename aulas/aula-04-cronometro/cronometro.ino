#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LCD.h>
#include <stdio.h>
#include <avr/interrupt.h>

// ─────────────────────────────────────────────────────────────────────────────
// 1) LCD (I2C) — mantido no padrão do projeto
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

// Sensores (fototransistor + resistor 10k + fio longo)
// sensor0: start/resume
// sensor1..3: marcações
static constexpr uint8_t kPinSensor0 = 5;
static constexpr uint8_t kPinSensor1 = 6;
static constexpr uint8_t kPinSensor2 = 7;
static constexpr uint8_t kPinSensor3 = 8;

// IDs internos para o buffer de eventos
static constexpr uint8_t kSensorCount = 4; // 0..3

// ─────────────────────────────────────────────────────────────────────────────
// 3) PARÂMETROS DO SISTEMA
// ─────────────────────────────────────────────────────────────────────────────

// BOTÃO (mecânico)
static constexpr uint16_t kDebounceButtonMs   = 25;
static constexpr uint16_t kLongPressMs        = 1200;
static constexpr uint16_t kMultiClickGapMs    = 350;

// LCD (reduzir escrita reduz tempo ocupado no I2C)
static constexpr uint16_t kLcdUpdateIntervalMs = 120; // ~8 Hz
static constexpr uint16_t kFinalPopupCycleMs   = 1400;
static constexpr uint16_t kLapSavedPopupMs     = 900;

// VOLTAS
static constexpr uint8_t kMaxLaps = 20;

// SENSORES (robustez contra ruído e pulsos curtos)
//
// kMinActivePulseUs:
//   Filtra ruídos muito rápidos (spikes no fio longo). Como a bolinha deve
//   bloquear o feixe por centenas de microssegundos ou mais, um mínimo de
//   ~150–300 us costuma funcionar bem.
// Ajuste prático: se perder eventos reais, diminua; se tiver falsos, aumente.
static constexpr uint16_t kMinActivePulseUs = 200;

// kSensorRearmUs:
//   Após aceitar um evento de um sensor, ignora novos do mesmo sensor por um tempo.
// Isso evita dupla marcação por oscilação/ruído.
// Em experimento de bolinha, 20–50 ms é seguro.
static constexpr uint32_t kSensorRearmUs = 25000;

// Assumimos que o “evento” é a transição HIGH->LOW (ativo em LOW).
// Se sua montagem estiver invertida, eu explico como trocar no final.
static constexpr bool kSensorActiveLow = true;

// ─────────────────────────────────────────────────────────────────────────────
// 4) DEBOUNCE DO BOTÃO (mecânico) — mantém a lógica de 1/2/3 cliques
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
// 5) CAPTURA DE SENSORES POR PCINT (INTERRUPÇÃO)  ✅ PRINCIPAL MUDANÇA
// ─────────────────────────────────────────────────────────────────────────────
//
// Estratégia:
// - Capturamos mudanças de pinos por Pin Change Interrupt (PCINT).
// - Registramos tempo de borda com micros().
// - Validamos o evento quando o pulso termina (borda de retorno), medindo
//   a largura do pulso ativo.
// - Se largura >= kMinActivePulseUs e passou rearm, empilhamos evento em buffer.
//
// Por que validar no fim do pulso?
// - Para rejeitar spikes rápidos (ruído em fio longo).
// - Mantemos o timestamp do início (queda) para precisão.
//
// OBS: Nunca atualize LCD dentro da ISR. ISR deve ser rápida.
//
struct SensorEvent {
  uint8_t sensor_id;   // 0..3
  uint32_t t_us;       // timestamp em micros (início do pulso ativo)
};

static constexpr uint8_t kEventBufSize = 16;
volatile SensorEvent gEventBuf[kEventBufSize];
volatile uint8_t gEventHead = 0;
volatile uint8_t gEventTail = 0;

// Estado por sensor (volátil porque é usado em ISR)
volatile uint32_t gFallUs[kSensorCount] = {0, 0, 0, 0};
volatile bool gSawFall[kSensorCount] = {false, false, false, false};
volatile uint32_t gLastAcceptedUs[kSensorCount] = {0, 0, 0, 0};

// Estado anterior das portas para detectar mudanças
volatile uint8_t gPrevPIND = 0;
volatile uint8_t gPrevPINB = 0;

static inline uint32_t usDiff(uint32_t a, uint32_t b) {
  // Diferença segura mesmo com overflow de micros() (desde que intervalos sejam curtos).
  return (uint32_t)(a - b);
}

static inline void pushEventIsr(uint8_t sensorId, uint32_t tUs) {
  const uint8_t next = (uint8_t)((gEventHead + 1) % kEventBufSize);
  if (next == gEventTail) {
    // Buffer cheio: descarta evento (muito improvável nesse experimento)
    return;
  }

  gEventBuf[gEventHead].sensor_id = sensorId;
  gEventBuf[gEventHead].t_us = tUs;
  gEventHead = next;
}

static void handleSensorEdgeIsr(uint8_t sensorId, bool pinIsHigh, uint32_t nowUs) {
  // “ativo” significa estado que representa detecção da bolinha.
  // Se ativo em LOW: pulso ativo = LOW, inicia em HIGH->LOW (queda), termina em LOW->HIGH (subida).
  // Se ativo em HIGH: seria o contrário (não implementamos aqui para manter claro).
  if (!kSensorActiveLow) {
    // Se precisar inverter, ajuste no final conforme instruções.
    return;
  }

  if (!pinIsHigh) {
    // Borda HIGH->LOW: início do pulso ativo
    gFallUs[sensorId] = nowUs;
    gSawFall[sensorId] = true;
    return;
  }

  // pinIsHigh == true: borda LOW->HIGH (fim do pulso)
  if (!gSawFall[sensorId]) {
    return; // subida sem ter visto queda: ignora
  }

  const uint32_t fall = gFallUs[sensorId];
  gSawFall[sensorId] = false;

  const uint32_t width = usDiff(nowUs, fall);
  if (width < kMinActivePulseUs) {
    // Muito curto: provável ruído/spike
    return;
  }

  // Rearm por sensor
  const uint32_t sinceLast = usDiff(fall, gLastAcceptedUs[sensorId]);
  if (sinceLast < kSensorRearmUs) {
    return;
  }

  gLastAcceptedUs[sensorId] = fall;
  pushEventIsr(sensorId, fall);
}

// PCINT para pinos 5,6,7 (PORTD)
ISR(PCINT2_vect) {
  const uint32_t nowUs = micros();

  const uint8_t cur = PIND;
  const uint8_t changed = (uint8_t)(cur ^ gPrevPIND);
  gPrevPIND = cur;

  // Pino 5 (PD5) -> sensor0
  if (changed & _BV(5)) {
    const bool isHigh = (cur & _BV(5)) != 0;
    handleSensorEdgeIsr(0, isHigh, nowUs);
  }

  // Pino 6 (PD6) -> sensor1
  if (changed & _BV(6)) {
    const bool isHigh = (cur & _BV(6)) != 0;
    handleSensorEdgeIsr(1, isHigh, nowUs);
  }

  // Pino 7 (PD7) -> sensor2
  if (changed & _BV(7)) {
    const bool isHigh = (cur & _BV(7)) != 0;
    handleSensorEdgeIsr(2, isHigh, nowUs);
  }
}

// PCINT para pino 8 (PB0)
ISR(PCINT0_vect) {
  const uint32_t nowUs = micros();

  const uint8_t cur = PINB;
  const uint8_t changed = (uint8_t)(cur ^ gPrevPINB);
  gPrevPINB = cur;

  // Pino 8 é PB0 -> sensor3
  if (changed & _BV(0)) {
    const bool isHigh = (cur & _BV(0)) != 0;
    handleSensorEdgeIsr(3, isHigh, nowUs);
  }
}

static bool popEvent(SensorEvent& out) {
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
  // Configura PCINT para:
  // - pinos 5,6,7 -> PCINT2_vect (PORTD)
  // - pino 8      -> PCINT0_vect (PORTB)

  noInterrupts();

  // Estado inicial das portas (para detectar mudanças)
  gPrevPIND = PIND;
  gPrevPINB = PINB;

  // Habilita grupos PCIE2 (PORTD) e PCIE0 (PORTB)
  PCICR |= (1 << PCIE2) | (1 << PCIE0);

  // PORTD: PCINT21 (PD5), PCINT22 (PD6), PCINT23 (PD7)
  PCMSK2 |= (1 << PCINT21) | (1 << PCINT22) | (1 << PCINT23);

  // PORTB: PCINT0 (PB0)
  PCMSK0 |= (1 << PCINT0);

  interrupts();
}

// ─────────────────────────────────────────────────────────────────────────────
// 6) CRONÔMETRO (agora em micros())
// ─────────────────────────────────────────────────────────────────────────────
enum class RunState : uint8_t {
  IDLE = 0,
  RUNNING,
  PAUSED,
  FINISHED
};

static RunState gState = RunState::IDLE;

// Tempo em microsegundos
static uint32_t gStartUs = 0;
static uint32_t gElapsedUs = 0;

// Voltas em microsegundos (tempo acumulado)
static uint32_t gLapUs[kMaxLaps];
static uint8_t gLapCount = 0;

// Popups finais (tempo humano em ms)
static uint8_t gFinalLapIndex = 0;
static bool gFinalShowDelta = false;
static uint32_t gNextFinalPopupMs = 0;

// Feedback “volta salva”
static bool gLapSavedPopupActive = false;
static uint32_t gLapSavedPopupUntilMs = 0;
static uint8_t gLapSavedIndex = 0;

// ─────────────────────────────────────────────────────────────────────────────
// 7) BOTÃO (mantém a lógica 1/2/3 cliques + longo)
// ─────────────────────────────────────────────────────────────────────────────
static DebouncedInput inBtnControl;

static uint32_t gBtnPressStartMs = 0;
static bool gLongPressHandled = false;

static uint8_t gPendingClickCount = 0;
static uint32_t gClickSequenceDeadlineMs = 0;

// ─────────────────────────────────────────────────────────────────────────────
// 8) LCD otimizado (cache por linha)
// ─────────────────────────────────────────────────────────────────────────────
static char gLcdCache0[17] = {0};
static char gLcdCache1[17] = {0};
static uint32_t gLastLcdUpdateMs = 0;

static void pad16(const char* src, char out16[17]) {
  for (uint8_t i = 0; i < 16; ++i) out16[i] = ' ';
  out16[16] = '\0';

  uint8_t i = 0;
  while (i < 16 && src[i] != '\0') {
    out16[i] = src[i];
    ++i;
  }
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
// 9) Formatação e telas
// ─────────────────────────────────────────────────────────────────────────────
static void formatTimeFromUs(uint32_t us, char* out, size_t outLen) {
  // Exibição continua em MM:SS:MMM (ms), mas o cálculo vem de micros.
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

static void renderMainScreen(uint32_t elapsedUs) {
  char line0[17];
  switch (gState) {
    case RunState::IDLE:    snprintf(line0, sizeof(line0), "Crono V:%02u", (unsigned)gLapCount); break;
    case RunState::RUNNING: snprintf(line0, sizeof(line0), "Rodando V:%02u", (unsigned)gLapCount); break;
    case RunState::PAUSED:  snprintf(line0, sizeof(line0), "Pausado V:%02u", (unsigned)gLapCount); break;
    case RunState::FINISHED:snprintf(line0, sizeof(line0), "Final  V:%02u", (unsigned)gLapCount); break;
  }
  lcdWriteLineCached(0, line0);

  char t[12];
  formatTimeFromUs(elapsedUs, t, sizeof(t));
  lcdWriteLineCached(1, t);
}

static void renderLapSavedPopup(uint8_t lapIndex) {
  char line0[17];
  snprintf(line0, sizeof(line0), "Volta %02u salva", (unsigned)(lapIndex + 1));
  lcdWriteLineCached(0, line0);

  char t[12];
  formatTimeFromUs(gLapUs[lapIndex], t, sizeof(t));
  lcdWriteLineCached(1, t);
}

static void renderFinalLapPopup(uint8_t lapIndex, bool showDelta) {
  char line0[17];
  snprintf(
    line0,
    sizeof(line0),
    showDelta ? "V%02u/%02u DELT" : "V%02u/%02u ACUM",
    (unsigned)(lapIndex + 1),
    (unsigned)gLapCount
  );
  lcdWriteLineCached(0, line0);

  const uint32_t valueUs = showDelta ? lapDeltaUs(lapIndex) : gLapUs[lapIndex];

  char t[12];
  formatTimeFromUs(valueUs, t, sizeof(t));
  lcdWriteLineCached(1, t);
}

// ─────────────────────────────────────────────────────────────────────────────
// 10) Ações do cronômetro
// ─────────────────────────────────────────────────────────────────────────────
static void resetAll() {
  gState = RunState::IDLE;
  gStartUs = 0;
  gElapsedUs = 0;

  gLapCount = 0;
  for (uint8_t i = 0; i < kMaxLaps; ++i) gLapUs[i] = 0;

  gFinalLapIndex = 0;
  gFinalShowDelta = false;
  gNextFinalPopupMs = 0;

  gLapSavedPopupActive = false;
  gLapSavedPopupUntilMs = 0;
  gLapSavedIndex = 0;

  gPendingClickCount = 0;
  gClickSequenceDeadlineMs = 0;

  gBtnPressStartMs = 0;
  gLongPressHandled = false;

  // Limpa cache do LCD para forçar refresh
  for (uint8_t i = 0; i < 16; ++i) {
    gLcdCache0[i] = '\0';
    gLcdCache1[i] = '\0';
  }
  gLcdCache0[16] = '\0';
  gLcdCache1[16] = '\0';

  // Zera buffer de eventos e estados dos sensores com segurança
  noInterrupts();
  gEventHead = gEventTail = 0;
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    gSawFall[i] = false;
    gFallUs[i] = 0;
    gLastAcceptedUs[i] = 0;
  }
  gPrevPIND = PIND;
  gPrevPINB = PINB;
  interrupts();

  renderMainScreen(0);
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

static void saveLapUs(uint32_t eventUs, uint32_t nowMs) {
  if (gState != RunState::RUNNING) return;
  if (gLapCount >= kMaxLaps) return;

  const uint32_t elapsed = eventUs - gStartUs;
  gLapUs[gLapCount] = elapsed;

  // Feedback visual curto
  gLapSavedIndex = gLapCount;
  gLapSavedPopupActive = true;
  gLapSavedPopupUntilMs = nowMs + kLapSavedPopupMs;

  gLapCount++;
}

static void finishAndShowLaps(uint32_t nowUs, uint32_t nowMs) {
  if (gState == RunState::RUNNING) {
    gElapsedUs = nowUs - gStartUs;
  }

  if (gState == RunState::IDLE && gLapCount == 0 && gElapsedUs == 0) return;

  gState = RunState::FINISHED;
  gLapSavedPopupActive = false;

  gFinalLapIndex = 0;
  gFinalShowDelta = false;
  gNextFinalPopupMs = nowMs;
}

static void processClickSequence(uint8_t clicks, uint32_t nowUs, uint32_t nowMs) {
  switch (clicks) {
    case 1:
      // 1 clique = marca volta pelo botão (usa “agora” como timestamp)
      saveLapUs(nowUs, nowMs);
      break;
    case 2:
      // 2 cliques = start/pause/resume
      togglePauseResumeByDoubleClick(nowUs);
      break;
    case 3:
      // 3 cliques = finaliza e mostra popups
      finishAndShowLaps(nowUs, nowMs);
      break;
    default:
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 11) Boas-vindas (setup)
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
// 12) Setup/Loop
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  lcd.begin(16, 2);

  // Botão com pull-up interno (mecânico)
  pinMode(kPinBtnControl, INPUT_PULLUP);
  inBtnControl.begin(kPinBtnControl, kDebounceButtonMs);

  // Sensores: como você tem resistor externo de 10k, NÃO usamos pull-up interno
  // para não “misturar” resistores e distorcer o ponto de operação.
  pinMode(kPinSensor0, INPUT);
  pinMode(kPinSensor1, INPUT);
  pinMode(kPinSensor2, INPUT);
  pinMode(kPinSensor3, INPUT);

  // Habilita PCINT para capturar os sensores (5–8)
  initPcintForSensors();

  showStartMessage();
  resetAll();
}

void loop() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  // ────────────────────────────────────────────────────────────────────────
  // A) Processar eventos de sensores capturados por interrupção (PCINT)
  // ────────────────────────────────────────────────────────────────────────
  // Aqui os eventos já passaram por:
  // - filtro de largura mínima do pulso (anti-ruído)
  // - rearm por sensor
  SensorEvent ev;
  while (popEvent(ev)) {
    if (ev.sensor_id == 0) {
      // sensor0: start/resume com timestamp real do sensor
      startOrResumeUs(ev.t_us);
    } else {
      // sensor1..3: marcações com timestamp real do sensor
      saveLapUs(ev.t_us, nowMs);
    }
  }

  // ────────────────────────────────────────────────────────────────────────
  // B) Botão (debounce + multi-clique + clique longo)
  // ────────────────────────────────────────────────────────────────────────
  inBtnControl.update(nowMs);

  if (inBtnControl.fell()) {
    gBtnPressStartMs = nowMs;
    gLongPressHandled = false;
  }

  // Clique longo -> reset
  if (inBtnControl.isPressed() && !gLongPressHandled) {
    if ((uint32_t)(nowMs - gBtnPressStartMs) >= kLongPressMs) {
      resetAll();
      gPendingClickCount = 0;
      gClickSequenceDeadlineMs = 0;
      gLongPressHandled = true;
    }
  }

  // Soltou -> conta clique curto (se não foi longo)
  if (inBtnControl.rose()) {
    if (!gLongPressHandled) {
      gPendingClickCount++;
      gClickSequenceDeadlineMs = nowMs + kMultiClickGapMs;
    }
  }

  // Janela expirou -> decide se foi 1/2/3 cliques
  if (gPendingClickCount > 0 && !inBtnControl.isPressed()) {
    if (nowMs >= gClickSequenceDeadlineMs) {
      processClickSequence(gPendingClickCount, nowUs, nowMs);
      gPendingClickCount = 0;
      gClickSequenceDeadlineMs = 0;
    }
  }

  // ────────────────────────────────────────────────────────────────────────
  // C) Atualizar tempo do cronômetro (em micros)
  // ────────────────────────────────────────────────────────────────────────
  if (gState == RunState::RUNNING) {
    gElapsedUs = nowUs - gStartUs;
  }

  // ────────────────────────────────────────────────────────────────────────
  // D) LCD: atualizar em taxa fixa e com cache
  // ────────────────────────────────────────────────────────────────────────
  if ((uint32_t)(nowMs - gLastLcdUpdateMs) < kLcdUpdateIntervalMs) {
    return;
  }
  gLastLcdUpdateMs = nowMs;

  // Modo final: alterna ACUM/DELT das voltas
  if (gState == RunState::FINISHED) {
    if (gLapCount == 0) {
      renderMainScreen(gElapsedUs);
      return;
    }

    if (nowMs >= gNextFinalPopupMs) {
      renderFinalLapPopup(gFinalLapIndex, gFinalShowDelta);

      // Alternância: ACUM -> DELT -> próxima volta
      if (!gFinalShowDelta) {
        gFinalShowDelta = true;
      } else {
        gFinalShowDelta = false;
        gFinalLapIndex++;
        if (gFinalLapIndex >= gLapCount) gFinalLapIndex = 0;
      }

      gNextFinalPopupMs = nowMs + kFinalPopupCycleMs;
    }
    return;
  }

  // Feedback “volta salva”
  if (gLapSavedPopupActive) {
    if (nowMs >= gLapSavedPopupUntilMs) {
      gLapSavedPopupActive = false;
    } else {
      renderLapSavedPopup(gLapSavedIndex);
      return;
    }
  }

  // Tela normal
  renderMainScreen(gElapsedUs);
}