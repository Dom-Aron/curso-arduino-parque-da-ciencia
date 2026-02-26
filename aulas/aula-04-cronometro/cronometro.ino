#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LCD.h>
#include <stdio.h>  // snprintf

// ─────────────────────────────────────────────────────────────────────────────
// 1) CONFIGURAÇÃO DO LCD (I2C)
// ─────────────────────────────────────────────────────────────────────────────
// Mantido no padrão do seu projeto original (backpack I2C com mapeamento).
static constexpr uint8_t kLcdAddr = 0x27;
static constexpr uint8_t kEn = 2;   // Enable
static constexpr uint8_t kRw = 1;   // Read/Write
static constexpr uint8_t kRs = 0;   // Register Select
static constexpr uint8_t kD4 = 4;   // Data 4
static constexpr uint8_t kD5 = 5;   // Data 5
static constexpr uint8_t kD6 = 6;   // Data 6
static constexpr uint8_t kD7 = 7;   // Data 7
static constexpr uint8_t kBacklightPin = 3;
static constexpr t_backlightPol kBacklightPolarity = POSITIVE;

LiquidCrystal_I2C lcd(
  kLcdAddr, kEn, kRw, kRs, kD4, kD5, kD6, kD7, kBacklightPin, kBacklightPolarity
);

// ─────────────────────────────────────────────────────────────────────────────
// 2) PINOS (BOTÃO + SENSORES)
// ─────────────────────────────────────────────────────────────────────────────
// Todas as entradas usam INPUT_PULLUP:
// - HIGH = repouso
// - LOW  = pressionado/acionado
static constexpr uint8_t kPinBtnControl = 2;

// Sensores externos
static constexpr uint8_t kPinSensorStart = 5; // sensor0 -> inicia/retoma
static constexpr uint8_t kPinSensor1 = 6;     // marca volta
static constexpr uint8_t kPinSensor2 = 7;     // marca volta
static constexpr uint8_t kPinSensor3 = 8;     // marca volta

// Sensores que marcam voltas (S1, S2, S3)
static constexpr uint8_t kMarkSensorCount = 3;
static constexpr uint8_t kMarkSensorPins[kMarkSensorCount] = {
  kPinSensor1, kPinSensor2, kPinSensor3
};

// ─────────────────────────────────────────────────────────────────────────────
// 3) PARÂMETROS DE TEMPORIZAÇÃO (SEM BLOQUEIO)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint16_t kDebounceButtonMs = 25;   // debounce do botão mecânico
static constexpr uint16_t kDebounceSensorMs = 5;    // debounce dos sensores
static constexpr uint16_t kLongPressMs = 1200;      // clique longo = reset
static constexpr uint16_t kMultiClickGapMs = 350;   // janela p/ distinguir 1,2,3 cliques

// Atualização do LCD em taxa fixa (evita flicker e sobrecarga)
static constexpr uint16_t kLcdUpdateIntervalMs = 50; // ~20 Hz

// Popups finais (modo FINISHED): tempo de cada tela ACUM/DELT
static constexpr uint16_t kFinalPopupCycleMs = 1400;

// Feedback visual ao salvar volta (durante RUNNING)
static constexpr uint16_t kLapSavedPopupMs = 900;

// Quantidade máxima de voltas armazenadas
static constexpr uint8_t kMaxLaps = 20;

// ─────────────────────────────────────────────────────────────────────────────
// 4) DEBOUNCE + EVENTOS DE BORDA (BOTÃO / SENSORES)
// ─────────────────────────────────────────────────────────────────────────────
// Esta estrutura:
// 1) lê o pino
// 2) filtra bounce por tempo (debounce)
// 3) gera eventos de borda:
//    - fell() : HIGH -> LOW  (pressionou/acionou)
//    - rose() : LOW  -> HIGH (soltou/liberou)
//
// Como usamos INPUT_PULLUP:
// - HIGH = repouso
// - LOW  = acionado
struct DebouncedInput {
  uint8_t pin = 255;
  uint16_t debounceMs = 0;

  bool stableState = true;      // estado estável atual (true=HIGH, false=LOW)
  bool lastRawState = true;     // última leitura crua
  uint32_t lastChangeMs = 0;    // instante da última mudança crua

  bool fellEvent = false;       // evento de borda de descida (pressionou)
  bool roseEvent = false;       // evento de borda de subida (soltou)

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

    // Se a leitura crua mudou, reinicia a "contagem de estabilização".
    if (raw != lastRawState) {
      lastRawState = raw;
      lastChangeMs = nowMs;
    }

    // Só aceita a mudança se ela ficar estável por tempo suficiente.
    if ((nowMs - lastChangeMs) >= debounceMs && stableState != raw) {
      const bool prevStable = stableState;
      stableState = raw;

      if (prevStable == true && stableState == false) {
        fellEvent = true;  // HIGH -> LOW
      } else if (prevStable == false && stableState == true) {
        roseEvent = true;  // LOW -> HIGH
      }
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
// 5) MÁQUINA DE ESTADOS DO CRONÔMETRO
// ─────────────────────────────────────────────────────────────────────────────
enum class RunState : uint8_t {
  IDLE = 0,    // parado no zero
  RUNNING,     // contando
  PAUSED,      // pausado
  FINISHED     // finalizado (exibindo popups)
};

static RunState gState = RunState::IDLE;

// Base temporal do cronômetro
static uint32_t gStartMs = 0;    // referência de início/retomada
static uint32_t gElapsedMs = 0;  // tempo decorrido atual (ms)

// Voltas (marcas) armazenadas como TEMPOS ACUMULADOS
static uint32_t gLapTimesMs[kMaxLaps];
static uint8_t gLapCount = 0;

// Controle de atualização do LCD
static uint32_t gLastLcdUpdateMs = 0;

// ─────────────────────────────────────────────────────────────────────────────
// 6) POPUPS FINAIS (ACUM / DELTA) PARA LCD 16x2
// ─────────────────────────────────────────────────────────────────────────────
// Como o LCD é 16x2, cada volta é exibida em 2 telas:
// - Tela A: ACUM (tempo acumulado até a volta)
// - Tela B: DELT (tempo da volta = diferença para a anterior)
static uint8_t gFinalPopupLapIndex = 0;   // índice da volta atual (0..gLapCount-1)
static bool gFinalPopupShowDelta = false; // false=ACUM, true=DELT
static uint32_t gNextFinalPopupMs = 0;

// ─────────────────────────────────────────────────────────────────────────────
// 7) FEEDBACK VISUAL AO SALVAR VOLTA (DURANTE RUNNING)
// ─────────────────────────────────────────────────────────────────────────────
static bool gLapSavedPopupActive = false;
static uint32_t gLapSavedPopupUntilMs = 0;
static uint8_t gLapSavedPopupLapIndex = 0; // última volta salva (índice 0-based)

// ─────────────────────────────────────────────────────────────────────────────
// 8) CONTROLE DO BOTÃO (CLIQUE CURTO / LONGO / MULTICLIQUE)
// ─────────────────────────────────────────────────────────────────────────────
// Entradas com debounce
static DebouncedInput inBtnControl;
static DebouncedInput inSensorStart;                    // sensor0
static DebouncedInput inMarkSensors[kMarkSensorCount];  // sensores 1,2,3

// Controle da pressão atual do botão (para detectar clique longo)
static uint32_t gBtnPressStartMs = 0;
static bool gLongPressHandled = false;

// Multi-clique (1/2/3 cliques)
// Armazenamos cliques curtos e aguardamos uma janela para decidir a ação.
static uint8_t gPendingClickCount = 0;
static uint32_t gClickSequenceDeadlineMs = 0;

// ─────────────────────────────────────────────────────────────────────────────
// 9) AUXILIARES DE DISPLAY (LCD 16x2)
// ─────────────────────────────────────────────────────────────────────────────

// Imprime exatamente 16 caracteres na linha (com espaços).
// Isso evita "sobras" visuais quando o texto novo é menor que o anterior.
static void lcdPrintLine(uint8_t row, const char* text) {
  char buf[17];
  for (uint8_t i = 0; i < 16; ++i) {
    buf[i] = ' ';
  }
  buf[16] = '\0';

  uint8_t i = 0;
  while (i < 16 && text[i] != '\0') {
    buf[i] = text[i];
    ++i;
  }

  lcd.setCursor(0, row);
  lcd.print(buf);
}

// Formata milissegundos como "MM:SS:MMM".
// Ex.: 03:12:045
static void formatTimeMs(uint32_t ms, char* out, size_t outLen) {
  const uint16_t minutes = (ms / 60000UL) % 100U;
  const uint8_t seconds = (ms / 1000UL) % 60U;
  const uint16_t millis = ms % 1000UL;

  snprintf(out, outLen, "%02u:%02u:%03u", minutes, seconds, millis);
}

// Calcula o tempo da volta (delta).
// - volta 1 -> delta = lap[0]
// - volta 2 -> delta = lap[1] - lap[0]
// - ...
static uint32_t getLapDeltaMs(uint8_t lapIndexZeroBased) {
  if (lapIndexZeroBased == 0) {
    return gLapTimesMs[0];
  }
  return gLapTimesMs[lapIndexZeroBased] - gLapTimesMs[lapIndexZeroBased - 1];
}

// Tela principal (IDLE / RUNNING / PAUSED)
static void renderMainScreen(uint32_t elapsedMs) {
  char line0[17];

  switch (gState) {
    case RunState::IDLE:
      snprintf(line0, sizeof(line0), "Crono V:%02u", (unsigned)gLapCount);
      break;

    case RunState::RUNNING:
      snprintf(line0, sizeof(line0), "Rodando V:%02u", (unsigned)gLapCount);
      break;

    case RunState::PAUSED:
      snprintf(line0, sizeof(line0), "Pausado V:%02u", (unsigned)gLapCount);
      break;

    case RunState::FINISHED:
      // Normalmente em FINISHED mostramos popups, mas deixamos isso coerente.
      snprintf(line0, sizeof(line0), "Final V:%02u", (unsigned)gLapCount);
      break;
  }

  lcdPrintLine(0, line0);

  char timeStr[12];
  formatTimeMs(elapsedMs, timeStr, sizeof(timeStr));
  lcdPrintLine(1, timeStr);
}

// Popup visual curto ao salvar uma volta (feedback imediato)
static void renderLapSavedPopup(uint8_t lapIndexZeroBased) {
  char line0[17];
  snprintf(line0, sizeof(line0), "Volta %02u salva", (unsigned)(lapIndexZeroBased + 1));
  lcdPrintLine(0, line0);

  char timeStr[12];
  formatTimeMs(gLapTimesMs[lapIndexZeroBased], timeStr, sizeof(timeStr));
  lcdPrintLine(1, timeStr);
}

// Popup final de uma volta específica (ACUM ou DELTA)
// Linha 0: "Vxx/yy ACUM" ou "Vxx/yy DELT"
// Linha 1: tempo formatado
static void renderFinalLapPopup(uint8_t lapIndexZeroBased, bool showDelta) {
  char line0[17];

  if (showDelta) {
    snprintf(
      line0,
      sizeof(line0),
      "V%02u/%02u DELT",
      (unsigned)(lapIndexZeroBased + 1),
      (unsigned)gLapCount
    );
  } else {
    snprintf(
      line0,
      sizeof(line0),
      "V%02u/%02u ACUM",
      (unsigned)(lapIndexZeroBased + 1),
      (unsigned)gLapCount
    );
  }

  lcdPrintLine(0, line0);

  const uint32_t valueMs = showDelta
    ? getLapDeltaMs(lapIndexZeroBased)
    : gLapTimesMs[lapIndexZeroBased];

  char timeStr[12];
  formatTimeMs(valueMs, timeStr, sizeof(timeStr));
  lcdPrintLine(1, timeStr);
}

// ─────────────────────────────────────────────────────────────────────────────
// 10) LÓGICA DO CRONÔMETRO
// ─────────────────────────────────────────────────────────────────────────────

// Reseta tudo e volta ao estado inicial (IDLE).
static void resetAll() {
  gState = RunState::IDLE;

  gStartMs = 0;
  gElapsedMs = 0;

  gLapCount = 0;
  for (uint8_t i = 0; i < kMaxLaps; ++i) {
    gLapTimesMs[i] = 0;
  }

  // Limpa popups finais
  gFinalPopupLapIndex = 0;
  gFinalPopupShowDelta = false;
  gNextFinalPopupMs = 0;

  // Limpa popup de feedback de volta salva
  gLapSavedPopupActive = false;
  gLapSavedPopupUntilMs = 0;
  gLapSavedPopupLapIndex = 0;

  // Limpa controle de cliques
  gPendingClickCount = 0;
  gClickSequenceDeadlineMs = 0;
  gBtnPressStartMs = 0;
  gLongPressHandled = false;

  renderMainScreen(0);
}

// Inicia ou retoma a contagem sem perder precisão.
// Técnica: start = agora - tempo_ja_decorrido
// Isso permite retomar corretamente após pausa.
static void startOrResume(uint32_t nowMs) {
  if (gState == RunState::FINISHED) {
    // Após finalizar, exigimos reset (clique longo) para nova sessão.
    return;
  }

  gStartMs = nowMs - gElapsedMs;
  gState = RunState::RUNNING;
}

// Pausa a contagem (congela tempo atual)
static void pauseStopwatch(uint32_t nowMs) {
  if (gState != RunState::RUNNING) {
    return;
  }

  gElapsedMs = nowMs - gStartMs;
  gState = RunState::PAUSED;
}

// Duplo clique: start / pause / resume
static void togglePauseResumeByDoubleClick(uint32_t nowMs) {
  switch (gState) {
    case RunState::IDLE:
      startOrResume(nowMs);
      break;

    case RunState::RUNNING:
      pauseStopwatch(nowMs);
      break;

    case RunState::PAUSED:
      startOrResume(nowMs);
      break;

    case RunState::FINISHED:
      // Sem ação para evitar reinício acidental após finalizar
      break;
  }
}

// Salva uma volta (tempo acumulado) se estiver em RUNNING.
// Também ativa popup de feedback visual.
static void saveLapMark(uint32_t nowMs) {
  if (gState != RunState::RUNNING) {
    return; // só marca volta enquanto está rodando
  }

  if (gLapCount >= kMaxLaps) {
    return; // limite atingido
  }

  const uint32_t currentElapsed = nowMs - gStartMs;
  gLapTimesMs[gLapCount] = currentElapsed;

  // Ativa popup visual da volta recém salva
  gLapSavedPopupLapIndex = gLapCount;
  gLapSavedPopupActive = true;
  gLapSavedPopupUntilMs = nowMs + kLapSavedPopupMs;

  gLapCount++;
}

// Finaliza a sessão e entra no modo de exibição dos popups das voltas.
static void finishAndShowLaps(uint32_t nowMs) {
  // Se estava rodando, congela o tempo final
  if (gState == RunState::RUNNING) {
    gElapsedMs = nowMs - gStartMs;
  }

  // Se estiver zerado e sem voltas, ignoramos o comando
  if (gState == RunState::IDLE && gLapCount == 0 && gElapsedMs == 0) {
    return;
  }

  gState = RunState::FINISHED;

  // Ao entrar no modo final, removemos popup de feedback (se existir)
  gLapSavedPopupActive = false;

  // Prepara ciclo dos popups finais
  gFinalPopupLapIndex = 0;
  gFinalPopupShowDelta = false; // começa por ACUM
  gNextFinalPopupMs = nowMs;    // mostra na próxima atualização do LCD
}

// Processa a sequência de cliques curtos após a janela de multi-clique expirar
// 1 clique = volta
// 2 cliques = start/pause/resume
// 3 cliques = finalizar e mostrar popups
static void processClickSequence(uint8_t clickCount, uint32_t nowMs) {
  switch (clickCount) {
    case 1:
      saveLapMark(nowMs);
      break;

    case 2:
      togglePauseResumeByDoubleClick(nowMs);
      break;

    case 3:
      finishAndShowLaps(nowMs);
      break;

    default:
      // 4+ cliques: ignorar para simplificar interface
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 11) MENSAGEM DE BOAS-VINDAS (NO SETUP)
// ─────────────────────────────────────────────────────────────────────────────
// O uso de delay() aqui é aceitável porque ocorre antes do cronômetro operar.
// Não afeta a precisão da medição no loop principal.
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
// 12) SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  // Inicializa o LCD 16x2
  lcd.begin(16, 2);

  // Configura botão e sensores como entrada com pull-up interno
  pinMode(kPinBtnControl, INPUT_PULLUP);

  pinMode(kPinSensorStart, INPUT_PULLUP);
  pinMode(kPinSensor1, INPUT_PULLUP);
  pinMode(kPinSensor2, INPUT_PULLUP);
  pinMode(kPinSensor3, INPUT_PULLUP);

  // Inicializa debounce
  inBtnControl.begin(kPinBtnControl, kDebounceButtonMs);

  inSensorStart.begin(kPinSensorStart, kDebounceSensorMs);
  for (uint8_t i = 0; i < kMarkSensorCount; ++i) {
    inMarkSensors[i].begin(kMarkSensorPins[i], kDebounceSensorMs);
  }

  // Mensagem de abertura (como você pediu)
  showStartMessage();

  // Estado inicial
  resetAll();
}

// ─────────────────────────────────────────────────────────────────────────────
// 13) LOOP PRINCIPAL (SEM BLOQUEIO)
// ─────────────────────────────────────────────────────────────────────────────
// Fluxo:
// A) Atualiza botão e sensores com debounce
// B) Trata clique longo (reset)
// C) Conta cliques curtos (1/2/3) e processa sequência
// D) Trata sensores (sensor0 start; sensores 1/2/3 marcam voltas)
// E) Atualiza tempo via millis()
// F) Atualiza LCD em taxa fixa
void loop() {
  const uint32_t nowMs = millis();

  // -------------------------------------------------------------------------
  // A) Atualizar entradas (debounce)
  // -------------------------------------------------------------------------
  inBtnControl.update(nowMs);
  inSensorStart.update(nowMs);

  for (uint8_t i = 0; i < kMarkSensorCount; ++i) {
    inMarkSensors[i].update(nowMs);
  }

  // -------------------------------------------------------------------------
  // B) Botão: pressão / soltura / clique longo (reset)
  // -------------------------------------------------------------------------
  // Ao pressionar, começamos a medir duração da pressão.
  if (inBtnControl.fell()) {
    gBtnPressStartMs = nowMs;
    gLongPressHandled = false;
  }

  // Se segurar por tempo suficiente, executa reset (uma única vez).
  if (inBtnControl.isPressed() && !gLongPressHandled) {
    if ((nowMs - gBtnPressStartMs) >= kLongPressMs) {
      resetAll();

      // Limpa qualquer sequência pendente para evitar mistura de eventos.
      gPendingClickCount = 0;
      gClickSequenceDeadlineMs = 0;

      gLongPressHandled = true;
    }
  }

  // Ao soltar:
  // - se não foi clique longo, conta como clique curto (pendente)
  // - se foi clique longo, ignora soltura
  if (inBtnControl.rose()) {
    if (!gLongPressHandled) {
      gPendingClickCount++;
      gClickSequenceDeadlineMs = nowMs + kMultiClickGapMs;
    }
  }

  // -------------------------------------------------------------------------
  // C) Processar sequência de cliques curtos (1 / 2 / 3)
  // -------------------------------------------------------------------------
  // Esperamos a janela de multi-clique expirar para saber se foi simples,
  // duplo ou triplo clique.
  if (gPendingClickCount > 0 && !inBtnControl.isPressed()) {
    if (nowMs >= gClickSequenceDeadlineMs) {
      processClickSequence(gPendingClickCount, nowMs);

      // Limpa sequência após processar
      gPendingClickCount = 0;
      gClickSequenceDeadlineMs = 0;
    }
  }

  // -------------------------------------------------------------------------
  // D) Sensores externos (reintegrados)
  // -------------------------------------------------------------------------
  // sensor0 -> inicia/retoma o cronômetro
  if (inSensorStart.fell()) {
    startOrResume(nowMs);
  }

  // sensores 1, 2 e 3 -> marcam voltas (iguais ao clique simples)
  for (uint8_t i = 0; i < kMarkSensorCount; ++i) {
    if (inMarkSensors[i].fell()) {
      saveLapMark(nowMs);
    }
  }

  // -------------------------------------------------------------------------
  // E) Atualizar tempo (precisão por diferença de millis)
  // -------------------------------------------------------------------------
  if (gState == RunState::RUNNING) {
    gElapsedMs = nowMs - gStartMs;
  }

  // -------------------------------------------------------------------------
  // F) Atualizar LCD em taxa fixa (evita flicker e uso excessivo)
  // -------------------------------------------------------------------------
  if ((nowMs - gLastLcdUpdateMs) < kLcdUpdateIntervalMs) {
    return;
  }
  gLastLcdUpdateMs = nowMs;

  // -------------------------------------------------------------------------
  // G) Escolher o que mostrar no LCD
  // -------------------------------------------------------------------------

  // 1) Modo finalizado: alterna popups ACUM / DELTA das voltas
  if (gState == RunState::FINISHED) {
    // Se não houver voltas salvas, mostra apenas o tempo final congelado
    if (gLapCount == 0) {
      renderMainScreen(gElapsedMs);
      return;
    }

    if (nowMs >= gNextFinalPopupMs) {
      renderFinalLapPopup(gFinalPopupLapIndex, gFinalPopupShowDelta);

      // Alternância:
      // ACUM -> DELTA -> próxima volta (ACUM)
      if (!gFinalPopupShowDelta) {
        gFinalPopupShowDelta = true;
      } else {
        gFinalPopupShowDelta = false;
        gFinalPopupLapIndex++;

        if (gFinalPopupLapIndex >= gLapCount) {
          gFinalPopupLapIndex = 0;
        }
      }

      gNextFinalPopupMs = nowMs + kFinalPopupCycleMs;
    }
    return;
  }

  // 2) Se houver popup de "volta salva" ativo, ele tem prioridade visual
  if (gLapSavedPopupActive) {
    if (nowMs >= gLapSavedPopupUntilMs) {
      gLapSavedPopupActive = false;
    } else {
      renderLapSavedPopup(gLapSavedPopupLapIndex);
      return;
    }
  }

  // 3) Tela normal (IDLE / RUNNING / PAUSED)
  renderMainScreen(gElapsedMs);
}