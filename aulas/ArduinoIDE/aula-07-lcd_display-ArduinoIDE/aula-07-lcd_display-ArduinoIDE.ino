/*
 * Curso de Formação de Professores – Atividades Experimentais de Física
 * Local: Parque da Ciência Newton Freire Maia (PR, Brasil)
 * Plataforma: Arduino
 * Ambiente alvo: Arduino IDE
 * Autor dos códigos originais: Aron da Rocha Battistella
 * Repositório: https://github.com/Dom-Aron/curso-arduino-parque-da-ciencia
 * Caminho no repositório: aulas/ArduinoIDE/aula-07-lcd_display-ArduinoIDE/aula-07-lcd_display-ArduinoIDE.ino
 * Data da última revisão: 17/04/2026
 *
 * Descrição:
 *   ============================================================
 *   Projeto   : Aula 07 - LCD Display
 *   Arquivo   : aula-07-lcd_display-ArduinoIDE.ino
 *   Pasta     : aula-07-lcd_display-ArduinoIDE
 *   Este sketch demonstra diferentes possibilidades de escrita e
 *   animação em um display LCD 16x2 via I2C usando a biblioteca
 *   New-LiquidCrystal.
 *
 *   A sequência automática inclui:
 *   - cartões de boas-vindas com referência ao Parque da Ciência
 *     e ao Lab Crie;
 *   - frase curta em movimento;
 *   - foguete cruzando a linha superior entre estrelas e planetas;
 *   - céu espacial animado;
 *   - estrelas em pixels com pisca do display;
 *   - movimento de onda;
 *   - múltiplas animações de abertura ao meio;
 *   - cartões finais com símbolos e frases curtas.
 *
 *   Observações:
 *   - O construtor expandido do LCD foi mantido, pois esse foi o
 *     padrão que funcionou corretamente no hardware testado.
 *   - As mensagens foram escritas sem acentos para melhorar a
 *     compatibilidade visual no display.
 *   ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

/*
  Configuração do LCD I2C compatível com a biblioteca
  New-LiquidCrystal e módulos backpack baseados em PCF8574.
*/
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

/* ------------------------- Constantes ------------------------- */

const uint8_t kLcdColumns = 16;
const uint8_t kLcdRows = 2;

const unsigned long kWelcomeDurationMs = 2200;
const unsigned long kScrollStepMs = 95;
const unsigned long kScrollHoldMs = 650;
const unsigned long kRocketStepMs = 120;
const unsigned long kRocketHoldMs = 700;
const unsigned long kSkyStepMs = 170;
const unsigned long kSkyHoldMs = 900;
const unsigned long kPixelStepMs = 135;
const unsigned long kPixelHoldMs = 900;
const unsigned long kWaveStepMs = 120;
const unsigned long kWaveHoldMs = 900;
const unsigned long kSplitStepMs = 135;
const unsigned long kSplitHoldMs = 1400;
const unsigned long kCurtainStepMs = 135;
const unsigned long kCurtainHoldMs = 1400;
const unsigned long kEchoStepMs = 130;
const unsigned long kEchoHoldMs = 1300;
const unsigned long kWingStepMs = 130;
const unsigned long kWingHoldMs = 1300;
const unsigned long kSymbolCardDurationMs = 1450;

const char kFlightText[] = "Lab Crie inspira   ";
const char kSplitLine0[] = "PQ. DA CIENCIA";
const char kSplitLine1[] = "LAB CRIE BRILHA";
const char kCurtainLine0[] = "PQ. DA CIENCIA";
const char kCurtainLine1[] = "LAB CRIE SURGE";
const char kEchoLine0[] = "ABRE DO MEIO";
const char kEchoLine1[] = "SABER EXPANDE";
const char kWingLine0[] = "PQ. DA CIENCIA";
const char kWingLine1[] = "LAB CRIE ABRE";

/* --------------------- Caracteres especiais --------------------- */

uint8_t kRocketChar[8] = {
  0b00100,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110,
  0b01010,
  0b10001
};

uint8_t kStarSmallChar[8] = {
  0b00000,
  0b00100,
  0b10101,
  0b01110,
  0b10101,
  0b00100,
  0b00000,
  0b00000
};

uint8_t kStarBigChar[8] = {
  0b00100,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};

uint8_t kPlanetChar[8] = {
  0b00110,
  0b01001,
  0b11111,
  0b11111,
  0b11111,
  0b10010,
  0b01100,
  0b00000
};

uint8_t kRingPlanetChar[8] = {
  0b00010,
  0b00111,
  0b11111,
  0b11111,
  0b11111,
  0b11100,
  0b01000,
  0b00000
};

uint8_t kAtomChar[8] = {
  0b00100,
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10001,
  0b00100,
  0b00000
};

uint8_t kFlaskChar[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b01010,
  0b01010,
  0b10001,
  0b11111,
  0b01110
};

uint8_t kCometChar[8] = {
  0b00001,
  0b00011,
  0b00111,
  0b01110,
  0b11100,
  0b01110,
  0b00100,
  0b00000
};

uint8_t kSparkleAChar[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b00000,
  0b00100,
  0b00000,
  0b00000,
  0b00000
};

uint8_t kSparkleBChar[8] = {
  0b00000,
  0b10000,
  0b00000,
  0b00010,
  0b00000,
  0b01000,
  0b00000,
  0b00000
};

uint8_t kSparkleCChar[8] = {
  0b00000,
  0b00001,
  0b00000,
  0b00100,
  0b00000,
  0b10000,
  0b00000,
  0b00000
};

uint8_t kWaveAChar[8] = {
  0b00000,
  0b00011,
  0b00110,
  0b01100,
  0b11000,
  0b10000,
  0b00000,
  0b00000
};

uint8_t kWaveBChar[8] = {
  0b00000,
  0b11000,
  0b01100,
  0b00110,
  0b00011,
  0b00001,
  0b00000,
  0b00000
};

uint8_t kWaveCChar[8] = {
  0b00000,
  0b00110,
  0b01100,
  0b11000,
  0b01100,
  0b00110,
  0b00000,
  0b00000
};

uint8_t kBlockChar[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

/* --------------------- Estruturas auxiliares --------------------- */

struct SymbolCard {
  uint8_t symbolCode;
  const char* topText;
  const char* bottomText;
};

enum DemoMode : uint8_t {
  MODE_WELCOME = 0,
  MODE_TEXT_FLIGHT,
  MODE_ROCKET_ORBIT,
  MODE_STAR_SKY,
  MODE_PIXEL_STARS,
  MODE_WAVES,
  MODE_SPLIT_OPEN,
  MODE_SPLIT_CURTAINS,
  MODE_SPLIT_ECHO,
  MODE_SPLIT_WINGS,
  MODE_SYMBOL_CARDS
};

/* --------------------- Estado global --------------------- */

DemoMode currentMode = MODE_WELCOME;
unsigned long modeStartMs = 0;
unsigned long lastStepMs = 0;
unsigned long holdStartMs = 0;
uint16_t animationIndex = 0;
bool modeInitialized = false;

const SymbolCard kSymbolCards[] = {
  {2, "BRILHO GUIA", "PQ. DA CIENCIA"},
  {3, "MUNDO CRIATIVO", "LAB CRIE ORBITA"},
  {4, "ANEIS NO CEU", "IDEIAS GIRAM"},
  {5, "ATOMO EM CENA", "FISICA VIVA"},
  {6, "LAB CRIE TESTA", "CIENCIA ATIVA"},
  {0, "FOGUETE PASSA", "RUMO AO SABER"}
};

const uint8_t kSymbolCardCount = sizeof(kSymbolCards) / sizeof(kSymbolCards[0]);

/* --------------------- Funções utilitárias --------------------- */

void loadSpaceCharacters() {
  lcd.createChar(0, kRocketChar);
  lcd.createChar(1, kStarSmallChar);
  lcd.createChar(2, kStarBigChar);
  lcd.createChar(3, kPlanetChar);
  lcd.createChar(4, kRingPlanetChar);
  lcd.createChar(5, kAtomChar);
  lcd.createChar(6, kFlaskChar);
  lcd.createChar(7, kCometChar);
}

void loadPixelCharacters() {
  lcd.createChar(0, kSparkleAChar);
  lcd.createChar(1, kSparkleBChar);
  lcd.createChar(2, kSparkleCChar);
  lcd.createChar(3, kWaveAChar);
  lcd.createChar(4, kWaveBChar);
  lcd.createChar(5, kWaveCChar);
  lcd.createChar(6, kBlockChar);
  lcd.createChar(7, kBlockChar);
}

void loadBlockCharacters() {
  lcd.createChar(0, kBlockChar);
  lcd.createChar(1, kBlockChar);
  lcd.createChar(2, kBlockChar);
  lcd.createChar(3, kBlockChar);
  lcd.createChar(4, kBlockChar);
  lcd.createChar(5, kBlockChar);
  lcd.createChar(6, kBlockChar);
  lcd.createChar(7, kBlockChar);
}

void clearRow(uint8_t row) {
  lcd.setCursor(0, row);
  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    lcd.print(' ');
  }
}

void printCentered(uint8_t row, const char* text) {
  const size_t textLength = strlen(text);
  const uint8_t visibleLength = (textLength > kLcdColumns)
    ? kLcdColumns
    : static_cast<uint8_t>(textLength);
  const uint8_t startColumn = (kLcdColumns - visibleLength) / 2;

  clearRow(row);
  lcd.setCursor(startColumn, row);

  for (uint8_t index = 0; index < visibleLength; ++index) {
    lcd.print(text[index]);
  }
}

void printTrimmed(uint8_t column, uint8_t row, const char* text, uint8_t maxLength) {
  lcd.setCursor(column, row);

  for (uint8_t index = 0; index < maxLength; ++index) {
    if (text[index] == '\0') {
      break;
    }
    lcd.print(text[index]);
  }
}

void putSymbol(uint8_t column, uint8_t row, uint8_t symbolCode) {
  if (column >= kLcdColumns || row >= kLcdRows) {
    return;
  }

  lcd.setCursor(column, row);
  lcd.write(static_cast<uint8_t>(symbolCode));
}

char charAtOrSpace(const char* text, uint8_t index) {
  const size_t textLength = strlen(text);
  if (index < textLength) {
    return text[index];
  }
  return ' ';
}

void renderSlidingText(const char* text, uint8_t row, uint16_t offset) {
  const size_t textLength = strlen(text);
  lcd.setCursor(0, row);

  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    const int16_t sourceIndex =
      static_cast<int16_t>(offset) +
      static_cast<int16_t>(column) -
      static_cast<int16_t>(kLcdColumns);

    if (sourceIndex >= 0 && sourceIndex < static_cast<int16_t>(textLength)) {
      lcd.print(text[sourceIndex]);
    } else {
      lcd.print(' ');
    }
  }
}

void renderSplitRevealLine(uint8_t row, const char* text, uint8_t frame) {
  const int8_t leftOpen = 7 - static_cast<int8_t>(frame);
  const int8_t rightOpen = 8 + static_cast<int8_t>(frame);

  lcd.setCursor(0, row);
  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    if (column <= leftOpen || column >= rightOpen) {
      lcd.write(static_cast<uint8_t>(0));
    } else {
      lcd.print(charAtOrSpace(text, column));
    }
  }
}

void renderCurtainLine(uint8_t row, const char* text, uint8_t frame) {
  const int8_t leftCurtain = 7 - static_cast<int8_t>(frame);
  const int8_t rightCurtain = 8 + static_cast<int8_t>(frame);

  lcd.setCursor(0, row);
  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    if (column == leftCurtain || column == rightCurtain) {
      lcd.write(static_cast<uint8_t>(0));
    } else if (column > leftCurtain && column < rightCurtain) {
      lcd.print(charAtOrSpace(text, column));
    } else {
      lcd.print(' ');
    }
  }
}

void renderCenterOutLine(uint8_t row, const char* text, uint8_t frame) {
  const int8_t leftLimit = 7 - static_cast<int8_t>(frame);
  const int8_t rightLimit = 8 + static_cast<int8_t>(frame);

  lcd.setCursor(0, row);
  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    if (column > leftLimit && column < rightLimit) {
      lcd.print(charAtOrSpace(text, column));
    } else {
      lcd.print(' ');
    }
  }
}

void renderWingLine(uint8_t row, const char* text, uint8_t frame) {
  const int8_t leftWing = 7 - static_cast<int8_t>(frame);
  const int8_t rightWing = 8 + static_cast<int8_t>(frame);

  lcd.setCursor(0, row);
  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    if (column == leftWing || column == rightWing) {
      lcd.write(static_cast<uint8_t>(2));
    } else if (column > leftWing && column < rightWing) {
      lcd.print(charAtOrSpace(text, column));
    } else {
      lcd.print(' ');
    }
  }
}

void setMode(DemoMode nextMode) {
  lcd.noAutoscroll();
  lcd.display();
  lcd.backlight();
  currentMode = nextMode;
  modeStartMs = millis();
  lastStepMs = 0;
  holdStartMs = 0;
  animationIndex = 0;
  modeInitialized = false;
}

/* --------------------- Boas-vindas --------------------- */

void enterWelcomeMode() {
  loadSpaceCharacters();
  lcd.clear();
  printCentered(0, "PQ. DA CIENCIA");
  lcd.setCursor(3, 1);
  lcd.write(static_cast<uint8_t>(2));
  lcd.print(" LAB CRIE ");
  lcd.write(static_cast<uint8_t>(2));
  modeInitialized = true;
}

void updateWelcomeMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterWelcomeMode();
  }

  if (nowMs - modeStartMs < kWelcomeDurationMs / 2) {
    return;
  }

  if (animationIndex == 0) {
    lcd.clear();
    printCentered(0, "LAB CRIE");
    lcd.setCursor(2, 1);
    lcd.write(static_cast<uint8_t>(5));
    lcd.print(" ciencia ");
    lcd.write(static_cast<uint8_t>(6));
    animationIndex = 1;
  }

  if (nowMs - modeStartMs >= kWelcomeDurationMs) {
    setMode(MODE_TEXT_FLIGHT);
  }
}

/* --------------------- Texto em movimento --------------------- */

void enterTextFlightMode() {
  loadSpaceCharacters();
  lcd.clear();
  printCentered(0, "Mensagem em voo");
  renderSlidingText(kFlightText, 1, 0);
  modeInitialized = true;
}

void updateTextFlightMode(unsigned long nowMs) {
  const size_t totalFrames = strlen(kFlightText) + kLcdColumns + 1;

  if (!modeInitialized) {
    enterTextFlightMode();
  }

  if (nowMs - lastStepMs < kScrollStepMs) {
    return;
  }

  lastStepMs = nowMs;
  renderSlidingText(kFlightText, 1, animationIndex);
  ++animationIndex;

  if (animationIndex > totalFrames) {
    if (holdStartMs == 0) {
      holdStartMs = nowMs;
    }

    if (nowMs - holdStartMs >= kScrollHoldMs) {
      setMode(MODE_ROCKET_ORBIT);
    }
  }
}

/* --------------------- Foguete orbital --------------------- */

void renderRocketOrbitFrame(uint8_t frame) {
  loadSpaceCharacters();
  lcd.clear();

  putSymbol(3, 0, 1);
  putSymbol(7, 0, 2);
  putSymbol(11, 0, 3);
  putSymbol(14, 0, 4);

  if (frame < kLcdColumns) {
    putSymbol(frame, 0, 0);
  }

  if (frame < 6) {
    printCentered(1, "Lab Crie no ar");
  } else if (frame < 11) {
    printCentered(1, "PQ. DA CIENCIA");
  } else {
    printCentered(1, "Rumo ao saber");
  }

  if ((frame % 3) == 0) {
    putSymbol(1, 1, 1);
    putSymbol(13, 1, 7);
  } else if ((frame % 3) == 1) {
    putSymbol(2, 1, 2);
    putSymbol(12, 1, 1);
  } else {
    putSymbol(0, 1, 7);
    putSymbol(15, 1, 2);
  }
}

void enterRocketOrbitMode() {
  renderRocketOrbitFrame(0);
  modeInitialized = true;
}

void updateRocketOrbitMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterRocketOrbitMode();
  }

  if (animationIndex < 18) {
    if (nowMs - lastStepMs < kRocketStepMs) {
      return;
    }

    lastStepMs = nowMs;
    renderRocketOrbitFrame(animationIndex);
    ++animationIndex;
    return;
  }

  if (holdStartMs == 0) {
    holdStartMs = nowMs;
  }

  if (nowMs - holdStartMs >= kRocketHoldMs) {
    setMode(MODE_STAR_SKY);
  }
}

/* --------------------- Ceu espacial --------------------- */

void renderStarSkyFrame(uint8_t frame) {
  loadSpaceCharacters();
  lcd.clear();

  uint8_t cometColumn = frame % kLcdColumns;
  uint8_t ringColumn = static_cast<uint8_t>((frame + 6) % kLcdColumns);
  uint8_t planetColumn = static_cast<uint8_t>((frame + 10) % kLcdColumns);

  putSymbol(1, 0, 2);
  putSymbol(5, 0, 1);
  putSymbol(9, 0, 2);
  putSymbol(13, 0, 1);

  putSymbol(3, 1, 1);
  putSymbol(8, 1, 2);
  putSymbol(12, 1, 1);

  putSymbol(cometColumn, 0, 7);
  putSymbol(ringColumn, 1, 4);
  putSymbol(planetColumn, 1, 3);

  if ((frame % 5) == 2) {
    lcd.noDisplay();
  } else {
    lcd.display();
  }
}

void enterStarSkyMode() {
  renderStarSkyFrame(0);
  modeInitialized = true;
}

void updateStarSkyMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterStarSkyMode();
  }

  if (animationIndex < 14) {
    if (nowMs - lastStepMs < kSkyStepMs) {
      return;
    }

    lastStepMs = nowMs;
    renderStarSkyFrame(animationIndex);
    ++animationIndex;
    return;
  }

  if (holdStartMs == 0) {
    holdStartMs = nowMs;
    lcd.display();
  }

  if (nowMs - holdStartMs >= kSkyHoldMs) {
    setMode(MODE_PIXEL_STARS);
  }
}

/* --------------------- Estrelas em pixels --------------------- */

void renderPixelStarsFrame(uint8_t frame) {
  loadPixelCharacters();
  lcd.clear();
  printCentered(0, "CEU ESTRELADO");

  switch (frame % 6) {
    case 0:
      putSymbol(1, 1, 0);
      putSymbol(5, 1, 1);
      putSymbol(10, 1, 2);
      putSymbol(14, 1, 0);
      break;

    case 1:
      putSymbol(3, 1, 2);
      putSymbol(7, 1, 0);
      putSymbol(11, 1, 1);
      putSymbol(15, 1, 2);
      break;

    case 2:
      putSymbol(0, 1, 1);
      putSymbol(4, 1, 0);
      putSymbol(9, 1, 2);
      putSymbol(13, 1, 1);
      break;

    case 3:
      putSymbol(2, 1, 2);
      putSymbol(6, 1, 1);
      putSymbol(12, 1, 0);
      putSymbol(15, 1, 2);
      break;

    case 4:
      putSymbol(1, 1, 1);
      putSymbol(8, 1, 0);
      putSymbol(10, 1, 2);
      putSymbol(14, 1, 1);
      break;

    default:
      putSymbol(0, 1, 2);
      putSymbol(5, 1, 0);
      putSymbol(9, 1, 1);
      putSymbol(13, 1, 2);
      break;
  }

  if ((frame % 4) == 1) {
    lcd.noDisplay();
  } else {
    lcd.display();
  }
}

void enterPixelStarsMode() {
  renderPixelStarsFrame(0);
  modeInitialized = true;
}

void updatePixelStarsMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterPixelStarsMode();
  }

  if (animationIndex < 12) {
    if (nowMs - lastStepMs < kPixelStepMs) {
      return;
    }

    lastStepMs = nowMs;
    renderPixelStarsFrame(animationIndex);
    ++animationIndex;
    return;
  }

  if (holdStartMs == 0) {
    holdStartMs = nowMs;
    lcd.display();
  }

  if (nowMs - holdStartMs >= kPixelHoldMs) {
    setMode(MODE_WAVES);
  }
}

/* --------------------- Ondas --------------------- */

void renderWaveFrame(uint8_t frame) {
  loadPixelCharacters();
  lcd.clear();

  lcd.setCursor(0, 0);
  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    const uint8_t phase = static_cast<uint8_t>((column + frame) % 3);
    lcd.write(static_cast<uint8_t>(3 + phase));
  }

  printCentered(1, (frame % 2 == 0) ? "PQ. DA CIENCIA" : "LAB CRIE");
}

void enterWaveMode() {
  renderWaveFrame(0);
  modeInitialized = true;
}

void updateWaveMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterWaveMode();
  }

  if (animationIndex < 12) {
    if (nowMs - lastStepMs < kWaveStepMs) {
      return;
    }

    lastStepMs = nowMs;
    renderWaveFrame(animationIndex);
    ++animationIndex;
    return;
  }

  if (holdStartMs == 0) {
    holdStartMs = nowMs;
  }

  if (nowMs - holdStartMs >= kWaveHoldMs) {
    setMode(MODE_SPLIT_OPEN);
  }
}

/* --------------------- Abertura ao meio --------------------- */

void renderSplitOpenFrame(uint8_t frame) {
  loadBlockCharacters();
  lcd.clear();
  renderSplitRevealLine(0, kSplitLine0, frame);
  renderSplitRevealLine(1, kSplitLine1, frame);
}

void enterSplitOpenMode() {
  renderSplitOpenFrame(0);
  modeInitialized = true;
}

void updateSplitOpenMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterSplitOpenMode();
  }

  if (animationIndex < 8) {
    if (nowMs - lastStepMs < kSplitStepMs) {
      return;
    }

    lastStepMs = nowMs;
    renderSplitOpenFrame(animationIndex);
    ++animationIndex;
    return;
  }

  renderSplitOpenFrame(8);

  if (holdStartMs == 0) {
    holdStartMs = nowMs;
  }

  if (nowMs - holdStartMs >= kSplitHoldMs) {
    setMode(MODE_SPLIT_CURTAINS);
  }
}

void renderSplitCurtainsFrame(uint8_t frame) {
  loadBlockCharacters();
  lcd.clear();
  renderCurtainLine(0, kCurtainLine0, frame);
  renderCurtainLine(1, kCurtainLine1, frame);
}

void enterSplitCurtainsMode() {
  renderSplitCurtainsFrame(0);
  modeInitialized = true;
}

void updateSplitCurtainsMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterSplitCurtainsMode();
  }

  if (animationIndex < 8) {
    if (nowMs - lastStepMs < kCurtainStepMs) {
      return;
    }

    lastStepMs = nowMs;
    renderSplitCurtainsFrame(animationIndex);
    ++animationIndex;
    return;
  }

  renderSplitCurtainsFrame(8);

  if (holdStartMs == 0) {
    holdStartMs = nowMs;
  }

  if (nowMs - holdStartMs >= kCurtainHoldMs) {
    setMode(MODE_SPLIT_ECHO);
  }
}

void renderSplitEchoFrame(uint8_t frame) {
  loadSpaceCharacters();
  lcd.clear();
  renderCenterOutLine(0, kEchoLine0, frame);
  renderCenterOutLine(1, kEchoLine1, frame);

  if (frame < 8) {
    putSymbol(static_cast<uint8_t>(7 - frame), 0, 2);
    putSymbol(static_cast<uint8_t>(8 + frame), 1, 1);
  }
}

void enterSplitEchoMode() {
  renderSplitEchoFrame(0);
  modeInitialized = true;
}

void updateSplitEchoMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterSplitEchoMode();
  }

  if (animationIndex < 8) {
    if (nowMs - lastStepMs < kEchoStepMs) {
      return;
    }

    lastStepMs = nowMs;
    renderSplitEchoFrame(animationIndex);
    ++animationIndex;
    return;
  }

  renderSplitEchoFrame(8);

  if (holdStartMs == 0) {
    holdStartMs = nowMs;
  }

  if (nowMs - holdStartMs >= kEchoHoldMs) {
    setMode(MODE_SPLIT_WINGS);
  }
}

void renderSplitWingsFrame(uint8_t frame) {
  loadSpaceCharacters();
  lcd.clear();
  renderWingLine(0, kWingLine0, frame);
  renderWingLine(1, kWingLine1, frame);
}

void enterSplitWingsMode() {
  renderSplitWingsFrame(0);
  modeInitialized = true;
}

void updateSplitWingsMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterSplitWingsMode();
  }

  if (animationIndex < 8) {
    if (nowMs - lastStepMs < kWingStepMs) {
      return;
    }

    lastStepMs = nowMs;
    renderSplitWingsFrame(animationIndex);
    ++animationIndex;
    return;
  }

  renderSplitWingsFrame(8);

  if (holdStartMs == 0) {
    holdStartMs = nowMs;
  }

  if (nowMs - holdStartMs >= kWingHoldMs) {
    setMode(MODE_SYMBOL_CARDS);
  }
}

/* --------------------- Cartoes com simbolos --------------------- */

void renderSymbolCard(uint8_t cardIndex) {
  loadSpaceCharacters();
  lcd.clear();

  const SymbolCard& card = kSymbolCards[cardIndex];
  putSymbol(0, 0, card.symbolCode);
  printTrimmed(2, 0, card.topText, 14);
  printCentered(1, card.bottomText);
}

void enterSymbolCardsMode() {
  renderSymbolCard(0);
  modeInitialized = true;
}

void updateSymbolCardsMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterSymbolCardsMode();
  }

  const unsigned long elapsedMs = nowMs - modeStartMs;
  const uint8_t cardIndex = static_cast<uint8_t>(elapsedMs / kSymbolCardDurationMs);

  if (cardIndex < kSymbolCardCount && cardIndex != animationIndex) {
    animationIndex = cardIndex;
    renderSymbolCard(cardIndex);
  }

  if (elapsedMs >= static_cast<unsigned long>(kSymbolCardCount) * kSymbolCardDurationMs) {
    setMode(MODE_WELCOME);
  }
}

/* --------------------- Setup e loop --------------------- */

void setup() {
  lcd.begin(kLcdColumns, kLcdRows);
  lcd.backlight();
  loadSpaceCharacters();
  setMode(MODE_WELCOME);
}

void loop() {
  const unsigned long nowMs = millis();

  switch (currentMode) {
    case MODE_WELCOME:
      updateWelcomeMode(nowMs);
      break;

    case MODE_TEXT_FLIGHT:
      updateTextFlightMode(nowMs);
      break;

    case MODE_ROCKET_ORBIT:
      updateRocketOrbitMode(nowMs);
      break;

    case MODE_STAR_SKY:
      updateStarSkyMode(nowMs);
      break;

    case MODE_PIXEL_STARS:
      updatePixelStarsMode(nowMs);
      break;

    case MODE_WAVES:
      updateWaveMode(nowMs);
      break;

    case MODE_SPLIT_OPEN:
      updateSplitOpenMode(nowMs);
      break;

    case MODE_SPLIT_CURTAINS:
      updateSplitCurtainsMode(nowMs);
      break;

    case MODE_SPLIT_ECHO:
      updateSplitEchoMode(nowMs);
      break;

    case MODE_SPLIT_WINGS:
      updateSplitWingsMode(nowMs);
      break;

    case MODE_SYMBOL_CARDS:
      updateSymbolCardsMode(nowMs);
      break;
  }
}
