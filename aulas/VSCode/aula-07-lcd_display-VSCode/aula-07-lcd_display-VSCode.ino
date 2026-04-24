#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

/*
 * Curso de Formação de Professores – Atividades Experimentais de Física
 * Local: Parque da Ciência Newton Freire Maia (PR, Brasil)
 * Plataforma: Arduino
 * Ambiente alvo: VS Code
 * Autor dos códigos originais: Aron da Rocha Battistella
 * Repositório: https://github.com/Dom-Aron/curso-arduino-parque-da-ciencia
 * Caminho no repositório: aulas/VSCode/aula-07-lcd_display-VSCode/aula-07-lcd_display-VSCode.ino
 * Data da última revisão: 17/04/2026
 *
 * Descrição:
 *   ============================================================
 *   Projeto   : Aula 07 - LCD Display
 *   Arquivo   : aula-07-lcd_display-VSCode.ino
 *   Pasta     : aula-07-lcd_display-VSCode
 *   Este sketch demonstra diferentes possibilidades de escrita e
 *   animação em um display LCD 16x2 com interface I2C usando a
 *   biblioteca New-LiquidCrystal.
 *
 *   A sequência automática inclui:
 *   - cartões de boas-vindas com referências ao Parque da Ciência
 *     e ao Lab Crie;
 *   - frases curtas em movimento;
 *   - voo espacial do foguete na linha superior, atravessando
 *     estrelas e planetas;
 *   - ceu estrelado animado, com estrelas, cometa e planetas;
 *   - campo de estrelas em pixels, com pisca do display;
 *   - movimento de onda;
 *   - variações de abertura ao meio, como cortinas e divisão do
 *     display em duas metades;
 *   - exibição final de símbolos acompanhados de frases curtas.
 *
 *   Observações:
 *   - O construtor do LCD foi mantido no formato expandido, pois
 *     esse foi o padrão que funcionou corretamente no display
 *     testado pelo autor.
 *   - As mensagens do LCD foram escritas sem acentos para manter
 *     melhor compatibilidade visual com o display.
 *   ============================================================
 */

/*
  Configuração do LCD I2C compatível com a biblioteca
  New-LiquidCrystal e módulos backpack baseados em PCF8574.
*/
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

/* ------------------------- Constantes ------------------------- */

const uint8_t kLcdColumns = 16;
const uint8_t kLcdRows = 2;

const unsigned long kWelcomeDurationMs = 2200;
const unsigned long kScrollStepMs = 105;
const unsigned long kScrollHoldMs = 450;
const unsigned long kRocketStepMs = 95;
const unsigned long kStarSkyStepMs = 175;
const unsigned long kStarsStepMs = 135;
const unsigned long kWaveStepMs = 120;
const unsigned long kSplitStepMs = 150;
const unsigned long kCurtainStepMs = 145;
const unsigned long kSplitEchoStepMs = 150;
const unsigned long kSymbolCardDurationMs = 1500;

const char kFlightText[] = "Lab Crie em orbita   ";
const char kSplitLine0[] = "PQ. DA CIENCIA! ";
const char kSplitLine1[] = "LAB CRIE BRILHA!";
const char kCurtainLine0[] = "ABRE AO MEIO....";
const char kCurtainLine1[] = "VAI AOS LADOS!";
const char kEchoLine0[] = "PARQUE DA";
const char kEchoLine1[] = "LAB CRIE";

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

/* --------------------- Estruturas e estado --------------------- */

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
  MODE_SYMBOL_CARDS
};

DemoMode currentMode = MODE_WELCOME;

unsigned long modeStartMs = 0;
unsigned long lastStepMs = 0;
unsigned long holdStartMs = 0;
uint16_t animationIndex = 0;
bool modeInitialized = false;

const SymbolCard kSymbolCards[] = {
  {2, "ESTRELA GUIA", "BRILHO NO LCD"},
  {4, "PLANETA EM GIRO", "ORBITA ELIPTICA"},
  {0, "FOGUETE NO AR", "LAB CRIE VOA"},
  {5, "ATOMO EM CENA", "FISICA VIVA"},
  {6, "LAB EM ACAO", "CIENCIA NA MAO"},
  {7, "COMETA PASSOU", "PARQUE BRILHOU"}
};

const uint8_t kSymbolCardCount = sizeof(kSymbolCards) / sizeof(kSymbolCards[0]);

/* --------------------- Funções utilitárias --------------------- */

/*!
 * @brief Carrega o conjunto de caracteres da cena espacial.
 */
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

/*!
 * @brief Carrega o conjunto de caracteres de movimento.
 */
void loadMotionCharacters() {
  lcd.createChar(0, kSparkleAChar);
  lcd.createChar(1, kSparkleBChar);
  lcd.createChar(2, kSparkleCChar);
  lcd.createChar(3, kWaveAChar);
  lcd.createChar(4, kWaveBChar);
  lcd.createChar(5, kBlockChar);
  lcd.createChar(6, kBlockChar);
  lcd.createChar(7, kBlockChar);
}

/*!
 * @brief Limpa completamente uma linha do display.
 *
 * @param row Linha a ser apagada.
 */
void clearRow(uint8_t row) {
  lcd.setCursor(0, row);

  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    lcd.print(' ');
  }
}

/*!
 * @brief Escreve um texto centralizado em uma linha do LCD.
 *
 * @param row Linha em que o texto será exibido.
 * @param text Texto a ser centralizado.
 */
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

/*!
 * @brief Posiciona um símbolo personalizado no display.
 *
 * @param column Coluna de destino.
 * @param row Linha de destino.
 * @param symbolCode Código do caractere personalizado.
 */
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

/*!
 * @brief Renderiza um texto deslizante da esquerda para a direita.
 *
 * @param text Texto base da animação.
 * @param row Linha em que o texto será exibido.
 * @param offset Quadro atual da animação.
 */
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

/*!
 * @brief Preenche uma linha com blocos, exceto em uma janela central.
 *
 * @param row Linha a ser desenhada.
 * @param text Texto revelado pela abertura.
 * @param frame Quadro atual da animação.
 */
void renderSplitRevealLine(uint8_t row, const char* text, uint8_t frame) {
  const int8_t leftOpen = 7 - static_cast<int8_t>(frame);
  const int8_t rightOpen = 8 + static_cast<int8_t>(frame);

  lcd.setCursor(0, row);

  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    if (column <= leftOpen || column >= rightOpen) {
      lcd.write(static_cast<uint8_t>(5));
    } else {
      lcd.print(charAtOrSpace(text, column));
    }
  }
}

/*!
 * @brief Preenche uma linha com cortinas que saem do centro.
 *
 * @param row Linha a ser desenhada.
 * @param text Texto a ser revelado.
 * @param frame Quadro atual.
 */
void renderCurtainLine(uint8_t row, const char* text, uint8_t frame) {
  const int8_t leftCurtain = 7 - static_cast<int8_t>(frame);
  const int8_t rightCurtain = 8 + static_cast<int8_t>(frame);

  lcd.setCursor(0, row);

  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    if (column == leftCurtain || column == rightCurtain) {
      lcd.write(static_cast<uint8_t>(5));
    } else if (column > leftCurtain && column < rightCurtain) {
      lcd.print(charAtOrSpace(text, column));
    } else {
      lcd.print(' ');
    }
  }
}

/*!
 * @brief Troca o modo atual da demonstração.
 *
 * @param nextMode Próximo modo a ser executado.
 */
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

/* --------------------- Cartões de boas-vindas --------------------- */

void enterWelcomeMode() {
  loadSpaceCharacters();
  lcd.clear();
  printCentered(0, "PARQUE DA");

  lcd.setCursor(0, 1);
  lcd.write(static_cast<uint8_t>(2));
  lcd.print(" CIENCIA ");
  lcd.write(static_cast<uint8_t>(2));
  lcd.print("LC");

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
    printCentered(0, "Lab Crie");
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

/* --------------------- Frase em movimento --------------------- */

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

/* --------------------- Voo espacial na linha superior --------------------- */

void renderRocketOrbitFrame(uint8_t frame) {
  loadSpaceCharacters();
  lcd.clear();

  putSymbol(4, 0, 1);
  putSymbol(8, 0, 2);
  putSymbol(11, 0, 3);
  putSymbol(14, 0, 4);

  if (frame < 6) {
    printCentered(1, "Lab Crie no ar");
  } else if (frame < 11) {
    printCentered(1, "PQ DA CIENCIA ");
  } else {
    printCentered(1, "Rumo ao saber!");
  }

  if ((frame % 4) == 1) {
    putSymbol(1, 1, 1);
    putSymbol(12, 1, 7);
  } else if ((frame % 4) == 2) {
    putSymbol(2, 1, 2);
    putSymbol(13, 1, 1);
  } else {
    putSymbol(0, 1, 7);
    putSymbol(15, 1, 2);
  }

  if (frame < kLcdColumns) {
    putSymbol(frame, 0, 0);
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

  if (nowMs - lastStepMs < kRocketStepMs) {
    return;
  }

  lastStepMs = nowMs;
  renderRocketOrbitFrame(animationIndex);
  ++animationIndex;

  if (animationIndex >= 18) {
    setMode(MODE_STAR_SKY);
  }
}

/* --------------------- Ceu estrelado classico --------------------- */

void renderStarSkyFrame(uint8_t frame) {
  loadSpaceCharacters();
  lcd.clear();

  const uint8_t cometColumn = static_cast<uint8_t>(frame % kLcdColumns);
  const uint8_t ringPlanetColumn = static_cast<uint8_t>(12 - ((frame / 2) % 3));
  const uint8_t planetColumn = static_cast<uint8_t>(3 + ((frame / 3) % 2));

  if ((frame % 2) == 0) {
    putSymbol(1, 0, 1);
    putSymbol(5, 0, 2);
    putSymbol(9, 0, 1);
    putSymbol(14, 0, 2);
    putSymbol(0, 1, 2);
    putSymbol(6, 1, 1);
    putSymbol(10, 1, 2);
    putSymbol(15, 1, 1);
  } else {
    putSymbol(0, 0, 2);
    putSymbol(4, 0, 1);
    putSymbol(8, 0, 2);
    putSymbol(13, 0, 1);
    putSymbol(2, 1, 1);
    putSymbol(7, 1, 2);
    putSymbol(11, 1, 1);
    putSymbol(14, 1, 2);
  }

  putSymbol(planetColumn, 1, 3);
  putSymbol(ringPlanetColumn, 0, 4);
  putSymbol(cometColumn, 1, 7);

  if ((frame % 6) == 3) {
    lcd.noDisplay();
  } else {
    lcd.display();
  }
}

void enterStarSkyMode() {
  lcd.display();
  renderStarSkyFrame(0);
  modeInitialized = true;
}

void updateStarSkyMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterStarSkyMode();
  }

  if (nowMs - lastStepMs < kStarSkyStepMs) {
    return;
  }

  lastStepMs = nowMs;
  renderStarSkyFrame(animationIndex);
  ++animationIndex;

  if (animationIndex >= 18) {
    lcd.display();
    setMode(MODE_PIXEL_STARS);
  }
}

/* --------------------- Estrelas em pixels --------------------- */

void renderPixelStarsFrame(uint8_t frame) {
  loadMotionCharacters();
  lcd.clear();
  printCentered(0, "ceu em pixels");

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
      putSymbol(8, 1, 2);
      putSymbol(10, 1, 0);
      putSymbol(14, 1, 1);
      break;

    default:
      putSymbol(3, 1, 0);
      putSymbol(5, 1, 2);
      putSymbol(9, 1, 1);
      putSymbol(12, 1, 2);
      break;
  }
}

void enterPixelStarsMode() {
  lcd.display();
  renderPixelStarsFrame(0);
  modeInitialized = true;
}

void updatePixelStarsMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterPixelStarsMode();
  }

  if (nowMs - lastStepMs < kStarsStepMs) {
    return;
  }

  lastStepMs = nowMs;

  if ((animationIndex % 5) == 4) {
    lcd.noDisplay();
  } else {
    lcd.display();
    renderPixelStarsFrame(animationIndex);
  }

  ++animationIndex;

  if (animationIndex >= 22) {
    lcd.display();
    setMode(MODE_WAVES);
  }
}

/* --------------------- Movimento de onda --------------------- */

void renderWaveFrame(uint8_t frame) {
  loadMotionCharacters();
  lcd.clear();

  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    const bool topUseWaveA = ((column + frame) % 4) < 2;
    const bool bottomUseWaveA = ((column + frame + 2) % 4) < 2;

    putSymbol(column, 0, topUseWaveA ? 3 : 4);
    putSymbol(column, 1, bottomUseWaveA ? 4 : 3);
  }
}

void enterWavesMode() {
  renderWaveFrame(0);
  modeInitialized = true;
}

void updateWavesMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterWavesMode();
  }

  if (nowMs - lastStepMs < kWaveStepMs) {
    return;
  }

  lastStepMs = nowMs;
  renderWaveFrame(animationIndex);
  ++animationIndex;

  if (animationIndex >= 18) {
    setMode(MODE_SPLIT_OPEN);
  }
}

/* --------------------- Abertura ao meio --------------------- */

void enterSplitOpenMode() {
  loadMotionCharacters();
  lcd.clear();
  renderSplitRevealLine(0, kSplitLine0, 0);
  renderSplitRevealLine(1, kSplitLine1, 0);
  modeInitialized = true;
}

void updateSplitOpenMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterSplitOpenMode();
  }

  if (nowMs - lastStepMs < kSplitStepMs) {
    return;
  }

  lastStepMs = nowMs;
  renderSplitRevealLine(0, kSplitLine0, animationIndex);
  renderSplitRevealLine(1, kSplitLine1, animationIndex);
  ++animationIndex;

  if (animationIndex >= 8) {
    setMode(MODE_SPLIT_CURTAINS);
  }
}

/* --------------------- Cortinas saindo do centro --------------------- */

void enterSplitCurtainsMode() {
  loadMotionCharacters();
  lcd.clear();
  renderCurtainLine(0, kCurtainLine0, 0);
  renderCurtainLine(1, kCurtainLine1, 0);
  modeInitialized = true;
}

void updateSplitCurtainsMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterSplitCurtainsMode();
  }

  if (nowMs - lastStepMs < kCurtainStepMs) {
    return;
  }

  lastStepMs = nowMs;
  renderCurtainLine(0, kCurtainLine0, animationIndex);
  renderCurtainLine(1, kCurtainLine1, animationIndex);
  ++animationIndex;

  if (animationIndex >= 8) {
    setMode(MODE_SPLIT_ECHO);
  }
}

/* --------------------- Divisão espelhada em duas metades --------------------- */

void renderSplitEchoFrame(uint8_t frame) {
  loadMotionCharacters();
  lcd.clear();

  for (uint8_t column = 0; column < kLcdColumns; ++column) {
    lcd.setCursor(column, 0);
    if (column < 8) {
      if (column <= frame) {
        lcd.print(kEchoLine0[column % strlen(kEchoLine0)]);
      } else {
        lcd.write(static_cast<uint8_t>(5));
      }
    } else {
      const uint8_t mirroredIndex = static_cast<uint8_t>(15 - column);
      if (mirroredIndex <= frame) {
        lcd.print(kEchoLine0[mirroredIndex % strlen(kEchoLine0)]);
      } else {
        lcd.write(static_cast<uint8_t>(5));
      }
    }

    lcd.setCursor(column, 1);
    if (column < 8) {
      if (column <= frame) {
        lcd.print(kEchoLine1[column % strlen(kEchoLine1)]);
      } else {
        lcd.write(static_cast<uint8_t>(5));
      }
    } else {
      const uint8_t mirroredIndex = static_cast<uint8_t>(15 - column);
      if (mirroredIndex <= frame) {
        lcd.print(kEchoLine1[mirroredIndex % strlen(kEchoLine1)]);
      } else {
        lcd.write(static_cast<uint8_t>(5));
      }
    }
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

  if (nowMs - lastStepMs < kSplitEchoStepMs) {
    return;
  }

  lastStepMs = nowMs;
  renderSplitEchoFrame(animationIndex);
  ++animationIndex;

  if (animationIndex >= 8) {
    setMode(MODE_SYMBOL_CARDS);
  }
}

/* --------------------- Símbolos com frases --------------------- */

void showSymbolCard(uint8_t cardIndex) {
  loadSpaceCharacters();
  lcd.clear();
  printCentered(0, kSymbolCards[cardIndex].topText);
  printCentered(1, kSymbolCards[cardIndex].bottomText);
  putSymbol(0, 0, kSymbolCards[cardIndex].symbolCode);
  putSymbol(15, 1, kSymbolCards[cardIndex].symbolCode);
}

void enterSymbolCardsMode() {
  showSymbolCard(0);
  modeInitialized = true;
}

void updateSymbolCardsMode(unsigned long nowMs) {
  if (!modeInitialized) {
    enterSymbolCardsMode();
  }

  if (nowMs - lastStepMs < kSymbolCardDurationMs) {
    return;
  }

  lastStepMs = nowMs;
  ++animationIndex;

  if (animationIndex >= kSymbolCardCount) {
    setMode(MODE_WELCOME);
    return;
  }

  showSymbolCard(animationIndex);
}

/* --------------------- Setup e loop principal --------------------- */

void setup() {
  lcd.begin(kLcdColumns, kLcdRows);
  lcd.backlight();
  lcd.clear();
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
      updateWavesMode(nowMs);
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

    case MODE_SYMBOL_CARDS:
      updateSymbolCardsMode(nowMs);
      break;
  }
}
