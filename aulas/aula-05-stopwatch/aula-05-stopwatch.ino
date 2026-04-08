/*
  ============================================================
  Projeto   : Aula 05 - Stopwatch
  Arquivo   : aula-05-stopwatch.ino
  Autor     : Aron da Rocha Battistella
  Plataforma: Arduino UNO/Nano + LCD 16x2 com módulo I2C
  Biblioteca: New-LiquidCrystal (LiquidCrystal_I2C)

  Descrição:
  Este programa implementa um cronômetro com display LCD I2C.
  O sistema possui dois botões:
    - Pause/Resume: inicia ou pausa a contagem
    - Reset       : zera o cronômetro

  Observações:
  - O LCD utiliza o módulo I2C com endereço 0x27.
  - Os botões foram mantidos em modo INPUT para preservar a
    montagem atual do circuito.
  - O código utiliza debounce por software para evitar leituras
    incorretas causadas pelo efeito de bouncing dos botões.
  ============================================================
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/*
  Configuração do LCD I2C compatível com a biblioteca
  New-LiquidCrystal e módulos backpack baseados em PCF8574.
*/
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

/* ------------------------- Constantes ------------------------- */

const uint8_t kPauseButtonPin = 2;
const uint8_t kResetButtonPin = 3;

const uint8_t kLcdColumns = 16;
const uint8_t kLcdRows = 2;

const uint8_t kTitleColumn = 3;
const uint8_t kTitleRow = 0;

const uint8_t kTimeColumn = 2;
const uint8_t kTimeRow = 1;

const unsigned long kDebounceDelayMs = 30;

/* --------------------- Estruturas auxiliares --------------------- */

/*!
 * @brief Armazena o estado de debounce de um botão.
 */
struct ButtonState {
  bool rawState = LOW;
  bool stableState = LOW;
  unsigned long lastChangeMs = 0;
};

/* --------------------- Estado do cronômetro --------------------- */

bool isTimerRunning = false;

unsigned long accumulatedElapsedMs = 0;
unsigned long runningStartMs = 0;

ButtonState pauseButtonState;
ButtonState resetButtonState;

/*
  Variáveis usadas para atualizar apenas os campos do display
  que realmente mudaram, reduzindo escritas desnecessárias no LCD.
*/
int lastDisplayedHours = -1;
int lastDisplayedMinutes = -1;
int lastDisplayedSeconds = -1;
int lastDisplayedMilliseconds = -1;

/* --------------------- Funções utilitárias --------------------- */

/*!
 * @brief Retorna o tempo total decorrido do cronômetro.
 *
 * Se o cronômetro estiver em execução, o valor retornado é a soma
 * do tempo já acumulado com o intervalo decorrido desde o último
 * início da contagem.
 *
 * @return Tempo decorrido total em milissegundos.
 */
unsigned long getElapsedTimeMs() {
  if (isTimerRunning) {
    return accumulatedElapsedMs + (millis() - runningStartMs);
  }

  return accumulatedElapsedMs;
}

/*!
 * @brief Escreve um valor com dois dígitos no display.
 *
 * Caso o valor possua apenas um dígito, a função adiciona um zero
 * à esquerda para manter o formato visual do cronômetro.
 *
 * @param column Coluna inicial no LCD.
 * @param row Linha do LCD.
 * @param value Valor a ser exibido com dois dígitos.
 */
void printTwoDigitValue(uint8_t column, uint8_t row, unsigned int value) {
  lcd.setCursor(column, row);

  if (value < 10) {
    lcd.print('0');
  }

  lcd.print(value);
}

/*!
 * @brief Escreve um valor com três dígitos no display.
 *
 * Caso o valor possua menos de três dígitos, a função adiciona
 * zeros à esquerda para manter o formato visual do cronômetro.
 *
 * @param column Coluna inicial no LCD.
 * @param row Linha do LCD.
 * @param value Valor a ser exibido com três dígitos.
 */
void printThreeDigitValue(uint8_t column, uint8_t row, unsigned int value) {
  lcd.setCursor(column, row);

  if (value < 100) {
    lcd.print('0');
  }

  if (value < 10) {
    lcd.print('0');
  }

  lcd.print(value);
}

/*!
 * @brief Desenha a máscara fixa do cronômetro no display.
 *
 * Esta função escreve o formato base "00:00:00.000", deixando
 * os separadores ":" e "." fixos na tela.
 */
void drawTimerMask() {
  lcd.setCursor(kTimeColumn, kTimeRow);
  lcd.print("00:00:00.000");
}

/*!
 * @brief Invalida o cache da última exibição do tempo.
 *
 * Isso força a atualização completa dos campos numéricos do LCD
 * na próxima chamada de atualização do display.
 */
void invalidateDisplayedTimeCache() {
  lastDisplayedHours = -1;
  lastDisplayedMinutes = -1;
  lastDisplayedSeconds = -1;
  lastDisplayedMilliseconds = -1;
}

/*!
 * @brief Atualiza os valores numéricos do cronômetro no LCD.
 *
 * A função recalcula horas, minutos, segundos e milissegundos
 * a partir do tempo decorrido e atualiza apenas os campos que
 * realmente mudaram desde a última exibição.
 *
 * @param elapsedMs Tempo decorrido total em milissegundos.
 */
void refreshTimerDisplay(unsigned long elapsedMs) {
  const unsigned int hours = (elapsedMs / 3600000UL) % 100;
  const unsigned int minutes = (elapsedMs / 60000UL) % 60;
  const unsigned int seconds = (elapsedMs / 1000UL) % 60;
  const unsigned int milliseconds = elapsedMs % 1000;

  if ((int)hours != lastDisplayedHours) {
    printTwoDigitValue(2, 1, hours);
    lastDisplayedHours = hours;
  }

  if ((int)minutes != lastDisplayedMinutes) {
    printTwoDigitValue(5, 1, minutes);
    lastDisplayedMinutes = minutes;
  }

  if ((int)seconds != lastDisplayedSeconds) {
    printTwoDigitValue(8, 1, seconds);
    lastDisplayedSeconds = seconds;
  }

  if ((int)milliseconds != lastDisplayedMilliseconds) {
    printThreeDigitValue(11, 1, milliseconds);
    lastDisplayedMilliseconds = milliseconds;
  }
}

/* ------------------- Controle do cronômetro ------------------- */

/*!
 * @brief Alterna o estado do cronômetro entre execução e pausa.
 *
 * Quando o cronômetro está em execução, a função armazena o tempo
 * decorrido acumulado e entra em pausa. Quando está pausado, a
 * função reinicia a contagem a partir do instante atual.
 */
void toggleTimerRunningState() {
  if (isTimerRunning) {
    accumulatedElapsedMs = getElapsedTimeMs();
    isTimerRunning = false;
    return;
  }

  runningStartMs = millis();
  isTimerRunning = true;
}

/*!
 * @brief Zera o cronômetro e atualiza o display.
 *
 * A função redefine o tempo acumulado para zero, preserva o estado
 * de execução atual e força a atualização visual do cronômetro.
 */
void resetTimer() {
  accumulatedElapsedMs = 0;

  if (isTimerRunning) {
    runningStartMs = millis();
  }

  invalidateDisplayedTimeCache();
  refreshTimerDisplay(0);
}

/* ------------------ Leitura e debounce dos botões ------------------ */

/*!
 * @brief Verifica se um botão foi pressionado com debounce por software.
 *
 * A função monitora o estado bruto e o estado estável do botão,
 * retornando verdadeiro apenas quando uma borda de subida válida
 * é detectada após o tempo de debounce configurado.
 *
 * @param pin Pino digital associado ao botão.
 * @param buttonState Estrutura que armazena o estado interno do botão.
 * @return true se um novo pressionamento válido foi detectado.
 * @return false caso contrário.
 */
bool wasButtonPressed(uint8_t pin, ButtonState& buttonState) {
  const bool currentReading = digitalRead(pin);
  const unsigned long currentTimeMs = millis();

  if (currentReading != buttonState.rawState) {
    buttonState.rawState = currentReading;
    buttonState.lastChangeMs = currentTimeMs;
  }

  if ((currentTimeMs - buttonState.lastChangeMs) < kDebounceDelayMs) {
    return false;
  }

  if (buttonState.stableState == buttonState.rawState) {
    return false;
  }

  buttonState.stableState = buttonState.rawState;
  return (buttonState.stableState == HIGH);
}

/*!
 * @brief Processa o botão de pausa e continuação.
 *
 * Quando um pressionamento válido é detectado, o estado do
 * cronômetro é alternado entre execução e pausa.
 */
void handlePauseButton() {
  if (wasButtonPressed(kPauseButtonPin, pauseButtonState)) {
    toggleTimerRunningState();
  }
}

/*!
 * @brief Processa o botão de reset.
 *
 * Quando um pressionamento válido é detectado, o cronômetro
 * é reiniciado para zero.
 */
void handleResetButton() {
  if (wasButtonPressed(kResetButtonPin, resetButtonState)) {
    resetTimer();
  }
}

/* -------------------------- Setup -------------------------- */

/*!
 * @brief Inicializa o hardware e a interface do cronômetro.
 *
 * Configura os pinos dos botões, inicializa o LCD, ativa a
 * iluminação de fundo e desenha a interface inicial do sistema.
 */
void setup() {
  pinMode(kPauseButtonPin, INPUT);
  pinMode(kResetButtonPin, INPUT);

  lcd.begin(kLcdColumns, kLcdRows);
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(kTitleColumn, kTitleRow);
  lcd.print("CRONOMETRO");

  drawTimerMask();
  resetTimer();
}

/* --------------------------- Loop --------------------------- */

/*!
 * @brief Executa continuamente a lógica principal do programa.
 *
 * O loop verifica o estado dos botões e atualiza o tempo exibido
 * no display LCD.
 */
void loop() {
  handlePauseButton();
  handleResetButton();
  refreshTimerDisplay(getElapsedTimeMs());
}