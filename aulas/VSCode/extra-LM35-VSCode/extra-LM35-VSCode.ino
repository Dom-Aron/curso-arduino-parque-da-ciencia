/*
 * Curso de Formação de Professores – Atividades Experimentais de Física
 * Local: Parque da Ciência Newton Freire Maia (PR, Brasil)
 * Plataforma: Arduino
 * Ambiente alvo: VS Code
 * Autores do curso: Aron da Rocha Battistella; Marcos Rocha; Alan Henrique Abreu Dias
 * Colaboradores do curso: Letícia Trzaskos Abbeg; Gabriel Cordeiro Chileider
 * Autoria dos códigos: Aron da Rocha Battistella e Marcos Rocha
 * Colaboração nos códigos: Letícia Trzaskos Abbeg, Gabriel Cordeiro Chileider e Alan Henrique Abreu Dias
 * Repositório: https://github.com/parquedaciencia/curso-arduino-parque-da-ciencia
 * Caminho no repositório: aulas/VSCode/extra-LM35-VSCode/extra-LM35-VSCode.ino
 * Data da última revisão: 29/04/2026
 *
 * Descrição:
 *   ============================================================
 *   Projeto   : Extra - LM35
 *   Arquivo   : extra-LM35-VSCode.ino
 *   Pasta     : extra-LM35-VSCode
 *   Este sketch implementa um sistema de monitoramento e controle
 *   térmico com o sensor LM35 conectado à entrada analógica A0 e uma
 *   saída digital de controle no pino D2.
 *
 *   O código realiza calibração automática da temperatura ambiente,
 *   aplica filtragem, histerese e janela mínima entre comutações,
 *   enviando dados pela Serial em formato adequado ao ambiente VS Code.
 *   ============================================================
 */
#include <math.h>

/*
  Controle com LM35 usando calibração automática do ambiente e histerese.
*/

static constexpr uint8_t kLm35Pin = A0;
static constexpr uint8_t kOutputPin = 2;

static constexpr float kAdcReferenceVoltage = 1.1F;

/*
  Limites automáticos de controle baseados na temperatura ambiente
  medida durante a calibração inicial:

  temperatura_de_liga  = temperatura_ambiente - deslocamento_inferior
  temperatura_de_desliga = temperatura_ambiente + deslocamento_superior
*/
static constexpr float kTurnOnOffsetBelowAmbientC = 0.20F;
static constexpr float kTurnOffOffsetAboveAmbientC = 0.25F;
static constexpr float kFilterAlpha = 0.15F;

static constexpr uint8_t kSamplesPerReading = 32;
static constexpr unsigned long kSampleIntervalMs = 500UL;
static constexpr unsigned long kCalibrationDurationMs = 5000UL;

/*
  Pisca inicial para indicar que a saída está operacional.
  Considera saída ativa em nível alto no pino D2.
*/
static constexpr unsigned long kStartupBlinkOnTimeMs = 300UL;
static constexpr unsigned long kStartupBlinkOffTimeMs = 300UL;

/*
  Janela opcional de estabilização após comutação da saída.
  Isso ajuda a ignorar interferências elétricas transitórias de relés,
  lâmpadas ou outras cargas acionadas.
*/
static constexpr unsigned long kSettleAfterSwitchMs = 700UL;

/* Configuração do monitoramento de ruído */
static constexpr float kNoiseAlpha = 0.20F;
static constexpr float kNoiseBadReferenceC = 0.20F;

bool g_outputState = false;
bool g_filterInitialized = false;
bool g_previousRawInitialized = false;

unsigned long g_lastSampleTimeMs = 0UL;
unsigned long g_lastSwitchTimeMs = 0UL;

float g_filteredTemperatureC = 0.0F;
float g_ambientTemperatureC = 0.0F;
float g_turnOnTemperatureC = 25.0F;
float g_turnOffTemperatureC = 26.0F;

float g_previousRawTemperatureC = 0.0F;
float g_rawStepC = 0.0F;
float g_noiseEmaC = 0.0F;
float g_rawFilterErrorC = 0.0F;
float g_noiseBadnessPct = 0.0F;

/* --------------------- Funções utilitárias --------------------- */

/*!
 * @brief Lê várias amostras do ADC e retorna a média aritmética.
 *
 * Esta função realiza múltiplas leituras na entrada analógica conectada
 * ao LM35 para reduzir a influência de ruídos instantâneos. Entre as
 * leituras há um pequeno atraso para tornar a média mais estável.
 *
 * @return Valor médio do ADC correspondente à leitura do sensor.
 */
float read_adc_average()
{
  unsigned long sum = 0UL;

  for (uint8_t i = 0; i < kSamplesPerReading; ++i)
  {
    sum += static_cast<unsigned long>(analogRead(kLm35Pin));
    delay(2);
  }

  return static_cast<float>(sum) / static_cast<float>(kSamplesPerReading);
}

/*!
 * @brief Converte o valor médio do ADC em tensão.
 *
 * A conversão é feita considerando a referência analógica interna de
 * 1,1 V configurada no Arduino.
 *
 * @param adcValue Valor médio lido pelo conversor analógico-digital.
 * @return Tensão equivalente, em volts.
 */
float convert_adc_to_voltage(const float adcValue)
{
  return (adcValue * kAdcReferenceVoltage) / 1023.0F;
}

/*!
 * @brief Converte uma tensão em temperatura, em graus Celsius.
 *
 * O LM35 fornece aproximadamente 10 mV por grau Celsius. Assim, a
 * conversão é feita multiplicando a tensão medida por 100.
 *
 * @param voltage Tensão de saída do sensor, em volts.
 * @return Temperatura correspondente, em graus Celsius.
 */
float convert_voltage_to_temperature_c(const float voltage)
{
  return voltage * 100.0F;
}

/*!
 * @brief Realiza uma leitura bruta completa da temperatura.
 *
 * A função obtém a média do ADC, converte esse valor em tensão e, em
 * seguida, converte a tensão em temperatura.
 *
 * @return Temperatura bruta lida pelo LM35, em graus Celsius.
 */
float read_raw_temperature_c()
{
  const float adcAverage = read_adc_average();
  const float voltage = convert_adc_to_voltage(adcAverage);
  return convert_voltage_to_temperature_c(voltage);
}

/*!
 * @brief Aplica filtragem exponencial à temperatura medida.
 *
 * Esta função suaviza variações rápidas na leitura do sensor, reduzindo
 * oscilações visíveis na medição e no controle. Na primeira execução,
 * o filtro é inicializado com o valor atual.
 *
 * @param currentTemperatureC Temperatura atual bruta, em graus Celsius.
 * @return Temperatura filtrada, em graus Celsius.
 */
float apply_exponential_filter(const float currentTemperatureC)
{
  if (!g_filterInitialized)
  {
    g_filteredTemperatureC = currentTemperatureC;
    g_filterInitialized = true;
    return g_filteredTemperatureC;
  }

  g_filteredTemperatureC =
      (kFilterAlpha * currentTemperatureC) +
      ((1.0F - kFilterAlpha) * g_filteredTemperatureC);

  return g_filteredTemperatureC;
}

/*!
 * @brief Pisca a saída digital uma vez durante a inicialização.
 *
 * Esse procedimento serve como indicação visual de que o pino de saída
 * está funcionando antes do início da calibração automática.
 */
void blink_startup_indicator()
{
  digitalWrite(kOutputPin, HIGH);
  delay(kStartupBlinkOnTimeMs);

  digitalWrite(kOutputPin, LOW);
  delay(kStartupBlinkOffTimeMs);
}

/*!
 * @brief Calcula automaticamente a temperatura ambiente no início.
 *
 * Durante um intervalo fixo de tempo, a função lê repetidamente o LM35,
 * acumula as leituras e calcula a média. Essa média será usada como base
 * para definir os limites de acionamento com histerese.
 *
 * @return Temperatura ambiente média determinada na calibração, em graus Celsius.
 */
float calibrate_ambient_temperature()
{
  const unsigned long startTimeMs = millis();

  float sumTemperatureC = 0.0F;
  unsigned int sampleCount = 0U;

  while ((millis() - startTimeMs) < kCalibrationDurationMs)
  {
    const float rawTemperatureC = read_raw_temperature_c();
    sumTemperatureC += rawTemperatureC;
    ++sampleCount;

    Serial.print("Calibrating ambient... sample ");
    Serial.print(sampleCount);
    Serial.print(" | Temp=");
    Serial.print(rawTemperatureC, 2);
    Serial.println(" C");

    delay(120);
  }

  if (sampleCount == 0U)
  {
    return read_raw_temperature_c();
  }

  return sumTemperatureC / static_cast<float>(sampleCount);
}

/*!
 * @brief Atualiza as métricas de ruído e estabilidade do sinal.
 *
 * A função calcula a variação entre leituras consecutivas, o erro entre
 * o valor bruto e o valor filtrado, e uma média exponencial da variação
 * bruta para estimar o nível de ruído do sistema.
 *
 * @param rawTemperatureC Temperatura bruta atual, em graus Celsius.
 * @param filteredTemperatureC Temperatura filtrada atual, em graus Celsius.
 */
void update_noise_metrics(
    const float rawTemperatureC,
    const float filteredTemperatureC)
{
  if (!g_previousRawInitialized)
  {
    g_previousRawTemperatureC = rawTemperatureC;
    g_previousRawInitialized = true;
    g_rawStepC = 0.0F;
    g_noiseEmaC = 0.0F;
    g_rawFilterErrorC = 0.0F;
    g_noiseBadnessPct = 0.0F;
    return;
  }

  g_rawStepC = fabs(rawTemperatureC - g_previousRawTemperatureC);
  g_rawFilterErrorC = fabs(rawTemperatureC - filteredTemperatureC);

  g_noiseEmaC =
      (kNoiseAlpha * g_rawStepC) +
      ((1.0F - kNoiseAlpha) * g_noiseEmaC);

  g_noiseBadnessPct = constrain(
      (g_noiseEmaC / kNoiseBadReferenceC) * 100.0F,
      0.0F,
      100.0F);

  g_previousRawTemperatureC = rawTemperatureC;
}

/*!
 * @brief Retorna um rótulo textual para o nível de ruído atual.
 *
 * O rótulo é definido a partir do valor da média exponencial do ruído,
 * permitindo identificar qualitativamente a estabilidade da leitura.
 *
 * @return Texto indicando a condição do ruído da leitura.
 */
const char* get_noise_state_label()
{
  if (g_noiseEmaC < 0.03F)
  {
    return "VERY_STABLE";
  }

  if (g_noiseEmaC < 0.08F)
  {
    return "OK";
  }

  if (g_noiseEmaC < 0.20F)
  {
    return "NOISY";
  }

  return "BAD";
}

/*!
 * @brief Verifica se o sistema ainda está na janela de estabilização.
 *
 * Após cada comutação da saída, esta função informa se o intervalo de
 * acomodação ainda está ativo, permitindo ignorar leituras temporariamente.
 *
 * @return true se ainda estiver na janela de estabilização; caso contrário, false.
 */
bool is_in_settle_window()
{
  return (millis() - g_lastSwitchTimeMs) < kSettleAfterSwitchMs;
}

/*!
 * @brief Atualiza o estado da saída digital, se necessário.
 *
 * A função evita escrita redundante no pino e registra o instante da
 * comutação para controle da janela de estabilização.
 *
 * @param newOutputState Novo estado desejado para a saída digital.
 */
void set_output_state(const bool newOutputState)
{
  if (g_outputState == newOutputState)
  {
    return;
  }

  g_outputState = newOutputState;
  digitalWrite(kOutputPin, g_outputState ? HIGH : LOW);
  g_lastSwitchTimeMs = millis();
}

/*!
 * @brief Decide se a saída deve ser ligada ou desligada.
 *
 * A lógica usa histerese: liga quando a temperatura filtrada fica abaixo
 * do limite inferior e desliga quando ela ultrapassa o limite superior.
 * Enquanto a leitura estiver dentro da faixa intermediária, o estado atual
 * é mantido.
 *
 * @param filteredTemperatureC Temperatura filtrada atual, em graus Celsius.
 */
void update_output_state(const float filteredTemperatureC)
{
  if (is_in_settle_window())
  {
    return;
  }

  if (!g_outputState && (filteredTemperatureC <= g_turnOnTemperatureC))
  {
    set_output_state(true);
  }
  else if (g_outputState && (filteredTemperatureC >= g_turnOffTemperatureC))
  {
    set_output_state(false);
  }
}

/*!
 * @brief Retorna um marcador auxiliar para visualização em gráficos.
 *
 * O valor retornado é deslocado para baixo ou para cima dos limiares,
 * dependendo do estado da saída, facilitando a identificação visual do
 * acionamento em ferramentas de plotagem.
 *
 * @return Valor de marcador, em graus Celsius.
 */
float get_output_marker_c()
{
  const float markerOffsetC = 2.0F;

  if (g_outputState)
  {
    return g_turnOnTemperatureC - markerOffsetC;
  }

  return g_turnOffTemperatureC + markerOffsetC;
}

/*!
 * @brief Envia pela Serial um relatório legível do estado atual.
 *
 * A saída inclui temperatura ambiente calibrada, limites de controle,
 * leitura bruta, leitura filtrada, métricas de ruído, estado da janela
 * de estabilização e estado da saída digital.
 *
 * @param rawTemperatureC Temperatura bruta atual, em graus Celsius.
 * @param filteredTemperatureC Temperatura filtrada atual, em graus Celsius.
 */
void print_human_readable_status(
    const float rawTemperatureC,
    const float filteredTemperatureC)
{
  Serial.print("AmbientC=");
  Serial.print(g_ambientTemperatureC, 2);

  Serial.print(" | TurnOnBelowC=");
  Serial.print(g_turnOnTemperatureC, 2);

  Serial.print(" | TurnOffAboveC=");
  Serial.print(g_turnOffTemperatureC, 2);

  Serial.print(" | RawC=");
  Serial.print(rawTemperatureC, 2);

  Serial.print(" | FilteredC=");
  Serial.print(filteredTemperatureC, 2);

  Serial.print(" | RawStepC=");
  Serial.print(g_rawStepC, 3);

  Serial.print(" | NoiseEmaC=");
  Serial.print(g_noiseEmaC, 3);

  Serial.print(" | RawFilterErrorC=");
  Serial.print(g_rawFilterErrorC, 3);

  Serial.print(" | NoiseBadnessPct=");
  Serial.print(g_noiseBadnessPct, 1);

  Serial.print(" | NoiseState=");
  Serial.print(get_noise_state_label());

  Serial.print(" | InSettleWindow=");
  Serial.print(is_in_settle_window() ? "YES" : "NO");

  Serial.print(" | D2=");
  Serial.println(g_outputState ? "ON" : "OFF");
}

/*!
 * @brief Envia os dados em formato adequado para plotagem.
 *
 * O formato produzido facilita o uso em scripts externos, serial plotters
 * ou rotinas de captura de dados, permitindo acompanhar a evolução das
 * variáveis principais do experimento.
 *
 * @param rawTemperatureC Temperatura bruta atual, em graus Celsius.
 * @param filteredTemperatureC Temperatura filtrada atual, em graus Celsius.
 */
void send_plot_data(
    const float rawTemperatureC,
    const float filteredTemperatureC)
{
  Serial.print(">raw_c:");
  Serial.print(rawTemperatureC, 2);

  Serial.print(",filtered_c:");
  Serial.print(filteredTemperatureC, 2);

  Serial.print(",ambient_c:");
  Serial.print(g_ambientTemperatureC, 2);

  Serial.print(",turn_on_c:");
  Serial.print(g_turnOnTemperatureC, 2);

  Serial.print(",turn_off_c:");
  Serial.print(g_turnOffTemperatureC, 2);

  Serial.print(",d2_marker_c:");
  Serial.print(get_output_marker_c(), 2);

  Serial.print(",raw_step_c:");
  Serial.print(g_rawStepC, 3);

  Serial.print(",noise_ema_c:");
  Serial.print(g_noiseEmaC, 3);

  Serial.print(",raw_filter_error_c:");
  Serial.print(g_rawFilterErrorC, 3);

  Serial.print(",noise_badness_pct:");
  Serial.println(g_noiseBadnessPct, 1);
}

/*!
 * @brief Inicializa o sistema, a Serial, a referência analógica e a calibração.
 *
 * Esta função configura o pino de saída, inicia a comunicação serial,
 * seleciona a referência analógica interna, estabiliza o ADC, pisca a
 * saída como teste visual e executa a calibração automática da temperatura
 * ambiente. Ao final, calcula os limiares de histerese e prepara o sistema
 * para o monitoramento contínuo.
 */
void setup()
{
  pinMode(kOutputPin, OUTPUT);
  digitalWrite(kOutputPin, LOW);
  g_outputState = false;
  g_lastSwitchTimeMs = 0UL;

  Serial.begin(115200);

  analogReference(INTERNAL);
  delay(100);

  for (uint8_t i = 0; i < 10; ++i)
  {
    analogRead(kLm35Pin);
    delay(5);
  }

  Serial.println();
  Serial.println("LM35 startup test");
  Serial.println("Blinking D2 once before calibration...");
  blink_startup_indicator();

  Serial.println();
  Serial.println("LM35 automatic calibration started");
  Serial.println("Keep the sensor untouched during startup calibration");

  g_ambientTemperatureC = calibrate_ambient_temperature();
  g_turnOnTemperatureC =
      g_ambientTemperatureC - kTurnOnOffsetBelowAmbientC;
  g_turnOffTemperatureC =
      g_ambientTemperatureC + kTurnOffOffsetAboveAmbientC;

  if (g_turnOnTemperatureC >= g_turnOffTemperatureC)
  {
    const float centerTemperatureC = g_ambientTemperatureC;
    g_turnOnTemperatureC = centerTemperatureC - 0.5F;
    g_turnOffTemperatureC = centerTemperatureC + 0.5F;
  }

  g_filteredTemperatureC = g_ambientTemperatureC;
  g_filterInitialized = true;
  g_previousRawTemperatureC = g_ambientTemperatureC;
  g_previousRawInitialized = true;
  g_rawStepC = 0.0F;
  g_noiseEmaC = 0.0F;
  g_rawFilterErrorC = 0.0F;
  g_noiseBadnessPct = 0.0F;

  digitalWrite(kOutputPin, LOW);
  g_outputState = false;
  g_lastSwitchTimeMs = millis();

  Serial.println();
  Serial.println("Calibration finished");
  Serial.print("Ambient temperature: ");
  Serial.print(g_ambientTemperatureC, 2);
  Serial.println(" C");

  Serial.print("Turn ON below: ");
  Serial.print(g_turnOnTemperatureC, 2);
  Serial.println(" C");

  Serial.print("Turn OFF above: ");
  Serial.print(g_turnOffTemperatureC, 2);
  Serial.println(" C");

  Serial.print("Settle window after switching: ");
  Serial.print(kSettleAfterSwitchMs);
  Serial.println(" ms");

  Serial.println();
  Serial.println("Monitoring started");
}

/*!
 * @brief Executa continuamente a leitura, o processamento e o controle.
 *
 * Em intervalos regulares, a função lê a temperatura do LM35, aplica o
 * filtro exponencial, atualiza as métricas de ruído, verifica a lógica de
 * histerese para a saída digital e envia as informações pela Serial.
 */
void loop()
{
  const unsigned long currentTimeMs = millis();

  if ((currentTimeMs - g_lastSampleTimeMs) >= kSampleIntervalMs)
  {
    g_lastSampleTimeMs = currentTimeMs;

    const float rawTemperatureC = read_raw_temperature_c();
    const float filteredTemperatureC = apply_exponential_filter(rawTemperatureC);

    update_noise_metrics(rawTemperatureC, filteredTemperatureC);
    update_output_state(filteredTemperatureC);
    print_human_readable_status(rawTemperatureC, filteredTemperatureC);
    send_plot_data(rawTemperatureC, filteredTemperatureC);
  }
}
