// ----------------------------------------------
// Parque da Ciência Newton Freire Maia
// Curso de Formação de Professores
// Atividades Experimentais de Física
// Utilizando a Plataforma Arduino
// ----------------------------------------------
// Projeto: Monitoramento de temperatura com 6 LM35
// Descrição:
// Realiza a leitura de 6 sensores LM35 conectados
// às entradas analógicas A0 até A5, calcula a média
// das temperaturas em uma janela de 15 segundos e
// envia os dados pela serial.
// ----------------------------------------------

/*
    Modos de saída disponíveis.

    OUTPUT_HUMAN_READABLE:
        Exibe os dados em formato legível no Monitor Serial.

    OUTPUT_SERIAL_PLOTTER:
        Exibe os dados em formato adequado para o Serial Plotter.
*/
enum OutputMode
{
    OUTPUT_HUMAN_READABLE,
    OUTPUT_SERIAL_PLOTTER
};

// -----------------------------------------------------------------------------
// Configuração do usuário
// -----------------------------------------------------------------------------

static constexpr OutputMode kOutputMode = OUTPUT_SERIAL_PLOTTER;

static constexpr uint8_t kSensorCount = 6;
static constexpr uint8_t kSensorPins[kSensorCount] = {A0, A1, A2, A3, A4, A5};

static constexpr unsigned long kSampleIntervalMs = 125UL;
static constexpr unsigned long kReportIntervalMs = 15000UL;  // 15 segundos

static constexpr long kSerialBaudRate = 115200L;
static constexpr float kAdcReferenceVoltage = 5.0f;   // Tensão de referência do ADC
static constexpr float kAdcResolution = 1023.0f;      // ADC de 10 bits
static constexpr float kLm35MilliVoltsPerC = 10.0f;   // LM35 = 10 mV / °C

// -----------------------------------------------------------------------------
// Estado global do sistema
// -----------------------------------------------------------------------------

float g_lastTemperatureC[kSensorCount] = {0.0f};
float g_sumTemperatureC[kSensorCount] = {0.0f};
float g_averageTemperatureC[kSensorCount] = {0.0f};

uint16_t g_sampleCount = 0;
unsigned long g_lastSampleMs = 0UL;
unsigned long g_windowStartMs = 0UL;

// -----------------------------------------------------------------------------
// Declarações de funções
// -----------------------------------------------------------------------------

float readTemperatureC(uint8_t analogPin);
void sampleAllSensors();
void computeAverages();
void resetAveragingWindow();
void printHumanReadableReport();
void printSerialPlotterReport();
void printStartupMessage();
void printSeparator();

// -----------------------------------------------------------------------------
// Configuração inicial
// -----------------------------------------------------------------------------

void setup()
{
    Serial.begin(kSerialBaudRate);

    for (uint8_t i = 0; i < kSensorCount; ++i)
    {
        pinMode(kSensorPins[i], INPUT);
    }

    g_lastSampleMs = millis();
    g_windowStartMs = millis();

    printStartupMessage();
}

// -----------------------------------------------------------------------------
// Laço principal
// -----------------------------------------------------------------------------

void loop()
{
    const unsigned long currentMs = millis();

    // Realiza a leitura periódica de todos os sensores.
    if (currentMs - g_lastSampleMs >= kSampleIntervalMs)
    {
        g_lastSampleMs += kSampleIntervalMs;
        sampleAllSensors();
    }

    // Ao final da janela de tempo, calcula as médias e envia os resultados.
    if (currentMs - g_windowStartMs >= kReportIntervalMs)
    {
        computeAverages();

        if (kOutputMode == OUTPUT_HUMAN_READABLE)
        {
            printHumanReadableReport();
        }
        else
        {
            printSerialPlotterReport();
        }

        resetAveragingWindow();
        g_windowStartMs += kReportIntervalMs;
    }
}

// -----------------------------------------------------------------------------
// Lê um sensor LM35 e converte o valor do ADC para temperatura em graus Celsius.
// -----------------------------------------------------------------------------

float readTemperatureC(uint8_t analogPin)
{
    const int adcValue = analogRead(analogPin);
    const float voltage = (static_cast<float>(adcValue) * kAdcReferenceVoltage) / kAdcResolution;
    const float temperatureC = (voltage * 1000.0f) / kLm35MilliVoltsPerC;

    return temperatureC;
}

// -----------------------------------------------------------------------------
// Lê todos os sensores e acumula os valores para o cálculo da média.
// -----------------------------------------------------------------------------

void sampleAllSensors()
{
    for (uint8_t i = 0; i < kSensorCount; ++i)
    {
        g_lastTemperatureC[i] = readTemperatureC(kSensorPins[i]);
        g_sumTemperatureC[i] += g_lastTemperatureC[i];
    }

    ++g_sampleCount;
}

// -----------------------------------------------------------------------------
// Calcula a média das temperaturas dentro da janela atual.
// -----------------------------------------------------------------------------

void computeAverages()
{
    if (g_sampleCount == 0)
    {
        for (uint8_t i = 0; i < kSensorCount; ++i)
        {
            g_averageTemperatureC[i] = 0.0f;
        }

        return;
    }

    for (uint8_t i = 0; i < kSensorCount; ++i)
    {
        g_averageTemperatureC[i] = g_sumTemperatureC[i] / static_cast<float>(g_sampleCount);
    }
}

// -----------------------------------------------------------------------------
// Reinicia os acumuladores para a próxima janela de média.
// -----------------------------------------------------------------------------

void resetAveragingWindow()
{
    for (uint8_t i = 0; i < kSensorCount; ++i)
    {
        g_sumTemperatureC[i] = 0.0f;
    }

    g_sampleCount = 0;
}

// -----------------------------------------------------------------------------
// Exibe um relatório em formato legível para o Monitor Serial.
// -----------------------------------------------------------------------------

void printHumanReadableReport()
{
    const unsigned long elapsedSeconds = g_windowStartMs / 1000UL;

    printSeparator();

    Serial.print(F("Tempo decorrido: "));
    Serial.print(elapsedSeconds);
    Serial.println(F(" s"));

    Serial.print(F("Numero de amostras na janela: "));
    Serial.println(g_sampleCount);

    for (uint8_t i = 0; i < kSensorCount; ++i)
    {
        Serial.print(F("Sensor "));
        Serial.print(i + 1);
        Serial.print(F(" | Media: "));
        Serial.print(g_averageTemperatureC[i], 1);
        Serial.print(F(" C | Ultima leitura: "));
        Serial.print(g_lastTemperatureC[i], 1);
        Serial.println(F(" C"));
    }
}

// -----------------------------------------------------------------------------
// Exibe uma linha formatada para o Serial Plotter.
//
// Cada linha enviada representa um ponto no gráfico.
// -----------------------------------------------------------------------------

void printSerialPlotterReport()
{
    for (uint8_t i = 0; i < kSensorCount; ++i)
    {
        Serial.print(F("S"));
        Serial.print(i + 1);
        Serial.print(F(":"));
        Serial.print(g_averageTemperatureC[i], 1);

        if (i < (kSensorCount - 1))
        {
            Serial.print('\t');
        }
    }

    Serial.println();
}

// -----------------------------------------------------------------------------
// Exibe a mensagem inicial do sistema.
// -----------------------------------------------------------------------------

void printStartupMessage()
{
    delay(300);

    printSeparator();
    Serial.println(F("Monitor de temperatura com 6 sensores LM35 iniciado."));

    Serial.print(F("Intervalo entre amostras: "));
    Serial.print(kSampleIntervalMs);
    Serial.println(F(" ms"));

    Serial.print(F("Janela de media: "));
    Serial.print(kReportIntervalMs / 1000UL);
    Serial.println(F(" s"));

    Serial.print(F("Modo de saida: "));
    if (kOutputMode == OUTPUT_HUMAN_READABLE)
    {
        Serial.println(F("Monitor Serial"));
    }
    else
    {
        Serial.println(F("Serial Plotter"));
    }

    Serial.println(F("Pinos utilizados: A0, A1, A2, A3, A4, A5"));
    printSeparator();
}

// -----------------------------------------------------------------------------
// Exibe uma linha separadora no Monitor Serial.
// -----------------------------------------------------------------------------

void printSeparator()
{
    Serial.println(F("--------------------------------------------------"));
}