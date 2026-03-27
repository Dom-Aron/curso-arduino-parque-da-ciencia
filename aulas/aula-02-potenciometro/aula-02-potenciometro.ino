// ----------------------------------------------
// Parque da Ciência Newton Freire Maia
// Curso de Formação de Professores
// Atividades Experimentais de Física
// Utilizando a Plataforma Arduino
// ----------------------------------------------
// Projeto: Controle de brilho com potenciômetro
// Descrição:
// Lê o valor de um potenciômetro conectado à entrada
// analógica A0 e ajusta o brilho de um LED conectado
// ao pino digital 3 com PWM.
// ----------------------------------------------

static constexpr uint8_t LED_PIN = 3;
static constexpr uint8_t POTENTIOMETER_PIN = A0;
static constexpr long SERIAL_BAUD_RATE = 9600L;
static constexpr unsigned long SERIAL_INTERVAL_MS = 200UL;

unsigned long g_lastSerialTimeMs = 0UL;

/*
    Lê o valor atual do potenciômetro na entrada analógica.

    Retorno:
        Valor inteiro entre 0 e 1023.
*/
int readPotentiometerValue()
{
    return analogRead(POTENTIOMETER_PIN);
}

/*
    Converte o valor lido do potenciômetro para a faixa
    de PWM utilizada no analogWrite.

    Parâmetros:
        potentiometerValue: valor lido entre 0 e 1023.

    Retorno:
        Valor de brilho entre 0 e 255.
*/
int convertPotentiometerToPwm(const int potentiometerValue)
{
    return map(potentiometerValue, 0, 1023, 0, 255);
}

/*
    Aplica ao LED o valor de brilho calculado.

    Parâmetros:
        brightness: valor entre 0 e 255.
*/
void applyLedBrightness(const int brightness)
{
    analogWrite(LED_PIN, brightness);
}

/*
    Exibe no monitor serial os valores relevantes do sistema.

    Parâmetros:
        potentiometerValue: valor lido do potenciômetro.
        brightness: valor PWM aplicado ao LED.
*/
void printStatus(const int potentiometerValue, const int brightness)
{
    Serial.print("Valor do potenciometro: ");
    Serial.print(potentiometerValue);
    Serial.print(" | Brilho PWM: ");
    Serial.println(brightness);
}

/*
    Função de configuração inicial do Arduino.

    Esta função é executada uma única vez no início.
*/
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);

    Serial.begin(SERIAL_BAUD_RATE);

    Serial.println("Sistema iniciado.");
    Serial.println("Controle de brilho do LED por potenciometro.");
}

/*
    Função principal do Arduino.

    Esta função é executada continuamente em loop.
*/
void loop()
{
    const int potentiometerValue = readPotentiometerValue();
    const int brightness = convertPotentiometerToPwm(potentiometerValue);

    applyLedBrightness(brightness);

    const unsigned long currentTimeMs = millis();

    if (currentTimeMs - g_lastSerialTimeMs >= SERIAL_INTERVAL_MS)
    {
        g_lastSerialTimeMs = currentTimeMs;
        printStatus(potentiometerValue, brightness);
    }
}