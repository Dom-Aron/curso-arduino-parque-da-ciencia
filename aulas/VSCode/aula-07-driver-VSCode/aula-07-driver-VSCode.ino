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
 * Caminho no repositório: aulas/VSCode/aula-07-driver-VSCode/aula-07-driver-VSCode.ino
 * Data da última revisão: 29/04/2026
 *
 * Descrição:
 *   ============================================================
 *   Projeto   : Aula 07 - Driver
 *   Arquivo   : aula-07-driver-VSCode.ino
 *   Pasta     : aula-07-driver-VSCode
 *   Este sketch implementa um controlador de acionamento para relé ou
 *   etapa de potência a partir de comandos recebidos pelo Monitor
 *   Serial.
 *
 *   O sistema suporta comandos manuais, alternância automática por
 *   intervalo e geração de pulso único, mantendo a execução
 *   responsiva com base em millis().
 *   ============================================================
 */
static constexpr int kRelayPin = 2;
static constexpr long kSerialBaudRate = 115200;
static constexpr unsigned long kDefaultAutoIntervalMs = 2000UL;
static constexpr unsigned long kDefaultPulseDurationMs = 500UL;
static constexpr unsigned long kCommandIdleTimeoutMs = 120UL;
static constexpr unsigned long kMinimumIntervalMs = 50UL;
static constexpr unsigned long kMaximumIntervalMs = 600000UL;

/*
 * Níveis lógicos de acionamento do relé.
 *
 * Relé ACTIVE HIGH:
 *   kRelayActiveLevel   = HIGH
 *   kRelayInactiveLevel = LOW
 *
 * Relé ACTIVE LOW:
 *   kRelayActiveLevel   = LOW
 *   kRelayInactiveLevel = HIGH
 */
static constexpr int kRelayActiveLevel = HIGH;
static constexpr int kRelayInactiveLevel = LOW;

String g_serialCommand;

bool g_isRelayOn = false;
bool g_isAutoModeEnabled = false;
bool g_isPulseActive = false;

unsigned long g_autoToggleIntervalMs = kDefaultAutoIntervalMs;
unsigned long g_pulseDurationMs = kDefaultPulseDurationMs;
unsigned long g_lastAutoToggleTimeMs = 0UL;
unsigned long g_pulseStartTimeMs = 0UL;
unsigned long g_lastSerialByteTimeMs = 0UL;

/* --------------------- Funções utilitárias --------------------- */

/*!
 * @brief Normaliza um comando recebido pela Serial.
 *
 * Remove espaços em branco no início e no fim do texto e converte
 * todos os caracteres para maiúsculas, facilitando a comparação com
 * os comandos reconhecidos pelo programa.
 *
 * @param command Texto original recebido do Monitor Serial.
 * @return Comando normalizado em letras maiúsculas.
 */
String normalizeCommand(String command)
{
    command.trim();
    command.toUpperCase();
    return command;
}

/*!
 * @brief Aplica ao pino físico o estado lógico atual do relé.
 *
 * Esta função escreve no pino configurado para o relé o nível lógico
 * correspondente ao estado armazenado em g_isRelayOn, respeitando se
 * o módulo utilizado é ACTIVE HIGH ou ACTIVE LOW.
 */
void applyRelayState()
{
    digitalWrite(kRelayPin, g_isRelayOn ? kRelayActiveLevel : kRelayInactiveLevel);
}

/*!
 * @brief Tenta extrair um valor de tempo, em milissegundos, de um texto.
 *
 * O texto é validado para garantir que contém apenas dígitos e que o
 * valor informado está dentro da faixa permitida pelo programa.
 *
 * @param text Texto que contém o valor numérico a ser interpretado.
 * @param valueMs Referência onde será armazenado o valor convertido.
 * @return true se a conversão for bem-sucedida.
 * @return false se o texto for inválido ou estiver fora da faixa aceita.
 */
bool tryParseMilliseconds(const String& text, unsigned long& valueMs)
{
    String normalizedText = text;
    normalizedText.trim();

    if (normalizedText.length() == 0)
    {
        return false;
    }

    for (unsigned int index = 0; index < normalizedText.length(); ++index)
    {
        if (!isDigit(normalizedText[index]))
        {
            return false;
        }
    }

    const unsigned long parsedValue = normalizedText.toInt();

    if (parsedValue < kMinimumIntervalMs || parsedValue > kMaximumIntervalMs)
    {
        return false;
    }

    valueMs = parsedValue;
    return true;
}

/*!
 * @brief Exibe no Monitor Serial o estado atual do controlador.
 *
 * Mostra informações como o pino utilizado, o estado do relé, se o
 * modo automático está ativado, o intervalo configurado e a duração
 * de pulso atualmente definida.
 */
void printRelayStatus()
{
    Serial.println(F("----- RELAY STATUS -----"));
    Serial.print(F("Relay pin: D"));
    Serial.println(kRelayPin);

    Serial.print(F("Relay state: "));
    Serial.println(g_isRelayOn ? F("ON") : F("OFF"));

    Serial.print(F("Automatic mode: "));
    Serial.println(g_isAutoModeEnabled ? F("ENABLED") : F("DISABLED"));

    Serial.print(F("Automatic interval (ms): "));
    Serial.println(g_autoToggleIntervalMs);

    Serial.print(F("Pulse mode active: "));
    Serial.println(g_isPulseActive ? F("YES") : F("NO"));

    Serial.print(F("Pulse duration (ms): "));
    Serial.println(g_pulseDurationMs);

    Serial.println(F("------------------------"));
}

/*!
 * @brief Define explicitamente o novo estado do relé.
 *
 * Atualiza a variável global que representa o estado do relé, aplica
 * esse estado ao pino físico e informa no Monitor Serial o resultado
 * da operação.
 *
 * @param turnOn true para ligar o relé, false para desligar.
 */
void setRelayState(const bool turnOn)
{
    g_isRelayOn = turnOn;
    applyRelayState();

    Serial.print(F("Relay state changed to: "));
    Serial.println(g_isRelayOn ? F("ON") : F("OFF"));
}

/*!
 * @brief Inverte o estado atual do relé.
 *
 * Se o relé estiver ligado, ele será desligado. Se estiver desligado,
 * ele será ligado.
 */
void toggleRelayState()
{
    setRelayState(!g_isRelayOn);
}

/*!
 * @brief Encerra o modo de pulso, caso ele esteja ativo.
 *
 * Esta função apenas marca internamente que não há mais um pulso em
 * andamento. O desligamento físico do relé é realizado em outra etapa
 * quando necessário.
 */
void stopPulseMode()
{
    g_isPulseActive = false;
}

/*!
 * @brief Inicia um pulso no relé usando a duração configurada.
 *
 * O modo de pulso em andamento é reiniciado, o instante inicial é
 * registrado e o relé é ligado imediatamente. O desligamento será
 * tratado posteriormente pela rotina de atualização do pulso.
 */
void startPulse()
{
    stopPulseMode();
    g_pulseStartTimeMs = millis();
    g_isPulseActive = true;

    setRelayState(true);

    Serial.print(F("Pulse started. Duration (ms): "));
    Serial.println(g_pulseDurationMs);
}

/*!
 * @brief Exibe a lista de comandos disponíveis no Monitor Serial.
 *
 * Também informa a faixa aceita para os parâmetros de tempo em
 * milissegundos.
 */
void printHelp()
{
    Serial.println(F(""));
    Serial.println(F("Available commands:"));
    Serial.println(F("  ON             -> Turn relay ON"));
    Serial.println(F("  OFF            -> Turn relay OFF"));
    Serial.println(F("  TOGGLE         -> Invert relay state"));
    Serial.println(F("  STATUS         -> Show current status"));
    Serial.println(F("  PING           -> Test serial communication"));
    Serial.println(F("  HELP           -> Show this help message"));
    Serial.println(F("  AUTO ON        -> Enable automatic mode"));
    Serial.println(F("  AUTO OFF       -> Disable automatic mode"));
    Serial.println(F("  AUTO <ms>      -> Set interval and enable automatic mode"));
    Serial.println(F("  INTERVAL <ms>  -> Set automatic interval only"));
    Serial.println(F("  PULSE          -> Execute one pulse using current duration"));
    Serial.println(F("  PULSE <ms>     -> Set pulse duration and execute one pulse"));
    Serial.println(F("  PULSECFG <ms>  -> Set pulse duration only"));
    Serial.println(F(""));
    Serial.print(F("Accepted range for time values: "));
    Serial.print(kMinimumIntervalMs);
    Serial.print(F(" to "));
    Serial.print(kMaximumIntervalMs);
    Serial.println(F(" ms"));
    Serial.println(F(""));
}

/*!
 * @brief Processa comandos relacionados ao modo automático.
 *
 * Reconhece os subcomandos AUTO ON, AUTO OFF e AUTO <ms>, permitindo
 * habilitar, desabilitar ou configurar o intervalo do modo automático.
 *
 * @param command Comando já normalizado recebido pela aplicação.
 */
void processAutoCommand(const String& command)
{
    if (command == F("AUTO ON"))
    {
        g_isAutoModeEnabled = true;
        g_lastAutoToggleTimeMs = millis();

        Serial.println(F("Automatic mode enabled."));
        Serial.print(F("Automatic interval (ms): "));
        Serial.println(g_autoToggleIntervalMs);
        return;
    }

    if (command == F("AUTO OFF"))
    {
        g_isAutoModeEnabled = false;
        Serial.println(F("Automatic mode disabled."));
        return;
    }

    if (command.startsWith(F("AUTO ")))
    {
        unsigned long intervalMs = 0UL;
        const String intervalText = command.substring(5);

        if (!tryParseMilliseconds(intervalText, intervalMs))
        {
            Serial.println(F("Invalid AUTO interval."));
            return;
        }

        g_autoToggleIntervalMs = intervalMs;
        g_isAutoModeEnabled = true;
        g_lastAutoToggleTimeMs = millis();

        Serial.print(F("Automatic interval updated to: "));
        Serial.print(g_autoToggleIntervalMs);
        Serial.println(F(" ms"));
        Serial.println(F("Automatic mode enabled."));
        return;
    }

    Serial.println(F("Unknown AUTO command."));
}

/*!
 * @brief Processa comandos de ajuste de intervalo automático.
 *
 * Reconhece o comando INTERVAL <ms> e atualiza apenas o valor do
 * intervalo, sem necessariamente alterar o estado do modo automático.
 *
 * @param command Comando já normalizado recebido pela aplicação.
 */
void processIntervalCommand(const String& command)
{
    if (!command.startsWith(F("INTERVAL ")))
    {
        Serial.println(F("Unknown INTERVAL command."));
        return;
    }

    unsigned long intervalMs = 0UL;
    const String intervalText = command.substring(9);

    if (!tryParseMilliseconds(intervalText, intervalMs))
    {
        Serial.println(F("Invalid interval value."));
        return;
    }

    g_autoToggleIntervalMs = intervalMs;

    Serial.print(F("Automatic interval updated to: "));
    Serial.print(g_autoToggleIntervalMs);
    Serial.println(F(" ms"));
}

/*!
 * @brief Processa comandos relacionados ao modo de pulso.
 *
 * Reconhece os comandos PULSE, PULSE <ms> e PULSECFG <ms>, permitindo
 * iniciar um pulso imediato, configurar a duração do pulso e iniciar
 * o acionamento em seguida, ou apenas alterar a configuração.
 *
 * @param command Comando já normalizado recebido pela aplicação.
 */
void processPulseCommand(const String& command)
{
    if (command == F("PULSE"))
    {
        startPulse();
        return;
    }

    if (command.startsWith(F("PULSE ")))
    {
        unsigned long durationMs = 0UL;
        const String durationText = command.substring(6);

        if (!tryParseMilliseconds(durationText, durationMs))
        {
            Serial.println(F("Invalid pulse duration."));
            return;
        }

        g_pulseDurationMs = durationMs;

        Serial.print(F("Pulse duration updated to: "));
        Serial.print(g_pulseDurationMs);
        Serial.println(F(" ms"));

        startPulse();
        return;
    }

    if (command.startsWith(F("PULSECFG ")))
    {
        unsigned long durationMs = 0UL;
        const String durationText = command.substring(9);

        if (!tryParseMilliseconds(durationText, durationMs))
        {
            Serial.println(F("Invalid pulse configuration value."));
            return;
        }

        g_pulseDurationMs = durationMs;

        Serial.print(F("Pulse duration configured to: "));
        Serial.print(g_pulseDurationMs);
        Serial.println(F(" ms"));
        return;
    }

    Serial.println(F("Unknown PULSE command."));
}

/*!
 * @brief Processa um comando completo recebido pela Serial.
 *
 * O texto é normalizado e comparado com a lista de comandos aceitos.
 * Dependendo do conteúdo, a função liga, desliga, alterna o relé,
 * consulta o estado, aciona modos automáticos ou mostra ajuda.
 *
 * @param rawCommand Texto bruto recebido do Monitor Serial.
 */
void processCommand(const String& rawCommand)
{
    const String command = normalizeCommand(rawCommand);

    if (command.length() == 0)
    {
        return;
    }

    Serial.print(F("Received command: "));
    Serial.println(command);

    if (command == F("ON"))
    {
        stopPulseMode();
        setRelayState(true);
    }
    else if (command == F("OFF"))
    {
        stopPulseMode();
        setRelayState(false);
    }
    else if (command == F("TOGGLE"))
    {
        stopPulseMode();
        toggleRelayState();
    }
    else if (command == F("STATUS"))
    {
        printRelayStatus();
    }
    else if (command == F("PING"))
    {
        Serial.println(F("PONG"));
    }
    else if (command == F("HELP"))
    {
        printHelp();
    }
    else if (command.startsWith(F("AUTO")))
    {
        processAutoCommand(command);
    }
    else if (command.startsWith(F("INTERVAL")))
    {
        processIntervalCommand(command);
    }
    else if (command.startsWith(F("PULSE")))
    {
        processPulseCommand(command);
    }
    else
    {
        Serial.println(F("Unknown command."));
        Serial.println(F("Type HELP to see the available commands."));
    }
}

/*!
 * @brief Finaliza e processa o comando atualmente armazenado no buffer.
 *
 * Caso exista texto acumulado em g_serialCommand, o comando é enviado
 * para a rotina de processamento e o buffer é limpo em seguida.
 */
void flushBufferedCommand()
{
    if (g_serialCommand.length() == 0)
    {
        return;
    }

    processCommand(g_serialCommand);
    g_serialCommand = "";
}

/*!
 * @brief Lê os caracteres recebidos pela Serial e monta comandos.
 *
 * A função aceita comandos encerrados por quebra de linha, retorno de
 * carro ou por tempo de inatividade entre bytes, permitindo uso mais
 * flexível do Monitor Serial.
 */
void readSerialCommands()
{
    while (Serial.available() > 0)
    {
        const char receivedChar = static_cast<char>(Serial.read());
        g_lastSerialByteTimeMs = millis();

        if (receivedChar == '\n' || receivedChar == '\r')
        {
            flushBufferedCommand();
        }
        else
        {
            g_serialCommand += receivedChar;
        }
    }

    if (g_serialCommand.length() > 0)
    {
        const unsigned long currentTimeMs = millis();

        if (currentTimeMs - g_lastSerialByteTimeMs >= kCommandIdleTimeoutMs)
        {
            flushBufferedCommand();
        }
    }
}

/*!
 * @brief Atualiza o modo automático de alternância do relé.
 *
 * Quando o modo automático está ativo e não há pulso em execução, a
 * função verifica se o intervalo programado foi atingido para inverter
 * o estado do relé.
 */
void handleAutoMode()
{
    if (!g_isAutoModeEnabled || g_isPulseActive)
    {
        return;
    }

    const unsigned long currentTimeMs = millis();

    if (currentTimeMs - g_lastAutoToggleTimeMs >= g_autoToggleIntervalMs)
    {
        g_lastAutoToggleTimeMs = currentTimeMs;
        toggleRelayState();
        Serial.println(F("Automatic mode executed a relay toggle."));
    }
}

/*!
 * @brief Atualiza o ciclo de vida do modo de pulso.
 *
 * Enquanto um pulso estiver ativo, a função verifica se o tempo de
 * duração configurado já foi atingido. Quando isso ocorre, o pulso é
 * encerrado e o relé é desligado.
 */
void handlePulseMode()
{
    if (!g_isPulseActive)
    {
        return;
    }

    const unsigned long currentTimeMs = millis();

    if (currentTimeMs - g_pulseStartTimeMs >= g_pulseDurationMs)
    {
        stopPulseMode();
        setRelayState(false);
        Serial.println(F("Pulse finished."));
    }
}

/*!
 * @brief Função de configuração inicial do Arduino.
 *
 * Define o pino do relé como saída, inicializa a comunicação Serial,
 * garante que o relé comece desligado e imprime as instruções iniciais
 * de uso no Monitor Serial.
 */
void setup()
{
    pinMode(kRelayPin, OUTPUT);
    Serial.begin(kSerialBaudRate);

    setRelayState(false);

    Serial.println(F(""));
    Serial.println(F("Relay serial controller started."));
    Serial.print(F("Baud rate: "));
    Serial.println(kSerialBaudRate);
    Serial.println(F("Commands can be sent with or without line ending."));
    printHelp();
    printRelayStatus();
}

/*!
 * @brief Laço principal do programa.
 *
 * Executa continuamente a leitura dos comandos recebidos pela Serial,
 * atualiza o estado do modo de pulso e controla o modo automático de
 * alternância do relé.
 */
void loop()
{
    readSerialCommands();
    handlePulseMode();
    handleAutoMode();
}
