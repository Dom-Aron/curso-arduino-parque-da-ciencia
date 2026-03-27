/*
    Relay controller for Arduino using Serial Monitor.

    Features:
    - Controls a relay connected to digital pin D2.
    - Supports manual ON/OFF/TOGGLE commands.
    - Supports automatic toggle mode with configurable interval.
    - Supports single pulse mode with configurable pulse duration.
    - Uses millis() instead of delay() to keep the system responsive.

    Supported commands:
    - ON
    - OFF
    - TOGGLE
    - STATUS
    - PING
    - HELP
    - AUTO ON
    - AUTO OFF
    - AUTO <ms>
    - INTERVAL <ms>
    - PULSE
    - PULSE <ms>
    - PULSECFG <ms>

    Important:
    Many relay modules are ACTIVE LOW instead of ACTIVE HIGH.
    If your relay behavior is inverted, swap:
    - kRelayActiveLevel
    - kRelayInactiveLevel
*/

static constexpr int kRelayPin = 2;
static constexpr long kSerialBaudRate = 115200;
static constexpr unsigned long kDefaultAutoIntervalMs = 2000UL;
static constexpr unsigned long kDefaultPulseDurationMs = 500UL;
static constexpr unsigned long kCommandIdleTimeoutMs = 120UL;
static constexpr unsigned long kMinimumIntervalMs = 50UL;
static constexpr unsigned long kMaximumIntervalMs = 600000UL;

/*
    Relay logic levels.

    ACTIVE HIGH relay:
        kRelayActiveLevel   = HIGH
        kRelayInactiveLevel = LOW

    ACTIVE LOW relay:
        kRelayActiveLevel   = LOW
        kRelayInactiveLevel = HIGH
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

/*
    Removes leading and trailing spaces and converts the command to uppercase.
*/
String normalizeCommand(String command)
{
    command.trim();
    command.toUpperCase();
    return command;
}

/*
    Applies the relay state to the physical output pin.
*/
void applyRelayState()
{
    digitalWrite(kRelayPin, g_isRelayOn ? kRelayActiveLevel : kRelayInactiveLevel);
}

/*
    Parses a positive unsigned long value from a command suffix.

    Returns true if parsing succeeds and the value is valid.
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

/*
    Prints the current relay state and controller configuration.
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

/*
    Updates the relay state and prints the result.
*/
void setRelayState(const bool turnOn)
{
    g_isRelayOn = turnOn;
    applyRelayState();

    Serial.print(F("Relay state changed to: "));
    Serial.println(g_isRelayOn ? F("ON") : F("OFF"));
}

/*
    Toggles the relay state and prints the result.
*/
void toggleRelayState()
{
    setRelayState(!g_isRelayOn);
}

/*
    Stops any active pulse session.
*/
void stopPulseMode()
{
    g_isPulseActive = false;
}

/*
    Starts a relay pulse using the configured pulse duration.
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

/*
    Prints the available serial commands.
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

/*
    Processes AUTO commands.
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

/*
    Processes INTERVAL commands.
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

/*
    Processes PULSE and PULSECFG commands.
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

/*
    Processes a single serial command.
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

/*
    Finalizes and processes the buffered command.
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

/*
    Reads serial data and accepts commands with or without line ending.
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

/*
    Handles automatic relay toggling.
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

/*
    Handles the pulse lifecycle.
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

/*
    Arduino setup function.
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

/*
    Arduino main loop function.
*/
void loop()
{
    readSerialCommands();
    handlePulseMode();
    handleAutoMode();
}