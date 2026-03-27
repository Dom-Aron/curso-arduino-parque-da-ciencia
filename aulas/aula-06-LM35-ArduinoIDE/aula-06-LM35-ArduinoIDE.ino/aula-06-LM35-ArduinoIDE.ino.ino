/*
  Stable LM35 control for Arduino IDE 2.

  Features:
  - LM35 on A0
  - D2 turns ON when temperature is below the ON threshold
  - D2 turns OFF when temperature is above the OFF threshold
  - Automatic ambient calibration at startup
  - INTERNAL 1.1 V analog reference for better resolution
  - Trimmed average to reject spikes
  - Separate control filter and display filter
  - Minimum dwell time to avoid fast toggling
  - Named output for Arduino IDE Serial Plotter
*/

static constexpr bool kUseSerialPlotter = true;

static constexpr uint8_t kLm35Pin = A0;
static constexpr uint8_t kOutputPin = 2;

static constexpr float kAdcReferenceVoltage = 1.1F;

/*
  Automatic thresholds based on ambient temperature.

  Example:
  ambient = 29.0 C
  threshold_on  = 29.5 C
  threshold_off = 29.9 C

  Behavior:
  - Turn ON  when control temperature goes below threshold_on
  - Turn OFF when control temperature goes above threshold_off
*/
static constexpr float kThresholdOffsetAboveAmbientC = 0.15F;
static constexpr float kHysteresisC = 1.0F;

/*
  Filtering:
  - control filter: faster, used for switching
  - display filter: smoother, used for plotting
*/
static constexpr float kControlFilterAlpha = 0.20F;
static constexpr float kDisplayFilterAlpha = 0.12F;

/*
  ADC sampling.
*/
static constexpr uint8_t kSamplesPerReading = 32;

/*
  Timing.
*/
static constexpr unsigned long kSampleIntervalMs = 350UL;
static constexpr unsigned long kCalibrationDurationMs = 4000UL;

/*
  Minimum time the output must remain in the current state
  before another transition is allowed.
*/
static constexpr unsigned long kMinimumStateHoldMs = 1500UL;

bool g_outputState = false;

bool g_controlFilterInitialized = false;
bool g_displayFilterInitialized = false;

unsigned long g_lastSampleTimeMs = 0UL;
unsigned long g_lastStateChangeTimeMs = 0UL;

float g_ambientTemperatureC = 0.0F;
float g_thresholdOnC = 25.0F;
float g_thresholdOffC = 25.4F;

float g_controlTemperatureC = 0.0F;
float g_displayTemperatureC = 0.0F;

/**
 * @brief Read the ADC multiple times, discard the minimum and maximum,
 * and return the trimmed average.
 *
 * This helps reject short spikes.
 *
 * @return Trimmed averaged ADC value.
 */
float read_adc_trimmed_average()
{
  unsigned long sum = 0UL;
  int minValue = 1023;
  int maxValue = 0;

  for (uint8_t i = 0; i < kSamplesPerReading; ++i)
  {
    const int currentValue = analogRead(kLm35Pin);

    sum += static_cast<unsigned long>(currentValue);

    if (currentValue < minValue)
    {
      minValue = currentValue;
    }

    if (currentValue > maxValue)
    {
      maxValue = currentValue;
    }

    delay(2);
  }

  if (kSamplesPerReading >= 3)
  {
    sum -= static_cast<unsigned long>(minValue);
    sum -= static_cast<unsigned long>(maxValue);

    return static_cast<float>(sum) /
           static_cast<float>(kSamplesPerReading - 2);
  }

  return static_cast<float>(sum) / static_cast<float>(kSamplesPerReading);
}

float read_adc_block_average(const uint8_t samplesPerBlock)
{
  unsigned long sum = 0UL;

  for (uint8_t i = 0; i < samplesPerBlock; ++i)
  {
    sum += static_cast<unsigned long>(analogRead(kLm35Pin));
    delay(2);
  }

  return static_cast<float>(sum) / static_cast<float>(samplesPerBlock);
}

float median_of_five(
    float a,
    float b,
    float c,
    float d,
    float e)
{
  float values[5] = {a, b, c, d, e};

  for (uint8_t i = 0; i < 4; ++i)
  {
    for (uint8_t j = i + 1; j < 5; ++j)
    {
      if (values[j] < values[i])
      {
        const float temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }

  return values[2];
}

float read_adc_stable()
{
  const float b1 = read_adc_block_average(8);
  const float b2 = read_adc_block_average(8);
  const float b3 = read_adc_block_average(8);
  const float b4 = read_adc_block_average(8);
  const float b5 = read_adc_block_average(8);

  return median_of_five(b1, b2, b3, b4, b5);
}

/**
 * @brief Convert ADC reading to voltage.
 *
 * @param adcValue Averaged ADC reading.
 * @return Voltage in volts.
 */
float convert_adc_to_voltage(const float adcValue)
{
  return (adcValue * kAdcReferenceVoltage) / 1023.0F;
}

/**
 * @brief Convert LM35 voltage to Celsius.
 *
 * LM35 scale:
 * 10 mV / degree Celsius
 *
 * @param voltage Sensor voltage in volts.
 * @return Temperature in Celsius.
 */
float convert_voltage_to_temperature_c(const float voltage)
{
  return voltage * 100.0F;
}

/**
 * @brief Read raw temperature in Celsius.
 *
 * @return Raw temperature.
 */
float read_raw_temperature_c()
{
  const float adcAverage = read_adc_stable();
  const float voltage = convert_adc_to_voltage(adcAverage);
  return convert_voltage_to_temperature_c(voltage);
}

/**
 * @brief Apply exponential filter to a signal.
 *
 * @param currentValue Current input value.
 * @param alpha Filter alpha.
 * @param filterState Persistent filter state.
 * @param isInitialized Filter initialization flag.
 * @return Filtered value.
 */
float apply_exponential_filter(
    const float currentValue,
    const float alpha,
    float& filterState,
    bool& isInitialized)
{
  if (!isInitialized)
  {
    filterState = currentValue;
    isInitialized = true;
    return filterState;
  }

  filterState =
      (alpha * currentValue) +
      ((1.0F - alpha) * filterState);

  return filterState;
}

/**
 * @brief Calibrate ambient temperature during startup.
 *
 * Keep the sensor untouched during this phase.
 *
 * @return Estimated ambient temperature in Celsius.
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

    if (!kUseSerialPlotter)
    {
      Serial.print("Calibrating ambient... sample=");
      Serial.print(sampleCount);
      Serial.print(" | Temp=");
      Serial.print(rawTemperatureC, 2);
      Serial.println(" C");
    }

    delay(80);
  }

  if (sampleCount == 0U)
  {
    return read_raw_temperature_c();
  }

  return sumTemperatureC / static_cast<float>(sampleCount);
}

/**
 * @brief Update output using Schmitt-trigger logic plus minimum dwell time.
 *
 * Rules:
 * - If currently ON, only turn OFF above threshold_off
 * - If currently OFF, only turn ON below threshold_on
 * - Ignore state changes until the minimum hold time has elapsed
 *
 * @param controlTemperatureC Stable temperature used for decisions.
 * @param currentTimeMs Current time from millis().
 */
void update_output_state(
    const float controlTemperatureC,
    const unsigned long currentTimeMs)
{
  if ((currentTimeMs - g_lastStateChangeTimeMs) < kMinimumStateHoldMs)
  {
    digitalWrite(kOutputPin, g_outputState ? HIGH : LOW);
    return;
  }

  if (g_outputState)
  {
    if (controlTemperatureC > g_thresholdOffC)
    {
      g_outputState = false;
      g_lastStateChangeTimeMs = currentTimeMs;
    }
  }
  else
  {
    if (controlTemperatureC < g_thresholdOnC)
    {
      g_outputState = true;
      g_lastStateChangeTimeMs = currentTimeMs;
    }
  }

  digitalWrite(kOutputPin, g_outputState ? HIGH : LOW);
}

/**
 * @brief Create a visual marker for D2 on the same plot.
 *
 * @return Marker value in Celsius scale.
 */
float get_output_marker_c()
{
  const float markerOffsetC = 0.35F;

  if (g_outputState)
  {
    return g_thresholdOnC - markerOffsetC;
  }

  return g_thresholdOffC + markerOffsetC;
}

/**
 * @brief Print text diagnostics for Serial Monitor.
 *
 * @param rawTemperatureC Raw temperature.
 * @param controlTemperatureC Control temperature.
 * @param displayTemperatureC Display temperature.
 */
void print_serial_monitor_status(
    const float rawTemperatureC,
    const float controlTemperatureC,
    const float displayTemperatureC)
{
  Serial.print("AmbientC=");
  Serial.print(g_ambientTemperatureC, 2);

  Serial.print(" | ThresholdOnC=");
  Serial.print(g_thresholdOnC, 2);

  Serial.print(" | ThresholdOffC=");
  Serial.print(g_thresholdOffC, 2);

  Serial.print(" | RawC=");
  Serial.print(rawTemperatureC, 2);

  Serial.print(" | ControlC=");
  Serial.print(controlTemperatureC, 2);

  Serial.print(" | DisplayC=");
  Serial.print(displayTemperatureC, 2);

  Serial.print(" | D2=");
  Serial.println(g_outputState ? "ON" : "OFF");
}

/**
 * @brief Send named data for Arduino IDE Serial Plotter.
 *
 * @param rawTemperatureC Raw temperature.
 * @param controlTemperatureC Control temperature.
 * @param displayTemperatureC Display temperature.
 */
void send_serial_plotter_data(
    const float rawTemperatureC,
    const float controlTemperatureC,
    const float displayTemperatureC)
{
  Serial.print("raw_c:");
  Serial.print(rawTemperatureC, 2);
  Serial.print('\t');

  Serial.print("control_c:");
  Serial.print(controlTemperatureC, 2);
  Serial.print('\t');

  Serial.print("display_c:");
  Serial.print(displayTemperatureC, 2);
  Serial.print('\t');

  Serial.print("ambient_c:");
  Serial.print(g_ambientTemperatureC, 2);
  Serial.print('\t');

  Serial.print("threshold_on_c:");
  Serial.print(g_thresholdOnC, 2);
  Serial.print('\t');

  Serial.print("threshold_off_c:");
  Serial.print(g_thresholdOffC, 2);
  Serial.print('\t');

  Serial.print("d2_marker_c:");
  Serial.println(get_output_marker_c(), 2);
}

void setup()
{
  pinMode(kOutputPin, OUTPUT);
  digitalWrite(kOutputPin, LOW);

  Serial.begin(115200);

  analogReference(INTERNAL);
  delay(100);

  /*
    Dummy reads to stabilize ADC after changing the reference.
  */
  for (uint8_t i = 0; i < 12; ++i)
  {
    analogRead(kLm35Pin);
    delay(5);
  }

  if (!kUseSerialPlotter)
  {
    Serial.println();
    Serial.println("LM35 automatic calibration started");
    Serial.println("Keep the sensor untouched during startup");
  }

  g_ambientTemperatureC = calibrate_ambient_temperature();

  g_thresholdOnC = g_ambientTemperatureC + kThresholdOffsetAboveAmbientC;
  g_thresholdOffC = g_thresholdOnC + kHysteresisC;

  /*
    Initialize filters at ambient temperature.
  */
  g_controlTemperatureC = g_ambientTemperatureC;
  g_displayTemperatureC = g_ambientTemperatureC;
  g_controlFilterInitialized = true;
  g_displayFilterInitialized = true;

  /*
    Start with the output state consistent with the calibrated ambient.
    Since ambient is below threshold_on, output starts ON.
  */
  g_outputState = true;
  g_lastStateChangeTimeMs = millis();
  digitalWrite(kOutputPin, HIGH);

  if (!kUseSerialPlotter)
  {
    Serial.println();
    Serial.println("Calibration finished");
    Serial.print("Ambient temperature: ");
    Serial.print(g_ambientTemperatureC, 2);
    Serial.println(" C");

    Serial.print("Threshold ON below: ");
    Serial.print(g_thresholdOnC, 2);
    Serial.println(" C");

    Serial.print("Threshold OFF above: ");
    Serial.print(g_thresholdOffC, 2);
    Serial.println(" C");

    Serial.print("Minimum state hold time: ");
    Serial.print(kMinimumStateHoldMs);
    Serial.println(" ms");

    Serial.println();
  }
}

void loop()
{
  const unsigned long currentTimeMs = millis();

  if ((currentTimeMs - g_lastSampleTimeMs) >= kSampleIntervalMs)
  {
    g_lastSampleTimeMs = currentTimeMs;

    const float rawTemperatureC = read_raw_temperature_c();

    const float controlTemperatureC = apply_exponential_filter(
        rawTemperatureC,
        kControlFilterAlpha,
        g_controlTemperatureC,
        g_controlFilterInitialized);

    const float displayTemperatureC = apply_exponential_filter(
        rawTemperatureC,
        kDisplayFilterAlpha,
        g_displayTemperatureC,
        g_displayFilterInitialized);

    update_output_state(controlTemperatureC, currentTimeMs);

    if (kUseSerialPlotter)
    {
      send_serial_plotter_data(
          rawTemperatureC,
          controlTemperatureC,
          displayTemperatureC);
    }
    else
    {
      print_serial_monitor_status(
          rawTemperatureC,
          controlTemperatureC,
          displayTemperatureC);
    }
  }
}