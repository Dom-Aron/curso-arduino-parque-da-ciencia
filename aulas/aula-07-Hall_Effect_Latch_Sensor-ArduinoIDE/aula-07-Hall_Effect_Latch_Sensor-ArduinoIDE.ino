/*
  Hall Effect Latch Sensor - Rotation Measurement
  ------------------------------------------------
  Sensor:
  - US1881 / U18 Hall latch sensor
  - Open-drain output

  This sketch computes:
  - Rotational frequency [Hz]
  - Angular velocity [rad/s]
  - Centripetal acceleration [m/s^2]
  - RPM

  Wiring:
  - Sensor pin 1 (VDD) -> 5V
  - Sensor pin 2 (GND) -> GND
  - Sensor pin 3 (OUT) -> D3
  - Recommended: 100 nF capacitor between VDD and GND near the sensor

  Notes:
  - INPUT_PULLUP is enabled because the sensor output is open-drain.
  - The sensor is a Hall latch, so CHANGE interrupt is appropriate.
  - For manual tests with alternating North/South magnetic poles,
    2 edges per revolution is a good initial assumption.
*/

constexpr uint8_t kHallPin = 3;
constexpr unsigned long kBaudRate = 115200UL;

// Physical setup
constexpr float kRadiusMeters = 0.05f;
constexpr float kEdgesPerRevolution = 2.0f;

// Timing and filtering
constexpr unsigned long kPublishIntervalMs = 200UL;
constexpr unsigned long kSignalTimeoutUs = 3000000UL;
constexpr unsigned long kMinEdgeIntervalUs = 1000UL;
constexpr float kFilterAlpha = 1.0f;

struct Measurement
{
  float rotationFrequencyHz;
  float angularVelocityRadPerSecond;
  float centripetalAccelerationMetersPerSecondSquared;
  float rpm;
};

volatile uint32_t gEdgeCount = 0U;
volatile uint32_t gLastEdgeMicros = 0U;
volatile uint32_t gLastEdgePeriodUs = 0U;

/**
 * @brief Interrupt Service Routine for Hall sensor edges.
 */
void onHallEdge()
{
  const uint32_t nowMicros = micros();

  if (gLastEdgeMicros != 0U)
  {
    const uint32_t edgeIntervalUs = nowMicros - gLastEdgeMicros;

    if (edgeIntervalUs < kMinEdgeIntervalUs)
    {
      return;
    }

    gLastEdgePeriodUs = edgeIntervalUs;
  }

  gLastEdgeMicros = nowMicros;
  ++gEdgeCount;
}

/**
 * @brief Atomically copies volatile edge data.
 *
 * @param edgeCount Total number of detected edges.
 * @param lastEdgeMicros Timestamp of the most recent valid edge.
 * @param lastEdgePeriodUs Time interval between the two most recent valid edges.
 */
void snapshotEdgeData(
  uint32_t& edgeCount,
  uint32_t& lastEdgeMicros,
  uint32_t& lastEdgePeriodUs)
{
  noInterrupts();
  edgeCount = gEdgeCount;
  lastEdgeMicros = gLastEdgeMicros;
  lastEdgePeriodUs = gLastEdgePeriodUs;
  interrupts();
}

/**
 * @brief Computes frequency, angular velocity, centripetal acceleration, and RPM.
 *
 * @param totalEdges Total number of detected edges.
 * @param lastEdgeMicros Timestamp of the last valid edge.
 * @param lastEdgePeriodUs Time interval between the two most recent valid edges.
 * @param nowMicros Current timestamp.
 * @param previousEdgeCount Previous total edge count at last publication.
 * @param previousPublishMicros Previous publication timestamp.
 * @param filteredEdgeFrequencyHz State variable for exponential smoothing.
 * @return Measurement Computed physical quantities.
 */
Measurement computeMeasurement(
  const uint32_t totalEdges,
  const uint32_t lastEdgeMicros,
  const uint32_t lastEdgePeriodUs,
  const uint32_t nowMicros,
  uint32_t& previousEdgeCount,
  uint32_t& previousPublishMicros,
  float& filteredEdgeFrequencyHz)
{
  const uint32_t elapsedUs = nowMicros - previousPublishMicros;
  const uint32_t deltaEdges = totalEdges - previousEdgeCount;

  const bool timedOut =
    (lastEdgeMicros == 0U) ||
    ((nowMicros - lastEdgeMicros) > kSignalTimeoutUs);

  float edgeFrequencyHz = 0.0f;

  if (deltaEdges > 0U && elapsedUs > 0U)
  {
    edgeFrequencyHz =
      (1.0e6f * static_cast<float>(deltaEdges)) /
      static_cast<float>(elapsedUs);
  }
  else if (!timedOut && lastEdgePeriodUs > 0U)
  {
    edgeFrequencyHz = 1.0e6f / static_cast<float>(lastEdgePeriodUs);
  }

  if (timedOut)
  {
    filteredEdgeFrequencyHz = 0.0f;
  }
  else
  {
    filteredEdgeFrequencyHz =
      (kFilterAlpha * edgeFrequencyHz) +
      ((1.0f - kFilterAlpha) * filteredEdgeFrequencyHz);
  }

  previousEdgeCount = totalEdges;
  previousPublishMicros = nowMicros;

  const float rotationFrequencyHz =
    filteredEdgeFrequencyHz / kEdgesPerRevolution;

  const float angularVelocityRadPerSecond =
    2.0f * PI * rotationFrequencyHz;

  const float centripetalAccelerationMetersPerSecondSquared =
    angularVelocityRadPerSecond *
    angularVelocityRadPerSecond *
    kRadiusMeters;

  const float rpm = rotationFrequencyHz * 60.0f;

  Measurement measurement;
  measurement.rotationFrequencyHz = rotationFrequencyHz;
  measurement.angularVelocityRadPerSecond = angularVelocityRadPerSecond;
  measurement.centripetalAccelerationMetersPerSecondSquared =
    centripetalAccelerationMetersPerSecondSquared;
  measurement.rpm = rpm;

  return measurement;
}

/**
 * @brief Prints values in a format suitable for Arduino IDE Serial Plotter.
 *
 * The Arduino Serial Plotter reads numeric columns separated by tabs.
 */
void printForArduinoPlotter(const Measurement& measurement)
{
  Serial.print("freq_hz:");
  Serial.print(measurement.rotationFrequencyHz, 4);
  Serial.print(',');

  Serial.print("omega_rad_s:");
  Serial.print(measurement.angularVelocityRadPerSecond, 4);
  Serial.print(',');

  Serial.print("ac_m_s2:");
  Serial.print(measurement.centripetalAccelerationMetersPerSecondSquared, 4);
  Serial.print(',');

  Serial.print("rpm:");
  Serial.println(measurement.rpm, 2);
}

void setup()
{
  Serial.begin(kBaudRate);
  pinMode(kHallPin, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(kHallPin),
    onHallEdge,
    CHANGE);

  delay(300);
}

void loop()
{
  static uint32_t previousPublishMillis = 0U;
  static uint32_t previousPublishMicros = 0U;
  static uint32_t previousEdgeCount = 0U;
  static float filteredEdgeFrequencyHz = 0.0f;

  const uint32_t nowMillis = millis();

  if ((nowMillis - previousPublishMillis) < kPublishIntervalMs)
  {
    return;
  }

  previousPublishMillis = nowMillis;

  uint32_t edgeCount = 0U;
  uint32_t lastEdgeMicros = 0U;
  uint32_t lastEdgePeriodUs = 0U;

  snapshotEdgeData(edgeCount, lastEdgeMicros, lastEdgePeriodUs);

  const uint32_t nowMicros = micros();

  if (previousPublishMicros == 0U)
  {
    previousPublishMicros = nowMicros;
  }

  const Measurement measurement = computeMeasurement(
    edgeCount,
    lastEdgeMicros,
    lastEdgePeriodUs,
    nowMicros,
    previousEdgeCount,
    previousPublishMicros,
    filteredEdgeFrequencyHz);

  printForArduinoPlotter(measurement);
}