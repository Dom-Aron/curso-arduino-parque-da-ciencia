#include <Arduino.h>

/*
  US1881 / U18 Hall latch rotation measurement
  --------------------------------------------Í
  This sketch is tailored for:
  - Hall latch sensors such as the U18-marked US1881
  - VS Code serial-plotter extension using lines like:
      >freq_hz:12.3,omega_rad_s:77.3,ac_m_s2:298.7,rpm:738.0

  What it computes:
  - Rotational frequency [Hz]
  - Angular velocity [rad/s]
  - Centripetal acceleration [m/s^2]
  - RPM

  Important:
  - The sensor is open-drain, so INPUT_PULLUP is used.
  - With a Hall latch and alternating N/S detection, CHANGE interrupt
    is usually the correct choice.
*/

namespace config
{
    constexpr uint8_t kHallPin = 3;
    constexpr unsigned long kBaudRate = 115200UL;

    // Physical setup
    constexpr float kRadiusMeters = 0.050f;      // Radius from axis to point of interest [m]

    /*
      For a Hall latch counting North/South alternation:
      - If one full mechanical revolution produces 2 output changes,
        keep kEdgesPerRevolution = 2.0f
      - If your geometry produces 1 valid output change per revolution,
        set it to 1.0f
    */
    constexpr float kEdgesPerRevolution = 2.0f;

    // Timing and filtering
    constexpr unsigned long kPublishIntervalMs = 100UL;
    constexpr unsigned long kSignalTimeoutUs = 500000UL;  // 0.5 s without edges => zero speed
    constexpr unsigned long kMinEdgeIntervalUs = 150UL;   // reject glitches
    constexpr float kFilterAlpha = 0.25f;                 // exponential smoothing

    // Hall latch mode: count both transitions
    constexpr int kInterruptMode = CHANGE;
}

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

        if (edgeIntervalUs < config::kMinEdgeIntervalUs)
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
 * @brief Computes rotation metrics from edge timing.
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
        ((nowMicros - lastEdgeMicros) > config::kSignalTimeoutUs);

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
            (config::kFilterAlpha * edgeFrequencyHz) +
            ((1.0f - config::kFilterAlpha) * filteredEdgeFrequencyHz);
    }

    previousEdgeCount = totalEdges;
    previousPublishMicros = nowMicros;

    const float rotationFrequencyHz =
        filteredEdgeFrequencyHz / config::kEdgesPerRevolution;

    const float angularVelocityRadPerSecond =
        2.0f * PI * rotationFrequencyHz;

    const float centripetalAccelerationMetersPerSecondSquared =
        angularVelocityRadPerSecond *
        angularVelocityRadPerSecond *
        config::kRadiusMeters;

    const float rpm = rotationFrequencyHz * 60.0f;

    return {
        rotationFrequencyHz,
        angularVelocityRadPerSecond,
        centripetalAccelerationMetersPerSecondSquared,
        rpm
    };
}

/**
 * @brief Sends one line in the format expected by the VS Code serial-plotter extension.
 */
void printForSerialPlotter(const Measurement& measurement)
{
    Serial.print(">freq_hz:");
    Serial.print(measurement.rotationFrequencyHz, 4);

    Serial.print(",omega_rad_s:");
    Serial.print(measurement.angularVelocityRadPerSecond, 4);

    Serial.print(",ac_m_s2:");
    Serial.print(measurement.centripetalAccelerationMetersPerSecondSquared, 4);

    Serial.print(",rpm:");
    Serial.println(measurement.rpm, 2);
}

void setup()
{
    Serial.begin(config::kBaudRate);

    pinMode(config::kHallPin, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(config::kHallPin),
        onHallEdge,
        config::kInterruptMode);

    delay(300);
    Serial.println("US1881 Hall latch monitor started");
}

void loop()
{
    static uint32_t previousPublishMillis = 0U;
    static uint32_t previousPublishMicros = 0U;
    static uint32_t previousEdgeCount = 0U;
    static float filteredEdgeFrequencyHz = 0.0f;

    const uint32_t nowMillis = millis();

    if ((nowMillis - previousPublishMillis) < config::kPublishIntervalMs)
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

    printForSerialPlotter(measurement);
}