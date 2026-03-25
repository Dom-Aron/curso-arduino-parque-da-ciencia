#include <Arduino.h>

/*
  US1881 / U18 Hall latch rotation measurement
  --------------------------------------------
  This sketch is tailored for:
  - Hall latch sensors such as the U18-marked US1881
  - VS Code serial-plotter extension using lines like:
      >freq_hz:12.3,omega_rad_s:77.3,ac_m_s2:298.7,rpm:738.0

  Measurement strategy used here:
  - Count only one pulse type per revolution using a single interrupt edge.
  - Compute speed from the period between valid pulses.
  - Keep the last valid speed until a timeout is reached.

  Why this version is better for 10 RPM to 2000 RPM:
  - Window-based edge counting is very inaccurate at low RPM.
  - Period-based measurement remains stable at low speed and still works well
    at higher speed.

  Important notes:
  - The sensor is open-drain, so INPUT_PULLUP is used.
  - This version assumes one valid pulse per mechanical revolution.
  - If your final geometry produces a different number of valid pulses per
    revolution, update kPulsesPerRevolution accordingly.
*/

namespace config
{
    constexpr uint8_t kHallPin = 3;
    constexpr unsigned long kBaudRate = 115200UL;

    // Physical setup
    constexpr float kRadiusMeters = 0.168f;  // Radius from axis to point of interest [m]

    // Mechanical/electrical relation
    constexpr float kPulsesPerRevolution = 1.0f;

    // Timing and filtering
    constexpr unsigned long kPublishIntervalMs = 100UL;
    constexpr unsigned long kSignalTimeoutUs = 9000000UL;  // 9 s, suitable for ~10 RPM with 1 pulse/rev
    constexpr unsigned long kMinPulseIntervalUs = 2000UL;  // Reject glitches/noise
    constexpr float kFilterAlpha = 0.30f;                  // Exponential smoothing on rotational frequency

    // For an open-drain Hall latch with pull-up, the active pulse is usually LOW.
    // Counting only FALLING avoids double counting caused by CHANGE.
    constexpr int kInterruptMode = FALLING;
}

struct Measurement
{
    float rotationFrequencyHz;
    float angularVelocityRadPerSecond;
    float centripetalAccelerationMetersPerSecondSquared;
    float rpm;
};

volatile uint32_t gLastPulseMicros = 0U;
volatile uint32_t gLastPulsePeriodUs = 0U;

/**
 * @brief Interrupt Service Routine for valid Hall pulses.
 */
void onHallPulse()
{
    const uint32_t nowMicros = micros();

    if (gLastPulseMicros != 0U)
    {
        const uint32_t pulseIntervalUs = nowMicros - gLastPulseMicros;

        if (pulseIntervalUs < config::kMinPulseIntervalUs)
        {
            return;
        }

        gLastPulsePeriodUs = pulseIntervalUs;
    }

    gLastPulseMicros = nowMicros;
}

/**
 * @brief Atomically copies volatile pulse timing data.
 */
void snapshotPulseData(
    uint32_t& lastPulseMicros,
    uint32_t& lastPulsePeriodUs)
{
    noInterrupts();
    lastPulseMicros = gLastPulseMicros;
    lastPulsePeriodUs = gLastPulsePeriodUs;
    interrupts();
}

/**
 * @brief Computes rotation metrics from the period between valid pulses.
 */
Measurement computeMeasurement(
    const uint32_t lastPulseMicros,
    const uint32_t lastPulsePeriodUs,
    const uint32_t nowMicros,
    float& filteredRotationFrequencyHz)
{
    const bool hasValidPeriod = (lastPulsePeriodUs > 0U);
    const bool timedOut =
        (lastPulseMicros == 0U) ||
        ((nowMicros - lastPulseMicros) > config::kSignalTimeoutUs);

    float rawRotationFrequencyHz = 0.0f;

    if (!timedOut && hasValidPeriod)
    {
        const float pulseFrequencyHz =
            1.0e6f / static_cast<float>(lastPulsePeriodUs);

        rawRotationFrequencyHz =
            pulseFrequencyHz / config::kPulsesPerRevolution;
    }

    if (timedOut)
    {
        filteredRotationFrequencyHz = 0.0f;
    }
    else if (!hasValidPeriod)
    {
        // A first pulse has been seen, but a full period is not available yet.
        filteredRotationFrequencyHz = 0.0f;
    }
    else if (filteredRotationFrequencyHz == 0.0f)
    {
        filteredRotationFrequencyHz = rawRotationFrequencyHz;
    }
    else
    {
        filteredRotationFrequencyHz =
            (config::kFilterAlpha * rawRotationFrequencyHz) +
            ((1.0f - config::kFilterAlpha) * filteredRotationFrequencyHz);
    }

    const float angularVelocityRadPerSecond =
        2.0f * PI * filteredRotationFrequencyHz;

    const float centripetalAccelerationMetersPerSecondSquared =
        angularVelocityRadPerSecond *
        angularVelocityRadPerSecond *
        config::kRadiusMeters;

    const float rpm = filteredRotationFrequencyHz * 60.0f;

    return {
        filteredRotationFrequencyHz,
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
        onHallPulse,
        config::kInterruptMode);

    delay(300);
    Serial.println("US1881 Hall latch monitor started");
}

void loop()
{
    static uint32_t previousPublishMillis = 0U;
    static float filteredRotationFrequencyHz = 0.0f;

    const uint32_t nowMillis = millis();

    if ((nowMillis - previousPublishMillis) < config::kPublishIntervalMs)
    {
        return;
    }

    previousPublishMillis = nowMillis;

    uint32_t lastPulseMicros = 0U;
    uint32_t lastPulsePeriodUs = 0U;

    snapshotPulseData(lastPulseMicros, lastPulsePeriodUs);

    const uint32_t nowMicros = micros();

    const Measurement measurement = computeMeasurement(
        lastPulseMicros,
        lastPulsePeriodUs,
        nowMicros,
        filteredRotationFrequencyHz);

    printForSerialPlotter(measurement);
}
