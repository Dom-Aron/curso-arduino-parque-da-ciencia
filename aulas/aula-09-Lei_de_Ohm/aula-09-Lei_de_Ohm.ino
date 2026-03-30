/*
  Ohm's Law V-I acquisition for Arduino UNO.

  Purpose:
  - Measure the voltage across a test component.
  - Infer the current through the component from the shunt resistor voltage.
  - Detect only the interval in which the potentiometer is actually moving.
  - Send structured serial data so a Python script on the computer can:
      1) save the useful interval to a .dat file;
      2) read the exported file;
      3) generate a scientific plot.

  Suggested circuit:
  - Arduino 5V  -> potentiometer terminal 1
  - Arduino GND -> potentiometer terminal 3
  - Potentiometer wiper -> input of the test component
  - Output of the test component -> shunt resistor (R_shunt)
  - Other side of the shunt resistor -> GND

  Measurement points:
  - A0: node before the test component  -> V_total
  - A1: node between component and shunt -> V_shunt

  Physical relations:
  - V_component = V_total - V_shunt
  - I_component = V_shunt / R_shunt

  Serial protocol sent to the PC:
  - Comment / feedback lines start with '#'
  - Sweep start marker: START\t<timestamp_ms>\t<sweep_id>
  - Data line:           DATA\t<timestamp_ms>\t<sweep_id>\t<V_comp_V>\t<I_comp_mA>\t<V_total_V>\t<V_shunt_V>
  - Sweep end marker:   END\t<timestamp_ms>\t<sweep_id>\t<sample_count>

  Notes:
  - The Arduino cannot directly create a .dat file on the computer.
    A Python host script must listen to the serial port and write the file.
*/

#include <math.h>

namespace {

constexpr float kAdcReferenceVolts = 5.0f;
constexpr float kShuntResistanceOhms = 220.0f;
constexpr unsigned long kSampleIntervalMs = 25UL;
constexpr unsigned long kMovementSettleTimeMs = 350UL;
constexpr unsigned long kIdleMessageIntervalMs = 2000UL;
constexpr uint8_t kSamplesPerChannel = 8U;
constexpr float kMovementThresholdVolts = 0.015f;  // 15 mV
constexpr float kNegativeClampVolts = -0.010f;     // tolerate small negative noise only

struct Measurement {
  float vTotal;
  float vShunt;
  float vComponent;
  float iMilliamp;
};

bool gIsSweepActive = false;
bool gHasReference = false;
unsigned long gLastSampleMs = 0UL;
unsigned long gLastChangeMs = 0UL;
unsigned long gLastIdleMessageMs = 0UL;
unsigned long gSweepStartMs = 0UL;
uint32_t gSweepId = 0UL;
uint32_t gSweepSampleCount = 0UL;
float gMovementReferenceVolts = 0.0f;

float convertRawToVolts(const int rawValue) {
  return (static_cast<float>(rawValue) * kAdcReferenceVolts) / 1023.0f;
}

int readAnalogAverage(const uint8_t pin) {
  long sum = 0L;

  for (uint8_t index = 0U; index < kSamplesPerChannel; ++index) {
    sum += analogRead(pin);
  }

  return static_cast<int>(sum / static_cast<long>(kSamplesPerChannel));
}

Measurement readMeasurement() {
  const int rawA0 = readAnalogAverage(A0);
  const int rawA1 = readAnalogAverage(A1);

  const float vTotal = convertRawToVolts(rawA0);
  const float vShunt = convertRawToVolts(rawA1);

  float vComponent = vTotal - vShunt;

  if (vComponent < 0.0f && vComponent > kNegativeClampVolts) {
    vComponent = 0.0f;
  }

  if (vComponent < kNegativeClampVolts) {
    // A clearly negative value is not physically expected in this arrangement.
    // Clamp it to zero to avoid propagating an obviously invalid point.
    vComponent = 0.0f;
  }

  float iMilliamp = (vShunt / kShuntResistanceOhms) * 1000.0f;

  if (iMilliamp < 0.0f) {
    iMilliamp = 0.0f;
  }

  return Measurement{vTotal, vShunt, vComponent, iMilliamp};
}

void printIdleMessage(const unsigned long nowMs, const Measurement& measurement) {
  if ((nowMs - gLastIdleMessageMs) < kIdleMessageIntervalMs) {
    return;
  }

  Serial.print(F("# Waiting for potentiometer movement | V_total="));
  Serial.print(measurement.vTotal, 4);
  Serial.print(F(" V | V_component="));
  Serial.print(measurement.vComponent, 4);
  Serial.print(F(" V | I_component="));
  Serial.print(measurement.iMilliamp, 4);
  Serial.println(F(" mA"));

  gLastIdleMessageMs = nowMs;
}

void startSweep(const unsigned long nowMs) {
  gIsSweepActive = true;
  gSweepStartMs = nowMs;
  gLastChangeMs = nowMs;
  gSweepSampleCount = 0UL;
  ++gSweepId;

  Serial.print(F("# Sweep started: id="));
  Serial.print(gSweepId);
  Serial.print(F(" at "));
  Serial.print(nowMs);
  Serial.println(F(" ms"));

  Serial.print(F("START\t"));
  Serial.print(nowMs);
  Serial.print(F("\t"));
  Serial.println(gSweepId);
}

void finishSweep(const unsigned long nowMs) {
  Serial.print(F("END\t"));
  Serial.print(nowMs);
  Serial.print(F("\t"));
  Serial.print(gSweepId);
  Serial.print(F("\t"));
  Serial.println(gSweepSampleCount);

  Serial.print(F("# Sweep finished: id="));
  Serial.print(gSweepId);
  Serial.print(F(" | duration="));
  Serial.print(nowMs - gSweepStartMs);
  Serial.print(F(" ms | samples="));
  Serial.println(gSweepSampleCount);

  gIsSweepActive = false;
}

void emitData(const unsigned long nowMs, const Measurement& measurement) {
  Serial.print(F("DATA\t"));
  Serial.print(nowMs);
  Serial.print(F("\t"));
  Serial.print(gSweepId);
  Serial.print(F("\t"));
  Serial.print(measurement.vComponent, 6);
  Serial.print(F("\t"));
  Serial.print(measurement.iMilliamp, 6);
  Serial.print(F("\t"));
  Serial.print(measurement.vTotal, 6);
  Serial.print(F("\t"));
  Serial.println(measurement.vShunt, 6);

  ++gSweepSampleCount;
}

}  // namespace

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    ;
  }

  analogReference(DEFAULT);

  Serial.println(F("# Ohm's law acquisition initialized."));
  Serial.println(F("# Move the potentiometer to start a sweep."));
  Serial.println(F("# Columns: DATA<TAB>timestamp_ms<TAB>sweep_id<TAB>V_component_V<TAB>I_component_mA<TAB>V_total_V<TAB>V_shunt_V"));
}

void loop() {
  const unsigned long nowMs = millis();

  if ((nowMs - gLastSampleMs) < kSampleIntervalMs) {
    return;
  }

  gLastSampleMs = nowMs;

  const Measurement measurement = readMeasurement();

  if (!gHasReference) {
    gMovementReferenceVolts = measurement.vTotal;
    gHasReference = true;
  }

  const float movementDelta = fabsf(measurement.vTotal - gMovementReferenceVolts);
  const bool significantChange = movementDelta >= kMovementThresholdVolts;

  if (significantChange) {
    gMovementReferenceVolts = measurement.vTotal;
    gLastChangeMs = nowMs;

    if (!gIsSweepActive) {
      startSweep(nowMs);
    }
  }

  if (gIsSweepActive) {
    emitData(nowMs, measurement);

    if ((nowMs - gLastChangeMs) >= kMovementSettleTimeMs) {
      finishSweep(nowMs);
    }
  } else {
    printIdleMessage(nowMs, measurement);
  }
}
