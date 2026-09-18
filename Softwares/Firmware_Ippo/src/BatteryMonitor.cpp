#include "BatteryMonitor.h"

#include "Config.h"

namespace {
constexpr uint8_t SAMPLE_COUNT = 32;
constexpr float FILTER_ALPHA = 0.15f;
}

void BatteryMonitor::begin() {
  analogReadResolution(12);
  analogSetPinAttenuation(Config::BATT, ADC_11db);
}

void BatteryMonitor::update(float cal) {
  if (millis() - last < 500) {
    return;
  }

  last = millis();
  uint32_t millivoltsSum = 0;
  for (uint8_t sample = 0; sample < SAMPLE_COUNT; ++sample) {
    millivoltsSum += analogReadMilliVolts(Config::BATT);
  }

  const float rawVoltage =
    (millivoltsSum / static_cast<float>(SAMPLE_COUNT) / 1000.0f) *
    Config::DIV_RATIO * cal;

  if (!filterReady) {
    filteredVoltage = rawVoltage;
    filterReady = true;
  } else {
    filteredVoltage += FILTER_ALPHA * (rawVoltage - filteredVoltage);
  }

  s.voltage = filteredVoltage;
  s.low = s.voltage < Config::LOW_BATT;
  s.critical = s.voltage < Config::CRIT_BATT;
}