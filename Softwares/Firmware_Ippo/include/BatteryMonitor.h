#pragma once

#include "RobotTypes.h"

class BatteryMonitor {
 public:
  void begin();
  void update(float cal);
  const BatteryState& state() const { return s; }

 private:
    float filteredVoltage = 0.0f;
    bool filterReady = false;
  BatteryState s;
  uint32_t last = 0;
};