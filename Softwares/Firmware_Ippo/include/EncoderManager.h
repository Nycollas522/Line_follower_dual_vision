#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>

#include "Config.h"

class EncoderManager {
 public:
  void begin();
  void reset();
  void read(int32_t v[Config::N]);

 private:
  int32_t c[Config::N] = {};
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  static constexpr pcnt_unit_t units[Config::N] = {
    PCNT_UNIT_0,
    PCNT_UNIT_1,
    PCNT_UNIT_2,
    PCNT_UNIT_3,
  };
};