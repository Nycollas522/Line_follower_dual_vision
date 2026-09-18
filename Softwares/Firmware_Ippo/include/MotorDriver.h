#pragma once

#include <Arduino.h>
#include "Config.h"

class MotorDriver {
public:
  void begin();

  void set(uint8_t index, int pwm);
  void stop();

  int last(uint8_t index) const;

private:
  // Cada motor possui um canal PWM reservado.
  // Canais 0 a 3 ficam livres para servo/buzzer.
  const uint8_t _channel[Config::N] = {
    4,  // FL
    5,  // FR
    6,  // RL
    7   // RR
  };

  int _output[Config::N] = {
    0, 0, 0, 0
  };

  // Pino que está recebendo PWM naquele motor.
  // -1 significa motor parado/desanexado.
  int8_t _activePwmPin[Config::N] = {
    -1, -1, -1, -1
  };

  void detachMotorPwm(uint8_t index);
  void setMotorPinsLow(uint8_t index);
};