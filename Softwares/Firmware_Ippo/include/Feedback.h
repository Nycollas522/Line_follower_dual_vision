#pragma once

#include <Arduino.h>

enum class LedState : uint8_t {
  BOOT,
  IDLE,
  RUN,
  MENU,
  TEST,
  BATTERY_LOW,
  BATTERY_CRITICAL,
  STOPPED,
};

class Feedback {
public:
  void begin();

  void set(LedState state);

  void beep(
    uint16_t frequencyHz,
    uint16_t durationMs
  );

  void update();

private:
  LedState _state = LedState::BOOT;

  uint32_t _lastBlinkMs = 0;
  uint32_t _lastBatteryBeepMs = 0;

  bool _blink = false;

  void setRgb(
    uint8_t red,
    uint8_t green,
    uint8_t blue
  );
};