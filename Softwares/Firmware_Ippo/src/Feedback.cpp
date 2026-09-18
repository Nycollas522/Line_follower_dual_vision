#include "Feedback.h"

#include "Config.h"

void Feedback::begin() {
  pinMode(Config::BUZZER, OUTPUT);
  digitalWrite(Config::BUZZER, LOW);

  setRgb(0, 0, 0);
}

void Feedback::set(LedState state) {
  if (_state == state) {
    return;
  }

  _state = state;

  if (_state == LedState::BOOT) {
    beep(1800, 80);
  }

  if (_state == LedState::TEST) {
    beep(2200, 100);
  }

  if (_state == LedState::STOPPED) {
    beep(900, 300);
  }
}

void Feedback::setRgb(
  uint8_t red,
  uint8_t green,
  uint8_t blue
) {
  // RGB endereçável WS2812/SK6812.
  neopixelWrite(
    Config::RGB,
    red,
    green,
    blue
  );
}

void Feedback::beep(
  uint16_t frequencyHz,
  uint16_t durationMs
) {
  tone(
    Config::BUZZER,
    frequencyHz,
    durationMs
  );
}

void Feedback::update() {
  const uint32_t now = millis();

  if (now - _lastBlinkMs >= 400) {
    _lastBlinkMs = now;
    _blink = !_blink;
  }

  switch (_state) {
    case LedState::BOOT:
      setRgb(0, 0, _blink ? 25 : 0);
      break;

    case LedState::IDLE:
      setRgb(0, 0, 20);
      break;

    case LedState::RUN:
      setRgb(0, 25, 0);
      break;

    case LedState::MENU:
      setRgb(20, 0, 20);
      break;

    case LedState::TEST:
      setRgb(
        _blink ? 28 : 0,
        _blink ? 12 : 0,
        0
      );
      break;

    case LedState::BATTERY_LOW:
      setRgb(
        _blink ? 28 : 0,
        _blink ? 10 : 0,
        0
      );

      if (now - _lastBatteryBeepMs >= 15000) {
        beep(1400, 100);
        _lastBatteryBeepMs = now;
      }
      break;

    case LedState::BATTERY_CRITICAL:
      setRgb(
        _blink ? 30 : 0,
        0,
        0
      );

      if (now - _lastBatteryBeepMs >= 3000) {
        beep(900, 180);
        _lastBatteryBeepMs = now;
      }
      break;

    case LedState::STOPPED:
      setRgb(30, 0, 0);
      break;
  }
}