#include "MotorDriver.h"

void MotorDriver::begin() {
  for (uint8_t i = 0; i < Config::N; ++i) {
    pinMode(Config::IN1[i], OUTPUT);
    pinMode(Config::IN2[i], OUTPUT);

    digitalWrite(Config::IN1[i], LOW);
    digitalWrite(Config::IN2[i], LOW);

    ledcSetup(
      _channel[i],
      Config::MOTOR_PWM_HZ,
      Config::MOTOR_PWM_BITS
    );

    _output[i] = 0;
    _activePwmPin[i] = -1;
  }
}

void MotorDriver::setMotorPinsLow(uint8_t index) {
  digitalWrite(Config::IN1[index], LOW);
  digitalWrite(Config::IN2[index], LOW);
}

void MotorDriver::detachMotorPwm(uint8_t index) {
  if (_activePwmPin[index] < 0) {
    return;
  }

  const uint8_t activePin =
    static_cast<uint8_t>(_activePwmPin[index]);

  ledcWrite(_channel[index], 0);
  ledcDetachPin(activePin);

  pinMode(activePin, OUTPUT);
  digitalWrite(activePin, LOW);

  _activePwmPin[index] = -1;
}

void MotorDriver::set(uint8_t index, int pwm) {
  if (index >= Config::N) {
    return;
  }

  pwm *= Config::MOTOR_SIGN[index];

  pwm = constrain(
    pwm,
    -Config::PWM_MAX,
    Config::PWM_MAX
  );

  if (pwm == 0) {
    detachMotorPwm(index);
    setMotorPinsLow(index);

    _output[index] = 0;
    return;
  }

  const uint8_t wantedPwmPin =
    (pwm > 0)
      ? Config::IN1[index]
      : Config::IN2[index];

  const uint8_t oppositePin =
    (pwm > 0)
      ? Config::IN2[index]
      : Config::IN1[index];

  // Se houve inversão de sentido, o PWM deve sair de um
  // pino e ser ligado ao outro.
  if (
    _activePwmPin[index] !=
    static_cast<int8_t>(wantedPwmPin)
  ) {
    detachMotorPwm(index);

    pinMode(oppositePin, OUTPUT);
    digitalWrite(oppositePin, LOW);

    ledcAttachPin(
      wantedPwmPin,
      _channel[index]
    );

    _activePwmPin[index] =
      static_cast<int8_t>(wantedPwmPin);
  }

  digitalWrite(oppositePin, LOW);

  ledcWrite(
    _channel[index],
    abs(pwm)
  );

  _output[index] = pwm;
}

void MotorDriver::stop() {
  for (uint8_t i = 0; i < Config::N; ++i) {
    set(i, 0);
  }
}

int MotorDriver::last(uint8_t index) const {
  if (index >= Config::N) {
    return 0;
  }

  return _output[index];
}