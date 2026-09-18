#pragma once

#include <Arduino.h>

struct SettingsData {
  float kp;
  float ki;
  float kd;
  float ticksRev;
  float battCal;
  float maxWheelMps;
  float yawEncoderWeight;
  int staticPwm;
  int pwmLimit;
  float maxWzAccel;
  float halfL;
  float halfW;
  float wheelDiameter;
  // Eixo do chip que aponta para cima (0=X 1=Y 2=Z) e o sinal da
  // guinada. Persistidos porque dependem de COMO a placa foi montada --
  // remontou, recalibra pelo menu, nao recompila.
  uint8_t imuYawAxis;
  int8_t imuYawSign;
};

class Settings {
 public:
  void begin();
  void load();
  bool save();
  void defaults();

  const SettingsData& get() const { return d; }
  SettingsData& edit() {
    dirty = true;
    return d;
  }
  bool changed() const { return dirty; }
  void clearDirty() { dirty = false; }

 private:
  SettingsData d = {};
  bool dirty = false;
};