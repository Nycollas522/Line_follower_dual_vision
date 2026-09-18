#pragma once

#include <Arduino.h>

class WheelPid {
 public:
  void gains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }

  void reset() {
    sum = 0;
    last = 0;
  }

  float update(float target, float measured, float dt);

 private:
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float sum = 0.0f;
  float last = 0.0f;
};