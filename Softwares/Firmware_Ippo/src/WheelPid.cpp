#include "WheelPid.h"

#include "Config.h"

float WheelPid::update(float target, float measured, float dt) {
  float error = target - measured;
  sum = constrain(
    sum + error * dt,
    -Config::I_LIMIT,
    Config::I_LIMIT
  );
  float output = kp * error + ki * sum + kd * (error - last) / dt;
  last = error;
  return output;
}