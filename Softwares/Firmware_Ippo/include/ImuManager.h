#pragma once

#include <Wire.h>
#include <Adafruit_MPU6050.h>

#include "RobotTypes.h"

class ImuManager {
 public:
  bool begin(TwoWire& w);
  void update();
  void calibrate();
  // Descobre qual eixo do chip aponta para cima usando a GRAVIDADE, com
  // o robo parado e nivelado. E o unico jeito de acertar isso sem
  // depender de como a placa foi parafusada.
  void detectYawAxis();
  void setYawAxis(uint8_t axis, int8_t sign);
  const ImuState& state() const { return s; }

 private:
  float rawYaw() const;

  Adafruit_MPU6050 m;
  ImuState s;
  float bias = 0;
};