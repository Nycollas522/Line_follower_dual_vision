#include "ImuManager.h"

#include "Config.h"

bool ImuManager::begin(TwoWire& w) {
  s.ready = m.begin(Config::MPU_ADDR, &w);
  s.yawAxis = Config::IMU_YAW_AXIS_DEFAULT;
  s.yawSign = Config::IMU_YAW_SIGN_DEFAULT;
  if (s.ready) {
    m.setAccelerometerRange(MPU6050_RANGE_4_G);
    m.setGyroRange(MPU6050_RANGE_500_DEG);
    m.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  return s.ready;
}

void ImuManager::setYawAxis(uint8_t axis, int8_t sign) {
  s.yawAxis = axis > 2 ? 2 : axis;
  s.yawSign = sign < 0 ? -1 : 1;
}

// Giro bruto no eixo escolhido, ainda com vies.
float ImuManager::rawYaw() const {
  const float v = s.yawAxis == 0 ? s.gx : (s.yawAxis == 1 ? s.gy : s.gz);
  return s.yawSign * v;
}

void ImuManager::update() {
  if (!s.ready) {
    return;
  }

  sensors_event_t a, g, t;
  m.getEvent(&a, &g, &t);
  s.ax += Config::ACCEL_ALPHA * (a.acceleration.x - s.ax);
  s.ay += Config::ACCEL_ALPHA * (a.acceleration.y - s.ay);
  s.az += Config::ACCEL_ALPHA * (a.acceleration.z - s.az);
  s.gx = g.gyro.x;
  s.gy = g.gyro.y;
  s.gz = g.gyro.z;
  // O vies e removido do eixo do YAW, nao de gz. Antes era de gz, que
  // neste robo e um eixo horizontal: o eixo que de fato mede guinada
  // ficava com -0.132 rad/s (-7.6 graus/s) de vies sem correcao.
  s.yawRate = rawYaw() - bias;
  s.bias = bias;
}

// A gravidade so aparece no eixo VERTICAL. Com o robo parado e nivelado,
// o eixo de maior aceleracao absoluta e o que aponta para cima -- e e
// nele que se mede guinada.
//
// MEDIDO em 18/09/2026 neste robo: ax +9.85, ay +0.27, az +2.09. Logo o
// vertical e o X do chip, porque a placa esta em pe com o chip apontando
// para a frente, e nao deitada como e convencional.
//
// O SINAL nao sai daqui: gravidade nao diz se girar para a esquerda da
// leitura positiva ou negativa. Ele vem do cruzamento com os encoders
// (ver a calibracao de yaw no menu) ou do default.
void ImuManager::detectYawAxis() {
  if (!s.ready) {
    return;
  }

  float acc[3] = {0, 0, 0};
  sensors_event_t a, g, t;
  for (int i = 0; i < 200; i++) {
    m.getEvent(&a, &g, &t);
    acc[0] += a.acceleration.x;
    acc[1] += a.acceleration.y;
    acc[2] += a.acceleration.z;
    delay(3);
  }

  uint8_t melhor = 0;
  for (uint8_t i = 1; i < 3; i++) {
    if (fabsf(acc[i]) > fabsf(acc[melhor])) {
      melhor = i;
    }
  }
  s.yawAxis = melhor;
}

void ImuManager::calibrate() {
  if (!s.ready) {
    return;
  }

  bias = 0;
  float acc = 0;
  sensors_event_t a, g, t;
  for (int i = 0; i < 500; i++) {
    m.getEvent(&a, &g, &t);
    s.gx = g.gyro.x;
    s.gy = g.gyro.y;
    s.gz = g.gyro.z;
    acc += rawYaw();
    delay(3);
  }
  bias = acc / 500.0f;
  s.bias = bias;
}
