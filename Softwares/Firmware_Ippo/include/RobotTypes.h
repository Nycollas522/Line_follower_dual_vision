#pragma once

#include <Arduino.h>

#include "Config.h"

struct Twist {
  float vx;
  float vy;
  float wz;
};

struct WheelState {
  int32_t count[Config::N];
  // speed: janela curta, e o que o PID realimenta.
  // speedAvg: janela longa, e o que sai na telemetria e na odometria.
  // Separados porque o PID aguenta ruido e quem le de fora nao -- ver
  // SPEED_WIN_PID / SPEED_WIN_TEL.
  float speed[Config::N];
  float speedAvg[Config::N];
  float target[Config::N];
  int pwm[Config::N];
  // Alvo nao-nulo com roda parada por tempo demais. So virou sinal
  // confiavel depois da decodificacao x4: com x1 a roda lia zero por
  // quantizacao o tempo todo.
  bool stalled[Config::N];
};

struct Odom {
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float wz;      // ja FUNDIDO com o giroscopio (ver odometry())
  float wzEnc;   // so encoders, guardado para comparacao
  float slip;    // wzEnc - giroscopio: patinagem medida, nao inferida
};

struct ImuState {
  bool ready;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  // Taxa de guinada ja no eixo CERTO e sem vies. A placa esta em pe, com
  // o chip apontando para frente, entao o eixo vertical e o X e nao o Z
  // (medido: gravidade em ax = +9.85). Quem precisa de yaw usa isto, nao
  // gz -- ver IMU_YAW_AXIS_DEFAULT.
  float yawRate;
  uint8_t yawAxis;   // 0=X 1=Y 2=Z, detectado pela gravidade
  int8_t yawSign;
  float bias;
};

struct BatteryState {
  float voltage;
  bool low;
  bool critical;
};