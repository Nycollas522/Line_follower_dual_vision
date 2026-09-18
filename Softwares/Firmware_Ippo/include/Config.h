#pragma once

#include <Arduino.h>

namespace Config {

constexpr char NAME[] = "ESP32S3_OMNI_LINE";
constexpr uint8_t N = 4;

enum Wheel : uint8_t {
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
};

constexpr uint8_t IN1[N] = {5, 39, 8, 47};   //5, 39, 8, 47
constexpr uint8_t IN2[N] = {4, 40, 9, 16};   //4, 40, 9, 16
constexpr uint8_t ENCA[N] = {6, 42, 11, 21};  //6, 42, 11, 21
constexpr uint8_t ENCB[N] = {7, 41, 10, 18};   //7, 41, 10, 18
constexpr int8_t MOTOR_SIGN[N] = {1, 1, 1, 1};
constexpr int8_t ENC_SIGN[N] = {1, 1, 1, 1};

constexpr uint8_t BTN1 = 12;
constexpr uint8_t BTN2 = 14;
constexpr uint8_t SERVO = 13;
constexpr uint8_t BATT = 15;
constexpr uint8_t BUZZER = 45;
constexpr uint8_t RGB = 48;

constexpr uint8_t SDA = 1;
constexpr uint8_t SCL = 2;
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t OLED_ADDR = 0x3C;

constexpr uint32_t I2C_HZ = 400000;
constexpr uint32_t CONTROL_US = 10000;

// Versao do esquema de Settings gravado em NVS. INCREMENTE sempre que o
// SIGNIFICADO de um campo mudar -- sem isso o valor antigo continua em
// NVS e o default novo nunca pega. A v2 marca a troca dos encoders para
// decodificacao x4, que multiplica ticksRev por 4.
constexpr uint32_t SETTINGS_VERSION = 2;

// Baud da UART para o CH343. MEDIDO em 10/09/2026: a telemetria antiga
// (209 B a 5 Hz) ocupava 9.1% de 115200. Com ODOM/WHEEL a 50 Hz o uso
// iria a 55%, apertado; a 921600 fica em 7%.
constexpr uint32_t SERIAL_BAUD = 921600;
// Telemetria em DOIS grupos. O controle roda a 100 Hz e o controlador do
// Pi a 50 Hz, mas tudo chegava a 5 Hz -- odometria com ate 200 ms de
// idade. Agora o que o controle usa vai rapido e o resto continua lento.
constexpr uint32_t TELEMETRY_MS = 200;       // ENC, TARGET, PWM, IMU, SERVO, BATT
constexpr uint32_t TELEMETRY_FAST_MS = 20;   // ODOM, WHEEL  (50 Hz)
constexpr uint32_t OLED_MS = 500;
constexpr uint32_t CMD_TIMEOUT_MS = 200;

constexpr uint32_t MOTOR_PWM_HZ = 20000;
constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr int PWM_MAX = 255;
constexpr int PWM_LIMIT_DEFAULT = 180;
constexpr int STATIC_PWM_DEFAULT = 100;

constexpr float WHEEL_D_M = 0.078f;
constexpr float WHEEL_C = PI * WHEEL_D_M;
// x4: o PCNT conta as duas bordas dos DOIS canais (ver EncoderManager).
//
// POR QUE MUDOU (18/09/2026): com x1 e janela de 10 ms a velocidade de
// roda so podia valer 0, 0.074, 0.149... e o robo trabalha entre 0.07 e
// 0.20 m/s. A roda direita, a 0.106 m/s, da 1.43 ticks por janela --
// abaixo de 1 ela le EXATAMENTE ZERO girando perfeitamente. Foi isso, e
// nao falha mecanica, que produziu o "roda parada" nos diagnosticos.
//
// x4 leva a quantizacao para 0.0186 m/s. Combinado com a janela mais
// longa do estimador (ver SPEED_WIN_*), a telemetria chega a ~0.004.
constexpr float TICKS_REV_DEFAULT = 1320.0f;

// Janelas do estimador de velocidade, em periodos de controle (10 ms).
// Curta para o PID (realimentacao rapida), longa para telemetria e
// odometria (leitura limpa). Sao independentes de proposito: o PID
// aguenta ruido, quem le do lado de fora nao.
constexpr uint8_t SPEED_WIN_PID = 3;    //  30 ms
constexpr uint8_t SPEED_WIN_TEL = 10;   // 100 ms
constexpr uint8_t SPEED_HIST = 16;      // tamanho do buffer circular

// Deteccao de roda travada: alvo acima de STALL_TARGET com velocidade
// medida abaixo de STALL_SPEED por mais de STALL_MS. Os limiares so
// fazem sentido com x4 + janela longa: a quantizacao da leitura e
// ~0.002 m/s, entao 0.01 e sinal de verdade e nao arredondamento.
constexpr float STALL_TARGET = 0.030f;
constexpr float STALL_SPEED = 0.010f;
constexpr uint32_t STALL_MS = 300;
// Medido: 17.6cm entre eixo dianteiro e traseiro, 15.4cm entre rodas
// esquerda/direita.
constexpr float HALF_L = 0.088f;
constexpr float HALF_W = 0.077f;
constexpr float K = HALF_L + HALF_W;

constexpr float MAX_WHEEL_MPS = 0.70f;
constexpr float MAX_WZ_ACCEL = 6.0f;
constexpr float KP_DEFAULT = 120.0f;
constexpr float KI_DEFAULT = 100.0f;
constexpr float KD_DEFAULT = 5.0f;
constexpr float I_LIMIT = 1.2f;

constexpr float YAW_ENCODER_WEIGHT = 0.15f;

// EIXO DO YAW NA IMU. A placa do MPU6050 NAO esta deitada: esta em pe,
// com o chip apontando para a frente do robo. MEDIDO em 18/09/2026 com
// o robo parado -- a gravidade aparece em ax = +9.85, nao em az:
//     ax +9.846   ay +0.273   az +2.094
// Ou seja o eixo VERTICAL e o X do chip, e o yaw se mede em gx.
//
// O firmware lia gz, que nesse arranjo e um eixo horizontal e fica em
// ~0.001 rad/s parado. Resultado: dg era sempre ~0 e o yaw integrado
// andava a 15% da taxa real (so o termo de encoder sobrevivia). E o
// calibrate() removia vies de gz enquanto gx carregava -0.132 rad/s
// (-7.6 graus/s) sem correcao nenhuma.
//
// Nao afetou o controle: o ROS le twist.angular.z, que vem so dos
// encoders. Afetou od.yaw, od.x e od.y, que ninguem consumia.
//
// 0 = X, 1 = Y, 2 = Z. Detectado automaticamente pela gravidade no
// boot; estes sao so o ponto de partida.
constexpr uint8_t IMU_YAW_AXIS_DEFAULT = 0;
constexpr int8_t IMU_YAW_SIGN_DEFAULT = 1;
constexpr float ACCEL_ALPHA = 0.20f;

constexpr float R1 = 100000.0f;
constexpr float R2 = 33000.0f;
constexpr float DIV_RATIO = (R1 + R2) / R2;
constexpr float LOW_BATT = 11.3f;
constexpr float CRIT_BATT = 11.0f;
}