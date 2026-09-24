#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "Config.h"
#include "RobotTypes.h"
#include "Settings.h"
#include "MotorDriver.h"
#include "EncoderManager.h"
#include "WheelPid.h"
#include "ImuManager.h"
#include "BatteryMonitor.h"
#include "Feedback.h"
#include "LocalMenu.h"
#include "SerialProtocol.h"

MotorDriver motor;
EncoderManager enc;
ImuManager imu;
BatteryMonitor batt;
Feedback fb;
LocalMenu menu;
SerialProtocol serial;
Settings cfg;
Servo servo;
WheelPid pid[Config::N];

// Versao do firmware e do protocolo serial. Subir PROTOCOL_VERSION
// sempre que o formato das linhas mudar de forma incompativel.
constexpr char FIRMWARE_VERSION[] = "2.0.0";
constexpr uint8_t PROTOCOL_VERSION = 2;
WheelState ws;
Odom od;
Twist cmd;
float appliedYaw = 0.0f;
uint32_t lastControl = 0;
uint32_t lastCmd = 0;
uint32_t lastTel = 0;
uint32_t lastTelFast = 0;
uint32_t stallSince[Config::N] = {};
uint32_t lastMoving = 0;

// INSTRUMENTACAO DO LACO DE CONTROLE.
//
// O controle quer rodar a cada 10 ms, mas roda "quando o loop() chegar
// la". Sem medir isso nao da para saber se vale mover o controle para
// uma task propria -- e complexidade de concorrencia so se paga com
// numero na mao. Publicado em DIAG e zerado a cada relatorio.
struct CtlStats {
  uint32_t periodoMin = 0xFFFFFFFF;
  uint32_t periodoMax = 0;
  uint64_t periodoSoma = 0;
  uint32_t execMax = 0;
  uint32_t amostras = 0;
  uint32_t atrasos = 0;      // periodos acima de 15 ms

  void registra(uint32_t periodoUs, uint32_t execUs) {
    if (periodoUs < periodoMin) periodoMin = periodoUs;
    if (periodoUs > periodoMax) periodoMax = periodoUs;
    if (execUs > execMax) execMax = execUs;
    periodoSoma += periodoUs;
    amostras++;
    if (periodoUs > 15000) atrasos++;
  }

  void zera() {
    periodoMin = 0xFFFFFFFF;
    periodoMax = 0;
    periodoSoma = 0;
    execMax = 0;
    amostras = 0;
    atrasos = 0;
  }
};
CtlStats ctl;
float servoAngle = 0;

// Aponta o servo: angulo do PROTOCOLO (0 = centro calibrado) + trim.
// writeMicroseconds em vez de write(int): write arredondava para grau
// inteiro, o que somava degraus de 1 grau a folga mecanica.
void servoAponta(float angulo) {
  const float fisico = constrain(angulo + 90.0f + cfg.get().servoTrim,
                                 0.0f, 180.0f);
  const float us = Config::SERVO_US_MIN
                   + fisico * (Config::SERVO_US_MAX - Config::SERVO_US_MIN)
                     / 180.0f;
  servo.writeMicroseconds(lroundf(us));
}
bool test = false;

float ticksPerMeter() {
  return cfg.get().ticksRev / (PI * cfg.get().wheelDiameter);
}

void apply() {
  for (uint8_t i = 0; i < Config::N; i++) {
    pid[i].gains(cfg.get().kp, cfg.get().ki, cfg.get().kd);
  }
}

// Motivo da ultima parada. Sai em DIAG: "o robo parou" sem dizer por que
// ja custou tempo de depuracao -- watchdog, bateria e botao produzem o
// mesmo sintoma visual e exigem acoes opostas.
enum StopReason : uint8_t {
  STOP_NONE = 0,
  STOP_ROS = 1,        // comando STOP pela serial
  STOP_WATCHDOG = 2,   // sem TWIST por CMD_TIMEOUT_MS
  STOP_MENU = 3,       // botao fisico (pressao longa)
  STOP_BATTERY = 4,    // tensao critica
};
uint8_t stopReason = STOP_NONE;

void stop() {
  cmd = {};
  appliedYaw = 0.0f;
  lastCmd = 0;
  test = false;
  motor.stop();
  for (auto& p : pid) {
    p.reset();
  }
  fb.set(LedState::STOPPED);
}

// Historico de contagens para o estimador de velocidade.
//
// POR QUE EXISTE (18/09/2026): a velocidade era (delta ticks)/dt numa
// janela unica de 10 ms. Mesmo com x4 isso da degraus de 0.0186 m/s, e
// o robo anda a 0.07-0.20. Medindo sobre uma janela mais longa o degrau
// cai proporcionalmente: 100 ms levam a ~0.004 m/s.
//
// Duas janelas de proposito. O PID realimenta a curta, porque atraso de
// fase em malha fechada custa mais que ruido. Telemetria e odometria
// usam a longa, porque quem le de fora nao tem como filtrar depois --
// foi exatamente a leitura instantanea quantizada que produziu o falso
// "roda parada" nos diagnosticos de pista.
struct SpeedHist {
  int32_t count[Config::SPEED_HIST][Config::N];
  float t[Config::SPEED_HIST];
  float now = 0.0f;
  uint8_t head = 0;
  uint8_t fill = 0;

  void push(const int32_t c[Config::N], float dt) {
    now += dt;
    head = (head + 1) % Config::SPEED_HIST;
    for (uint8_t i = 0; i < Config::N; i++) {
      count[head][i] = c[i];
    }
    t[head] = now;
    if (fill < Config::SPEED_HIST) {
      fill++;
    }
  }

  // Velocidade da roda sobre os ultimos `win` periodos de controle.
  // Enquanto o buffer nao encheu, usa o que tem -- assim o robo nao
  // passa os primeiros 100 ms sem leitura nenhuma.
  float speed(uint8_t wheel, uint8_t win, float tpm) const {
    uint8_t usar = win;
    if (usar > fill - 1) {
      usar = fill > 1 ? fill - 1 : 0;
    }
    if (usar == 0) {
      return 0.0f;
    }
    const uint8_t old = (head + Config::SPEED_HIST - usar) %
        Config::SPEED_HIST;
    const float dt = t[head] - t[old];
    if (dt <= 1e-6f) {
      return 0.0f;
    }
    return ((count[head][wheel] - count[old][wheel]) / tpm) / dt;
  }
};

void odometry(float dt) {
  static int32_t prev[4] = {};
  static SpeedHist hist;
  const float tpm = ticksPerMeter();
  float d[4];
  for (uint8_t i = 0; i < 4; i++) {
    d[i] = (ws.count[i] - prev[i]) / tpm;
    prev[i] = ws.count[i];
  }
  hist.push(ws.count, dt);
  for (uint8_t i = 0; i < 4; i++) {
    ws.speed[i] = hist.speed(i, Config::SPEED_WIN_PID, tpm);
    ws.speedAvg[i] = hist.speed(i, Config::SPEED_WIN_TEL, tpm);
  }
  // Inversa do modelo direto de control() (mecanum X convencional):
  //   w0 = vx - vy - k*wz     w1 = vx + vy + k*wz
  //   w2 = vx + vy - k*wz     w3 = vx - vy + k*wz
  // Como as colunas sao ortogonais, a inversa e a propria transposta
  // normalizada:
  //   vx -> (+,+,+,+)/4     vy -> (-,+,+,-)/4     wz -> (-,+,-,+)/(4k)
  // Os padroes de vy e wz sao diferentes -- e o que permite separar
  // andar de lado de girar. Na versao anterior os dois eram iguais e
  // essa separacao era impossivel.
  float dx = (d[0] + d[1] + d[2] + d[3]) * 0.25f;
  float dy = (-d[0] + d[1] + d[2] - d[3]) * 0.25f;
  const float k = cfg.get().halfL + cfg.get().halfW;
  float de = (-d[0] + d[1] - d[2] + d[3]) / (4 * k);
  // yawRate, nao gz: a placa da IMU esta em pe e o eixo vertical e o X
  // do chip (gravidade medida em ax = +9.85). Lendo gz o termo girava em
  // ~0 e o yaw integrado andava a 15% da taxa real.
  float dg = imu.state().yawRate * dt;
  float da = imu.state().ready
    ? (1 - cfg.get().yawEncoderWeight) * dg +
      cfg.get().yawEncoderWeight * de
    : de;
  float ym = od.yaw + da * 0.5f;
  od.x += cosf(ym) * dx - sinf(ym) * dy;
  od.y += sinf(ym) * dx + cosf(ym) * dy;
  od.yaw = atan2f(sinf(od.yaw + da), cosf(od.yaw + da));
  // speedAvg (janela longa), nao speed: estes saem na telemetria e o Pi
  // nao tem como desfazer a quantizacao depois.
  od.vx = (ws.speedAvg[0] + ws.speedAvg[1] +
      ws.speedAvg[2] + ws.speedAvg[3]) * 0.25f;
  od.vy = (-ws.speedAvg[0] + ws.speedAvg[1] +
      ws.speedAvg[2] - ws.speedAvg[3]) * 0.25f;

  // GUINADA: giroscopio como fonte primaria, encoder como apoio.
  //
  // JUSTIFICATIVA CORRIGIDA EM 22/09/2026. A anterior dizia que "num
  // mecanum os roletes patinam por projeto" e que por isso o encoder
  // superestimava rotacao. ISSO ESTAVA ERRADO: o robo tinha uma roda
  // montada FORA DO X, e era ela que fazia o robo deslizar ao girar no
  // proprio eixo. Corrigida a montagem, as duas fontes concordam.
  //
  // MEDIDO com o robo no chao, girando por comando em regime:
  //     wz 0.40 -> ganho giro/encoder 1.04
  //     wz 0.80 -> ganho 0.96
  //     wz 1.20 -> ganho 0.94
  // Escala global 0.95: ha patinagem, mas PROGRESSIVA com a velocidade
  // e modesta (6% a 1.2 rad/s), nao um efeito inerente e grande.
  //
  // O peso continua pendendo para o giroscopio, mas agora por RUIDO e
  // nao por vies: o giroscopio tem 0.032 graus/s de ruido e vies que
  // nao deriva (medido: -0.13169 -> -0.13170 em 60 s), enquanto a wz
  // dos encoders herda a quantizacao das quatro rodas. O encoder entra
  // como ancora sem vies, que e o que o giroscopio nao tem.
  //
  // MEDIDO antes de ligar isto: ruido 0.032 graus/s, vies -0.13169
  // rad/s que nao deriva (-0.13170 apos 60 s) e some na calibracao
  // (+0.008 graus/s corrigido). O SINAL foi conferido girando o robo a
  // mao para a direita: 97 de 97 amostras negativas, que e o que a
  // REP-103 manda (z para cima, positivo anti-horario).
  //
  // Isto entra direto no controle: o line_follower_node le
  // twist.twist.angular.z, que e este campo.
  od.wzEnc = (-ws.speedAvg[0] + ws.speedAvg[1] -
      ws.speedAvg[2] + ws.speedAvg[3]) / (4 * k);
  if (imu.state().ready) {
    const float w = cfg.get().yawEncoderWeight;
    od.wz = (1.0f - w) * imu.state().yawRate + w * od.wzEnc;
    // Patinagem MEDIDA, nao inferida. Responde a pergunta que ficou em
    // aberto nas corridas: nos pivos de quebra o comando era wz=-0.90 e
    // o odom lia -0.22 a -0.67, e nao dava para separar "patinou" de
    // "motor nao entregou". A diferenca entre as duas fontes separa.
    od.slip = od.wzEnc - imu.state().yawRate;
  } else {
    od.wz = od.wzEnc;
    od.slip = 0.0f;
  }
}

void control(float dt) {
  enc.read(ws.count);
  imu.update();
  odometry(dt);

  if (millis() - lastCmd > Config::CMD_TIMEOUT_MS) {
    if (cmd.vx != 0.0f || cmd.vy != 0.0f || cmd.wz != 0.0f) {
      stopReason = STOP_WATCHDOG;
    }
    cmd = {};
  }

  const float maxYawStep = cfg.get().maxWzAccel * dt;
  appliedYaw = constrain(
    cmd.wz,
    appliedYaw - maxYawStep,
    appliedYaw + maxYawStep
  );

  float k = cfg.get().halfL + cfg.get().halfW;
  const float lateral = cmd.vy;
  const float yaw = appliedYaw;

  // Mecanum X convencional: roletes apontando para o centro, com
  // FL/RR numa orientacao e FR/RL na outra.
  //
  // Para roda em (a, b) e rolete s = +-1, a cinematica e
  //     u = vx + s*vy + wz*(s*a - b)
  // Com o X correto, s*a - b vale +-(halfL + halfW) nas quatro rodas.
  //
  // HISTORICO: ate a troca das rodas traseiras, os roletes de tras
  // estavam espelhados. Ali o coeficiente de giro real das traseiras
  // caia para (halfL - halfW) = 0.011, mas esta matriz as comandava com
  // (halfL + halfW) = 0.165 -- 15x acima do que elas podiam entregar.
  // O resultado era arrasto lateral das traseiras em toda curva, o que
  // tambem explica o yawEncoderWeight baixo (0.15) herdado: a odometria
  // por encoder nao tinha como concordar com a IMU nesse regime.
  // Com as rodas corrigidas, os quatro coeficientes voltam a ser iguais.
  ws.target[0] = cmd.vx - lateral - k * yaw;  // FL
  ws.target[1] = cmd.vx + lateral + k * yaw;  // FR
  ws.target[2] = cmd.vx + lateral - k * yaw;  // RL
  ws.target[3] = cmd.vx - lateral + k * yaw;  // RR

  for (uint8_t i = 0; i < 4; i++) {
    ws.target[i] = constrain(
      ws.target[i],
      -cfg.get().maxWheelMps,
      cfg.get().maxWheelMps
    );

    if (fabsf(ws.target[i]) < 0.002f) {
      pid[i].reset();
      motor.set(i, 0);
    } else {
      // Feedforward PROPORCIONAL: o degrau vence o atrito estatico e
      // o termo em kv entrega a velocidade pedida. Sem o termo em kv,
      // todo o restante caia no integral, que e lento demais.
      const float mag = cfg.get().staticPwm +
          cfg.get().kvPwm * fabsf(ws.target[i]);
      float ff = ws.target[i] > 0 ? mag : -mag;
      int u = lroundf(constrain(
        ff + pid[i].update(ws.target[i], ws.speed[i], dt),
        -float(cfg.get().pwmLimit),
        float(cfg.get().pwmLimit)
      ));
      if (u != 0 && abs(u) < 45) {
        u = u > 0 ? 45 : -45;
      }
      motor.set(i, u);
    }
    ws.pwm[i] = motor.last(i);

    // Roda travada: pedimos movimento e ela nao anda. Com x1 isto era
    // inutil (a roda lia zero por quantizacao); com x4 + janela longa
    // vira diagnostico de verdade e sai na telemetria.
    if (fabsf(ws.target[i]) > Config::STALL_TARGET &&
        fabsf(ws.speedAvg[i]) < Config::STALL_SPEED) {
      stallSince[i] += lroundf(dt * 1000.0f);
    } else {
      stallSince[i] = 0;
    }
    ws.stalled[i] = stallSince[i] > Config::STALL_MS;
  }
}

// GRUPO RAPIDO: o que o controle do Pi realimenta. 50 Hz.
// Antes tudo saia a 5 Hz, entao a odometria chegava com ate 200 ms de
// idade num laco que decide a 50 Hz.
void telemetryFast() {
  if (millis() - lastTelFast < Config::TELEMETRY_FAST_MS) {
    return;
  }
  lastTelFast = millis();

  Serial.printf(
    "ODOM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    od.x, od.y, od.yaw, od.vx, od.vy, od.wz
  );

  Serial.printf(
    "WHEEL,%.4f,%.4f,%.4f,%.4f\n",
    ws.speedAvg[0], ws.speedAvg[1], ws.speedAvg[2], ws.speedAvg[3]
  );
}

// GRUPO LENTO: diagnostico e coisas que mudam devagar. 5 Hz.
void telemetry() {
  if (millis() - lastTel < Config::TELEMETRY_MS) {
    return;
  }
  lastTel = millis();

  Serial.printf(
    "ENC,%ld,%ld,%ld,%ld\n",
    (long) ws.count[0],
    (long) ws.count[1],
    (long) ws.count[2],
    (long) ws.count[3]
  );

  Serial.printf(
    "TARGET,%.4f,%.4f,%.4f,%.4f\n",
    ws.target[0], ws.target[1], ws.target[2], ws.target[3]
  );

  Serial.printf(
    "PWM,%d,%d,%d,%d\n",
    ws.pwm[0], ws.pwm[1], ws.pwm[2], ws.pwm[3]
  );

  if (imu.state().ready) {
    // O 7o campo e o yawRate: eixo ja escolhido e vies ja removido. Os
    // seis primeiros continuam CRUS de proposito -- e com eles que se
    // descobre a orientacao da placa pela gravidade.
    Serial.printf(
      "IMU,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
      imu.state().ax,
      imu.state().ay,
      imu.state().az,
      imu.state().gx,
      imu.state().gy,
      imu.state().gz,
      imu.state().yawRate
    );
  }

  // Alvo nao-nulo com roda parada. So virou sinal confiavel com x4:
  // com x1 a roda lia zero por quantizacao o tempo todo.
  Serial.printf(
    "STALL,%d,%d,%d,%d\n",
    ws.stalled[0] ? 1 : 0,
    ws.stalled[1] ? 1 : 0,
    ws.stalled[2] ? 1 : 0,
    ws.stalled[3] ? 1 : 0
  );

  Serial.printf("SERVO_STATE,%.2f\n", servoAngle);
  Serial.printf("BATT,%.2f\n", batt.state().voltage);

  // SLIP: wz dos encoders, wz do giroscopio, e a diferenca.
  Serial.printf(
    "SLIP,%.4f,%.4f,%.4f\n",
    od.wzEnc,
    imu.state().ready ? imu.state().yawRate : 0.0f,
    od.slip
  );

  // DIAG: periodo real do controle (us), pico de execucao, e quantos
  // periodos passaram de 15 ms. Alvo e 10000 us com atrasos em zero.
  Serial.printf(
    "DIAG,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu\n",
    (unsigned long) (ctl.amostras ? ctl.periodoMin : 0),
    (unsigned long) (ctl.amostras ? ctl.periodoSoma / ctl.amostras : 0),
    (unsigned long) ctl.periodoMax,
    (unsigned long) ctl.execMax,
    (unsigned long) ctl.atrasos,
    (unsigned long) ctl.amostras,
    stopReason,
    (unsigned long) serial.rejeitadas()
  );
  // A tela de diagnostico le os mesmos numeros que saem na serial.
  menu.setDiag(ctl.periodoMax, ctl.atrasos, serial.rejeitadas(), stopReason);
  ctl.zera();
}

void setup() {
  // Buffer de TX maior ANTES do begin. MEDIDO em 18/09/2026, logo apos
  // subir a telemetria para 48 Hz: 2 linhas em 3283 sairam truncadas no
  // instante em que o host abre a porta -- um fim de ODOM colado num IMU
  // sem o prefixo, assinatura de bytes DESCARTADOS, nao de corrupcao.
  // O default (256 B) nao cobre a rajada do grupo lento caindo em cima
  // do rapido enquanto o host ainda nao esta drenando.
  Serial.setTxBufferSize(2048);
  Serial.setRxBufferSize(512);
  Serial.begin(Config::SERIAL_BAUD);
  pinMode(Config::BTN1, INPUT_PULLUP);
  pinMode(Config::BTN2, INPUT_PULLUP);
  cfg.begin();
  apply();

  servo.setPeriodHertz(50);
  servo.attach(Config::SERVO, Config::SERVO_US_MIN, Config::SERVO_US_MAX);
  servoAponta(0);   // ja nasce no centro calibrado

  motor.begin();
  enc.begin();
  Wire.begin(Config::SDA, Config::SCL);
  Wire.setClock(Config::I2C_HZ);
  imu.begin(Wire);
  if (imu.state().ready) {
    // Eixo do yaw: usa o que esta gravado, mas confere com a gravidade.
    // Se a placa foi remontada desde a ultima calibracao, a gravidade
    // ganha -- e melhor acertar sozinho do que integrar um eixo morto,
    // que foi o que aconteceu ate 18/09/2026 (lia gz num robo cujo eixo
    // vertical e o X).
    imu.setYawAxis(cfg.get().imuYawAxis, cfg.get().imuYawSign);
    imu.detectYawAxis();
    if (imu.state().yawAxis != cfg.get().imuYawAxis) {
      cfg.edit().imuYawAxis = imu.state().yawAxis;
      cfg.save();
    }
    // Calibrar DEPOIS de escolher o eixo: o vies tem de ser do eixo que
    // vai ser integrado, nao de um qualquer.
    imu.calibrate();
  }
  batt.begin();
  fb.begin();
  menu.begin();
  lastControl = micros();
  // Versao no READY: o Pi passa a saber com que firmware esta falando,
  // e a versao do protocolo permite recusar combinacao incompativel.
  Serial.printf(
    "READY,%s,fw=%s,proto=%u,ticks=%.0f,imuaxis=%u\n",
    Config::NAME,
    FIRMWARE_VERSION,
    PROTOCOL_VERSION,
    cfg.get().ticksRev,
    imu.state().yawAxis
  );
}

void loop() {
  SerialProtocol::Cmd s;
  while (serial.poll(s)) {
    if (s.type == SerialProtocol::TWIST) {
      // TWIST,vx,vy,wz -- segue geometry_msgs/Twist: vy = lateral,
      // wz = giro (yaw).
      cmd = {s.a, s.b, s.c};
      lastCmd = millis();
    } else if (s.type == SerialProtocol::STOP) {
      stopReason = STOP_ROS;
      stop();
    } else if (s.type == SerialProtocol::SERVO) {
      servoAngle = constrain(s.a, -90.0f, 90.0f);
      servoAponta(servoAngle);
    } else if (s.type == SerialProtocol::PID) {
      auto& x = cfg.edit();
      x.kp = s.a;
      x.ki = s.b;
      x.kd = s.c;
      apply();
    } else if (s.type == SerialProtocol::SAVE) {
      Serial.println(cfg.save() ? "OK,SAVED" : "ERROR,SAVE");
    } else if (s.type == SerialProtocol::STATUS) {
      Serial.printf(
        "STATUS,IMU_%s,OLED_%s\n",
        imu.state().ready ? "READY" : "NOT_FOUND",
        menu.inTest() ? "TEST" : "READY"
      );
    } else if (s.type == SerialProtocol::MODE_STATE) {
      menu.setModeFeedback((uint8_t) s.a);
    } else if (s.type == SerialProtocol::INFO) {
      menu.setInfo(s.slot, s.text);
    } else if (s.type == SerialProtocol::CONTROL_STATE) {
      menu.setRosControlEnabled(s.a > 0.5f);
    }
  }

  uint32_t now = micros();
  if (now - lastControl >= Config::CONTROL_US) {
    const uint32_t periodoUs = now - lastControl;
    const uint32_t t0 = now;
    float dt = periodoUs * 1e-6f;
    lastControl = now;
    if (dt < 0.1f) {
      if (menu.motorTest()) {
        for (uint8_t i = 0; i < Config::N; i++) {
          motor.set(i, menu.motorTestPwm());
        }
      } else if (menu.servoTest()) {
        // Teste do menu: 90 = centro, agora o CALIBRADO.
        servoAponta(menu.servoTestAngle() - 90.0f);
      } else {
        control(dt);
      }
    }
    ctl.registra(periodoUs, micros() - t0);
  }

  batt.update(cfg.get().battCal);
  MenuAction a = menu.update(cfg);
  if (a == MenuAction::STOP) {
    stopReason = STOP_MENU;
    stop();
    servoAponta(0);
  }
  if (a == MenuAction::RESET_ODOM) {
    od = {};
    fb.beep(1800, 60);
  }
  if (a == MenuAction::RESET_ENCODERS) {
    enc.reset();
    fb.beep(1800, 60);
  }
  if (a == MenuAction::CAL_IMU) {
    stop();
    imu.calibrate();
    fb.beep(2000, 150);
  }
  if (a == MenuAction::SHUTDOWN_PI) {
    // ETAPA 1: so avisa. O Pi anota no log e NAO desliga.
    Serial.println("MENU,SHUTDOWN_REQ");
    fb.beep(1500, 200);
  }
  // Rearma a tela depois do pedido, para dar para testar varias vezes
  // seguidas sem sair e voltar no menu.
  //
  // 1200 ms (era 4000). MEDIDO: com 4 s, cinco toques seguidos do
  // operador viraram DOIS pedidos -- os outros tres caiam dentro da
  // janela e sumiam em silencio, que faz parecer que funcionou quando
  // metade se perdeu. 1200 ms ainda e muito acima de repique de botao
  // (dezenas de ms), entao um toque continua valendo um pedido.
  if (menu.shutdownState() == LocalMenu::Shutdown::PEDIDO &&
      millis() - menu.shutdownAge() > 1200) {
    menu.rearmaShutdown();
  }
  if (a == MenuAction::SET_MODE) {
    // O ESP32 PEDE o modo; quem manda de fato e o Pi, que responde com
    // MODE_STATE. Assim a tela nunca anuncia um modo que o Pi nao esta
    // executando -- por exemplo se o no do joystick nem estiver no ar.
    Serial.printf("MENU,MODE,%u\n", menu.modeRequested());
    fb.beep(2000, 80);
  }
  if (a == MenuAction::TOGGLE_ROS_CONTROL) {
    Serial.println("MENU,CONTROL_TOGGLE");
    fb.beep(2000, 100);
  }
  if (a == MenuAction::SAVE) {
    apply();
    fb.beep(2200, 90);
    Serial.println(cfg.save() ? "OK,SAVED" : "ERROR,SAVE");
  }

  if (menu.inTest() && !menu.motorTest() && !menu.servoTest()) {
    motor.stop();
  }

  bool ros = millis() - lastCmd < Config::CMD_TIMEOUT_MS;
  if (batt.state().critical) {
    stopReason = STOP_BATTERY;
    fb.set(LedState::BATTERY_CRITICAL);
  } else if (batt.state().low) {
    fb.set(LedState::BATTERY_LOW);
  } else if (menu.editing()) {
    fb.set(LedState::MENU);
  } else {
    fb.set(
      ros
        ? LedState::RUN
        : LedState::IDLE
    );
  }

  fb.update();
  // "Andando" = ha comando de movimento de fato, com histerese de 1 s
  // para a tela nao piscar em cada parada curta de curva.
  const bool comandado = fabsf(cmd.vx) > 0.005f ||
      fabsf(cmd.vy) > 0.005f || fabsf(cmd.wz) > 0.01f;
  if (comandado) {
    lastMoving = millis();
  }
  const bool moving = lastMoving != 0 && millis() - lastMoving < 1000;
  menu.draw(batt.state(), ws, od, imu.state(), cfg, ros, moving);
  telemetryFast();
  telemetry();
}