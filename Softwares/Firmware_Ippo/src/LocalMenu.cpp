#include "LocalMenu.h"

#include "Config.h"

namespace {
constexpr uint8_t CONFIG_COUNT = 14;
constexpr uint8_t TEST_COUNT = 8;
constexpr uint32_t LONG_PRESS_MS = 700;
// Toque curto = avanca, toque medio = volta. So o B1 longo (acima de
// LONG_PRESS_MS) continua sendo a parada de emergencia, de qualquer tela.
constexpr uint32_t MEDIUM_PRESS_MS = 250;

// Sem toque por este tempo, o menu volta para STATUS. Nao se aplica a
// tela de teste em andamento nem a edicao de valor: nesses casos o
// operador esta claramente usando, e sumir com a tela no meio seria
// pior que deixar aberto.
constexpr uint32_t IDLE_MS = 10000;

// --- ICONES 8x8 -------------------------------------------------------
// Bitmap de 1 bit, MSB a esquerda. Desenhados a mao para serem legiveis
// a 8 px: numa apresentacao o publico ve a tela de longe, e simbolo
// cheio le melhor que contorno fino.

// Plug: link serial com o Pi vivo.
const uint8_t ICON_LINK[8] = {
  0b00011000,
  0b00111100,
  0b00011000,
  0b00011000,
  0b01011010,
  0b01011010,
  0b00111100,
  0b00011000,
};

// Alvo: IMU respondendo.
const uint8_t ICON_IMU[8] = {
  0b00011000,
  0b00011000,
  0b01111110,
  0b11011011,
  0b11011011,
  0b01111110,
  0b00011000,
  0b00011000,
};

// Triangulo com o vazio do "!": aviso.
const uint8_t ICON_WARN[8] = {
  0b00011000,
  0b00011000,
  0b00111100,
  0b00111100,
  0b01111110,
  0b01100110,
  0b11111111,
  0b11111111,
};

// Roda travada.
const uint8_t ICON_STALL[8] = {
  0b00111100,
  0b01000010,
  0b10100101,
  0b10011001,
  0b10011001,
  0b10100101,
  0b01000010,
  0b00111100,
};

constexpr int16_t BAR_H = 11;   // altura da barra superior

const char* CONFIG_NAMES[CONFIG_COUNT] = {
  "KP", "KI", "KD", "PWM MIN", "PWM LIMITE", "TICKS/VOLTA",
  "CAL BATERIA", "MAX VELOCIDADE", "PESO YAW", "ACEL GIRO",
  "EIXO L", "EIXO W", "DIAM RODA", "SALVAR",
};

const char* TEST_NAMES[TEST_COUNT] = {
  "BATERIA", "SERVO", "4 MOTORES", "ENCODERS", "IMU",
  "RESET ENCODERS", "RESET ODOM", "CONTROLE ROS",
};
}

bool LocalMenu::begin() {
  ready = display.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDR);
  if (!ready) ready = display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  Serial.printf("OLED,%s\n", ready ? "READY" : "NOT_FOUND");
  if (ready) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
  }
  return ready;
}

void LocalMenu::changeConfig(Settings& cfg, int direction) {
  auto& value = cfg.edit();
  switch (configIndex) {
    case 0: value.kp = constrain(value.kp + direction * 5.0f, 0.0f, 500.0f); break;
    case 1: value.ki = constrain(value.ki + direction * 5.0f, 0.0f, 500.0f); break;
    case 2: value.kd = constrain(value.kd + direction * 0.5f, 0.0f, 100.0f); break;
    case 3: value.staticPwm = constrain(value.staticPwm + direction * 5, 0, 255); break;
    case 4: value.pwmLimit = constrain(value.pwmLimit + direction * 5, 0, 255); break;
    case 5: value.ticksRev = constrain(value.ticksRev + direction, 1.0f, 5000.0f); break;
    case 6: value.battCal = constrain(value.battCal + direction * 0.001f, 0.5f, 1.5f); break;
    case 7: value.maxWheelMps = constrain(value.maxWheelMps + direction * 0.05f, 0.1f, 2.0f); break;
    case 8: value.yawEncoderWeight = constrain(value.yawEncoderWeight + direction * 0.05f, 0.0f, 1.0f); break;
    case 9: value.maxWzAccel = constrain(value.maxWzAccel + direction * 0.5f, 0.5f, 20.0f); break;
    case 10: value.halfL = constrain(value.halfL + direction * 0.001f, 0.02f, 0.30f); break;
    case 11: value.halfW = constrain(value.halfW + direction * 0.001f, 0.02f, 0.30f); break;
    case 12: value.wheelDiameter = constrain(value.wheelDiameter + direction * 0.001f, 0.02f, 0.20f); break;
    default: break;
  }
}

void LocalMenu::formatConfigValue(
  uint8_t index,
  const Settings& cfg,
  char* buf,
  size_t len
) {
  const auto& v = cfg.get();
  switch (index) {
    case 0: snprintf(buf, len, "KP %.1f", v.kp); break;
    case 1: snprintf(buf, len, "KI %.1f", v.ki); break;
    case 2: snprintf(buf, len, "KD %.1f", v.kd); break;
    case 3: snprintf(buf, len, "PWM MIN %d", v.staticPwm); break;
    case 4: snprintf(buf, len, "PWM LIM %d", v.pwmLimit); break;
    case 5: snprintf(buf, len, "TICKS %.0f", v.ticksRev); break;
    case 6: snprintf(buf, len, "CAL %.2f", v.battCal); break;
    case 7: snprintf(buf, len, "MAX %.2f", v.maxWheelMps); break;
    case 8: snprintf(buf, len, "YAW %.2f", v.yawEncoderWeight); break;
    case 9: snprintf(buf, len, "ACEL %.1f", v.maxWzAccel); break;
    case 10: snprintf(buf, len, "EIXO L %.1fcm", v.halfL * 100.0f); break;
    case 11: snprintf(buf, len, "EIXO W %.1fcm", v.halfW * 100.0f); break;
    case 12: snprintf(buf, len, "DIAM %.1fcm", v.wheelDiameter * 100.0f); break;
    default: snprintf(buf, len, "SALVAR"); break;
  }
}

MenuAction LocalMenu::update(Settings& cfg) {
  const bool button1 = digitalRead(Config::BTN1);
  const bool button2 = digitalRead(Config::BTN2);
  const uint32_t now = millis();
  MenuAction action = MenuAction::NONE;

  if (previousButton1 && !button1) button1Down = now;
  if (previousButton2 && !button2) button2Down = now;

  // Qualquer atividade de botao adia o retorno automatico.
  if (!button1 || !button2 || previousButton1 != button1 ||
      previousButton2 != button2) {
    lastInput = now;
  }

  if (!previousButton1 && button1) {
    const uint32_t heldMs = now - button1Down;
    if (heldMs >= LONG_PRESS_MS) {
      // Parada de emergencia: sempre disponivel, de qualquer tela.
      action = MenuAction::STOP;
      screen = Screen::HOME;
      editingValue = false;
      motorPwm = 0;
    } else {
      // Toque curto avanca (+1), toque medio volta (-1).
      const int direction = heldMs >= MEDIUM_PRESS_MS ? -1 : 1;
      if (screen == Screen::STATUS) {
        screen = Screen::HOME;
      } else if (screen == Screen::HOME) {
        homeTab = (uint8_t) ((homeTab + 6 + direction) % 6);
      } else if (screen == Screen::MODE) {
        modeIndex = (uint8_t) ((modeIndex + 3 + direction) % 3);
      } else if (screen == Screen::CONFIG_LIST) {
        if (editingValue) changeConfig(cfg, 1);
        else configIndex = (configIndex + CONFIG_COUNT + direction) % CONFIG_COUNT;
      } else if (screen == Screen::TEST_LIST) {
        testIndex = (testIndex + TEST_COUNT + direction) % TEST_COUNT;
      } else if (screen == Screen::TEST_SERVO) {
        servoAngle = constrain(servoAngle + direction * 10, 0, 180);
      } else if (screen == Screen::TEST_MOTORS) {
        motorPwm = constrain(
          motorPwm + direction * 50,
          -Config::PWM_MAX,
          Config::PWM_MAX
        );
      } else if (screen == Screen::TEST_IMU) {
        action = MenuAction::CAL_IMU;
      }
    }
  }

  if (!previousButton2 && button2) {
    if (now - button2Down >= LONG_PRESS_MS) {
      if (screen == Screen::CONFIG_LIST && editingValue) {
        editingValue = false;
        action = MenuAction::NONE;
      } else {
        action = MenuAction::SAVE;
        screen = Screen::HOME;
        editingValue = false;
        motorPwm = 0;
      }
    } else if (screen == Screen::STATUS) {
      screen = Screen::HOME;
    } else if (screen == Screen::HOME) {
      switch (homeTab) {
        case 0: screen = Screen::MODE; break;
        case 1: screen = Screen::CONFIG_LIST; break;
        case 2: screen = Screen::TEST_LIST; break;
        case 3: screen = Screen::HOST; break;
        case 4: screen = Screen::DIAG; break;
        default:
          screen = Screen::SHUTDOWN;
          shutdown = Shutdown::CONFIRMA;
          break;
      }
    } else if (screen == Screen::MODE) {
      // B2 confirma o modo destacado. O Pi so muda de comportamento
      // quando recebe isto -- ate la a tela mostra o modo ANTIGO.
      action = MenuAction::SET_MODE;
    } else if (screen == Screen::SHUTDOWN) {
      // TOQUE CURTO confirma, a pedido do operador.
      //
      // Continua exigindo DOIS toques no total: um entra na tela (pelo
      // HOME) e outro confirma. Nesta etapa nada e desligado, entao o
      // toque curto e seguro e facilita justamente o que interessa
      // agora -- disparar de proposito e ver se o log registra, para
      // depois reconhecer um disparo que NAO foi de proposito.
      if (shutdown == Shutdown::CONFIRMA) {
        action = MenuAction::SHUTDOWN_PI;
        shutdown = Shutdown::PEDIDO;
        shutdownSince = now;
      }
    } else if (screen == Screen::HOST || screen == Screen::DIAG) {
      screen = Screen::HOME;
    } else if (screen == Screen::CONFIG_LIST) {
      if (editingValue) changeConfig(cfg, -1);
      else if (configIndex == CONFIG_COUNT - 1) action = MenuAction::SAVE;
      else editingValue = true;
    } else if (screen == Screen::TEST_LIST) {
      switch (testIndex) {
        case 0: screen = Screen::TEST_BATTERY; break;
        case 1: screen = Screen::TEST_SERVO; break;
        case 2: screen = Screen::TEST_MOTORS; break;
        case 3: screen = Screen::TEST_ENCODERS; break;
        case 4: screen = Screen::TEST_IMU; break;
        case 5: action = MenuAction::RESET_ENCODERS; break;
        case 6: action = MenuAction::RESET_ODOM; break;
        default: action = MenuAction::TOGGLE_ROS_CONTROL; break;
      }
      motorPwm = 0;
    } else {
      screen = Screen::TEST_LIST;
      motorPwm = 0;
    }
  }

  // Volta sozinho para STATUS. Fora de teste e de edicao, porque ali o
  // operador esta no meio de alguma coisa. Ao sair, zera o PWM de teste
  // por seguranca -- se a tela de motores expirasse com valor aplicado,
  // o robo ficaria andando sem ninguem olhando.
  if (screen != Screen::STATUS && !editingValue && !inTest() &&
      now - lastInput > IDLE_MS) {
    screen = Screen::STATUS;
    motorPwm = 0;
  }

  previousButton1 = button1;
  previousButton2 = button2;
  return action;
}

void LocalMenu::printFrame(const char* title, const char* value, const char* help) {
  const uint32_t elapsed = millis() - transitionStart;
  const int offset = elapsed < 150 ? 64 - (elapsed * 64 / 150) : 0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, offset);
  display.println(title);
  display.println();
  display.println(value);
  display.setCursor(0, 48 + offset);
  display.println(help);
  display.display();
}

void LocalMenu::drawHome(bool ros) {
  const uint32_t elapsed = millis() - transitionStart;
  const int offset = elapsed < 150 ? 64 - (elapsed * 64 / 150) : 0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, offset);
  display.println("MENU PRINCIPAL");
  display.setCursor(0, 20 + offset);
  display.print(homeTab == 0 ? "> " : "  ");
  display.println("CONFIGURACOES");
  display.print(homeTab == 1 ? "> " : "  ");
  display.println("TESTES");
  display.setCursor(0, 52 + offset);
  display.println(ros ? "B1 troca  B2 entra" : "B1 troca  B2 entra");
  display.display();
}

void LocalMenu::drawStatus(
  const BatteryState& battery,
  const Odom& odometry,
  const ImuState& imuState,
  bool ros
) {
  drawTopBar("STATUS", battery, imuState, ros);

  // Tensao em fonte grande: e o numero que mais se olha, e o unico que
  // precisa ser legivel de longe numa apresentacao.
  char buf[24];
  snprintf(buf, sizeof buf, "%.2fV", battery.voltage);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, BAR_H + 3);
  display.print(buf);

  display.setTextSize(1);
  display.setCursor(78, BAR_H + 3);
  display.print(battery.critical ? "CRIT"
                : battery.low ? "BAIXA" : "OK");
  display.setCursor(78, BAR_H + 12);
  static const char* CURTO[3] = {"PARADO", "LINHA", "RC"};
  display.print(CURTO[(uint8_t) mode]);

  // Barras bipolares: o sinal da velocidade e tao informativo quanto o
  // modulo, e barra le mais rapido que numero com sinal.
  display.setCursor(2, BAR_H + 22);
  display.print("VX");
  drawBar(20, BAR_H + 23, 76, odometry.vx, 0.30f);
  snprintf(buf, sizeof buf, "%+.2f", odometry.vx);
  display.setCursor(96, BAR_H + 22);
  display.print(buf);

  display.setCursor(2, BAR_H + 32);
  display.print("WZ");
  drawBar(20, BAR_H + 33, 76, odometry.wz, 2.00f);
  snprintf(buf, sizeof buf, "%+.2f", odometry.wz);
  display.setCursor(96, BAR_H + 32);
  display.print(buf);

  // 21 caracteres e o limite real (6 px cada em 128). O IP sozinho tem
  // 13 e sobra; com "B1 menu   " na frente dava 23 e era cortado.
  display.setCursor(2, BAR_H + 43);
  if (infoRecebido && info[0][0]) {
    display.print(info[0]);
  } else {
    display.print("B1 abre o menu");
  }
  display.display();
}

void LocalMenu::drawConfigList(const Settings& cfg) {
  char value[24];
  formatConfigValue(configIndex, cfg, value, sizeof value);

  // Em edicao o valor vai grande e sozinho: quem esta ajustando quer ver
  // o numero mudar, nao a lista.
  if (editingValue) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(2, BAR_H + 3);
    display.print(CONFIG_NAMES[configIndex]);
    display.setTextSize(2);
    display.setCursor(2, BAR_H + 16);
    display.print(value);
    display.setTextSize(1);
    display.setCursor(2, 55);
    display.print("B1 +   B2 -   B2long ok");
    return;
  }

  drawList(CONFIG_NAMES, CONFIG_COUNT, configIndex, value);
}

void LocalMenu::drawTestList() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("TESTES %u/%u", testIndex + 1, TEST_COUNT);
  for (uint8_t row = 0; row < 3; row++) {
    const uint8_t index = (testIndex + row + TEST_COUNT - 1) % TEST_COUNT;
    display.setCursor(0, 12 + row * 12);
    display.printf("%c %s", row == 1 ? '>' : ' ', TEST_NAMES[index]);
  }
  display.setCursor(0, 52);
  if (testIndex == TEST_COUNT - 1) {
    display.printf("ROS %s B2 alterna", rosControlEnabled ? "ON" : "OFF");
  } else {
    display.println("B1/med volta  B2 abre");
  }
  display.display();
}

void LocalMenu::drawTestScreen(
  const BatteryState& battery,
  const WheelState& wheels,
  const Odom& odometry,
  const ImuState& imuState
) {
  char value[32];
  (void) odometry;
  if (screen == Screen::TEST_BATTERY) {
    snprintf(value, sizeof value, "%.2f V %s", battery.voltage, battery.critical ? "CRITICA" : battery.low ? "BAIXA" : "OK");
    printFrame("TESTE BATERIA", value, "B2 voltar");
  } else if (screen == Screen::TEST_SERVO) {
    snprintf(value, sizeof value, "ANGULO %d graus", servoAngle);
    printFrame("TESTE SERVO", value, "B1 +  med -  B2 voltar");
  } else if (screen == Screen::TEST_MOTORS) {
    snprintf(value, sizeof value, "TODOS PWM %d", motorPwm);
    printFrame("TESTE 4 MOTORES", value, "B1 +  med -  B2 voltar");
  } else if (screen == Screen::TEST_IMU) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("TESTE IMU");
    if (!imuState.ready) {
      display.println("IMU NAO ENCONTRADA");
    } else {
      char line[32];
      snprintf(line, sizeof line, "A %.2f %.2f %.2f", imuState.ax, imuState.ay, imuState.az);
      display.println(line);
      snprintf(line, sizeof line, "G %.2f %.2f %.2f", imuState.gx, imuState.gy, imuState.gz);
      display.println(line);
      snprintf(line, sizeof line, "BIAS GZ %.3f", imuState.bias);
      display.println(line);
    }
    display.setCursor(0, 52);
    display.println("B1 calibra  B2 volta");
    display.display();
  } else {
    snprintf(
      value, sizeof value, "%ld %ld %ld %ld",
      (long) wheels.count[0], (long) wheels.count[1],
      (long) wheels.count[2], (long) wheels.count[3]
    );
    printFrame("TESTE ENCODERS", value, "B2 voltar");
  }
}

void LocalMenu::draw(
  const BatteryState& battery,
  const WheelState& wheels,
  const Odom& odometry,
  const ImuState& imuState,
  const Settings& cfg,
  bool ros,
  bool moving
) {
  if (!ready) return;

  // TELA APAGADA COM O CONTROLE ATIVO.
  //
  // MEDIDO em 10/09/2026: display.display() empurra 1024 bytes por I2C a
  // 400 kHz, ~23 ms BLOQUEANDO o loop() -- que e o mesmo loop do
  // controle a 100 Hz. A cada 500 ms o dt do PID pulava de 10 para
  // ~33 ms. Na pista ninguem le OLED, entao o preco nao se justifica.
  //
  // Excecoes: edicao de valor e tela de teste. Ali a intencao humana e
  // explicita e vale mais que o jitter.
  // CORRIGIDO 18/09/2026: a condicao era !ros, e estava errada. O
  // motor_serial_node publica TWIST a 50 Hz mesmo com o robo PARADO,
  // entao "ros" fica verdadeiro o tempo todo com a pilha no ar e a tela
  // nunca mais acendia -- inutil justamente na bancada, que e onde ela
  // serve. O que custa os 23 ms de I2C e o robo ANDANDO, nao o link
  // estar vivo.
  const bool querTela = !moving || editingValue || inTest();
  if (!querTela) {
    if (displayOn) {
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      displayOn = false;
    }
    return;
  }
  if (!displayOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    displayOn = true;
    lastDraw = 0;          // redesenha ja, sem esperar o periodo
    transitionStart = millis();
  }

  if (millis() - lastDraw < Config::OLED_MS) return;
  lastDraw = millis();
  if (screen != drawnScreen) {
    drawnScreen = screen;
    transitionStart = millis();
  }

  // UM clear para todas as telas. Antes o clear estava espalhado por
  // cada funcao de desenho e o STATUS ficou sem -- ele desenhava POR
  // CIMA do menu anterior, e o efeito era titulo ilegivel, "ROS ATIVO"
  // com lixo em cima e, na volta automatica dos 10 s, as duas telas
  // aparentemente fundidas. Centralizar aqui torna impossivel repetir.
  display.clearDisplay();

  if (screen == Screen::STATUS) {
    drawStatus(battery, odometry, imuState, ros);
    return;
  }

  if (screen == Screen::SHUTDOWN) {
    drawTopBar("DESLIGAR PI", battery, imuState, ros);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    if (shutdown == Shutdown::PEDIDO) {
      display.setCursor(2, BAR_H + 4);
      display.println("Desligando...");
      display.setCursor(2, BAR_H + 18);
      display.println("NAO corte a energia");
      display.setCursor(2, BAR_H + 28);
      display.println("enquanto esta tela");
      display.setCursor(2, BAR_H + 38);
      display.println("estiver acesa.");
      display.setCursor(2, BAR_H + 46);
      display.println("Apagou = pode cortar.");
      display.setCursor(2, 55);
      display.printf("ha %lus", (unsigned long) ((millis() - shutdownSince) / 1000));
    } else {
      display.setCursor(2, BAR_H + 4);
      display.println("Desligar a Raspberry");
      display.setCursor(2, BAR_H + 14);
      display.println("da Raspberry?");
      display.setCursor(2, BAR_H + 30);
      display.println("B2 confirma");
      display.setCursor(2, BAR_H + 40);
      display.println("B1 volta");
    }
    display.display();
    return;
  }
  if (screen == Screen::MODE) {
    drawTopBar("MODO", battery, imuState, ros);
    static const char* NOMES[3] = {"Parado", "Seguidor de linha", "RC (joystick)"};
    drawList(NOMES, 3, modeIndex, nullptr);
    // O modo ATIVO fica escrito embaixo, separado do destacado: o
    // operador precisa ver o que o robo esta fazendo AGORA, nao so o
    // que ele esta prestes a escolher.
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(2, 54);
    display.print("ativo: ");
    display.print(NOMES[(uint8_t) mode]);
    display.display();
    return;
  }
  if (screen == Screen::HOST) {
    drawTopBar("HOST", battery, imuState, ros);
    drawHostScreen(ros);
    display.display();
    return;
  }
  if (screen == Screen::DIAG) {
    drawTopBar("DIAGNOSTICO", battery, imuState, ros);
    drawDiagScreen();
    display.display();
    return;
  }
  if (screen == Screen::HOME) {
    drawTopBar("MENU", battery, imuState, ros);
    static const char* ITENS[6] = {
      "Modo de operacao", "Configuracoes", "Testes",
      "Host / IP", "Diagnostico", "Desligar Raspberry"
    };
    drawList(ITENS, 6, homeTab, nullptr);
    display.display();
    return;
  }
  if (screen == Screen::CONFIG_LIST) {
    drawTopBar("CONFIG", battery, imuState, ros);
    drawConfigList(cfg);
    display.display();
    return;
  }
  if (screen == Screen::TEST_LIST) {
    drawTopBar("TESTES", battery, imuState, ros);
    drawList(TEST_NAMES, TEST_COUNT, testIndex, nullptr);
    display.display();
    return;
  }
  drawTestScreen(battery, wheels, odometry, imuState);
}

// --- PRIMITIVAS DA INTERFACE -----------------------------------------

void LocalMenu::drawIcon(int16_t x, int16_t y, const uint8_t* icon) {
  for (uint8_t linha = 0; linha < 8; linha++) {
    const uint8_t bits = icon[linha];
    for (uint8_t col = 0; col < 8; col++) {
      if (bits & (0x80 >> col)) {
        display.drawPixel(x + col, y + linha, SSD1306_WHITE);
      }
    }
  }
}

// Bateria desenhada, nao bitmap: o NIVEL precisa ser visivel de relance,
// e barra preenchida comunica isso melhor que qualquer simbolo fixo.
// 3S LiPo: 12.6 cheia, 11.1 nominal, 11.0 critica.
void LocalMenu::drawBatteryIcon(
  int16_t x,
  int16_t y,
  const BatteryState& battery
) {
  display.drawRect(x, y + 1, 13, 7, SSD1306_WHITE);
  display.drawRect(x + 13, y + 3, 2, 3, SSD1306_WHITE);
  float frac = (battery.voltage - 10.8f) / (12.6f - 10.8f);
  frac = frac < 0.0f ? 0.0f : (frac > 1.0f ? 1.0f : frac);
  const int16_t cheio = (int16_t) (frac * 11.0f + 0.5f);
  if (cheio > 0) {
    display.fillRect(x + 1, y + 2, cheio, 5, SSD1306_WHITE);
  }
  // Critica pisca: no meio de uma apresentacao, ninguem le numero, mas
  // todo mundo enxerga algo piscando.
  if (battery.critical && (millis() / 400) % 2) {
    display.fillRect(x, y + 1, 15, 7, SSD1306_BLACK);
  }
}

void LocalMenu::drawTopBar(
  const char* title,
  const BatteryState& battery,
  const ImuState& imuState,
  bool ros
) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 1);
  display.print(title);

  // Icones alinhados a direita, na ordem: aviso, travada, IMU, link,
  // bateria. A bateria fica sempre na ponta para virar ponto de
  // referencia fixo do olho.
  int16_t x = 127 - 15;
  drawBatteryIcon(x, 0, battery);
  x -= 10;
  if (ros) {
    drawIcon(x, 0, ICON_LINK);
    x -= 10;
  }
  if (imuState.ready) {
    drawIcon(x, 0, ICON_IMU);
    x -= 10;
  }
  if (battery.low || battery.critical) {
    drawIcon(x, 0, ICON_WARN);
    x -= 10;
  }
  display.drawFastHLine(0, BAR_H - 1, 128, SSD1306_WHITE);
}

// Lista rolavel. Mostra 4 itens, mantem o selecionado sempre visivel e
// desenha a barra de rolagem so quando a lista nao cabe -- sem isso o
// operador nao sabe se ha mais coisa abaixo.
void LocalMenu::drawList(
  const char* const* items,
  uint8_t count,
  uint8_t index,
  const char* valueOfSelected
) {
  constexpr uint8_t VISIVEIS = 4;
  uint8_t topo = 0;
  if (count > VISIVEIS) {
    if (index >= VISIVEIS - 1) {
      topo = index - (VISIVEIS - 2);
    }
    if (topo + VISIVEIS > count) {
      topo = count - VISIVEIS;
    }
  }

  for (uint8_t linha = 0; linha < VISIVEIS && topo + linha < count; linha++) {
    const uint8_t i = topo + linha;
    const int16_t y = BAR_H + 1 + linha * 10;
    const bool sel = (i == index);
    if (sel) {
      display.fillRect(0, y - 1, 122, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(2, y);
    display.print(items[i]);
    if (sel && valueOfSelected && valueOfSelected[0]) {
      const int16_t largura = (int16_t) strlen(valueOfSelected) * 6;
      display.setCursor(120 - largura, y);
      display.print(valueOfSelected);
    }
  }
  display.setTextColor(SSD1306_WHITE);

  if (count > VISIVEIS) {
    display.drawRect(124, BAR_H, 4, 64 - BAR_H, SSD1306_WHITE);
    const int16_t alturaTotal = 64 - BAR_H - 2;
    int16_t h = alturaTotal * VISIVEIS / count;
    if (h < 3) h = 3;
    const int16_t y = BAR_H + 1 +
        (alturaTotal - h) * index / (count - 1);
    display.fillRect(125, y, 2, h, SSD1306_WHITE);
  }
}

// Barra bipolar centrada: negativo cresce para a esquerda, positivo para
// a direita. Serve para velocidade de roda, onde o SINAL importa tanto
// quanto o modulo.
void LocalMenu::drawBar(
  int16_t x,
  int16_t y,
  int16_t w,
  float value,
  float maxAbs
) {
  const int16_t meio = x + w / 2;
  display.drawFastVLine(meio, y - 1, 7, SSD1306_WHITE);
  if (maxAbs <= 0.0f) return;
  float frac = value / maxAbs;
  frac = frac < -1.0f ? -1.0f : (frac > 1.0f ? 1.0f : frac);
  const int16_t comp = (int16_t) (frac * (w / 2 - 1));
  if (comp >= 0) {
    display.fillRect(meio, y, comp + 1, 5, SSD1306_WHITE);
  } else {
    display.fillRect(meio + comp, y, -comp, 5, SSD1306_WHITE);
  }
}

void LocalMenu::setInfo(uint8_t slot, const char* text) {
  if (slot >= INFO_SLOTS) return;
  strncpy(info[slot], text, INFO_LEN - 1);
  info[slot][INFO_LEN - 1] = 0;
  infoRecebido = true;
}

void LocalMenu::setDiag(
  uint32_t periodoMaxUs,
  uint32_t atrasos,
  uint32_t crcBad,
  uint8_t stopReason
) {
  diagPeriodoMax = periodoMaxUs;
  diagAtrasos = atrasos;
  diagCrcBad = crcBad;
  diagStopReason = stopReason;
}

// Tela HOST: o que o Pi empurra por SET_INFO. Existe por um motivo bem
// pratico -- descobrir o IP para abrir SSH sem precisar de monitor, de
// teclado ou de adivinhar no roteador.
void LocalMenu::drawHostScreen(bool ros) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  if (!infoRecebido) {
    display.setCursor(2, BAR_H + 6);
    display.println("Sem dados do host.");
    display.setCursor(2, BAR_H + 18);
    display.println("O Pi envia por");
    display.setCursor(2, BAR_H + 28);
    display.println("SET_INFO na serial.");
    display.setCursor(2, BAR_H + 42);
    display.print(ros ? "Link: OK" : "Link: ausente");
    return;
  }
  static const char* rotulos[INFO_SLOTS] = {"IP ", "HOST", "ROS", ""};
  int16_t y = BAR_H + 3;
  for (uint8_t i = 0; i < INFO_SLOTS; i++) {
    if (!info[i][0]) continue;
    display.setCursor(2, y);
    if (rotulos[i][0]) {
      display.print(rotulos[i]);
      display.print(' ');
    }
    display.print(info[i]);
    y += 10;
  }
}

// Tela DIAG: os numeros que dizem se o firmware esta saudavel. Foram
// eles que provaram, em 18/09/2026, que apagar a tela durante o controle
// zera o jitter -- e que por isso nao era preciso task separada.
void LocalMenu::drawDiagScreen() {
  static const char* MOTIVOS[5] = {
    "-", "ROS", "WATCHDOG", "BOTAO", "BATERIA"
  };
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  char linha[24];

  snprintf(linha, sizeof linha, "ctl max %lu us",
           (unsigned long) diagPeriodoMax);
  display.setCursor(2, BAR_H + 3);
  display.println(linha);

  snprintf(linha, sizeof linha, "atrasos  %lu",
           (unsigned long) diagAtrasos);
  display.setCursor(2, BAR_H + 13);
  display.println(linha);

  snprintf(linha, sizeof linha, "crc ruim %lu",
           (unsigned long) diagCrcBad);
  display.setCursor(2, BAR_H + 23);
  display.println(linha);

  snprintf(linha, sizeof linha, "parou: %s",
           MOTIVOS[diagStopReason < 5 ? diagStopReason : 0]);
  display.setCursor(2, BAR_H + 33);
  display.println(linha);

  const uint32_t s = millis() / 1000;
  snprintf(linha, sizeof linha, "ligado %lu:%02lu:%02lu",
           (unsigned long) (s / 3600),
           (unsigned long) ((s / 60) % 60),
           (unsigned long) (s % 60));
  display.setCursor(2, BAR_H + 43);
  display.println(linha);
}
