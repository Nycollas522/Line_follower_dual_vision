#pragma once

#include <Adafruit_SSD1306.h>

#include "Settings.h"
#include "RobotTypes.h"

enum class MenuAction : uint8_t {
  NONE,
  STOP,
  RESET_ODOM,
  RESET_ENCODERS,
  CAL_IMU,
  SET_MODE,
  SHUTDOWN_PI,
  SAVE,
  TOGGLE_ROS_CONTROL,
};

class LocalMenu {
 public:
  // MODO DE OPERACAO, escolhido aqui e obedecido pelo Pi.
  //
  // Mora no firmware de proposito: com o Pi subindo sozinho no boot, o
  // ESP32 e a unica interface que existe sem PC por perto. Trocar de
  // modo NAO reinicia processo nenhum do lado do Pi -- os nos ficam
  // todos no ar e apenas mudam de comportamento, senao cada troca
  // custaria os ~25 s de subida da pilha.
  enum class RobotMode : uint8_t {
    IDLE = 0,      // nada dirige; cameras seguem publicando
    FOLLOWER = 1,  // seguidor de linha autonomo
    RC = 2,        // joystick; percepcao desligada para liberar CPU
  };

  // Etapas do desligamento da Raspberry.
  //
  // ETAPA 1 (22/09/2026): o Pi apenas REGISTRA o pedido, nao desliga.
  // A primeira tentativa foi entregue sem teste possivel -- o ESP32
  // precisava ser gravado e quem tem o robo e o operador -- e o Pi
  // passou a desligar sozinho. O caminho de confirmacao que eu escrevi
  // era INALCANCAVEL (ficava numa cadeia else-if depois do ramo de
  // pressao longa, que captura tudo), entao a causa real nunca foi
  // encontrada. Registrar sem desligar e o que permite encontra-la.
  enum class Shutdown : uint8_t {
    IDLE,
    CONFIRMA,  // na tela, esperando pressao LONGA
    PEDIDO,    // enviado; nesta etapa o Pi so anota no log
  };

  bool begin();
  MenuAction update(Settings& cfg);
  void draw(
    const BatteryState& battery,
    const WheelState& wheels,
    const Odom& odometry,
    const ImuState& imuState,
    const Settings& cfg,
    bool ros,
    bool moving
  );

  void setRosControlEnabled(bool enabled) { this->rosControlEnabled = enabled; }

  // Texto empurrado pelo Pi via SET_INFO. Slot 0 = IP, 1 = host,
  // 2 = estado do ROS, 3 = livre. O firmware nao interpreta nada: so
  // guarda e desenha, entao acrescentar informacao nao exige reflash.
  static constexpr uint8_t INFO_SLOTS = 4;
  static constexpr uint8_t INFO_LEN = 22;
  void setInfo(uint8_t slot, const char* text);
  bool hasInfo() const { return infoRecebido; }

  // Numeros de diagnostico vindos do laco de controle.
  void setDiag(
    uint32_t periodoMaxUs,
    uint32_t atrasos,
    uint32_t crcBad,
    uint8_t stopReason
  );

  bool editing() const { return editingValue; }
  RobotMode currentMode() const { return mode; }
  uint8_t modeRequested() const { return modeIndex; }
  Shutdown shutdownState() const { return shutdown; }
  uint32_t shutdownAge() const { return shutdownSince; }
  void rearmaShutdown() {
    if (screen == Screen::SHUTDOWN) {
      shutdown = Shutdown::CONFIRMA;
    } else {
      shutdown = Shutdown::IDLE;
    }
  }
  // O no serial avisa qual modo o Pi confirmou, para a tela nunca
  // mostrar um modo que o Pi nao esta de fato executando.
  void setModeFeedback(uint8_t m) {
    mode = m > 2 ? RobotMode::IDLE : (RobotMode) m;
    modeIndex = m > 2 ? 0 : m;
  }
  bool inTest() const {
    return screen == Screen::TEST_BATTERY ||
           screen == Screen::TEST_SERVO ||
           screen == Screen::TEST_MOTORS ||
           screen == Screen::TEST_ENCODERS ||
           screen == Screen::TEST_IMU;
  }
  bool motorTest() const { return screen == Screen::TEST_MOTORS; }
  bool servoTest() const { return screen == Screen::TEST_SERVO; }
  int motorTestPwm() const { return motorPwm; }
  int servoTestAngle() const { return servoAngle; }
  enum class Screen : uint8_t {
    STATUS,
    HOME,
    CONFIG_LIST,
    TEST_LIST,
    TEST_BATTERY,
    TEST_SERVO,
    TEST_MOTORS,
    TEST_ENCODERS,
    TEST_IMU,
    HOST,      // IP da Raspberry e estado do ROS, empurrados pela serial
    DIAG,      // jitter do controle, CRC recusado, motivo da parada
    MODE,      // escolhe o que o Pi deve fazer: parado, seguidor ou RC
    SHUTDOWN,  // pede desligamento seguro da Raspberry
  };


  Adafruit_SSD1306 display{128, 64, &Wire, -1};
  Screen screen = Screen::STATUS;
  uint8_t homeTab = 0;
  uint8_t configIndex = 0;
  uint8_t testIndex = 0;
  uint8_t motorIndex = 0;
  bool editingValue = false;
  uint32_t button1Down = 0;
  uint32_t button2Down = 0;
  uint32_t lastDraw = 0;
  // Ultimo instante em que um botao foi tocado. Depois de IDLE_MS sem
  // toque o menu volta sozinho para STATUS -- ninguem quer achar o robo
  // parado numa tela de config no dia seguinte.
  uint32_t lastInput = 0;
  // Estado FISICO do painel. O SSD1306 guarda o proprio framebuffer,
  // entao parar de desenhar CONGELA a imagem em vez de apagar -- numeros
  // velhos parecendo vivos enganam mais que tela escura.
  bool displayOn = true;
  uint32_t transitionStart = 0;
  bool previousButton1 = true;
  bool previousButton2 = true;
  bool ready = false;
  char info[INFO_SLOTS][INFO_LEN] = {};
  bool infoRecebido = false;
  uint32_t diagPeriodoMax = 0;
  uint32_t diagAtrasos = 0;
  uint32_t diagCrcBad = 0;
  uint8_t diagStopReason = 0;
  Screen drawnScreen = Screen::HOME;
  int motorPwm = 0;
  int servoAngle = 90;
  bool rosControlEnabled = false;
  RobotMode mode = RobotMode::IDLE;
  Shutdown shutdown = Shutdown::IDLE;
  uint32_t shutdownSince = 0;
  uint8_t modeIndex = 0;

 private:
  // --- primitivas da interface ---
  // Barra superior com titulo e icones de estado. Presente em TODAS as
  // telas para que a leitura seja a mesma em qualquer lugar do menu.
  void drawTopBar(
    const char* title,
    const BatteryState& battery,
    const ImuState& imuState,
    bool ros
  );
  void drawIcon(int16_t x, int16_t y, const uint8_t* icon);
  void drawBatteryIcon(int16_t x, int16_t y, const BatteryState& battery);
  // Lista rolavel com selecao invertida e indicador de rolagem.
  void drawList(
    const char* const* items,
    uint8_t count,
    uint8_t index,
    const char* valueOfSelected
  );
  void drawBar(int16_t x, int16_t y, int16_t w, float value, float maxAbs);
  void drawHostScreen(bool ros);
  void drawDiagScreen();

  void changeConfig(Settings& cfg, int direction);
  void formatConfigValue(
    uint8_t index,
    const Settings& cfg,
    char* buf,
    size_t len
  );
  void drawHome(bool ros);
  void drawStatus(
    const BatteryState& battery,
    const Odom& odometry,
    const ImuState& imuState,
    bool ros
  );
  void drawConfigList(const Settings& cfg);
  void drawTestList();
  void drawTestScreen(
    const BatteryState& battery,
    const WheelState& wheels,
    const Odom& odometry,
    const ImuState& imuState
  );
  void printFrame(const char* title, const char* value, const char* help);
};