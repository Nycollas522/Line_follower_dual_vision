#pragma once

#include <Arduino.h>

class SerialProtocol {
 public:
  enum Type : uint8_t {
    NONE,
    TWIST,
    STOP,
    SERVO,
    PID,
    STATUS,
    SAVE,
    CONTROL_STATE,
    INFO,
    MODE_STATE,
  };

  struct Cmd {
    Type type = NONE;
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    // Texto do SET_INFO. O firmware nao interpreta o conteudo: guarda e
    // desenha. Assim o Pi pode mandar IP, hostname ou o que for util
    // depois sem exigir firmware novo.
    uint8_t slot = 0;
    char text[22] = {};
  };

  bool poll(Cmd& x);

  // Linhas recebidas com "*XX" no fim e CRC errado. Sai em DIAG: se este
  // numero sair do zero, o link esta corrompendo comando -- e comando
  // corrompido num robo que anda e o pior modo de falha do sistema.
  uint32_t rejeitadas() const { return bad; }

  static uint8_t crc8(const char* data, uint8_t len);

 private:
  // Confere e REMOVE o sufixo "*XX" se existir. Devolve false so quando
  // existe e nao bate -- linha sem sufixo passa, para nao quebrar
  // ferramenta antiga nem o `echo > /dev/ttyACM0` na mao.
  bool checaCrc(char* linha, uint8_t& len);

  char buf[96] = {};
  uint8_t n = 0;
  uint32_t bad = 0;
};