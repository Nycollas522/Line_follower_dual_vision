#include "SerialProtocol.h"

#include <cstring>

// CRC-8 poly 0x07 (CRC-8/ATM). Sem tabela: sao 96 bytes no pior caso a
// 50 Hz, entao o custo e irrelevante e nao gasta flash com tabela.
uint8_t SerialProtocol::crc8(const char* data, uint8_t len) {
  uint8_t c = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    c ^= static_cast<uint8_t>(data[i]);
    for (uint8_t b = 0; b < 8; b++) {
      c = (c & 0x80) ? static_cast<uint8_t>((c << 1) ^ 0x07)
                     : static_cast<uint8_t>(c << 1);
    }
  }
  return c;
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Formato aceito: "COMANDO,args*XX" onde XX e o CRC8 em hex do trecho
// antes do '*'. Sem '*' a linha passa inalterada -- compatibilidade
// deliberada: o protocolo antigo continua valendo e da para digitar
// comando a mao durante depuracao.
bool SerialProtocol::checaCrc(char* linha, uint8_t& len) {
  if (len < 3) {
    return true;
  }
  const uint8_t pos = len - 3;
  if (linha[pos] != '*') {
    return true;
  }
  const int hi = hexVal(linha[pos + 1]);
  const int lo = hexVal(linha[pos + 2]);
  if (hi < 0 || lo < 0) {
    return true;   // nao e sufixo de CRC, e conteudo que por acaso tem '*'
  }
  const uint8_t esperado = static_cast<uint8_t>((hi << 4) | lo);
  linha[pos] = 0;
  len = pos;
  if (crc8(linha, len) != esperado) {
    bad++;
    return false;
  }
  return true;
}

bool SerialProtocol::poll(Cmd& x) {
  x = {};

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (!n) {
        continue;
      }

      buf[n] = 0;
      uint8_t len = n;
      n = 0;

      if (!checaCrc(buf, len)) {
        return false;   // CRC errado: descarta em silencio, conta em bad
      }

      // TWIST primeiro: e o comando mais frequente (50 Hz) e antes
      // pagava 3 strcmp e um sscanf falho antes de casar.
      if (sscanf(buf, "TWIST,%f,%f,%f", &x.a, &x.b, &x.c) == 3) {
        x.type = TWIST;
        return true;
      }
      if (sscanf(buf, "SERVO,%f", &x.a) == 1) {
        x.type = SERVO;
        return true;
      }
      if (!strcmp(buf, "STOP")) {
        x.type = STOP;
        return true;
      }
      if (!strcmp(buf, "STATUS")) {
        x.type = STATUS;
        return true;
      }
      // GET_SETTINGS: devolve SETTINGS,... com o que esta em vigor. Existe
      // porque um ajuste pelo menu do OLED nao deixava rastro no Pi: em
      // 24/09/2026 um SERVO TRIM "salvo" nao tinha sido salvo, e so deu
      // para descobrir medindo o efeito com varredura.
      if (!strcmp(buf, "GET_SETTINGS")) {
        x.type = GET_SETTINGS;
        return true;
      }
      if (!strcmp(buf, "SAVE_SETTINGS")) {
        x.type = SAVE;
        return true;
      }
      // MODE_STATE,<n>: o Pi confirmando qual modo esta de fato ativo.
      if (sscanf(buf, "MODE_STATE,%f", &x.a) == 1) {
        x.type = MODE_STATE;
        return true;
      }
      if (sscanf(buf, "CONTROL_STATE,%f", &x.a) == 1) {
        x.type = CONTROL_STATE;
        return true;
      }
      // SET_INFO,<slot>,<texto livre com espacos>
      if (!strncmp(buf, "SET_INFO,", 9)) {
        const char* p = buf + 9;
        const char* virgula = strchr(p, ',');
        if (virgula && virgula > p) {
          x.slot = (uint8_t) atoi(p);
          strncpy(x.text, virgula + 1, sizeof(x.text) - 1);
          x.text[sizeof(x.text) - 1] = 0;
          x.type = INFO;
          return true;
        }
      }
      if (sscanf(buf, "SET_PID,%f,%f,%f", &x.a, &x.b, &x.c) == 3) {
        x.type = PID;
        return true;
      }
      return false;
    }

    if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }

  return false;
}