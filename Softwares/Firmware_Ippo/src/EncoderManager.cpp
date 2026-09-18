#include "EncoderManager.h"

constexpr pcnt_unit_t EncoderManager::units[Config::N];

void EncoderManager::begin() {
  for (uint8_t i = 0; i < Config::N; i++) {
    pinMode(Config::ENCA[i], INPUT_PULLUP);
    pinMode(Config::ENCB[i], INPUT_PULLUP);

    // DECODIFICACAO x4. Antes so o canal 0 contava, e so na borda de
    // SUBIDA de ENCA (pos_mode=INC, neg_mode=DIS) -- uma contagem por
    // ciclo de quadratura. Agora os dois canais contam as duas bordas:
    // ENCA com ENCB como direcao, e ENCB com ENCA como direcao.
    //
    // POR QUE (18/09/2026): com x1 e janela de 10 ms a velocidade so
    // podia valer multiplos de 0.074 m/s, e o robo anda a 0.07-0.20.
    // Uma roda a 0.106 m/s da 1.43 ticks por janela; abaixo de 1 ela
    // lia ZERO girando normalmente. x4 leva a quantizacao a 0.0186.
    //
    // ticksRev precisa acompanhar (330 -> 1320). A migracao de Settings
    // faz isso multiplicando o valor gravado por 4 -- assim qualquer
    // calibracao fina que ja existisse e preservada.
    pcnt_config_t config = {};
    config.unit = units[i];
    config.counter_h_lim = 32767;
    config.counter_l_lim = -32768;
    config.lctrl_mode = PCNT_MODE_REVERSE;
    config.hctrl_mode = PCNT_MODE_KEEP;

    config.channel = PCNT_CHANNEL_0;
    config.pulse_gpio_num = Config::ENCA[i];
    config.ctrl_gpio_num = Config::ENCB[i];
    config.pos_mode = PCNT_COUNT_DEC;
    config.neg_mode = PCNT_COUNT_INC;
    ESP_ERROR_CHECK(pcnt_unit_config(&config));

    config.channel = PCNT_CHANNEL_1;
    config.pulse_gpio_num = Config::ENCB[i];
    config.ctrl_gpio_num = Config::ENCA[i];
    config.pos_mode = PCNT_COUNT_INC;
    config.neg_mode = PCNT_COUNT_DEC;
    ESP_ERROR_CHECK(pcnt_unit_config(&config));
    // 1000 ciclos de APB (80 MHz) = 12.5 us. Com x4 na velocidade
    // maxima (0.7 m/s) as bordas ficam a ~66 us uma da outra, entao o
    // filtro continua com folga de 5x.
    ESP_ERROR_CHECK(pcnt_set_filter_value(units[i], 1000));
    ESP_ERROR_CHECK(pcnt_filter_enable(units[i]));
    ESP_ERROR_CHECK(pcnt_counter_pause(units[i]));
    ESP_ERROR_CHECK(pcnt_counter_clear(units[i]));
    ESP_ERROR_CHECK(pcnt_counter_resume(units[i]));
  }
  reset();
}

void EncoderManager::reset() {
  portENTER_CRITICAL(&mux);
  for (uint8_t i = 0; i < Config::N; i++) {
    c[i] = 0;
    pcnt_counter_pause(units[i]);
    pcnt_counter_clear(units[i]);
    pcnt_counter_resume(units[i]);
  }
  portEXIT_CRITICAL(&mux);
}

void EncoderManager::read(int32_t v[Config::N]) {
  portENTER_CRITICAL(&mux);
  for (uint8_t i = 0; i < Config::N; i++) {
    int16_t count = 0;
    pcnt_get_counter_value(units[i], &count);
    c[i] += static_cast<int32_t>(count) * Config::ENC_SIGN[i];
    pcnt_counter_pause(units[i]);
    pcnt_counter_clear(units[i]);
    pcnt_counter_resume(units[i]);
    v[i] = c[i];
  }
  portEXIT_CRITICAL(&mux);
}