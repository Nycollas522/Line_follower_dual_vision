# Observacao geometrica da linha, produzida por UM no de percepcao a partir
# de UM frame. Mensagem atomica: erro, confianca e validade viajam juntos,
# para que o controlador nunca combine o erro de um frame com o status de
# outro (essa era a fonte de comandos espurios na versao anterior).
#
# Convencao de sinais (valida para as duas cameras):
#   +x = direita do robo     -> lateral_error > 0 = linha a DIREITA
#   +heading = linha inclina para a direita conforme se afasta
#   +curvature = a pista curva para a DIREITA
# O controlador converte para ROS (wz > 0 = girar a esquerda) invertendo
# o sinal uma unica vez, em line_controller.py.

std_msgs/Header header        # stamp = stamp do frame da camera (nao o de agora)

bool valid                    # false = nao ha linha utilizavel neste frame
float32 confidence            # 0.0 (nada) .. 1.0 (deteccao completa e coerente)

float32 lateral_error         # m, linha - eixo do robo, na banda mais proxima
float32 heading_error         # rad, inclinacao local da linha
float32 curvature             # 1/m, curvatura estimada (0 se bandas insuficientes)

float32 lookahead_distance    # m, distancia a frente do robo da banda usada como erro
float32 line_width            # m, largura medida da linha (plausibilidade)

uint8 bands_valid             # bandas com deteccao aceita
uint8 bands_total             # bandas amostradas
bool heading_valid            # false quando so houve 1 banda: nao ha orientacao
bool curvature_valid          # false quando ha menos bandas que o minimo do ajuste

# Residuo RMS do ajuste de RETA feito sobre as bandas, em metros.
# Grande quando ha uma QUEBRA dentro da janela (reta -> reta anguladas),
# ~zero quando a linha e reta ou de curvatura suave -- inclusive com o
# robo TORTO, que e o caso que o heading sozinho nao distingue.
# Serve para frear antes da quebra sem frear a toa so por desalinhamento.
# false quando ha menos de 3 bandas: com 2 pontos a reta passa exata e o
# residuo seria sempre zero.
float32 fit_residual
bool residual_valid

float32 contrast              # 0..255, separacao linha/fundo (diagnostico de luz)
float32 processing_time       # s, custo de CPU deste frame
float32 pipeline_latency      # s, do stamp da imagem ate o fim do processamento
