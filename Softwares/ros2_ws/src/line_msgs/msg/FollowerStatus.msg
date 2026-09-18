# Estado interno do seguidor, publicado a cada ciclo de controle.
# Existe para depuracao e para o relatorio de iniciacao cientifica:
# gravando so este topico + /line/detection ja se reproduz uma volta inteira.

std_msgs/Header header

uint8 STATE_IDLE=0            # autonomia desligada ou ainda sem primeira linha
uint8 STATE_FOLLOWING=1       # deteccao fresca e confiavel
uint8 STATE_DEGRADED=2        # deteccao fresca, confianca baixa: anda devagar
uint8 STATE_COASTING=3        # perda curta (1-3 frames): mantem curva, desacelera
uint8 STATE_RECOVERING=4      # perda confirmada: para de avancar e procura
uint8 STATE_SAFE_STOP=5       # nao recuperou dentro do limite: parado

uint8 state
string state_name

float32 cmd_vx                # m/s efetivamente publicado em /cmd_vel_auto
float32 cmd_wz                # rad/s efetivamente publicado
float32 wz_limit              # rad/s, limite cinematico ativo neste ciclo

float32 lateral_error         # m, entrada usada
float32 heading_error         # rad, entrada usada
float32 curvature             # 1/m, entrada usada
float32 confidence            # 0..1, entrada usada
float32 severity              # 0..1, quanto a velocidade foi reduzida
float32 speed_scale           # cmd_vx / v_max

float32 vision_age            # s, idade da ultima deteccao valida
float32 vision_fps            # Hz, taxa medida da camera inferior
float32 vision_latency        # s, latencia media percepcao->controle
float32 loop_dt               # s, periodo real deste ciclo de controle

bool preview_active           # camera superior contribuindo neste ciclo
float32 preview_curvature     # 1/m, curvatura antecipada pela camera superior
float32 preview_ff            # rad/s, feedforward aplicado (ja saturado)

uint32 loss_events            # perdas confirmadas desde o enable
float32 last_recovery_time    # s, duracao da ultima recuperacao bem sucedida
float32 distance_travelled    # m, odometria acumulada desde o enable
float32 mean_abs_error        # m, erro lateral absoluto medio desde o enable
