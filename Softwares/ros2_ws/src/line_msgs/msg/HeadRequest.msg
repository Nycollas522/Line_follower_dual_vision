# Pedido de apontamento da camera superior. NAO e um comando de servo:
# quem converte pedido em angulo (com clamp e limite de taxa) e o
# head_servo_node, unico publicador de /servo/command.
#
# Qualquer no pode pedir; a politica de seguranca fica num lugar so.

std_msgs/Header header

uint8 MODE_CENTER=0           # volta ao zero e fica
uint8 MODE_HOLD=1             # segura o angulo pedido em angle_deg
uint8 MODE_SCAN=2             # varredura lenta esquerda/direita (recuperacao)
uint8 MODE_TRACK=3            # rastreia a linha na camera superior (loop
                               # proprio do head_servo_node); o angulo
                               # resultante vira sinal de bearing do corpo,
                               # lido de volta em /servo/state (padrao no
                               # seguimento normal quando habilitado)

uint8 mode
float32 angle_deg             # graus, so usado em MODE_HOLD (+ = direita)
