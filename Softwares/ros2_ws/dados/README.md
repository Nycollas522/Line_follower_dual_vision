# Dados de corrida

Saída do `scripts/corrida.py`: percepção das duas câmeras, comando,
atuação (`/odom`, `/wheel_states`) e bateria, amostrados juntos.

| arquivo | o que é |
|---|---|
| `corrida_20260910_193825.csv` | 3 min, só a inferior nos papéis de direção |
| `corrida_20260910_203230.csv` | 3 min, fechamento, com a mira nova da cabeça |

| `corrida_20260924_112344.csv` | 45 s, antes da memória de freio: freio do preview soltava 12-16 cm antes da quina, 2 perdas em 5 |
| `corrida_20260924_120518.csv` | 3 min, memória de freio (renovada também pela curvatura): 74% do tempo segurando, vx 0,087, 1º mapa fechado (1,5 cm) |
| `corrida_20260924_130437.csv` | 3 min, perfil SUAVE (antes de existir o perfil), trim do servo no firmware |
| `corrida_20260924_132102.csv` | 3 min, MÉDIA; mostra a curvatura-ruído da superior prendendo o robô em v_min nas retas |
| `corrida_20260924_132830.csv` | 3 min, MÉDIA com a memória só pelo heading: vx 0,124, volta 76 s |
| `corrida_20260924_133532.csv` | 3 min, RÁPIDA (0,12/0,44): vx 0,168, volta 58 s, 8 recuperações < 0,8 s |

As de 24/09 gravam **uma linha por quadro da câmera inferior** (~21-27 Hz)
e têm `x_m`, `y_m`, `yaw_deg` (pose do firmware, para `tools/mapa.py`) e,
a partir de 13:21, `perfil` (0 SUAVE, 1 MÉDIA, 2 RÁPIDA).

São estas duas que sustentam o **limite de quebra de 57,8°** citado no
README: toda quebra acima dele travou nas duas corridas (n=6), e nenhuma
abaixo travou na primeira.

**Ao derivar taxas destes CSVs, reamostre a 20 ms primeiro.** O arquivo
grava a ~123 Hz e o controle roda a 50 Hz; derivar linha a linha infla a
aceleração — deu 1.485 m/s² onde o valor real era 0.857.
