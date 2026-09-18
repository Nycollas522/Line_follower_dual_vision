# Dados de corrida

Saída do `scripts/corrida.py`: percepção das duas câmeras, comando,
atuação (`/odom`, `/wheel_states`) e bateria, amostrados juntos.

| arquivo | o que é |
|---|---|
| `corrida_20260910_193825.csv` | 3 min, só a inferior nos papéis de direção |
| `corrida_20260910_203230.csv` | 3 min, fechamento, com a mira nova da cabeça |

São estas duas que sustentam o **limite de quebra de 57,8°** citado no
README: toda quebra acima dele travou nas duas corridas (n=6), e nenhuma
abaixo travou na primeira.

**Ao derivar taxas destes CSVs, reamostre a 20 ms primeiro.** O arquivo
grava a ~123 Hz e o controle roda a 50 Hz; derivar linha a linha infla a
aceleração — deu 1.485 m/s² onde o valor real era 0.857.
