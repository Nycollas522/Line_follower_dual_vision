# logs/ — dados e ferramentas de bancada

## Corridas gravadas

`corrida_AAAAMMDD_HHMMSS.csv` — saída de `scripts/corrida.py`. Grava
percepção das DUAS câmeras, comando, atuação (`/odom`, `/wheel_states`)
e bateria. Nome com timestamp de propósito: comparar duas configurações
exige as duas corridas, e um caminho fixo já sobrescreveu uma delas.

| arquivo | o que é |
|---|---|
| `corrida_20260910_193825.csv` | 3 min, só a inferior nos papéis de direção |
| `corrida_20260910_203230.csv` | 3 min, fechamento, com a mira nova da cabeça |
| `corrida_com_superior.csv` | 45 s, primeira com a superior religada |
| `corrida_instrumentada.csv` | 45 s, primeira com as duas câmeras gravadas |

**Ao derivar taxas destes CSVs, reamostre a 20 ms primeiro.** O arquivo
grava a ~123 Hz e o controle roda a 50 Hz; derivar linha a linha infla a
aceleração (deu 1.485 m/s² onde o valor real era 0.857).

## Ferramentas

| script | para quê |
|---|---|
| `amostra.py` | leitura rápida das duas câmeras, parado. Sem mover nada. |
| `captura.py` | salva imagens cruas e de debug das duas câmeras em `~/quebra/` |
| `quebra_teste.py` | **antes/depois da mira da cabeça num disparo só** — detecções, servo e imagens no MESMO instante |
| `servo_prova.py` | prova que o servo gira a câmera, comparando quadros em ±35° |
| `varredura.py` | varre o servo por N segundos para inspeção visual do eixo |

`servo_prova.py` e `varredura.py` calam o seguidor
(`head_control_enabled: false`) e devolvem a autoridade no fim. **Todo
script que comande a cabeça precisa fazer isso** — `line_follower_node`
publica `/head/request` a 50 Hz e disputa o servo, o que já produziu um
diagnóstico de hardware falso.

`obsoletos/` guarda versões anteriores de `quebra_teste.py` que mediam em
disparos separados. Produziram leituras contraditórias porque a cena
mudava entre as capturas. Não reutilizar.
