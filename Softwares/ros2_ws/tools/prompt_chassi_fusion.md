# Prompt para o agente de CAD (Fusion 360)

Contexto: robô seguidor de linha, chassi mecanum 4x4, com Raspberry Pi 5
+ placa ESP32-S3 DevKitC-1 de baixo nível, câmera inferior fixa apontada
para o chão e câmera superior em servo pan. Preciso de uma estrutura
protetora (chassi + carroceria) que cubra o robô por completo — hoje ele
anda exposto. Uso as fotos anexadas como referência de forma; as medidas
abaixo são a fonte de verdade para escala e posicionamento (foram
medidas com régua ou extraídas do firmware, não estimadas a olho).

## Requisitos funcionais (obrigatórios)

1. **O robô não pode ficar exposto.** Estrutura fechada cobrindo chassi,
   eletrônica e fiação, com teto.
2. **Peças em partes** que caibam na área útil da minha impressora:
   [PREENCHER: dimensões da mesa/bed, ex. 220x220mm, 256x256mm...]
   — desenhar já pensando em como dividir o chassi em impressões
   separadas com encaixe (não preciso de peça única).
3. **Laterais entre as rodas com painéis removíveis por imã**, para
   acesso à bateria e ao interior do robô. Os ímãs serão colados por
   mim depois de impresso — a peça precisa ter os rebaixos/bolsos para
   encaixar os ímãs (não para imprimir o ímã).
   - Diâmetro/espessura do ímã: [PREENCHER, ex. 6x2mm, 5x1mm...]
   - Quantidade de imãs por painel: [PREENCHER, ex. 2 ou 4 por porta]
4. **Teto cobrindo o robô**, com uma **mini tampa própria, também com
   ímã**, posicionada sobre a placa do ESP32-S3, alinhada aos dois
   botões físicos (BTN1 e BTN2) da placa, para poder apertá-los sem
   abrir o teto inteiro.

## Medidas confirmadas (não estimar — usar estas)

### Chassi / cinemática (do firmware, `Config.h`)
| Grandeza | Valor |
|---|---|
| Entre-eixos (distância entre eixo dianteiro e traseiro) | 176 mm (2× 88 mm) |
| Bitola (distância entre roda esquerda e direita) | 154 mm (2× 77 mm) |
| Diâmetro da roda mecanum | 78 mm |
| Chassi mecanum 4x4 (4 rodas mecanum independentes) | — |

### Câmera inferior (fixa, olhando reto para baixo)
| Grandeza | Valor |
|---|---|
| Posição em relação ao centro cinemático | 132 mm à frente |
| Posição em relação ao eixo dianteiro | 44 mm à frente |
| Orientação | reto para baixo (90°), sem inclinação |
| Sensor | módulo tipo OV5647 (câmera CSI Raspberry Pi, ~25×24 mm de placa) |

### Câmera superior (em servo pan)
| Grandeza | Valor |
|---|---|
| Sensor | módulo tipo OV5647 (mesma família da inferior) |
| Movimento | pan (giro horizontal) apenas, sem tilt |
| Faixa de giro usada em operação | aproximadamente ±45° |
| Offset mecânico já calibrado em software | -3° (não precisa ser perfeito no encaixe do horn) |
| Modelo/dimensões do micro servo | [PREENCHER, ex. SG90 / MG90S / outro] |
| Altura/posição pretendida no robô | [PREENCHER] |

### Eletrônica
| Componente | Especificação |
|---|---|
| Computador principal | Raspberry Pi 5 (85 × 56 mm, espessura ~17mm sem cooler) |
| Cooler/fan ativo no Pi 5? | [PREENCHER: sim/não — se sim, precisa ventilação no teto] |
| Placa de baixo nível | ESP32-S3-DevKitC-1 (módulo ~63,5 × 25,4mm + pinos; placa completa costuma ~69×27mm — CONFERIR com a peça física) |
| Botões físicos a acessar pela tampa | BTN1 (GPIO12) e BTN2 (GPIO14) — [PREENCHER: posição exata dos botões na placa física, em mm a partir de uma quina de referência] |
| Outros componentes visíveis na placa (para decidir se precisam de janela/abertura) | display OLED SSD1306 (I2C), LED RGB de status, buzzer — **avaliar se quer que fiquem visíveis/audíveis pela carroceria ou se ficam totalmente internos** |
| Bateria | LiPo 3S (11.1V nominal, ~12.6V carga plena) — [PREENCHER: dimensões físicas do pack, L×A×P] |
| Motores das rodas | [PREENCHER: modelo/dimensões do motor+gearbox, tipo de eixo (D-shaft?), padrão de furação de fixação] |

## Fabricação

- Método: impressão 3D FDM.
- Impressora / bico: [PREENCHER, ex. Ender 3, bico 0.4mm]
- Material pretendido: [PREENCHER, ex. PLA / PETG]
- Preferência de tolerância de encaixe entre peças: [PREENCHER, ou pedir sugestão]

## O que preciso como entrega

1. Modelo Fusion 360 paramétrico (não malha fechada), organizado em
   componentes: chassi base, teto, tampa do ESP32, painéis laterais
   removíveis (um por vão entre rodas onde fizer sentido).
2. Divisão das peças respeitando o tamanho de mesa informado acima,
   com juntas/encaixes entre as partes do chassi (indicar o método:
   encaixe por pino, rebaixo, parafuso, etc. — sugerir o mais simples
   de imprimir e montar).
3. Bolsos/rebaixos para os ímãs nos painéis laterais e na mini tampa
   do ESP32, dimensionados para o ímã informado acima, com folga de
   encaixe por atrito (não colado por padrão, mas aceitando cola).
4. Espaço interno para a bateria com acesso pelos painéis laterais
   (a bateria deve poder ser retirada por ali sem desmontar o teto).
5. Recorte/abertura no teto sobre a posição da câmera superior e do
   seu servo, permitindo o giro pan de ±45° sem interferência
   mecânica com a estrutura.
6. Abertura ou vão livre embaixo, na posição da câmera inferior
   (132mm à frente do centro, olhando para baixo), sem obstruir o
   campo de visão dela.

## Notas para o agente de visão (fotos)

As fotos anexadas mostram o robô no estado atual (exposto, sem
carroceria). Use como referência de escala as medidas físicas já
confirmadas acima (diâmetro de roda = 78mm, entre-eixos = 176mm,
bitola = 154mm) — são objetos visíveis nas próprias fotos e mais
confiáveis que qualquer estimativa visual.
