# 066 — O `nuvem_pontos` é o gargalo do Nav2 no notebook do robô 3: vetorizar

**Data**: 2026-09-30 (noite; lab, robô na mesa, só o lidar ligado)
**Status**: causa **medida e confirmada** no robô; conserto **escrito e
provado offline** (§6) e **no robô** (§7): gargalo resolvido. Abertos: o
heartbeat do `collision_monitor` e o STOP piscando (§7), que NÃO são CPU.
**Toca**: `ros2_packages/robot_base/robot_base/nuvem_pontos.py` (só o laço de
conversão) e o teste dele.
**Não toca**: tópicos, campos, frame, raio cego (decisão 027), launch.
**Dados**: `docs/dados/2026-09-30-robo3-cpu/` (`resumo.txt` primeiro).

---

## 1. O sintoma

Primeira subida do Nav2 no robô 3 real (sessão `nav2_20260930_174129`, mapa
`maps/andar3_robo3/`): os nós ativaram, o AMCL nasceu em (0, 0), três
objetivos da web falharam no planejamento com *"Costmap timed out waiting for
update"*, os filtros descartaram nuvem *"earlier than all the data in the
transform cache"*, e em ~1 min o `lifecycle_manager` perdeu o `map_server` e
derrubou a pilha. **O robô não andou.** Load 27 num i5-7200U (2 núcleos
físicos, 4 threads).

## 2. A medição (robô parado na mesa, placa desligada, lidar ligado)

`pidstat -u -h -l 1 60` e `mpstat -P ALL 1 60`, uma camada por etapa.
Uso somado sobre 400 %:

| etapa | total | quem pesa |
|---|---|---|
| E0 nada de ROS | 58 % | gnome-shell 29, gnome-control-center 11, systemd-logind 7 |
| E1 `localizacao` | 193 % | **`nuvem_pontos` 99**, fastlio 35, driver Livox 28 |
| E2 + `sobe-robo3 mux:=false` | 196 % | controle ≈ 3 |
| E3 + `pilha.launch.py` (sem objetivo) | 254 % | `nuvem_pontos` 92; Nav2 ≈ 50 no total |

Na E3, parada e sem web, o `collision_monitor` ficou 4 s sem heartbeat e o
`lifecycle_manager` derrubou a pilha de novo. A E4 (+ web) não foi medida.

Depois, só a localização, as duas pontas do nó (`ros2 topic hz/delay`):

| tópico | taxa | atraso |
|---|---|---|
| `/livox/lidar` (entrada, 20 064 pontos/msg) | 10,0 Hz | — |
| `/livox/pontos` (saída) | **7,2 Hz** | **0,69 s** |
| `/Odometry` (referência) | 10,0 Hz | 0,02 s |

## 3. A causa

O `nuvem_pontos` é Python de uma thread, e 99 % é o teto de uma thread: ele
satura já na E1, antes do Nav2 existir. O laço quente é o `passo()`, que
percorre `msg.points` objeto por objeto (`(p.x, p.y, p.z, …) for p in
msg.points`) — o rclpy materializa 20 mil objetos de mensagem por nuvem, 10
vezes por segundo. Sai 72 % das nuvens, com 0,7 s de atraso. Tudo que vê
obstáculo bebe dessa saída (`/scan`, os dois costmaps, `collision_monitor`),
o que explica os três sintomas de uma vez — inclusive o `/scan` a 4–5 Hz
observado no SLAM da mesma tarde.

## 4. Alternativas

| | ganho | custo | veredito |
|---|---|---|---|
| **A. Vetorizar o `passo()` com numpy** sobre a mensagem crua (`raw=True`, layout do `CustomPoint` como `dtype` estruturado) | deve cair de ~100 % para poucos % | um arquivo; saída conferível byte a byte contra a atual | **escolhida** |
| B. Reescrever o nó em C++ | o maior | pacote novo, build, mais superfície | só se A não bastar |
| C. Tirar o nó e dar aos costmaps a nuvem do FAST-LIO (`/cloud_registered_body`) | tira ~1 núcleo inteiro | frame `body` fora da árvore TF, perde o raio cego da 027 | pede decisão própria |
| D. Driver publicando PointCloud2 (`xfer_format 0`) | tira o nó | o FAST-LIO do Mid-360 lê `CustomMsg` | descartada |

Cortes menores, **fora desta decisão**, para medir depois de A, um por vez:
sessão gráfica (≈ 45 % parado), `ros2 bag record --all-topics` da pilha.

## 5. Como A vai ser provada

1. Offline: nuvem `CustomMsg` sintética com 20 064 pontos (o bag da E3 saiu
   vazio — pilha morta antes de gravar), incluindo pontos dentro do raio cego
   e inválidos; `data` da versão nova **idêntico** ao da atual; tempo por
   nuvem medido nas duas.
2. Robô na mesa, só o lidar: repetir a tabela do §2 (`/livox/pontos` a
   10 Hz, atraso da ordem do `/Odometry`) e a E3.
3. Só então Nav2 com rodas no chão.

## 6. O que foi provado offline (30-09, noite)

O perfil no notebook mudou o alvo: numa nuvem de 20 064 pontos, o rclpy leva
**83,5 ms só para desserializar** o `CustomMsg` em objetos Python, e o
`empacota` antigo 11,7 ms. Vetorizar só o laço não bastaria. Por isso o nó
passou a assinar com `raw=True` e o `nuvem.py` ganhou `custommsg_cru` (lê o
CDR direto num array estruturado do numpy) e `empacota_np`.

- Suíte do `robot_base`: 132/0, com 5 testes novos (CDR montado à mão com
  `struct`, frame de vários tamanhos, nuvem vazia, big-endian recusado,
  mesmos bytes que o caminho antigo com raio 0 e 0,15).
- No notebook, contra o CDR **do próprio rclpy** (401 327 bytes — o último
  ponto vem sem o byte de preenchimento): saída **idêntica byte a byte** à
  antiga com raio 0 e 0,15; `sec`, `nanosec` e `frame_id` iguais.
- Tempo por nuvem, mesmo notebook: **92,3 ms → 1,0 ms** (92×).
  (`docs/dados/2026-09-30-robo3-cpu/nuvem_pontos_066_offline.txt`)

## 7. Medido no robô depois do conserto (30-09, 18h10; mesa, placa desligada)

`docs/dados/2026-09-30-robo3-cpu/depois_066/`.

| | antes | depois |
|---|---|---|
| `/livox/pontos` | 7,2 Hz, atraso 0,69 s | **10,0 Hz, atraso 0,12 s** (o carimbo é o início da varredura de 100 ms) |
| `nuvem_pontos` | 92–99 % | **4 %** |
| E1 (localização), uso total | 56 % | **34 %** |
| E3 (+ controle + pilha), soma por processo | 254 % de 400 | **186 % de 400**, 45 % ocioso |

**O gargalo de CPU acabou. Dois problemas sobraram, e não são CPU:**

1. O `lifecycle_manager` **ainda** declarou o `collision_monitor` fora do ar
   (4 s sem heartbeat) 14 s depois de ativar — com a máquina 45 % ociosa e o
   processo vivo, publicando. A pilha se reergueu sozinha 7 s depois. Causa
   não investigada.
2. Parado na mesa, o `collision_monitor` alternou **"stop due to PolygonStop"
   / "continue" 220 vezes** em ~100 s. Na mesa pode ser o entorno (gente,
   objetos a menos de 0,5 m), mas é o mesmo padrão do defeito de 12-08 que
   deu origem à 027 (peça do robô piscando dentro do polígono). Precisa ser
   olhado no chão, com a nuvem gravada.
