# 065 — O robô 3 real liberado pelo dono: SLAM e Nav2, sem parada física

**Data**: 2026-09-30 (fim da tarde; PC de dev, robô e lidar DESLIGADOS)
**Status**: aplicada em código e testada offline. A primeira subida no robô real
é o roteiro do lab de 30-09 (`ESTADO_PROJETO.md`, topo). **Nada medido no
robô ainda.**
**Toca**: `pilha.launch.py` (argumentos `libera_real` e `v_max`),
`controle_robo3.launch.py` (argumento `mux`), `bin/sobe-robo3` (um tópico a mais
no bag), `tools/linha_de_base/argumentos_launch.py` (os três permitidos),
`test_pilha_robo3.py` e `test_controle_robo3_launch.py`.
**Não toca**: nenhum default. Sem os argumentos novos, todo comando antigo sobe
igual, e `robo:=3 sim:=false` continua recusado.

---

## 1. A decisão, e de quem é

O dono, a caminho do lab: *"quero testar ele se movendo no nav2 hoje, se foda,
quero fazer o mapa com slam e depois usar o mapa pra usar o nav2"*. Mais cedo
ele tinha escolhido "Xbox + LIO hoje", e mudou de ideia com o quadro de risco
na mesa. O quadro, dito a ele antes:

- a pilha recusava o robô 3 real de propósito (etapa 6): faltavam as duas
  pontas da cadeia, o atuador e a localização;
- a **057** estava declarada bloqueadora para hardware (061 §2.3.2, regra do
  próprio dono de 28-09), e o SIGSEGV dela apareceu duas vezes em 30-09;
- **não existe parada física** independente do Xbox (resposta do dono), e o
  `PLANO_NAV2_ROBO3.md` §8 a exige para toda etapa com rodas no chão;
- a pose do Livox no URDF ainda é provisória (etapa 7 não medida), e a
  curvatura, a escala e a zona morta do robô 3 real não foram medidas (etapa 8).

**Liberar é decisão do dono, com esse risco conhecido.** O que cabe aqui é
fazer a liberação ser deliberada, estreita e reversível.

## 2. O que mudou

1. **`libera_real:=true`** na `pilha.launch.py`. Sem ele, `robo:=3 sim:=false`
   segue recusado com o mesmo texto de antes, mais o caminho da liberação. É
   argumento, e não troca de default, para ninguém subir o robô real por
   acidente.
2. **`controle_robo3.launch.py mux:=false`** (pelo `bin/sobe-robo3 mux:=false`).
   Não sobe o `twist_mux` dele, e o `cmd_vel_to_wheels` passa a ouvir
   `/hoverboard_base_controller/cmd_vel`, que é a saída do `compensador_rumo` da
   pilha no modo real. Quem arbitra é o mux da pilha, que já tinha o Xbox
   (`joy_vel` 100, `dpad_vel` 110) acima da autonomia (`auto_vel` 10).
3. **`v_max:=`** opcional na pilha: teto do seguidor na primeira subida.
   Vazio, nada muda.

A localização não precisou de código: o `localizacao.launch.py` (Livox,
FAST-LIO, `tf_odom`, `/scan`) sobe à parte, e a pilha usa
`localizacao:=amcl` contra o mapa do SLAM (`robot_nav/slam.launch.py`).

## 3. O que foi provado offline

- Testes: recusa sem a liberação (a de antes, intacta), subida com ela, robô 2
  intocado com o argumento, `v_max` só quando passado, `mux:=false` sem
  `twist_mux` e com o atuador na saída da pilha, `mux` inválido morre.
- **Fumaça no PC de dev, sem hardware** (domínio 52, só localhost): a pilha no
  modo liberado, com mapa, AMCL, `v_max:=0.25` e compensador neutro, mais um
  `cmd_vel_to_wheels` avulso. Nenhum nó morreu na subida. A cadeia fechou por
  descoberta: `compensador_rumo` publica `/hoverboard_base_controller/cmd_vel`
  (`TwistStamped`) e o `cmd_vel_to_wheels` assina; o `twist_mux` assina
  `/joy_vel` e `/dpad_vel`. `v_max` 0,25 e `curv_frente` 0,0 lidos nos nós vivos.
- ⚠️ Visto e não resolvido: o `planner_server` loga *"Inflation layer either not
  found or inflation is not set sufficiently for optimized non-circular
  collision checking"* no modo real, com o `perfil_nav2.yaml` materializado
  **idêntico** ao do Gazebo, onde a mensagem não aparece. A consequência
  documentada é o Smac cair na checagem de colisão completa, que é mais lenta e
  não menos segura. Fica para investigar.

## 4. O que NÃO está provado, e o roteiro trata

- **Parada:** soltar o LB **não** para a autonomia (o mux volta para
  `auto_vel`). Frear é segurar o LB com o analógico solto, cancelar pelo web ou
  derrubar a pilha. O firmware da MEGA zera os motores 500 ms depois do último
  setpoint (`SETPOINT_TIMEOUT_MS`), e `cmd_vel_to_wheels`/`mega_bridge` só
  escrevem quando chega comando. Pela leitura do código, a pilha morta para as
  rodas em ~0,5 s. **Isso se confere com as rodas no ar antes do chão.**
- Sentido de frente e giro com a pilha (o Xbox já foi aprovado em 14/16-09):
  rodas no ar, objetivo curto à frente.
- Curvatura do robô 3: desconhecida. O roteiro passa `curv_frente:=0.0
  curv_re:=0.0`, porque os −0,817/−0,098 de default são do robô 2.

## 5. Alternativas descartadas

- **Trocar o default de `robo:=3 sim:=false` para aceito**: faria o robô real
  subir sem ninguém ter pedido. Rejeitada; o argumento explícito custa uma
  palavra.
- **Um `atuador_robo3.launch.py` novo**: duplicaria a parte de atuador do
  `controle_robo3` (MEGA, sinais, bitola, escala), que já foi aprovada no Xbox.
  Um argumento no mesmo launch reaproveita os números que andaram.
- **Manter dois muxes em série**: o do `controle_robo3` publicaria no atuador
  junto com a pilha, e seriam dois donos do comando.
