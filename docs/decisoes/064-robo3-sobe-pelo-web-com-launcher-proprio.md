# 064 — O robô 3 sobe pelo web, com launcher próprio, e o RViz sai de cena

**Data**: 2026-09-30 (PC de dev `rbe-luis-20429`; robô e lidar DESLIGADOS, Gazebo)
**Status**: aplicada. Launcher testado offline (15 testes, mutações conferidas);
a primeira subida real por ele vem logo depois do commit
**Toca**: `bin/sobe-robo3-web` (novo), `tools/sobe_robo3_web/test_sobe_robo3_web.py`
(novo), `pytest.ini` (coleta)
**Não toca**: `bin/sobe-robo3` (é do robô FÍSICO, com Xbox e MEGA), a pilha, o
`controle_web/` (nenhuma linha), o `tools/valida_etapa6/lib.sh` e o
`tools/subidas_robo3/shm_recupera.py` (usados como estão)

---

## 1. Contexto

Até 29-09, o objetivo do robô 3 no Gazebo se mandava pelo RViz. Pedido do dono:
*"quero poder criar rotas por lá [pelo web] e não ficar refém do rviz"*. Em
30-09 a pilha subiu à mão com `rviz:=false` e o `controle_web/app.py` num venv.
Ida e volta por uma rota de 2 pontos criada no navegador, sem código novo no web
(`~/sessao-robo3/20260930_web-robo3/`). Depois dela veio a regra:

> *"agora sempre iremos subir usando a web, sem mais rviz nesse robô, ok?"*

A subida à mão levou dois scripts, seis variáveis de ambiente e um teardown feito
na mão, com os dois grupos (pilha e web) sinalizados à parte. Quem sobe o robô é
o dono, e ele não deveria precisar lembrar disso tudo.

## 2. O contrato do launcher

Definido na revisão (30-09) antes de escrever código:

| item | como está |
|---|---|
| um comando | `bin/sobe-robo3-web` sobe Gazebo + pilha + web e fica em primeiro plano |
| pilha | `robo:=3 sim:=true gui:=true rviz:=false localizacao:=fixa freio_linear:=false bag:=false log_dir:=<pasta>` |
| web | `ROBOT_MODE=nav2 ROBOT_SIM=true WEB_TELEOP=off`, Python do `controle_web/.venv` |
| rede | `ROS_DOMAIN_ID=50`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `GZ_PARTITION=sobe_robo3_web_<carimbo>` |
| ambiente | reexecução com `env -i`: só Jazzy + `install/setup.bash` DESTE clone |
| registro | `~/sessao-robo3/<carimbo>-web/`: `git.txt`, `ambiente.txt`, `comandos.txt`, `launch.log`, `web.log`, `console.txt`, SHM antes/depois/final, `teardown.env`, `shm.env` |
| recusa (nunca mata) | processo ROS/Gazebo/`app.py`, qualquer marca de rodada, segmento Fast DDS em `/dev/shm`, porta 5000 ocupada, sessão registrada, árvore suja |
| pronto | Nav2 (`bt_navigator`, `planner_server`, `controller_server`) `active`, `/navigate_to_pose`, TF `map→base_link`, HTTP 200, linha `[ROS2Controller]` no `web.log`, **nenhum** `EchoController` |
| derrubar | Ctrl+C, ou `bin/sobe-robo3-web --mata` de outro terminal. Com o wrapper morto, o `--mata` derruba pelos grupos que ele registrou |
| teardown | `lib.sh` da etapa 6: grupos com (PGID, STARTTIME) e marca, SIGINT, até 20 s de espera, SIGKILL só nos sobreviventes |
| SHM | decisão 061 §2.3.2: inventário ANTES da subida; depois `confere`, `fastdds shm clean` e `shm_recupera.py remove`; recontagem zero |

Sessão com órfão de SHM recuperado sai com **código 0**, mas marcada
(`teardown_anomalo=1`, `limpeza_manual_recuperada=1` no `shm.env`). Código 1 fica
para o que não fechou: não ficou pronto, limpeza reprovada, SHM que sobrou, ou
SIGSEGV. Todas as sessões reais até aqui deixaram órfãos. Se o órfão virasse
código 1, o código 1 viraria ruído e deixaria de avisar.

⚠️ O web escuta em `0.0.0.0:5000` (fixo no `app.py`), então fica acessível pela
rede local enquanto a sessão estiver no ar. O launcher avisa e não muda isso.

## 3. O venv

Esta máquina não tinha Flask. O venv fica em `controle_web/.venv`, ignorado pelo
git:

```bash
python3 -m venv --system-site-packages controle_web/.venv   # o rclpy do Jazzy aparece lá dentro
controle_web/.venv/bin/pip install -r controle_web/requirements.txt
```

Sem ele, o launcher recusa e aponta para esta seção.

## 4. Alternativas descartadas

- **Estender o `bin/sobe-robo3`.** Ele é do robô físico: sobe MEGA, Xbox e o
  `--mata` dele conhece aquela pilha. Misturar o Gazebo ali faria uma correção
  de um mudar o outro.
- **Estender o `launch.sh` da raiz.** É do robô 1 (mapa `hotmilk`, mundo
  `sala.sdf`, `pkill` na limpeza de órfãos). O `pkill -f` já matou uma sessão
  ssh neste projeto.
- **Copiar o `lib.sh` para dentro do launcher.** O `bin/subidas-robo3` já o
  carrega por `source`. Uma terceira cópia da mesma política de sinal seria mais
  um lugar para ela divergir.
- **Checar `ros2 node list` na pré-condição.** Ele cria um participante DDS antes
  do inventário de SHM, e a 061 já o tirou de gate. A varredura olha o `/proc`,
  as marcas, o SHM e a porta.

## 5. O que os testes acharam antes do hardware

Os testes offline (repositório temporário, calços, domínio 91) acharam três
defeitos no próprio launcher antes de qualquer subida:

1. **SIGPIPE no meio do teardown.** Com o console fechado (terminal fechado, ou o
   harness que abandonou o processo), o wrapper começava a derrubar, sinalizava
   os grupos e morria na primeira escrita. O registro de sessão, o SHM e o
   `teardown.env` ficavam pendentes. Conserto: `trap '' PIPE`. Travado por
   `test_terminal_fechado_ainda_derruba_ate_o_fim`. Sem a linha, reprova.
2. **O critério "ROS2Controller" casava com o fallback.** A mensagem de falha é
   `ROS2Controller falhou (…); caindo para EchoController`. Conserto: exigir a
   linha exata `[ROS2Controller]`, além da trava explícita do `EchoController`.
3. **Sessão viva era recusada pela trava errada.** A trava de sessão vinha depois
   da criação da pasta. Ela subiu para antes.

Mutações conferidas, cada uma reprova um teste específico: RViz ligado, freio
ligado, sem trava do EchoController, sem a remoção controlada da 061, sem trava
da porta, sem `trap '' PIPE`, sem trava de árvore suja. **A mutação que tira o
`HUP` do trap sobrevive**, porque o bash roda o `trap EXIT` também quando morre
por SIGHUP. Essa linha só dá a mensagem "interrompido" e não é carga.

## 6. Referências

- Decisão 061 §2.3.2 (remoção controlada do SHM) e `bin/subidas-robo3` (padrão
  de varredura, calços e marca).
- `docs/DIARIO.md`, 30-09: a subida à mão, as duas corridas e o tropeço do
  pipe.
