# 067 — A TF `odom → camera_init` do robô 3 mora no `localizacao.launch.py`

**Data**: 2026-10-01 (PC de dev, fora do lab; robô e lidar desligados)
**Status**: implementado e coberto por teste estrutural de launch; **ainda sem
prova funcional** no Gazebo nem no robô.
**Toca**: `ros2_packages/robot_base/launch/localizacao.launch.py` (um `Node`) e
`ros2_packages/robot_base/test/test_tf_odom.py`.
**Não toca**: `bin/sobe-robo3`, `bin/sobe-robo`, `path_follower`, `tf_odom`.
**Origem**: 066 §8.1 (Nav2 no chão, 30-09, sessão em
`docs/dados/2026-09-30-robo3-nav2-chao/`).

---

## 1. O defeito

No chão, em 30-09, o objetivo foi aceito, o plano saiu e o `path_follower`
descartou tudo com *"sem TF camera_init<-map"*. O seguidor lê a pose do
`/Odometry` do FAST-LIO, cujo `header.frame_id` é `camera_init`, e nada na
árvore liga esse nome ao `odom`. **O robô não andou.**

É o mesmo defeito do robô 2 em 14-08: diagnóstico em `4467f9f`, remendo em
`ffaa2f0` (`bin/sobe-robo`, uma `static_transform_publisher --frame-id odom
--child-frame-id camera_init`). Nunca foi portado para o robô 3.

A identidade é exata: o `tf_odom` monta `odom → base_link` a partir da própria
pose do LIO, então a origem do `odom` é a origem do `camera_init`.

## 2. Onde pôr

| | prós | contras | veredito |
|---|---|---|---|
| **A. `Node` no `localizacao.launch.py`**, ao lado do `tf_odom` | quem sobe o FAST-LIO é quem nomeia o `camera_init`; vive e morre com ele; o Gazebo não é afetado (este launch só roda no hardware) | o robô 2 (`sobe-robo` → `base.launch.py` → este launch) passa a ter a mesma TF publicada duas vezes | **escolhida** |
| B. No `bin/sobe-robo3`, como no robô 2 | cópia literal do que funcionou | o `sobe-robo3` sobe RSP + Xbox + MEGA, não é dono do FAST-LIO; a TF fica no ar sem localização e o `--mata` a derruba sem derrubar o LIO; exige grupo próprio no registro, regex e renumeração | descartada |
| C. O `tf_odom` publicar também `odom → camera_init` | sem processo novo | mistura duas responsabilidades num nó testado; a TF estática é o idioma padrão do tf2 para isto | descartada |

A duplicata do robô 2 é inofensiva: dois publicadores estáticos com o mesmo
pai, filho e valores. O robô 2 não é alvo deste repo, então o `sobe-robo` não
foi mexido.

## 3. O que este remendo NÃO conserta

O seguidor usa a pose do **sensor**, não do corpo. No robô 3 o Mid-360 fica
**9,3 cm atrás** do `base_link` (URDF provisório: x −0,093, z 0,240); no robô 2
era só altura, e por isso lá não doía. Ao girar no lugar, o ponto que o
seguidor acompanha descreve um círculo de 9,3 cm de raio. **Conserto de
verdade, pendente e com decisão própria:** o seguidor ler a pose do
`base_link` por TF.

## 4. Prova

- **Feita (estrutural):** teste lendo a launch como texto, no padrão dos testes
  do `frame_da_pose`: o `static_transform_publisher` existe com
  `--frame-id odom --child-frame-id camera_init` exatos.
- **Pendente (funcional, no Gazebo):** o Gazebo publica `/Odometry` em `odom`,
  então uma corrida normal não reproduz o defeito. Injetar `camera_init` como
  frame da odometria simulada só na corrida, sem commit: sem a TF, *"sem TF
  camera_init<-map"*; com ela, o seguidor comanda. Como a simulação não sobe
  este launch, ali a TF vai na mão (o mesmo `static_transform_publisher`):
  prova o remendo, não o launch.
- **Pendente (no robô):** objetivo curto no chão; o robô anda ou não anda.
