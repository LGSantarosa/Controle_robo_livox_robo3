# Andar 3 — mapa do robô 3 (30-09)

Primeiro mapa desenhado pelo robô 3 em hardware: Mid-360 → FAST-LIO →
`/scan` → `slam_toolbox`, dirigido no Xbox pelo dono e salvo pelo botão
"Salvar mapa" da web (`ROBOT_MODE=slam`). Sessão no notebook:
`~/sessao-robo3/slam_20260930_172740` (logs t1–t4).

```
arquivo                      mapa_3_andar_completo.{pgm,yaml}
tamanho                      391 x 1158 células a 5 cm  (~19,6 x 57,9 m)
origem                       [-2.202, -12.239, 0]
(0, 0) do mapa               largada do SLAM = a marca de fita (sala de baixo)
```

O Nav2 nasce com `pose_x:=0 pose_y:=0 pose_yaw:=0`, então o robô tem de
estar na marca, virado para a mesma direção da largada.

⚠️ **Guardado, não validado.** A 1ª subida do Nav2 contra ele (30-09,
`nav2_20260930_174129`) não chegou a localizar: o notebook saturou a CPU
(load 27 em 4 núcleos) e os costmaps expiraram antes de o robô se mover.
