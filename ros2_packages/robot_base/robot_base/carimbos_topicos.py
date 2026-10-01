#!/usr/bin/env python3
"""Grava em CSV a chegada e o `header.stamp` de cada mensagem da localização.

    ros2 run robot_base carimbos_topicos \
        --ros-args -p saida:=$HOME/carimbos.csv

## Por que isto existe

Em 30-09 o FAST-LIO divergiu no chão às 18:25:44, antes do snap (066 §8.2,
nota de 01-10), e às 18:33 recebeu nuvens 11–15 s atrasadas da IMU. Para ver o
desalinhamento LiDAR–IMU é preciso o carimbo dos DOIS lados, e o da nuvem pelo
`ros2 topic` desserializa os 20 mil pontos (83,5 ms por nuvem, decisão 066).

Aqui as quatro assinaturas são `raw=True` e só o `Header` é lido
(`robot_base.carimbos.header_cru`). Não é custo zero: o DDS ainda entrega e
copia a mensagem inteira. **Só para a rodada de diagnóstico**, nunca na de CPU
e nunca no bringup (`localizacao.launch.py`).

## O CSV

    topico, seq, chegada_mono_ns, chegada_ros_ns, recebido_dds_ns,
    publicado_dds_ns, stamp_ns, atraso_s, atraso_dds_s, espera_s, frame_id,
    bytes

`seq` é a contagem local por tópico (o `Header` do ROS 2 não tem sequência).
`chegada_*` é o instante do CALLBACK; `recebido_dds_ns` e `publicado_dds_ns`
vêm do `message_info` do RMW. Com isso o atraso se separa em duas partes:
`atraso_dds_s` (do `header.stamp` até o DDS receber) e `espera_s` (do DDS até
o callback, que é a fila deste coletor sob contenção). `atraso_s` é a soma.
Ver `robot_base.carimbos.linha_csv`. Escrita bufferizada, esvaziada a cada
`flush_s` segundos e no fim, nunca por mensagem.

Fila de 5 nas nuvens (~400 kB cada: filas de 100 reteriam ~80 MB e esticariam
artificialmente um backlog) e de 100 nos tópicos leves (a IMU chega a 200 Hz).
"""

import csv
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from robot_base.carimbos import COLUNAS, header_cru, linha_csv

FILA_NUVEM = 5
FILA_LEVE = 100


def _qos(fila):
    """Melhor esforço: casa com publicador confiável e com o de sensor."""
    q = QoSProfile(depth=fila)
    q.reliability = qos_profile_sensor_data.reliability
    q.durability = qos_profile_sensor_data.durability
    q.history = qos_profile_sensor_data.history
    return q


class CarimbosTopicos(Node):
    def __init__(self):
        super().__init__('carimbos_topicos')
        p = self.declare_parameters('', [
            ('saida', ''),
            ('lidar', '/livox/lidar'),
            ('pontos', '/livox/pontos'),
            ('imu', '/livox/imu'),
            ('odometria', '/Odometry'),
            ('flush_s', 5.0),
        ])
        self.par = {x.name: x.value for x in p}

        saida = self.par['saida'] or os.path.expanduser(
            time.strftime('~/carimbos_%Y%m%d_%H%M%S.csv'))
        os.makedirs(os.path.dirname(os.path.abspath(saida)), exist_ok=True)
        self.arq = open(saida, 'w', newline='', buffering=1 << 20)
        self.csv = csv.writer(self.arq)
        self.csv.writerow(COLUNAS)

        tipos = [(self.par['pontos'], PointCloud2, FILA_NUVEM),
                 (self.par['imu'], Imu, FILA_LEVE),
                 (self.par['odometria'], Odometry, FILA_LEVE)]
        # Importado AQUI pelo mesmo motivo do `nuvem_pontos`: o driver Livox só
        # existe onde foi compilado. Sem ele, os outros três seguem gravando.
        try:
            from livox_ros_driver2.msg import CustomMsg
            tipos.insert(0, (self.par['lidar'], CustomMsg, FILA_NUVEM))
        except ImportError as e:  # noqa: BLE001
            self.get_logger().error(
                f"sem `livox_ros_driver2.msg` ({e}): {self.par['lidar']} NÃO "
                'será gravado')

        self.seq = {}
        self.recusadas = {}
        for topico, tipo, fila in tipos:
            self.seq[topico] = 0
            self.recusadas[topico] = 0
            # DOIS argumentos obrigatórios: é pela assinatura que o rclpy
            # decide entregar o `message_info` (com um só, ele não entrega).
            self.create_subscription(
                tipo, topico,
                lambda raw, info, t=topico: self.chegou(t, raw, info),
                _qos(fila), raw=True)

        self.create_timer(self.par['flush_s'], self.arq.flush)
        self.create_timer(10.0, self.resumo)
        self.get_logger().warn(
            f'carimbos_topicos: {", ".join(t[0] for t in tipos)} -> {saida}. '
            'Diagnóstico: o DDS ainda copia cada nuvem; não usar na rodada de '
            'CPU.')

    def chegou(self, topico, raw, info):
        mono = time.monotonic_ns()
        ros = self.get_clock().now().nanoseconds
        try:
            seg, nseg, frame = header_cru(raw)
        except ValueError as e:
            self.recusadas[topico] += 1
            self.get_logger().error(
                f'{topico}: CDR recusado ({e})', throttle_duration_sec=5.0)
            return
        self.seq[topico] += 1
        self.csv.writerow(linha_csv(topico, self.seq[topico], mono, ros, info,
                                    seg, nseg, frame, len(raw)))

    def resumo(self):
        self.get_logger().info(' '.join(
            f'{t}={n}' + (f'(recusadas {self.recusadas[t]})'
                          if self.recusadas[t] else '')
            for t, n in self.seq.items()))

    def fecha(self):
        self.arq.flush()
        self.arq.close()


def main():
    rclpy.init()
    no = CarimbosTopicos()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        no.fecha()
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
