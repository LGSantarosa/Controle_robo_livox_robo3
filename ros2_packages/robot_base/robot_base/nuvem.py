"""Empacotar pontos do Livox como `PointCloud2` — lógica pura, sem ROS.

Está separada do nó (`nuvem_pontos.py`) pelo padrão da casa: aqui mora o que
precisa ser testável na máquina de dev, onde as mensagens do
`livox_ros_driver2` **não existem** (o pacote só é compilado no NUC).

## Por que este módulo existe

Medido no robô em 10-08:

    /livox/lidar   publisher:  livox_ros_driver2/msg/CustomMsg
    costmaps, collision_monitor e o pré-voo assinam  sensor_msgs/PointCloud2

O FAST-LIO funciona porque lê CustomMsg — por isso a localização ia bem e a
percepção era ZERO, com a nuvem medida a 9,96 Hz o tempo todo. No simulador o
lidar é um `gpu_lidar` que publica PointCloud2 no MESMO nome de tópico, então a
decisão 014 passou lá e nunca foi exercitada de verdade: **o nome batia e o
contrato não**.
"""
import math
import struct

# x, y, z, intensidade — todos float32. Quatro campos de 4 bytes.
#
# `intensity` não é usada por nenhum consumidor nosso (costmap e reflexo só
# querem geometria), mas entra porque é o layout que as ferramentas de
# visualização esperam encontrar, e porque um ponto de 16 bytes fica alinhado.
PASSO_PONTO = 16
CAMPOS = (
    # (nome, deslocamento, datatype FLOAT32 = 7, quantidade)
    ('x', 0, 7, 1),
    ('y', 4, 7, 1),
    ('z', 8, 7, 1),
    ('intensity', 12, 7, 1),
)

_PONTO = struct.Struct('<ffff')


def ponto_valido(x, y, z, raio_cego=0.0):
    """O Mid-360 emite (0,0,0) quando o raio não volta.

    ⚠️ Deixar esses pontos passar não é ruído inocente: (0,0,0) no frame do
    sensor é o PRÓPRIO ROBÔ. A camada de obstáculo marcaria célula letal em
    cima dele a cada quadro, e o planejador passaria a recusar todo caminho por
    estar "dentro" de um obstáculo — com a nuvem aparentemente perfeita.

    ⚠️ **E a origem exata não era o único ponto do próprio robô** — medido no
    robô em 12-08, com ele PARADO no meio da sala e o `collision_monitor`
    disparando 928 vezes:

        0 a 2 pontos por quadro, sempre no MESMO lugar, no frame do sensor:
            x ≈ +0,04 m   y ≈ +0,09 m   z ≈ 0,00 a 0,05 m
            raio horizontal 0,10 m       (`min_points` do reflexo é 2)

    É peça do próprio robô a 10 cm do Mid-360 e na altura dele. Como oscilava
    entre 1 e 2 pontos, o reflexo ligava e desligava sozinho o tempo todo, e
    durante a primeira navegação autônoma **vetou 26 dos 156 comandos (17%)**,
    cada veto zerando o comando por ~0,1 s dentro de uma malha com 0,94 s de
    tempo morto. Ver a decisão 027.

    Por isso `raio_cego` corta um CILINDRO em torno do eixo do sensor, e não uma
    esfera: o corpo do robô fica ABAIXO do lidar, e o que o cega é a estrutura
    ao longo de todo o z, não uma vizinhança da origem.

    ⚠️ **O custo, e ele é real**: obstáculo de verdade a menos de `raio_cego`
    do eixo do sensor fica invisível. Com 0,15 m isso cai inteiramente dentro
    do chassi (o polígono de parada vai de −0,28 a +0,49 em x e ±0,26 em y), ou
    seja, para haver algo ali ele já teria de estar encostado no robô. Subir
    este número troca segurança por silêncio.
    """
    if x == 0.0 and y == 0.0 and z == 0.0:
        return False
    return math.hypot(x, y) >= raio_cego


def empacota(pontos, raio_cego=0.0):
    """`pontos` iterável de (x, y, z, intensidade) -> (bytes, quantos, descartados).

    Devolve o buffer no layout de `CAMPOS`, já sem os pontos inválidos. Conta os
    descartados porque essa contagem é diagnóstico: nuvem que chega com metade
    dos pontos zerados é sensor sujo ou obstruído, e isso tem de aparecer no
    log em vez de virar silêncio.

    `raio_cego` em metros: pontos a menos disso do EIXO do sensor são do próprio
    robô e saem junto (ver `ponto_valido` e a decisão 027). Zero mantém o
    comportamento anterior — é o que se usa para MEDIR se a peça que estava ao
    lado do sensor era mesmo a culpada.
    """
    buf = bytearray()
    n = 0
    fora = 0
    for x, y, z, i in pontos:
        if not ponto_valido(x, y, z, raio_cego):
            fora += 1
            continue
        buf += _PONTO.pack(x, y, z, i)
        n += 1
    return bytes(buf), n, fora


def custommsg_cru(raw):
    """Bytes CDR de um `livox_ros_driver2/CustomMsg` -> (seg, nseg, frame, pontos).

    `pontos` é um array estruturado do numpy que APONTA para `raw` (sem cópia),
    com os campos do `CustomPoint`. Existe por causa da decisão 066, medido no
    notebook do robô 3 em 30-09: o rclpy leva **83,5 ms** para desserializar uma
    nuvem de 20 064 pontos em objetos Python, com 100 ms de orçamento a 10 Hz.
    O nó saturava uma thread e entregava 7,2 Hz com 0,69 s de atraso.

    Layout CDR (little-endian, alinhamento contado a partir do fim dos 4 bytes
    de encapsulamento):

        int32 sec, uint32 nanosec, string frame_id (uint32 tamanho com o NUL),
        [alinha 8] uint64 timebase, uint32 point_num, uint8 lidar_id,
        uint8[3] rsvd, uint32 tamanho da sequência, e então os pontos:
        uint32 offset_time, float32 x y z, uint8 reflectivity tag line
        = 19 bytes + 1 de preenchimento (o próximo uint32 alinha em 4) = 20.
    """
    import numpy as np
    if len(raw) < 4 or raw[0:2] != b'\x00\x01':
        raise ValueError(
            f'CDR não é little-endian (encapsulamento {bytes(raw[0:4])!r})')
    base = 4

    def alinha(o, n):
        return base + ((o - base + n - 1) // n) * n

    seg, nseg, tam = struct.unpack_from('<iII', raw, base)
    o = base + 12
    frame = bytes(raw[o:o + tam - 1]).decode('utf-8')
    o = alinha(o + tam, 8) + 8               # timebase
    o += 4 + 1 + 3                           # point_num, lidar_id, rsvd
    o = alinha(o, 4)
    (n,) = struct.unpack_from('<I', raw, o)
    o += 4
    if len(raw) < o + n * _PONTO_LIVOX.itemsize - 1:
        raise ValueError(f'CDR curto: {n} pontos não cabem em {len(raw)} bytes')
    # O ÚLTIMO ponto não leva o byte de preenchimento: a mensagem real acaba 1
    # byte antes de n*20 (401 327 bytes medidos, não 401 328).
    pontos = np.ndarray(shape=(n,), dtype=_PONTO_LIVOX,
                        buffer=_completa(raw, o, n))
    return seg, nseg, frame, pontos


def _completa(raw, o, n):
    """O trecho dos pontos com o preenchimento do último garantido."""
    fim = o + n * 20
    trecho = memoryview(raw)[o:fim]
    if len(trecho) == n * 20:
        return trecho
    return bytes(trecho) + b'\x00' * (n * 20 - len(trecho))


def empacota_np(pontos, raio_cego=0.0):
    """Mesmo contrato de `empacota`, sobre o array de `custommsg_cru`.

    Tem de dar os MESMOS bytes que `empacota` (o teste confere): o critério de
    `ponto_valido` é refeito em float64, como o `math.hypot` faz com os float32
    promovidos a float do Python.
    """
    import numpy as np
    x = pontos['x'].astype(np.float64)
    y = pontos['y'].astype(np.float64)
    z = pontos['z'].astype(np.float64)
    ok = ~((x == 0.0) & (y == 0.0) & (z == 0.0))
    ok &= np.hypot(x, y) >= raio_cego
    sai = np.empty((int(ok.sum()), 4), dtype='<f4')
    sai[:, 0] = pontos['x'][ok]
    sai[:, 1] = pontos['y'][ok]
    sai[:, 2] = pontos['z'][ok]
    sai[:, 3] = pontos['reflectivity'][ok]
    return sai.tobytes(), len(sai), len(pontos) - len(sai)


def _dtype_livox():
    import numpy as np
    return np.dtype({
        'names': ['offset_time', 'x', 'y', 'z', 'reflectivity', 'tag', 'line'],
        'formats': ['<u4', '<f4', '<f4', '<f4', 'u1', 'u1', 'u1'],
        'offsets': [0, 4, 8, 12, 16, 17, 18],
        'itemsize': 20,
    })


try:
    _PONTO_LIVOX = _dtype_livox()
except ImportError:  # sem numpy só a versão antiga funciona
    _PONTO_LIVOX = None


def desempacota(buf):
    """Volta de bytes para tuplas — existe para o teste conferir o que foi
    escrito, e para diagnóstico à mão."""
    return [_PONTO.unpack_from(buf, o)
            for o in range(0, len(buf) - PASSO_PONTO + 1, PASSO_PONTO)]
