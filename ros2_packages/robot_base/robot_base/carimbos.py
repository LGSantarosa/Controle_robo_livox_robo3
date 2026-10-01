"""O `std_msgs/Header` lido direto dos bytes CDR, sem desserializar a mensagem.

Para o coletor da rodada de diagnóstico (`carimbos_topicos`, roteiro de 01-10 e
066 §8.2): `CustomMsg`, `PointCloud2`, `Imu` e `Odometry` começam todos pelo
`Header`, então o mesmo parser serve aos quatro. Pelo rclpy, ler o carimbo de
uma nuvem custaria desserializar os 20 mil pontos (83,5 ms, decisão 066).
"""

import struct

_ENCAPSULAMENTO_LE = b'\x00\x01'


def header_cru(raw):
    """CDR de uma mensagem que começa por `Header` -> (sec, nanosec, frame).

    Layout (little-endian, depois dos 4 bytes de encapsulamento; o `Header` é o
    primeiro campo, então não há preenchimento antes dele):

        int32 sec, uint32 nanosec, uint32 tamanho (conta o NUL), bytes + NUL

    Recusa com `ValueError`: big-endian (ou outro encapsulamento), mensagem
    curta, string que não cabe, tamanho zero e string sem o NUL final.
    """
    raw = memoryview(raw)
    if len(raw) < 4 or bytes(raw[0:2]) != _ENCAPSULAMENTO_LE:
        raise ValueError(
            f'CDR não é little-endian (encapsulamento {bytes(raw[0:4])!r})')
    if len(raw) < 16:
        raise ValueError(f'CDR curto: {len(raw)} bytes não têm um Header')
    seg, nseg, tam = struct.unpack_from('<iII', raw, 4)
    if tam == 0:
        raise ValueError('frame_id com tamanho 0 (o CDR conta o NUL)')
    if 16 + tam > len(raw):
        raise ValueError(
            f'CDR curto: frame_id de {tam} bytes não cabe em {len(raw)}')
    if raw[16 + tam - 1] != 0:
        raise ValueError('frame_id sem o NUL final')
    frame = bytes(raw[16:16 + tam - 1]).decode('utf-8')
    return seg, nseg, frame


COLUNAS = ['topico', 'seq', 'chegada_mono_ns', 'chegada_ros_ns',
           'recebido_dds_ns', 'publicado_dds_ns', 'stamp_ns', 'atraso_s',
           'atraso_dds_s', 'espera_s', 'frame_id', 'bytes']


def linha_csv(topico, seq, mono, ros, info, seg, nseg, frame, nbytes):
    """Uma linha do CSV do `carimbos_topicos`, na ordem de `COLUNAS`.

    `ros` é o instante do CALLBACK; `info` é o `message_info` do rclpy, com
    `received_timestamp` (o RMW recebeu) e `source_timestamp` (o publicador
    escreveu). Separa as duas esperas que se misturam sob contenção:

        atraso_dds_s = recebido − header.stamp   (publicação até o DDS)
        espera_s     = callback − recebido        (fila do próprio coletor)

    Instante 0 ou ausente (RMW que não informa) sai VAZIO, nunca como uma
    diferença enorme e falsa.
    """
    stamp = seg * 1_000_000_000 + nseg
    info = info or {}
    rec = info.get('received_timestamp') or None
    pub = info.get('source_timestamp') or None

    def s(ns):
        return '' if ns is None else f'{ns / 1e9:.6f}'

    return [topico, seq, mono, ros, rec or '', pub or '', stamp,
            s(ros - stamp), s(rec - stamp if rec else None),
            s(ros - rec if rec else None), frame, nbytes]
