"""O `header.stamp` lido direto do CDR, sem desserializar a mensagem.

Base do coletor da rodada de diagnóstico (roteiro de 01-10, 066 §8.2): os
quatro tópicos (`CustomMsg`, `PointCloud2`, `Imu`, `Odometry`) começam por
`std_msgs/Header`, então um parser só serve para todos. Ler o cabeçalho pelo
rclpy custaria desserializar a nuvem inteira — os 83,5 ms da 066.
"""

import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from robot_base.carimbos import header_cru  # noqa: E402


def _cdr_header(seg, nseg, frame, resto=b'', enc=b'\x00\x01\x00\x00'):
    f = frame.encode('utf-8') + b'\x00'
    return enc + struct.pack('<iII', seg, nseg, len(f)) + f + resto


def _custommsg(n, frame='livox_frame', seg=1790801684, nseg=838874289):
    """Mesmo layout do `_cdr` do `test_nuvem.py` (066)."""
    b = bytearray(_cdr_header(seg, nseg, frame))
    while (len(b) - 4) % 8:
        b += b'\x00'
    b += struct.pack('<QIB3s', 123, n, 0, b'\x00\x00\x00')
    b += struct.pack('<I', n)
    for k in range(n):
        b += struct.pack('<IfffBBB', k, 1.0, 2.0, 3.0, 9, 0, k % 4)
        if k < n - 1:
            b += b'\x00'
    return bytes(b)


def test_le_o_cabecalho_de_um_custommsg():
    raw = _custommsg(500)
    assert header_cru(raw) == (1790801684, 838874289, 'livox_frame')


@pytest.mark.parametrize('frame', ['', 'a', 'ab', 'abc', 'odom', 'body',
                                   'camera_init', 'livox_frame', 'x' * 64])
def test_frames_de_todo_tamanho(frame):
    """O tamanho da string muda o preenchimento do que vem DEPOIS dela."""
    raw = _cdr_header(7, 8, frame, resto=b'\x00' * 32)
    assert header_cru(raw) == (7, 8, frame)


def test_frame_vazio_tem_so_o_NUL():
    raw = _cdr_header(1, 2, '')
    assert header_cru(raw) == (1, 2, '')


def test_sec_negativo_e_int32():
    assert header_cru(_cdr_header(-1, 0, 'x'))[0] == -1


def test_big_endian_e_recusado():
    raw = _cdr_header(1, 2, 'odom', enc=b'\x00\x00\x00\x00')
    with pytest.raises(ValueError, match='little-endian'):
        header_cru(raw)


@pytest.mark.parametrize('raw', [
    b'',
    b'\x00\x01\x00\x00',
    b'\x00\x01\x00\x00' + b'\x00' * 11,
])
def test_curto_demais_e_recusado(raw):
    with pytest.raises(ValueError):
        header_cru(raw)


def test_string_maior_que_a_mensagem_e_recusada():
    raw = b'\x00\x01\x00\x00' + struct.pack('<iII', 1, 2, 999) + b'odom\x00'
    with pytest.raises(ValueError, match='curto'):
        header_cru(raw)


def test_string_sem_NUL_e_recusada():
    raw = b'\x00\x01\x00\x00' + struct.pack('<iII', 1, 2, 4) + b'odom'
    with pytest.raises(ValueError, match='NUL'):
        header_cru(raw)


def test_string_de_tamanho_zero_e_recusada():
    """O CDR do ROS 2 conta o NUL: tamanho 0 não é string válida."""
    raw = b'\x00\x01\x00\x00' + struct.pack('<iII', 1, 2, 0) + b'\x00' * 8
    with pytest.raises(ValueError):
        header_cru(raw)


def test_aceita_memoryview_e_bytearray():
    raw = _cdr_header(3, 4, 'odom')
    assert header_cru(memoryview(raw)) == (3, 4, 'odom')
    assert header_cru(bytearray(raw)) == (3, 4, 'odom')


# ---------------------------------------------------------------------------
# Contra o serializador do próprio ROS. O importorskip fica DENTRO de cada
# teste: no nível do módulo ele pularia também os testes acima (o defeito do
# `0c1b583`).
# ---------------------------------------------------------------------------

def _serializa(msg):
    ser = pytest.importorskip('rclpy.serialization')
    return ser.serialize_message(msg)


@pytest.mark.parametrize('frame', ['', 'imu', 'odom', 'livox_frame',
                                   'camera_init'])
def test_igual_ao_rclpy_imu_odometry_pointcloud2(frame):
    sensor = pytest.importorskip('sensor_msgs.msg')
    nav = pytest.importorskip('nav_msgs.msg')
    for msg in (sensor.Imu(), nav.Odometry(), sensor.PointCloud2()):
        msg.header.frame_id = frame
        msg.header.stamp.sec = 1790803572
        msg.header.stamp.nanosec = 312497612
        if isinstance(msg, nav.Odometry):
            msg.child_frame_id = 'body'
        if isinstance(msg, sensor.PointCloud2):
            msg.width = 3
            msg.data = bytes(range(37))
        assert header_cru(_serializa(msg)) == (
            1790803572, 312497612, frame), type(msg).__name__


# ---------------------------------------------------------------------------
# A linha do CSV: as duas esperas separadas (revisão de 01-10).
# ---------------------------------------------------------------------------

from robot_base.carimbos import COLUNAS, linha_csv  # noqa: E402


def test_a_linha_separa_o_atraso_do_dds_da_espera_do_callback():
    stamp = 100 * 10**9
    info = {'received_timestamp': stamp + 300_000_000,
            'source_timestamp': stamp + 100_000_000}
    linha = linha_csv('/livox/lidar', 7, 55, stamp + 1_000_000_000, info,
                      100, 0, 'livox_frame', 401327)
    d = dict(zip(COLUNAS, linha))
    assert len(linha) == len(COLUNAS)
    assert d['atraso_s'] == '1.000000'
    assert d['atraso_dds_s'] == '0.300000'
    assert d['espera_s'] == '0.700000'
    assert d['recebido_dds_ns'] == stamp + 300_000_000
    assert d['publicado_dds_ns'] == stamp + 100_000_000
    assert (d['frame_id'], d['bytes'], d['seq']) == ('livox_frame', 401327, 7)


@pytest.mark.parametrize('info', [None, {}, {'received_timestamp': 0,
                                             'source_timestamp': 0}])
def test_instante_do_rmw_ausente_sai_vazio_e_nao_negativo(info):
    d = dict(zip(COLUNAS, linha_csv('/Odometry', 1, 5, 2 * 10**9, info,
                                    1, 0, 'camera_init', 716)))
    assert d['atraso_s'] == '1.000000'
    assert d['recebido_dds_ns'] == d['publicado_dds_ns'] == ''
    assert d['atraso_dds_s'] == d['espera_s'] == ''
