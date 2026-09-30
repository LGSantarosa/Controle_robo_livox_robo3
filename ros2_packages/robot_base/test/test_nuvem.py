"""A ponte CustomMsg -> PointCloud2 (decisão 017).

Testa a parte pura: o empacotamento. O nó em si não é testável aqui — as
mensagens do `livox_ros_driver2` só existem onde o driver foi compilado, que é
o NUC. É por isso que a lógica mora em `nuvem.py` e não dentro do nó.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'robot_base'))

from nuvem import (CAMPOS, PASSO_PONTO, desempacota,  # noqa: E402
                   empacota, ponto_valido)


def test_o_ponto_volta_como_entrou():
    dados, n, fora = empacota([(1.0, 2.0, 3.0, 40.0)])
    assert (n, fora) == (1, 0)
    assert len(dados) == PASSO_PONTO
    assert desempacota(dados) == [(1.0, 2.0, 3.0, 40.0)]


def test_o_layout_e_o_que_os_CAMPOS_prometem():
    """O consumidor lê o buffer pelos offsets declarados em `fields`. Se o
    empacotamento e a declaração discordarem, a nuvem chega com x no lugar de y
    e ninguém percebe — ela continua "uma nuvem"."""
    dados, _, _ = empacota([(7.0, 8.0, 9.0, 10.0)])
    for nome, desloc, datatype, quantos in CAMPOS:
        assert datatype == 7 and quantos == 1, nome     # FLOAT32, um por campo
        (v,) = struct.unpack_from('<f', dados, desloc)
        assert v == {'x': 7.0, 'y': 8.0, 'z': 9.0, 'intensity': 10.0}[nome]
    assert CAMPOS[-1][1] + 4 == PASSO_PONTO, 'sobrou ou faltou byte no ponto'


def test_o_ponto_ZERADO_e_DESCARTADO():
    """O Mid-360 emite (0,0,0) quando o raio não volta, e (0,0,0) no frame do
    sensor é o PRÓPRIO ROBÔ. Deixar passar marcaria célula letal em cima dele a
    cada quadro, e o planejador recusaria todo caminho por estar "dentro" de um
    obstáculo — com a nuvem parecendo perfeita."""
    assert not ponto_valido(0.0, 0.0, 0.0)
    dados, n, fora = empacota([(0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 5.0)])
    assert (n, fora) == (1, 1)
    assert desempacota(dados) == [(1.0, 0.0, 0.0, 5.0)]


def test_zero_em_UM_eixo_nao_e_invalido():
    """Só o ponto inteiro zerado é no-return. Um ponto em (0, 3, 0) é uma
    parede exatamente ao lado do robô — descartar isso apagaria obstáculo real
    no eixo do sensor."""
    assert ponto_valido(0.0, 3.0, 0.0)
    _, n, fora = empacota([(0.0, 3.0, 0.0, 1.0)])
    assert (n, fora) == (1, 0)


def test_nuvem_inteira_invalida_nao_explode():
    """Sensor tampado: zero ponto e a contagem contando. O nó publica nuvem
    vazia em vez de parar de publicar — costmap sem quadro fica NÃO-CURRENT e
    PARA de atualizar, que é falha silenciosa (10-08)."""
    dados, n, fora = empacota([(0.0, 0.0, 0.0, 0.0)] * 5)
    assert (dados, n, fora) == (b'', 0, 5)


def test_o_tamanho_do_buffer_bate_com_a_contagem():
    """`row_step` e `width` saem daqui. Buffer maior que `width × point_step`
    faz o consumidor ler lixo no fim; menor, ele lê fora do buffer."""
    pontos = [(float(i), 1.0, 2.0, 3.0) for i in range(1, 101)]
    dados, n, fora = empacota(pontos)
    assert (n, fora) == (100, 0)
    assert len(dados) == n * PASSO_PONTO


# ---------------------------------------------------------------------------
# O raio cego — decisão 027. Os números vêm do robô real (12-08), não de
# suposição: com ele PARADO, 0 a 2 pontos por quadro caíam dentro do polígono
# de parada, sempre em x ≈ +0,04 · y ≈ +0,09 · z ≈ 0,00–0,05 no frame do
# sensor, e o `collision_monitor` disparou 928 vezes.
# ---------------------------------------------------------------------------

PONTO_DO_ROBO = (0.045, 0.102, 0.019)      # medido, raio horizontal 0,112 m


def test_o_ponto_do_proprio_robo_medido_em_12_08_sai_com_o_raio_padrao():
    """O caso que produziu a decisão 027. Com 0,15 m ele tem de sair; se este
    teste passar a falhar, o reflexo volta a disparar com o robô parado."""
    x, y, z = PONTO_DO_ROBO
    assert not ponto_valido(x, y, z, 0.15)
    _, n, fora = empacota([(x, y, z, 1.0)], raio_cego=0.15)
    assert (n, fora) == (1 - 1, 1)


def test_o_mesmo_ponto_PASSA_com_raio_zero():
    """`raio_cego=0.0` é o comportamento de antes de 12-08, e existe para
    MEDIR: com a peça arrancada do robô, este ponto tem de sumir da nuvem
    sozinho. Se sumir com 0,0, a causa está confirmada por intervenção."""
    x, y, z = PONTO_DO_ROBO
    assert ponto_valido(x, y, z, 0.0)
    _, n, fora = empacota([(x, y, z, 1.0)], raio_cego=0.0)
    assert (n, fora) == (1, 0)


def test_o_raio_e_um_CILINDRO_e_nao_uma_esfera():
    """O corpo do robô fica ABAIXO do lidar: o que cega é estrutura ao longo de
    todo o z. Um ponto perto do eixo tem de sair mesmo estando longe em z —
    esfera deixaria passar exatamente a coluna que atrapalha."""
    assert not ponto_valido(0.05, 0.05, -2.0, 0.15)     # longe em z, perto do eixo
    assert ponto_valido(0.50, 0.00, 0.01, 0.15)         # longe do eixo, perto em z


def test_obstaculo_de_verdade_continua_visivel():
    """O custo do filtro tem limite: a 0,20 m do eixo — ainda dentro do
    polígono de parada, que vai a 0,49 m em x — o ponto PASSA. Raio que
    engolisse isto trocaria segurança por silêncio."""
    assert ponto_valido(0.20, 0.0, 0.10, 0.15)
    assert ponto_valido(0.0, -0.26, 0.10, 0.15)


def test_a_origem_exata_sai_mesmo_com_raio_zero():
    """A regra da 017 não pode ter sido substituída pela da 027: (0,0,0) é o
    raio que não voltou e sai sempre, inclusive com o filtro desligado."""
    assert not ponto_valido(0.0, 0.0, 0.0, 0.0)
    _, n, fora = empacota([(0.0, 0.0, 0.0, 0.0)], raio_cego=0.0)
    assert (n, fora) == (0, 1)


# ── decisão 066: a leitura CRUA do CustomMsg, vetorizada ────────────────────

def _cdr(pontos, frame='livox_frame', seg=1790801684, nseg=838874289):
    """CustomMsg em CDR montado à mão, independente do parser. Sem o byte de
    preenchimento do último ponto, como o rclpy serializa (401 327 bytes
    medidos no robô para 20 064 pontos)."""
    b = bytearray(b'\x00\x01\x00\x00')
    f = frame.encode() + b'\x00'
    b += struct.pack('<iII', seg, nseg, len(f)) + f
    while (len(b) - 4) % 8:
        b += b'\x00'
    b += struct.pack('<QIB3s', 123, len(pontos), 0, b'\x00\x00\x00')
    while (len(b) - 4) % 4:
        b += b'\x00'
    b += struct.pack('<I', len(pontos))
    for k, (x, y, z, r) in enumerate(pontos):
        b += struct.pack('<IfffBBB', k, x, y, z, r, 0, k % 4) + b'\x00'
    return bytes(b[:-1]) if pontos else bytes(b)


def _nuvem_de_teste(n=2000):
    import random
    rnd = random.Random(66)
    pts = [(rnd.uniform(-8, 8), rnd.uniform(-8, 8), rnd.uniform(-1, 2),
            rnd.randrange(256)) for _ in range(n)]
    # os casos que o filtro decide: origem, eixo, borda do raio, zero num eixo
    pts += [(0.0, 0.0, 0.0, 7), (0.04, 0.09, 0.02, 9), (0.05, 0.05, -2.0, 1),
            (0.15, 0.0, 0.3, 2), (0.0, -0.26, 0.1, 3), (0.2, 0.0, 0.0, 4)]
    return pts


def _f32(pts):
    """Os mesmos pontos depois de passar por float32, como chegam do sensor."""
    return [struct.unpack('<fff', struct.pack('<fff', x, y, z)) + (float(r),)
            for x, y, z, r in pts]


def test_066_o_cabecalho_sai_do_cdr():
    from nuvem import custommsg_cru
    seg, nseg, frame, pontos = custommsg_cru(_cdr(_nuvem_de_teste(3)))
    assert (seg, nseg, frame) == (1790801684, 838874289, 'livox_frame')
    assert len(pontos) == 9


def test_066_mesmos_bytes_que_o_caminho_antigo():
    """A vetorização não pode mudar a nuvem: com e sem o raio cego da 027."""
    from nuvem import custommsg_cru, empacota_np
    pts = _nuvem_de_teste()
    _, _, _, arr = custommsg_cru(_cdr(pts))
    for raio in (0.0, 0.15):
        assert empacota_np(arr, raio) == empacota(_f32(pts), raio)


def test_066_nuvem_vazia():
    from nuvem import custommsg_cru, empacota_np
    _, _, _, arr = custommsg_cru(_cdr([]))
    assert empacota_np(arr, 0.15) == (b'', 0, 0)


def test_066_frame_de_tamanho_qualquer_nao_desalinha():
    """O alinhamento de 8 do timebase depende do tamanho do frame_id."""
    from nuvem import custommsg_cru, empacota_np
    pts = _nuvem_de_teste(5)
    for frame in ('', 'a', 'livox', 'livox_frame_x', 'x' * 30):
        _, _, f, arr = custommsg_cru(_cdr(pts, frame=frame))
        assert f == frame
        assert empacota_np(arr, 0.15) == empacota(_f32(pts), 0.15)


def test_066_cdr_big_endian_e_recusado():
    import pytest
    from nuvem import custommsg_cru
    with pytest.raises(ValueError):
        custommsg_cru(b'\x00\x00\x00\x00' + _cdr([])[4:])
