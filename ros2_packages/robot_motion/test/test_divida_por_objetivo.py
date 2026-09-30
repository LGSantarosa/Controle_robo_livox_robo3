"""A dívida das rés é POR OBJETIVO (30-09).

`res_seguidas` e `dist_antes_da_re` são a contabilidade da fuga (12-08):
enquanto uma ré não trouxe o robô mais perto do que ele estava antes dela, o
rearme paga o teto cheio de 4 s. As duas só zeravam no `__init__` do nó.

🔴 O DEFEITO, medido no loop de 30-09 (`20260930_144318-web`): ao chegar num
objetivo, `dist_antes_da_re` ficava com poucos centímetros; na perna seguinte,
o primeiro escape punha `res_seguidas = 1` e a dívida só "pagava" chegando mais
perto daqueles centímetros DO OBJETIVO NOVO — ou seja, no fim da perna. A perna
inteira ficava no teto de 4 s, e o gatilho rápido da 063 nunca disparou em
corrida: os 15 gatilhos do loop foram com ~4,1 s, os dois STOPs da porta 2
manobraram em 5,48 e 5,05 s, com o portão de mapa respondendo "parede" ali.

Distância até um objetivo não se compara com distância até outro. O zero vai no
mesmo ramo do `cb_plano` que já impede herdar a trava de rota da missão
anterior (`mesmo_objetivo`, fim do plano a mais de 0,30 m).
"""
import os
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'robot_motion'))

from robot_motion.path_follower import PathFollower     # noqa: E402


class LoggerFalso:
    def warn(self, msg, **kwargs):
        pass

    def info(self, msg, **kwargs):
        pass


def _mapa_com_ombreira(x, y, resolucao=0.05, largura=120, altura=120):
    """Parede mapeada a 0,40 m ao LADO de (x, y): a porta 2."""
    dados = [0] * (largura * altura)
    dados[int((y + 0.40) / resolucao) * largura + int(x / resolucao)] = 100
    return NS(header=NS(frame_id='odom'), data=dados,
              info=NS(width=largura, height=altura, resolution=resolucao,
                      origin=NS(position=NS(x=0.0, y=0.0),
                                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))


class _P:
    def __init__(self, x, y):
        self.position = NS(x=x, y=y)
        self.orientation = NS(x=0.0, y=0.0, z=0.0, w=1.0)


def _msg(pontos):
    return NS(poses=[NS(pose=_P(x, y)) for x, y in pontos],
              header=NS(frame_id='odom'))


class SeguidorFalso:
    """O que `cb_plano` e `teto_de_emperramento` tocam — e nada mais."""

    def __init__(self):
        self.par = {'timeout_plano': 2.0, 'aceita_plano_cru': False,
                    're_parado_s': 4.0, 're_parado_s_mapeado': 2.0,
                    're_mapeado_alcance': 0.50, 're_mapeado_raio_perto': 0.6,
                    're_mapeado_vizinhanca': 0.22, 're_mapeado_limiar': 65,
                    'avanco_para_choque': 0.0825}
        self.t = 100.0
        self.plano = []
        self.plano_frame = None
        self.t_plano = self.t_plano_suave = None
        self.aceita_replano = True
        self.passagem_ativa = None
        self.passagem_fase = ''
        self.rumo_objetivo = None
        self.res_sem_plano = 0
        self.estado = 'seguindo'
        self.progresso = NS(reinicia=lambda: None)
        self.mapa = _mapa_com_ombreira(1.0, 1.0)
        self.pose = NS(header=NS(frame_id='odom'))
        self.tf_buffer = None
        # A dívida deixada pela CHEGADA ao objetivo anterior: uma ré na perna,
        # e a melhor distância encolhida até a tolerância de chegada.
        self.res_seguidas = 1
        self.dist_antes_da_re = 0.2

    def agora(self):
        return self.t

    def get_logger(self):
        return LoggerFalso()

    def tem_objetivo(self):
        return True

    def atualiza_passagens(self):
        pass

    def bloqueio_mapeado(self, x, y, rumo):
        return PathFollower.bloqueio_mapeado(self, x, y, rumo)


def _plano(s, fim, suave=True):
    PathFollower.cb_plano(s, _msg([(0.0, 0.0), (0.5, 0.0), fim]), suave=suave)


def _teto_na_porta(s):
    return PathFollower.teto_de_emperramento(s, 1.0, 1.0, 0.0)


def _com_plano_do_objetivo_A():
    s = SeguidorFalso()
    s.plano = [(0.0, 0.0), (0.5, 0.0), (5.0, 5.0)]      # A = (5, 5)
    return s


def test_objetivo_novo_zera_a_divida():
    s = _com_plano_do_objetivo_A()
    _plano(s, (9.0, 1.0))                               # B, longe de A
    assert s.res_seguidas == 0
    assert s.dist_antes_da_re is None


def test_replano_do_MESMO_objetivo_preserva_a_divida():
    """A proteção de 12-08 continua inteira dentro de um objetivo: replano do
    Nav2 para o mesmo fim (≤ 0,30 m) não é objetivo novo."""
    s = _com_plano_do_objetivo_A()
    s.aceita_replano = True                  # replano aceito, não só recusado
    _plano(s, (5.2, 5.1))                    # 0,22 m do fim anterior
    assert s.res_seguidas == 1
    assert s.dist_antes_da_re == pytest.approx(0.2)


def test_replano_recusado_do_mesmo_objetivo_preserva_a_divida():
    s = _com_plano_do_objetivo_A()
    s.aceita_replano = False                 # rota travada: o ramo que retorna cedo
    _plano(s, (5.0, 5.0))
    assert s.res_seguidas == 1
    assert s.dist_antes_da_re == pytest.approx(0.2)


def test_na_porta_do_objetivo_novo_o_teto_e_o_rapido():
    """O fim da cadeia: com a dívida velha o teto na porta era 4 s mesmo com a
    ombreira mapeada; trocado o objetivo, o portão de mapa volta a valer."""
    s = _com_plano_do_objetivo_A()
    assert _teto_na_porta(s) == pytest.approx(4.0), 'o antes: dívida herdada'
    _plano(s, (9.0, 1.0))
    assert _teto_na_porta(s) == pytest.approx(2.0)


def test_objetivo_novo_em_area_livre_segue_com_o_teto_cheio():
    """Zerar a dívida não encurta nada sozinho: sem parede mapeada, 4 s."""
    s = _com_plano_do_objetivo_A()
    s.mapa = _mapa_com_ombreira(5.0, 5.0)    # parede longe da pose (1, 1)
    _plano(s, (9.0, 1.0))
    assert _teto_na_porta(s) == pytest.approx(4.0)
