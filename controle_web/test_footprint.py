"""O desenho do robô no mapa do web usa o contorno REAL (30-09).

Antes: quadrado fixo de 0,5 m centrado no `base_link` (o robô 1). Agora: o
polígono da `geometria_robo3.yaml`, a mesma fonte do Nav2 e do reflexo. A
função de desenho do `map.js` roda no node, como no `test_ui_route_edit.py`.
"""
import json
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

import footprint

RAIZ = Path(__file__).resolve().parents[1]
GEOMETRIA = RAIZ / 'ros2_packages' / 'robot_base' / 'config' / footprint.ARQUIVO
PERFIL = RAIZ / 'ros2_packages' / 'robot_motion' / 'config' / 'perfil_robo3.yaml'
MAPJS = RAIZ / 'controle_web' / 'static' / 'js' / 'map.js'
APP = RAIZ / 'controle_web' / 'app.py'
NODE = shutil.which('node') or shutil.which('nodejs')
precisa_node = pytest.mark.skipif(NODE is None, reason='node ausente (ex.: a Pi)')


# ---- a carga: mesma fonte do Nav2 -------------------------------------------

def test_le_o_mesmo_arquivo_que_o_perfil_nav2_do_robo3():
    """Fonte única: o perfil do robô 3 aponta a geometria que o Nav2 recebe, e
    é esse o arquivo que o web lê. Trocar um sem o outro reprova aqui."""
    perfil = yaml.safe_load(PERFIL.read_text(encoding='utf-8'))
    assert perfil['geometria']['arquivo'] == footprint.ARQUIVO


def test_carrega_o_poligono_da_geometria():
    p = footprint.carrega(str(GEOMETRIA))
    esperado = yaml.safe_load(GEOMETRIA.read_text(encoding='utf-8'))['footprint']['poligono']
    assert p == [[float(x), float(y)] for x, y in esperado]
    assert all(type(c) is float for v in p for c in v)


def test_o_robo3_nao_e_centrado_no_base_link():
    """O motivo do conserto: o quadrado centrado punha metade do robô à frente
    do eixo, e o robô 3 tem o corpo quase todo para trás dele."""
    p = footprint.carrega(str(GEOMETRIA))
    xs = [v[0] for v in p]
    ys = [v[1] for v in p]
    assert max(xs) < abs(min(xs)), 'a frente devia ser mais curta que a traseira'
    assert max(ys) - min(ys) < 0.5, 'mais estreito que o quadrado de 0,5 m do robô 1'


def test_sem_arquivo_devolve_none(tmp_path):
    assert footprint.carrega(str(tmp_path / 'nao_existe.yaml')) is None


@pytest.mark.parametrize('conteudo', [
    'footprint: {}\n',
    'outra: 1\n',
    'footprint: {poligono: [[0.1, 0.1], [0.1, -0.1]]}\n',            # 2 vértices
    'footprint: {poligono: [[0.1, 0.1], [0.1, -0.1], [.nan, 0.1]]}\n',
    'footprint: {poligono: [[0.1, 0.1], [0.1, -0.1], [true, 0.1]]}\n',
    'footprint: {poligono: [[0.1, 0.1], [0.1, -0.1], ["a", 0.1]]}\n',
    'footprint: {poligono: [[0.1, 0.1, 0], [0.1, -0.1], [0, 0.1]]}\n',
    'footprint: [\n',                                                   # YAML quebrado
])
def test_poligono_invalido_devolve_none(tmp_path, conteudo):
    f = tmp_path / 'g.yaml'
    f.write_text(conteudo)
    assert footprint.carrega(str(f)) is None


# ---- a função de desenho, a REAL, no node -----------------------------------

def _funcao(nome):
    src = MAPJS.read_text(encoding='utf-8')
    i = src.index(f'function {nome}(')
    j = src.index('{', i)
    nivel = 0
    for k in range(j, len(src)):
        if src[k] == '{':
            nivel += 1
        elif src[k] == '}':
            nivel -= 1
            if nivel == 0:
                return src[i:k + 1]
    raise AssertionError(f'{nome}: chaves não fecham')


def _poligono_do_robo(entrada_js):
    js = _funcao('poligonoDoRobo') + \
        f'\nconsole.log(JSON.stringify(poligonoDoRobo({entrada_js})));'
    r = subprocess.run([NODE, '-e', js], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


QUADRADO = [[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]


@precisa_node
def test_desenha_o_poligono_do_servidor():
    p = footprint.carrega(str(GEOMETRIA))
    assert _poligono_do_robo(json.dumps(p)) == p


@precisa_node
@pytest.mark.parametrize('entrada', [
    'null', 'undefined', '[]', '[[0.1, 0.1], [0.1, -0.1]]',
    '[[0.1, 0.1], [0.1, -0.1], [NaN, 0.1]]', '[[0.1, 0.1], [0.1, -0.1], ["0.1", 0.1]]',
    '"texto"',
])
def test_sem_poligono_valido_cai_no_quadrado_antigo(entrada):
    assert _poligono_do_robo(entrada) == QUADRADO


# ---- fiação (roda sem node) --------------------------------------------------

def test_o_servidor_emite_e_o_mapa_escuta():
    app = APP.read_text(encoding='utf-8')
    assert 'ROBOT_FOOTPRINT = footprint.carrega()' in app
    assert "emit('robot_footprint', {'poligono': ROBOT_FOOTPRINT})" in app
    js = MAPJS.read_text(encoding='utf-8')
    assert "socket.on('robot_footprint'" in js
    assert 'const poli = poligonoDoRobo(robotFootprint);' in js
    assert 'ROBOT_SIZE_M' not in js, 'sobrou o quadrado fixo do robô 1'
