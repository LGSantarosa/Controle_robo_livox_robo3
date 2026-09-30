"""O contorno do robô para o desenho do mapa no web (30-09).

O web desenhava um quadrado fixo de 0,5 m centrado no `base_link`, que é o
robô 1. O robô 3 é mais estreito e NÃO é centrado: o `base_link` fica no eixo
das motrizes, perto da frente, e o corpo vai quase todo para trás.

A fonte é a MESMA do Nav2 e do reflexo: `robot_base/config/geometria_robo3.yaml`,
cujo polígono é conferido contra o URDF pelo `test_urdf_robo3.py`. O cabeçalho
dele manda: "os vértices NÃO se redigitam em outro lugar". Por isso este módulo
lê o arquivo instalado, e não copia números.

Sem o arquivo (outro robô, `install/` ausente, YAML quebrado), devolve `None` e
o `map.js` cai no quadrado antigo. O desenho é só visualização: faltar não pode
derrubar o web.
"""
import math
import os

import yaml

ARQUIVO = 'geometria_robo3.yaml'


def caminho_instalado():
    """Onde o `colcon build` pôs a geometria, ou `None` sem o ament."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('robot_base'), 'config', ARQUIVO)
    except Exception:
        return None


def poligono_valido(p):
    """Lista de >=3 vértices [x, y] finitos, em metros no frame `base_link`."""
    if not isinstance(p, list) or len(p) < 3:
        return False
    for v in p:
        if not isinstance(v, (list, tuple)) or len(v) != 2:
            return False
        for c in v:
            if isinstance(c, bool) or not isinstance(c, (int, float)) or not math.isfinite(c):
                return False
    return True


def carrega(caminho=None):
    """O polígono do `footprint` como [[x, y], ...] em float, ou `None`."""
    caminho = caminho or caminho_instalado()
    if not caminho:
        return None
    try:
        with open(caminho, encoding='utf-8') as f:
            dado = yaml.safe_load(f)
        p = dado['footprint']['poligono']
    except (OSError, yaml.YAMLError, KeyError, TypeError):
        return None
    if not poligono_valido(p):
        return None
    return [[float(x), float(y)] for x, y in p]
