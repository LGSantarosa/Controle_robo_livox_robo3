#!/usr/bin/env python3
"""O `bin/sobe-robo3-web` EXECUTADO com comandos calçados (decisão 064).

Prova, sem ROS, sem Gazebo e sem Flask:
  · o contrato dos argumentos da pilha e do ambiente do web;
  · recusa sem matar nada (processo alheio, porta, SHM, árvore suja, sessão);
  · prontidão exige Nav2 + ação + TF + HTTP 200 sem EchoController;
  · derrubada por Ctrl+C, por `--mata` e por `--mata` com o wrapper morto;
  · o SHM pela decisão 061 depois do teardown.

🔴 Isolamento, como no teste do `subidas-robo3`: repositório TEMPORÁRIO com
árvore limpa de verdade; calços re-prefixados no PATH depois de cada `source`;
domínio 91, nunca o 50; SHM, estado e saída em pastas do teste; todo calço
deixa `CALCO <comando>` e o teste exige as marcas; nenhum `gz sim` real.
"""
import os
import shutil
import signal
import stat
import subprocess
import textwrap
import time

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
WRAPPER = os.path.join(RAIZ, 'bin', 'sobe-robo3-web')

COPIAS = ('bin/sobe-robo3-web', 'tools/valida_etapa6/lib.sh',
          'tools/valida_etapa6/processos.py', 'tools/subidas_robo3/shm_recupera.py')

# Processo que dorme até SIGINT/SIGTERM, anunciando o fim num arquivo. Python,
# não `trap`: lançado com `&` por shell não interativo, o SIGINT nasce ignorado.
DORME = textwrap.dedent(r'''
    import os, signal, sys, time
    d, papel, cenario = sys.argv[1], sys.argv[2], sys.argv[3]
    def fim(*_):
        shm = "SHM_DO_TESTE"
        if papel == "pilha" and "orfaos" in cenario:
            for n in ("fastrtps_port7001", "fastrtps_port7001_el",
                      "sem.fastrtps_port7001_mutex",
                      "fastrtps_port7002", "sem.fastrtps_port7002_mutex"):
                open(os.path.join(shm, n), "w").close()
        open(os.path.join(d, "encerrado_" + papel), "w").close()
        sys.exit(0)
    signal.signal(signal.SIGINT, fim)
    signal.signal(signal.SIGTERM, fim)
    while True:
        time.sleep(0.1)
''')

ROS2 = textwrap.dedent(r'''
    #!/usr/bin/env bash
    D="$CALCO_DIR"; C="$(cat "$D/cenario")"
    echo "CALCO ros2 $* | AMENT=$AMENT_PREFIX_PATH | DOM=$ROS_DOMAIN_ID | RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE | GZ=$GZ_PARTITION" >> "$D/chamadas.txt"
    case ":$AMENT_PREFIX_PATH:" in
      *:/opt/ros/jazzy:*) ;;
      *) echo "calço: ambiente do ROS não carregado" >&2; exit 1 ;;
    esac
    case "$1 $2" in
      "launch robot_motion") exec python3 "$D/dorme.py" "$D" pilha "$C" ;;
      "lifecycle get") case "$C" in *nav2_inativo*) echo "inactive [2]" ;; *) echo "active [3]" ;; esac ;;
      "action list") echo "/navigate_to_pose" ;;
      "run tf2_ros") echo "- Translation: [2.000, 5.000, 0.000]" ;;
      "node list") ;;
      *) echo "calço: comando não previsto: $*" >&2; exit 97 ;;
    esac
''').lstrip()

PY_WEB = textwrap.dedent(r'''
    #!/usr/bin/env bash
    D="$CALCO_DIR"; C="$(cat "$D/cenario")"
    echo "CALCO web $* | cwd=$PWD" >> "$D/chamadas.txt"
    env | sort > "$D/web_env.txt"
    case "$C" in
      *echo*) echo "[app] ROS2Controller falhou (x); caindo para EchoController." ;;
      *) echo "[ROS2Controller] WEB_TELEOP=off — nó vivo, SEM publicar em /web_vel" ;;
    esac
    exec python3 "$D/dorme.py" "$D" web "$C"
''').lstrip()

CURL = textwrap.dedent(r'''
    #!/usr/bin/env bash
    echo "CALCO curl $*" >> "$CALCO_DIR/chamadas.txt"
    case "$(cat "$CALCO_DIR/cenario")" in *http500*) printf 500 ;; *) printf 200 ;; esac
''').lstrip()

SS = textwrap.dedent(r'''
    #!/usr/bin/env bash
    echo "CALCO ss $*" >> "$CALCO_DIR/chamadas.txt"
    case "$(cat "$CALCO_DIR/cenario")" in
      *porta_ocupada*) echo "LISTEN 0 128 0.0.0.0:5000 0.0.0.0:*" ;;
    esac
''').lstrip()

# Como o real (medido em 28-09): só remove o conjunto que tem a trava _el.
FASTDDS = textwrap.dedent(r'''
    #!/usr/bin/env bash
    echo "CALCO fastdds $*" >> "$CALCO_DIR/chamadas.txt"
    [ "$1 $2" = "shm clean" ] || exit 97
    for el in "$SOBE_WEB_SHM"/*_el; do
      [ -e "$el" ] || continue
      base="${el%_el}"; porta="$(basename "$base")"
      rm -f "$el" "$base" "$SOBE_WEB_SHM/sem.${porta}_mutex"
    done
    echo "shm.clean:"; echo "1 zombie ports cleaned"
''').lstrip()


def _executavel(caminho, texto):
    caminho.parent.mkdir(parents=True, exist_ok=True)
    caminho.write_text(texto)
    caminho.chmod(caminho.stat().st_mode | stat.S_IXUSR)


def _gz_real():
    achados = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = f.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if 'gz sim' in argv or 'gzserver' in argv:
            achados.append(argv)
    return achados


class Bancada:
    def __init__(self, tmp_path, cenario='ok', suja=False):
        self.tmp = tmp_path
        self.repo = tmp_path / 'repo'
        for rel in COPIAS:
            destino = self.repo / rel
            destino.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(os.path.join(RAIZ, rel), destino)
        (self.repo / 'controle_web').mkdir()
        (self.repo / 'controle_web' / 'app.py').write_text('# calço\n')
        (self.repo / 'maps').mkdir()
        (self.repo / 'maps' / '.keep').write_text('')
        (self.repo / '.gitignore').write_text('install/\ncontrole_web/.venv/\n')
        git = ['git', '-C', str(self.repo), '-c', 'user.name=t', '-c', 'user.email=t@t']
        subprocess.run(git + ['init', '-q'], check=True)
        subprocess.run(git + ['add', '-A'], check=True)
        subprocess.run(git + ['commit', '-qm', 'calço'], check=True)
        if suja:
            (self.repo / 'maps' / 'novo.txt').write_text('sujo\n')
        self.shm = tmp_path / 'shm'
        self.shm.mkdir()
        # A reexecução limpa zera o ambiente: os calços não podem depender de
        # variável do teste, então nascem com os caminhos gravados.
        self.calcos = tmp_path / 'calcos'
        ponha = lambda t: t.replace('$CALCO_DIR', str(self.calcos))
        _executavel(self.calcos / 'ros2', ponha(ROS2))
        _executavel(self.calcos / 'curl', ponha(CURL))
        _executavel(self.calcos / 'ss', ponha(SS))
        _executavel(self.calcos / 'fastdds', ponha(FASTDDS))
        _executavel(self.repo / 'controle_web' / '.venv' / 'bin' / 'python', ponha(PY_WEB))
        (self.calcos / 'dorme.py').write_text(DORME.replace('SHM_DO_TESTE', str(self.shm)))
        (self.calcos / 'cenario').write_text(cenario + '\n')
        (self.repo / 'install').mkdir()
        (self.repo / 'install' / 'setup.bash').write_text(
            f'export AMENT_PREFIX_PATH="{self.repo}/install/robot_motion:$AMENT_PREFIX_PATH"\n')
        self.ros_setup = tmp_path / 'jazzy_setup.bash'
        self.ros_setup.write_text('export AMENT_PREFIX_PATH=/opt/ros/jazzy\n')
        self.saida = tmp_path / 'saida'
        self.estado = tmp_path / 'estado'
        self.wrapper = str(self.repo / 'bin' / 'sobe-robo3-web')

    def env(self):
        e = dict(os.environ)
        # O que o wrapper NÃO pode herdar: é o que a reexecução limpa descarta.
        e['AMENT_PREFIX_PATH'] = '/home/x/Controle_robo_web/install/prefixo_sujo'
        e['PYTHONPATH'] = '/home/x/prefixo_sujo'
        e.update(SOBE_WEB_CALCOS=str(self.calcos), SOBE_WEB_RAIZ_SAIDA=str(self.saida),
                 SOBE_WEB_DOMINIO='91', SOBE_WEB_SHM=str(self.shm),
                 SOBE_WEB_ROS_SETUP=str(self.ros_setup), SOBE_WEB_ESTADO=str(self.estado),
                 SOBE_WEB_PRAZO='8')
        e.pop('VALIDA_ETAPA4_MARCA', None)
        return e

    def roda(self, *args, timeout=60):
        return subprocess.run([self.wrapper, *args], env=self.env(), timeout=timeout,
                              capture_output=True, text=True)

    def inicia(self):
        return subprocess.Popen([self.wrapper], env=self.env(),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def espera_pronto(self, proc, prazo=30):
        fim = time.time() + prazo
        while time.time() < fim:
            if list(self.saida.glob('*-web/resultado.txt')):
                r = self.pasta() / 'resultado.txt'
                if 'web: HTTP 200' in r.read_text() and self.estado.joinpath('sessao').exists():
                    time.sleep(0.5)
                    return
            if proc.poll() is not None:
                pytest.fail('o wrapper saiu antes de ficar pronto:\n' + proc.stdout.read())
            time.sleep(0.2)
        pytest.fail('o wrapper não ficou pronto')

    def pasta(self):
        (p,) = list(self.saida.glob('*-web'))
        return p

    def chamadas(self):
        f = self.calcos / 'chamadas.txt'
        return f.read_text() if f.exists() else ''


def _vivos_do_teste(raiz):
    """(pid, argv) de todo processo com o caminho deste teste no argv ou na marca."""
    achados = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = f.read().replace(b'\0', b' ').decode(errors='replace')
            with open(f'/proc/{pid}/environ', 'rb') as f:
                amb = f.read().decode(errors='replace')
        except OSError:
            continue
        if int(pid) != os.getpid() and (raiz in argv or f'VALIDA_ETAPA4_MARCA={raiz}' in amb):
            achados.append((int(pid), argv))
    return achados


@pytest.fixture(autouse=True)
def sem_sobra(tmp_path):
    """Todo teste termina sem processo seu vivo. Falha de teste que abandona o
    wrapper deixava a sessão de pé e envenenava os testes seguintes (30-09)."""
    yield
    raiz = str(tmp_path)
    # 5 s de saída natural (o `tee` do console fecha depois do wrapper); um
    # wrapper abandonado não sai sozinho nesse prazo.
    fim = time.time() + 5
    while time.time() < fim and _vivos_do_teste(raiz):
        time.sleep(0.2)
    sobras = _vivos_do_teste(raiz)
    for pid, argv in sobras:
        if 'sobe-robo3-web' in argv:
            os.kill(pid, signal.SIGINT)
    fim = time.time() + 40
    while time.time() < fim and _vivos_do_teste(raiz):
        time.sleep(0.2)
    for pid, _ in _vivos_do_teste(raiz):
        os.kill(pid, signal.SIGKILL)
    assert not sobras, f'o teste deixou processo vivo: {sobras}'


def _confere_derrubado(b):
    assert (b.calcos / 'encerrado_pilha').exists(), 'a pilha não recebeu o SIGINT'
    assert (b.calcos / 'encerrado_web').exists(), 'o web não recebeu o SIGINT'
    assert not (b.estado / 'sessao').exists(), 'o registro de sessão ficou'
    assert not _gz_real(), 'subiu Gazebo de verdade'


def test_sobe_e_derruba_por_mata_com_o_contrato(tmp_path):
    b = Bancada(tmp_path)
    p = b.inicia()
    b.espera_pronto(p)
    r = b.roda('--mata')
    assert r.returncode == 0, r.stdout + r.stderr
    saida, _ = p.communicate(timeout=60)
    assert p.returncode == 0, saida
    _confere_derrubado(b)
    ch = b.chamadas()
    (launch,) = [l for l in ch.splitlines() if 'CALCO ros2 launch' in l]
    for arg in ('robot_motion pilha.launch.py', 'robo:=3', 'sim:=true', 'rviz:=false',
                'localizacao:=fixa', 'freio_linear:=false', 'bag:=false',
                f'log_dir:={b.pasta()}'):
        assert arg in launch, arg
    assert 'DOM=91' in launch and 'RANGE=LOCALHOST' in launch
    assert 'GZ=sobe_robo3_web_' in launch
    # ambiente limpo: Jazzy + install deste clone, nada do prefixo sujo
    assert 'prefixo_sujo' not in ch
    assert f'{b.repo}/install/robot_motion:/opt/ros/jazzy' in launch
    web = dict(l.split('=', 1) for l in (b.calcos / 'web_env.txt').read_text().splitlines() if '=' in l)
    assert (web['ROBOT_MODE'], web['ROBOT_SIM'], web['WEB_TELEOP']) == ('nav2', 'true', 'off')
    assert web['ROS_DOMAIN_ID'] == '91'
    assert web['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert 'prefixo_sujo' not in web.get('PYTHONPATH', '')
    assert f'cwd={b.repo}/controle_web' in ch
    pasta = b.pasta()
    for f in ('git.txt', 'ambiente.txt', 'comandos.txt', 'launch.log', 'web.log',
              'shm_antes.tsv', 'shm_final.tsv', 'grupos.txt', 'teardown.env', 'shm.env',
              'console.txt', 'http.txt'):
        assert (pasta / f).exists(), f
    assert 'pronto=1' in (pasta / 'teardown.env').read_text()
    assert 'teardown_anomalo=0' in (pasta / 'shm.env').read_text()


def test_ctrl_c_derruba_tudo(tmp_path):
    b = Bancada(tmp_path)
    p = b.inicia()
    b.espera_pronto(p)
    p.send_signal(signal.SIGINT)
    saida, _ = p.communicate(timeout=60)
    assert p.returncode == 0, saida
    _confere_derrubado(b)


def test_mata_com_o_wrapper_morto_derruba_os_grupos_registrados(tmp_path):
    b = Bancada(tmp_path)
    p = b.inicia()
    b.espera_pronto(p)
    p.kill()                                    # terminal fechado, sem finalização
    p.communicate(timeout=10)
    assert (b.estado / 'sessao').exists()
    assert not (b.calcos / 'encerrado_pilha').exists()
    r = b.roda('--mata')
    # Sessão órfã é anomalia: derruba, mas não sai 0.
    assert r.returncode == 1, r.stdout + r.stderr
    _confere_derrubado(b)


def test_orfaos_de_shm_seguem_a_061(tmp_path):
    b = Bancada(tmp_path, cenario='orfaos')
    p = b.inicia()
    b.espera_pronto(p)
    b.roda('--mata')
    saida, _ = p.communicate(timeout=60)
    assert p.returncode == 0, saida
    assert not list(b.shm.iterdir()), 'sobrou segmento no shm de teste'
    shm = (b.pasta() / 'shm.env').read_text()
    assert 'shm_fast_dds_depois_do_teardown=5' in shm
    assert 'teardown_anomalo=1' in shm and 'limpeza_manual_recuperada=1' in shm
    assert 'shm_fast_dds_final=0' in shm
    assert 'CALCO fastdds shm clean' in b.chamadas()
    assert 'removidos (2)' in (b.pasta() / 'shm_remove.txt').read_text()


def test_web_em_echocontroller_nao_fica_pronto(tmp_path):
    b = Bancada(tmp_path, cenario='echo')
    p = b.inicia()
    saida, _ = p.communicate(timeout=90)
    assert p.returncode == 1, saida
    assert 'EchoController' in (b.pasta() / 'resultado.txt').read_text()
    _confere_derrubado(b)


def test_http_diferente_de_200_nao_fica_pronto(tmp_path):
    b = Bancada(tmp_path, cenario='http500')
    p = b.inicia()
    saida, _ = p.communicate(timeout=120)
    assert p.returncode == 1, saida
    assert (b.pasta() / 'http.txt').read_text().strip() == '500'
    _confere_derrubado(b)


def test_nav2_inativo_nao_sobe_o_web(tmp_path):
    b = Bancada(tmp_path, cenario='nav2_inativo')
    p = b.inicia()
    saida, _ = p.communicate(timeout=90)
    assert p.returncode == 1, saida
    assert 'CALCO web' not in b.chamadas()
    assert (b.calcos / 'encerrado_pilha').exists()
    assert not (b.estado / 'sessao').exists()


@pytest.mark.parametrize('cenario,frase', [
    ('porta_ocupada', 'porta 5000 está ocupada'),
])
def test_recusa_sem_subir_nada(tmp_path, cenario, frase):
    b = Bancada(tmp_path, cenario=cenario)
    r = b.roda()
    assert r.returncode == 1
    assert frase in r.stdout
    assert 'CALCO ros2' not in b.chamadas()
    assert not (b.estado / 'sessao').exists()


def test_recusa_arvore_suja(tmp_path):
    b = Bancada(tmp_path, suja=True)
    r = b.roda()
    assert r.returncode == 1 and 'não commitada' in r.stdout
    assert 'CALCO ros2' not in b.chamadas()


def test_recusa_shm_de_antes(tmp_path):
    b = Bancada(tmp_path)
    (b.shm / 'fastrtps_port9999').write_text('')
    r = b.roda()
    assert r.returncode == 1 and 'segmento Fast DDS' in r.stdout
    assert (b.shm / 'fastrtps_port9999').exists(), 'mexeu no SHM de antes'
    assert 'CALCO ros2' not in b.chamadas()


def test_recusa_processo_ros_alheio_e_nao_o_mata(tmp_path):
    b = Bancada(tmp_path)
    alheio = subprocess.Popen(['bash', '-c', 'exec -a /opt/ros/jazzy/lib/no_alheio sleep 60'],
                              start_new_session=True)
    try:
        time.sleep(0.3)
        r = b.roda()
        assert r.returncode == 1 and 'já há processo' in r.stdout
        assert alheio.poll() is None, 'matou processo que não era dele'
        assert 'CALCO ros2' not in b.chamadas()
    finally:
        alheio.kill()
        alheio.wait()


def test_recusa_segunda_sessao(tmp_path):
    b = Bancada(tmp_path)
    p = b.inicia()
    b.espera_pronto(p)
    try:
        # A 2ª instância vê o registro da 1ª e recusa antes de criar pasta.
        # 1 s de espera: no mesmo segundo a trava da PASTA respondia primeiro.
        time.sleep(1.1)
        r = b.roda()
        assert r.returncode == 1, r.stdout
        assert 'há sessão registrada' in r.stdout and 'nada foi subido' in r.stdout
        assert b.chamadas().count('CALCO ros2 launch') == 1
    finally:
        b.roda('--mata')
        p.communicate(timeout=60)
    _confere_derrubado(b)


def test_mata_sem_sessao(tmp_path):
    b = Bancada(tmp_path)
    r = b.roda('--mata')
    assert r.returncode == 0 and 'nada para derrubar' in r.stdout


@pytest.mark.parametrize('como', ['pipe_fechado', 'sighup'])
def test_terminal_fechado_ainda_derruba_ate_o_fim(tmp_path, como):
    """Terminal fechado: o `tee` e o pipe somem. O wrapper tem de terminar o
    teardown inteiro (grupos, SHM, registro), não só começar."""
    b = Bancada(tmp_path, cenario='orfaos')
    p = b.inicia()
    b.espera_pronto(p)
    if como == 'pipe_fechado':
        p.stdout.close()
        p.send_signal(signal.SIGINT)
    else:
        p.send_signal(signal.SIGHUP)
    p.wait(timeout=60)
    _confere_derrubado(b)
    assert 'shm_fast_dds_final=0' in (b.pasta() / 'shm.env').read_text()
    assert (b.pasta() / 'teardown.env').exists()


@pytest.mark.parametrize('calcos', ['', '/tmp/x'])
def test_calcos_devolve_zero_com_e_sem_calco(calcos):
    """1ª subida real (30-09): sem calço, o `calcos` devolvia 1 e, como último
    comando do `carrega_ros`, derrubava o wrapper sem mensagem. Os testes
    calçados nunca passavam por aí — este passa, e não sobe nada."""
    with open(WRAPPER) as f:
        (linha,) = [l for l in f if l.startswith('calcos()')]
    r = subprocess.run(['bash', '-c', linha + 'calcos; echo "rc=$?"'],
                       env={'PATH': '/usr/bin:/bin', 'SOBE_WEB_CALCOS': calcos},
                       capture_output=True, text=True)
    assert r.stdout.strip() == 'rc=0', r.stdout + r.stderr
