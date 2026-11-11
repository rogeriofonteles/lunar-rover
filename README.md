# Lunar Rover — ROS 2, Drake e MuJoCo

Projeto com três ambientes Docker independentes:

| Ambiente | Conteúdo | Guia |
| --- | --- | --- |
| Drake | Rover, Acrobot, LQR finito/infinito, iLQR, MPC e benchmarks STOMP | [drake/README.md](drake/README.md) |
| MuJoCo | Braço de seis eixos, LQR, iLQR e MPC com diferentes solvers | [mujoco/README.md](mujoco/README.md) |
| ROS 2 Jazzy | Workspace ROS, modelos de rovers e Gazebo | `lunar_rover/` |

O [laboratório interativo do Acrobot](drake/acrobot/control_lab/acrobot_interativo.html)
é um HTML independente, em PT-BR, com 11 desafios, seletor de clipping em ±3 N·m
ou torque ilimitado, gráficos e vídeos incorporados. A comparação de MPC não linear
usa iLQR, SQP e SLSQP com derivadas atualizadas ao longo da trajetória; as referências
MPC lineares ficam em uma visão separada.
Baixe o arquivo e abra no navegador; não precisa instalar os simuladores para explorá-lo.
O GitHub exibe o arquivo como código, portanto use o download para abri-lo localmente.
Também é possível [abrir a comparação de MPCs no GitHub Pages](https://rogeriofonteles.github.io/lunar-rover/drake/acrobot/control_lab/?comparacao=solvers&torque=clipped).

## Executar em outro PC

Os comandos abaixo são para um host Linux com Bash, Git, Docker Engine e o plugin
Docker Compose. Use um usuário com acesso ao Docker. As imagens e dependências
precisam de internet durante o primeiro build. Os scripts resolvem a pasta do clone
automaticamente e usam seu UID/GID; não é necessário usar o nome `rogerio` nem
clonar dentro de `~/Projects`.

```bash
git clone git@github.com:rogeriofonteles/lunar-rover.git
cd lunar-rover
```

A chave SSH desse PC precisa ter acesso ao repositório. Também é possível usar
`https://github.com/rogeriofonteles/lunar-rover.git` com sua autenticação do GitHub.

### Drake / Acrobot — sem GPU obrigatória

```bash
./docker/scripts/build_image.sh drake
./docker/scripts/create_jazzy_docker_container.sh drake --headless
```

O segundo comando acompanha os logs. Quando o contêiner estiver iniciado,
pressione **Ctrl+C**; ele continua rodando. O perfil headless dispensa NVIDIA,
X11 e `/dev/dri`, mas permite visualizar Meshcat no navegador.

```bash
# Teste pequeno e explícito, sem depender dos parâmetros atuais de lqr.yaml.
docker exec lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/acrobot_test.py --mode infinite --initial-state 3.111592653589793 0.03 0 0 --torque-limit-nm 15 --torque-clipping --timestep 0.05 --steps 120 --R 0.1 --headless --no-wait'

# Testes dos controladores.
docker exec lunar-rover-jazzy-drake fish -c 'cd ~/drake; python -m unittest discover -s acrobot -p "test_*.py"'

# Rover com Meshcat: abra a URL localhost:7000 informada no terminal.
docker exec -it lunar-rover-jazzy-drake fish -c 'cd ~/drake; python simulate.py'
```

A imagem fixa as versões Python do conjunto de experimentos em
[drake/requirements.txt](drake/requirements.txt). O ambiente usa Ubuntu 26.04 e
Python 3.14. Para recompilar STOMP e baixar seus modelos fixados:

```bash
docker exec lunar-rover-jazzy-drake fish -c 'cd ~/drake/stomp_benchmark; python setup.py'
```

Consulte também [Acrobot](drake/acrobot/README.md),
[laboratório HTML](drake/acrobot/control_lab/README.md) e
[STOMP](drake/stomp_benchmark/README.md). A configuração `lqr.yaml` pode selecionar
MPC e exigir muitas otimizações; comece pelo comando curto acima.

### MuJoCo — simulação numérica sem GPU

```bash
./docker/scripts/build_image.sh mujoco
./docker/scripts/create_jazzy_docker_container.sh mujoco --headless
# Ctrl+C após a criação, para sair dos logs.
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python simulate.py --headless --no-wait'
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python -m unittest discover -s tests -p "test_*.py"'
```

Para usar o viewer e renderização NVIDIA, configure o driver e NVIDIA Container
Toolkit no host, use X11/XWayland e crie o contêiner **sem** `--headless`:

```bash
./docker/scripts/create_jazzy_docker_container.sh mujoco
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python -m arm.check_graphics --require-nvidia'
docker exec -it lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python simulate.py'
```

O Compose normal mantém a renderização NVIDIA; o perfil headless é separado e
serve à física sem viewer. Detalhes, parâmetros e gravação de vídeos estão no
[guia MuJoCo](mujoco/README.md).

### ROS 2 Jazzy / Gazebo

O Compose ROS atual usa NVIDIA e X11/XWayland. Prepare o driver e NVIDIA Container
Toolkit antes de utilizá-lo. O setup importa as dependências de
`config/dependencies.repos` e compila o workspace externo; pode levar vários minutos.

```bash
./docker/scripts/build_image.sh ros
./docker/scripts/create_jazzy_docker_container.sh ros
# Aguarde "Setup complete" e saia dos logs com Ctrl+C.
docker exec -it lunar-rover-jazzy-ros bash
```

Dentro do contêiner:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ext/install/setup.bash
cd ~/lunar_rover
colcon build --symlink-install
source install/setup.bash
ros2 launch lunar_rover_sim simulation.launch.py
```

O launch padrão usa o modelo Perseverance. Os arquivos atuais do modelo Curiosity
estão incluídos diretamente no repositório, preservando a reorganização local;
não é necessário clonar um submódulo. Sua origem está registrada em
[Curiosity/UPSTREAM.md](lunar_rover/src/lunar_rover_sim/urdf/curiosity/UPSTREAM.md).

## Retomar os ambientes

```bash
./docker/scripts/start_jazzy_container.sh drake
./docker/scripts/enter_jazzy_container.sh mujoco
```

`start` inicia um contêiner existente e abre Fish; `enter` abre Fish em um
contêiner que já está rodando. Os scripts aceitam `ros`, `drake` e `mujoco` e
não dependem de `yq` nem de arquivos `.env` copiados de outro PC.

## O que é versionado

Código, testes, configurações, modelos, dados de experimentos e entregáveis
como o HTML do Acrobot fazem parte do repositório. Builds do ROS, caches Python,
`.env`, bibliotecas copiadas para o editor e temporários ficam fora. O benchmark
STOMP baixa novamente seus assets e compila sua biblioteca nativa pelo `setup.py`.
Saídas locais do MuJoCo seguem o `.gitignore` desse módulo.

O diretório `output/` contém relatórios de estudo; `drake/output/` contém relatórios
e visualizações já produzidos. Esses arquivos não são dependências dos simuladores.

## Verificação desta preparação

As configurações Compose e os caminhos foram conferidos para o clone em outro
diretório. Os testes de Drake/MuJoCo são executados nos contêineres existentes.
Isso não substitui um build completo das três imagens em uma máquina limpa;
especialmente o setup ROS baixa dependências externas e depende da GPU do host.
