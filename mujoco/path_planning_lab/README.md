# Laboratório de planejamento e controle — MuJoCo

Abra `index.html` no navegador. A página funciona também por `file://`: os
dados dos gráficos estão em `data.js` e os vídeos são arquivos MP4 locais.
Nenhuma seleção na página dispara uma simulação ou altera os resultados.

## Planejar e executar

Dentro do contêiner `lunar-rover-jazzy-mujoco`, em `~/mujoco`:

```sh
python plan_buttons.py --sequence red yellow green red --mode mpc --solver sqp --horizon 20
python plan_buttons.py --sequence green red yellow --mode infinite
python plan_buttons.py --mode finite --discretization euler
python plan_buttons.py --mode receding --horizon 20
python plan_buttons.py --mode ilqr --ilqr-max-iterations 5
python plan_buttons.py --mode ilqr_receding --horizon 20 --ilqr-max-iterations 5
python plan_buttons.py --mode mpc --solver slsqp --horizon 20
python plan_buttons.py --mode mpc --solver dual_ascent --horizon 20
```

A execução começa **em repouso no primeiro botão da sequência**, não na pose
aleatória dos ensaios de regulação. O estado medido é contínuo: não há reinício
entre destinos. `simulate.py --button ...` mantém seu comportamento anterior.
`--headless` executa sem janela; `--wait` mantém a janela aberta ao concluir.

Os parâmetros `--seed`, `--speed` (rad/s), `--acceleration` (rad/s²) e `--dwell`
(segundos) controlam o planejamento e a temporização. As durações das arestas
dependem do caminho encontrado. O limite de velocidade vale para cada junta.
Use botões consecutivos distintos. Planejamento inválido ou esgotamento do
orçamento do RRT produz erro, sem interpolação direta não verificada.

```sh
python plan_buttons.py --mode mpc --solver sqp --horizon 20 --headless \
  --output output/minha-rota --video output/minha-rota.mp4
```

O vídeo é renderizado pelo MuJoCo, com NVIDIA EGL, a partir dos estados
integrados. O encoder FFmpeg produz H.264, 1920×1080, 30 quadros/s. Não há áudio.
O tempo do vídeo representa o tempo simulado, não a espera pelo controlador.

## Reproduzir o relatório

```sh
python path_planning_lab/generate.py
# Apenas dados, sem renderização:
python path_planning_lab/generate.py --skip-videos
# Renderizar novamente vídeos a partir de execuções já salvas:
python path_planning_lab/generate.py --render-only
# Repetir apenas um método:
python path_planning_lab/generate.py --only mpc_sqp
```

`runs/` contém estados, referências, torques e métricas completos (`.npz` e
`.json`). `videos/` contém vídeos e imagens finais. `route.json` contém os
caminhos, parâmetros e métricas do planejador. `data.json` e `data.js` contêm
as séries dos gráficos, reduzidas a uma amostra a cada três passos. Métricas
e critérios de chegada são calculados sobre as amostras completas, a 50 Hz.
O gerador usa a mesma rota e a mesma preparação nominal para todos os métodos.

## Formulação

- **RRT-Connect:** duas árvores no espaço de seis juntas, semente 6, extensão
  de 0,25 rad em norma euclidiana e no máximo 6.000 iterações por trecho.
  Arestas são verificadas a cada 0,025 rad no máximo por junta. O encurtamento
  testa até 150 pares de vértices; só aceita ligações verificadas.
- **Temporização:** polinômio `10 s³ − 15 s⁴ + 6 s⁵` em cada aresta, com
  repouso nas quinas. Limites de referência: 0,65 rad/s e 1,2 rad/s². Pausas
  de 0,6 s nos destinos. Nenhuma spline arredonda quinas fora do caminho validado.
- **Referência dinâmica:** posição, velocidade e aceleração analíticas.
  `mj_inverse` calcula o torque nominal. A linearização ao longo da referência
  é discretizada por Euler ou retenção de ordem zero (ZOH). A diferença entre
  o próximo estado integrado e a referência entra como termo afim.
- **LQR infinito:** ganhos de Riccati estacionário para cada modelo congelado
  ao longo da referência; usa torque nominal mais realimentação. Não é um
  único ganho fixo para toda a rota.
- **LQR finito:** recursão de Riccati variante no tempo sobre a rota inteira,
  incluindo o termo afim. O modo `receding` usa a janela móvel de 20 passos.
- **MPC:** QP condensado com modelos e referências variantes no tempo. Mantém
  os três solvers existentes. Reutiliza a sequência anterior de torques como
  estimativa primal; nesta implementação, a Hessiana é remontada e os
  multiplicadores duais são reinicializados em cada amostra.
- **iLQR:** mesma implementação não linear, ampliada para referência por
  instante. O modo `ilqr` otimiza a rota completa; `ilqr_receding` usa janela
  móvel. O limite do relatório é de cinco iterações, com convergência registrada.

## Alcance das verificações

`arm/planning_scene.json` define limites de busca, raios das cápsulas, esfera,
caixas e margem. Cápsulas aproximam os elos; a esfera, o pedestal, a bancada e
o armário entram no teste. Elos não vizinhos também são verificados. Cabos,
parafusos, suportes dos botões e contato de acionamento ficam fora do modelo.
As distâncias a caixas usam uma amostragem espacial com correção conservadora.
Ao alterar a esfera no JSON, execute `python arm/models/build_scene.py` para
atualizar também sua geometria visível antes de gerar novos resultados.

A margem do planejamento é 25 mm. A execução registra folga a cada 20 ms e
para se detectar interpenetração no modelo geométrico. Isso não certifica
colisão contínua nem colisão das malhas detalhadas. A dinâmica do cenário
continua sem forças de contato. Os controladores não incluem colisões em
seus custos/restrições e podem desviar da referência; a checagem é separada.

Uma chegada exige erro da ponta menor que 10 mm e velocidade máxima menor
que 0,05 rad/s no instante planejado. Isso é independente da convergência do
solver. As medições de tempo são de uma execução nesta máquina, sem garantia
de tempo real; atrasos não são injetados na planta.

## Testes

```sh
python -m unittest discover -s tests -p 'test_*.py'
```

Os testes cobrem desvios, validade de arestas, reprodutibilidade, falha explícita
do RRT, limites de velocidade/aceleração, dinâmica inversa, condensação afim,
referência móvel do iLQR e uma rota contínua sem reiniciar o estado.
