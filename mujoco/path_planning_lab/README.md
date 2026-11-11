# Laboratório de planejamento e controle — MuJoCo

Abra `index.html` no navegador. A página funciona também por `file://`: os
dados dos gráficos estão em `data.js` e os vídeos são arquivos MP4 locais.
Nenhuma seleção na página dispara uma simulação ou altera os resultados.

## Escolher o caso e entender as diferenças

O seletor **Caso do experimento** troca os resultados registrados. Cada caso
compara os oito métodos, com o mesmo caminho RRT, a mesma condição inicial e
os mesmos parâmetros físicos para todos eles. Não há uma nova simulação no
navegador. A seleção também atualiza o vídeo, o comando de reprodução e os
links para os dados. Links diretos aceitam, por exemplo,
`?case=torque&method=mpc_sqp`.

| `--case` | Alteração | Onde olhar |
|---|---|---|
| `nominal` | 0,65 rad/s; 1,2 rad/s²; modelo conhecido | Erros pequenos versus custo de cálculo |
| `fast` | 1,8 rad/s; 5 rad/s² | Picos de erro nas acelerações e frenagens |
| `torque` | Mesma referência rápida; limites `[3, 29, 20, 2.2, 2, 0.6]` N·m | Saturação, erro e destinos alcançados; compare com `fast` |
| `payload` | 0,5 kg adicionais na ponta, desconhecidos pelo controlador | Erro persistente e perda de folga geométrica |
| `disturbance` | Pulso externo `[6, -10, 5, 0, 0, 0]` N·m, de 6,0 a 6,3 s | Pico de erro e recuperação após o pulso |

A pausa nos botões é 0,6 s. O nominal dura 24,18 s; a referência rápida dura
10,5 s. Os limites dos casos, seus textos e o pulso estão em
`arm/route_cases.py`. `--speed`, `--acceleration` e `--dwell` podem sobrescrever
os valores de um caso na CLI; os valores efetivos ficam no JSON da execução.

**Leitura da comparação:**

1. Leia o bloco “Onde olhar” e o resumo calculado a partir das execuções.
2. Veja execução completa/interrompida e as chegadas **no prazo, com atraso
   ou não observadas** antes de comparar médias. Chegar com atraso conta como
   chegada; terminar a simulação não garante que todos os destinos foram atingidos.
3. Marque os métodos para sobrepor **erros**, não apenas posições. Selecione
   uma junta e amplie o evento ou a região de maior erro. Passe o ponteiro ou
   toque na curva para ler valores.
4. Escolha A e B para obter uma comparação numérica; “Sobrepor apenas A e B”
   isola suas curvas. Diferenças pequenas entre solvers MPC são esperadas:
   todos resolvem o mesmo QP.
5. Use o gráfico erro × custo e a tabela ordenável para avaliar o esforço
   computacional. O orçamento é 20 ms, mas atrasos não são injetados na planta.

O **RMS comum** usa o intervalo de zero até o menor tempo final entre todos
os métodos do caso. Isso impede comparar médias de trajetórias com durações
diferentes por causa de uma interrupção. As curvas mostram todo o trecho
realmente executado; pico, saturação e os cartões da execução usam esse trecho.
Uma execução interrompida nunca é classificada como sucesso por ter RMS menor.

Saturação é a fração dos passos em que alguma junta atinge pelo menos 99,9%
do limite. A recuperação exige erro da ponta **menor que 5 mm durante 0,5 s**
após o pulso. Ausência de recuperação é exibida explicitamente. Esse critério
não é o mesmo que chegar a um botão (10 mm e 0,05 rad/s).

### Modelo nominal e planta independente

A dinâmica inversa, as linearizações e as previsões do controlador usam o
braço nominal. A planta recebe a carga adicional ou o torque externo, sem
informação antecipada para os controladores. Não há ação integral, observador
de distúrbios ou identificação da carga. O pulso usa `qfrc_applied`, separado
dos torques dos motores; esses torques externos não contam como saturação.

A carga é uma esfera rígida de raio 3 cm e massa 0,5 kg, centrada no site
`button_tip`. Sua massa, centro de massa e tensor de inércia são incorporados
à ferramenta pelo teorema dos eixos paralelos; `mj_setConst` atualiza as
constantes do modelo. A esfera amarela no vídeo identifica a carga. Seu volume
não entra no verificador geométrico de elos. Referência da API:
[alteração de parâmetros e forças externas no MuJoCo](https://mujoco.readthedocs.io/en/3.3.7/programming/simulation.html).

No caso de torque limitado, a referência é mantida mesmo quando seu torque
nominal excede os limites. O termo afim e os Jacobianos usam a dinâmica sem
saturação no ponto nominal; as restrições do MPC e o recorte do comando atuam
separadamente. A planta sempre respeita os limites dos atuadores. Não se reduz
a velocidade silenciosamente para tornar a referência viável.

## Planejar e executar

Dentro do contêiner `lunar-rover-jazzy-mujoco`, em `~/mujoco`:

```sh
python plan_buttons.py --case nominal --sequence red yellow green red --mode mpc --solver sqp --horizon 20
python plan_buttons.py --case torque --mode mpc --solver sqp --horizon 20
python plan_buttons.py --case disturbance --mode infinite
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
# Todos os cinco casos e oito métodos (iLQR móvel pode demorar):
python path_planning_lab/generate.py
# Apenas dados, sem renderização:
python path_planning_lab/generate.py --skip-videos
# Renderizar novamente vídeos a partir de execuções já salvas:
python path_planning_lab/generate.py --render-only
# Repetir apenas um método:
python path_planning_lab/generate.py --cases torque --only mpc_sqp
# Apenas remontar a página com os dados existentes:
python path_planning_lab/generate.py --cases nominal fast torque payload disturbance --assemble-only
```

O caso nominal mantém `runs/`, `videos/` e `route.json` na raiz. Os demais
usam `cases/<caso>/`, com a mesma estrutura. `cases.js` contém o catálogo. Os
dados são carregados sob demanda, inclusive ao abrir a página por `file://`.

`runs/` contém estados, referências, torques e métricas completos (`.npz` e
`.json`). `videos/` contém vídeos e imagens finais. `route.json` contém os
caminhos, parâmetros e métricas do planejador. `data.json` e `data.js` contêm
as séries de detalhes, reduzidas a uma amostra a cada três passos (incluindo
o estado final). As **curvas comparativas mantêm todas as amostras**, para
não esconder picos curtos. Métricas
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
que 0,05 rad/s na mesma amostra, com folga geométrica não negativa:

- **No prazo:** os critérios são atendidos no instante planejado.
- **Com atraso:** a primeira amostra que atende aos critérios ocorre depois,
  até o fim da pausa planejada no botão (0,6 s nestes casos).
- **Não chegou:** o critério não foi observado nessa janela. O relatório
  diferencia uma janela encerrada de uma execução interrompida antes ou durante
  a avaliação. Isso não afirma que o botão seja fisicamente inalcançável.

O fim da pausa está incluído; amostras do trecho seguinte não são usadas.
A primeira pose é condição inicial e não conta como destino. O atraso tem
resolução de 20 ms. Trata-se da primeira amostra válida, sem exigir permanência;
a recuperação de uma perturbação continua exigindo 0,5 s abaixo de 5 mm.
No caso de torque limitado, os três MPCs e o iLQR móvel chegam ao vermelho
160 ms atrasados: agora aparecem com **3/3 chegadas, sendo 2 no prazo e 1 com
atraso**. Os resultados são recalculados dos arquivos NPZ completos, sem novas
simulações ou alterações nos vídeos e torques registrados.

O JSON mantém `reached` com seu significado original (no prazo). Os campos
`status`, `reached_within_window`, `actual_arrival_time_s`, `delay_s`, as métricas
na chegada e `arrival_counts` tornam a distinção explícita, tanto na página
quanto nos arquivos de métricas para download. `--assemble-only` atualiza esses
arquivos preservando o log do solver. Novas simulações usam a mesma avaliação.

Chegada e pontualidade são independentes da convergência do solver. As medições de tempo são de uma execução nesta máquina, sem isolamento de
CPU e com possível concorrência de geração/renderização de outros casos.
Elas não são um benchmark estatístico e não garantem tempo real; atrasos não são injetados na planta.

## Testes

```sh
python -m unittest discover -s tests -p 'test_*.py'
```

Os testes cobrem desvios, validade de arestas, reprodutibilidade, falha explícita
do RRT, limites de velocidade/aceleração, dinâmica inversa, condensação afim,
referência móvel do iLQR, uma rota contínua sem reiniciar o estado, isolamento
do modelo nominal, massa/inércia da carga, duração e remoção do pulso e
linearização afim em torno de torque nominal inviável.
