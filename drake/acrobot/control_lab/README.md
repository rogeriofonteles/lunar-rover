# Laboratório interativo do Acrobot — PT-BR

Abra **index.html** no navegador. HTML, CSS, JavaScript, dados e vídeos são locais;
não há CDN, telemetria ou servidor de simulação. Mantenha esta pasta inteira
para preservar os vídeos e os dados. Opcionalmente, sirva a pasta:

```sh
python3 -m http.server 8765 --directory drake/acrobot/control_lab
```

Abra `http://localhost:8765`. O parâmetro `?cenario=far` abre um cenário específico.

## O que explorar

- Onze desafios × três opções de torque (original, ±3 N·m e sem clipping), com seis segundos de física em cada execução. São sete métodos registrados por configuração: 231 trajetórias.
- Explicações, conceitos, roteiro de investigação e parâmetros acompanham a comparação, o desafio e o torque selecionados. Os resultados narrados vêm dos dados da execução; a explicação do custo acompanha o solver escolhido.
- Visão principal LQR infinito × MPC não linear; opção de mostrar os quatro controladores; comparação separada de solvers MPC.
- Replay sincronizado com pausa, velocidade, scrubbing, alvo e rastros.
- Plano nominal iLQR e previsões MPC (guardadas a cada dez amostras).
- Ângulos das juntas, orientações absolutas dos links, velocidades, torque,
  distância normalizada do equilíbrio e resíduo da previsão linear.
- WallTime do primeiro solve, mediana/p95 online, deadline misses e estabilização física.
- Custo por atualização aceita, motivo de parada, initial guess e avisos do otimizador.
- Vídeos MP4 e exportação CSV/JSON.

Os parâmetros da interface selecionam **experimentos pré-calculados**. A página
não roda Drake em JavaScript nem inventa resultados para combinações não testadas.

## Reproduzir com Drake

A partir da raiz do projeto, com o contêiner iniciado:

```sh
docker exec -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/control_lab/generate.py --variants original clipped unclipped'
```

O gerador importa os controladores de `acrobot/lqr.py`, `acrobot/ilqr.py` e `acrobot/linear_mpc.py`.
Não lê nem altera `lqr.yaml`. A planta real é `AcrobotPlant`, integrada pelo
`Simulator` com precisão alvo 1e-9 e torque constante entre amostras de 0,05 s.

Cada execução fica em `runs/<cenario>_<metodo>.json`, com hash da configuração
e das fontes. Dados válidos são reutilizados; `--force` força o recálculo.
`--scenarios near far --modes finite infinite` restringe o trabalho.
`--export-only` recompõe `data.json` e `data.js` a partir dos resultados existentes.
Use `--variants original clipped unclipped` para produzir o conjunto completo antes de distribuir o HTML.

Para criar outras experiências, edite `BASE` / `SCENARIOS` em `generate.py`,
execute novamente e acrescente a explicação correspondente em `app.js`.

## Gerar vídeos no host

Requer Python 3 com NumPy/Pillow e `ffmpeg`. Os vídeos são visualizações 2D das
trajetórias registradas pelo Drake, e não capturas da interface Meshcat.

```sh
python3 drake/acrobot/control_lab/render_videos.py --force
```

Formato: H.264, 1280 × 720, 30 fps, sem áudio. A geometria usa l1 = 1 m e
l2 = 2 m do vetor de parâmetros do modelo; θ2 é relativo ao link 1.
Interpolação linear entre amostras serve somente à apresentação. Saltos de
velocidade são preservados como eventos. Os dois gráficos pequenos no vídeo
usam escala própria por método; para comparação quantitativa, use o HTML,
onde as escalas são compartilhadas entre os métodos.

## Método e limites

- LQR finito: sequência de ganhos durante os 6 s; terminal Q, exceto no cenário 09.
- LQR infinito: ganho constante do mesmo modelo linear Euler.
- iLQR: um plano não linear de 6 s com feedback local; não reotimiza após perturbações.
- MPC: iLQR em receding horizon, horizonte padrão 2 s, warm start deslocado, 80 iterações
  máximas no primeiro solve e 6 nos seguintes. Cada cenário pode substituir esses valores.
- MPC/LQR: solução analítica via Riccati, ganho preparado uma vez, seguida de clipping quando habilitado.
- MPC/SQP e MPC/SLSQP: o mesmo QP convexo linear com limites de torque, até 300 iterações por solve. SQP usa Hessiana exata/active set; SLSQP usa SciPy. Isto não implementa SQP não linear.
- A parametrização por Riccati completa quadrados e dá Hessiana identidade, evitando condensar potências do modelo instável. A transformação preserva o problema e os limites físicos.
- SLSQP resolve o mesmo QP em `y = v / escala`, com `escala = max(1, ||initial_guess||∞)`, objetivo dividido pela escala² e restrições divididas pela escala. Isso preserva a solução; `ftol=1e-13` se aplica ao problema normalizado.
- Candidatos com violação de torque são projetados no intervalo físico e têm o rollout linear refeito. O melhor plano viável entre o candidato reparado e o initial guess é retornado. Um candidato não finito ou um reparo de custo pior aciona fallback explícito. Pequenas violações não descartam automaticamente todo o plano. Projeção candidata, violação original e fallback ficam nos dados. Ajustes ≤ 1e-8 N·m são tratados como arredondamento: `recovery` indica se a projeção foi aplicada; `repair_max_nm` registra sua magnitude avaliada para o candidato aproveitado.
- O certificado comum dos QPs verifica estacionariedade relativa, viabilidade primal e gap dual relativo, com tolerância 1e-6; exige também violação física ≤ 1e-8 N·m. Multiplicadores não negativos são ajustados às restrições próximas de ativas (folga < 1e-5 nas linhas normalizadas). A estacionariedade é dividida por `max(1, ||v||∞)`. O gap `0.5 ||v+Gᵀλ||² + λᵀ(h−Gv)` é dividido por `max(1, 0.5 vᵀv)`; folgas negativas de arredondamento são truncadas em zero neste termo, e verificadas separadamente na viabilidade. Esse gap também verifica complementaridade, evitando depender apenas da classificação de restrições ativas.
- O status nativo do SLSQP é preservado separadamente do certificado do plano retornado. Uma mensagem nativa de parada sem sucesso pode acompanhar uma solução que passa na verificação independente. Isso é informado, não ocultado.
- `costs` mantém o histórico dos candidatos do solver. `initial_cost`, `candidate_cost` e `returned_cost` distinguem o ponto inicial, o candidato bruto e o plano realmente retornado. O custo retornado é recalculado por rollout. O ponto final laranja no gráfico mostra esse valor após reparo/fallback; a redução percentual usa esse custo, não o candidato descartado. Esse ponto não conta como iteração adicional.
- Sem limites, os três métodos lineares usam a mesma solução analítica. Com terminal DARE, ela equivale ao LQR infinito do mesmo modelo; diferenças numéricas pequenas podem se amplificar em execuções que perdem estabilidade.
- Ambos os LQRs e MPC/Riccati têm clipping posterior ao projeto quando habilitado. iLQR/MPC consideram o limite
  escalar no backward pass e no rollout. Não há restrições de posição/velocidade.
- iLQR/MPC usam RK4 (subpassos ≤ 0,01 s) e terminal DARE local por padrão.
  Há diferenças de discretização e terminal em relação ao LQR: a comparação
  não isola exclusivamente o nome do algoritmo.
- A inicialização automática pode usar direct collocation do Drake para desvios
  maiores. Esse custo está incluído na primeira solução, e o resultado registra
  o initial guess selecionado e avisos de fallback.
- No iLQR/MPC não linear, “convergido” é um critério local, sem garantia de ótimo global. Nos QPs convexos, o certificado verifica otimalidade nas tolerâncias descritas; isso não garante estabilidade na planta não linear.
- Estabilização: cada erro angular periódico < 0,05 rad e cada velocidade
  < 0,1 rad/s, por pelo menos 1 s até o final da janela. O tempo informado é o
  primeiro instante desse trecho final contínuo. A região pode ter sido atingida
  antes e perdida após uma perturbação.
- Saltos de velocidade são aplicados diretamente ao estado e não têm custo de
  impulso contabilizado. Não são torques perturbadores de duração finita.
- O resíduo linear combina erro de discretização Euler e não linearidade; sua
  norma mistura rad e rad/s. Descontinuidades da representação angular periódica
  também podem causar picos. Não é uma distância física nem uma medida pura de
  não linearidade. A perturbação conhecida é descontada no passo correspondente.
- A simulação **espera** o cálculo. Prazos excedidos são medidos, mas os atrasos
  não são inseridos na dinâmica: os vídeos não demonstram viabilidade em tempo real.
- As medidas de WallTime das trajetórias são de uma execução por caso, nesta máquina, com possíveis efeitos de
  carga e aquecimento de caches. Mediana/p95 são entre comandos da mesma execução,
  não entre repetições estatísticas. O primeiro solve é separado dos seguintes.
- No cenário 05, horizonte e terminal mudam juntos. Nos cenários 07/08, também
  mudam inicialização e orçamento. Estes são estudos de configurações ruins,
  não ablações que isolam cada parâmetro.

## Arquivos

- `index.html`, `style.css`, `app.js`: interface sem dependências externas.
- `data.js`: dados locais carregáveis também via `file://`.
- `data.json`, `runs/`: dados completos com configuração, estados, comandos,
  tempos, custos aceitos, parâmetros físicos e hashes de origem.
- `generate.py`: simulação e medição dentro do contêiner Drake.
- `render_videos.py`, `videos/`: visualização dos mesmos dados em MP4.
- `validate.py`: integridade do conjunto, limites e consistência com o Drake.

Referências: [LQR](https://underactuated.mit.edu/lqr.html),
[otimização de trajetórias](https://underactuated.mit.edu/trajopt.html),
[Acrobot no Drake](https://drake.mit.edu/pydrake/pydrake.examples.html).

## HTML independente e verificações

`acrobot_interativo.html` incorpora CSS, JavaScript, dados e os vídeos das duas famílias de comparação para cada opção de torque.
Esse arquivo (aproximadamente 55 MiB nesta geração) pode ser movido sozinho e aberto sem internet. Para hospedagem web, prefira `index.html` com os arquivos da pasta: os vídeos carregam sob demanda. Para reconstruí-lo:

```sh
python3 drake/acrobot/control_lab/build_standalone.py
```

Verificações realizadas nesta entrega:

- 28 testes do Acrobot e da formulação MPC linear passaram no contêiner Drake, incluindo referências OSQP/Clarabel, regressão da inversão de torque por fallback e custo do plano realmente retornado.
- As 231 execuções foram reproduzidas independentemente no Drake a partir dos torques salvos,
  com diferença máxima de estado igual a zero.
- Validação de limites, custos aceitos, métricas de convergência, prazos e
  equivalência LQR finito com terminal P∞ / LQR infinito.
- Teste das interações JavaScript com um DOM simulado (`node test_ui.cjs`).
- 66 vídeos decodificados por ffmpeg (1280 × 720, 181 quadros cada); quadros das duas famílias inspecionados.
- O serviço local respondeu HTTP 200. Não havia navegador disponível pela
  ferramenta de inspeção: layout e reprodução de mídia no navegador não foram
  verificados visualmente. O teste com DOM simulado não substitui essa revisão.

## Terminologia da interface

As explicações permanecem em PT-BR. A interface usa WallTime, solver/solve,
deadline misses, initial guess, warm start, direct collocation, receding horizon,
rollout, backward pass, line search, feedback, clipping e damping. Um glossário
na seção “Transparência & reprodução” explica esses termos em português.
WallTime é o tempo decorrido no relógio, incluindo esperas; difere tanto do
tempo simulado quanto do tempo de CPU. Os nomes das métricas nos dados brutos
permanecem iguais para preservar a compatibilidade dos arquivos.


## SQP × SLSQP e apresentação condicional

Depois de gerar todas as trajetórias, rode sem outras simulações concorrentes:

```sh
docker exec -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/control_lab/compare_solvers.py; and python acrobot/control_lab/generate.py --export-only'
python3 drake/acrobot/control_lab/render_videos.py --force
python3 drake/acrobot/control_lab/build_standalone.py
```

O benchmark usa seis estados de cada um dos onze desafios com ±3 N·m,
da trajetória SQP original: 66 problemas pareados. Os estados, configurações e initial guesses foram congelados em `solver_benchmark_cases.json` antes da correção; não são substituídos pelas novas trajetórias. Cada par usa o mesmo
estado, modelo, custo, horizonte, limites e initial guess; são sete repetições,
com aquecimento excluído e ordem alternada. Não se usam estados diferentes de
duas trajetórias para alegar uma diferença intrínseca entre solvers. Alguns desafios
diferem apenas em parâmetros dos métodos não lineares e repetem o mesmo QP;
os 66 pares não são 66 problemas independentes. Cada par também é comparado ao solver cônico Clarabel do Drake, fora da medição de WallTime. O JSON guarda diferença de todos os torques do plano, erro relativo de custo, reparos, fallbacks, status nativo e certificado QP, além dos hashes das entradas e do controlador.

Os critérios definidos antes da comparação para mostrar SLSQP separadamente são:
Δu₀ > 0,1 N·m; diferença no certificado QP em pelo menos 5% dos pares; ou razão de
medianas de WallTime ≥ 3 com diferença absoluta ≥ 1 ms. Basta um critério.
A conclusão e os detalhes estão em `solver_comparison.json`, também incorporado
no JSON principal. Se os critérios não forem atingidos, a visão principal de
solvers mostra SQP, Riccati e MPC não linear. A curva SLSQP pode ser ativada na opção adicional; os dados e diagnósticos de ambos permanecem disponíveis. O MP4 segue a comparação padrão do benchmark.

As opções “sem clipping” usam `limit: null`, não um limite grande aproximado.
Torques e velocidades muito grandes podem ocorrer. O replay reconstrói o movimento
a partir de amostras de 50 ms: não resolve todas as rotações em execuções muito
rápidas, e mostra um aviso de aliasing nesses casos.


## Resultados desta geração

Com ±3 N·m, no desafio 01 o LQR infinito estabilizou em 1,85 s e o MPC
não linear em 1,70 s. No desafio 11 (17°), somente o MPC não linear entre
os quatro controladores estabilizou: 3,90 s. No desafio 02 (57°), esse tempo
foi 2,75 s. Isso não garante que um desvio menor será sempre mais fácil para
um solver local. Perto e longe do equilíbrio, vários solves do MPC não linear
excederam o período de 50 ms; a planta simulada aguardou esses cálculos.

## Correção da comparação SQP/SLSQP

A versão anterior podia descartar um candidato próximo do ótimo por uma pequena
violação de torque e aplicar um initial guess com torque oposto. No caso auditado,
um candidato de aproximadamente −3,000048 N·m foi substituído por +3 N·m.
O gráfico mostrava a melhora do candidato descartado (92,38%), sem representar
o retorno ao custo inicial. Reproduzir a física não detectava esse erro de
interpretação. Essa comparação anterior não demonstrava inferioridade do SLSQP.

A revisão normaliza as variáveis, repara a viabilidade, compara o custo dos planos
viáveis, registra o plano retornado e verifica o QP com um certificado comum e
uma referência Clarabel independente. Foram recalculadas as 99 execuções dos
três MPCs lineares; as 132 execuções dos quatro controladores originais não
mudaram, pois não usam essa implementação.


Após a correção, os 66 pares passaram no certificado QP para ambos os métodos,
sem fallback do SLSQP. A maior diferença no primeiro torque SQP/SLSQP foi
9,57 × 10⁻⁹ N·m. Contra o Clarabel, a maior diferença entre todos os torques dos
planos foi 1,63 × 10⁻⁶ N·m e o maior erro relativo de custo foi 8,08 × 10⁻⁸.
As medianas de WallTime pareadas foram 2,54 ms (SQP) e 1,05 ms (SLSQP).
Os critérios predefinidos não foram atingidos; a curva SLSQP fica opcional.

Em seis pares, o SLSQP ainda retornou status nativo sem sucesso. Os planos desses
casos, após pequenos reparos, passaram no certificado comum e concordaram com a
referência independente. A interface informa os dois resultados separadamente;
não converte o status nativo em sucesso nem afirma convergência exata.

O Clarabel de referência usa a mesma escala de variáveis, regularização estática
desabilitada, até 500 iterações e tolerâncias de 1e-12, incluindo as tolerâncias
reduzidas. Sua solução precisa também respeitar o limite físico dentro de 1e-6
N·m. Não aceitamos apenas o indicador de sucesso: a configuração padrão pôde
retornar uma solução de precisão reduzida com violação física significativa
em um caso mal escalado. A viabilidade é conferida antes de usá-la como referência.
