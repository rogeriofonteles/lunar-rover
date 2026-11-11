# Laboratório interativo do Acrobot — PT-BR

Abra `index.html` com os arquivos desta pasta, ou baixe `acrobot_interativo.html`
para abrir o laboratório inteiro offline em um único arquivo. Não há CDN,
telemetria nem servidor de simulação. Para servir a versão modular:

```sh
python3 -m http.server 8765 --directory drake/acrobot/control_lab
```

Acesse `http://localhost:8765/?cenario=nonlinear&torque=clipped&comparacao=solvers`.
[Abrir a comparação no GitHub Pages](https://rogeriofonteles.github.io/lunar-rover/drake/acrobot/control_lab/?comparacao=solvers&torque=clipped).
A hospedagem usa os mesmos arquivos; o HTML independente continua disponível para download.

## Comparações disponíveis

Onze desafios × três opções de torque (original, ±3 N·m, ilimitado), com seis
segundos de física: nove métodos por configuração, 297 trajetórias e 99 MP4.
A interface seleciona experimentos pré-calculados; não executa Drake no navegador.

| Visão | Métodos |
| --- | --- |
| Principal | LQR infinito × NMPC com iLQR |
| Quatro controladores | LQR finito, LQR infinito, plano único iLQR, NMPC com iLQR |
| MPC não linear | iLQR, SQP Gauss–Newton e SLSQP, todos reotimizando trajetórias não lineares |
| Referência linear | Riccati e active set; SLSQP opcional; NMPC-iLQR como referência |

O experimento antigo `mpc_sqp` é um **QP linear resolvido por active set**, não
SQP para dinâmica não linear. Ele permanece disponível, com esse nome explícito
na interface. Os novos métodos são `mpc_nonlinear_sqp` e `mpc_nonlinear_slsqp`.
O SLSQP não linear sempre aparece na sua comparação; o agrupamento opcional do
benchmark antigo se aplica exclusivamente aos QPs lineares.

Explicações e resultados acompanham o desafio, a comparação e o torque selecionados.
Há replay sincronizado, posições e velocidades das juntas/links, torque, custo,
WallTime, deadline misses, previsões, vídeos e exportação CSV/JSON. A tabela
“Comparar ±3 N·m com torque ilimitado” usa execuções próprias para cada opção.

## SQP e SLSQP ao longo da trajetória

Os três NMPCs usam exatamente o mesmo mapa discreto não linear `F(x,u)`:
RK4 do `AcrobotPlant`, com subpassos de no máximo 10 ms. As derivadas são
calculadas por **AutoDiff do Drake da própria integração RK4**:

```text
A_k = ∂F/∂x (x_k, u_k)
B_k = ∂F/∂u (x_k, u_k)
δx_(k+1) = A_k δx_k + B_k δu_k
J = 0.5 Σ [e_kᵀ Q e_k + R u_k²] + 0.5 e_Nᵀ Qf e_N
```

A cada pedido de derivadas, SQP e SLSQP recalculam A/B **em todos os pontos da
trajetória atual**. Não reutilizam a linearização fixa do equilíbrio. O erro
angular `e` é periódico; há descontinuidades nos cortes em ±π.

- **iLQR:** backward pass com aproximação quadrática, seguido de rollout com
  feedback e line search. Explora a estrutura temporal do controle.
- **SQP Gauss–Newton:** single shooting, com os estados eliminados pelo rollout
  não linear. Propaga sensibilidades `S_(k+1) = A_k S_k + B_k E_k`, calcula o
  gradiente de J e `H = R I + Σ S_kᵀ Q S_k + S_Nᵀ Qf S_N`. Ignora termos de
  segunda derivada da dinâmica. Cada iteração resolve um **novo** QP com Clarabel,
  Hessiana `H + λI` e limites nos incrementos de torque. A line search Armijo
  avalia novamente o custo não linear. Não é a implementação linear active set.
- **SLSQP:** SciPy recebe o custo do rollout não linear e seu gradiente atualizado
  pela mesma cadeia de A/B. Constrói seus subproblemas SQP e mantém uma aproximação
  quase-Newton da Hessiana. Limites de torque entram como bounds. As equações
  da dinâmica são satisfeitas por construção no single shooting, não impostas
  como restrições fixas lineares. **Sem clipping ele continua executando SLSQP.**

Cada solve guarda `linearizations`: número da avaliação, hash da trajetória,
maior mudança de A entre avaliações e normas de A no início/fim do horizonte.
A interface permite abrir essa auditoria. Repetir o hash é legítimo ao verificar
um ponto já avaliado. A contagem inclui a verificação final e não equivale ao
número de iterações aceitas. Testes independentes com diferenças centrais validam
os Jacobianos RK4 e o gradiente do custo completo.

### Condições comuns e diferenças que permanecem

Q, R, horizonte, limite físico, regra de initial guess e Qf são comuns aos três
NMPCs. O primeiro initial guess e Qf são verificados entre os dados dos métodos.
A inicialização compara torque zero e rollout LQR local; pode tentar direct
collocation para desvios maiores que 0,25 rad. Seu WallTime entra no primeiro
solve. `initialize_only` seleciona o plano sem realizar uma otimização iLQR prévia.
Após o primeiro comando, cada método desloca sua própria solução como warm start:
os estados medidos e os problemas resolvidos em malha fechada podem divergir.

Por padrão, horizonte 2 s (40 passos), 80 iterações no primeiro solve e seis
online. O desafio 05 usa 0,2 s e terminal Q. Os desafios 07/08 mudam também
inicialização e orçamento; não são ablações de um único parâmetro.

- Qf padrão: DARE do RK4 local. O custo de iLQR de plano único cobre seis segundos;
  não se compara diretamente seu valor com o de um MPC de dois segundos.
- SQP/SLSQP normalizam o objetivo por `max(1, J_inicial/N)`. SLSQP usa
  `y = u/max(1, ||u_inicial||∞)`, bounds transformados e `ftol = 1e-9` no laboratório.
  Essas escalas positivas preservam o minimizador; afetam as tolerâncias numéricas.
- SQP pré-condiciona cada QP pela diagonal da Hessiana; Clarabel usa tolerâncias
  de gap/viabilidade de 1e-11 e regularização estática desabilitada. A direção
  retornada é limitada ao intervalo do incremento e reavaliada no rollout.
- O SQP limita cada incremento a 10 N·m quando não há clipping, ou a
  `max(1, 2·limite)` quando há. É um limite de passo por iteração, não uma
  limitação do torque acumulado do atuador.
- λ atua no backward pass do iLQR e na Hessiana Gauss–Newton do SQP de formas
  diferentes. O SLSQP do SciPy não recebe esse λ: mantém sua atualização interna.
  No desafio 08, portanto, a regularização excessiva não é aplicada igualmente.
- Single shooting pode ter sensibilidades mal condicionadas para dinâmicas
  instáveis e horizontes longos. iLQR organiza o cálculo com uma recursão temporal.
  Resultados desta implementação não provam superioridade de uma família inteira.

### Como entender o caso 08: damping numérico

É um freio nas alterações que o otimizador propõe ao plano de controles. Uma
regularização adequada pode tornar o cálculo mais robusto; em excesso, pode
reduzir tanto os passos que o plano melhora pouco no orçamento disponível.
Não acrescenta atrito à planta nem implica que o torque aplicado seja pequeno:
o plano inicial já pode conter torques grandes.

No SQP, o modelo do passo recebe `(λ/2) ||δU||²`. Sem limites e sem line search,
a solução desse modelo é `δU = −(H + λI)⁻¹ g`. Por exemplo, em um problema escalar
com `g = −4` e `H = 2`, o passo é 2 para λ = 0 e 0,04 para λ = 98: fica 50 vezes
menor. É uma ilustração matemática, não um resultado medido da simulação.
R penaliza o torque planejado; λ regulariza a alteração proposta. O rollout é
avaliado com o custo original. No iLQR, λ atua na recursão do backward pass.

Compare o caso 08 com o 02, mantendo a mesma opção de torque:

| Ajuste | Caso 02 | Caso 08 |
| --- | --- | --- |
| λ inicial de iLQR/SQP não linear | 0,00001 | 1.000.000 |
| Initial guess do primeiro solve | Automático, com possibilidade de direct collocation | Melhor rollout entre torque zero e LQR local, sem direct collocation |
| Iterações máximas do MPC, primeiro/seguintes | 80/6 | 12/2 |

Estado inicial (desvio de 1 rad em cada junta), Q, R e horizonte são iguais.
Como três ajustes mudam juntos, não se pode atribuir uma falha somente a λ.
O SLSQP não linear recebe a mudança de initial guess e orçamento, **mas não esse
λ**; mantém sua atualização interna da Hessiana. LQRs e referências MPC lineares
mantêm a configuração do caso 02. O plano único iLQR também muda de 80 para
12 iterações. A página explica o efeito apenas nos métodos da visão selecionada.

Veja a redução do custo do plano retornado e também os erros de posição e
velocidade. Pouca melhoria do plano não é o mesmo que estabilização física.

### Parada, custo retornado e estabilização

SQP/SLSQP retornam o melhor rollout finito e viável **efetivamente avaliado**,
incluindo o initial guess e tentativas de line search. O histórico SQP guarda
passos aceitos; o do SLSQP guarda iterados, que podem aumentar o custo. O ponto
laranja é o custo do plano realmente retornado, não uma iteração extra.
`initial_cost`, `candidate_cost` e `returned_cost` distinguem esses resultados.

Para os dois novos métodos, `converged` verifica o gradiente projetado:

```text
||u − clip(u − ∇J / escala_objetivo, −limite, +limite)||∞ ≤ 1e-5
```

Esse é um teste de estacionariedade local, sem certificado de ótimo global.
O status nativo do solver fica separado. O iLQR usa seus próprios critérios de
atualização/redução de custo; percentuais de “convergência” não são testes idênticos
entre os três. Em problemas mal condicionados, custo quase igual ao melhor encontrado
pode coexistir com gradiente acima da tolerância. O certificado de gap dual dos antigos QPs convexos **não** se aplica
aos novos problemas não lineares.

Estabilização física exige cada erro angular < 0,05 rad e cada velocidade
< 0,1 rad/s por pelo menos um segundo até o fim da janela de seis segundos.
O tempo é o início desse trecho contínuo final. Não atingir essa condição não
prova instabilidade assintótica, assim como o status do solver não prova equilíbrio.

## Planta, gráficos e WallTime

A planta real usa o Simulator do Drake, precisão alvo 1e-9 e comando constante
entre amostras de 50 ms. Planta e preditores usam os mesmos parâmetros físicos,
sem ruído. Perturbações são saltos de velocidade; não são impulsos de torque
integrados por um intervalo finito. Não há restrições de posição ou velocidade.

LQRs e referências MPC lineares usam Euler no equilíbrio; o preditor não linear
usa RK4. O gráfico de resíduo linear usa o modelo Euler como referência em todas
as visões: não é o erro do preditor RK4 dos NMPCs. Sua norma mistura rad e rad/s,
inclui discretização/não linearidade e pode apresentar picos por wrapping angular.
Saltos de velocidade conhecidos são descontados nesse diagnóstico.

A simulação aguarda o solver. Atrasos medidos não são injetados na física:
**os vídeos não demonstram execução em tempo real**. WallTime inclui cálculo e
esperas; não é tempo simulado nem somente CPU. Mediana/p95 online excluem o
primeiro solve dos MPCs; são calculadas entre comandos de uma única execução,
não entre repetições estatísticas. Carga da máquina pode afetá-las.

Configurações exatamente iguais dentro da mesma geração reutilizam o experimento
medido, identificado por `measurement_reused_from`. Isso ocorre, por exemplo,
quando o limite original já é ±3 N·m ou quando os desafios 09/10 diferem somente
no terminal do LQR finito. Não são apresentadas como repetições independentes.
Os controladores lineares preservam suas medições da geração anterior; comparações
absolutas de WallTime entre gerações não constituem um benchmark controlado.

O replay interpola amostras de 50 ms. Torque ilimitado pode produzir movimentos
rápidos demais para essa representação; a interface avisa sobre aliasing.

## Reprodução

A partir da raiz do projeto, execute simulações/testes no contêiner Drake:

```sh
docker exec -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/control_lab/generate.py --variants original clipped unclipped'
docker exec -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 lunar-rover-jazzy-drake fish -c 'cd ~/drake; python -m unittest discover -s acrobot -p "test_*.py"'
docker exec -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/control_lab/validate.py'
```

`generate.py` importa `lqr.py`, `ilqr.py`, `nonlinear_sqp.py` e `linear_mpc.py`.
Não lê nem altera o YAML. Guarda configurações e hashes de origem em `runs/`.
`--force` força recálculo; `--scenarios near --modes mpc_nonlinear_sqp
mpc_nonlinear_slsqp` restringe o trabalho. `--export-only` recompõe `data.json` e
`data.js`. Use as três variantes antes de distribuir o laboratório completo.

No host, requer Python com NumPy/Pillow, Node e ffmpeg:

```sh
python3 drake/acrobot/control_lab/render_videos.py --force
python3 drake/acrobot/control_lab/build_standalone.py
node drake/acrobot/control_lab/test_ui.cjs
node drake/acrobot/control_lab/test_ui.cjs --standalone
```

Os MP4 são reconstruções 2D dos estados do Drake, não capturas Meshcat. São três
famílias por configuração: controladores, referência linear (`_solvers`) e
solvers não lineares (`_nonlinear_solvers`). O HTML independente incorpora todos.
Para hospedagem, `index.html` carrega os vídeos sob demanda. O renderizador usa
três processos por padrão; `--workers 1` permite gerar os vídeos em sequência.

As verificações incluem testes dos Jacobianos/gradientes, atualização da trajetória,
warm start, limites, acordo local entre métodos e replay independente dos comandos
no Drake. O teste JavaScript usa DOM/canvas simulados; não substitui revisão visual
e playback em um navegador real.

## Benchmark preservado: QPs lineares

O arquivo `solver_comparison.json` compara active set e SLSQP **no modelo linear**.
São 66 pares congelados, sete repetições com ordem alternada, mesma entrada e
referência Clarabel independente. Alguns desafios repetem o mesmo problema;
não são 66 problemas independentes. Esse benchmark não mede os novos NMPCs.

A formulação linear usa completion of squares por Riccati para obter Hessiana
identidade. SLSQP normaliza variáveis, usa `ftol=1e-13`, projeta pequenas violações
de torque e reavalia o rollout linear. O melhor candidato viável é retornado;
status nativo, reparo/fallback e certificado do QP ficam separados. O certificado
verifica estacionariedade, viabilidade primal e gap dual relativo em 1e-6, além
de violação física ≤ 1e-8 N·m. Sem clipping, as três referências lineares usam a
mesma solução LQ analítica; essa simplificação não é usada nos novos NMPCs.

Após a correção anterior, ambos passaram nos 66 pares. Maior diferença em u₀:
9,57 × 10⁻⁹ N·m. Contra Clarabel, maior diferença do plano: 1,63 × 10⁻⁶ N·m;
erro relativo de custo: 8,08 × 10⁻⁸. Medianas pareadas: 2,54 ms (active set) e
1,05 ms (SLSQP). Seis status nativos SLSQP sem sucesso acompanharam planos que
passaram no certificado independente; nenhum fallback foi necessário.

O critério predefinido para separar curvas era Δu₀ > 0,1 N·m, discrepância no
certificado em ≥5% dos pares ou razão de medianas ≥3 com diferença ≥1 ms.
Não foi atingido; a curva linear SLSQP fica opcional. Pode-se reproduzir esse
benchmark, sem outras simulações concorrentes, com `compare_solvers.py` no
contêiner e depois `generate.py --export-only`.

A versão anterior descartava planos quase viáveis e podia trocar um candidato
−3,000048 N·m por um initial guess +3 N·m, enquanto o gráfico omitia o custo do
fallback. Ela não demonstrava inferioridade do SLSQP. A correção desse problema
permanece preservada; o novo experimento acrescenta a dinâmica não linear.

## Verificações desta entrega

- 33 testes passaram no contêiner Drake, incluindo Jacobianos, gradiente do
  rollout, limites, atualização da trajetória, warm start e comparação local.
- As 297 trajetórias foram reproduzidas independentemente a partir dos comandos
  salvos no Drake; diferença máxima de estado igual a zero.
- Previsões RK4 e seus custos foram recalculados. Initial guesses e matrizes
  terminais do primeiro solve coincidiram nos três NMPCs em todas as configurações.
- Os testes de interface passaram nas 132 combinações (11 desafios × três
  torques × quatro visões), nas versões modular e independente, incluindo os
  diagnósticos não lineares e a regressão de custo/fallback do QP linear. Usam
  DOM/canvas simulados; não são uma revisão de layout em navegador real.
- Os 99 MP4 foram decodificados sem erros: H264, 1280 × 720, 30 fps, 181 quadros
  cada. Um quadro codificado da nova comparação foi inspecionado visualmente.

## Resultados selecionados desta geração

Tempo de estabilização com ±3 N·m; “não atingida” significa que a condição
não foi mantida por pelo menos um segundo até o final dos seis segundos.

| Desafio | NMPC · iLQR | NMPC · SQP | NMPC · SLSQP |
| --- | --- | --- | --- |
| 01 · Perto do equilíbrio | 1,70 s | 1,70 s | 1,70 s |
| 02 · Longe do equilíbrio | 2,75 s | 2,75 s | 2,75 s |
| 11 · A trajetória faz diferença | 3,90 s | Não atingida | 4,55 s |

Esses valores descrevem as configurações registradas, não garantias gerais.
Por exemplo, no desafio 01 sem clipping o SLSQP não estabilizou com este orçamento,
apesar de estabilizar com ±3 N·m. Consulte os custos, status e WallTime dos solves
na interface; limites podem mudar o condicionamento e o caminho da otimização.

## Fontes e termos

[MIT · Trajectory Optimization](https://underactuated.mit.edu/trajopt.html),
[MIT · LQR](https://underactuated.mit.edu/lqr.html),
[SciPy · SLSQP](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html),
[Drake · AcrobotPlant](https://drake.mit.edu/pydrake/pydrake.examples.html).

A interface usa WallTime, solver/solve, deadline, initial guess, warm start,
direct collocation, receding horizon, rollout, backward pass, line search,
feedback, clipping e damping, com explicações em português no glossário.
