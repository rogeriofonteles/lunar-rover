# Laboratório interativo do Acrobot — PT-BR

Abra **index.html** no navegador. HTML, CSS, JavaScript, dados e vídeos são locais;
não há CDN, telemetria ou servidor de simulação. Mantenha esta pasta inteira
para preservar os vídeos e os dados. Opcionalmente, sirva a pasta:

```sh
python3 -m http.server 8765 --directory drake/acrobot/control_lab
```

Abra `http://localhost:8765`. O parâmetro `?cenario=far` abre um cenário específico.

## O que explorar

- Onze cenários, quatro controladores por cenário, seis segundos de física em cada execução.
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
docker exec -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/control_lab/generate.py'
```

O gerador importa os controladores existentes de `acrobot/lqr.py` e `acrobot/ilqr.py`.
Não lê nem altera `lqr.yaml`. A planta real é `AcrobotPlant`, integrada pelo
`Simulator` com precisão alvo 1e-9 e torque constante entre amostras de 0,05 s.

Cada execução fica em `runs/<cenario>_<metodo>.json`, com hash da configuração
e das fontes. Dados válidos são reutilizados; `--force` força o recálculo.
`--scenarios near far --modes finite infinite` restringe o trabalho.
`--export-only` recompõe `data.json` e `data.js` a partir dos resultados existentes.
Execute sem filtros para produzir o conjunto completo antes de distribuir o HTML.

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
- Ambos os LQRs têm saturação posterior ao projeto. iLQR/MPC consideram o limite
  escalar no backward pass e no rollout. Não há restrições de posição/velocidade.
- iLQR/MPC usam RK4 (subpassos ≤ 0,01 s) e terminal DARE local por padrão.
  Há diferenças de discretização e terminal em relação ao LQR: a comparação
  não isola exclusivamente o nome do algoritmo.
- A inicialização automática pode usar direct collocation do Drake para desvios
  maiores. Esse custo está incluído na primeira solução, e o resultado registra
  o initial guess selecionado e avisos de fallback.
- “Convergido” é o critério numérico implementado no solver local. Não é um
  certificado de ótimo global, de condições KKT exatas ou de estabilidade.
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
- As medidas de WallTime são de uma execução por caso, nesta máquina, com possíveis efeitos de
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

`acrobot_interativo.html` incorpora CSS, JavaScript, dados e os onze vídeos.
Esse arquivo pode ser movido sozinho e aberto sem internet. Para reconstruí-lo:

```sh
python3 drake/acrobot/control_lab/build_standalone.py
```

Verificações realizadas nesta entrega:

- 19 testes existentes do Acrobot passaram no contêiner Drake.
- 44 execuções reproduzidas independentemente no Drake a partir dos torques salvos,
  com diferença máxima de estado igual a zero.
- Validação de limites, custos aceitos, métricas de convergência, prazos e
  equivalência LQR finito com terminal P∞ / LQR infinito.
- Teste das interações JavaScript com um DOM simulado (`node test_ui.cjs`).
- Vídeos decodificados por ffmpeg e quadros inspecionados.
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
