# Acrobot LQR

Run from the `drake` directory in a Python environment with Drake installed
(for example `cd ~/drake` inside the Drake container):

```sh
python acrobot/acrobot_test.py --mode infinite
python acrobot/acrobot_test.py --mode finite --horizon 100
python acrobot/acrobot_test.py --mode receding --horizon 100 --steps 1000
```

- `infinite`: use a constant infinite-horizon gain (default).
- `finite`: apply the time-varying sequence K[0] through K[N-1]. Simulation
  ends at the horizon, or earlier if `--steps` is smaller. There is no
  automatic controller switch after the endpoint.
- `receding`: apply K[0] for an N-step horizon at every control step.
  With the fixed linear model, costs, and horizon used here, K[0] is identical
  at every solve and is computed once. Feedback still uses the current state.

`--horizon` counts control steps; duration is horizon × `--timestep`.
Defaults: horizon 100, timestep 0.1 seconds, maximum 1000 execution steps.
Use `--headless` for simulation without Meshcat, or `--no-wait` to exit
without waiting for Enter. Use `--help` for all options.

Torque clipping is enabled by default at **±3 N·m** for all controller modes.
Edit `torque_limit_nm` in `lqr.yaml` to change the maximum magnitude, or pass
`--config /path/to/config.yaml` to use another YAML file. YAML loading requires
PyYAML. Use `--no-torque-clipping` to disable saturation.
Python callers can use `run_simulation(..., clip_torque=False)` or
`run_simulation(..., clip_torque=True, config_path="/path/to/config.yaml")`.
The settings are loaded once per run. Clipping saturates the applied command;
the LQR gains are still computed using the unconstrained quadratic problem.

These controllers balance near upright. They use the same finite-difference
linearization and forward Euler discretization as the original example.
Short horizons can fail to stabilize the nonlinear plant: the 20-step horizon
lost balance in a smoke check, while 100 steps balanced the default initial
state. Finite-horizon terminal cost defaults to Q and does not guarantee a
particular final state or arrival time.

Files:

- `acrobot_test.py`: Drake simulation, visualization, and mode selection.
- `lqr.py`: shared model and Riccati step, infinite gain, finite gain sequence.
  `finite_horizon_lqr(..., return_sequence=True)` returns shape `(N, 1, 4)`;
  its default returns only K[0] for compatibility. Python callers can provide
  `Q`, `R`, and `terminal_cost` to customize the costs.
- `lqr_comparison/`: existing comparison tools, plots, recordings, and videos.

Run the controller tests from `drake`:

```sh
python -m unittest discover -s acrobot -p 'test_*.py'
```

All simulation settings can be edited in `lqr.yaml`: `mode`, `horizon`,
`timestep`, `steps`, `initial_state`, `clip_torque`, `torque_limit_nm`,
`headless`, and `wait`. Run without flags to use that file, or choose one
with `--config`. Explicit CLI flags override YAML; omitted keys use built-in
defaults. Python function arguments also override YAML when not `None`.

`initial_state` is `[theta1, theta2, theta1dot, theta2dot]`, with angles in
radians and velocities in rad/s. The upright target is `[pi, 0, 0, 0]`.
For example:

```sh
python acrobot/acrobot_test.py --mode finite --horizon 100 --initial-state 3.1 0.05 0 0 --torque-clipping
python acrobot/acrobot_test.py --mode infinite --initial-state 3.1 0.05 0 0 --no-torque-clipping
```

Use `--torque-clipping` / `--no-torque-clipping`, `--headless` /
`--no-headless`, and `--wait` / `--no-wait` to override YAML booleans in
either direction. `--torque-limit-nm 5` overrides the configured magnitude.

The Meshcat model uses a mechanical lab-robot illustration: slotted metal
links, an orange upper arm, a passive shoulder bearing, an elbow motor and
gearbox, fasteners, a cable, and a bench-edge mounting fixture. The default
camera frames the upright balancing experiment; drag to orbit or scroll to
zoom when inspecting other poses.

`visuals.py` shares this appearance with the legacy `../acrobot_test.py`
entry point. It replaces only illustration geometry on Drake's original
link frames. Plant parameters, dynamics, collision behavior, and controllers
are unchanged. All geometry is generated locally; no downloaded assets or
extra Python dependencies are needed.

### Nonlinear iLQR

Select `mode: ilqr` or `--mode ilqr` to optimize a finite nonlinear trajectory
from the configured initial state. It executes at most `horizon` controls,
with feedback around the optimized states and torques, then ends. It does
not replan each step. At timestep 0.01, horizon 100 covers only one second;
choose the horizon to cover the intended maneuver duration.

```sh
docker exec -it lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/acrobot_test.py --mode ilqr --horizon 500'
```

The optimizer uses RK4 with internal steps at most 0.01 seconds and finite
differences of that nonlinear discrete model. It minimizes quadratic state
error and torque costs (Q = diag(10, 10, 1, 1), R = 1). The default iLQR terminal
cost is the local discrete Riccati value matrix computed from the RK4 model,
so it accounts for balancing beyond the endpoint (Q is the fallback if no
stabilizing local solution is available).
The Python `iterative_lqr` API in `ilqr.py` also accepts custom costs and an
initial control sequence. By default, the optimizer compares zero controls
with a local infinite-horizon LQR feedback rollout and chooses the lower-cost
finite trajectory. Explicit initial controls bypass this seed selection.
Angles use wrapped errors; the objective has a
branch boundary at ±pi, so global swing-up convergence is not guaranteed.

`clip_torque` and `torque_limit_nm` also apply during optimization. For this
single-input plant the backward pass clips the scalar feedforward update
and disables feedback on an active bound; rollout and execution enforce the
same bounds. This is a local constrained optimization, not a guarantee of
stabilization under the chosen torque limit.

The following YAML keys have matching CLI flags (replace underscores with
hyphens): `ilqr_max_iterations`, `ilqr_tolerance`, `ilqr_regularization`,
`ilqr_state_damping`, and `ilqr_control_damping`. Regularization adaptively
adds positive curvature to the value-state and control Q_uu Hessians. State
and control damping scale those additions; they do not change physical
friction or the configured state-cost Q matrix. A backtracking line search
accepts only cost-reducing rollouts. The optimizer reports convergence,
iteration count, cost change, and a termination reason. If it reaches its
iteration limit or fails line search, simulation explicitly reports that it
is executing the best accepted plan without achieved convergence.

### Receding-horizon iLQR

Set `mode: ilqr_receding` (selected in `lqr.yaml`) or pass
`--mode ilqr_receding`. At every control sample the controller:

1. Reads the simulated plant's measured state.
2. Solves the nonlinear iLQR problem for `horizon` future controls.
3. Applies only the first optimized torque and advances one `timestep`.
4. Shifts the optimized controls left, repeats the final torque at the tail,
   and uses that sequence to initialize the next solve.

Unlike finite `ilqr`, execution lasts `steps * timestep` even when `steps`
exceeds `horizon`. For the current YAML, the controller looks ahead three
seconds and replans every 0.01 simulated seconds. The initial seed and the
terminal cost follow the same rules as finite iLQR; the terminal matrix is
cached after the first solve. Torque clipping applies to every optimization
and applied command when enabled.

```sh
docker exec -it lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/acrobot_test.py --mode ilqr_receding'
```

`ilqr_max_iterations` limits **each replan**, not the total simulation.
The existing tolerance, regularization, and damping settings apply to every
solve. Replanning is synchronous: simulation waits for optimization, so a
0.01-second simulated control period does not imply real-time computation.
The console reports the first replan, periodic status, and total solves
without achieved convergence. Such solves use their best accepted finite
plan; a solve with no finite rollout raises an error instead of reusing a
stale command. This local nonlinear MPC does not guarantee global swing-up
or recovery under arbitrary state and torque constraints.

### Initialization for larger disturbances

`ilqr_initialization: auto` / `--ilqr-initialization auto` also considers a
Drake direct-collocation seed for the first solve when either initial angle
error exceeds 0.25 rad. It solves for a trajectory ending at upright over the
configured horizon, then uses time-varying feedback to convert its continuous
inputs into a dynamically consistent sample-and-hold RK4 rollout. Simply
sampling collocation torques without feedback can diverge during conversion.
The lowest-cost finite seed is passed to iLQR. Later receding solves continue
to warm-start from the shifted accepted plan, so collocation is not run at
every timestep.

`ilqr_initialization: lqr` keeps the previous zero/local-LQR candidates only;
`collocation` always attempts a collocation candidate on the first solve.
An unsuccessful forced collocation initialization raises; `auto` reports a
warning and falls back to local candidates. Explicit Python `initial_controls`
bypass seed generation. Collocation requires an available Drake nonlinear
solver and adds startup work. It respects configured torque bounds; success
with clipping disabled does not establish feasibility at ±3 N m. This
initializer improves the local solver's starting trajectory but does not
turn iLQR into a globally convergent method.

The simulation YAML now exposes the state-cost matrix `Q` (a symmetric
positive-semidefinite 4 by 4 matrix) and positive scalar torque cost `R` for
all controller modes. CLI `--Q-diagonal 10 10 1 1` and `--R 0.01` override
them. The current larger-disturbance configuration uses `R: 0.01` instead of
1 to permit stronger corrective torques with clipping disabled. The
collocation initializer uses a torque penalty of at least 1 to condition its
feasibility search; subsequent iLQR solves use the exact configured Q and R.
Progress and final output include actual state error, independently of
optimizer convergence, and the final report includes peak applied torque.

### Laboratório interativo em PT-BR

Abra [control_lab/acrobot_interativo.html](control_lab/acrobot_interativo.html)
no navegador: um único arquivo offline com onze cenários, 44 execuções reais
no Drake, gráficos sincronizados e onze vídeos comparando LQR finito/infinito,
iLQR e MPC não linear. A interface distingue estabilização física,
convergência do solver e viabilidade do período de controle.

Os experimentos são pré-calculados. Código, dados brutos, procedimentos de
reprodução e limites de validação estão em
[control_lab/README.md](control_lab/README.md).
