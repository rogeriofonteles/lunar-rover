# MuJoCo six-axis arm

An independent Docker environment, following the Drake directory/container
workflow. The local MJCF arm has six revolute joints and six torque motors;
its meshes and textures are included, with no model downloads required. MuJoCo 3.3.7, NumPy, SciPy,
PyYAML, and Pillow are installed in `/opt/mujoco-env` (Python 3.12).
The `jazzy` name follows this repository's naming convention; this image
does not require ROS or Drake.

## Build and start

From the repository root:

```sh
./docker/scripts/build_image.sh mujoco
export USERNAME="$(whoami)" PROJECT_ROOT="$PWD"
docker compose -f docker/lunar_rover_jazzy_mujoco/docker-compose.create-container.yaml up -d
docker exec -it lunar-rover-jazzy-mujoco fish
```

The existing `create_jazzy_docker_container.sh mujoco` also works; its final
`docker logs -f` stays attached until Ctrl-C. The container keeps running.
Use `docker start lunar-rover-jazzy-mujoco` to restart an existing container.

To build directly with Docker Compose (the helpers also work without `yq`):

```sh
USERNAME="$(whoami)" MY_UID="$(id -u)" MY_GID="$(id -g)" \
  docker compose -f docker/lunar_rover_jazzy_mujoco/docker-compose.build-image.yaml build
```

Source is included in the image and bind-mounted at `~/mujoco` for development.
Rebuild after dependency changes; code edits are visible immediately in the
running development container. Host ownership is preserved. The default Compose
configuration enables NVIDIA GPU rendering with `runtime: nvidia`, `gpus: all`,
and `NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute`. It requires a
working host NVIDIA driver and NVIDIA Container Toolkit registered with Docker.
The runtime injects graphics libraries matching the host driver; no NVIDIA
driver or CUDA toolkit needs to be installed inside this image. A privileged
container is not required. Host IPC sharing remains enabled for the optional
Mesa/X11 fallback.

**Rendering uses the GPU; MuJoCo physics, LQR, iLQR, and MPC remain CPU code.**
This does not convert the simulation to MJX or move SciPy optimization to CUDA.
The setup was verified with a GeForce RTX 2070 SUPER and driver 580.178.04.

Check actual rendering, rather than relying only on GPU visibility:

```sh
docker exec lunar-rover-jazzy-mujoco nvidia-smi
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python -m arm.check_graphics --require-nvidia'
# Headless NVIDIA EGL context (no X11 display required):
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python -m arm.check_graphics --backend egl --require-nvidia'
```

Both graphics checks should report `NVIDIA Corporation` and the GPU name.
See [NVIDIA's driver-capability documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html)
and [Docker Compose GPU support](https://docs.docker.com/compose/how-tos/gpu-support/).

## Run

```sh
# Headless default infinite-horizon LQR
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python simulate.py --headless --no-wait'

# Interactive viewer: authorize the same host/container user for X11 first
xhost +si:localuser:"$(whoami)"
docker exec -it lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python simulate.py'
```

Inside the container, from `~/mujoco`:

```sh
python simulate.py --mode infinite --discretization zoh
python simulate.py --mode finite --horizon 100 --discretization euler
python simulate.py --mode receding --horizon 60
python simulate.py --mode ilqr --horizon 100
python simulate.py --mode ilqr_receding --horizon 40 --steps 200
python simulate.py --mode mpc --solver dual_ascent --horizon 40
python simulate.py --mode mpc --solver sqp --horizon 40
python simulate.py --mode mpc --solver slsqp --horizon 40
python simulate.py --help
```

Append `--headless --no-wait` for automated runs. `--no-realtime` disables
viewer pacing. By default the final pose remains visible until the viewer
closes; `--no-wait` exits after execution. The viewer needs a host X11/XWayland
display. Headless numerical simulation does not need a display or OpenGL.

### Reach the colored buttons

Three reachable buttons sit on separate stands around the arm: red is lower
and forward, yellow is high, and green is on the opposite side. Their centers
are more than 0.9 m apart, with 0.48 m between the lowest and highest targets.
From `~/mujoco` inside the container, select a recorded joint-space goal:

```sh
python simulate.py --button red
python simulate.py --button yellow
python simulate.py --button green
# The same target selection works with the existing controller options:
python simulate.py --button red --mode mpc --solver sqp --horizon 20
```

Every invocation starts at the same fixed randomized `initial_state` from
`arm/config.yaml`, with zero initial velocities. Choosing a button only changes
the target; it does not reuse the last run's final pose. Explicit `--initial-state`
or custom YAML can still override the start. The default infinite-horizon LQR
reaches all three buttons within the default 200 steps (four simulated seconds).
Other controllers and horizons retain their existing convergence behavior.

[`arm/button_targets.json`](arm/button_targets.json) records each button's world
position in meters and its six joint angles in radians, ordered `j1` through `j6`.
Each goal appends six zero velocities. These configurations place the added
stylus's `button_tip` site at the center of the cap, with the tool axis pointing
into the button face. The scene generator reads button positions from the same
file. The buttons are visual positioning targets: there is no physical switch
travel, contact force control, or collision avoidance.

`button: red`, `yellow`, or `green` also works in YAML and takes precedence over
the YAML `target_state`; `button: null` retains the ordinary target. CLI `--button`
and `--target-state` are mutually exclusive. A button run uses the `buttons`
camera for snapshots and a matching interactive viewpoint. Its console/JSON
summary includes the selected button, initial/target states, button position,
and final tip error in meters; `--output output/red` saves the trajectory too.

Record all three demonstrations inside the container:

```sh
python record_buttons.py
# Or record just one:
python record_buttons.py --button red
# Record all three with MPC, SQP, and a 20-step horizon:
python record_buttons.py --mode mpc --solver sqp --horizon 20 \
  --output-dir output/button-videos-mpc-sqp-h20
```

This writes `output/button-videos/{red,yellow,green}.mp4` at 1920×1080, 30 fps,
with the same camera and initial pose. Videos use infinite-horizon LQR by default;
`--mode mpc --solver sqp --horizon 20` selects MPC instead. The overlay identifies
the controller, solver, and MPC horizon, also saved in JSON metadata. Each video
plays the trajectory at simulated real time, and includes one second at the
start and two seconds at the goal. The recorder also saves trajectory arrays,
JSON metadata, and start/final PNGs. It uses NVIDIA EGL for rendering and FFmpeg
for H.264 encoding; FFmpeg is included in the Dockerfile. Rebuild older images
to install it. `--width`, `--height`, and `--fps` control recording quality.

If an older running container prints `failed to load driver: nvidia-drm`,
`glx: failed to create dri3 screen`, or an X11 `MIT-SHM` / `X_ShmPutImage`
error, recreate it with the updated Compose configuration. From the host
repository root (this closes shells attached to the old container):

```sh
export USERNAME="$(whoami)" PROJECT_ROOT="$PWD"
docker compose -f docker/lunar_rover_jazzy_mujoco/docker-compose.create-container.yaml up -d
docker exec -it lunar-rover-jazzy-mujoco fish
```

Remove any previously saved Fish software-rendering overrides:

```fish
set -eU LIBGL_ALWAYS_SOFTWARE
set -eU GALLIUM_DRIVER
```

For manual GPU `docker run`, pass `--gpus all --runtime nvidia`,
`-e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute`,
`-e __GLX_VENDOR_LIBRARY_NAME=nvidia`, plus the X11 socket mount and `DISPLAY`.
For a one-off software-rendering fallback inside the running container:

```sh
env __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe python simulate.py
```

Software X11 presentation requires the existing `ipc: host` setting. For a
machine without NVIDIA, run the image directly without GPU arguments for
headless numerical simulation, or use that Mesa environment and `--ipc=host`
with the display/socket for a viewer. The default Compose service requires NVIDIA.

Edit [`arm/config.yaml`](arm/config.yaml) for defaults, or pass
`--config path/to/custom.yaml`. Partial custom files inherit the checked-in
defaults; explicit CLI options override YAML. Unknown YAML keys are rejected.
`Q`, `R`, and `terminal_cost` accept diagonal lists or full symmetric matrices.
`R` must be positive definite. CLI `--Q-diagonal` and `--R-diagonal` accept
12 and 6 numbers. `torque_limit` accepts six bounds or one scalar in YAML;
`--torque-limit 30` applies a shared magnitude. Bounds are always enforced.

## Planejamento entre botões e relatório em PT-BR

O novo fluxo planeja uma rota com RRT-Connect e acompanha uma referência de
posição/velocidade, mantendo o estado contínuo entre botões:

```sh
python plan_buttons.py --sequence red yellow green red --mode mpc --solver sqp --horizon 20
python plan_buttons.py --mode finite
python plan_buttons.py --mode ilqr_receding --horizon 20
python plan_buttons.py --headless --video output/rota.mp4
```

Abra [`path_planning_lab/index.html`](path_planning_lab/index.html) para ver a
comparação em PT-BR, os gráficos e os vídeos renderizados na cena do MuJoCo.
Veja [`path_planning_lab/README.md`](path_planning_lab/README.md) para reproduzir
os resultados e entender a aproximação geométrica de colisão. A rota começa
no primeiro botão; o fluxo original `simulate.py --button ...` não muda.

## Industrial workcell

The orange-and-graphite arm uses custom tapered casting meshes, machined
joint covers, fasteners, service panels, cable routing, and a two-finger
gripper illustration. It sits on a steel machine pedestal inside a paneled
workshop, with factory glazing, overhead lights, a tooling bench and vise,
parts bins, a control cabinet, an emergency-stop button, and safety markings.
The cabinet screen is decorative, not a live telemetry display.

The default camera frames the complete cell; drag and scroll in the viewer
to inspect the arm. The viewer requests a maximized window and hides the side
panels so more screen pixels go to the scene. Use `--no-maximize` for a smaller
window or `--viewer-ui` to restore the native control panels. The window manager
and monitor determine the live framebuffer resolution. If your window manager
ignores the maximize request, press **F5** in the viewer for native-resolution
full screen; F5 toggles back. Tab and Shift-Tab toggle the left and right panels.

Rounded castings use dense meshes with smooth normals. Curved primitives use
80 slices and 32 stacks, signs are generated at twice their original texture
resolution, and the floor and metal have subtle material grain. Lighting uses
an 8192-square shadow map and tighter depth ranges. Four-sample anti-aliasing
is enabled and is also compatible with the Mesa software fallback.
Higher visual quality increases rendering work, particularly on software rendering.

Named `overview` and `detail` cameras are included in the model. PNG snapshots
use the overview camera at **2560 by 1920** by default. Override the export size:

```sh
env MUJOCO_GL=egl python simulate.py --headless --no-wait \
  --snapshot output/arm-4k.png --snapshot-width 3840 --snapshot-height 2160
```

Snapshot dimensions affect exported images, not the native viewer window.
The corresponding YAML settings are `maximize`, `viewer_ui`, `snapshot_width`,
and `snapshot_height`. MuJoCo's [render-quality settings](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#visual-quality)
describe the distinction between offscreen and live rendering.

All decorative geometry is massless and contact-free. The original link
masses, centers of mass, inertias, joint transforms, friction, and motors
are retained explicitly. Raising the base onto its pedestal changes only
its world height. The room, vise, and gripper do not add contact or grasping
dynamics to this control benchmark.

The assets are generated locally and stored alongside the MJCF. To change
the design, edit `arm/models/build_scene.py`, then regenerate inside Docker:

```sh
python arm/models/build_scene.py
```

This uses the existing NumPy and Pillow dependencies and Pillow's bundled font, without
external asset downloads. Restart the simulator to load visual changes;
the development container already sees them through its bind mount.

## Dynamics and objective

State is `[q1,...,q6,v1,...,v6]` in radians and radians/second. Input is six
joint torques in N m (unit motor gears). All joints are continuous revolute
joints. State error is an **unwrapped coordinate difference**, so the task
tracks the specified joint coordinates, including revolutions. The model is
a smooth free-space teaching benchmark: it has gravity, inertia, damping,
and motor saturation, but no contact, collision avoidance, or joint limits.
It is not a calibrated model of a commercial arm.

The target must be stationary. `model.py` obtains the equilibrium torque
`u_ref` from MuJoCo's gravity/passive forces and rejects an infeasible target.
Every controller minimizes per-sample quadratic state and torque-deviation
costs, with a terminal state cost:

```text
e = x - x_ref,  v = u - u_ref
J = 1/2 sum(k=0..N-1) [e[k]' Q e[k] + v[k]' R v[k]]
    + 1/2 e[N]' Qf e[N]
```

`Q` and `R` are discrete per-sample weights; selecting ZOH does not discretize
a continuous-time cost integral. Changing the control timestep changes the
physical horizon `N * dt` and may require retuning the weights.

MuJoCo executes RK4 at `physics_timestep` or smaller, holding each torque
constant across the control period. The linear design model comes from
central finite differences of continuous MuJoCo forward dynamics:

* `euler`: `Ad = I + dt Ac`, `Bd = dt Bc`.
* `zoh`: the exact held-input **linear** model, from the matrix exponential
  of `dt * [[Ac, Bc], [0, 0]]`. This is not an exact nonlinear integrator.

## Controllers

| Mode | Behavior |
| --- | --- |
| `infinite` | DARE solution; apply `u_ref - K e` with saturation. |
| `finite` | Backward Riccati recursion; execute `K[0]` through `K[N-1]`, stopping at `min(steps,N)`. |
| `receding` | Apply the finite-horizon `K[0]` every sample for `steps`. The fixed linear problem only needs one Riccati solve. |
| `ilqr` | Optimize a nonlinear trajectory once; execute nominal torques plus time-varying feedback, stopping at `min(steps,N)`. |
| `ilqr_receding` | Re-optimize nonlinear iLQR from each measured state, apply its first torque, shift the plan, and continue for `steps`. |
| `mpc` | Solve a torque-constrained linear prediction problem at every measured state using the selected solver. |

The default terminal matrix `Qf` is the local DARE value matrix. Consequently,
unconstrained finite LQR with this terminal cost gives the same gain as
infinite LQR. Set `terminal_cost` to a different matrix (for example the `Q`
diagonal) to study genuinely time-varying finite gains and horizon effects.
Actuator clipping does not make LQR itself a constrained optimizer.

### Nonlinear iLQR

Like the Drake implementation, iLQR compares a local-LQR seed with a constant
equilibrium-torque seed, performs a regularized backward pass, and accepts
only cost-reducing line-search rollouts. Jacobians differentiate the actual
substepped MuJoCo transition. Euler/ZOH only affect its local LQR seed and
default terminal cost, not its nonlinear prediction dynamics.

The six-input backward pass solves the coupled box quadratic problem; simply
clipping a six-dimensional unconstrained Newton step would be incorrect.
Free actuators retain feedback; active bound rows have zero local feedback.
`ilqr_regularization`, `ilqr_state_damping`, and `ilqr_control_damping` adjust
numerical curvature, not physical damping. Gains are recomputed around the
final accepted plan. A receding solve shifts the prior control plan and
repeats its last torque at the tail. There is no Drake collocation dependency
or collocation initializer here.

### MPC solver comparison

All three solvers use the **same fixed equilibrium linearization, costs, horizon,
and torque bounds**, with dynamics eliminated by condensing into
`min 1/2 v' H v + g' v`, `lower <= v <= upper`. Each sample updates `g` from
the measured nonlinear plant state. Predicted MPC states are linear-model
predictions; actual execution always uses MuJoCo.

* `dual_ascent`: explicit projected coordinate ascent on nonnegative upper
  and lower inequality multipliers. Each coordinate exactly maximizes the
  dual along that coordinate using `H^-1`; controls minimize the Lagrangian.
  The factorization/inverse are cached, and dual multipliers are shifted
  between samples. `mpc_max_iterations` counts full dual coordinate sweeps.
* `sqp`: explicit exact-Hessian SQP specialized to this convex box QP. Its
  quadratic subproblem is solved with a primal active set: solve the reduced
  Newton system on free controls, take a feasible step to the first blocking
  bound, and release bounds whose multipliers have the wrong sign. It uses
  diagonal scaling and the exact Hessian, and does not call SciPy's optimizer.
  Since the MPC objective is already quadratic and constraints are linear,
  the SQP subproblem is the original QP in step coordinates; no nonlinear
  outer iterations are needed. `mpc_max_iterations` counts active-set updates
  and Newton steps, not nonlinear SQP passes. This is a specialized QP solver,
  not a general nonlinear SQP implementation.
* `slsqp`: SciPy SLSQP with analytic objective gradients, box bounds, diagonal
  variable scaling, and shifted primal warm starts. The shifted plan competes
  with a clipped unconstrained seed; the lower-cost feasible seed is used.
  This is SQP applied to
  the same convex QP, not a separate nonlinear MPC formulation.

All independently check the infinity norm of the box projected-gradient
residual `v - clip(v - (Hv+g), lower, upper)` against `mpc_tolerance`.
An iteration limit is reported as nonconvergence; Dual-Ascent returns its
lowest-cost feasible candidate, and SQP/SLSQP return their feasible final iterate.
The controller applies that bounded plan and counts nonconverged solves.
iLQR likewise reports nonconvergence and executes the best accepted plan.
No finite nonlinear rollout is an error, not permission to reuse stale input.

This is local regulation. Nonlinear predictive control is `ilqr_receding`.
None of these solvers has a global maneuver or real-time execution guarantee;
replanning runs synchronously. Dense condensing is intended for modest
horizons (e.g. 20–100), not very large MPC problems.

Before the solver split, `--solver sqp` called SciPy SLSQP. To reproduce that
implementation, now use `--solver slsqp`; `--solver sqp` selects the new explicit
implementation. Previously recorded videos labeled `MPC / SQP` used SLSQP.
The same three solver choices are available in YAML and `record_buttons.py`.

## Results, rendering, and tests

```sh
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python simulate.py --headless --no-wait --output output/lqr.npz'
docker exec -e MUJOCO_GL=egl lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python simulate.py --headless --no-wait --snapshot output/arm.png'
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python -m unittest discover -s tests -p "test_*.py" -v'
```

The NPZ contains measured states, applied controls, times, target, and gravity
compensation torque. Its sibling JSON records state-error norms, peak torque,
timing, optimizer statuses, and residuals. An optimizer's convergence is
distinct from successful tracking; inspect both. Outputs are ignored by Git.

Tests cover analytic discretization, Riccati identities, gravity compensation,
deterministic dynamics, nonlinear closed-loop stabilization, bounded iLQR,
coupled actuator limits, MPC solver agreement under active constraints,
unconstrained MPC/LQR equivalence, nonconvergence reporting, warm starts,
execution duration, configuration validation, and exported trajectories.

Refresh the host editor's MuJoCo sources/type stubs with:

```sh
python3 scripts/sync_python_editor_libs.py --only mujoco # from repository root
```

Implementation references: [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/3.3.7/python.html),
[MuJoCo functions](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html),
[SciPy discretization](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.cont2discrete.html),
and [SciPy SLSQP](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html).
