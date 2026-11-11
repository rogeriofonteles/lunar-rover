# UR5 STOMP benchmark without ROS

For the subsequent **nine-joint X/Z/yaw gantry experiment** (2 m X travel,
70 cm Z travel, ±180° yaw, arm base pointing +X), see
[GANTRY_RESULTS.md](GANTRY_RESULTS.md). The six-joint results below are preserved.

This experiment runs the **unmodified standalone ROS-Industrial STOMP C++ core**
with task callbacks that evaluate costs using Drake 1.55.0. No ROS nodes, ROS
messages, MoveIt planning pipeline, or robot hardware are involved. It runs in
the existing `lunar-rover-jazzy-drake` container.

The adapter is specific to this experiment: it supplies correlated Gaussian
noise, fixed endpoints, joint limits, smoothed updates, collision costs, and a
small joint-acceleration cost. This is **not a port of the MoveIt STOMP plugin's
complete configuration**. Comparing its runtime with a previous MoveIt session
cannot isolate ROS overhead.

## Measured results

Measured on this machine (AMD Ryzen 7 5700X, 8 cores / 16 logical CPUs), with
100 waypoints, 20 rollouts, seeds 0–9, a maximum of 500 iterations, and a soft
10-second computation budget. Optimization stops at the first valid solution
after an iteration. There are 6 target configurations in each of 2 scenes:
120 trials in total. All 120 passed the final discrete collision check,
endpoint preservation check, and joint-limit check.

| Scene / target | Mean planning time | Median | Maximum | Success |
|---|---:|---:|---:|---:|
| Open scene, all six poses | 0.069 s | — | 0.079 s | 60/60 |
| Pole, left reach | 2.603 s | 2.262 s | 8.149 s | 10/10 |
| Pole, across pole | 0.787 s | 0.608 s | 1.990 s | 10/10 |

The other four pole-scene targets have collision-free initial paths and mean
times of 0.071–0.079 s. Both obstacle-avoidance targets have valid endpoints but
colliding straight-line initial paths. The slowest trial used 116 iterations,
2,437 trajectory-cost evaluations and 243,700 configuration-distance queries.
It spent 8.083 of its 8.149 seconds inside the task's Python cost callback.
That includes Python loops, Drake geometry queries, and cost arithmetic; this
measurement does not separate those components.

Final validation averaged 0.0037 s per trial. Scene construction took about
0.02 s per scene and is reported separately. Planning times include per-call
STOMP construction, noise covariance setup, optimization and cost evaluation;
they exclude process startup, model loading, IK, visualization and execution.

**Conclusion:** seconds-long planning is possible without ROS. This experiment
does not quantify ROS middleware overhead or reproduce the user's ROS scene.
For that, instrument the original application and compare identical STOMP
tasks/backends directly versus through the ROS interface.

## Run

From the host:

```sh
docker start lunar-rover-jazzy-drake
docker exec -it lunar-rover-jazzy-drake bash
```

Inside the existing container, setup is already complete:

```sh
cd ~/drake/stomp_benchmark
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python benchmark.py
python plot.py results
python view.py results/pole_across_pole_0.npy
```

The last command exports a standalone HTML animation beside the trajectory.
Open that file in a browser. The red line is the initial end-effector path;
the green line is the optimized end-effector path. The arm animation shows
the whole robot; collision avoidance is checked for the robot, not just its
end effector. The five-second playback is illustrative and is not a dynamically
validated or time-parameterized robot command.

For a smaller run or to compare waypoint counts:

```sh
python benchmark.py --seeds 3 --targets left_reach across_pole --scenes pole --output quick
python benchmark.py --waypoints 40 --rollouts 12 --output coarse
```

On a fresh Debian/Ubuntu container, install `g++`, `libeigen3-dev` and
`libconsole-bridge-dev`, plus Python `drake` and `matplotlib`. Then run
`python setup.py`. It downloads pinned source/model assets and builds
`libstomp_bridge.so`. No ROS package is needed. The Drake Dockerfile now includes the compiler and system libraries;
`setup.py` still downloads the pinned assets and builds the native bridge.

## Model, scenes and validation limits

- UR5 kinematics and collision meshes come from `example-robot-data`, pinned
  in `provenance.json`. Original upstream source is `ros-industrial/universal_robot`.
  STL meshes are converted to OBJ; Drake uses convex hulls for proximity queries.
  Collision geometry also supplies the display geometry.
- The base is fixed. The open scene still checks self-collision. The pole scene
  adds a 0.12 × 0.12 × 0.80 m box centered at (0, 0.50, 0.40) m. Neither scene
  includes a floor, table, gripper, payload, or other environment geometry.
- Goals are known joint configurations representing six end-effector poses,
  not Cartesian pose requests requiring an IK search. Their world positions,
  rotations and joint angles are saved in `results/metadata.json`.
- The cost uses a 5 cm influence distance, squared clearance deficits, and a
  small second-difference penalty. Optimizer validity requires at least 2 mm
  clearance at waypoints. These are experiment settings, not universal STOMP defaults.
- The final check samples each joint-space segment with at most 0.025 rad
  Euclidean increments, with zero added collision padding. It is a discrete
  collision check, not proof of continuous collision freedom or a 2 mm margin
  everywhere. There are no velocity, acceleration, torque or dynamics constraints.
- The budget is checked between cost/noise callbacks and can overrun by one
  callback. It is not a hard real-time guarantee. Invalid endpoints are rejected
  before planning and marked separately; no such rejection occurred in the full run.
- Ten seeds expose stochastic variation but do not estimate rare failure rates
  or reliable tail percentiles. Timings are from a general-purpose desktop,
  not an isolated real-time benchmark host.

The distance-cost backend was cross-checked against Drake's configuration
collision checker on 100 random configurations per scene (56 and 58 colliding
configurations respectively); all 200 comparisons agreed. Each recorded trial
also checks endpoint preservation, limits and the final path.

## Files

- `benchmark.py`: configuration, scenes, timing and CSV/JSON output.
- `bridge.cpp`: task adapter and C ABI to the upstream STOMP core.
- `setup.py`: pinned source/model download and compilation.
- `results/trials.csv`: every trial, including callback time, validation time,
  iteration counts, query counts and validity flags.
- `results/summary.json`: per-pose means, medians, maxima and success counts.
- `results/planning_times.png`: timing plot including every seed.
- `results/*.npy`: joint paths; these are experimental artifacts, not hardware commands.

Sources: [standalone STOMP](https://github.com/ros-industrial/stomp),
[UR5 model](https://github.com/Gepetto/example-robot-data/tree/01645279cbca89c9f9cce00a9b1452bf022a8da1/robots/ur_description),
[Drake collision checker](https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1_scene_graph_collision_checker.html).
