# Project environment

## Drake and Acrobot

Drake and its Python dependencies are installed inside the running Docker
container `lunar-rover-jazzy-drake`. Run Drake simulations and tests there;
the host Python environment may not have `pydrake` installed.

The project's `drake/` directory is mounted at `~/drake` in the container.
The container uses the Fish shell. The user's simulation command is:

```sh
docker exec -it lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot_test.py'
```

For automated commands, omit `-it`. The modular Acrobot implementation
containing `acrobot/lqr.py` has its simulation entry point at
`acrobot/acrobot_test.py`:

```sh
docker exec lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/acrobot_test.py --headless --no-wait'
docker exec lunar-rover-jazzy-drake fish -c 'cd ~/drake; python -m unittest discover -s acrobot -p "test_*.py"'
```

Choose the entry point corresponding to the code being changed:
`drake/acrobot_test.py` and `drake/acrobot/acrobot_test.py` are separate files.

## MuJoCo and the six-axis arm

Run MuJoCo simulations and tests in `lunar-rover-jazzy-mujoco`. The host
`mujoco/` directory is mounted at `~/mujoco`; Python dependencies live in
`/opt/mujoco-env`. This is a separate image from Drake.

```sh
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python simulate.py --headless --no-wait'
docker exec lunar-rover-jazzy-mujoco fish -c 'cd ~/mujoco; python -m unittest discover -s tests -p "test_*.py"'
```

See `mujoco/README.md` for build commands, viewer setup, controller modes,
Euler/ZOH discretization, and the Dual-Ascent/SQP linear MPC solvers.

The MuJoCo Compose service enables NVIDIA rendering with the NVIDIA Container
Toolkit, `graphics,display,utility,compute` driver capabilities, and the NVIDIA
GLX vendor. Do not force `LIBGL_ALWAYS_SOFTWARE` or `GALLIUM_DRIVER=llvmpipe`
for normal runs. Check the actual renderer using `python -m arm.check_graphics
--require-nvidia` inside `~/mujoco`. For headless GPU snapshots use
`MUJOCO_GL=egl`; `osmesa` explicitly selects CPU rendering. Physics and control
optimization still run on the CPU.
