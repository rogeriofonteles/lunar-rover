# Drake with Python

Edit `simulate.py` in Vim and run it from a terminal. Jupyter is not required.
Use a Python environment with `pydrake` installed, such as this project's Drake
container. The `drake` directory is mounted at `~/drake` in that container, so
edits made on the host are available immediately.

Inside the Drake container:

```sh
cd ~/drake
python simulate.py
```

Open the printed Meshcat URL in your host browser (normally
`http://localhost:7000`). The container configuration publishes ports 7000–7010.
The rover runs for 10 seconds and publishes a recording for playback. Keep the
terminal open while inspecting it; press Enter or Ctrl-C to exit.

```sh
python simulate.py --duration 20 --time-step 0.001
python simulate.py iiwa
python simulate.py pendulum
python simulate.py --duration 0.1 --no-wait
python simulate.py --help
```

The model viewers provide interactive controls in Meshcat; use their **Stop
Running** button or Ctrl-C to exit. The iiwa example may download Drake's example
model assets on its first run. `--no-wait` runs a viewer for one iteration or
exits after the rover simulation, closing its Meshcat server.

You can also run `python /path/to/drake/simulate.py` from another directory:
the rover's `robot.urdf` is resolved relative to the script.

Acrobot balancing examples and LQR comparisons now live in
[`acrobot/`](acrobot/README.md). For example:

```sh
python acrobot/acrobot_test.py --mode infinite
python acrobot/acrobot_test.py --mode finite --horizon 100
python acrobot/acrobot_test.py --mode receding --horizon 100
```
