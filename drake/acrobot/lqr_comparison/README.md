# Acrobot LQR comparison

This folder contains an interactive browser dashboard, static plot exports, and
the script that regenerates them.

From the Drake container, run:

```sh
cd ~/drake/lqr_comparison
python compare_lqr.py
```

Open `lqr_comparison.html` in a browser. The dashboard compares finite-horizon
controllers with horizons of 10 and 100 steps against the infinite-horizon
controller. It includes state and torque traces, feedback gains, and a plot of
finite-horizon gain convergence.

### Infinite-horizon initial-error and clipping report

Run all six nonlinear Drake cases and export a four-page PDF, three PNG plots,
six trajectory CSVs and JSON containing the gain, eigenvalues and run metrics:

```sh
docker exec lunar-rover-jazzy-drake fish -c 'cd ~/drake; python acrobot/lqr_comparison/compare_infinite_clipping.py'
```

Outputs are in `drake/output/pdf/` on the host. The current report is
`infinite_lqr_wrapped_shoulder_comparison.pdf`; the earlier report is preserved. The script reads the control
period, number of steps and torque limit from `acrobot/lqr.yaml`; optional
`--timestep`, `--steps`, and `--output-dir` arguments override these settings.
Every run uses infinite-horizon LQR with initial state `[pi-e, 0, 0, 0]`,
for `e = 0.05, 0.25, 0.6` rad, with and without torque clipping. Angular errors
are wrapped to `[-pi, pi)`. CSV torque values refer to the interval starting
at that row; the final row has no torque command (NaN).
The PDF records both the forward-Euler design poles and the exact zero-order
hold local poles. Saturation leaves the designed gain unchanged; these local
poles do not characterize the full nonlinear saturated trajectory.
