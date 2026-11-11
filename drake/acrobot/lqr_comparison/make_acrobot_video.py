"""Simulate receding-horizon LQR and stream RGBA animation frames to ffmpeg.

Run in the Drake environment; stdout contains only raw 1600x900 video frames.
The nonlinear plant is integrated by Drake with sample-and-hold elbow torque.

From the repository root (ffmpeg runs on the host):
    set -o pipefail
    docker exec lunar-rover-jazzy-drake python \
        /home/rogerio/drake/acrobot/lqr_comparison/make_acrobot_video.py | \
        ffmpeg -y -f rawvideo -pixel_format rgba -video_size 1600x900 \
        -framerate 30 -i pipe:0 -an -c:v libx264 -crf 18 \
        -pix_fmt yuv420p -movflags +faststart \
        drake/acrobot/lqr_comparison/acrobot_video.mp4
"""

import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.systems.analysis import Simulator

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from acrobot_test import finite_horizon_lqr


OUTPUT = Path(__file__).resolve().parent / "acrobot_video"
FPS = 30
DT = 0.1
DURATION = 10.0
HORIZON = 100
TARGET = np.array([math.pi, 0., 0., 0.])
INITIAL = np.array([math.pi - 0.15, 0.15, 0., 0.])


def simulate():
    plant = AcrobotPlant()
    simulator = Simulator(plant)
    simulator.get_mutable_integrator().set_target_accuracy(1e-9)
    context = simulator.get_mutable_context()
    context.SetContinuousState(INITIAL)
    port = plant.get_input_port(0)
    port.FixValue(context, [0.])
    simulator.Initialize()
    # The equilibrium, dynamics, costs and horizon are fixed: solving the
    # finite-horizon problem again gives the same first-stage gain every time.
    gain = finite_horizon_lqr(plant, timestep=DT, horizon=HORIZON)
    times = np.arange(round(DURATION * FPS) + 1) / FPS
    states = []
    torques = []
    torque = 0.
    for index, time in enumerate(times):
        simulator.AdvanceTo(float(time))
        state = context.get_continuous_state_vector().CopyToVector()
        if index % round(DT * FPS) == 0:
            error = state - TARGET
            error[:2] = np.arctan2(np.sin(error[:2]), np.cos(error[:2]))
            torque = float((-gain @ error)[0])
            port.FixValue(context, [torque])
        states.append(state)
        torques.append(torque)
    states = np.asarray(states)
    torques = np.asarray(torques)
    errors = states - TARGET
    errors[:, :2] = np.arctan2(np.sin(errors[:, :2]), np.cos(errors[:, :2]))
    assert np.isfinite(states).all(), "Simulation diverged"
    assert np.max(np.abs(errors[-1, :2])) < 1e-3, "Angles did not settle"
    assert np.max(np.abs(errors[-1, 2:])) < 1e-3, "Velocities did not settle"
    OUTPUT.mkdir(exist_ok=True)
    np.savez(OUTPUT / "trajectory.npz", time=times, state=states,
             torque=torques, target=TARGET, gain=gain)
    summary = dict(
        controller="Finite-horizon LQR applied in receding-horizon form",
        horizon_steps=HORIZON, control_interval_s=DT,
        prediction_horizon_s=HORIZON * DT, simulation_duration_s=DURATION,
        initial_state=INITIAL.tolist(), target_state=TARGET.tolist(),
        final_state=states[-1].tolist(), final_error=errors[-1].tolist(),
        peak_abs_elbow_torque_Nm=float(np.max(np.abs(torques))),
        Q=[10., 10., 1., 1.], R=1., terminal_cost="Q",
        prediction_model="Forward-Euler discretization about upright",
        simulation_model="Nonlinear Drake AcrobotPlant",
        torque_constraints=None,
    )
    (OUTPUT / "simulation.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary), file=sys.stderr)
    return times, states, errors, torques


def coordinates(state):
    q1, q2 = state[:2]
    # Drake's default Acrobot link lengths are both 1 m; q2 is relative to q1.
    elbow = np.array([math.sin(q1), -math.cos(q1)])
    tip = elbow + np.array([math.sin(q1 + q2), -math.cos(q1 + q2)])
    return np.array([[0., 0.], elbow, tip])


def render(times, states, errors, torques):
    bg, ink, muted = "#0c1422", "#e8eef7", "#9aabc0"
    blue, orange, green = "#65baff", "#ffb366", "#77d8ae"
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 12,
        "text.color": ink, "axes.labelcolor": muted,
        "xtick.color": muted, "ytick.color": muted,
        "axes.edgecolor": "#334256", "axes.facecolor": bg,
        "figure.facecolor": bg, "savefig.facecolor": bg,
    })
    fig = plt.figure(figsize=(16, 9), dpi=100)
    fig.text(.055, .94, "ACROBOT", fontsize=14, color=blue, weight="bold")
    fig.text(.055, .885, "Receding-horizon stabilization", fontsize=29, weight="bold")
    fig.text(.055, .845, "100-step finite-horizon LQR  /  0.10 s control interval  /  10 s look-ahead",
             fontsize=13, color=muted)
    robot = fig.add_axes([.055, .19, .49, .61])
    robot.set_aspect("equal")
    robot.set_xlim(-1.45, 1.45)
    robot.set_ylim(-.23, 2.35)
    robot.axis("off")
    robot.axvline(0, color="#34465c", ls="--", lw=1.5)
    robot.plot([-.35, .35], [-.08, -.08], color="#43546a", lw=6, solid_capstyle="round")
    for pose, color, style, alpha in [
        (INITIAL, muted, ":", .55), (TARGET, green, "--", .7),
    ]:
        points = coordinates(pose)
        robot.plot(points[:, 0], points[:, 1], style, lw=3, color=color, alpha=alpha)
    robot.text(.14, 2.15, "Upright target", color=green, fontsize=12)
    robot.text(-1.4, .04, "Fixed shoulder", color=muted, fontsize=11)
    link1, = robot.plot([], [], color=blue, lw=12, solid_capstyle="round")
    link2, = robot.plot([], [], color=orange, lw=12, solid_capstyle="round")
    joints, = robot.plot([], [], "o", ms=12, color=ink, mec=bg, mew=3)
    robot.text(-1.4, 2.23, "NONLINEAR PLANT", color=muted, fontsize=11, weight="bold")
    robot.text(-1.4, -.2, "Blue: passive link     Orange: actuated elbow link", color=muted, fontsize=10)
    angle = fig.add_axes([.61, .51, .34, .25])
    torque_ax = fig.add_axes([.61, .215, .34, .19])
    degree_errors = np.rad2deg(errors[:, :2])
    def style_axis(axis, label, values):
        axis.set_xlim(0, DURATION)
        bound = max(float(np.max(np.abs(values))) * 1.2, .1)
        axis.set_ylim(-bound, bound)
        axis.set_ylabel(label)
        axis.grid(color="#28364a", alpha=.65, lw=.7)
        axis.axhline(0, color=green, lw=.8, alpha=.65)
        axis.spines[["top", "right"]].set_visible(False)
        axis.tick_params(labelsize=10)
    style_axis(angle, "Angle error (deg)", degree_errors)
    style_axis(torque_ax, "Elbow torque (N m)", torques)
    torque_ax.set_xlabel("Simulation time (s)")
    angle.set_title("Approaching equilibrium", loc="left", fontsize=14, pad=15)
    curves = [angle.plot([], [], color=c, lw=2.3, label=l)[0]
              for c, l in [(blue, "Shoulder"), (orange, "Elbow")]]
    angle.legend(loc="upper right", frameon=False, labelcolor=ink, fontsize=10)
    torque_curve, = torque_ax.step([], [], where="post", color=orange, lw=1.8)
    cursors = [axis.axvline(0, color=ink, alpha=.4, lw=1) for axis in (angle, torque_ax)]
    clock = fig.text(.055, .115, "", fontsize=20, weight="bold")
    status = fig.text(.055, .067, "", fontsize=12, color=muted)
    fig.text(.61, .067, "Replan → apply first input → advance → repeat", fontsize=11, color=muted)

    def draw(index):
        points = coordinates(states[index])
        link1.set_data(points[:2, 0], points[:2, 1])
        link2.set_data(points[1:, 0], points[1:, 1])
        joints.set_data(points[:, 0], points[:, 1])
        for j, line in enumerate(curves):
            line.set_data(times[:index+1], degree_errors[:index+1, j])
        torque_curve.set_data(times[:index+1], torques[:index+1])
        for line in cursors:
            line.set_xdata([times[index], times[index]])
        clock.set_text(f"t = {times[index]:05.2f} s     |     u = {torques[index]:+.3f} N m")
        settled = np.max(np.abs(errors[index, :2])) < .001 and np.max(np.abs(errors[index, 2:])) < .001
        status.set_text(("Equilibrium reached" if settled else "Correcting the initial displacement")
                        + f"   •   max angle error: {np.max(np.abs(degree_errors[index])):.4f}°")
        status.set_color(green if settled else muted)
        fig.canvas.draw()

    # One second to inspect the initial pose, real-time motion, then a final hold.
    indices = [0] * FPS + list(range(len(times))) + [len(times)-1] * (2 * FPS)
    for frame, index in enumerate(indices):
        draw(index)
        if frame == 0:
            fig.savefig(OUTPUT / "initial.png", dpi=100)
        if frame == len(indices)-1:
            fig.savefig(OUTPUT / "equilibrium.png", dpi=100)
        sys.stdout.buffer.write(fig.canvas.buffer_rgba())
    plt.close(fig)


if __name__ == "__main__":
    render(*simulate())
