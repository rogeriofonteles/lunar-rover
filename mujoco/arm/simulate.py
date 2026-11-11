"""Run the six-axis arm with LQR, iLQR, or constrained linear MPC."""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time

import mujoco
import numpy as np
import yaml

from .ilqr import iterative_lqr, RecedingHorizonILQR
from .lqr import finite_horizon_lqr
from .model import ArmModel, positive_int, vector
from .mpc import MPC_SOLVERS, ModelPredictiveController
from .problem import ControlProblem
from .buttons import BUTTON_NAMES, button_target


DEFAULT_CONFIG = Path(__file__).with_name("config.yaml")
MODES = ("infinite", "finite", "receding", "ilqr", "ilqr_receding", "mpc")


def load_config(path=None, **overrides):
    with DEFAULT_CONFIG.open() as stream:
        config = yaml.safe_load(stream)
    if path is not None:
        with Path(path).open() as stream:
            custom = yaml.safe_load(stream)
        if not isinstance(custom, dict):
            raise ValueError("Configuration must be a YAML mapping")
        unknown = set(custom) - set(config)
        if unknown:
            raise ValueError(f"Unknown configuration keys: {sorted(unknown)}")
        config.update(custom)
    unknown = set(overrides) - set(config)
    if unknown:
        raise ValueError(f"Unknown overrides: {sorted(unknown)}")
    config.update({key: value for key, value in overrides.items() if value is not None})
    if config["button"] is not None:
        if overrides.get("target_state") is not None:
            raise ValueError("Choose either a button or an explicit target_state")
        target = button_target(config["button"])
        config["target_state"] = target["joint_positions_rad"] + [0.]*6
    if config["mode"] not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    if config["discretization"] not in ("euler", "zoh"):
        raise ValueError("discretization must be euler or zoh")
    if config["solver"] not in MPC_SOLVERS:
        raise ValueError(f"solver must be one of {MPC_SOLVERS}")
    for key in ("headless", "wait", "realtime", "maximize", "viewer_ui"):
        if not isinstance(config[key], bool):
            raise ValueError(f"{key} must be a YAML boolean")
    for key in ("steps", "horizon", "snapshot_width", "snapshot_height"):
        positive_int(config[key], key)
    return config


def run_simulation(config_path=None, **overrides):
    c = load_config(config_path, **overrides)
    model = ArmModel(c["timestep"], c["physics_timestep"], c["torque_limit"])
    p = ControlProblem(model, c["target_state"], c["horizon"], c["Q"], c["R"],
                       c["terminal_cost"], c["discretization"])
    x = vector(c["initial_state"], 12, "initial_state")
    mode = c["mode"]
    count = min(c["steps"], c["horizon"]) if mode in ("finite", "ilqr") else c["steps"]
    options = {key.removeprefix("ilqr_"): c[key] for key in c if key.startswith("ilqr_")}
    controller, plan, gains = None, None, None
    start = time.perf_counter()
    print(f"MuJoCo {mujoco.__version__} | {mode} | {c['discretization']} | dt={model.dt:g}s | horizon={p.horizon}", flush=True)
    if c["button"] is not None:
        print(f"Button: {c['button']} | target joints (rad): {p.target[:6].tolist()}", flush=True)
    if mode in ("finite", "receding"):
        gains, _ = finite_horizon_lqr(p.A, p.B, p.Q, p.R, p.horizon, p.Qf)
    elif mode == "ilqr":
        plan = iterative_lqr(p, x, **options)
        print(f"iLQR: {plan.status}, iterations={plan.iterations}, cost={plan.costs[0]:.6g} -> {plan.costs[-1]:.6g}", flush=True)
    elif mode == "ilqr_receding":
        controller = RecedingHorizonILQR(p, **options)
    elif mode == "mpc":
        controller = ModelPredictiveController(p, c["solver"], c["mpc_max_iterations"], c["mpc_tolerance"])

    display = mujoco.MjData(model.model)
    display.qpos[:], display.qvel[:] = x[:6], x[6:]
    mujoco.mj_forward(model.model, display)
    viewer_context = nullcontext(None)
    if not c["headless"]:
        from .viewer import native_viewer
        viewer_context = native_viewer(model.model, display, c["maximize"], c["viewer_ui"])
    states, controls, solver_log = [x.copy()], [], []
    with viewer_context as viewer:
        if viewer is not None:
            viewer.cam.lookat[:] = [.15, .15, 1.3]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 4.4, 125, -13
            if c["button"] is not None:
                viewer.cam.lookat[:] = [0, 0, 1.35]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.73, 59, -20
        for k in range(count):
            step_start = time.perf_counter()
            if viewer is not None and not viewer.is_running():
                break
            if controller is not None:
                u = controller.command(x)
                result = controller.last_result
                record = {"step": k, "converged": bool(result.converged),
                          "iterations": result.iterations, "status": result.status}
                if mode == "mpc":
                    record["residual"] = result.residual
                solver_log.append(record)
                if k == 0 or (k+1) % 25 == 0:
                    print(f"step {k+1}: {result.status}, iterations={result.iterations}, state error={np.linalg.norm(x-p.target):.4g}", flush=True)
            elif mode == "ilqr":
                u = plan.controls[k] - plan.gains[k] @ (x - plan.states[k])
            else:
                K = p.K if mode == "infinite" else gains[k if mode == "finite" else 0]
                u = p.u_ref - K @ (x - p.target)
            u = p.clip(u)
            x = model.step(x, u)
            controls.append(u.copy())
            states.append(x.copy())
            if viewer is not None:
                # Separate viewer data prevents optimizer rollouts from moving the display.
                with viewer.lock():
                    display.qpos[:], display.qvel[:], display.ctrl[:] = x[:6], x[6:], u
                    display.time = (k + 1) * model.dt
                    mujoco.mj_forward(model.model, display)
                viewer.sync()
                if c["realtime"]:
                    time.sleep(max(0., model.dt - (time.perf_counter() - step_start)))
        if viewer is not None and c["wait"] and viewer.is_running():
            print("Simulation finished. Close the viewer to exit.", flush=True)
            while viewer.is_running():
                viewer.sync()
                time.sleep(.05)
    xs, us = np.asarray(states), np.asarray(controls).reshape(-1, 6)
    summary = {"mode": mode, "discretization": c["discretization"],
               "button": c["button"], "initial_state": xs[0].tolist(),
               "target_state": p.target.tolist(),
               "solver": c["solver"] if mode == "mpc" else None,
               "steps": len(us), "simulated_seconds": len(us)*model.dt,
               "wall_seconds": time.perf_counter()-start,
               "initial_error": float(np.linalg.norm(xs[0]-p.target)),
               "final_error": float(np.linalg.norm(xs[-1]-p.target)),
               "peak_torque_nm": np.max(np.abs(us), axis=0).tolist() if len(us) else [0.]*6,
               "nonconverged_solves": controller.nonconverged_count if controller else int(plan is not None and not plan.converged),
               "solver_log": solver_log}
    if c["button"] is not None:
        display.qpos[:], display.qvel[:] = xs[-1, :6], xs[-1, 6:]
        mujoco.mj_forward(model.model, display)
        target_position = button_target(c["button"])["position_m"]
        summary["button_position_m"] = target_position
        summary["final_tip_error_m"] = float(np.linalg.norm(
            display.site("button_tip").xpos - target_position))
    if plan is not None:
        summary["plan"] = {"converged": plan.converged, "status": plan.status,
                           "iterations": plan.iterations, "costs": plan.costs}
    if c["output"]:
        destination = Path(c["output"]).with_suffix(".npz")
        destination.parent.mkdir(parents=True, exist_ok=True)
        np.savez(destination, states=xs, controls=us, times=np.arange(len(xs))*model.dt,
                 target=p.target, equilibrium_control=p.u_ref)
        destination.with_suffix(".json").write_text(json.dumps(summary, indent=2)+"\n")
    if c["snapshot"]:
        from PIL import Image
        destination = Path(c["snapshot"])
        destination.parent.mkdir(parents=True, exist_ok=True)
        display.qpos[:], display.qvel[:] = xs[-1, :6], xs[-1, 6:]
        mujoco.mj_forward(model.model, display)
        # XML sizes are defaults; explicit snapshot dimensions also resize the FBO.
        model.model.vis.global_.offwidth = c["snapshot_width"]
        model.model.vis.global_.offheight = c["snapshot_height"]
        with mujoco.Renderer(model.model, height=c["snapshot_height"], width=c["snapshot_width"]) as renderer:
            renderer.update_scene(display, camera="buttons" if c["button"] is not None else "overview")
            Image.fromarray(renderer.render()).save(destination)
    print(json.dumps({key: value for key, value in summary.items() if key not in ("solver_log", "plan")}, indent=2), flush=True)
    return xs, us, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--mode", choices=MODES)
    parser.add_argument("--discretization", choices=("euler", "zoh"))
    parser.add_argument("--solver", choices=MPC_SOLVERS)
    for key in ("horizon", "steps", "ilqr_max_iterations", "mpc_max_iterations", "snapshot_width", "snapshot_height"):
        parser.add_argument("--"+key.replace("_", "-"), type=int)
    for key in ("timestep", "physics_timestep", "ilqr_tolerance", "ilqr_regularization",
                "ilqr_state_damping", "ilqr_control_damping", "mpc_tolerance", "torque_limit"):
        parser.add_argument("--"+key.replace("_", "-"), type=float)
    for key in ("headless", "wait", "realtime", "maximize", "viewer_ui"):
        parser.add_argument("--"+key.replace("_", "-"), action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--initial-state", nargs=12, type=float)
    target_options = parser.add_mutually_exclusive_group()
    target_options.add_argument("--target-state", nargs=12, type=float)
    target_options.add_argument("--button", choices=BUTTON_NAMES,
                                help="reach a recorded button pose from the configured initial state")
    parser.add_argument("--Q-diagonal", dest="Q", nargs=12, type=float)
    parser.add_argument("--R-diagonal", dest="R", nargs=6, type=float)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--snapshot", type=Path)
    args = vars(parser.parse_args())
    config_path = args.pop("config")
    try:
        run_simulation(config_path, **args)
    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
