from pydrake.examples import AcrobotPlant, AcrobotState
from pydrake.geometry import SceneGraph, StartMeshcat 
from pydrake.geometry import MeshcatVisualizer 
from pydrake.systems.framework import DiagramBuilder 
from pydrake.systems.analysis import Simulator
import argparse
import math
from pathlib import Path
import yaml
import numpy as np


if __package__:
    from .lqr import _state_cost, wrapped_state_error, controller_gains, control_torque, finite_horizon_lqr, infinite_horizon_lqr
    from .ilqr import iterative_lqr, RecedingHorizonILQR
    from .visuals import add_robot_geometry, configure_meshcat
else:
    from lqr import _state_cost, wrapped_state_error, controller_gains, control_torque, finite_horizon_lqr, infinite_horizon_lqr
    from ilqr import iterative_lqr, RecedingHorizonILQR
    from visuals import add_robot_geometry, configure_meshcat


def build_acrobot(acrobot_plant, meshcat):
    builder = DiagramBuilder()
    builder.AddSystem(acrobot_plant)
    scene_graph = builder.AddSystem(SceneGraph())
    
    add_robot_geometry(builder, acrobot_plant, scene_graph)
    if meshcat is not None:
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        configure_meshcat(meshcat)

    return builder.Build()


def setup_simulation(diagram, initial_state=None) -> Simulator:
    simulator = Simulator(diagram)
    root_context = simulator.get_mutable_context()
    plant = diagram.GetSubsystemByName("acrobot")

    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    plant_context.SetContinuousState(
        [math.pi - 0.15, 0.15, 0., 0.] if initial_state is None else initial_state
    )
    plant.get_input_port(0).FixValue(plant_context, [0.])

    return simulator, root_context, plant_context


def simulation_options(config_path=None, **overrides):
    """Resolve built-in defaults < YAML < explicit function/CLI arguments."""
    options = dict(mode="infinite", horizon=100, timestep=0.1, steps=1000,
                   headless=False, wait=True, clip_torque=True,
                   torque_limit_nm=3.0, Q=None, R=1.0, ilqr_initialization="auto", ilqr_max_iterations=50, ilqr_tolerance=1e-6,
                   ilqr_regularization=1e-5, ilqr_state_damping=1.0, ilqr_control_damping=1.0, initial_state=[math.pi - 0.15, 0.15, 0., 0.])
    path = Path(__file__).with_name("lqr.yaml") if config_path is None else Path(config_path)
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Simulation YAML must be a mapping")
    unknown = config.keys() - options.keys()
    if unknown:
        raise ValueError(f"Unknown simulation options: {sorted(unknown)}")
    options.update(config)
    options.update({key: value for key, value in overrides.items() if value is not None})
    if options["mode"] not in ("infinite", "finite", "receding", "ilqr", "ilqr_receding"):
        raise ValueError("mode must be infinite, finite, receding, ilqr, or ilqr_receding")
    if options["ilqr_initialization"] not in ("auto", "lqr", "collocation"):
        raise ValueError("ilqr_initialization must be auto, lqr, or collocation")
    for key in ("steps", "horizon", "ilqr_max_iterations"):
        value = options[key]
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f"{key} must be a positive integer")
    for key in ("timestep", "torque_limit_nm", "ilqr_tolerance", "ilqr_regularization", "R"):
        value = options[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be positive and finite")
    for key in ("ilqr_state_damping", "ilqr_control_damping"):
        value = options[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"{key} must be nonnegative and finite")
    if options["ilqr_state_damping"] + options["ilqr_control_damping"] == 0:
        raise ValueError("at least one iLQR damping coefficient must be positive")
    for key in ("headless", "wait", "clip_torque"):
        if not isinstance(options[key], bool):
            raise ValueError(f"{key} must be a boolean")
    options["Q"] = _state_cost(options["Q"], "Q")
    state = np.asarray(options["initial_state"], dtype=float)
    if state.shape != (4,) or not np.all(np.isfinite(state)):
        raise ValueError("initial_state must contain four finite values")
    options["initial_state"] = state
    return options


def run_simulation(mode=None, horizon=None, timestep=None, steps=None,
                   headless=None, wait=None, clip_torque=None, config_path=None,
                   initial_state=None, torque_limit_nm=None, ilqr_max_iterations=None,
                   ilqr_tolerance=None, ilqr_regularization=None,
                   ilqr_state_damping=None, ilqr_control_damping=None, ilqr_initialization=None, Q=None, R=None):
    """Run with YAML settings; explicit non-None arguments override the file.

    initial_state is [theta1, theta2, theta1dot, theta2dot] in rad and rad/s.
    """
    options = simulation_options(
        config_path, Q=Q, R=R, mode=mode, horizon=horizon, timestep=timestep, steps=steps,
        headless=headless, wait=wait, clip_torque=clip_torque,
        initial_state=initial_state, torque_limit_nm=torque_limit_nm,
        ilqr_max_iterations=ilqr_max_iterations, ilqr_tolerance=ilqr_tolerance,
        ilqr_regularization=ilqr_regularization, ilqr_state_damping=ilqr_state_damping,
        ilqr_control_damping=ilqr_control_damping, ilqr_initialization=ilqr_initialization,
    )
    mode, horizon, timestep, steps = (options[key] for key in ("mode", "horizon", "timestep", "steps"))
    headless, wait = options["headless"], options["wait"]
    torque_limit = options["torque_limit_nm"] if options["clip_torque"] else None
    meshcat = None if headless else StartMeshcat()

    acrobot_plant = AcrobotPlant()
    acrobot_plant.set_name("acrobot")

    print(f"Input port 0: {acrobot_plant.get_input_port(0).GetFullDescription()}")
    for port_index in range(acrobot_plant.num_output_ports()):
        port = acrobot_plant.get_output_port(port_index)
        print(f"Output port {port_index}: {port.get_name()} "
              f"(type={port.get_data_type()}, size={port.size()})")
        if port.size() == 4:
            print("  State vector components:")
            for index, (name, description, units) in enumerate([
                ("theta1", AcrobotState.theta1.__doc__, "rad"),
                ("theta2", AcrobotState.theta2.__doc__, "rad"),
                ("theta1dot", AcrobotState.theta1dot.__doc__, "rad/s"),
                ("theta2dot", AcrobotState.theta2dot.__doc__, "rad/s"),
            ]):
                description = description.split("\n\n")[0].strip()
                print(f"    [{index}] {name}: {description} ({units})")

    diagram = build_acrobot(acrobot_plant, meshcat)
    simulation, root_context, plant_context = setup_simulation(diagram, options["initial_state"])

    equilibrium_state = np.array([math.pi, 0., 0., 0.])
    plan = None
    receding_controller = None
    if mode in ("ilqr", "ilqr_receding"):
        solver_options = dict(
            timestep=timestep, horizon=horizon, Q=options["Q"], R=options["R"],
            torque_limit=torque_limit, max_iterations=options["ilqr_max_iterations"],
            tolerance=options["ilqr_tolerance"], regularization=options["ilqr_regularization"],
            state_damping=options["ilqr_state_damping"], control_damping=options["ilqr_control_damping"],
            initialization=options["ilqr_initialization"],
        )
        if mode == "ilqr_receding":
            receding_controller = RecedingHorizonILQR(acrobot_plant, **solver_options)
        else:
            plan = iterative_lqr(acrobot_plant, options["initial_state"], **solver_options)
            print(f"iLQR: converged={plan.converged}; {plan.status}; iterations={plan.iterations}; "
                  f"cost={plan.costs[0]:.6g} -> {plan.costs[-1]:.6g}")
            if not plan.converged:
                print("iLQR executing the best accepted plan; convergence was not achieved.")
    else:
        gains = controller_gains(
            acrobot_plant, mode=mode, timestep=timestep, horizon=horizon,
            equilibrium_state=equilibrium_state, Q=options["Q"], R=options["R"],
        )
    execution_steps = min(steps, horizon) if mode in ("finite", "ilqr") else steps
    print(f"Controller: {mode}; duration: {execution_steps * timestep:g}s")

    simulation.Initialize()
    diagram.ForcedPublish(root_context)

    if meshcat is not None:
        meshcat.StartRecording()

    peak_torque = 0.
    for step in range(execution_steps):
        state = acrobot_plant.get_output_port(0).Eval(plant_context)
        if receding_controller is not None:
            torque = receding_controller.command(np.asarray(state).copy())
            if step == 0 or (step + 1) % max(1, round(1 / timestep)) == 0 or step == execution_steps - 1:
                result = receding_controller.last_result
                print(f"iLQR replan {step+1}: converged={result.converged}; "
                      f"{result.status}; iterations={result.iterations}; "
                      f"cost={result.costs[-1]:.6g}; seed={result.initialization}; "
                      f"state error={np.linalg.norm(wrapped_state_error(state, equilibrium_state)):.6g}", flush=True)
        elif plan is not None:
            deviation = wrapped_state_error(state, plan.states[step])
            torque = plan.controls[step] + control_torque(plan.gains[step], deviation)
            if torque_limit is not None:
                torque = float(np.clip(torque, -torque_limit, torque_limit))
        else:
            gain = gains[step] if mode == "finite" else gains[0]
            state_error = wrapped_state_error(state, equilibrium_state)
            torque = control_torque(gain, state_error, torque_limit)
        peak_torque = max(peak_torque, abs(torque))
        acrobot_plant.get_input_port(0).FixValue(plant_context, [torque])
        simulation.AdvanceTo((step + 1) * timestep)
    if receding_controller is not None:
        print(f"iLQR replans: {receding_controller.solve_count}; "
              f"solves without convergence: {receding_controller.nonconverged_count}")
    if meshcat is not None:
        meshcat.StopRecording()
        meshcat.PublishRecording()
        print(f"Replay: {meshcat.web_url()}")
        if wait:
            input("Press enter to exit simulation ...")
    final_state = np.asarray(acrobot_plant.get_output_port(0).Eval(plant_context)).copy()
    final_error = wrapped_state_error(final_state, equilibrium_state)
    print(f"Final state: {np.array2string(final_state, precision=6)}; "
          f"upright error norm: {np.linalg.norm(final_error):.6g}; "
          f"peak torque: {peak_torque:.6g} N m", flush=True)
    return final_state


def main(argv=None):
    parser = argparse.ArgumentParser(description="Acrobot upright LQR balancing",
                                     argument_default=argparse.SUPPRESS)
    parser.add_argument("--config", dest="config_path", help="YAML settings (default: acrobot/lqr.yaml)")
    parser.add_argument("--mode", choices=("infinite", "finite", "receding", "ilqr", "ilqr_receding"))
    parser.add_argument("--horizon", type=int, help="finite horizon in control steps")
    parser.add_argument("--R", type=float, help="positive torque cost weight")
    parser.add_argument("--Q-diagonal", type=float, nargs=4, help="four nonnegative state cost weights")
    parser.add_argument("--timestep", type=float, help="control period in seconds")
    parser.add_argument("--steps", type=int, help="maximum simulation control steps")
    parser.add_argument("--ilqr-initialization", choices=("auto", "lqr", "collocation"))
    parser.add_argument("--ilqr-max-iterations", type=int)
    parser.add_argument("--ilqr-tolerance", type=float)
    parser.add_argument("--ilqr-regularization", type=float)
    parser.add_argument("--ilqr-state-damping", type=float)
    parser.add_argument("--ilqr-control-damping", type=float)
    parser.add_argument("--initial-state", type=float, nargs=4,
                        metavar=("THETA1", "THETA2", "THETA1DOT", "THETA2DOT"),
                        help="initial state in rad, rad, rad/s, rad/s")
    parser.add_argument("--torque-limit-nm", type=float, help="maximum absolute torque in N m")
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, help="run without Meshcat")
    parser.add_argument("--wait", action=argparse.BooleanOptionalAction, help="wait for Enter after replay")
    clipping = parser.add_mutually_exclusive_group()
    clipping.add_argument("--torque-clipping", dest="clip_torque", action="store_true")
    clipping.add_argument("--no-torque-clipping", dest="clip_torque", action="store_false")
    args = parser.parse_args(argv)
    if hasattr(args, "Q_diagonal"):
        args.Q = np.diag(args.Q_diagonal)
        del args.Q_diagonal
    try:
        run_simulation(**vars(args))
    except (ValueError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
