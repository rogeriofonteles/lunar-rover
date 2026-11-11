from pydrake.examples import AcrobotPlant, AcrobotState
from pydrake.geometry import SceneGraph, Meshcat, StartMeshcat 
from pydrake.geometry import MeshcatVisualizer 
from pydrake.systems.framework import DiagramBuilder 
from pydrake.systems.analysis import Simulator
import math
import numpy as np

from acrobot.visuals import add_robot_geometry, configure_meshcat


def finite_horizon_lqr(plant, timestep=0.1, horizon=10,
                       Q=None, R=1.0,
                       equilibrium_state=None):
    """Linearize the Acrobot and use finite-horizon value iteration.

    Returns the first feedback gain for receding-horizon control. The model is
    discretized with forward Euler at ``timestep`` seconds.
    """
    if horizon < 1:
        raise ValueError("horizon must be at least 1")

    if equilibrium_state is None:
        equilibrium_state = np.array([math.pi, 0., 0., 0.])
    else:
        equilibrium_state = np.asarray(equilibrium_state, dtype=float)
    if Q is None:
        Q = np.diag([10., 10., 1., 1.])
    else:
        Q = np.asarray(Q, dtype=float)
    if Q.shape != (4, 4):
        raise ValueError("Q must be a 4x4 state cost matrix")
    if R <= 0:
        raise ValueError("R must be positive")

    context = plant.CreateDefaultContext()
    input_port = plant.get_input_port(0)
    input_port.FixValue(context, [0.])

    def derivatives(state, torque):
        context.SetContinuousState(state)
        input_port.FixValue(context, [torque])
        return plant.EvalTimeDerivatives(context).CopyToVector()

    # Central finite differences of x_dot = f(x, u) at the upright equilibrium.
    epsilon = 1e-6
    A_continuous = np.empty((4, 4))
    for i in range(4):
        basis_vector = np.zeros(4)  # 1 at index i, 0 everywhere else.
        basis_vector[i] = 1.0
        delta = epsilon * basis_vector
        A_continuous[:, i] = (
            derivatives(equilibrium_state + delta, 0.)
            - derivatives(equilibrium_state - delta, 0.)
        ) / (2. * epsilon)
    B_continuous = (
        derivatives(equilibrium_state, epsilon)
        - derivatives(equilibrium_state, -epsilon)
    ) / (2. * epsilon)

    A = np.eye(4) + timestep * A_continuous
    B = timestep * B_continuous.reshape(4, 1)

    # Backward dynamic programming for the finite-horizon quadratic cost.
    value_matrix = Q.copy()  # terminal cost
    first_gain = None
    for _ in range(horizon):
        control_cost = R + (B.T @ value_matrix @ B)[0, ]
        gain = np.linalg.solve(
            np.array([[control_cost]]), B.T @ value_matrix @ A
        )
        value_matrix = Q + A.T @ value_matrix @ A - A.T @ value_matrix @ B @ gain
        first_gain = gain

    return first_gain


def infinite_horizon_lqr(plant, timestep=0.1, Q=None, R=1.0,
                         equilibrium_state=None, tolerance=1e-9,
                         max_iterations=10000):
    """Compute the infinite-horizon discrete LQR gain by value iteration."""
    if tolerance <= 0:
        raise ValueError("tolerance must be positive")
    if max_iterations < 1:
        raise ValueError("max_iterations must be at least 1")
    if equilibrium_state is None:
        equilibrium_state = np.array([math.pi, 0., 0., 0.])
    else:
        equilibrium_state = np.asarray(equilibrium_state, dtype=float)
    if Q is None:
        Q = np.diag([10., 10., 1., 1.])
    else:
        Q = np.asarray(Q, dtype=float)
    if Q.shape != (4, 4):
        raise ValueError("Q must be a 4x4 state cost matrix")
    if R <= 0:
        raise ValueError("R must be positive")

    # Linearize x_dot = f(x, u) around the upright equilibrium.
    context = plant.CreateDefaultContext()
    input_port = plant.get_input_port(0)
    input_port.FixValue(context, [0.])

    def derivatives(state, torque):
        context.SetContinuousState(state)
        input_port.FixValue(context, [torque])
        return plant.EvalTimeDerivatives(context).CopyToVector()

    epsilon = 1e-6
    A_continuous = np.empty((4, 4))
    for i in range(4):
        basis_vector = np.zeros(4)  # 1 at index i, 0 everywhere else.
        basis_vector[i] = 1.0
        delta = epsilon * basis_vector
        A_continuous[:, i] = (
            derivatives(equilibrium_state + delta, 0.)
            - derivatives(equilibrium_state - delta, 0.)
        ) / (2. * epsilon)
    B_continuous = (
        derivatives(equilibrium_state, epsilon)
        - derivatives(equilibrium_state, -epsilon)
    ) / (2. * epsilon)
    A = np.eye(4) + timestep * A_continuous
    B = timestep * B_continuous.reshape(4, 1)

    # Iterate the discrete Riccati equation until the feedback gain settles.
    value_matrix = Q.copy()
    previous_gain = None
    for _ in range(max_iterations):
        control_cost = R + (B.T @ value_matrix @ B)[0, 0]
        gain = np.linalg.solve(
            np.array([[control_cost]]), B.T @ value_matrix @ A
        )
        next_value_matrix = (
            Q + A.T @ value_matrix @ A
            - A.T @ value_matrix @ B @ gain
        )
        next_value_matrix = 0.5 * (next_value_matrix + next_value_matrix.T)
        if (previous_gain is not None
                and np.max(np.abs(gain - previous_gain)) < tolerance):
            return gain
        value_matrix = next_value_matrix
        previous_gain = gain

    raise RuntimeError(
        f"Infinite-horizon LQR did not converge in {max_iterations} iterations"
    )

def build_acrobot(acrobot_plant, meshcat):
    builder = DiagramBuilder()
    builder.AddSystem(acrobot_plant)
    scene_graph = builder.AddSystem(SceneGraph())
    
    add_robot_geometry(builder, acrobot_plant, scene_graph)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    configure_meshcat(meshcat)

    return builder.Build()


def setup_simulation(diagram) -> Simulator:
    simulator = Simulator(diagram)
    root_context = simulator.get_mutable_context()
    plant = diagram.GetSubsystemByName("acrobot")

    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    plant_context.SetContinuousState([math.pi - 0.15, 0.15, 0., 0.])
    plant.get_input_port(0).FixValue(plant_context, [0.])

    return simulator, root_context, plant_context


def run_simulation():
    meshcat = StartMeshcat()

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
    simulation, root_context, plant_context = setup_simulation(diagram)

    timestep = 0.1
    equilibrium_state = np.array([math.pi, 0., 0., 0.])
    gain = infinite_horizon_lqr(
        acrobot_plant, timestep=timestep,
        equilibrium_state=equilibrium_state,
    )

    simulation.Initialize()
    diagram.ForcedPublish(root_context)

    meshcat.StartRecording()

    for step in range(1000):
        state = acrobot_plant.get_output_port(0).Eval(plant_context)
        state_error = np.asarray(state) - equilibrium_state
        state_error[0] = math.atan2(math.sin(state_error[0]),
                                    math.cos(state_error[0]))
        state_error[1] = math.atan2(math.sin(state_error[1]),
                                    math.cos(state_error[1]))
        torque = float((-gain @ state_error)[0])
        acrobot_plant.get_input_port(0).FixValue(plant_context, [torque])
        simulation.AdvanceTo((step + 1) * timestep)
    meshcat.StopRecording()
    meshcat.PublishRecording()

    print(f"Replay: {meshcat.web_url()}")
    input("Press enter to exit simulation ...")


if __name__ == "__main__":
    run_simulation()
