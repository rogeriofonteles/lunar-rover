"""Run the rover simulation or Drake model viewers from a terminal."""

import argparse
from pathlib import Path


def add_ground(plant):
    from pydrake.geometry import HalfSpace
    from pydrake.math import RigidTransform
    from pydrake.multibody.plant import CoulombFriction

    plant.RegisterCollisionGeometry(
        plant.world_body(), RigidTransform(), HalfSpace(),
        "ground_collision", CoulombFriction(0.7, 0.5),
    )
    plant.RegisterVisualGeometry(
        plant.world_body(), RigidTransform(), HalfSpace(),
        "ground_visual", [0.5, 0.5, 0.5, 0.1],
    )


def create_scene(meshcat, time_step):
    from pydrake.math import RigidTransform
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.visualization import AddDefaultVisualization

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    Parser(builder).AddModels(file_name=str(Path(__file__).with_name("robot.urdf")))
    add_ground(plant)
    plant.Finalize()
    plant.SetDefaultFloatingBaseBodyPose(
        plant.GetBodyByName("base_link"), RigidTransform([0, 0, 0.5]),
    )
    AddDefaultVisualization(builder=builder, meshcat=meshcat)
    return builder.Build()


def run_simulation(meshcat, time_step, duration):
    from pydrake.systems.analysis import Simulator

    diagram = create_scene(meshcat, time_step)
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    meshcat.StartRecording()
    simulator.AdvanceTo(duration)
    meshcat.StopRecording()
    meshcat.PublishRecording()


def view_model(meshcat, example, loop_once):
    from pydrake.common import FindResourceOrThrow
    from pydrake.visualization import ModelVisualizer

    visualizer = ModelVisualizer(meshcat=meshcat)
    if example == "iiwa":
        visualizer.parser().AddModels(
            url="package://drake_models/iiwa_description/sdf/iiwa7_with_box_collision.sdf",
        )
    else:
        visualizer.parser().AddModels(
            file_name=FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"),
        )
    visualizer.Run(loop_once=loop_once)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("example", nargs="?", choices=("rover", "iiwa", "pendulum"), default="rover")
    parser.add_argument("--duration", type=float, default=10.0, help="Rover simulation seconds (default: 10)")
    parser.add_argument("--time-step", type=float, default=0.001, help="Rover time step in seconds (default: 0.001)")
    parser.add_argument("--no-wait", action="store_true", help="Exit after simulation or one viewer iteration")
    args = parser.parse_args()
    if not 0 < args.duration < float("inf"):
        parser.error("--duration must be positive and finite")
    if not 0 <= args.time_step < float("inf"):
        parser.error("--time-step must be nonnegative and finite")

    from pydrake.geometry import StartMeshcat

    meshcat = StartMeshcat()
    print(f"Open Meshcat in your browser: {meshcat.web_url()}", flush=True)
    if args.example == "rover":
        run_simulation(meshcat, args.time_step, args.duration)
        if not args.no_wait:
            input("Simulation complete. Press Enter to close Meshcat... ")
    else:
        view_model(meshcat, args.example, loop_once=args.no_wait)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        pass
