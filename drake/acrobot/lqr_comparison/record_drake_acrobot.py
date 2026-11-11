"""Save the actual Drake AcrobotGeometry/Meshcat recording as standalone HTML."""

import math
from pathlib import Path
import sys

import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.geometry import Meshcat

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from acrobot_test import build_acrobot, setup_simulation, finite_horizon_lqr


def main():
    meshcat = Meshcat()
    plant = AcrobotPlant()
    plant.set_name("acrobot")
    diagram = build_acrobot(plant, meshcat)
    simulator, root, context = setup_simulation(diagram)
    simulator.get_mutable_integrator().set_target_accuracy(1e-9)
    target = np.array([math.pi, 0., 0., 0.])
    gain = finite_horizon_lqr(plant, timestep=0.1, horizon=100)
    # build_acrobot configures the shared robot appearance and camera.
    simulator.Initialize()
    meshcat.StartRecording(frames_per_second=30)
    diagram.ForcedPublish(root)
    for step in range(100):
        error = plant.get_output_port(0).Eval(context) - target
        error[:2] = np.arctan2(np.sin(error[:2]), np.cos(error[:2]))
        plant.get_input_port(0).FixValue(context, [float((-gain @ error)[0])])
        simulator.AdvanceTo((step + 1) * .1)
    diagram.ForcedPublish(root)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    error = context.get_continuous_state_vector().CopyToVector() - target
    assert np.max(np.abs(error)) < 1e-3, error
    output = Path(__file__).with_name("acrobot_video") / "drake_meshcat.html"
    output.parent.mkdir(exist_ok=True)
    output.write_text(meshcat.StaticHtml())
    print(f"Saved {output}; final state error: {error}")


if __name__ == "__main__":
    main()
