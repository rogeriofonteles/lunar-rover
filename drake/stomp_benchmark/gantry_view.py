"""Export a nine-joint gantry trial as a standalone Meshcat animation."""
import argparse
import json
from pathlib import Path
import numpy as np
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
                        Meshcat, MeshcatVisualizer, Rgba)
from gantry_model import model_path,obstacle

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('trajectory',type=Path)
    args=ap.parse_args();trajectory=args.trajectory.resolve()
    metadata=json.loads((trajectory.parent/'metadata.json').read_text())
    config=metadata['gantry'];q=np.load(trajectory)
    scene=trajectory.stem.split('_')[0]
    builder=DiagramBuilder();plant,sg=AddMultibodyPlantSceneGraph(builder,time_step=0.)
    Parser(plant).AddModels(str(model_path(config)))
    if scene=='bar':
        transform,shape=obstacle(config)
        plant.RegisterVisualGeometry(plant.world_body(),transform,shape,'crossbar',
                                     np.array([.85,.35,.1,.8]))
    plant.Finalize();assert plant.num_positions()==9
    meshcat=Meshcat();MeshcatVisualizer.AddToBuilder(builder,sg,meshcat)
    diagram=builder.Build();context=diagram.CreateDefaultContext()
    pc=plant.GetMyMutableContextFromRoot(context)
    def path(states,body):
        points=[]
        for state in states:
            plant.SetPositions(pc,state)
            points.append(plant.EvalBodyPoseInWorld(pc,plant.GetBodyByName(body)).translation())
        return np.array(points).T
    meshcat.SetLine('paths/initial_tool',path(np.linspace(q[0],q[-1],len(q)),'tool0'),
                    line_width=3,rgba=Rgba(.9,.2,.2,1))
    meshcat.SetLine('paths/optimized_tool',path(q,'tool0'),line_width=3,rgba=Rgba(.1,.6,.2,1))
    meshcat.SetLine('paths/gantry_mount',path(q,'base_link'),line_width=3,rgba=Rgba(.1,.3,.9,1))
    meshcat.SetCameraPose([2.,-2.1,1.9],[.15,.10,1.])
    meshcat.StartRecording(set_visualizations_while_recording=True)
    for t,state in zip(np.linspace(0,5,len(q)),q):
        context.SetTime(float(t));plant.SetPositions(pc,state);diagram.ForcedPublish(context)
    meshcat.StopRecording();meshcat.PublishRecording()
    output=trajectory.with_suffix('.html');output.write_text(meshcat.StaticHtml())
    print(output)

if __name__=='__main__':main()
