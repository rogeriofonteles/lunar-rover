"""Export a saved trial as a standalone Meshcat HTML animation."""
import argparse
from pathlib import Path
import numpy as np
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
    Meshcat, MeshcatVisualizer, Box, RigidTransform, Rgba)
from benchmark import ROOT

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('trajectory',type=Path)
    args=ap.parse_args()
    trajectory=args.trajectory.resolve()
    q=np.load(trajectory)
    scene=trajectory.stem.split('_')[0]
    builder=DiagramBuilder()
    plant,sg=AddMultibodyPlantSceneGraph(builder,time_step=0.)
    Parser(plant).AddModels(str(ROOT/'assets/ur5.urdf'))
    if scene=='pole':
        plant.RegisterVisualGeometry(plant.world_body(),
            RigidTransform([0.,.50,.40]),Box(.12,.12,.80),'pole',
            np.array([.85,.35,.1,.8]))
    plant.Finalize()
    meshcat=Meshcat()
    MeshcatVisualizer.AddToBuilder(builder,sg,meshcat)
    diagram=builder.Build()
    context=diagram.CreateDefaultContext()
    pc=plant.GetMyMutableContextFromRoot(context)
    def tool_path(states):
        points=[]
        for state in states:
            plant.SetPositions(pc,state)
            points.append(plant.EvalBodyPoseInWorld(pc,plant.GetBodyByName('tool0')).translation())
        return np.array(points).T
    meshcat.SetLine('paths/initial',tool_path(np.linspace(q[0],q[-1],len(q))),
                    line_width=3,rgba=Rgba(.9,.2,.2,1))
    meshcat.SetLine('paths/optimized',tool_path(q),line_width=3,rgba=Rgba(.1,.6,.2,1))
    meshcat.SetCameraPose([1.6,-1.7,1.3],[0.,.15,.35])
    meshcat.StartRecording(set_visualizations_while_recording=True)
    # Five seconds is visualization playback time, not optimized motion timing.
    for t,state in zip(np.linspace(0,5,len(q)),q):
        context.SetTime(float(t)); plant.SetPositions(pc,state)
        diagram.ForcedPublish(context)
    meshcat.StopRecording(); meshcat.PublishRecording()
    dest=trajectory.with_suffix('.html')
    dest.write_text(meshcat.StaticHtml())
    print(dest)

if __name__=='__main__':
    main()
