"""Check model axes, mixed-unit collision handling, and stored trial invariants."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from benchmark import Costs
from gantry_model import (make_gantry_scene,indices_and_scales,start_and_goal,
                         nominal_mount,gantry_names)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('results',type=Path);args=ap.parse_args()
    metadata=json.loads((args.results/'metadata.json').read_text())
    config=metadata['gantry'];second=config.get('second_axis','z')
    rng=np.random.default_rng(871)
    with (args.results/'trials.csv').open() as f: rows=list(csv.DictReader(f))
    for scene in ('open','bar'):
        checker,plant,pc,query,sc=make_gantry_scene(scene,config)
        indices,scales=indices_and_scales(plant,config)
        assert indices==metadata['joint_indices']
        costs=Costs(plant,pc,query,sc)
        s,_=start_and_goal(plant,config,'nearby')
        plant.SetPositions(pc,s)
        base=plant.EvalBodyPoseInWorld(pc,plant.GetBodyByName('base_link'))
        assert np.allclose(base.translation(),nominal_mount(config))
        assert np.allclose(base.rotation().matrix()@np.array([0,0,1]),[1,0,0])
        for name,delta in [('gantry_x',np.array([.1,0,0])),
                           ('gantry_'+second,np.array([0,.1,0]) if second=='y' else np.array([0,0,.1]))]:
            q=s.copy();q[indices[name]]+=.1;plant.SetPositions(pc,q)
            actual=plant.EvalBodyPoseInWorld(pc,plant.GetBodyByName('base_link'))
            assert np.allclose(actual.translation()-base.translation(),delta)
        q=s.copy();q[indices['gantry_yaw']]=np.pi/2;plant.SetPositions(pc,q)
        actual=plant.EvalBodyPoseInWorld(pc,plant.GetBodyByName('base_link'))
        assert np.allclose(actual.rotation().matrix()@np.array([0,0,1]),[0,1,0],atol=1e-12)
        collisions=0
        for _ in range(100):
            q=rng.uniform(plant.GetPositionLowerLimits(),plant.GetPositionUpperLimits())
            free=all(d.distance>=0 for d in costs.distances(q))
            assert free==checker.CheckConfigCollisionFree(q)
            collisions+=not free
        for row in rows:
            if row['scene']!=scene:continue
            filename=f"{scene}_{row['mode']}_{row['target']}_{row['seed']}.npy"
            path=np.load(args.results/filename)
            goaldata=metadata['targets'][f"{row['mode']}/{row['target']}"]
            assert path.shape==(metadata['arguments']['waypoints'],9)
            assert np.all(np.isfinite(path))
            assert np.allclose(path[0],goaldata['start_q'],atol=1e-10,rtol=0)
            assert np.allclose(path[-1],goaldata['goal_q'],atol=1e-10,rtol=0)
            assert np.all(path>=plant.GetPositionLowerLimits()-1e-10)
            assert np.all(path<=plant.GetPositionUpperLimits()+1e-10)
            if row['mode']=='locked6':
                cols=[indices[n] for n in gantry_names(config)]
                assert np.all(path[:,cols]==path[0,cols])
            if row['success']=='True':
                assert row['dense_valid']=='True' and row['optimizer_success']=='True'
        print(scene,': axes and 100 collision comparisons verified;',collisions,'colliding states')
    print(len(rows),'saved paths checked for finite values, endpoints, limits and locked gantry coordinates')

if __name__=='__main__':main()
