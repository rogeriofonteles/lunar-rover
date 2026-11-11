"""ROS-free UR5 / STOMP timing experiment using Drake geometry queries."""
from pathlib import Path
import argparse
import csv
import ctypes as ct
import importlib.metadata
import json
import os
import platform
import time
import traceback
import numpy as np
from pydrake.all import (RobotDiagramBuilder, RigidTransform, Box, CoulombFriction,
                        SceneGraphCollisionChecker)

ROOT = Path(__file__).resolve().parent
START = np.deg2rad([0, -90, 90, -90, -90, 0])
TARGETS = {
    'nearby': [20, -85, 95, -100, -85, 15],
    'left_reach': [110, -60, 75, -105, -90, 0],
    'right_reach': [-80, -60, 75, -105, -90, 0],
    'low_reach': [35, -25, 45, -110, -90, 30],
    'high_fold': [-35, -120, 110, -80, -70, -30],
    'across_pole': [150, -70, 85, -105, -90, 0],
}

def make_scene(scene):
    builder = RobotDiagramBuilder(time_step=0.)
    robot = builder.parser().AddModels(str(ROOT / 'assets/ur5.urdf'))[0]
    plant = builder.plant()
    if scene == 'pole':
        plant.RegisterCollisionGeometry(plant.world_body(),
            RigidTransform([0.0, 0.50, 0.40]), Box(.12,.12,.80),
            'pole', CoulombFriction(.9,.5))
    plant.Finalize()
    checker = SceneGraphCollisionChecker(model=builder.Build(),
        robot_model_instances=[robot], edge_step_size=.025,
        env_collision_padding=0., self_collision_padding=0.)
    diagram = checker.model()
    context = diagram.CreateDefaultContext()
    plant = diagram.plant()
    pc = plant.GetMyMutableContextFromRoot(context)
    sg = diagram.scene_graph()
    sc = sg.GetMyMutableContextFromRoot(context)
    query = sg.get_query_output_port()
    return checker, plant, pc, query, sc

class Costs:
    def __init__(self, plant, pc, query, sc):
        self.plant, self.pc, self.query, self.sc = plant, pc, query, sc
        self.calls = self.states = 0
        self.seconds = 0.
        self.error = None

    def distances(self, q, influence=.05):
        self.plant.SetPositions(self.pc, q)
        return self.query.Eval(self.sc).ComputeSignedDistancePairwiseClosestPoints(influence)

    def evaluate(self, q):
        costs = np.zeros(len(q))
        valid = True
        for i, state in enumerate(q):
            distances = self.distances(state)
            self.states += 1
            for pair in distances:
                # Smooth clearance penalty plus penetration penalty.
                costs[i] += 100.*max(0., .05-pair.distance)**2
                valid = valid and pair.distance >= .002
        # Small joint-acceleration penalty; unlike the original paper's
        # workspace-speed-weighted cost, this task uses joint-space curvature.
        costs[1:-1] += .01*np.sum(np.diff(q,n=2,axis=0)**2,axis=1)
        return costs, valid

    def callback(self, raw, n, out, valid):
        started = time.perf_counter()
        try:
            q = np.ctypeslib.as_array(raw, shape=(n*6,)).reshape(n,6)
            c, ok = self.evaluate(q)
            np.ctypeslib.as_array(out, shape=(n,))[:] = c
            valid[0] = int(ok)
            self.calls += 1
            return 1
        except Exception:
            self.error = traceback.format_exc()
            return 0
        finally:
            self.seconds += time.perf_counter()-started

PTR = ct.POINTER(ct.c_double)
CALLBACK = ct.CFUNCTYPE(ct.c_int, PTR, ct.c_int, PTR, ct.POINTER(ct.c_int))

def load_library():
    lib = ct.CDLL(str(ROOT / 'libstomp_bridge.so'))
    lib.run_stomp.argtypes = [PTR,ct.c_int,ct.c_int,ct.c_int,ct.c_uint,
        ct.c_double,ct.c_double,PTR,PTR,CALLBACK,PTR,ct.POINTER(ct.c_int),
        ct.POINTER(ct.c_int)]
    lib.run_stomp.restype = ct.c_int
    return lib

def ptr(a):
    return a.ctypes.data_as(PTR)

def validate(checker, trajectory):
    # Discrete edge checks at <= .025 rad Euclidean joint-space increments.
    # This is denser than optimizer waypoints, NOT continuous certification.
    return all(checker.CheckEdgeCollisionFree(a,b)
               for a,b in zip(trajectory[:-1],trajectory[1:]))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds',type=int,default=10)
    parser.add_argument('--budget',type=float,default=10.)
    parser.add_argument('--waypoints',type=int,default=100)
    parser.add_argument('--rollouts',type=int,default=20)
    parser.add_argument('--iterations',type=int,default=500)
    parser.add_argument('--noise',type=float,default=.6)
    parser.add_argument('--scenes',nargs='+',choices=['open','pole'],default=['open','pole'])
    parser.add_argument('--targets',nargs='+',choices=list(TARGETS),default=list(TARGETS))
    parser.add_argument('--output',default='results')
    args = parser.parse_args()
    if min(args.seeds,args.rollouts,args.iterations)<1 or args.waypoints<8 or args.budget<=0:
        parser.error('Require positive counts/budget and at least 8 waypoints')
    output = ROOT / args.output
    output.mkdir(parents=True,exist_ok=True)
    lib = load_library()
    rows, poses = [], {}
    for scene in args.scenes:
        setup_start = time.perf_counter()
        checker, plant, pc, query, sc = make_scene(scene)
        scene_setup_s = time.perf_counter()-setup_start
        assert plant.num_positions()==6
        costs = Costs(plant,pc,query,sc)
        lower = np.ascontiguousarray(plant.GetPositionLowerLimits())
        upper = np.ascontiguousarray(plant.GetPositionUpperLimits())
        for name in args.targets:
            goal = np.deg2rad(TARGETS[name])
            plant.SetPositions(pc,goal)
            pose = plant.EvalBodyPoseInWorld(pc,plant.GetBodyByName('tool0'))
            poses[name] = {'joint_degrees':TARGETS[name],
                'tool_xyz_m':pose.translation().tolist(),
                'tool_rotation_matrix':pose.rotation().matrix().tolist()}
            initial = np.ascontiguousarray(np.linspace(START,goal,args.waypoints))
            # Warm collision backend before timed trials.
            costs.evaluate(initial)
            endpoints_valid = (checker.CheckConfigCollisionFree(START) and
                               checker.CheckConfigCollisionFree(goal))
            initial_valid = validate(checker, initial)
            print(f'{scene}/{name}: endpoints={endpoints_valid}, straight_path={initial_valid}',flush=True)
            for seed in range(args.seeds):
                optimized = initial.copy()
                costs.calls = costs.states = 0
                costs.seconds = 0.
                it, timeout = ct.c_int(), ct.c_int()
                t0 = time.perf_counter()
                if endpoints_valid:
                    callback = CALLBACK(costs.callback)
                    status = lib.run_stomp(ptr(initial),args.waypoints,args.iterations,
                        args.rollouts,seed,args.budget,args.noise,ptr(lower),ptr(upper),
                        callback,ptr(optimized),ct.byref(it),ct.byref(timeout))
                else:
                    status = 0
                planning_s = time.perf_counter()-t0
                if costs.error:
                    raise RuntimeError(costs.error)
                if status == -1:
                    raise RuntimeError('C++ STOMP bridge failed')
                vt = time.perf_counter()
                endpoint_match = (np.allclose(optimized[0],START,atol=1e-10,rtol=0) and
                                  np.allclose(optimized[-1],goal,atol=1e-10,rtol=0))
                limits_ok = bool(np.all(optimized>=lower-1e-10) and np.all(optimized<=upper+1e-10))
                dense_valid = bool(endpoints_valid and validate(checker,optimized))
                validation_s = time.perf_counter()-vt
                success = bool(status==1 and dense_valid and endpoint_match and limits_ok)
                row = dict(scene=scene,target=name,seed=seed,success=success,
                    optimizer_success=status==1,timed_out=bool(timeout.value),
                    endpoints_valid=endpoints_valid,initial_valid=initial_valid,
                    dense_valid=dense_valid,endpoint_match=endpoint_match,limits_ok=limits_ok,
                    planning_s=planning_s,validation_s=validation_s,
                    total_s=planning_s+validation_s,cost_callback_s=costs.seconds,
                    iterations=it.value,cost_calls=costs.calls,state_queries=costs.states,
                    scene_setup_s=scene_setup_s)
                rows.append(row)
                np.save(output / f'{scene}_{name}_{seed}.npy',optimized)
                with (output/'trials.csv').open('w',newline='') as f:
                    writer=csv.DictWriter(f,fieldnames=list(row))
                    writer.writeheader(); writer.writerows(rows)
                print(f'  seed={seed} success={success} planning={planning_s:.3f}s '
                      f'validation={validation_s:.3f}s iterations={it.value}',flush=True)
    metadata = dict(arguments=vars(args),python=platform.python_version(),
        drake=importlib.metadata.version('drake'),numpy=np.__version__,
        platform=platform.platform(),cpu_count=os.cpu_count(),
        start_joint_degrees=np.rad2deg(START).tolist(),target_poses=poses,
        provenance=json.loads((ROOT/'provenance.json').read_text()))
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    summaries=[]
    for scene in args.scenes:
        for name in args.targets:
            group=[r for r in rows if r['scene']==scene and r['target']==name]
            times=np.array([r['planning_s'] for r in group])
            successful=[r['planning_s'] for r in group if r['success']]
            summaries.append(dict(scene=scene,target=name,trials=len(group),
                successes=len(successful),mean_s=float(times.mean()),
                median_s=float(np.median(times)),p95_s=float(np.percentile(times,95)),
                max_s=float(times.max()),
                success_mean_s=float(np.mean(successful)) if successful else None))
    (output/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
    print(json.dumps(summaries,indent=2))

if __name__=='__main__':
    main()
