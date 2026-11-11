"""Compare a fixed torso arm, free 9-DOF gantry, and moving gantry goals."""
import argparse
import csv
import ctypes as ct
import importlib.metadata
import json
import platform
import time
import traceback
import numpy as np
from benchmark import ROOT, TARGETS, Costs, CALLBACK, PTR, ptr, validate, load_library
from gantry_model import (DEFAULT, ARM_NAMES, gantry_names, indices_and_scales,
                          start_and_goal, make_gantry_scene)

MODES=['locked6','free9','moving9']

class GantryCosts(Costs):
    def __init__(self,plant,pc,query,sc,active,fixed,scales):
        super().__init__(plant,pc,query,sc)
        self.active=np.array(active); self.fixed=fixed; self.scales=scales
    def expand(self,q):
        full=np.tile(self.fixed,(len(q),1))
        full[:,self.active]=q
        return full
    def evaluate(self,q):
        full=self.expand(q)
        costs=np.zeros(len(q)); valid=True
        for i,state in enumerate(full):
            for pair in self.distances(state):
                costs[i]+=100.*max(0.,.05-pair.distance)**2
                valid=valid and pair.distance>=.002
            self.states+=1
        # Normalize metres/radians before comparing joint-space curvature.
        normalized=q/self.scales[self.active]
        costs[1:-1]+=.01*np.sum(np.diff(normalized,n=2,axis=0)**2,axis=1)
        return costs,valid
    def callback(self,raw,n,out,valid):
        started=time.perf_counter()
        try:
            d=len(self.active)
            q=np.ctypeslib.as_array(raw,shape=(n*d,)).reshape(n,d)
            c,ok=self.evaluate(q)
            np.ctypeslib.as_array(out,shape=(n,))[:]=c
            valid[0]=int(ok); self.calls+=1
            return 1
        except Exception:
            self.error=traceback.format_exc(); return 0
        finally: self.seconds+=time.perf_counter()-started

def write_rows(path,rows):
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',help='JSON overriding gantry model defaults')
    ap.add_argument('--seeds',type=int,default=10)
    ap.add_argument('--waypoints',type=int,default=100)
    ap.add_argument('--rollouts',type=int,default=20)
    ap.add_argument('--iterations',type=int,default=500)
    ap.add_argument('--budget',type=float,default=10.)
    ap.add_argument('--noise',type=float,default=.6,help='Normalized noise; pris noise = this * translation_scale')
    ap.add_argument('--modes',nargs='+',choices=MODES,default=MODES)
    ap.add_argument('--scenes',nargs='+',choices=['open','bar'],default=['open','bar'])
    ap.add_argument('--targets',nargs='+',choices=list(TARGETS),default=list(TARGETS))
    ap.add_argument('--output',default='gantry_results')
    args=ap.parse_args()
    if min(args.seeds,args.rollouts,args.iterations)<1 or args.waypoints<8 or args.budget<=0 or args.noise<=0:
        ap.error('Require positive counts, noise/budget and at least 8 waypoints')
    config=DEFAULT.copy()
    if args.config:
        from pathlib import Path
        overrides=json.loads(Path(args.config).read_text())
        if set(overrides)-set(config): ap.error('Unknown gantry configuration keys')
        config.update(overrides)
    for key in ('x_limits','slide_limits','yaw_limits'):
        if len(config[key])!=2 or config[key][0]>=config[key][1]: ap.error(f'Invalid {key}')
    if config['translation_scale']<=0: ap.error('translation_scale must be positive')
    if config['second_axis'] not in ('y','z'): ap.error('second_axis must be y or z')
    output=ROOT/args.output;output.mkdir(parents=True,exist_ok=True)
    if (output/'trials.csv').exists():
        ap.error(f'{output} already contains trials; choose a new --output directory')
    lib=load_library()
    lib.run_stomp_nd.argtypes=[PTR,ct.c_int,ct.c_int,ct.c_int,ct.c_int,ct.c_uint,
        ct.c_double,PTR,PTR,PTR,CALLBACK,PTR,ct.POINTER(ct.c_int),ct.POINTER(ct.c_int)]
    lib.run_stomp_nd.restype=ct.c_int
    metadata=dict(arguments=vars(args),gantry=config,python=platform.python_version(),
        drake=importlib.metadata.version('drake'),numpy=np.__version__,
        provenance=json.loads((ROOT/'provenance.json').read_text()),targets={},
        timer='perf_counter; includes adapter and STOMP construction; excludes final validation, IK and execution')
    rows=[]
    for scene in args.scenes:
        began=time.perf_counter()
        checker,plant,pc,query,sc=make_gantry_scene(scene,config)
        scene_setup_s=time.perf_counter()-began
        indices,scales=indices_and_scales(plant,config)
        metadata['joint_indices']=indices;metadata['joint_scales']=scales.tolist()
        for mode in args.modes:
            # Arm-first active ordering gives the same arm noise draws in the
            # 6- and 9-dimensional task's first rollout for a given seed.
            active=[indices[n] for n in ARM_NAMES]
            if mode!='locked6': active += [indices[n] for n in gantry_names(config)]
            lower=np.ascontiguousarray(plant.GetPositionLowerLimits()[active])
            upper=np.ascontiguousarray(plant.GetPositionUpperLimits()[active])
            noise=np.ascontiguousarray(args.noise*scales[active])
            for target in args.targets:
                start,goal=start_and_goal(plant,config,target,moving=mode=='moving9')
                plant.SetPositions(pc,goal)
                pose=plant.EvalBodyPoseInWorld(pc,plant.GetBodyByName('tool0'))
                metadata['targets'][f'{mode}/{target}']=dict(start_q=start.tolist(),goal_q=goal.tolist(),
                    tool_xyz_m=pose.translation().tolist(),tool_rotation=pose.rotation().matrix().tolist(),
                    active_indices=active,noise_stddev=noise.tolist())
                initial=np.ascontiguousarray(np.linspace(start[active],goal[active],args.waypoints))
                costs=GantryCosts(plant,pc,query,sc,active,start,scales)
                costs.evaluate(initial) # warm up geometry backend
                endpoints_valid=bool(checker.CheckConfigCollisionFree(start) and checker.CheckConfigCollisionFree(goal)
                    and np.all(goal>=plant.GetPositionLowerLimits()) and np.all(goal<=plant.GetPositionUpperLimits()))
                initial_valid=validate(checker,costs.expand(initial))
                print(f'{scene}/{mode}/{target}: endpoints={endpoints_valid} straight_path={initial_valid}',flush=True)
                (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
                for seed in range(args.seeds):
                    result=initial.copy();costs.calls=costs.states=0;costs.seconds=0.
                    it,timeout=ct.c_int(),ct.c_int();t0=time.perf_counter()
                    if endpoints_valid:
                        cb=CALLBACK(costs.callback)
                        status=lib.run_stomp_nd(ptr(initial),len(active),args.waypoints,args.iterations,
                            args.rollouts,seed,args.budget,ptr(noise),ptr(lower),ptr(upper),cb,
                            ptr(result),ct.byref(it),ct.byref(timeout))
                    else: status=0
                    planning_s=time.perf_counter()-t0
                    if status==-1 or costs.error: raise RuntimeError(costs.error or 'STOMP bridge failed')
                    full=costs.expand(result); vt=time.perf_counter()
                    endpoints_match=bool(np.allclose(full[0],start,atol=1e-10,rtol=0)
                        and np.allclose(full[-1],goal,atol=1e-10,rtol=0))
                    limits_ok=bool(np.all(full>=plant.GetPositionLowerLimits()-1e-10)
                                  and np.all(full<=plant.GetPositionUpperLimits()+1e-10))
                    dense_valid=bool(endpoints_valid and validate(checker,full))
                    validation_s=time.perf_counter()-vt
                    success=bool(status==1 and endpoints_match and limits_ok and dense_valid)
                    gantry_peak=np.max(np.abs(full[:,[indices[n] for n in gantry_names(config)]]
                        -start[[indices[n] for n in gantry_names(config)]]),axis=0)
                    reason=('success' if success else 'invalid_endpoints' if not endpoints_valid else
                            'timeout' if timeout.value else 'validation_failure' if status==1 else 'iteration_limit')
                    rows.append(dict(scene=scene,mode=mode,target=target,seed=seed,success=success,
                        reason=reason,optimizer_success=status==1,timed_out=bool(timeout.value),
                        endpoints_valid=endpoints_valid,initial_valid=initial_valid,dense_valid=dense_valid,
                        endpoints_match=endpoints_match,limits_ok=limits_ok,planning_s=planning_s,
                        validation_s=validation_s,total_s=planning_s+validation_s,cost_callback_s=costs.seconds,
                        iterations=it.value,cost_calls=costs.calls,state_queries=costs.states,scene_setup_s=scene_setup_s,
                        max_x_excursion_m=gantry_peak[0],max_slide_excursion_m=gantry_peak[1],
                        max_yaw_excursion_rad=gantry_peak[2]))
                    np.save(output/f'{scene}_{mode}_{target}_{seed}.npy',full)
                    write_rows(output/'trials.csv',rows)
                    print(f' seed={seed} {reason} {planning_s:.3f}s iterations={it.value}',flush=True)
    summaries=[]
    for scene in args.scenes:
        for mode in args.modes:
            for target in args.targets:
                group=[r for r in rows if (r['scene'],r['mode'],r['target'])==(scene,mode,target)]
                times=[r['planning_s'] for r in group if r['endpoints_valid']]
                successful=[r['planning_s'] for r in group if r['success']]
                summaries.append(dict(scene=scene,mode=mode,target=target,trials=len(group),
                    successes=len(successful),timeouts=sum(r['timed_out'] for r in group),
                    invalid_endpoints=sum(not r['endpoints_valid'] for r in group),
                    mean_s=float(np.mean(times)) if times else None,
                    median_s=float(np.median(times)) if times else None,
                    max_s=float(np.max(times)) if times else None,
                    success_mean_s=float(np.mean(successful)) if successful else None))
    (output/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
    print('Saved',output,flush=True)

if __name__=='__main__':main()
