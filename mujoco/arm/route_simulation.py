"""Run a continuous button route on the nonlinear MuJoCo plant."""
from contextlib import nullcontext
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from .buttons import button_target
from .planning import CollisionWorld, plan_route, show_planning_obstacles
from .model import ArmModel
from .simulate import load_config
from .tracking import TrajectoryController, prepare_nominal

LABELS={"infinite":"LQR infinito", "finite":"LQR finito", "receding":"LQR em horizonte móvel",
        "ilqr":"iLQR · plano completo", "ilqr_receding":"iLQR em horizonte móvel",
        "mpc_sqp":"MPC · SQP", "mpc_slsqp":"MPC · SLSQP", "mpc_dual_ascent":"MPC · Dual-Ascent"}


def run_route(config,arm,route,nominal,headless=True,wait=False,output=None):
    start=time.perf_counter()
    controller=TrajectoryController(arm,nominal,config)
    world=CollisionWorld(arm)
    x=route.states[0].copy()
    states=[x.copy()];controls=[];timings=[];solver_log=[]
    termination=None
    clearance=[world.clearance(x[:6])]
    show_planning_obstacles(arm.model)
    display=mujoco.MjData(arm.model)
    display.qpos[:]=x[:6];mujoco.mj_forward(arm.model,display)
    context=nullcontext(None)
    if not headless:
        from .viewer import native_viewer
        context=native_viewer(arm.model,display,True,False)
    with context as viewer:
        if viewer is not None:
            viewer.cam.lookat[:]=[0,0,1.35]
            viewer.cam.distance,viewer.cam.azimuth,viewer.cam.elevation=3.73,59,-20
        for k in range(len(route.states)-1):
            start_step=time.perf_counter()
            if viewer is not None and not viewer.is_running():
                break
            u=controller.command(x,k)
            timings.append(time.perf_counter()-start_step)
            solver_log.append(controller.last_info.copy())
            x=arm.step(x,u)
            distance=world.clearance(x[:6])
            states.append(x.copy());controls.append(u.copy());clearance.append(distance)
            if distance < 0:
                termination=f"Colisão geométrica detectada em t={(k+1)*arm.dt:.3f}s"
                print(termination,flush=True)
                break
            if viewer is not None:
                with viewer.lock():
                    display.qpos[:],display.qvel[:],display.ctrl[:]=x[:6],x[6:],u
                    display.time=(k+1)*arm.dt
                    mujoco.mj_forward(arm.model,display)
                viewer.sync()
                time.sleep(max(0,arm.dt-(time.perf_counter()-start_step)))
            if k%200==0:
                print(f"{config['mode']} / {config['solver']}: passo {k+1}/{len(route.states)-1}",flush=True)
        if viewer is not None and wait:
            while viewer.is_running():
                viewer.sync();time.sleep(.05)
    states=np.asarray(states);controls=np.asarray(controls).reshape(-1,6)
    times=np.arange(len(states))*arm.dt
    reference=route.states[:len(states)]
    actual_tip=np.array([world.points(q)[-1] for q in states[:,:6]])
    planned_tip=np.array([world.points(q)[-1] for q in reference[:,:6]])
    qerror=states[:,:6]-reference[:,:6]
    visits=[]
    for visit in route.arrivals:
        idx=visit['index']
        if idx>=len(states):
            continue
        position=button_target(visit['button'])['position_m']
        error=float(np.linalg.norm(actual_tip[idx]-position))
        visits.append(dict(button=visit['button'],index=idx,time_s=times[idx],tip_error_m=error,
                           speed_rad_s=float(np.max(np.abs(states[idx,6:]))),
                           reached=bool(error<.01 and np.max(np.abs(states[idx,6:]))<.05)))
    key=config['mode'] if config['mode']!='mpc' else 'mpc_'+config['solver']
    summary=dict(key=key,label=LABELS[key],mode=config['mode'],solver=config['solver'] if config['mode']=='mpc' else None,
                 horizon=config['horizon'],discretization=config['discretization'],sequence=route.sequence,
                 completed=len(states)==len(route.states) and termination is None,termination=termination,
                 steps=len(controls),simulated_seconds=float(times[-1]),
                 wall_seconds=time.perf_counter()-start,controller_setup_seconds=controller.setup_seconds,
                 nominal_preparation_seconds=nominal.seconds,planning=route.planning,
                 initial_state=states[0].tolist(),visits=visits,
                 rms_joint_error_rad=float(np.sqrt(np.mean(qerror**2))),
                 max_joint_error_rad=float(np.max(np.abs(qerror))),
                 rms_tip_error_m=float(np.sqrt(np.mean(np.sum((actual_tip-planned_tip)**2,axis=1)))),
                 min_clearance_m=float(np.min(clearance)),
                 peak_torque_nm=np.max(np.abs(controls),axis=0).tolist() if len(controls) else [0.]*6,
                 command_median_ms=float(np.median(timings)*1000) if timings else 0.,
                 command_p95_ms=float(np.percentile(timings,95)*1000) if timings else 0.,
                 deadline_misses=int(np.sum(np.array(timings)>arm.dt)),
                 nonconverged_solves=sum(not s['converged'] for s in solver_log) if config['mode'] in ('mpc','ilqr_receding') else 0,
                 ilqr_plan=None if controller.plan is None else dict(converged=controller.plan.converged,
                      iterations=controller.plan.iterations,status=controller.plan.status,costs=controller.plan.costs),
                 solver_log=solver_log,config=config)
    arrays=dict(states=states,controls=controls,times=times,reference=reference,reference_controls=nominal.controls[:len(controls)],
                actual_tip=actual_tip,planned_tip=planned_tip,clearance=np.array(clearance),command_seconds=np.array(timings))
    if output is not None:
        path=Path(output);path.parent.mkdir(parents=True,exist_ok=True)
        np.savez(path.with_suffix('.npz'),**arrays)
        path.with_suffix('.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    return arrays,summary


def prepare(sequence,seed,speed,acceleration,dwell,**options):
    config=load_config(**options)
    arm=ArmModel(config['timestep'],config['physics_timestep'],config['torque_limit'])
    route,world=plan_route(arm,sequence,seed,speed,acceleration,dwell)
    print(f"RRT-Connect: {len(route.states)} estados, {len(route.paths)} trechos",flush=True)
    nominal=prepare_nominal(arm,route,config)
    return config,arm,route,nominal
