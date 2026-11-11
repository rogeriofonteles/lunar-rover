"""Reproducible offline experiments using the project's controllers and Drake.
Run in lunar-rover-jazzy-drake from ~/drake. Never uses the mutable YAML.
"""
import argparse
import copy
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import sys
import warnings
from time import perf_counter
from datetime import datetime, timezone

import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.systems.analysis import Simulator

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent.parent))
from acrobot.lqr import controller_gains, control_torque, wrapped_state_error, _linear_model, _riccati_step
from acrobot.ilqr import iterative_lqr, RecedingHorizonILQR
from acrobot.linear_mpc import LinearMPC
from acrobot.nonlinear_sqp import RecedingHorizonSQP

TARGET = np.array([math.pi, 0., 0., 0.])
MODES = ['finite', 'infinite', 'ilqr', 'mpc', 'mpc_lqr', 'mpc_sqp', 'mpc_slsqp', 'mpc_nonlinear_sqp', 'mpc_nonlinear_slsqp']
VARIANTS = ['original', 'clipped', 'unclipped']

def configurations(variants=VARIANTS):
    for s in SCENARIOS:
        for variant in variants:
            cfg=BASE|s
            cfg['challenge_id']=s['id']; cfg['torque_mode']=variant
            if variant!='original':
                cfg['id']=s['id']+'__'+variant
                cfg['limit']=3. if variant=='clipped' else None
            yield cfg
BASE = dict(dt=.05, duration=6., x0=[math.pi-.03,.03,0.,0.], Q=[10.,10.,1.,1.], R=.1,
            limit=15., mpc_horizon=2., iterations=80, mpc_iterations=6,
            regularization=1e-5, initialization='auto', terminal='Q', kick=None)
SCENARIOS = [
    dict(id='near', title='01 · Perto do equilíbrio', subtitle='O ponto de partida: quando a aproximação linear funciona.', **{}),
    dict(id='far', title='02 · Longe do equilíbrio', subtitle='Um desvio de 57° muda a geometria do problema.', x0=[math.pi-1.,1.,0.,0.]),
    dict(id='saturation', title='03 · O atuador tem limite', subtitle='Mesmo estado inicial distante, agora com apenas ±3 N·m.', x0=[math.pi-1.,1.,0.,0.], limit=3.),
    dict(id='disturbance', title='04 · O mundo muda no caminho', subtitle='Um salto de velocidade em t = 2 s testa a recuperação.', kick=dict(time=2., delta=[0.,0.,1.5,-2.])),
    dict(id='short', title='05 · Um horizonte míope', subtitle='O MPC olha apenas 0,20 s à frente; seu custo terminal é Q.', mpc_horizon=.2, mpc_terminal='Q'),
    dict(id='expensive', title='06 · Economizar torque custa', subtitle='R sobe de 0,1 para 100; todos passam a penalizar mais o esforço.', R=100.),
    dict(id='budget', title='07 · Uma única iteração', subtitle='Estado distante, semente local e orçamento de uma iteração.', x0=[math.pi-1.,1.,0.,0.], iterations=1, mpc_iterations=1, initialization='lqr'),
    dict(id='damping', title='08 · Regularização excessiva', subtitle='O amortecimento numérico sobe para 10⁶; não é atrito físico.', x0=[math.pi-1.,1.,0.,0.], regularization=1e6, initialization='lqr', iterations=12, mpc_iterations=2),
    dict(id='terminal', title='09 · O fim do horizonte importa', subtitle='Uma perturbação tardia compara Q com P∞ no LQR finito.', kick=dict(time=5., delta=[0.,0.,.4,-.6]), terminal='Pinf'),
    dict(id='late', title='10 · Sem cuidar do depois', subtitle='A mesma perturbação tardia, mas o LQR finito termina com Q.', kick=dict(time=5., delta=[0.,0.,.4,-.6])),
    dict(id='nonlinear', title='11 · A trajetória faz diferença', subtitle='Um desvio de 17° testa a vantagem de otimizar a trajetória.', x0=[math.pi-.3,.3,0.,0.]),
]


def signature(cfg, mode):
    h = hashlib.sha256(json.dumps([cfg, mode], sort_keys=True).encode())
    for f in [Path(__file__), ROOT.parent/'lqr.py', ROOT.parent/'ilqr.py', ROOT.parent/'trajectory_seed.py', ROOT.parent/'linear_mpc.py', ROOT.parent/'nonlinear_sqp.py']:
        h.update(f.read_bytes())
    return h.hexdigest()


def run(cfg, mode):
    start = perf_counter()
    dt = cfg['dt']; n = round(cfg['duration']/dt)
    plant = AcrobotPlant(); model = AcrobotPlant()
    sim = Simulator(plant)
    sim.get_mutable_integrator().set_target_accuracy(1e-9)
    ctx = sim.get_mutable_context(); ctx.SetContinuousState(cfg['x0'])
    port = plant.get_input_port(0); port.FixValue(ctx, [0.])
    sim.Initialize()
    Q = np.diag(cfg['Q']); R = cfg['R']; bound = cfg['limit']
    options = dict(timestep=dt, horizon=n, Q=Q, R=R, torque_limit=bound,
                   max_iterations=cfg['iterations'], tolerance=1e-5,
                   regularization=cfg['regularization'], initialization=cfg['initialization'])
    plan = None; receding = None; linear = None; solves = []; warning_messages=[]
    def record(result, seconds, k):
        solves.append(dict(step=k, time=k*dt, ms=seconds*1000, converged=bool(result.converged),
                           iterations=result.iterations, status=result.status, initialization=result.initialization,
                           costs=[float(c) for c in result.costs],
                           residual=getattr(result,'residual',None), bound_violation=getattr(result,'bound_violation',None),
                           initial_cost=getattr(result,'initial_cost',None), candidate_cost=getattr(result,'candidate_cost',None),
                           returned_cost=getattr(result,'returned_cost',None), native_status=getattr(result,'native_status',None),
                           native_success=getattr(result,'native_success',None), variable_scale=getattr(result,'variable_scale',None),
                           candidate_violation=getattr(result,'candidate_violation',None), repair_max_nm=getattr(result,'repair_max_nm',None),
                           recovery=getattr(result,'recovery',None), relative_dual_gap=getattr(result,'relative_dual_gap',None),
                           formulation=getattr(result,'formulation','linear_qp' if linear else 'nonlinear_ilqr'),
                           linearizations=getattr(result,'linearizations',[]),
                           initial_controls=result.initial_controls.tolist() if k==0 and hasattr(result,'initial_controls') and result.initial_controls is not None else None,
                           terminal_cost=result.terminal_cost.tolist() if k==0 and hasattr(result,'terminal_cost') else None,
                           history_kind='iterates' if linear or mode=='mpc_nonlinear_slsqp' else 'accepted'))
    t0 = perf_counter()
    if mode in ('finite', 'infinite'):
        terminal = None
        if cfg['terminal'] == 'Pinf' and mode == 'finite':
            A,B,_ = _linear_model(model,dt,Q,R,None)
            terminal=Q.copy()
            for _ in range(10000):
                _, updated = _riccati_step(A,B,Q,R,terminal)
                if np.max(np.abs(updated-terminal)) < 1e-10*max(1., np.max(abs(terminal))):
                    terminal=updated; break
                terminal=updated
            else:
                raise RuntimeError('Terminal Riccati iteration did not converge')
        gains = controller_gains(model, mode=mode, timestep=dt, horizon=n, Q=Q, R=R, terminal_cost=terminal)
    elif mode == 'ilqr':
        with warnings.catch_warnings(record=True) as notices:
            warnings.simplefilter('always')
            plan = iterative_lqr(model, cfg['x0'], **options)
        warning_messages.extend(str(w.message) for w in notices)
        record(plan, perf_counter()-t0, 0)
    elif mode in ('mpc_lqr','mpc_sqp','mpc_slsqp'):
        solver={'mpc_lqr':'riccati','mpc_sqp':'sqp','mpc_slsqp':'slsqp'}[mode]
        linear=LinearMPC(model,timestep=dt,horizon=round(cfg['mpc_horizon']/dt),Q=Q,R=R,torque_limit=bound,solver=solver,terminal_cost=Q if cfg.get('mpc_terminal')=='Q' else None)
    else:
        options['horizon'] = round(cfg['mpc_horizon']/dt)
        if cfg.get('mpc_terminal') == 'Q': options['terminal_cost'] = Q
        receding = RecedingHorizonILQR(model, **options) if mode=='mpc' else RecedingHorizonSQP(model,solver=mode.removeprefix('mpc_nonlinear_'),**options)
    setup_ms = (perf_counter()-t0)*1000
    states=[]; torques=[]; command_ms=[]; predicted=[]
    A,B,_ = _linear_model(model,dt,Q,R,None)
    for k in range(n):
        if cfg['kick'] and k == round(cfg['kick']['time']/dt):
            x = ctx.get_continuous_state_vector().CopyToVector() + cfg['kick']['delta']
            ctx.SetContinuousState(x)
            sim.Initialize()
        x = ctx.get_continuous_state_vector().CopyToVector()
        states.append(x.tolist())
        tic=perf_counter()
        if linear:
            u=linear.command(x)
            elapsed=perf_counter()-tic
            record(linear.last_result,elapsed,k)
            if k%10==0:predicted.append(dict(step=k,states=linear.last_result.states.tolist()))
        elif receding:
            with warnings.catch_warnings(record=True) as notices:
                warnings.simplefilter('always')
                u = receding.command(x)
            warning_messages.extend(str(w.message) for w in notices)
            elapsed=perf_counter()-tic
            record(receding.last_result, elapsed, k)
            receding.solver_options['max_iterations']=cfg['mpc_iterations']
            if k % 10 == 0:
                predicted.append(dict(step=k, states=receding.last_result.states.tolist(),controls=receding.last_result.controls.tolist()))
        elif plan:
            u = float(plan.controls[k] - (plan.gains[k] @ wrapped_state_error(x, plan.states[k])).item())
            if bound is not None:u = float(np.clip(u,-bound,bound))
            elapsed=perf_counter()-tic
        else:
            gain = gains[k] if mode=='finite' else gains[0]
            u = control_torque(gain,wrapped_state_error(x,TARGET),bound)
            elapsed=perf_counter()-tic
        torques.append(float(u)); command_ms.append(elapsed*1000)
        port.FixValue(ctx,[u]); sim.AdvanceTo((k+1)*dt)
        if k % 40 == 0 and receding:
            print(f"    MPC {k}/{n} · {solves[-1]['ms']:.0f} ms",flush=True)
    states.append(ctx.get_continuous_state_vector().CopyToVector().tolist())
    xs=np.array(states); us=np.array(torques)
    errors=np.array([wrapped_state_error(x,TARGET) for x in xs])
    # Physical success: simultaneous joint tolerances, sustained for >= 1 s through the endpoint.
    inside=(np.max(abs(errors[:,:2]),axis=1)<.05)&(np.max(abs(errors[:,2:]),axis=1)<.1)
    settled=None
    last_out=np.flatnonzero(~inside)
    first=int(last_out[-1]+1) if len(last_out) else 0
    if first <= n-round(1/dt): settled=first*dt
    # Independent nonlinear-model discrepancy: predict one sample with Euler linearization under the APPLIED u.
    residual=[]
    for k,u in enumerate(us):
        ep=A@errors[k]+B[:,0]*u
        actual=errors[k+1].copy()
        if cfg['kick'] and k+1==round(cfg['kick']['time']/dt): actual-=cfg['kick']['delta']
        residual.append(float(np.linalg.norm(actual-ep)))
    solve_ms=[s['ms'] for s in solves]
    online=command_ms[1:] if receding or linear else command_ms
    result=dict(mode=mode, cfg=cfg, warnings=warning_messages, time=(np.arange(n+1)*dt).tolist(), states=states, torque=torques,
                command_ms=command_ms, solves=solves, predictions=predicted,
                nominal=plan.states.tolist() if plan else None,
                metrics=dict(setup_ms=setup_ms, first_solution_ms=(setup_ms+solve_ms[0] if linear else solve_ms[0]) if solves else setup_ms,
                             online_median_ms=float(np.median(online)), online_p95_ms=float(np.percentile(online,95)),
                             online_max_ms=float(max(online)), total_solver_ms=sum(solve_ms) if solves else setup_ms,
                             deadline_misses=int(sum(t>dt*1000 for t in online)), online_samples=len(online),
                             settling_s=settled, final_angle_rad=float(max(abs(errors[-1,:2]))),
                             final_speed_rad_s=float(max(abs(errors[-1,2:]))),
                             peak_torque=float(max(abs(us))), saturation_pct=float(np.mean(abs(us)>=bound-1e-8)*100) if bound is not None else 0.,
                             angle_rms_rad=float(np.sqrt(np.mean(errors[:,:2]**2))),
                             effort=float(dt*np.sum(us**2)),
                             nonconverged=sum(not s['converged'] for s in solves),
                             wall_s=perf_counter()-start),
                linear_residual=residual,
                parameters=dict(zip(['m1','m2','l1','l2','lc1','lc2','Ic1','Ic2','b1','b2','gravity'], map(float, plant.get_parameters(ctx).CopyToVector()))))
    assert np.isfinite(xs).all() and np.isfinite(us).all()
    return result


def export():
    scenarios=[]
    for cfg in configurations():
        runs={}
        for m in MODES:
            p=ROOT/'runs'/f"{cfg['id']}_{m}.json"
            if p.exists(): runs[m]=json.loads(p.read_text())
        if runs: scenarios.append(dict(config=cfg,runs=runs))
    meta=dict(generated_at=datetime.now(timezone.utc).isoformat(), drake=importlib.metadata.version('drake'),
              scipy=importlib.metadata.version('scipy'), python=platform.python_version(), platform=platform.platform(), cpu=platform.processor(),
              threads={k:os.getenv(k) for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS']},
              source_hashes={f:hashlib.sha256((ROOT.parent/f).read_bytes()).hexdigest() for f in ['lqr.py','ilqr.py','trajectory_seed.py','linear_mpc.py','nonlinear_sqp.py']})
    data=dict(metadata=meta,scenarios=scenarios)
    comparison=ROOT/'solver_comparison.json'
    if comparison.exists():data['solver_comparison']=json.loads(comparison.read_text())
    (ROOT/'data.json').write_text(json.dumps(data,ensure_ascii=False,allow_nan=False))
    (ROOT/'data.js').write_text('window.ACROBOT_DATA = '+json.dumps(data,ensure_ascii=False,allow_nan=False)+';\n')


def main():
    p=argparse.ArgumentParser(); p.add_argument('--scenarios',nargs='*'); p.add_argument('--modes',nargs='*',default=MODES,choices=MODES); p.add_argument('--variants',nargs='*',default=['original'],choices=VARIANTS); p.add_argument('--force',action='store_true'); p.add_argument('--export-only',action='store_true')
    args=p.parse_args(); (ROOT/'runs').mkdir(exist_ok=True)
    if not args.export_only:
        measured={}
        for cfg in configurations(args.variants):
            if args.scenarios and cfg['challenge_id'] not in args.scenarios: continue
            for mode in args.modes:
                path=ROOT/'runs'/f"{cfg['id']}_{mode}.json"; digest=signature(cfg,mode)
                if path.exists() and not args.force and json.loads(path.read_text()).get('signature')==digest: continue
                print(f"RUN {cfg['id']} / {mode}",flush=True)
                physical={k:v for k,v in cfg.items() if k not in ['id','title','subtitle','challenge_id','torque_mode']}
                if mode not in ('finite','infinite'):physical.pop('terminal',None)
                key=json.dumps([physical,mode],sort_keys=True)
                if key in measured:
                    result=copy.deepcopy(measured[key]);result['measurement_reused_from']=result['cfg']['id'];result['cfg']=cfg
                    print('    Identical experiment; reuse recorded trajectory and timing from '+result['measurement_reused_from'],flush=True)
                else:
                    result=run(cfg,mode);measured[key]=result
                result['signature']=digest
                path.write_text(json.dumps(result,ensure_ascii=False,allow_nan=False))
                print(json.dumps(result['metrics']),flush=True)
                export()
    export()

if __name__=='__main__': main()
