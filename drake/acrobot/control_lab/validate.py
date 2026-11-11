"""Validate saved experiments and independently replay their commands in Drake.
Run inside the Drake container: python acrobot/control_lab/validate.py
"""
import json
from pathlib import Path
import sys
import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.systems.analysis import Simulator

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent.parent))
from acrobot.lqr import wrapped_state_error
from acrobot.linear_mpc import LinearMPC


def main():
    data=json.loads((ROOT/'data.json').read_text()); assert len(data['scenarios'])==33
    for challenge in {s['config']['challenge_id'] for s in data['scenarios']}:
        assert {s['config']['torque_mode'] for s in data['scenarios'] if s['config']['challenge_id']==challenge}=={'original','clipped','unclipped'}
    maximum=0.; count=0
    for scenario in data['scenarios']:
        cfg=scenario['config'];dt=cfg['dt'];n=round(cfg['duration']/dt)
        linear=LinearMPC(AcrobotPlant(),timestep=dt,horizon=round(cfg['mpc_horizon']/dt),Q=np.diag(cfg['Q']),R=cfg['R'],torque_limit=cfg['limit'],terminal_cost=np.diag(cfg['Q']) if cfg.get('mpc_terminal')=='Q' else None)
        physical_tolerance=1e-8
        assert set(scenario['runs'])=={'finite','infinite','ilqr','mpc','mpc_lqr','mpc_sqp','mpc_slsqp'}
        for mode,r in scenario['runs'].items():
            xs=np.array(r['states']);u=np.array(r['torque']);times=np.array(r['time']);m=r['metrics']
            assert xs.shape==(n+1,4) and u.shape==(n,)
            assert np.isfinite(xs).all() and np.isfinite(u).all()
            np.testing.assert_allclose(times,np.arange(n+1)*dt,atol=1e-12)
            np.testing.assert_allclose(xs[0],cfg['x0'],atol=1e-12)
            if cfg['limit'] is not None:assert max(abs(u))<=cfg['limit']+1e-8
            for solve in r['solves']:
                assert solve['ms']>=0 and solve['iterations']>=0
                if mode in ['ilqr','mpc']:assert all(b<a for a,b in zip(solve['costs'],solve['costs'][1:]))
                if mode.startswith('mpc_'):
                    assert solve['returned_cost']>=0 and solve['initial_cost']>=0
                    assert solve['native_status'] and solve['recovery'] in ['none','box_projection','initial_guess']
                    if mode in ['mpc_sqp','mpc_slsqp']:
                        assert solve['bound_violation']<=physical_tolerance
                        if solve['converged']:assert solve['residual']<=1e-6 and solve['relative_dual_gap']<=1e-6
                        assert solve['returned_cost']<=solve['initial_cost']+1e-7*max(1.,solve['initial_cost'])
                # Independently integrate each saved forecast and recompute returned cost.
            if mode.startswith('mpc_'):
                for prediction in r['predictions']:
                    k=prediction['step'];pred=np.array(prediction['states'])-linear.target
                    controls=np.array([np.linalg.lstsq(linear.B,(right-linear.A@left),rcond=None)[0][0] for left,right in zip(pred[:-1],pred[1:])])
                    np.testing.assert_allclose(pred[1:],pred[:-1]@linear.A.T+controls[:,None]*linear.B[:,0],rtol=1e-8,atol=1e-7)
                    cost=.5*sum(e@linear.Q@e+linear.R*u*u for e,u in zip(pred[:-1],controls))+.5*pred[-1]@linear.Qf@pred[-1]
                    np.testing.assert_allclose(r['solves'][k]['returned_cost'],cost,rtol=1e-7,atol=1e-7)
                    if cfg['limit'] is not None and mode!='mpc_lqr':assert max(abs(controls))<=cfg['limit']+1e-7
            assert len(r['solves'])==(n if mode.startswith('mpc') else 1 if mode=='ilqr' else 0)
            err=wrapped_state_error(xs,[np.pi,0,0,0]);ok=(abs(err[:,:2]).max(axis=1)<.05)&(abs(err[:,2:]).max(axis=1)<.1)
            last=np.flatnonzero(~ok);first=last[-1]+1 if len(last) else 0
            expected=float(first*dt) if first<=n-round(1/dt) else None
            assert m['settling_s']==expected
            assert abs(m['final_angle_rad']-max(abs(err[-1,:2])))<1e-12
            sample=r['command_ms'][1:] if mode.startswith('mpc') else r['command_ms']
            assert m['deadline_misses']==sum(t>dt*1000 for t in sample)
            np.testing.assert_allclose(m['online_p95_ms'],np.percentile(sample,95))
            # Independent execution, using saved torques rather than controller calls.
            plant=AcrobotPlant();sim=Simulator(plant);sim.get_mutable_integrator().set_target_accuracy(1e-9)
            context=sim.get_mutable_context();context.SetContinuousState(cfg['x0']);port=plant.get_input_port(0);port.FixValue(context,[0.]);sim.Initialize()
            replay=[]
            for k,torque in enumerate(u):
                if cfg['kick'] and k==round(cfg['kick']['time']/dt):
                    context.SetContinuousState(context.get_continuous_state_vector().CopyToVector()+cfg['kick']['delta']);sim.Initialize()
                replay.append(context.get_continuous_state_vector().CopyToVector())
                port.FixValue(context,[float(torque)]);sim.AdvanceTo((k+1)*dt)
            replay.append(context.get_continuous_state_vector().CopyToVector())
            error=float(abs(np.array(replay)-xs).max());maximum=max(maximum,error)
            np.testing.assert_allclose(replay,xs,rtol=1e-8,atol=1e-8,err_msg=f"{cfg['id']}/{mode}")
            count+=1
    comparison=data['solver_comparison']
    assert comparison['pairs']==66 and comparison['repetitions']==7
    assert comparison['reference_solver']=='Drake Clarabel'
    assert comparison['reference_max_plan_diff']<1e-3
    assert comparison['reference_max_relative_cost_error']<1e-6
    for pair in comparison['details']:
        assert pair['sqp_violation']<1e-5 and pair['slsqp_violation']<1e-5
    terminal=next(s for s in data['scenarios'] if s['config']['id']=='terminal')['runs']
    np.testing.assert_allclose(terminal['finite']['states'],terminal['infinite']['states'],atol=1e-5,rtol=1e-5)
    print(f'PASS: {count} runs; complete data, torque limits, costs, timings, settling and Drake replay.')
    print(f'Maximum replay state difference: {maximum:g}; finite terminal Pinf matches infinite LQR.')

if __name__=='__main__':main()
