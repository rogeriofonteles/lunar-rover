"""Paired SQP/SLSQP benchmark: identical state, QP and initial guess per pair.
Run only after simulations finish, to avoid concurrent benchmark CPU load.
"""
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
from pydrake.examples import AcrobotPlant
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent.parent))
from acrobot.linear_mpc import LinearMPC
from acrobot.lqr import wrapped_state_error

REPETITIONS=7
THRESHOLDS=dict(first_torque_nm=.1,convergence_disagreement_fraction=.05,median_time_ratio=3.,median_time_difference_ms=1.)


def main():
    data=json.loads((ROOT/'data.json').read_text());pairs=[]
    # All challenges with 3 Nm, with shared states from the same recorded SQP trajectory.
    # Sampling rule is fixed, not chosen after observing solver differences.
    for s in data['scenarios']:
        cfg=s['config']
        if cfg.get('torque_mode')!='clipped':continue
        kwargs=dict(timestep=cfg['dt'],horizon=round(cfg['mpc_horizon']/cfg['dt']),Q=np.diag(cfg['Q']),R=cfg['R'],torque_limit=3.,terminal_cost=np.diag(cfg['Q']) if cfg.get('mpc_terminal')=='Q' else None)
        a=LinearMPC(AcrobotPlant(),solver='sqp',**kwargs);b=LinearMPC(AcrobotPlant(),solver='slsqp',**kwargs)
        source=s['runs']['mpc_sqp']
        for k in [0,10,30,50,80,110]:
            x=np.array(source['states'][k]);e=wrapped_state_error(x,a.target)
            a.last_result=None;b.last_result=None
            seed,_=a.initial_guess(e)
            # Same initial guess on every call; exclude one warm-up per implementation.
            a.solve(x,initial=seed);b.solve(x,initial=seed)
            timings={'sqp':[],'slsqp':[]};results={}
            for repeat in range(REPETITIONS):
                order=[('sqp',a),('slsqp',b)]
                if repeat%2:order.reverse()
                for name,controller in order:
                    tick=perf_counter();result=controller.solve(x,initial=seed);timings[name].append(1000*(perf_counter()-tick));results[name]=result
            ra,rb=results['sqp'],results['slsqp']
            pairs.append(dict(challenge_id=cfg['challenge_id'],step=k,time=k*cfg['dt'],
                              initial_state=x.tolist(),initial_guess=seed.tolist(),
                              sqp_ms=timings['sqp'],slsqp_ms=timings['slsqp'],
                              first_torque_diff=float(abs(ra.controls[0]-rb.controls[0])),
                              sqp_converged=bool(ra.converged),slsqp_converged=bool(rb.converged),
                              sqp_residual=ra.residual,slsqp_residual=rb.residual,
                              sqp_violation=ra.bound_violation,slsqp_violation=rb.bound_violation,
                              sqp_iterations=ra.iterations,slsqp_iterations=rb.iterations,
                              sqp_status=ra.status,slsqp_status=rb.status))
    ta=float(np.median([np.median(p['sqp_ms']) for p in pairs]));tb=float(np.median([np.median(p['slsqp_ms']) for p in pairs]))
    delta=max(p['first_torque_diff'] for p in pairs)
    disagree=float(np.mean([p['sqp_converged']!=p['slsqp_converged'] for p in pairs]))
    ratio=max(ta,tb)/max(1e-12,min(ta,tb));diff=abs(ta-tb)
    reasons=[]
    if delta>THRESHOLDS['first_torque_nm']:reasons.append('diferença no primeiro torque acima de 0,1 N·m')
    if disagree>=THRESHOLDS['convergence_disagreement_fraction']:reasons.append('diferença de convergência em pelo menos 5% dos problemas')
    if ratio>=THRESHOLDS['median_time_ratio'] and diff>=THRESHOLDS['median_time_difference_ms']:reasons.append('diferença relevante de WallTime')
    conclusion=('O critério foi acionado por '+', '.join(reasons)+'. Isso compara estas implementações e tolerâncias, não garante superioridade geral de um algoritmo.') if reasons else 'Os critérios de torque, convergência e WallTime não justificam duas curvas principais. Os dados de ambos permanecem disponíveis no JSON.'
    output=dict(show_slsqp=bool(reasons),thresholds=THRESHOLDS,repetitions=REPETITIONS,pairs=len(pairs),sqp_median_ms=ta,slsqp_median_ms=tb,max_first_torque_diff=delta,convergence_disagreement=disagree,median_time_ratio=ratio,median_time_difference_ms=diff,conclusion=conclusion,details=pairs)
    (ROOT/'solver_comparison.json').write_text(json.dumps(output,ensure_ascii=False,allow_nan=False))
    print(json.dumps({k:v for k,v in output.items() if k!='details'},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
