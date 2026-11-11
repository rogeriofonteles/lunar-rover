"""Paired SQP/SLSQP benchmark: identical state, QP and initial guess per pair.
Run only after simulations finish, to avoid concurrent benchmark CPU load.
"""
import json
import hashlib
from datetime import datetime, timezone
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.solvers import MathematicalProgram, ClarabelSolver, SolverOptions
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent.parent))
from acrobot.linear_mpc import LinearMPC
from acrobot.lqr import wrapped_state_error

REPETITIONS=7
THRESHOLDS=dict(first_torque_nm=.1,convergence_disagreement_fraction=.05,median_time_ratio=3.,median_time_difference_ms=1.)


def main():
    fixture=ROOT/'solver_benchmark_cases.json'
    cases=json.loads(fixture.read_text())['cases'];pairs=[]
    for case in cases:
            cfg=case['config'];k=case['step']
            kwargs=dict(timestep=cfg['dt'],horizon=round(cfg['mpc_horizon']/cfg['dt']),Q=np.diag(cfg['Q']),R=cfg['R'],torque_limit=3.,terminal_cost=np.diag(cfg['Q']) if cfg.get('mpc_terminal')=='Q' else None)
            a=LinearMPC(AcrobotPlant(),solver='sqp',**kwargs);b=LinearMPC(AcrobotPlant(),solver='slsqp',**kwargs)
            x=np.array(case['initial_state']);e=wrapped_state_error(x,a.target)
            seed=np.array(case['initial_guess'])
            # Same initial guess on every call; exclude one warm-up per implementation.
            a.solve(x,initial=seed);b.solve(x,initial=seed)
            timings={'sqp':[],'slsqp':[]};results={}
            for repeat in range(REPETITIONS):
                order=[('sqp',a),('slsqp',b)]
                if repeat%2:order.reverse()
                for name,controller in order:
                    tick=perf_counter();result=controller.solve(x,initial=seed);timings[name].append(1000*(perf_counter()-tick));results[name]=result
            ra,rb=results['sqp'],results['slsqp']
            # Independent conic QP solve, outside the timing measurements.
            scale=max(1.,float(np.max(abs(seed))));h=a.constraints(e)
            prog=MathematicalProgram();y=prog.NewContinuousVariables(a.N,'y')
            prog.AddQuadraticCost(np.eye(a.N),np.zeros(a.N),y)
            prog.AddLinearConstraint(a.G,np.full(len(h),-np.inf),h/scale,y)
            solver=ClarabelSolver();options=SolverOptions()
            for name in ['tol_gap_abs','tol_gap_rel','tol_feas','reduced_tol_gap_abs','reduced_tol_gap_rel','reduced_tol_feas']:options.SetOption(solver.id(),name,1e-12)
            options.SetOption(solver.id(),'static_regularization_enable',False)
            options.SetOption(solver.id(),'max_iter',500)
            independent=solver.Solve(prog,None,options)
            if not independent.is_success():raise RuntimeError(f"Clarabel reference failed: {cfg['id']}/{k}")
            reference_v=independent.GetSolution(y)*scale
            reference_u=a.D@e+a.L@reference_v
            reference_violation=float(max(0.,np.max(abs(reference_u))-3.))
            if reference_violation>1e-6:raise RuntimeError(f'Clarabel reference infeasible: {reference_violation}')
            reference_cost=float(.5*e@a.P0@e+.5*reference_v@reference_v)
            pairs.append(dict(challenge_id=cfg['challenge_id'],step=k,time=k*cfg['dt'],
                              initial_state=x.tolist(),initial_guess=seed.tolist(),
                              sqp_ms=timings['sqp'],slsqp_ms=timings['slsqp'],
                              first_torque_diff=float(abs(ra.controls[0]-rb.controls[0])),
                              sqp_converged=bool(ra.converged),slsqp_converged=bool(rb.converged),
                              sqp_residual=ra.residual,slsqp_residual=rb.residual,
                              sqp_violation=ra.bound_violation,slsqp_violation=rb.bound_violation,
                              sqp_iterations=ra.iterations,slsqp_iterations=rb.iterations,
                              sqp_status=ra.status,slsqp_status=rb.status,
                              sqp_native_success=ra.native_success,slsqp_native_success=rb.native_success,
                              sqp_native_status=ra.native_status,slsqp_native_status=rb.native_status,
                              sqp_recovery=ra.recovery,slsqp_recovery=rb.recovery,
                              sqp_returned_cost=ra.returned_cost,slsqp_returned_cost=rb.returned_cost,
                              slsqp_repair_max_nm=rb.repair_max_nm,
                              reference_first_torque=float(reference_u[0]),reference_cost=reference_cost,reference_violation=reference_violation,
                              sqp_reference_plan_diff=float(np.max(abs(ra.controls-reference_u))),
                              slsqp_reference_plan_diff=float(np.max(abs(rb.controls-reference_u))),
                              sqp_reference_cost_relative=abs(ra.returned_cost-reference_cost)/max(1.,abs(reference_cost)),
                              slsqp_reference_cost_relative=abs(rb.returned_cost-reference_cost)/max(1.,abs(reference_cost))))
    ta=float(np.median([np.median(p['sqp_ms']) for p in pairs]));tb=float(np.median([np.median(p['slsqp_ms']) for p in pairs]))
    delta=max(p['first_torque_diff'] for p in pairs)
    disagree=float(np.mean([p['sqp_converged']!=p['slsqp_converged'] for p in pairs]))
    ratio=max(ta,tb)/max(1e-12,min(ta,tb));diff=abs(ta-tb)
    reasons=[]
    if delta>THRESHOLDS['first_torque_nm']:reasons.append('diferença no primeiro torque acima de 0,1 N·m')
    if disagree>=THRESHOLDS['convergence_disagreement_fraction']:reasons.append('diferença de convergência em pelo menos 5% dos problemas')
    if ratio>=THRESHOLDS['median_time_ratio'] and diff>=THRESHOLDS['median_time_difference_ms']:reasons.append('diferença relevante de WallTime')
    conclusion=('O critério foi acionado por '+', '.join(reasons)+'. Isso compara estas implementações e tolerâncias, não garante superioridade geral de um algoritmo.') if reasons else 'Os critérios de torque, convergência e WallTime não justificam duas curvas principais. Os dados de ambos permanecem disponíveis no JSON.'
    output=dict(generated_at=datetime.now(timezone.utc).isoformat(),fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),controller_sha256=hashlib.sha256((ROOT.parent/'linear_mpc.py').read_bytes()).hexdigest(),reference_solver='Drake Clarabel',reference_max_plan_diff=max(max(p['sqp_reference_plan_diff'],p['slsqp_reference_plan_diff']) for p in pairs),reference_max_relative_cost_error=max(max(p['sqp_reference_cost_relative'],p['slsqp_reference_cost_relative']) for p in pairs),slsqp_native_failures=sum(not p['slsqp_native_success'] for p in pairs),slsqp_certificate_failures=sum(not p['slsqp_converged'] for p in pairs),slsqp_fallbacks=sum(p['slsqp_recovery']=='initial_guess' for p in pairs),slsqp_repairs=sum(p['slsqp_recovery']=='box_projection' for p in pairs),show_slsqp=bool(reasons),thresholds=THRESHOLDS,repetitions=REPETITIONS,pairs=len(pairs),sqp_median_ms=ta,slsqp_median_ms=tb,max_first_torque_diff=delta,convergence_disagreement=disagree,median_time_ratio=ratio,median_time_difference_ms=diff,conclusion=conclusion,details=pairs)
    (ROOT/'solver_comparison.json').write_text(json.dumps(output,ensure_ascii=False,allow_nan=False))
    print(json.dumps({k:v for k,v in output.items() if k!='details'},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
