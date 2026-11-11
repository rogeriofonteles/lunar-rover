"""Acrobot linear receding-horizon control: Riccati, exact-Hessian SQP, SLSQP.

SQP is specialized to a convex quadratic problem with linear torque bounds;
it is NOT a nonlinear MPC solver. Riccati completion of squares yields a
well-conditioned identity Hessian instead of condensing unstable open-loop A.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import minimize

from .lqr import _linear_model, _riccati_step, _positive_integer, _torque_limit, _state_cost, wrapped_state_error


@dataclass
class LinearMPCResult:
    states: np.ndarray
    controls: np.ndarray
    costs: list
    converged: bool
    iterations: int
    status: str
    initialization: str
    residual: float
    bound_violation: float
    unconstrained: bool


class LinearMPC:
    def __init__(self, plant, timestep=.05, horizon=40, Q=None, R=.1,
                 torque_limit=3., solver='riccati', terminal_cost=None,
                 max_iterations=300, tolerance=1e-6):
        if solver not in ('riccati','sqp','slsqp'):
            raise ValueError('solver must be riccati, sqp, or slsqp')
        self.N=_positive_integer(horizon,'horizon')
        self.max_iterations=_positive_integer(max_iterations,'max_iterations')
        if not np.isfinite(tolerance) or tolerance<=0:raise ValueError('tolerance must be positive')
        self.tolerance=tolerance;self.solver=solver
        self.bound=np.inf if torque_limit is None else _torque_limit(torque_limit)
        self.A,self.B,self.Q=_linear_model(plant,timestep,Q,R,None)
        self.R=R;self.target=np.array([math.pi,0.,0.,0.]);self.last_result=None
        value=self.Q.copy()
        if terminal_cost is None:
            for _ in range(10000):
                _,new=_riccati_step(self.A,self.B,self.Q,R,value)
                if np.max(abs(new-value))<1e-11*max(1.,np.max(abs(value))):
                    value=new;break
                value=new
            else:raise RuntimeError('DARE terminal cost did not converge')
        else:value=_state_cost(terminal_cost,'terminal_cost')
        self.Qf=value.copy();self.gains=np.empty((self.N,1,4));self.sqrtW=np.empty(self.N)
        for k in range(self.N-1,-1,-1):
            self.sqrtW[k]=np.sqrt(R+(self.B.T@value@self.B).item())
            self.gains[k],value=_riccati_step(self.A,self.B,self.Q,R,value)
        self.P0=value
        # u = D e0 + L v; J = .5 e0' P0 e0 + .5 v'v.
        self.D=np.zeros((self.N,4));self.L=np.zeros((self.N,self.N))
        self.T=np.empty((self.N+1,4,4));self.S=np.zeros((self.N+1,4,self.N));self.T[0]=np.eye(4)
        for k in range(self.N):
            self.D[k]=-self.gains[k]@self.T[k]
            self.L[k]=-self.gains[k]@self.S[k];self.L[k,k]+=1/self.sqrtW[k]
            self.T[k+1]=self.A@self.T[k]+self.B@self.D[k:k+1]
            self.S[k+1]=self.A@self.S[k]+self.B@self.L[k:k+1]
        self.row_norm=np.linalg.norm(self.L,axis=1)
        self.G=np.vstack((self.L,-self.L))/np.tile(self.row_norm,2)[:,None]

    def constraints(self,error):
        d=self.D@error
        return np.concatenate((self.bound-d,self.bound+d))/np.tile(self.row_norm,2)

    def from_controls(self,error,controls):
        x=error.copy();v=np.empty(self.N)
        for k,u in enumerate(controls):
            v[k]=(u+(self.gains[k]@x).item())*self.sqrtW[k]
            x=self.A@x+self.B[:,0]*u
        return v

    def initial_guess(self,error):
        x=error.copy();controls=[]
        for K in self.gains:
            u=float(np.clip(-(K@x).item(),-self.bound,self.bound))
            controls.append(u);x=self.A@x+self.B[:,0]*u
        v=self.from_controls(error,controls);label='clipped_lqr'
        if self.last_result is not None:
            old=self.last_result.controls
            shifted=np.clip(np.r_[old[1:],old[-1]],-self.bound,self.bound)
            warm=self.from_controls(error,shifted)
            if warm@warm < v@v:v=warm;label='provided'
        return v,label

    def optimality(self,v,h):
        """Relative stationarity (scaled by max(1, ||v||inf)) and primal feasibility."""
        if not np.isfinite(self.bound):return float(np.max(abs(v)))
        slack=h-self.G@v;active=np.flatnonzero(slack<1e-6)
        if len(active):
            # Nonnegative multipliers avoid interpreting negative multipliers as KKT.
            from scipy.optimize import nnls
            multipliers,_=nnls(self.G[active].T,-v,maxiter=max(100,10*len(active)))
            stationarity=v+self.G[active].T@multipliers
        else:stationarity=v
        return float(max(np.max(abs(stationarity))/max(1.,np.max(abs(v))),max(0.,-np.min(slack))))

    def solve_sqp(self,initial,h):
        """Feasible active-set SQP for identity Hessian and linear inequalities.

        Each subproblem has the exact Hessian. QR projects the Newton direction
        onto the current active face; blocking constraints restrict its step.
        Iterations count face changes/Newton steps, not nonlinear SQP passes.
        """
        v=initial.copy();G=self.G;active=[];history=[.5*v@v]
        for i in np.flatnonzero(abs(h-G@v)<1e-8):
            candidate=active+[int(i)]
            if np.linalg.matrix_rank(G[candidate],tol=1e-9)==len(candidate):active=candidate
        status='maximum iterations reached'
        for iteration in range(1,self.max_iterations+1):
            if active:
                Q,R=np.linalg.qr(G[active].T,mode='reduced')
                p=-v+Q@(Q.T@v)
                lam=np.linalg.solve(R,-Q.T@v)
            else:p=-v;lam=np.array([])
            if np.max(abs(p))<=self.tolerance*.1:
                if not active or np.min(lam)>=-self.tolerance*.1:
                    status='KKT tolerance';break
                active.pop(int(np.argmin(lam)));continue
            gp=G@p;slack=h-G@v
            ratios=np.full(len(h),np.inf)
            candidates=gp>1e-10
            if active:candidates[active]=False
            ratios[candidates]=np.maximum(0.,slack[candidates])/gp[candidates]
            blocking=int(np.argmin(ratios));alpha=min(1.,float(ratios[blocking]))
            v+=alpha*p;history.append(float(.5*v@v))
            if ratios[blocking]<1.-1e-10:
                candidate=active+[blocking]
                if np.linalg.matrix_rank(G[candidate],tol=1e-9)==len(candidate):active=candidate
                else:status='dependent active constraints';break
        return v,iteration,status,history

    def solve(self,measured_state,initial=None):
        error=wrapped_state_error(np.asarray(measured_state,dtype=float),self.target)
        if error.shape!=(4,) or not np.isfinite(error).all():raise ValueError('state must be finite and length 4')
        if initial is None:seed,label=self.initial_guess(error)
        else:seed=np.asarray(initial,dtype=float).copy();label='paired_initial_guess'
        constant=float(.5*error@self.P0@error)
        if self.solver=='riccati':
            v=np.zeros(self.N);history=[constant];status='Riccati analytical solution';iteration=0
            residual=0.;converged=True
        else:
            h=self.constraints(error)
            if not np.isfinite(self.bound):
                # Without bounds the same unconstrained LQ solution is exact.
                v=np.zeros(self.N);iteration=0;status='unconstrained analytical solution';history=[constant];residual=0.;converged=True
            else:
                if np.max(self.G@seed-h)>1e-5:raise ValueError('Initial guess must be feasible')
                if self.solver=='sqp':v,iteration,status,hist=self.solve_sqp(seed,h)
                else:
                    hist=[float(.5*seed@seed)]
                    res=minimize(lambda z:.5*z@z,seed,jac=lambda z:z,
                                 constraints={'type':'ineq','fun':lambda z:h-self.G@z,'jac':lambda z:-self.G},
                                 method='SLSQP',callback=lambda z:hist.append(float(.5*z@z)),
                                 options={'maxiter':self.max_iterations,'ftol':1e-11})
                    v=res.x;iteration=int(res.nit);status=str(res.message)
                # Always return a finite feasible plan; never silently call it converged.
                if not np.isfinite(v).all() or np.max(self.G@v-h)>1e-5:
                    v=seed;status+='; feasible initial-guess fallback'
                residual=self.optimality(v,h);converged=residual<=self.tolerance
                history=[constant+float(c) for c in hist]
                if converged:status='KKT tolerance'
                elif status=='KKT tolerance':status='KKT check tolerance not achieved'
        controls=self.D@error+self.L@v
        states=self.target+np.einsum('kij,j->ki',self.T,error)+np.einsum('kij,j->ki',self.S,v)
        violation=0. if not np.isfinite(self.bound) else float(max(0.,np.max(abs(controls))-self.bound))
        result=LinearMPCResult(states,controls,history,converged,iteration,status,label,residual,violation,self.solver=='riccati')
        self.last_result=result
        return result

    def command(self,measured_state):
        result=self.solve(measured_state)
        # For Riccati only, clipping is applied AFTER the unconstrained solve.
        return float(np.clip(result.controls[0],-self.bound,self.bound))
