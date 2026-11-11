"""Trajectory tracking with the existing Riccati, iLQR, and box-QP solvers."""
from dataclasses import dataclass
import time

import mujoco
import numpy as np
from scipy.linalg import block_diag

from .ilqr import iterative_lqr
from .lqr import cost_matrix, discretize, infinite_horizon_lqr
from .mpc import BoxQPSolver
from .problem import ControlProblem


@dataclass
class Nominal:
    states: np.ndarray
    controls: np.ndarray
    A: np.ndarray
    B: np.ndarray
    defect: np.ndarray
    K: np.ndarray
    P: np.ndarray
    Q: np.ndarray
    R: np.ndarray
    seconds: float


def prepare_nominal(arm, route, config):
    start=time.perf_counter()
    Q,R=cost_matrix(config["Q"],12,"Q"),cost_matrix(config["R"],6,"R",True)
    us,As,Bs,ds,Ks,Ps=[],[],[],[],[],[]
    # Include a stationary terminal sample for padded receding horizons.
    for i,x in enumerate(route.states):
        arm._set(x,np.zeros(6))
        arm.data.qacc[:]=route.accelerations[i]
        mujoco.mj_inverse(arm.model,arm.data)
        u=arm.data.qfrc_inverse.copy()
        if np.any(np.abs(u)>arm.limit):
            raise ValueError("Referência exige torque inviável; reduza velocidade/aceleração")
        Ac,Bc=arm.linearize(x,u)
        A,B=discretize(Ac,Bc,arm.dt,config["discretization"])
        K,P=infinite_horizon_lqr(A,B,Q,R)
        ds.append(arm.step(x,u)-route.states[min(i+1,len(route.states)-1)])
        us.append(u);As.append(A);Bs.append(B);Ks.append(K);Ps.append(P)
    return Nominal(route.states,np.asarray(us),np.asarray(As),np.asarray(Bs),
                   np.asarray(ds),np.asarray(Ks),np.asarray(Ps),Q,R,time.perf_counter()-start)


def riccati_tracking(A,B,defect,Q,R,P):
    """Finite, time-varying affine LQR: delta_u = -K e - k."""
    gains=np.empty((len(A),6,12)); offsets=np.empty((len(A),6))
    v=np.zeros(12)
    for t in range(len(A)-1,-1,-1):
        H=R+B[t].T@P@B[t]
        cross=B[t].T@P@A[t]
        rhs=B[t].T@(P@defect[t]+v)
        K=np.linalg.solve(H,cross); k=np.linalg.solve(H,rhs)
        v=A[t].T@(P@defect[t]+v)-cross.T@k
        P=Q+A[t].T@P@A[t]-cross.T@K
        P=(P+P.T)/2
        gains[t],offsets[t]=K,k
    return gains,offsets


class TrackingProblem(ControlProblem):
    """Reuse nonlinear iLQR with a time-indexed reference and inverse dynamics."""
    def __init__(self,arm,nominal,index,horizon):
        self.model=arm;self.horizon=horizon
        ix=np.minimum(np.arange(index,index+horizon+1),len(nominal.states)-1)
        self.reference_states=nominal.states[ix]
        self.reference_controls=nominal.controls[ix[:-1]]
        self.target=self.reference_states[-1]
        self.u_ref=self.reference_controls[-1]
        self.Q,self.R,self.Qf=nominal.Q,nominal.R,nominal.P[ix[-1]]
        self.K=nominal.K[ix[0]]


def condense(A,B,defect):
    """e_{t+1}=A_t e_t+B_t du_t+d_t -> E=T e0+S DU+D."""
    N=len(A);S=np.zeros((N*12,N*6));T=np.empty((N*12,12));D=np.empty(N*12)
    st=np.zeros((12,N*6));tt=np.eye(12);dd=np.zeros(12)
    for k in range(N):
        st=A[k]@st;st[:,k*6:(k+1)*6]=B[k]
        tt=A[k]@tt;dd=A[k]@dd+defect[k]
        S[k*12:(k+1)*12]=st;T[k*12:(k+1)*12]=tt;D[k*12:(k+1)*12]=dd
    return S,T,D


class TrajectoryController:
    def __init__(self,arm,nominal,config):
        self.arm,self.n,self.c=arm,nominal,config
        self.mode=config["mode"]; self.N=config["horizon"]
        self.previous=None;self.plan=None;self.last_info={}
        start=time.perf_counter()
        if self.mode=="finite":
            self.gains,self.offsets=riccati_tracking(nominal.A[:-1],nominal.B[:-1],nominal.defect[:-1],
                                                   nominal.Q,nominal.R,nominal.P[-1])
        elif self.mode=="ilqr":
            problem=TrackingProblem(arm,nominal,0,len(nominal.states)-1)
            # Over a full route, pure inverse-dynamics rollout can drift far
            # from the path. Let iLQR compare it with its feedback tracking seed.
            self.plan=iterative_lqr(problem,nominal.states[0],**self.ilqr_options())
        self.setup_seconds=time.perf_counter()-start

    def ilqr_options(self):
        return {key.removeprefix("ilqr_"):value for key,value in self.c.items() if key.startswith("ilqr_")}

    def command(self,x,index):
        n,c=self.n,self.c
        error=x-n.states[index]
        self.last_info={"converged":True,"iterations":0,"residual":0.}
        if self.mode=="infinite":
            u=n.controls[index]-n.K[index]@error
        elif self.mode=="finite":
            u=n.controls[index]-self.gains[index]@error-self.offsets[index]
        elif self.mode=="ilqr":
            p=self.plan
            self.last_info={"converged":bool(p.converged),"iterations":p.iterations,"status":p.status}
            u=p.controls[index]-p.gains[index]@(x-p.states[index])
        else:
            ix=np.minimum(np.arange(index,index+self.N+1),len(n.states)-1)
            A,B,d=n.A[ix[:-1]],n.B[ix[:-1]],n.defect[ix[:-1]]
            refs=n.controls[ix[:-1]]
            if self.mode=="receding":
                K,k=riccati_tracking(A,B,d,n.Q,n.R,n.P[ix[-1]])
                u=refs[0]-K[0]@error-k[0]
            elif self.mode=="mpc":
                S,T,D=condense(A,B,d)
                Qbar=block_diag(*([n.Q]*(self.N-1)+[n.P[ix[-1]]]))
                H=S.T@Qbar@S+np.kron(np.eye(self.N),n.R)
                H=(H+H.T)/2
                gradient=S.T@Qbar@(T@error+D)
                lower=(-self.arm.limit-refs).ravel();upper=(self.arm.limit-refs).ravel()
                seed=None if self.previous is None else (np.vstack((self.previous[1:],self.previous[-1]))-refs).ravel()
                result=BoxQPSolver(H,lower,upper,c["solver"],c["mpc_max_iterations"],c["mpc_tolerance"]).solve(gradient,seed)
                self.previous=refs+result.x.reshape(self.N,6)
                u=self.previous[0]
                self.last_info={"converged":bool(result.converged),"iterations":result.iterations,
                                "residual":result.residual,"status":result.status}
            elif self.mode=="ilqr_receding":
                problem=TrackingProblem(self.arm,n,index,self.N)
                seed=refs if self.previous is None else np.vstack((self.previous[1:],refs[-1]))
                result=iterative_lqr(problem,x,initial_controls=seed,**self.ilqr_options())
                self.previous=result.controls;u=result.controls[0]
                self.last_info={"converged":bool(result.converged),"iterations":result.iterations,"status":result.status}
            else:
                raise ValueError(f"Unknown tracking mode: {self.mode}")
        return np.clip(u,-self.arm.limit,self.arm.limit)
