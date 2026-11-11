"""Planeje uma rota entre botões e acompanhe-a no MuJoCo."""
import argparse
import json
import os
from pathlib import Path
import sys

if any(arg=='--video' or arg.startswith('--video=') for arg in sys.argv):
    os.environ['MUJOCO_GL']='egl'

from arm.buttons import BUTTON_NAMES
from arm.mpc import MPC_SOLVERS
from arm.simulate import MODES
from arm.route_simulation import prepare, run_route


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence',nargs='+',choices=BUTTON_NAMES,default=['red','yellow','green','red'])
    parser.add_argument('--mode',choices=MODES,default='mpc')
    parser.add_argument('--solver',choices=MPC_SOLVERS,default='sqp')
    parser.add_argument('--horizon',type=int,default=20)
    parser.add_argument('--discretization',choices=['euler','zoh'],default='zoh')
    parser.add_argument('--seed',type=int,default=6)
    parser.add_argument('--speed',type=float,default=.65,help='Velocidade máxima da referência, rad/s')
    parser.add_argument('--acceleration',type=float,default=1.2,help='Aceleração máxima, rad/s²')
    parser.add_argument('--dwell',type=float,default=.6,help='Pausa em cada botão, segundos')
    parser.add_argument('--ilqr-max-iterations',type=int,default=5)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--wait',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('output/button-route/run'))
    parser.add_argument('--video',type=Path)
    args=parser.parse_args()
    try:
        config,arm,route,nominal=prepare(args.sequence,args.seed,args.speed,args.acceleration,args.dwell,
            mode=args.mode,solver=args.solver,horizon=args.horizon,discretization=args.discretization,
            ilqr_max_iterations=args.ilqr_max_iterations)
        arrays,summary=run_route(config,arm,route,nominal,args.headless,args.wait,args.output)
        if args.video:
            from arm.route_video import render_route_video
            render_route_video(arm,arrays,summary,args.video)
        print(json.dumps({k:v for k,v in summary.items() if k not in ('solver_log','config')},indent=2,ensure_ascii=False))
    except (ValueError,RuntimeError) as error:
        parser.error(str(error))


if __name__=='__main__':
    main()
