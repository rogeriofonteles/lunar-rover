"""Gera execuções, relatório de dados e vídeos MuJoCo para o laboratório."""
import argparse
import json
import os
from pathlib import Path
import sys
from datetime import datetime,timezone

# This generator never opens a viewer; override the container's GLFW default.
os.environ['MUJOCO_GL']='egl'
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent))

import numpy as np
from arm.model import ArmModel
from arm.planning import SCENE_PATH,CollisionWorld
from arm.route_simulation import LABELS,prepare,run_route
from arm.route_video import render_route_video


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only',nargs='+',choices=list(LABELS),default=list(LABELS))
    parser.add_argument('--skip-videos',action='store_true')
    parser.add_argument('--render-only',action='store_true')
    args=parser.parse_args()
    (ROOT/'runs').mkdir(exist_ok=True);(ROOT/'videos').mkdir(exist_ok=True)
    if not args.render_only:
        config,arm,route,nominal=prepare(['red','yellow','green','red'],6,.65,1.2,.6,
            mode='mpc',solver='sqp',horizon=20,ilqr_max_iterations=5)
        world=CollisionWorld(arm)
        reference_clearance=[world.clearance(x[:6]) for x in route.states]
        common=dict(sequence=route.sequence,seed=6,speed_rad_s=.65,acceleration_rad_s2=1.2,dwell_s=.6,
                    timestep=arm.dt,planning=route.planning,paths=[p.tolist() for p in route.paths],
                    arrivals=route.arrivals,reference_min_clearance_m=min(reference_clearance),
                    scene=json.loads(SCENE_PATH.read_text()),nominal_preparation_seconds=nominal.seconds,
                    generated_utc=datetime.now(timezone.utc).isoformat())
        (ROOT/'route.json').write_text(json.dumps(common,ensure_ascii=False,indent=2)+'\n')
        for key in args.only:
            c=dict(config)
            c['mode']='mpc' if key.startswith('mpc_') else key
            c['solver']=key.removeprefix('mpc_') if key.startswith('mpc_') else 'sqp'
            run_route(c,arm,route,nominal,output=ROOT/'runs'/key)
    common=json.loads((ROOT/'route.json').read_text())
    data=dict(common=common,runs={})
    for key in LABELS:
        filename=ROOT/'runs'/f'{key}.json'
        if not filename.exists():
            continue
        summary=json.loads(filename.read_text())
        with np.load(filename.with_suffix('.npz')) as archive:
            arrays={name:archive[name] for name in archive.files}
        if not args.skip_videos and key in args.only:
            c=summary['config'];arm=ArmModel(c['timestep'],c['physics_timestep'],c['torque_limit'])
            summary['video']=render_route_video(arm,arrays,summary,ROOT/'videos'/f'{key}.mp4')
            filename.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        sampled={name:value[::3].tolist() for name,value in arrays.items()}
        sampled['times']=arrays['times'][::3].tolist()
        summary.pop('solver_log',None)
        data['runs'][key]=dict(summary=summary,series=sampled)
    raw=json.dumps(data,ensure_ascii=False,separators=(',',':'))
    (ROOT/'data.json').write_text(raw+'\n')
    (ROOT/'data.js').write_text('window.ROUTE_DATA = '+raw+';\n')
    print(f"Relatório: {ROOT/'index.html'}",flush=True)


if __name__=='__main__':
    main()
