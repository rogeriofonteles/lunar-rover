"""Gera casos, execuções e vídeos reais do laboratório MuJoCo."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import sys
import math

os.environ['MUJOCO_GL']='egl'
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent))

import numpy as np
from arm.route_cases import CASES,case_settings,make_plant
from arm.planning import SCENE_PATH,CollisionWorld
from arm.route_simulation import LABELS,prepare,run_route
from arm.route_video import render_route_video


def case_directory(key):
    # Keep existing nominal links and videos working.
    return ROOT if key=='nominal' else ROOT/'cases'/key


def json_safe(value):
    """A failed geometric bound may yield -inf; public JSON uses a missing value."""
    if isinstance(value,float) and not math.isfinite(value):
        return None
    if isinstance(value,dict):
        return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value,list):
        return [json_safe(v) for v in value]
    return value


def assemble(key):
    folder=case_directory(key)
    common=json.loads((folder/'route.json').read_text())
    common.setdefault('experiment',case_settings(key))
    data=dict(common=common,runs={})
    for method in LABELS:
        filename=folder/'runs'/f'{method}.json'
        if not filename.exists():
            continue
        summary=json.loads(filename.read_text())
        with np.load(filename.with_suffix('.npz')) as archive:
            arrays={name:archive[name] for name in archive.files}
        errors=np.linalg.norm(arrays['actual_tip']-arrays['planned_tip'],axis=1)*1000
        utilization=np.max(np.abs(arrays['controls'])/summary['config']['torque_limit'],axis=1)*100
        summary.update(case=key,experiment=common['experiment'],max_tip_error_m=float(errors.max()/1000),
            saturated_step_percent=float(np.mean(utilization>=99.9)*100))
        summary.pop('solver_log',None)
        summary.setdefault('recovery_seconds',None)
        # Keep every sample in comparison curves so brief peaks are not hidden.
        comparison=dict(times=arrays['times'].tolist(),tip_error_mm=errors.tolist(),
            joint_error_deg=np.rad2deg(arrays['states'][:,:6]-arrays['reference'][:,:6]).tolist(),
            utilization_percent=utilization.tolist(),command_ms=(arrays['command_seconds']*1000).tolist())
        ix=np.unique(np.r_[np.arange(0,len(arrays['times']),3),len(arrays['times'])-1])
        sampled={name:value[ix[ix<len(value)]].tolist() for name,value in arrays.items()}
        video=folder/'videos'/f'{method}.mp4'
        # Existence alone isn't enough: regeneration drops stale video metadata.
        has_video=bool(summary.get('video')) and video.exists()
        prefix='' if key=='nominal' else f'cases/{key}/'
        data['runs'][method]=dict(summary=summary,series=sampled,comparison=comparison,
            assets=dict(video=f'{prefix}videos/{method}.mp4' if has_video else None,
                        poster=f'{prefix}videos/{method}-start.png' if has_video else None,
                        run=f'{prefix}runs/{method}.npz',summary=f'{prefix}runs/{method}.json'))
    if not data['runs']:
        raise ValueError(f'Nenhuma execução salva para {key}')
    raw=json.dumps(json_safe(data),ensure_ascii=False,separators=(',',':'),allow_nan=False)
    (folder/'data.json').write_text(raw+'\n')
    script=f'window.ROUTE_CASE_DATA = window.ROUTE_CASE_DATA || {{}};\nwindow.ROUTE_CASE_DATA[{json.dumps(key)}] = '+raw+';\n'
    if key=='nominal':
        script+='window.ROUTE_DATA = window.ROUTE_CASE_DATA.nominal;\n'
    (folder/'data.js').write_text(script)
    return data


def write_catalog():
    catalog={}
    for key in CASES:
        folder=case_directory(key)
        if (folder/'data.js').exists():
            prefix='' if key=='nominal' else f'cases/{key}/'
            catalog[key]=dict(**case_settings(key),data=f'{prefix}data.js?v=2',folder=prefix)
    (ROOT/'cases.js').write_text('window.ROUTE_CASES = '+json.dumps(catalog,ensure_ascii=False,indent=2)+';\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',nargs='+',choices=list(CASES),default=list(CASES))
    parser.add_argument('--only',nargs='+',choices=list(LABELS),default=list(LABELS))
    parser.add_argument('--skip-videos',action='store_true')
    parser.add_argument('--render-only',action='store_true')
    parser.add_argument('--assemble-only',action='store_true',help='Atualiza gráficos usando execuções salvas')
    args=parser.parse_args()
    if args.render_only and args.assemble_only:
        parser.error('Escolha --render-only ou --assemble-only')
    for key in args.cases:
        folder=case_directory(key)
        (folder/'runs').mkdir(parents=True,exist_ok=True);(folder/'videos').mkdir(exist_ok=True)
        if not args.render_only and not args.assemble_only:
            config,arm,route,nominal=prepare(['red','yellow','green','red'],6,case=key,
                mode='mpc',solver='sqp',horizon=20,ilqr_max_iterations=5)
            world=CollisionWorld(arm)
            common=dict(experiment=config['experiment'],sequence=route.sequence,seed=6,
                speed_rad_s=config['experiment']['speed'],acceleration_rad_s2=config['experiment']['acceleration'],
                dwell_s=config['experiment']['dwell'],timestep=arm.dt,planning=route.planning,
                paths=[p.tolist() for p in route.paths],arrivals=route.arrivals,
                reference_min_clearance_m=min(world.clearance(x[:6]) for x in route.states),
                scene=json.loads(SCENE_PATH.read_text()),nominal_preparation_seconds=nominal.seconds,
                generated_utc=datetime.now(timezone.utc).isoformat())
            (folder/'route.json').write_text(json.dumps(common,ensure_ascii=False,indent=2)+'\n')
            for method in args.only:
                print(f'CASO {key} / {method}',flush=True)
                c=dict(config,mode='mpc' if method.startswith('mpc_') else method,
                    solver=method.removeprefix('mpc_') if method.startswith('mpc_') else 'sqp')
                run_route(c,arm,route,nominal,output=folder/'runs'/method)
        if not args.skip_videos and not args.assemble_only:
            for method in args.only:
                filename=folder/'runs'/f'{method}.json'
                summary=json.loads(filename.read_text())
                with np.load(filename.with_suffix('.npz')) as archive:
                    arrays={name:archive[name] for name in archive.files}
                arm=make_plant(summary['config'])
                summary['video']=render_route_video(arm,arrays,summary,folder/'videos'/f'{method}.mp4')
                filename.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        assemble(key)
        write_catalog()
    print(f"Relatório: {ROOT/'index.html'}",flush=True)


if __name__=='__main__':
    main()
