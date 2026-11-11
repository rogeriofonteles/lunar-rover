"""Render comparison MP4s from recorded Drake states, on the host.
Requires Pillow, numpy, and ffmpeg. No physics or controller is run here.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
W,H,FPS = 1280,720,30
BG='#132a26'; INK='#e7eee3'; MUTED='#a6bdae'; GRID='#2b4338'
MODES=['finite','infinite','ilqr','mpc']
NAMES=['LQR finito','LQR infinito','iLQR · plano único','MPC não linear']
COLORS=['#e9b16d','#80bfdd','#c8a3e1','#7dcdb0']
FONT=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
BOLD=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')


def font(size,bold=False): return ImageFont.truetype(str(BOLD if bold else FONT),size)
FONTS={n:font(n) for n in [11,12,13,14,16,18,20,24,28]}

def text(d,xy,value,size=14,fill=INK): d.text(xy,str(value),font=FONTS[size],fill=fill)

def state_at(run,t):
    q=min(len(run['states'])-1,max(0,t/run['cfg']['dt']));i=int(q);j=min(i+1,len(run['states'])-1);a=q-i
    right=np.array(run['states'][j]);kick=run['cfg']['kick']
    if kick and i<j and abs(j*run['cfg']['dt']-kick['time'])<1e-8 and t<kick['time']:right-=kick['delta']
    return np.array(run['states'][i])*(1-a)+right*a


def points(x,p,ox,oy,scale):
    elbow=[p['l1']*math.sin(x[0]),-p['l1']*math.cos(x[0])]
    tip=[elbow[0]+p['l2']*math.sin(x[0]+x[1]),elbow[1]-p['l2']*math.cos(x[0]+x[1])]
    return [(ox,oy),(ox+elbow[0]*scale,oy-elbow[1]*scale),(ox+tip[0]*scale,oy-tip[1]*scale)]


def frame(s,t,modes=MODES,names=NAMES,colors=COLORS):
    im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im);cfg=s['config']
    text(d,(30,18),'ACROBOT / CONTROLE ÓTIMO',12,MUTED)
    text(d,(30,44),cfg['title'],28)
    limit_text="sem clipping" if cfg["limit"] is None else f"±{cfg['limit']:g} N·m"
    text(d,(30,84),f"Planta Drake não linear  ·  Δt = {cfg['dt']:.2f} s  ·  {limit_text}  ·  R = {cfg['R']:g}",14,MUTED)
    text(d,(1040,35),f't = {t:04.2f} s',24)
    for j,(mode,name,color) in enumerate(zip(modes,names,colors)):
        r=s['runs'][mode];x=state_at(r,t);m=r['metrics'];width=(W-20)/len(modes)-10;left=20+j*(width+10);ox=left+width/2;oy=333;scale=43
        d.rounded_rectangle((left,119,left+width,641),radius=7,outline=GRID,width=1)
        d.ellipse((left+14,139,left+22,147),fill=color);text(d,(left+30,131),name,18,color)
        for k in range(-3,4):
            d.line((left+12,oy+k*scale,left+width-12,oy+k*scale),fill=GRID,width=1)
            d.line((ox+k*scale,184,ox+k*scale,478),fill=GRID,width=1)
        for yy in range(int(oy-3*scale),int(oy),10):d.line((ox,yy,ox,yy+4),fill='#648770',width=3)
        end=min(len(r['states'])-1,int(t/cfg['dt']))
        trail=[points(q,r['parameters'],ox,oy,scale)[2] for q in r['states'][max(0,end-30):end+1]]
        if len(trail)>1:d.line(trail,fill='#577365',width=2)
        ps=points(x,r['parameters'],ox,oy,scale)
        d.line(ps,fill=color,width=10);d.line(ps,fill=BG,width=2)
        for k,(px,py) in enumerate(ps):
            rr=8 if k==1 else 5;d.ellipse((px-rr,py-rr,px+rr,py+rr),fill=color if k==1 else INK,outline=BG,width=2)
        d.line((ox-15,oy+10,ox+15,oy+10),fill=MUTED,width=3)
        k=min(len(r['torque'])-1,int(t/cfg['dt']));text(d,(left+13,164),f"u = {r['torque'][k]:+.2f} N·m",14,MUTED)
        e=(x[:2]-[math.pi,0]+math.pi)%(2*math.pi)-math.pi
        text(d,(left+13,480),f"Erro máx. θ: {max(abs(e)):.3f} rad",13)
        text(d,(left+13,501),f"Velocidade máx.: {max(abs(x[2:])):.3f} rad/s",13)
        text(d,(left+13,532),'ERRO ANGULAR (ombro / cotovelo)',11,MUTED)
        errors=(np.array(r['states'])[:,:2]-[math.pi,0]+math.pi)%(2*math.pi)-math.pi
        lim=max(.05,float(abs(errors).max()));plot_y=580;plot_h=25
        d.line((left+13,plot_y,left+width-13,plot_y),fill=GRID,width=1)
        for q,col in [(0,color),(1,'#c1cbbb')]:
            pp=[(left+13+k/(len(errors)-1)*(width-26),plot_y-errors[k,q]/lim*plot_h) for k in range(end+1)]
            if len(pp)>1:d.line(pp,fill=col,width=2)
        settle='não atingida' if m['settling_s'] is None else f"{m['settling_s']:.2f} s"
        text(d,(left+13,617),f'Estabilização: {settle}',12,MUTED)
    d.line((30,664,1250,664),fill=GRID,width=3);d.line((30,664,30+1220*t/cfg['duration'],664),fill='#7dcdb0',width=3)
    kick=cfg['kick'];msg='Geometria 2D dos estados registrados · tempo simulado · não comprova tempo real'
    if kick and t>=kick['time']:msg=f"Perturbação aplicada em t = {kick['time']:g} s  ·  Δω = {kick['delta'][2:]} rad/s"
    fast=any(max(max(abs(q[2]),abs(q[2]+q[3])) for q in s['runs'][m]['states'])*cfg['dt']>math.pi/2 for m in modes)
    if fast:msg='Movimento muito rápido: replay sujeito a aliasing entre amostras de 50 ms · consulte os gráficos'
    text(d,(30,683),msg,13,MUTED)
    return im


def main():
    p=argparse.ArgumentParser();p.add_argument('--scenarios',nargs='*');p.add_argument('--force',action='store_true');p.add_argument('--family',choices=['controllers','solvers','both'],default='both');a=p.parse_args()
    data=json.loads((ROOT/'data.json').read_text());out=ROOT/'videos';out.mkdir(exist_ok=True)
    jobs=[]
    include=data.get('solver_comparison',{}).get('show_slsqp',False)
    for s in data['scenarios']:
        name=s['config']['id']
        if a.scenarios and name not in a.scenarios:continue
        if a.family in ('controllers','both'):jobs.append((s,name,MODES,NAMES,COLORS))
        if a.family in ('solvers','both'):
            modes=['mpc_lqr','mpc_sqp']+(['mpc_slsqp'] if include else [])+['mpc']
            names=['MPC · LQR/Riccati','MPC · SQP']+(['MPC · SLSQP'] if include else [])+['MPC não linear']
            colors=[COLORS[1],COLORS[0]]+([COLORS[2]] if include else [])+[COLORS[3]]
            jobs.append((s,name+'_solvers',modes,names,colors))
    for s,name,modes,names,colors in jobs:
        target=out/f'{name}.mp4'
        if any(m not in s['runs'] for m in modes):raise ValueError(f'{name}: missing runs')
        if target.exists() and not a.force:continue
        print('Rendering',name,flush=True)
        proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{W}x{H}','-framerate',str(FPS),'-i','pipe:0','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(target)],stdin=subprocess.PIPE)
        try:
            count=round(s['config']['duration']*FPS)
            for k in range(count+1):
                im=frame(s,k/FPS,modes,names,colors);proc.stdin.write(im.tobytes())
                if k==count//2:im.save(out/f'{name}-preview.jpg',quality=90)
            proc.stdin.close()
            if proc.wait():raise RuntimeError('ffmpeg failed')
        finally:
            if proc.poll() is None:proc.kill();proc.wait()
        print(target,flush=True)

if __name__=='__main__':main()
