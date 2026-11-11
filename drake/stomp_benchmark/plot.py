"""Plot all trials, including failures, from benchmark.py output."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('results',type=Path)
    args=ap.parse_args()
    with (args.results/'trials.csv').open() as f:
        rows=list(csv.DictReader(f))
    config=json.loads((args.results/'metadata.json').read_text())['arguments']
    targets=list(dict.fromkeys(r['target'] for r in rows))
    fig,ax=plt.subplots(figsize=(10,5),layout='constrained')
    for scene,offset,color in [('open',-.16,'#247ba0'),('pole',.16,'#c45a18')]:
        for i,target in enumerate(targets):
            group=[r for r in rows if r['scene']==scene and r['target']==target]
            if not group: continue
            values=np.array([float(r['planning_s']) for r in group])
            xs=i+offset+np.linspace(-.065,.065,len(values))
            for x,y,r in zip(xs,values,group):
                ax.scatter(x,y,color=color,s=24,
                    marker='o' if r['success']=='True' else 'x',alpha=.7)
            ax.plot([i+offset-.09,i+offset+.09],[np.median(values)]*2,color=color,lw=3,
                label=scene if i==0 else None)
    ax.set_xticks(range(len(targets)),[t.replace('_','\n') for t in targets])
    ax.set_yscale('log'); ax.set_ylabel('STOMP planning time (seconds, log scale)')
    ax.set_title('UR5 + Drake + standalone STOMP — no ROS nodes\n'
                 f"{config['waypoints']} waypoints · {config['rollouts']} rollouts · "
                 f"{config['seeds']} seeds per pose and scene")
    ax.grid(axis='y',alpha=.2,which='both'); ax.legend(title='Scene')
    ax.text(.01,.98,'Dots: successful trials; crosses: failures; bars: medians',
            transform=ax.transAxes,va='top',fontsize=9)
    fig.savefig(args.results/'planning_times.png',dpi=180)

if __name__=='__main__':
    main()
