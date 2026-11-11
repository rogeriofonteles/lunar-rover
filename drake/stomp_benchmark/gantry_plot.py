"""Plot gantry benchmark timing and success rates without hiding timeouts."""
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
    ap.add_argument('results',type=Path);args=ap.parse_args()
    with (args.results/'trials.csv').open() as f: rows=list(csv.DictReader(f))
    meta=json.loads((args.results/'metadata.json').read_text())
    config=meta['arguments'];targets=config['targets']
    modes=[('locked6','6 joints: gantry fixed','#247ba0'),
           ('free9','9 joints: same endpoints','#ca6a21'),
           ('moving9','9 joints: moving gantry goals','#30855c')]
    fig,axes=plt.subplots(1,2,figsize=(13,5),sharey=True,layout='constrained')
    for ax,scene in zip(axes,['open','bar']):
        for k,(mode,label,color) in enumerate(modes):
            for i,target in enumerate(targets):
                group=[r for r in rows if (r['scene'],r['mode'],r['target'])==(scene,mode,target)]
                if not group: continue
                valid=[r for r in group if r['endpoints_valid']=='True']
                vals=[float(r['planning_s']) for r in valid]
                offset=(k-1)*.23
                for x,r in zip(i+offset+np.linspace(-.055,.055,len(valid)),valid):
                    ax.scatter(x,float(r['planning_s']),color=color,s=18,alpha=.75,
                               marker='o' if r['success']=='True' else 'x')
                if vals:
                    ax.plot([i+offset-.08,i+offset+.08],[np.median(vals)]*2,color=color,lw=3,
                            label=label if i==0 else None)
        ax.set_title('Open scene' if scene=='open' else 'Crossbar obstacle')
        ax.set_xticks(range(len(targets)),[t.replace('_','\n') for t in targets],fontsize=9)
        ax.set_yscale('log');ax.grid(axis='y',which='both',alpha=.18)
        ax.axhline(config['budget'],color='gray',ls='--',lw=1)
    axes[0].set_ylabel('Planning time (seconds, log scale)')
    axes[0].legend(loc='upper left',fontsize=9)
    axis=meta['gantry'].get('second_axis','z').upper()
    fig.suptitle(f'Side-mounted UR5 on X/{axis}/yaw gantry — standalone STOMP + Drake\n'
        f"{config['waypoints']} waypoints · {config['rollouts']} rollouts · {config['seeds']} seeds per case\n"
        'Dots: success; crosses: failure; bars: medians; dashed line: time budget',fontsize=12)
    fig.savefig(args.results/'gantry_planning_times.png',dpi=180)

if __name__=='__main__':main()
