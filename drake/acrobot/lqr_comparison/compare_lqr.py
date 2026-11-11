"""Plot finite- and infinite-horizon LQR behavior for the Acrobot."""

import math
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.systems.analysis import Simulator

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from acrobot_test import finite_horizon_lqr, infinite_horizon_lqr


TIMESTEP = 0.1
DURATION = 10.0
EQUILIBRIUM = np.array([math.pi, 0., 0., 0.])
INITIAL_STATE = np.array([math.pi - 0.1, 0.1, 0., 0.])
STATE_NAMES = [r"$	heta_1$ error", r"$	heta_2$ error",
               r"$dot\theta_1$", r"$dot\theta_2$"]
STATE_UNITS = ["degrees", "degrees", "rad/s", "rad/s"]
CONTROLLERS = ["Finite horizon (N=10)", "Finite horizon (N=100)",
               "Infinite horizon"]


def simulate(gain):
    plant = AcrobotPlant()
    simulator = Simulator(plant)
    context = simulator.get_mutable_context()
    context.SetContinuousState(INITIAL_STATE)
    plant.get_input_port(0).FixValue(context, [0.])
    simulator.Initialize()

    times = np.arange(round(DURATION / TIMESTEP) + 1) * TIMESTEP
    states = [np.asarray(plant.get_output_port(0).Eval(context)).copy()]
    torques = []
    for step in range(len(times) - 1):
        error = states[-1] - EQUILIBRIUM
        error[0] = math.atan2(math.sin(error[0]), math.cos(error[0]))
        error[1] = math.atan2(math.sin(error[1]), math.cos(error[1]))
        torque = float((-gain @ error)[0])
        plant.get_input_port(0).FixValue(context, [torque])
        torques.append(torque)
        simulator.AdvanceTo(times[step + 1])
        states.append(np.asarray(plant.get_output_port(0).Eval(context)).copy())

    errors = np.asarray(states) - EQUILIBRIUM
    errors[:, 0] = np.arctan2(np.sin(errors[:, 0]), np.cos(errors[:, 0]))
    errors[:, 1] = np.arctan2(np.sin(errors[:, 1]), np.cos(errors[:, 1]))
    errors[:, :2] = np.rad2deg(errors[:, :2])
    return times, errors, np.asarray(torques)


def write_interactive_dashboard(results, gains, horizons, distances):
    colors = ["#e68632", "#2878b5", "#26845b"]
    data = {
        "controllers": CONTROLLERS,
        "colors": colors,
        "signals": {},
        "gainConvergence": {
            "x": horizons.tolist(),
            "y": np.log10(np.maximum(distances, 1e-15)).tolist(),
            "raw": distances.tolist(),
        },
        "gains": {name: gain.ravel().tolist()
                  for name, gain in gains.items()},
    }
    signal_names = ["theta1", "theta2", "theta1dot", "theta2dot", "torque"]
    for signal_index, signal_name in enumerate(signal_names):
        data["signals"][signal_name] = {}
        for name, (times, errors, torques) in results.items():
            data["signals"][signal_name][name] = {
                "x": (times[:-1] if signal_index == 4 else times).tolist(),
                "y": (torques if signal_index == 4
                      else errors[:, signal_index]).tolist(),
            }

    embedded_data = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    html = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Acrobot LQR comparison</title>
<style>
:root{color-scheme:light;--ink:#17212b;--muted:#5b6772;--line:#dce2e7;--panel:#fff;--bg:#f3f6f8;--accent:#425b76}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1180px;margin:32px auto;padding:0 22px}.intro{margin-bottom:22px}.intro h1{font-size:clamp(26px,4vw,38px);line-height:1.15;margin:0 0 9px}.intro p{max-width:850px;color:var(--muted);margin:0}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:18px 20px 14px;box-shadow:0 5px 20px #17212b0a}
.toolbar{display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin-bottom:14px}.toolbar label{font-weight:650}.toolbar select,button{font:inherit;color:var(--ink);background:#fff;border:1px solid #bac5ce;border-radius:8px;padding:8px 11px}.toolbar select{min-width:245px}button{cursor:pointer}button:hover{background:#edf2f6}
.methods{display:flex;gap:15px;flex-wrap:wrap;margin-left:auto}.methods label{font-weight:500;white-space:nowrap}.methods input{accent-color:var(--accent);vertical-align:-1px;margin-right:5px}
.chart-wrap{position:relative;width:100%;height:min(62vh,550px);min-height:340px}canvas{width:100%;height:100%;display:block;touch-action:none;cursor:crosshair}.tip{position:absolute;display:none;pointer-events:none;background:#17212bf2;color:white;padding:8px 10px;border-radius:7px;font-size:13px;white-space:nowrap;box-shadow:0 4px 14px #0003;z-index:2}
.hint{color:var(--muted);font-size:13px;margin:6px 0 0}.findings{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-top:16px}.finding{background:#fff;border:1px solid var(--line);border-radius:11px;padding:14px 15px}.finding strong{display:block;margin-bottom:4px}.finding span{color:var(--muted);font-size:14px}
.gain-card{margin-top:16px}.gain-card h2{font-size:19px;margin:0 0 4px}.gain-card p{margin:0;color:var(--muted)}.gain-values{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-top:12px}.gain-values pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f5f7f9;padding:12px;border-radius:8px;margin:0;font-size:12px;color:#34404b}
@media(max-width:700px){main{margin:20px auto;padding:0 12px}.card{padding:14px 12px}.methods{margin-left:0}.findings,.gain-values{grid-template-columns:1fr}.chart-wrap{height:430px;min-height:320px}}
</style></head><body><main>
<header class="intro"><h1>Acrobot LQR: finite vs. infinite horizon</h1><p>Both controllers use the same upright equilibrium, initial state, cost weights, and 0.1 s control interval. Explore the trajectories and feedback gains below. The finite-horizon controller is evaluated with a fixed look-ahead and reused at each step.</p></header>
<section class="card"><div class="toolbar"><label for="signal">Plot</label><select id="signal"><option value="theta1">Shoulder angle error (degrees)</option><option value="theta2">Elbow angle error (degrees)</option><option value="theta1dot">Shoulder angular velocity (rad/s)</option><option value="theta2dot">Elbow angular velocity (rad/s)</option><option value="torque">Control torque (N·m)</option><option value="gain">Gain convergence vs. horizon</option></select><button id="reset">Reset zoom</button><div class="methods" id="methods"></div></div>
<div class="chart-wrap"><canvas id="chart" aria-label="Interactive Acrobot controller comparison chart"></canvas><div class="tip" id="tip"></div></div><p class="hint">Hover to inspect values. Use the mouse wheel or pinch to zoom; drag to pan. Use Reset zoom to restore the full view.</p></section>
<section class="findings"><div class="finding"><strong>Short look-ahead (N=10)</strong><span>Produces much larger, oscillatory corrections for this initial state.</span></div><div class="finding"><strong>Long look-ahead (N=100)</strong><span>Its gain is nearly identical to the infinite-horizon solution.</span></div><div class="finding"><strong>What changes</strong><span>Increasing the horizon changes the finite-horizon value estimate and therefore the feedback gain; at this model and sampling interval, convergence is rapid after the first few dozen steps.</span></div></section>
<section class="card gain-card"><h2>Feedback gains</h2><p>Torque is computed as u = −Kx, where x is the state error from upright. Coefficients below correspond to shoulder angle, elbow angle, shoulder velocity, and elbow velocity.</p><div class="gain-values" id="gain-values"></div></section>
</main><script>
const D=__DATA__, canvas=document.getElementById('chart'),ctx=canvas.getContext('2d'),select=document.getElementById('signal'),tip=document.getElementById('tip'),methods=document.getElementById('methods');
const labels={theta1:'Shoulder angle error (degrees)',theta2:'Elbow angle error (degrees)',theta1dot:'Shoulder angular velocity (rad/s)',theta2dot:'Elbow angular velocity (rad/s)',torque:'Control torque (N·m)',gain:'log₁₀ ‖Kₙ − K∞‖₂'};
const unit={theta1:'degrees',theta2:'degrees',theta1dot:'rad/s',theta2dot:'rad/s',torque:'N·m',gain:'log₁₀ gain distance'};
const enabled=Object.fromEntries(D.controllers.map(n=>[n,true]));let view=null,drag=null,lastPoints=[];
D.controllers.forEach((name,i)=>{const l=document.createElement('label'),c=document.createElement('input');c.type='checkbox';c.checked=true;c.addEventListener('change',()=>{enabled[name]=c.checked;draw()});l.append(c,name);methods.append(l)});
const gainValues=document.getElementById('gain-values');D.controllers.forEach((name,i)=>{const box=document.createElement('pre'),k=D.gains[name];box.style.borderLeft='4px solid '+D.colors[i];box.textContent=name+'\nK = ['+k.map(v=>v.toFixed(4)).join(', ')+']';gainValues.append(box)});
function dims(){const r=canvas.getBoundingClientRect(),d=window.devicePixelRatio||1;if(canvas.width!==Math.round(r.width*d)||canvas.height!==Math.round(r.height*d)){canvas.width=Math.round(r.width*d);canvas.height=Math.round(r.height*d)}ctx.setTransform(d,0,0,d,0,0);return {w:r.width,h:r.height}}
function series(){const key=select.value;if(key==='gain')return [{name:'Finite-horizon gain distance',color:'#6655a5',x:D.gainConvergence.x,y:D.gainConvergence.y,raw:D.gainConvergence.raw}];return D.controllers.filter(n=>enabled[n]).map((n,i)=>({name:n,color:D.colors[i],...D.signals[key][n]}))}
function draw(){const {w,h}=dims(),s=series(),p={l:76,r:24,t:22,b:58},pw=w-p.l-p.r,ph=h-p.t-p.b;ctx.clearRect(0,0,w,h);if(!s.length){ctx.fillStyle='#52616e';ctx.fillText('Select a controller to display its trace.',p.l,p.t+30);return}
let xmin=Math.min(...s.flatMap(a=>a.x)),xmax=Math.max(...s.flatMap(a=>a.x)),ymin=Math.min(...s.flatMap(a=>a.y)),ymax=Math.max(...s.flatMap(a=>a.y));if(ymin===ymax){ymin-=1;ymax+=1}if(!view){const pad=(ymax-ymin)*.09;view={xmin,xmax,ymin:ymin-pad,ymax:ymax+pad}}let {xmin:x0,xmax:x1,ymin:y0,ymax:y1}=view;
const X=x=>p.l+(x-x0)/(x1-x0)*pw,Y=y=>p.t+ph-(y-y0)/(y1-y0)*ph;
ctx.font='12px system-ui';ctx.strokeStyle='#e4e9ed';ctx.fillStyle='#586570';ctx.lineWidth=1;
for(let i=0;i<=5;i++){let y=p.t+ph*i/5,val=y1-(y1-y0)*i/5;ctx.beginPath();ctx.moveTo(p.l,y);ctx.lineTo(p.l+pw,y);ctx.stroke();ctx.textAlign='right';ctx.textBaseline='middle';ctx.fillText(Number(val.toPrecision(3)).toString(),p.l-9,y)}
for(let i=0;i<=5;i++){let x=p.l+pw*i/5,val=x0+(x1-x0)*i/5;ctx.beginPath();ctx.moveTo(x,p.t);ctx.lineTo(x,p.t+ph);ctx.stroke();ctx.textAlign='center';ctx.textBaseline='top';ctx.fillText(Number(val.toPrecision(3)).toString(),x,p.t+ph+8)}
ctx.fillStyle='#26333e';ctx.textAlign='center';ctx.textBaseline='bottom';ctx.font='14px system-ui';ctx.fillText(labels[select.value],p.l+pw/2,h-5);ctx.save();ctx.translate(17,p.t+ph/2);ctx.rotate(-Math.PI/2);ctx.fillText(unit[select.value],0,0);ctx.restore();lastPoints=[];
s.forEach(a=>{ctx.beginPath();ctx.strokeStyle=a.color;ctx.lineWidth=2.4;let points=[];a.x.forEach((x,i)=>{const q={x:X(x),y:Y(a.y[i]),xv:x,yv:a.y[i],raw:a.raw?.[i],name:a.name,color:a.color};points.push(q);i?ctx.lineTo(q.x,q.y):ctx.moveTo(q.x,q.y)});ctx.stroke();lastPoints.push(...points);ctx.fillStyle=a.color;ctx.textAlign='left';ctx.textBaseline='top';ctx.font='12px system-ui';let li=lastPoints.length;});
let ly=p.t+ph+35,legendX=p.l;ctx.font='12px system-ui';s.forEach(a=>{ctx.fillStyle=a.color;ctx.fillRect(legendX,ly,15,3);ctx.fillStyle='#34414c';ctx.textAlign='left';ctx.textBaseline='middle';ctx.fillText(a.name,legendX+21,ly+1);legendX+=Math.min(225,ctx.measureText(a.name).width+42)});}
function reset(){view=null;draw()}document.getElementById('reset').onclick=reset;select.onchange=()=>{view=null;methods.style.visibility=select.value==='gain'?'hidden':'visible';draw()};
canvas.addEventListener('pointerdown',e=>{drag={x:e.offsetX,y:e.offsetY,view:{...view}};canvas.setPointerCapture(e.pointerId)});canvas.addEventListener('pointermove',e=>{const r=canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;if(drag){const w=canvas.clientWidth-100,h=canvas.clientHeight-80,dx=(x-drag.x)/w*(drag.view.xmax-drag.view.xmin),dy=(y-drag.y)/h*(drag.view.ymax-drag.view.ymin);view={xmin:drag.view.xmin-dx,xmax:drag.view.xmax-dx,ymin:drag.view.ymin+dy,ymax:drag.view.ymax+dy};draw();return}let best=null,dist=Infinity;for(const q of lastPoints){const d=(q.x-x)**2+(q.y-y)**2;if(d<dist){best=q;dist=d}}if(best&&dist<900){tip.style.display='block';tip.style.left=Math.min(x+14,canvas.clientWidth-225)+'px';tip.style.top=Math.max(5,y-35)+'px';const val=best.raw??best.yv;tip.textContent=best.name+' · '+(select.value==='gain'?'N = '+best.xv+' · ‖ΔK‖ = '+val.toExponential(3):'t = '+best.xv.toFixed(2)+' s · '+val.toFixed(4)+' '+unit[select.value])}else tip.style.display='none'});canvas.addEventListener('pointerup',()=>drag=null);canvas.addEventListener('pointercancel',()=>drag=null);canvas.addEventListener('wheel',e=>{e.preventDefault();if(!view)return;const f=e.deltaY<0?.82:1.22,xfrac=e.offsetX/canvas.clientWidth,yfrac=1-e.offsetY/canvas.clientHeight;const cx=view.xmin+xfrac*(view.xmax-view.xmin),cy=view.ymin+yfrac*(view.ymax-view.ymin);view={xmin:cx+(view.xmin-cx)*f,xmax:cx+(view.xmax-cx)*f,ymin:cy+(view.ymin-cy)*f,ymax:cy+(view.ymax-cy)*f};draw()},{passive:false});new ResizeObserver(draw).observe(canvas);draw();
</script></body></html>'''.replace("__DATA__", embedded_data)
    output = Path(__file__).with_name("lqr_comparison.html")
    output.write_text(html)
    return output


def main():
    plant = AcrobotPlant()
    gains = {
        "Finite horizon (N=10)": finite_horizon_lqr(
            plant, timestep=TIMESTEP, horizon=10,
            equilibrium_state=EQUILIBRIUM),
        "Finite horizon (N=100)": finite_horizon_lqr(
            plant, timestep=TIMESTEP, horizon=100,
            equilibrium_state=EQUILIBRIUM),
        "Infinite horizon": infinite_horizon_lqr(
            plant, timestep=TIMESTEP, equilibrium_state=EQUILIBRIUM),
    }
    results = {name: simulate(gain) for name, gain in gains.items()}
    colors = ["#e68632", "#2878b5", "#26845b"]

    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True,
                             constrained_layout=True)
    for index, (axis, name, unit) in enumerate(zip(
            axes.flat[:4], STATE_NAMES, STATE_UNITS)):
        for (label, (times, errors, _)), color in zip(results.items(), colors):
            axis.plot(times, errors[:, index], label=label, color=color,
                      linewidth=2)
        axis.axhline(0, color="black", linewidth=0.7, alpha=0.5)
        axis.set_title(name)
        axis.set_ylabel(unit)
        axis.grid(True, alpha=0.25)

    torque_axis = axes.flat[4]
    for (label, (times, _, torques)), color in zip(results.items(), colors):
        torque_axis.step(times[:-1], torques, where="post", label=label,
                         color=color, linewidth=1.8)
    torque_axis.axhline(0, color="black", linewidth=0.7, alpha=0.5)
    torque_axis.set_title("Control torque")
    torque_axis.set_ylabel("N·m")
    torque_axis.grid(True, alpha=0.25)
    torque_axis.legend(fontsize=8)

    # Compare the feedback coefficients directly; these determine how each
    # state error contributes to the torque command u = -K x_error.
    gain_axis = axes.flat[5]
    indices = np.arange(4)
    width = 0.24
    for i, ((label, gain), color) in enumerate(zip(gains.items(), colors)):
        gain_axis.bar(indices + (i - 1) * width, gain.ravel(), width,
                      label=label, color=color)
    gain_axis.set_xticks(indices, [r"$e_{\theta_1}$", r"$e_{\theta_2}$",
                                   r"$\dot\theta_1$", r"$\dot\theta_2$"])
    gain_axis.axhline(0, color="black", linewidth=0.7)
    gain_axis.set_title("Feedback gain coefficients")
    gain_axis.set_ylabel("N·m per state unit")
    gain_axis.grid(True, axis="y", alpha=0.25)

    torque_axis.set_xlabel("Time (s)")
    gain_axis.set_xlabel("")
    fig.suptitle("Acrobot LQR: finite vs. infinite horizon", fontsize=16)
    output = Path(__file__).with_name("lqr_comparison.png")
    fig.savefig(output, dpi=180)
    plt.close(fig)

    # A second plot shows how the finite-horizon first-stage gain approaches
    # the infinite-horizon gain as the look-ahead horizon grows.
    horizons = np.array([1, 2, 5, 10, 15, 20, 25, 30, 35, 40, 45,
                         50, 55, 60, 70, 80, 100, 150, 200])
    infinite_gain = gains["Infinite horizon"]
    distances = []
    for horizon in horizons:
        finite_gain = finite_horizon_lqr(
            plant, timestep=TIMESTEP, horizon=int(horizon),
            equilibrium_state=EQUILIBRIUM)
        distances.append(np.linalg.norm(finite_gain - infinite_gain))

    dashboard = write_interactive_dashboard(
        results, gains, horizons, np.asarray(distances))

    fig, axis = plt.subplots(figsize=(8, 5), constrained_layout=True)
    axis.plot(horizons, distances, marker="o", linewidth=2, color="#6655a5")
    axis.set_xscale("log")
    axis.set_yscale("log")
    axis.set_xlabel("Finite horizon length N (steps of 0.1 s)")
    axis.set_ylabel(r"Gain difference $\|K_N - K_\infty\|_2$")
    axis.set_title("Finite-horizon gain convergence")
    axis.grid(True, which="both", alpha=0.25)
    convergence_output = Path(__file__).with_name("lqr_gain_convergence.png")
    fig.savefig(convergence_output, dpi=180)
    plt.close(fig)

    print(f"Saved {output}")
    print(f"Saved {convergence_output}")
    print(f"Saved interactive dashboard {dashboard}")
    for name, gain in gains.items():
        print(f"{name}: K = {np.array2string(gain, precision=4)}")


if __name__ == "__main__":
    main()
