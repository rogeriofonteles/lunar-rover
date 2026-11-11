'use strict';
const data=window.ROUTE_DATA;
const $=id=>document.getElementById(id);
const names={red:'Vermelho',yellow:'Amarelo',green:'Verde'};
const descriptions={
 infinite:'LQR infinito com ganhos calculados ao longo da referência. Soma torque nominal de dinâmica inversa e correção do erro medido.',
 finite:'LQR de horizonte finito sobre a rota completa. A recursão de Riccati usa modelos que variam no tempo e corrige o desvio da discretização.',
 receding:'LQR com janela móvel de 20 passos. Recalcula a política linear a cada medição e aplica apenas o primeiro comando.',
 ilqr:'iLQR otimiza uma vez toda a rota com a dinâmica não linear. Executa o plano com realimentação local; até cinco iterações de otimização.',
 ilqr_receding:'iLQR reotimiza os próximos 20 passos na dinâmica não linear, a cada medição. Até cinco iterações por comando.',
 mpc_sqp:'MPC linear variante no tempo com o SQP explícito especializado: Hessiana exata e conjunto ativo. Impõe limites de torque nos próximos 20 passos.',
 mpc_slsqp:'O mesmo MPC, resolvido pelo SLSQP do SciPy. Mesma referência, custos, modelos e limites de torque.',
 mpc_dual_ascent:'O mesmo MPC, resolvido por ascensão coordenada no dual. Nesta versão de acompanhamento, o plano primal anterior é reutilizado; multiplicadores duais são reinicializados.'
};
const fmt=(v,d=2)=>Number(v).toLocaleString('pt-BR',{minimumFractionDigits:d,maximumFractionDigits:d});
let selected=new URLSearchParams(location.search).get('method')||'mpc_sqp',simTime=0;
const keys=Object.keys(data.runs);
if(!keys.includes(selected))selected=keys[0];
keys.forEach(key=>{const option=document.createElement('option');option.value=key;option.textContent=data.runs[key].summary.label;$('method').append(option);});
const colors={ref:'#2b819e',actual:'#c75132'};

function chart(id,series,options={}){
 const canvas=$(id),rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;
 canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);
 const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);
 const w=rect.width,h=rect.height,p={l:54,r:16,t:12,b:33};
 const transformed=v=>options.log?Math.log10(Math.max(v,1e-4)):v;
 const points=series.flatMap(s=>s.points).filter(p=>p.every(Number.isFinite));
 if(!points.length)return;
 let xmin=options.xmin??Math.min(...points.map(p=>p[0])),xmax=options.xmax??Math.max(...points.map(p=>p[0]));
 let ys=points.map(p=>transformed(p[1]));(options.horizontal||[]).forEach(v=>ys.push(transformed(v)));
 let ymin=options.ymin??Math.min(...ys),ymax=options.ymax??Math.max(...ys);
 let span=Math.max(ymax-ymin,1e-3);ymin-=span*.09;ymax+=span*.09;
 if(options.equal){const unit=Math.max((xmax-xmin)/(w-p.l-p.r),(ymax-ymin)/(h-p.t-p.b));const midx=(xmin+xmax)/2,midy=(ymin+ymax)/2;xmin=midx-unit*(w-p.l-p.r)/2;xmax=midx+unit*(w-p.l-p.r)/2;ymin=midy-unit*(h-p.t-p.b)/2;ymax=midy+unit*(h-p.t-p.b)/2;}
 const X=x=>p.l+(x-xmin)/(xmax-xmin||1)*(w-p.l-p.r),Y=y=>h-p.b-(transformed(y)-ymin)/(ymax-ymin)*(h-p.t-p.b);
 ctx.font='11px Arial';ctx.lineWidth=1;
 for(let i=0;i<5;i++){const f=i/4,x=xmin+f*(xmax-xmin),y=ymin+f*(ymax-ymin),yp=h-p.b-f*(h-p.t-p.b);ctx.strokeStyle='#e0e4da';ctx.beginPath();ctx.moveTo(p.l,yp);ctx.lineTo(w-p.r,yp);ctx.stroke();ctx.fillStyle='#64736b';ctx.textAlign='right';ctx.fillText(fmt(options.log?10**y:y,options.log?2:2),p.l-7,yp+4);ctx.textAlign='center';ctx.fillText(fmt(x,1),X(x),h-10);}
 ctx.save();ctx.beginPath();ctx.rect(p.l,p.t,w-p.l-p.r,h-p.t-p.b);ctx.clip();
 if(options.circle){const c=options.circle;ctx.fillStyle='#ae46b52b';ctx.strokeStyle='#a23cab';ctx.beginPath();ctx.ellipse(X(c[0]),Y(c[1]),c[2]*(w-p.l-p.r)/(xmax-xmin),c[2]*(h-p.t-p.b)/(ymax-ymin),0,0,Math.PI*2);ctx.fill();ctx.stroke();}
 (options.horizontal||[]).forEach(v=>{ctx.setLineDash([5,5]);ctx.strokeStyle='#78877a';ctx.beginPath();ctx.moveTo(p.l,Y(v));ctx.lineTo(w-p.r,Y(v));ctx.stroke();});ctx.setLineDash([]);
 series.forEach(s=>{ctx.strokeStyle=s.color;ctx.lineWidth=s.width||2;ctx.setLineDash(s.dashed?[5,4]:[]);ctx.beginPath();s.points.forEach((point,i)=>{if(!point.every(Number.isFinite))return;i?ctx.lineTo(X(point[0]),Y(point[1])):ctx.moveTo(X(point[0]),Y(point[1]));});ctx.stroke();ctx.setLineDash([]);if(s.marker){ctx.fillStyle=s.color;ctx.beginPath();ctx.arc(X(s.marker[0]),Y(s.marker[1]),4,0,Math.PI*2);ctx.fill();}});
 if(options.cursor){ctx.strokeStyle='#203b3470';ctx.beginPath();ctx.moveTo(X(simTime),p.t);ctx.lineTo(X(simTime),h-p.b);ctx.stroke();}
 ctx.restore();
}

function draw(){
 const run=data.runs[selected],s=run.series,t=s.times,j=Number($('joint').value),deg=$('unit').value==='deg',scale=deg?180/Math.PI:1;
 const series=(values,color)=>({points:values.map((v,i)=>[t[i],v]),color});
 const base={cursor:true,xmin:0,xmax:run.summary.simulated_seconds};
 chart('angle',[series(s.reference.map(x=>x[j]*scale),colors.ref),series(s.states.map(x=>x[j]*scale),colors.actual)],base);
 $('angle-unit').textContent=deg?'graus':'rad';
 chart('velocity',[series(s.reference.map(x=>x[j+6]),colors.ref),series(s.states.map(x=>x[j+6]),colors.actual)],base);
 chart('torque',[series(s.controls.map(x=>x[j]),colors.actual)],{...base,horizontal:[-run.summary.config.torque_limit[j],run.summary.config.torque_limit[j]]});
 chart('error',[series(s.actual_tip.map((x,i)=>1000*Math.hypot(...x.map((v,k)=>v-s.planned_tip[i][k]))),colors.actual)],{...base,ymin:0});
 chart('timing',[series(s.command_seconds.map(x=>x*1000),colors.actual)],{...base,log:true,horizontal:[data.common.timestep*1000]});
 chart('clearance',[series(s.clearance.map(x=>x*1000),colors.actual)],{...base,horizontal:[data.common.scene.margin_m*1000],ymin:0});
 const found=t.findIndex(x=>x>=simTime),index=found<0?t.length-1:found;
 for(const [id,axes] of [['xy',[0,1]],['xz',[0,2]]]){
   const pair=x=>axes.map(i=>x[i]),ob=data.common.scene.spheres[0];
   chart(id,[{points:s.planned_tip.map(pair),color:colors.ref,dashed:true,marker:pair(s.planned_tip[index])},
             {points:s.actual_tip.map(pair),color:colors.actual,marker:pair(s.actual_tip[index])}],
             {equal:true,circle:[...pair(ob.center),ob.radius]});
 }
}

function select(key){
 selected=key;$('method').value=key;simTime=0;
 const run=data.runs[key],s=run.summary;
 $('method-description').textContent=descriptions[key];
 $('parameters').innerHTML=`<dt>Planejador</dt><dd>RRT-Connect</dd><dt>Semente</dt><dd>${data.common.seed}</dd><dt>Período</dt><dd>${fmt(data.common.timestep*1000,0)} ms</dd><dt>Discretização linear</dt><dd>${s.discretization==='zoh'?'Retenção de ordem zero':'Euler'}</dd><dt>Horizonte móvel</dt><dd>${['mpc','receding','ilqr_receding'].includes(s.mode)?s.horizon+' passos':'—'}</dd><dt>Tempo simulado</dt><dd>${fmt(s.simulated_seconds)} s</dd>`;
 $('video').src=`videos/${key}.mp4`;$('video').poster=`videos/${key}-start.png`;
 $('video').playbackRate=Number($('speed').value);
 $('download-video').href=`videos/${key}.mp4`;$('download-run').href=`runs/${key}.npz`;
 const reached=s.visits.slice(1).filter(v=>v.reached).length;
 $('run-status').textContent=s.termination||`${reached} de ${data.common.sequence.length-1} destinos alcançados. ${s.nonconverged_solves} soluções online sem atingir a tolerância numérica.${s.ilqr_plan?' Plano iLQR: '+(s.ilqr_plan.converged?'convergente.':'não convergente; aplicado com realimentação.'):' '}`;
 $('run-status').classList.toggle('bad',!s.completed||reached!==data.common.sequence.length-1);
 $('metrics').innerHTML=[['Destinos alcançados',`${reached} / ${data.common.sequence.length-1}`,'Critério físico na passagem'],['Erro RMS da ponta',fmt(s.rms_tip_error_m*1000,3)+' mm','Referência × execução'],['Menor folga geométrica',fmt(s.min_clearance_m*1000,1)+' mm','Volumes aproximados de colisão'],['Cálculo por comando',fmt(s.command_median_ms,3)+' ms','Mediana; preparação separada']].map(([label,v,note])=>`<div class="metric"><span>${label}</span><strong>${v}</strong><small>${note}</small></div>`).join('');
 $('visits').innerHTML=s.visits.slice(1).map(v=>`<article class="visit"><b>${names[v.button]} <span class="${v.reached?'good':'bad'}">${v.reached?'✓':'×'}</span></b><p>t = ${fmt(v.time_s)} s<br>Erro: ${fmt(v.tip_error_m*1000,3)} mm<br>Velocidade máxima: ${fmt(v.speed_rad_s,4)} rad/s</p></article>`).join('');
 $('command').textContent=`python plan_buttons.py \\\n  --sequence red yellow green red \\\n  --mode ${s.mode}${s.mode==='mpc'?' --solver '+s.solver:''} --horizon 20 \\\n  --seed 6 --headless \\\n  --output output/rota-${key} \\\n  --video output/rota-${key}.mp4`;
 $('config-dump').textContent=JSON.stringify(s,null,2);
 document.querySelectorAll('#comparison tr').forEach(row=>row.classList.toggle('selected',row.dataset.key===key));
 updateClock();draw();
}
function updateClock(){simTime=Math.max(0,Math.min(data.runs[selected].summary.simulated_seconds,($('video').currentTime||0)-1));$('video-status').textContent=`Tempo simulado: ${fmt(simTime)} / ${fmt(data.runs[selected].summary.simulated_seconds)} s`;}
$('planning-table').innerHTML=data.common.planning.map(p=>`<tr><td>${names[p.source]} → ${names[p.destination]}</td><td class="${p.direct_valid?'good':'bad'}">${p.direct_valid?'Livre':'Bloqueada'}</td><td>${p.nodes}</td><td>${p.raw_waypoints} / ${p.waypoints}</td><td>${fmt(p.length_rad)} rad</td><td>${fmt(p.seconds)} s</td></tr>`).join('');
$('shared-preparation').textContent=`Preparação comum: ${fmt(data.common.planning.reduce((sum,p)=>sum+p.seconds,0))} s no planejador e ${fmt(data.common.nominal_preparation_seconds)} s para dinâmica inversa, linearizações e ganhos nominais. Esses cálculos são feitos antes da execução e reutilizados na comparação.`;
$('comparison').innerHTML=keys.map(key=>{const s=data.runs[key].summary;return `<tr data-key="${key}"><td><button class="button" data-select="${key}">${s.label}</button></td><td>${s.visits.slice(1).filter(v=>v.reached).length} / 3</td><td>${fmt(s.rms_tip_error_m*1000,3)} mm</td><td>${fmt(s.min_clearance_m*1000,1)} mm</td><td>${fmt(s.controller_setup_seconds*1000,2)} ms</td><td>${fmt(s.command_median_ms,3)} / ${fmt(s.command_p95_ms,3)} ms</td><td>${s.deadline_misses}</td><td>${s.mode==='ilqr'?(s.ilqr_plan.converged?'0 planos':'1 plano'):s.nonconverged_solves}</td></tr>`;}).join('');
document.querySelectorAll('[data-select]').forEach(button=>button.onclick=()=>{select(button.dataset.select);$('experimento').scrollIntoView({behavior:'smooth'});});
$('method').onchange=()=>select($('method').value);
$('joint').onchange=draw;$('unit').onchange=draw;
$('speed').onchange=()=>{$('video').playbackRate=Number($('speed').value);};
$('video').ontimeupdate=()=>{updateClock();draw();};
$('video').onloadedmetadata=()=>{$('video').playbackRate=Number($('speed').value);};
$('copy-command').onclick=async()=>{try{await navigator.clipboard.writeText($('command').textContent);$('copy-command').textContent='Comando copiado ✓';}catch{$('copy-command').textContent='Selecione e copie o comando acima';}};
window.addEventListener('resize',draw);
select(selected);
