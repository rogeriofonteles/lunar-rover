'use strict';
const $=id=>document.getElementById(id);
const catalog=window.ROUTE_CASES;
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

const fmt=(v,d=2)=>v==null?'—':Number(v).toLocaleString('pt-BR',{minimumFractionDigits:d,maximumFractionDigits:d});
const params=new URLSearchParams(location.search);
let selected=params.get('method')||'mpc_sqp',caseKey='nominal',data,keys=[],simTime=0;
const visible=new Set(['infinite','finite','mpc_sqp']);
const palette={infinite:'#ba4d2c',finite:'#287d59',receding:'#a2740c',ilqr:'#2c7897',ilqr_receding:'#a5448d',mpc_sqp:'#423aa0',mpc_slsqp:'#75533d',mpc_dual_ascent:'#62757c'};
const colors={ref:'#2b819e',actual:'#c75132'};
const label=k=>data.runs[k].summary.label;
const comp=k=>data.runs[k].comparison;
const reached=s=>s.visits.slice(1).filter(v=>v.reached).length;
const commonEnd=()=>Math.min(...keys.map(k=>data.runs[k].summary.simulated_seconds));
const rms=k=>{const c=comp(k),values=c.tip_error_mm.filter((_,i)=>c.times[i]<=commonEnd()+1e-9);return Math.sqrt(values.reduce((a,v)=>a+v*v,0)/values.length);};
const pulse=()=>data.common.experiment.pulse;
const extrema=values=>[Math.min(...values),Math.max(...values)];

function chart(id,series,options={}){
 const canvas=$(id),rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;
 if(!rect.width||!rect.height)return;
 canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);
 const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);
 const w=rect.width,h=rect.height,p={l:57,r:18,t:22,b:34};
 const TX=v=>options.xlog?Math.log10(Math.max(v,1e-5)):v;
 const TY=v=>options.log?Math.log10(Math.max(v,1e-5)):v;
 const valid=p=>p.every(Number.isFinite);
 const points=series.flatMap(s=>s.points).filter(valid);
 ctx.font='11px Arial';
 if(!points.length){ctx.fillStyle='#64736b';ctx.fillText('Sem amostras nesta janela.',p.l,h/2);return;}
 let xmin=TX(options.xmin??Math.min(...points.map(p=>p[0]))),xmax=TX(options.xmax??Math.max(...points.map(p=>p[0])));
 const ys=points.map(p=>TY(p[1]));(options.horizontal||[]).forEach(v=>ys.push(TY(v)));
 let ymin=options.ymin==null?Math.min(...ys):TY(options.ymin),ymax=options.ymax==null?Math.max(...ys):TY(options.ymax);
 const span=Math.max(ymax-ymin,1e-3);if(options.ymin==null)ymin-=span*.06;ymax+=span*.12;
 if(options.equal){const unit=Math.max((xmax-xmin)/(w-p.l-p.r),(ymax-ymin)/(h-p.t-p.b));const mx=(xmin+xmax)/2,my=(ymin+ymax)/2;xmin=mx-unit*(w-p.l-p.r)/2;xmax=mx+unit*(w-p.l-p.r)/2;ymin=my-unit*(h-p.t-p.b)/2;ymax=my+unit*(h-p.t-p.b)/2;}
 const X=x=>p.l+(TX(x)-xmin)/(xmax-xmin||1)*(w-p.l-p.r),Y=y=>h-p.b-(TY(y)-ymin)/(ymax-ymin)*(h-p.t-p.b);
 ctx.lineWidth=1;
 for(let i=0;i<5;i++){const f=i/4,x=xmin+f*(xmax-xmin),y=ymin+f*(ymax-ymin),yp=h-p.b-f*(h-p.t-p.b);ctx.strokeStyle='#e0e4da';ctx.beginPath();ctx.moveTo(p.l,yp);ctx.lineTo(w-p.r,yp);ctx.stroke();ctx.fillStyle='#64736b';ctx.textAlign='right';ctx.fillText(fmt(options.log?10**y:y,2),p.l-7,yp+4);ctx.textAlign='center';ctx.fillText(fmt(options.xlog?10**x:x,options.xlog?3:1),p.l+f*(w-p.l-p.r),h-10);}
 ctx.save();ctx.beginPath();ctx.rect(p.l,p.t,w-p.l-p.r,h-p.t-p.b);ctx.clip();
 for(const band of options.bands||[]){ctx.fillStyle='#c49b3330';ctx.fillRect(X(band[0]),p.t,X(band[1])-X(band[0]),h-p.t-p.b);}
 if(options.circle){const c=options.circle;ctx.fillStyle='#ae46b52b';ctx.strokeStyle='#a23cab';ctx.beginPath();ctx.ellipse(X(c[0]),Y(c[1]),c[2]*(w-p.l-p.r)/(xmax-xmin),c[2]*(h-p.t-p.b)/(ymax-ymin),0,0,Math.PI*2);ctx.fill();ctx.stroke();}
 for(const v of options.horizontal||[]){ctx.setLineDash([5,5]);ctx.strokeStyle='#78877a';ctx.beginPath();ctx.moveTo(p.l,Y(v));ctx.lineTo(w-p.r,Y(v));ctx.stroke();}
 for(const v of options.vertical||[]){ctx.setLineDash([5,5]);ctx.strokeStyle='#78877a';ctx.beginPath();ctx.moveTo(X(v),p.t);ctx.lineTo(X(v),h-p.b);ctx.stroke();}
 ctx.setLineDash([]);
 series.forEach(s=>{
   ctx.strokeStyle=s.color;ctx.fillStyle=s.color;ctx.lineWidth=s.width||2;ctx.setLineDash(s.dashed?[5,4]:[]);ctx.beginPath();let started=false;
   s.points.forEach(point=>{if(!valid(point)){started=false;return;}if(s.dots){ctx.moveTo(X(point[0])+5,Y(point[1]));ctx.arc(X(point[0]),Y(point[1]),5,0,Math.PI*2);}else{started?ctx.lineTo(X(point[0]),Y(point[1])):ctx.moveTo(X(point[0]),Y(point[1]));started=true;}});
   s.dots?ctx.fill():ctx.stroke();ctx.setLineDash([]);
   if(s.marker){ctx.beginPath();ctx.arc(X(s.marker[0]),Y(s.marker[1]),4,0,Math.PI*2);ctx.fill();}
 });
 if(options.cursor){ctx.strokeStyle='#203b3470';ctx.beginPath();ctx.moveTo(X(simTime),p.t);ctx.lineTo(X(simTime),h-p.b);ctx.stroke();}
 ctx.restore();
 if(options.bands?.length){ctx.fillStyle='#8b671b';ctx.textAlign='left';ctx.fillText('Faixa amarela: pulso externo (6–6,3 s)',p.l,13);}
 if(series.some(s=>s.label)){
  let readout=canvas.parentElement.querySelector('.chart-readout');
  if(!readout){readout=document.createElement('p');readout.className='chart-readout caption';readout.textContent='Passe o ponteiro ou toque no gráfico para ler os valores.';canvas.after(readout);}
  const version=JSON.stringify([caseKey,[...visible],$('compare-joint').value,$('window').value]);
  if(readout.dataset.version!==version){readout.textContent='Passe o ponteiro ou toque no gráfico para ler os valores.';readout.dataset.version=version;}
  const inspect=event=>{
   const px=event.clientX-canvas.getBoundingClientRect().left;
   if(options.xlog){
    const py=event.clientY-canvas.getBoundingClientRect().top;
    const nearest=series.filter(s=>s.points.length).map(s=>({s,p:s.points[0]})).sort((a,b)=>Math.hypot(X(a.p[0])-px,Y(a.p[1])-py)-Math.hypot(X(b.p[0])-px,Y(b.p[1])-py))[0];
    if(nearest)readout.textContent=`${nearest.s.label}: ${fmt(nearest.p[0],3)} ms por comando; ${fmt(nearest.p[1],3)} mm de erro RMS comum.`;
   }else{
    const samples=series.map(s=>({s,p:s.points.reduce((best,p)=>!best||Math.abs(X(p[0])-px)<Math.abs(X(best[0])-px)?p:best,null)})).filter(x=>x.p);
    readout.textContent=samples.length?`t ≈ ${fmt(samples[0].p[0])} s · `+samples.map(x=>`${x.s.label}: ${fmt(x.p[1],3)} ${options.unit||''}`).join(' · '):'Sem amostras.';
   }
  };
  canvas.onpointermove=inspect;canvas.onpointerdown=inspect;
 }
}

function windowBounds(){
 const end=Math.max(...keys.map(k=>data.runs[k].summary.simulated_seconds));
 if($('window').value==='event'&&pulse())return [Math.max(0,pulse().start_s-1),Math.min(end,pulse().start_s+pulse().duration_s+3)];
 if($('window').value==='peak'){
   let peak=-1,time=0;
   keys.filter(k=>visible.has(k)).forEach(k=>{const c=comp(k);c.tip_error_mm.forEach((e,i)=>{if(e>peak){peak=e;time=c.times[i];}});});
   return [Math.max(0,time-1.5),Math.min(end,time+1.5)];
 }
 return [0,end];
}

function drawComparisons(){
 const [start,end]=windowBounds(),j=Number($('compare-joint').value);
 const current=keys.filter(k=>visible.has(k));
 const series=(field,convert=v=>v)=>current.map(k=>({label:label(k),color:palette[k],width:k===selected?2.8:1.7,
   points:comp(k)[field].map((v,i)=>[comp(k).times[i],convert(v)]).filter(p=>p[0]>=start-1e-9&&p[0]<=end+1e-9)}));
 const base={xmin:start,xmax:end,cursor:true,bands:pulse()?[[pulse().start_s,pulse().start_s+pulse().duration_s]]:[]};
 chart('compare-error',series('tip_error_mm'),{...base,ymin:0,unit:'mm'});
 chart('compare-joint-error',series('joint_error_deg',x=>x[j]),{...base,horizontal:[0],unit:'°'});
 chart('compare-torque',series('utilization_percent'),{...base,ymin:0,horizontal:[100],unit:'%'});
 const costs=current.map(k=>data.runs[k].summary.command_median_ms);
 chart('tradeoff',current.map(k=>({label:label(k),color:palette[k],dots:true,points:[[data.runs[k].summary.command_median_ms,rms(k)]]})),
   {xlog:true,xmin:Math.min(...costs,20)*.5,xmax:Math.max(...costs,20)*2,ymin:0,vertical:[20]});
 $('window-note').textContent=`Janela: ${fmt(start)}–${fmt(end)} s. As cores identificam os mesmos métodos em todos os gráficos.`;
}

function draw(){
 if(!data)return;
 drawComparisons();
 const run=data.runs[selected],s=run.series,t=s.times,j=Number($('joint').value),deg=$('unit').value==='deg',scale=deg?180/Math.PI:1;
 const series=(values,color)=>({points:values.map((v,i)=>[t[i],v]),color});
 const base={cursor:true,xmin:0,xmax:run.summary.simulated_seconds};
 if($('details-plots').open){
  chart('angle',[series(s.reference.map(x=>x[j]*scale),colors.ref),series(s.states.map(x=>x[j]*scale),colors.actual)],base);
  $('angle-unit').textContent=deg?'graus':'rad';
  chart('velocity',[series(s.reference.map(x=>x[j+6]),colors.ref),series(s.states.map(x=>x[j+6]),colors.actual)],base);
  chart('torque',[{...series(s.reference_controls.map(x=>x[j]),colors.ref),dashed:true},series(s.controls.map(x=>x[j]),colors.actual)],{...base,horizontal:[-run.summary.config.torque_limit[j],run.summary.config.torque_limit[j]]});
  chart('error',[{points:comp(selected).tip_error_mm.map((x,i)=>[comp(selected).times[i],x]),color:colors.actual}],{...base,ymin:0});
  chart('timing',[{points:comp(selected).command_ms.map((x,i)=>[comp(selected).times[i],x]),color:colors.actual}],{...base,log:true,horizontal:[data.common.timestep*1000]});
  chart('clearance',[series(s.clearance.map(x=>x==null?NaN:x*1000),colors.actual)],{...base,horizontal:[data.common.scene.margin_m*1000,0]});
 }
 const found=t.findIndex(x=>x>=simTime),index=found<0?t.length-1:found;
 for(const [id,axes] of [['xy',[0,1]],['xz',[0,2]]]){
   const pair=x=>axes.map(i=>x[i]),ob=data.common.scene.spheres[0];
   chart(id,[{points:s.planned_tip.map(pair),color:colors.ref,dashed:true,marker:pair(s.planned_tip[index])},
             {points:s.actual_tip.map(pair),color:colors.actual,marker:pair(s.actual_tip[index])}],
             {equal:true,circle:[...pair(ob.center),ob.radius]});
 }
}

function findings(){
 const summaries=keys.map(k=>data.runs[k].summary),complete=summaries.filter(s=>s.completed).length;
 const full=summaries.filter(s=>s.completed&&reached(s)===3).length;
 const [lo,hi]=extrema(keys.map(rms));
 let title,text;
 if(caseKey==='nominal'){
  title='Precisão parecida; custo diferente.';
  text=`O erro RMS no intervalo comum varia de <strong>${fmt(lo,3)} a ${fmt(hi,3)} mm</strong>. A referência suave e a dinâmica conhecida deixam pouca correção para os controladores. Compare o gráfico de custo antes de atribuir vantagem a diferenças submilimétricas.`;
 }else if(caseKey==='fast'){
  title='Acelerar expõe os transientes.';
  const peak=Math.max(...summaries.map(s=>s.max_tip_error_m))*1000;
  text=`A rota dura <strong>${fmt(Math.max(...summaries.map(s=>s.simulated_seconds)))} s</strong>, contra 24,18 s no nominal. O maior pico observado é <strong>${fmt(peak,2)} mm</strong>. O RMS comum varia de <strong>${fmt(lo,2)} a ${fmt(hi,2)} mm</strong>: acelerar aumenta a exigência, mas não separa necessariamente todos os métodos. Amplie a região de maior erro e compare as frenagens.`;
 }else if(caseKey==='torque'){
  title='Limites ativos mudam o problema.';
  const [a,b]=extrema(summaries.map(s=>s.saturated_step_percent));
  text=`Os motores atingem o limite em <strong>${fmt(a,1)}% a ${fmt(b,1)}%</strong> dos passos executados. Concluir a rota não significa chegar a cada botão na hora prevista. Compare erro e saturação junto com a coluna de destinos; uma média menor não compensa uma colisão.`;
  const lqr=data.runs.infinite?.summary,mpc=data.runs.mpc_sqp?.summary;
  if(lqr&&mpc)text+=` <strong>LQR infinito:</strong> ${lqr.completed?'concluiu':'interrompido em '+fmt(lqr.simulated_seconds)+' s'}, ${reached(lqr)}/3 destinos. <strong>MPC-SQP:</strong> ${mpc.completed?'concluiu':'interrompido em '+fmt(mpc.simulated_seconds)+' s'}, ${reached(mpc)}/3 destinos.`;
 }else if(caseKey==='payload'){
  title='Um modelo errado afeta todos.';
  text=`A carga não modelada é de <strong>0,5 kg</strong>. O erro RMS comum fica entre <strong>${fmt(lo,1)} e ${fmt(hi,1)} mm</strong>. Observe o erro persistente e as interrupções. Reotimizar com o modelo nominal não equivale a estimar a carga desconhecida.`;
 }else{
  title='Compare o pico e a recuperação.';
  const rec=summaries.map(s=>s.recovery_seconds).filter(v=>v!=null);
  text=`O pulso atua de <strong>6,0 a 6,3 s</strong>. ${rec.length?`Entre os métodos que recuperaram, o tempo após o fim do pulso varia de <strong>${fmt(Math.min(...rec))} a ${fmt(Math.max(...rec))} s</strong>.`:'Nenhum método atingiu o critério de recuperação.'} A resolução dessa medida é <strong>20 ms</strong>. Amplie a janela do evento; o RMS da rota inteira dilui um distúrbio curto.`;
 }
 const sorted=[...keys].sort((a,b)=>data.runs[a].summary.command_median_ms-data.runs[b].summary.command_median_ms);
 const fastest=data.runs[sorted[0]].summary,slowest=data.runs[sorted.at(-1)].summary;
 $('case-findings').innerHTML=`<article class="finding"><h3>${title}</h3><p>${text}</p></article><article class="finding"><h3>${complete} / ${keys.length} concluíram a rota.</h3><p><strong>${full} / ${keys.length}</strong> também atenderam ao critério nos três destinos. Custo mediano: <strong>${fmt(fastest.command_median_ms,3)} ms</strong> (${fastest.label}) a <strong>${fmt(slowest.command_median_ms,3)} ms</strong> (${slowest.label}). O orçamento por comando é <strong>20 ms</strong>; o vídeo não mostra a espera pelo cálculo.</p></article>`;
}

function pairReading(){
 const a=$('compare-a').value,b=$('compare-b').value,sa=data.runs[a].summary,sb=data.runs[b].summary;
 if(a===b){$('pair-reading').textContent='Selecione dois métodos diferentes para comparar.';return;}
 const delta=rms(b)-rms(a),ratio=sb.command_median_ms/sa.command_median_ms;
 $('pair-reading').textContent=`B em relação a A: ${Math.abs(delta)<.0005?'erro RMS comum igual na precisão exibida':fmt(Math.abs(delta),3)+' mm '+(delta<0?'menos':'mais')+' de erro RMS comum'}. Custo por comando: ${fmt(ratio,2)}× o de A. Rota: A ${sa.completed?'concluída':'interrompida'}, B ${sb.completed?'concluída':'interrompida'}; destinos A ${reached(sa)}/3, B ${reached(sb)}/3. ${a.startsWith('mpc_')&&b.startsWith('mpc_')?`São solvers do mesmo MPC: trajetórias semelhantes são esperadas. Soluções fora da tolerância numérica: A = ${sa.nonconverged_solves}, B = ${sb.nonconverged_solves}. Compare também tempo e convergência.`:'Esses números descrevem este caso, não uma superioridade geral.'}`;
}

function comparisonTable(){
 let order=[...keys];
 const sort=$('sort').value;
 if(sort==='error')order.sort((a,b)=>rms(a)-rms(b));
 if(sort==='time')order.sort((a,b)=>data.runs[a].summary.command_median_ms-data.runs[b].summary.command_median_ms);
 if(sort==='saturation')order.sort((a,b)=>data.runs[a].summary.saturated_step_percent-data.runs[b].summary.saturated_step_percent);
 $('comparison-window').textContent=`Erro RMS calculado em 0–${fmt(commonEnd())} s para TODOS os métodos, inclusive os interrompidos. Este intervalo comum evita comparar médias de trechos diferentes. As curvas mostram todo o trecho disponível de cada execução.`;
 $('comparison').innerHTML=order.map(key=>{const s=data.runs[key].summary;return `<tr data-key="${key}" class="${key===selected?'selected':''}"><td><button class="button method-name" data-select="${key}" style="--method-color:${palette[key]}"><i></i>${s.label}</button></td><td><span class="${s.completed?'good':'bad'}">${s.completed?'Concluída':'Interrompida'}</span><small>${reached(s)} / 3 destinos · ${fmt(s.simulated_seconds)} s</small></td><td>${fmt(rms(key),3)} mm</td><td>${fmt(s.max_tip_error_m*1000,2)} mm</td><td>${fmt(s.saturated_step_percent,1)}%</td><td>${pulse()?(s.recovery_seconds==null?'Não recuperou':fmt(s.recovery_seconds)+' s'):'—'}</td><td>${fmt(s.controller_setup_seconds*1000,2)} ms</td><td>${fmt(s.command_median_ms,3)} / ${fmt(s.command_p95_ms,3)} ms</td><td>${s.deadline_misses} / ${s.steps}</td><td>${s.mode==='ilqr'?(s.ilqr_plan.converged?'0 planos':'1 plano'):s.nonconverged_solves}</td></tr>`;}).join('');
 document.querySelectorAll('[data-select]').forEach(button=>button.onclick=()=>select(button.dataset.select));
 pairReading();
}

function select(key){
 selected=key;$('method').value=key;simTime=0;
 const run=data.runs[key],s=run.summary,assets=run.assets;
 $('method-description').textContent=descriptions[key];
 $('parameters').innerHTML=`<dt>Planejador</dt><dd>RRT-Connect</dd><dt>Semente</dt><dd>${data.common.seed}</dd><dt>Período</dt><dd>${fmt(data.common.timestep*1000,0)} ms</dd><dt>Discretização linear</dt><dd>${s.discretization==='zoh'?'Retenção de ordem zero':'Euler'}</dd><dt>Horizonte móvel</dt><dd>${['mpc','receding','ilqr_receding'].includes(s.mode)?s.horizon+' passos':'—'}</dd><dt>Velocidade da referência</dt><dd>${fmt(data.common.speed_rad_s)} rad/s</dd><dt>Aceleração da referência</dt><dd>${fmt(data.common.acceleration_rad_s2)} rad/s²</dd>`;
 $('video').pause();$('video').hidden=!assets.video;$('no-video').hidden=!!assets.video;
 $('no-video').textContent='Esta execução possui dados e gráficos registrados. O vídeo deste caso e método ainda não foi renderizado.';
 $('download-video').hidden=!assets.video;
 if(assets.video){$('video').src=assets.video;$('video').poster=assets.poster;$('download-video').href=assets.video;}
 else{$('video').removeAttribute('src');$('video').load();}
 $('download-run').href=assets.run;
 $('download-summary').href=assets.summary||assets.run.replace('.npz','.json');
 $('video').playbackRate=Number($('speed').value);
 $('run-status').textContent=s.termination||`${reached(s)} de 3 destinos alcançados no instante previsto. ${s.nonconverged_solves} soluções online sem atingir a tolerância numérica.${s.ilqr_plan?' Plano iLQR: '+(s.ilqr_plan.converged?'convergente.':'não convergente; aplicado com realimentação.'):' '}`;
 $('run-status').classList.toggle('bad',!s.completed||reached(s)!==3);
 $('metrics').innerHTML=[['Destinos alcançados',`${reached(s)} / 3`,'Critério no instante planejado'],['Erro RMS da ponta',fmt(s.rms_tip_error_m*1000,3)+' mm','Trecho executado deste método'],['Pico de erro da ponta',fmt(s.max_tip_error_m*1000,2)+' mm','Maior desvio do trecho executado'],['Cálculo por comando',fmt(s.command_median_ms,3)+' ms','Mediana; preparação separada']].map(([label,v,note])=>`<div class="metric"><span>${label}</span><strong>${v}</strong><small>${note}</small></div>`).join('');
 $('visits').innerHTML=data.common.arrivals.slice(1).map(a=>{const v=s.visits.find(v=>v.index===a.index);return `<article class="visit"><b>${names[a.button]} <span class="${v?.reached?'good':'bad'}">${v?.reached?'✓':'×'}</span></b><p>${v?`t = ${fmt(v.time_s)} s<br>Erro: ${fmt(v.tip_error_m*1000,3)} mm<br>Velocidade máxima: ${fmt(v.speed_rad_s,4)} rad/s`:'Execução interrompida antes desta passagem.'}</p></article>`;}).join('');
 $('command').textContent=String.raw`python plan_buttons.py --case ${caseKey} \
  --sequence red yellow green red \
  --mode ${s.mode}${s.mode==='mpc'?' --solver '+s.solver:''} --horizon ${s.horizon} \
  --seed 6 --headless --ilqr-max-iterations 5 \
  --output output/${caseKey}-${key} \
  --video output/${caseKey}-${key}.mp4`;
 $('config-dump').textContent=JSON.stringify(s,null,2);
 comparisonTable();updateClock();draw();
 try{const query=new URLSearchParams({case:caseKey,method:key});history.replaceState(null,'','?'+query+location.hash);}catch{}
}
function updateClock(){if(!data)return;simTime=Math.max(0,Math.min(data.runs[selected].summary.simulated_seconds,($('video').currentTime||0)-1));$('video-status').textContent=`Tempo simulado: ${fmt(simTime)} / ${fmt(data.runs[selected].summary.simulated_seconds)} s`;}

async function selectCase(key){
 $('case').disabled=true;$('compare-case').disabled=true;$('case-loading').textContent='Carregando resultados…';
 try{
  if(!window.ROUTE_CASE_DATA[key])await new Promise((resolve,reject)=>{const script=document.createElement('script');script.src=catalog[key].data;script.onload=resolve;script.onerror=()=>{script.remove();reject(new Error('Falha ao carregar os dados. Tente novamente.'));};document.head.append(script);});
  data=window.ROUTE_CASE_DATA[key];caseKey=key;keys=Object.keys(data.runs);
  if(!keys.includes(selected))selected=keys[0];
  $('case').value=key;$('compare-case').value=key;$('case-description').textContent=data.common.experiment.description;$('case-focus').textContent=data.common.experiment.focus;
  for(const id of ['method','compare-a','compare-b'])$(id).innerHTML=keys.map(k=>`<option value="${k}">${label(k)}</option>`).join('');
  $('compare-a').value=keys.includes('infinite')?'infinite':keys[0];$('compare-b').value=keys.includes('mpc_sqp')?'mpc_sqp':keys.at(-1);
  $('compare-methods').innerHTML=keys.map(k=>`<label class="method-choice" style="--method-color:${palette[k]}"><input type="checkbox" value="${k}" ${visible.has(k)?'checked':''}><span class="method-dot"></span>${label(k)}</label>`).join('');
  document.querySelectorAll('#compare-methods input').forEach(input=>input.onchange=()=>{input.checked?visible.add(input.value):visible.delete(input.value);drawComparisons();});
  $('window').querySelector('[value="event"]').disabled=!pulse();$('window').value=pulse()?'event':'full';
  $('planning-table').innerHTML=data.common.planning.map(p=>`<tr><td>${names[p.source]} → ${names[p.destination]}</td><td class="${p.direct_valid?'good':'bad'}">${p.direct_valid?'Livre':'Bloqueada'}</td><td>${p.nodes}</td><td>${p.raw_waypoints} / ${p.waypoints}</td><td>${fmt(p.length_rad)} rad</td><td>${fmt(p.seconds)} s</td></tr>`).join('');
  $('shared-preparation').textContent=`Preparação comum: ${fmt(data.common.planning.reduce((sum,p)=>sum+p.seconds,0))} s no planejador e ${fmt(data.common.nominal_preparation_seconds)} s para dinâmica inversa, linearizações e ganhos. Esses cálculos precedem a execução e são reutilizados pelos métodos do caso.`;
  $('download-data').href=catalog[key].folder+'data.json';$('download-route').href=catalog[key].folder+'route.json';
  findings();select(selected);$('case-loading').textContent='';
 }catch(error){$('case-loading').textContent=error.message;$('case').value=caseKey;$('compare-case').value=caseKey;}
 finally{$('case').disabled=false;$('compare-case').disabled=false;}
}

Object.entries(catalog).forEach(([key,c])=>{const option=document.createElement('option');option.value=key;option.textContent=c.label;$('case').append(option);});
$('compare-case').innerHTML=$('case').innerHTML;
$('compare-case').onchange=()=>selectCase($('compare-case').value);
$('case').onchange=()=>selectCase($('case').value);
$('method').onchange=()=>select($('method').value);
$('joint').onchange=draw;$('unit').onchange=draw;$('compare-joint').onchange=drawComparisons;$('window').onchange=drawComparisons;
$('compare-a').onchange=pairReading;$('compare-b').onchange=pairReading;$('sort').onchange=comparisonTable;
$('show-all').onclick=()=>{keys.forEach(k=>visible.add(k));document.querySelectorAll('#compare-methods input').forEach(x=>x.checked=true);drawComparisons();};
$('show-pair').onclick=()=>{visible.clear();visible.add($('compare-a').value);visible.add($('compare-b').value);document.querySelectorAll('#compare-methods input').forEach(x=>x.checked=visible.has(x.value));drawComparisons();};
$('details-plots').ontoggle=draw;
$('speed').onchange=()=>{$('video').playbackRate=Number($('speed').value);};
$('video').ontimeupdate=()=>{updateClock();draw();};
$('video').onloadedmetadata=()=>{$('video').playbackRate=Number($('speed').value);};
$('copy-command').onclick=async()=>{try{await navigator.clipboard.writeText($('command').textContent);$('copy-command').textContent='Comando copiado ✓';}catch{$('copy-command').textContent='Selecione e copie o comando acima';}};
window.addEventListener('resize',draw);
selectCase(catalog[params.get('case')]?params.get('case'):'nominal');
