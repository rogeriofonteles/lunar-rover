'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const data = window.ACROBOT_DATA;
  if (!data?.scenarios?.length) {
    $('scenario-title').textContent = 'Dados ainda não gerados';
    $('scenario-subtitle').textContent = 'Execute generate.py no contêiner Drake e mantenha data.js ao lado deste HTML.';
    return;
  }
  const names = {finite:'LQR finito',infinite:'LQR infinito',ilqr:'iLQR',mpc:'MPC não linear',mpc_lqr:'MPC · LQR/Riccati',mpc_sqp:'MPC · SQP',mpc_slsqp:'MPC · SLSQP'};
  const colors = {finite:'#bb7527',infinite:'#287a9a',ilqr:'#995eae',mpc:'#148269',mpc_lqr:'#287a9a',mpc_sqp:'#bb7527',mpc_slsqp:'#995eae'};
  const bright = {finite:'#e9b16d',infinite:'#80bfdd',ilqr:'#c8a3e1',mpc:'#7dcdb0',mpc_lqr:'#80bfdd',mpc_sqp:'#e9b16d',mpc_slsqp:'#c8a3e1'};
  let modes = ['infinite','mpc'];
  const challenges=data.scenarios.filter(s=>(s.config.torque_mode||'original')==='original');
  const wrap = q => ((q+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI;
  const fmt = (x,n=2) => Number(x).toLocaleString('pt-BR',{minimumFractionDigits:n,maximumFractionDigits:n});
  const ms = x => x >= 1000 ? `${fmt(x/1000)} s` : `${fmt(x,x<1?3:1)} ms`;
  const escape = x => String(x).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const textStatus = s => ({'maximum iterations reached':'Limite de iterações','relative cost tolerance':'Tolerância relativa de custo','stationary control update':'Atualização estacionária','line search failed':'Line search falhou','regularization limit reached':'Limite de regularização','final backward pass failed':'Backward pass final falhou'}[s] || s);
  const seedName = s => ({zero:'Torque zero',lqr:'LQR local',collocation:'Direct collocation',provided:'Warm start (plano deslocado)',clipped_lqr:'Rollout LQR com clipping',paired_initial_guess:'Initial guess pareado'}[s] || s);
  const technicalText = text => text.replaceAll('semente local', 'initial guess local').replaceAll('amortecimento numérico', 'damping numérico');
  const notes = {
    nonlinear:'<strong>Uma perturbação inicial intermediária.</strong> Apenas o estado inicial muda em relação a 01: desvio de 0,30 rad (17,19°). Observe se o plano não linear com sua inicialização automática consegue estabilizar onde o ganho linear falha. Compare o torque e o WallTime do primeiro solve.',
    near:'<strong>Comece aqui.</strong> Um desvio de 0,03 rad (1,72°) em cada junta testa a região local. Compare a qualidade da resposta com o WallTime de cada comando.',
    far:'<strong>Agora o desvio é de 1 rad (57,30°).</strong> Os custos e o limite de torque são os mesmos do desafio 01. O initial guess automático pode recorrer a direct collocation; o diagnóstico do solver informa qual foi escolhido e se houve fallback.',
    saturation:'<strong>Um teste de viabilidade física.</strong> Na opção de torque original, o limite cai de ±15 para ±3 N·m em relação ao desafio 02. Com o seletor global em ±3 N·m ou sem clipping, os desafios 02 e 03 têm a mesma configuração física. O LQR aplica clipping à ação depois do projeto; iLQR e MPC consideram esse limite na busca. Nenhum deles cria torque que o atuador não tem.',
    disturbance:'<strong>Uma surpresa em t = 2 s.</strong> As velocidades recebem Δω₁ = +1,5 e Δω₂ = −2 rad/s. O plano único do iLQR tem feedback, mas não é reotimizado. O MPC usa o novo estado na próxima decisão.',
    short:'<strong>Olhar pouco à frente pode ser insuficiente.</strong> Só o MPC muda: horizonte de 0,20 s e custo terminal Q. Ambos mudam juntos neste cenário; não atribua o resultado apenas ao horizonte. Compare com 01, que usa 2 s e terminal DARE.',
    expensive:'<strong>O esforço ficou mil vezes mais caro.</strong> Apenas R muda de 0,1 para 100 em relação ao desafio 01. Observe os picos de torque, o tempo de estabilização e a velocidade residual; R maior não implica automaticamente falha.',
    budget:'<strong>Orçamento mínimo, initial guess local.</strong> iLQR e MPC recebem só uma iteração e não usam direct collocation. Compare com 02 sabendo que orçamento e inicialização mudam juntos. O LQR permanece igual.',
    damping:'<strong>Um passo numérico excessivamente cauteloso.</strong> λ inicial = 10⁶, initial guess local e até 12 iterações no primeiro plano (2 nas reotimizações). Observe a variação do custo; uma mudança pequena não certifica um bom movimento.',
    terminal:'<strong>Uma comparação controlada do custo terminal.</strong> Em t = 5 s, Δω = [+0,4; −0,6] rad/s. O LQR finito usa Qf = P∞ do seu próprio modelo Euler. Compare com 10: os outros parâmetros são iguais.',
    late:'<strong>O plano tem só mais um segundo.</strong> A mesma perturbação do desafio 09, com Qf = Q no LQR finito. Compare os dois para avaliar quanto o projeto se preocupa com o equilíbrio depois de t = 6 s.'
  };
  let selected=0, time=0, playing=false, lastFrame=0, active=new Set(modes), charts=new Map(), hoveredPlot=null;
  let scenario=challenges[0];
  const initial=new URLSearchParams(location.search).get('cenario');
  if(initial) selected=Math.max(0,challenges.findIndex(s=>s.config.id===initial));
  const list=$('scenario-list');
  challenges.forEach((s,i)=>{
    const b=document.createElement('button');b.innerHTML=`${escape(s.config.title)}<span>${escape(technicalText(s.config.subtitle))}</span>`;
    b.onclick=()=>selectScenario(i);list.appendChild(b);
  });
  function renderPanels(){
    const family=$('comparison-mode').value;
    modes=family==='focus'?['infinite','mpc']:family==='all'?['finite','infinite','ilqr','mpc']:['mpc_lqr','mpc_sqp',...(data.solver_comparison?.show_slsqp?['mpc_slsqp']:[]),'mpc'];
    active=new Set(modes);
    $('robots').setAttribute('data-count',String(modes.length));
    $('legend').innerHTML=modes.map(m=>`<label style="--color:${colors[m]}"><input type="checkbox" data-mode="${m}" checked><i></i>${names[m]}</label>`).join('');
    $('robots').innerHTML=modes.map(m=>`<div class="robot-panel" style="--color:${bright[m]}"><div class="robot-name"><i></i>${names[m]}</div><canvas id="robot-${m}" role="img" aria-label="Animação ${names[m]}"></canvas><div class="robot-state" id="state-${m}"></div><div class="robot-badge" id="badge-${m}"></div></div>`).join('');
    const optimizers=modes.filter(m=>m==='ilqr'||m.startsWith('mpc'));
    $('solver-select').innerHTML=optimizers.map(m=>`<option value="${m}">${names[m]}</option>`).join('');
    $('solver-select').value=optimizers.includes('mpc')?'mpc':optimizers[0];
    $('solver-family-note').hidden=family!=='solvers';
    $('video-context').textContent=family==='solvers'?'Vídeo da comparação de MPC.':'O vídeo preserva os quatro controladores, mesmo na visão principal com dois.';
  }
  $('legend').addEventListener('change',e=>{const m=e.target.dataset.mode;if(!m)return;e.target.checked?active.add(m):active.delete(m);buildCharts();render();});


  function stateAt(r,t){
    const f=Math.max(0,Math.min(r.states.length-1,t/r.cfg.dt));const i=Math.floor(f), j=Math.min(i+1,r.states.length-1),a=f-i;
    // A velocity jump is an event, not a gradual impulse between samples.
    const kick=r.cfg.kick;if(kick && i<j && Math.abs(j*r.cfg.dt-kick.time)<1e-8 && t<kick.time) {
      return r.states[i].map((v,k)=>v*(1-a)+(r.states[j][k]-kick.delta[k])*a);
    }
    return r.states[i].map((v,k)=>v*(1-a)+r.states[j][k]*a);
  }
  function ctxFor(id){const canvas=$(id), rect=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);const w=rect.width,h=rect.height;if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);return {c,w,h};}
  function joints(x,p){const a=[p.l1*Math.sin(x[0]),-p.l1*Math.cos(x[0])];return [[0,0],a,[a[0]+p.l2*Math.sin(x[0]+x[1]),a[1]-p.l2*Math.cos(x[0]+x[1])]];}
  function drawRobot(m){const r=scenario.runs[m],{c,w,h}=ctxFor(`robot-${m}`);if(!r){c.fillStyle='#bbd0c0';c.fillText('Execução indisponível',10,40);return;}
    const total=r.parameters.l1+r.parameters.l2,scale=Math.min((w-20)/(2*total+.6),(h-25)/(2*total+.6));const ox=w/2,oy=h/2;
    const xy=p=>[ox+p[0]*scale,oy-p[1]*scale];
    c.strokeStyle='#263f35';c.lineWidth=.6;for(let k=-3;k<=3;k++){c.beginPath();c.moveTo(ox+k*scale,10);c.lineTo(ox+k*scale,h-10);c.stroke();c.beginPath();c.moveTo(5,oy+k*scale);c.lineTo(w-5,oy+k*scale);c.stroke();}
    function pose(x,color,width=5,dash=[]){const points=joints(x,r.parameters).map(xy);c.beginPath();c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);c.lineCap='round';points.forEach((p,i)=>i?c.lineTo(...p):c.moveTo(...p));c.stroke();c.setLineDash([]);return points;}
    pose([Math.PI,0,0,0],'#4d7762',3,[3,5]);
    if($('trail').checked){const end=Math.floor(time/r.cfg.dt);c.beginPath();c.strokeStyle=bright[m]+'66';c.lineWidth=1;for(let k=Math.max(0,end-45);k<=end;k++){const p=xy(joints(r.states[k],r.parameters)[2]);k===Math.max(0,end-45)?c.moveTo(...p):c.lineTo(...p);}c.stroke();}
    const i=Math.min(r.states.length-1,Math.round(time/r.cfg.dt));
    if(m==='ilqr'&&r.nominal&&$('nominal').checked)pose(r.nominal[i],'#c8a3e166',2,[4,3]);
    if(m.startsWith('mpc')&&$('predictions').checked&&r.predictions.length){const pred=[...r.predictions].reverse().find(p=>p.step<=i)||r.predictions[0];c.beginPath();c.strokeStyle='#72c5a87a';c.lineWidth=1.5;c.setLineDash([3,3]);pred.states.forEach((x,k)=>{const p=xy(joints(x,r.parameters)[2]);k?c.lineTo(...p):c.moveTo(...p);});c.stroke();c.setLineDash([]);}
    const x=stateAt(r,time),ps=pose(x,bright[m],7);pose(x,'#142b27',1.5);
    ps.forEach((p,k)=>{c.beginPath();c.arc(...p,k===1?6:4,0,2*Math.PI);c.fillStyle=k===1?bright[m]:'#e3e8d9';c.fill();c.strokeStyle='#12261f';c.lineWidth=2;c.stroke();});
    c.strokeStyle='#829b8b';c.lineWidth=3;c.beginPath();c.moveTo(ox-12,oy+8);c.lineTo(ox+12,oy+8);c.stroke();
    const ki=Math.min(r.torque.length-1,Math.floor(time/r.cfg.dt));
    $(`state-${m}`).textContent=`u = ${fmt(r.torque[ki])} N·m`;
    const inside=Math.max(Math.abs(wrap(x[0]-Math.PI)),Math.abs(wrap(x[1])))<.05&&Math.max(Math.abs(x[2]),Math.abs(x[3]))<.1;
    $(`badge-${m}`).textContent=inside?'● Dentro da tolerância agora':'○ Fora da tolerância agora';$(`badge-${m}`).style.color=inside?'#8ed2ad':'#b5c8bc';
  }

  function normalized(x){return Math.max(Math.abs(wrap(x[0]-Math.PI))/.05,Math.abs(wrap(x[1]))/.05,Math.abs(x[2])/.1,Math.abs(x[3])/.1);}
  function buildCharts(){
    charts.clear();const am=$('angle-mode').value,deg=$('unit').value==='deg',fac=deg?180/Math.PI:1,unit=deg?'°':'rad';
    $('angle1-title').textContent=am==='error'?'Erro do ombro':am==='joint'?'Posição da junta 1':'Orientação absoluta do link 1';
    $('angle2-title').textContent=am==='error'?'Erro do cotovelo':am==='joint'?'Posição da junta 2':'Orientação absoluta do link 2';
    $('speed2-title').textContent=am==='absolute'?'Velocidade absoluta do link 2':'Velocidade do cotovelo';
    const defs={
      q1:{unit,fn:x=>fac*(am==='error'?wrap(x[0]-Math.PI):x[0]),ref:am==='error'?0:Math.PI*fac},
      q2:{unit,fn:x=>fac*(am==='error'?wrap(x[1]):am==='absolute'?x[0]+x[1]:x[1]),ref:am==='absolute'?Math.PI*fac:0},
      v1:{unit:unit+'/s',fn:x=>x[2]*fac,ref:0},v2:{unit:unit+'/s',fn:x=>(am==='absolute'?x[2]+x[3]:x[3])*fac,ref:0},
      torque:{unit:'N·m',key:'torque',bounds:scenario.config.limit===null?null:[-scenario.config.limit,scenario.config.limit],step:true},
      error:{unit:'erro normalizado',fn:normalized,log:true,ref:1},
      timing:{unit:'WallTime (ms)',key:'command_ms',log:true,ref:scenario.config.dt*1000},
      residual:{unit:'norma mista',key:'linear_residual',log:true}
    };
    for(const [id,def] of Object.entries(defs)){
      const series=modes.filter(m=>active.has(m)&&scenario.runs[m]).map(m=>{const r=scenario.runs[m];return {name:names[m],color:colors[m],values:def.key?r[def.key]:r.states.map(def.fn),x:r.time};});
      charts.set(id,{...def,series,maxX:scenario.config.duration});
    }
  }
  function tick(v){const a=Math.abs(v);return a>=10000?v.toExponential(0):a>=10?fmt(v,0):a>=1?fmt(v,1):a>=.01?fmt(v,2):a===0?'0':v.toExponential(0);}
  function drawChart(id,opt,cursor=true){
    const {c,w,h}=ctxFor(id),left=58,right=14,top=25,bottom=30,pw=w-left-right,ph=h-top-bottom;if(pw<10)return;
    const tr=v=>opt.log?Math.log10(Math.max(1e-6,v)):v;const vals=opt.series.flatMap(s=>s.values).filter(Number.isFinite);
    if(opt.ref!==undefined)vals.push(opt.ref);if(opt.bounds)vals.push(...opt.bounds);
    let low=vals.length?Math.min(...vals.map(tr)):0,high=vals.length?Math.max(...vals.map(tr)):1;
    if(opt.log){low=Math.max(-6,Math.floor(low));high=Math.ceil(high);if(low===high)high=low+1;}
    else{const padding=Math.max((high-low)*.09,.01);low-=padding;high+=padding;}
    const x=v=>left+v/Math.max(opt.maxX,1)*pw,y=v=>top+ph-(tr(v)-low)/(high-low)*ph;
    c.font='10px Arial';c.fillStyle='#718075';c.textAlign='left';c.fillText(opt.unit,left,12);
    for(let j=0;j<=4;j++){const p=top+j*ph/4,val=high-j*(high-low)/4;c.strokeStyle='#e1e5da';c.lineWidth=.8;c.beginPath();c.moveTo(left,p);c.lineTo(w-right,p);c.stroke();c.textAlign='right';c.fillText(tick(opt.log?10**val:val),left-8,p+3);}
    for(let j=0;j<=4;j++){const v=opt.maxX*j/4;c.textAlign='center';c.fillText(tick(v)+(j===4&&cursor?' s':''),x(v),h-9);}
    c.save();c.beginPath();c.rect(left,top,pw,ph);c.clip();
    function line(value,color){c.strokeStyle=color;c.lineWidth=1;c.setLineDash([4,4]);c.beginPath();c.moveTo(left,y(value));c.lineTo(w-right,y(value));c.stroke();c.setLineDash([]);}
    if(opt.ref!==undefined)line(opt.ref,'#879b73');if(opt.bounds)opt.bounds.forEach(v=>line(v,'#b9a38b'));
    if(cursor&&scenario.config.kick){c.strokeStyle='#b96449';c.setLineDash([3,4]);c.beginPath();c.moveTo(x(scenario.config.kick.time),top);c.lineTo(x(scenario.config.kick.time),top+ph);c.stroke();c.setLineDash([]);}
    for(const s of opt.series){c.strokeStyle=s.color;c.lineWidth=1.8;c.beginPath();s.values.forEach((v,i)=>{const xx=x(s.x[i]??i), yy=y(v);if(i===0)c.moveTo(xx,yy);else{if(opt.step)c.lineTo(xx,y(s.values[i-1]));c.lineTo(xx,yy);}});c.stroke();}
    if(cursor){c.strokeStyle='#2b473c99';c.lineWidth=1;c.beginPath();c.moveTo(x(time),top);c.lineTo(x(time),top+ph);c.stroke();for(const s of opt.series){const i=Math.min(s.values.length-1,Math.round(time/scenario.config.dt));c.beginPath();c.arc(x(s.x[i]),y(s.values[i]),3,0,2*Math.PI);c.fillStyle=s.color;c.fill();}}
    c.restore();
    if(cursor && hoveredPlot===id && opt.series.length){
      const bw=Math.min(180,pw-10),bh=24+opt.series.length*17,bx=w-right-bw-5,by=top+5;
      c.fillStyle='#f8faf0ee';c.fillRect(bx,by,bw,bh);c.strokeStyle='#d1d8c8';c.strokeRect(bx,by,bw,bh);
      c.textAlign='left';c.fillStyle='#42594b';c.fillText(`t = ${fmt(time)} s`,bx+8,by+14);
      opt.series.forEach((s,j)=>{const i=Math.min(s.values.length-1,Math.round(time/scenario.config.dt));c.fillStyle=s.color;c.fillText(`${s.name}: ${tick(s.values[i])}`,bx+8,by+32+j*17);});
    }
    $(id).setAttribute('aria-label', `${opt.unit}. ${opt.series.map(s=>s.name+': '+tick(s.values[Math.min(s.values.length-1,Math.round(time/scenario.config.dt))])).join('; ')}`);
    $(id).dataset.left=left;$(id).dataset.width=pw;
    if(!opt.series.length){c.fillStyle='#6a7e70';c.textAlign='center';c.fillText('Selecione um método na legenda',w/2,h/2);}
  }
  function render(){
    $('clock').textContent=`${fmt(time)} / ${fmt(scenario.config.duration)} s`;$('timeline').value=time;
    const kick=scenario.config.kick;$('event-indicator').textContent=kick?`Perturbação em ${fmt(kick.time,0)} s${time>=kick.time?' · já aplicada':''}`:'Torque aplicado no cotovelo';
    const fast=modes.some(m=>scenario.runs[m]&&scenario.runs[m].states.some(x=>Math.max(Math.abs(x[2]),Math.abs(x[2]+x[3]))*scenario.config.dt>Math.PI/2));
    $('replay-warning').textContent=fast?'Movimento muito rápido: o replay a partir de amostras de 50 ms pode apresentar aliasing. Use os gráficos e os picos de velocidade para interpretar a execução.':'';
    modes.forEach(drawRobot);charts.forEach((opt,id)=>drawChart(id,opt));
    $('readout').innerHTML=modes.filter(m=>scenario.runs[m]).map(m=>{const x=stateAt(scenario.runs[m],time);return `<span><b style="color:${colors[m]}">${names[m]}</b>: θ = [${fmt(x[0])}; ${fmt(x[1])}] rad · ω = [${fmt(x[2])}; ${fmt(x[3])}] rad/s</span>`;}).join('');
  }
  function renderSolve(){
    const m=$('solver-select').value,r=scenario.runs[m];$('solve-index').disabled=(r?.solves?.length||0)<=1;
    if(!r?.solves?.length){$('solver-detail').textContent='Sem solve registrado.';return;}
    $('solve-index').max=r.solves.length-1;const index=Math.min(+$('solve-index').value,r.solves.length-1);const s=r.solves[index];
    $('solve-time').textContent=`t = ${fmt(s.time)} s`;
    drawChart('cost',{unit:'custo J (log)',series:[{color:colors[m],values:s.costs,x:s.costs.map((_,i)=>i)}],log:true,maxX:Math.max(1,s.costs.length-1)},false);
    $('cost-caption').textContent=s.history_kind==='iterates'?'Histórico dos iterados do QP. SLSQP pode explorar iterados intermediários inviáveis; verifique o resíduo KKT e a viabilidade do plano retornado. Riccati tem solução analítica, sem iterações.':'Custo do problema de otimização, incluindo o terminal. O eixo conta atualizações aceitas, não todas as tentativas; valores absolutos entre horizontes ou pesos diferentes não são comparáveis.';
    const improvement=s.costs[0]>0?(1-s.costs.at(-1)/s.costs[0])*100:0;
    const converged=r.solves.filter(s=>s.converged).length;
    $('solver-detail').innerHTML=`<span class="status ${s.converged?'good':'bad'}">${m==='mpc_lqr'?'Solução LQ analítica (sem restrições)':s.converged?'Critério numérico satisfeito':'Sem convergência declarada'}</span><dl><dt>Parada</dt><dd>${escape(textStatus(s.status))}</dd><dt>Iterações tentadas</dt><dd>${s.iterations}</dd><dt>${s.history_kind==='iterates'?'Iterados registrados':'Atualizações aceitas'}</dt><dd>${s.costs.length-1}</dd><dt>WallTime deste solve</dt><dd>${ms(s.ms)}</dd><dt>Redução de custo</dt><dd>${fmt(improvement)}%</dd><dt>Initial guess escolhido</dt><dd>${escape(seedName(s.initialization))}</dd>${s.residual!==null&&s.residual!==undefined?`<dt>Resíduo KKT normalizado</dt><dd>${s.residual.toExponential(2)}</dd>`:''}${s.bound_violation!==null&&s.bound_violation!==undefined?`<dt>Violação prevista de torque</dt><dd>${fmt(s.bound_violation,4)} N·m</dd>`:''}<dt>Solves convergidos / total</dt><dd>${converged} / ${r.solves.length}</dd></dl><p>${r.metrics.settling_s===null?'A execução não cumpriu o critério físico de estabilização nos 6 s observados.':`A execução cumpriu o critério físico a partir de ${fmt(r.metrics.settling_s)} s.`}</p>${r.warnings?.length?`<details><summary>Avisos da inicialização</summary><p>Houve fallback para initial guesses locais. Mensagem original do Drake:</p><pre>${escape(r.warnings.join('\n'))}</pre></details>`:''}`;
  }
  function summary(){
    $('metrics').innerHTML=modes.filter(m=>scenario.runs[m]).map(m=>{const r=scenario.runs[m],v=r.metrics;return `<tr><td style="color:${colors[m]}">${names[m]}</td><td>${ms(v.first_solution_ms)}</td><td>${ms(v.online_median_ms)} / ${ms(v.online_p95_ms)}</td><td class="${v.deadline_misses?'bad':'good'}">${v.deadline_misses} / ${v.online_samples}</td><td>${v.settling_s===null?'Não atingida':fmt(v.settling_s)+' s'}</td><td>${fmt(v.final_angle_rad,4)} rad<span class="metric-note">${fmt(v.final_speed_rad_s,4)} rad/s</span></td><td>${fmt(v.peak_torque)} N·m<span class="metric-note">${r.cfg.limit===null?'Sem clipping':fmt(v.saturation_pct,1)+'% das amostras saturadas'}</span></td></tr>`;}).join('');
    const runs=modes.filter(m=>scenario.runs[m]);const successes=runs.filter(m=>scenario.runs[m].metrics.settling_s!==null);
    const mpc=scenario.runs.mpc,ilqr=modes.includes('ilqr')?scenario.runs.ilqr:null;
    const final=runs.map(m=>({m,value:scenario.runs[m].metrics.final_angle_rad})).sort((a,b)=>a.value-b.value);
    const cards=[['Estabilização física',`${successes.length} / ${runs.length}`,successes.length?`${successes.map(m=>names[m]).join(', ')} cumpriram as tolerâncias por pelo menos 1 s até o fim.`:'Nenhum método manteve as tolerâncias por pelo menos 1 s até o fim. Isso descreve esta execução, não uma impossibilidade global.'],['Menor erro angular final',`${fmt(final[0].value,4)} rad`,`${names[final[0].m]} terminou com o menor erro angular máximo. Verifique também a velocidade final; posição sozinha não comprova equilíbrio.`]];
    if(mpc)cards.push(['WallTime de replanejamento',ms(mpc.metrics.online_median_ms),`${mpc.metrics.deadline_misses} de ${mpc.metrics.online_samples} reotimizações após o primeiro solve excederam 50 ms. O simulador aguardou cada cálculo.`]);
    if(ilqr)cards.push(['Otimização do plano único',ilqr.solves[0].converged?'Critério atingido':'Orçamento / parada',`${textStatus(ilqr.solves[0].status)} após ${ilqr.solves[0].iterations} iterações. ${ilqr.metrics.settling_s===null?'O robô não cumpriu o critério físico de estabilização.':'O robô também cumpriu o critério físico.'}`]);
    renderTorqueImpact();
    $('findings').innerHTML=cards.map(c=>`<article class="finding"><h4>${c[0]}</h4><span class="number">${c[1]}</span><p>${c[2]}</p></article>`).join('');
  }
  function renderTorqueImpact(){
    const id=scenario.config.challenge_id||scenario.config.id;
    const find=v=>data.scenarios.find(s=>(s.config.challenge_id||s.config.id)===id&&s.config.torque_mode===v);
    const clipped=find('clipped'),free=find('unclipped');
    const result=r=>r?(r.metrics.settling_s===null?'Não atingida':fmt(r.metrics.settling_s)+' s'):'Ainda não calculado';
    $('torque-impact').innerHTML=modes.map(m=>{const a=clipped?.runs[m],b=free?.runs[m];return `<tr><td>${names[m]}</td><td>${result(a)}</td><td>${result(b)}</td><td>${a?fmt(a.metrics.angle_rms_rad,3):'—'} / ${b?fmt(b.metrics.angle_rms_rad,3):'—'} rad</td><td>${b?fmt(b.metrics.peak_torque)+' N·m':'—'}</td></tr>`;}).join('');
  }
  function renderSolverComparison(){
    const b=data.solver_comparison;
    if(!b){$('solver-comparison').textContent='Comparação pareada dos solvers ainda não disponível.';return;}
    $('solver-comparison').innerHTML=`<p><strong>${b.show_slsqp?'SQP e SLSQP aparecem separadamente: houve diferença relevante neste benchmark.':'SQP e SLSQP foram agrupados: não houve diferença relevante neste benchmark.'}</strong> ${escape(b.conclusion)}</p><p>Critério definido antes da comparação: diferença de torque inicial > 0,1 N·m; discrepância de convergência em ≥ 5% dos problemas; ou razão de medianas de WallTime ≥ 3× com diferença absoluta ≥ 1 ms. Mesmo modelo, custos, limites, horizonte e initial guess em cada par; ${b.repetitions} repetições com ordem alternada.</p><div class="table-wrap"><table><thead><tr><th>Casos com clipping</th><th>SQP · mediana</th><th>SLSQP · mediana</th><th>Maior Δu₀</th><th>Convergência diferente</th></tr></thead><tbody><tr><td>${b.pairs}</td><td>${ms(b.sqp_median_ms)}</td><td>${ms(b.slsqp_median_ms)}</td><td>${fmt(b.max_first_torque_diff,5)} N·m</td><td>${fmt(100*b.convergence_disagreement,1)}%</td></tr></tbody></table></div><p class="caption">Com torque ilimitado, ambos usam a solução LQ analítica equivalente; não há vantagem em rodar uma otimização restrita. SQP aqui usa Hessiana exata e active set para um QP convexo, não um solver NMPC genérico. SLSQP é uma implementação da família SQP. Diferenças grandes de torque devem ser lidas junto da convergência: se um solver falha na viabilidade, retorna o initial guess viável e registra esse fallback. Alguns desafios repetem o mesmo QP linear; os 66 casos não são 66 problemas independentes.</p>`;
  }
  function selectScenario(i){
    selected=i;
    const id=challenges[i].config.id,variant=$('torque-mode').value;
    scenario=data.scenarios.find(s=>(s.config.challenge_id||s.config.id)===id&&(s.config.torque_mode||'original')===variant)||challenges[i];
    renderPanels();time=0;playing=false;updatePlay();
    [...list.children].forEach((b,j)=>{b.classList.toggle('active',i===j);b.setAttribute('aria-pressed',i===j?'true':'false');});
    const cfg=scenario.config;$('scenario-title').textContent=cfg.title;$('scenario-subtitle').textContent=technicalText((cfg.challenge_id||cfg.id)==='saturation'?'Mesmo estado distante: compare o torque original com as opções do seletor global.':cfg.subtitle);$('scenario-note').innerHTML=notes[cfg.challenge_id||cfg.id]||'';
    $('parameters').innerHTML=[['Estado inicial [rad; rad/s]',`[${cfg.x0.map(v=>fmt(v,2)).join('; ')}]`],['R / Q diagonal',`${fmt(cfg.R,1)} / [${cfg.Q.join(', ')}]`],['Limite de torque',cfg.limit===null?'Sem clipping':`±${fmt(cfg.limit,0)} N·m`],['Horizonte MPC',`${fmt(cfg.mpc_horizon,2)} s`],['LQR finito · Qf',cfg.terminal],['iLQR/MPC · iterações',`${cfg.iterations} / ${cfg.mpc_iterations} online`],['Regularização',cfg.regularization.toExponential(0)]].map(([label,val])=>`<div class="parameter"><span>${label}</span>${val}</div>`).join('');
    $('timeline').max=cfg.duration;$('solve-index').value=0;
    const videoKey=cfg.id+($('comparison-mode').value==='solvers'?'_solvers':'');
    const videoURL=window.ACROBOT_MEDIA?.[videoKey]||`videos/${videoKey}.mp4`;
    $('video').src=videoURL;$('download-video').href=videoURL;$('download-video').download=`acrobot-${videoKey}.mp4`;
    $('config-dump').textContent=JSON.stringify({cenario:cfg,parametros_fisicos:Object.values(scenario.runs)[0]?.parameters,metadados:data.metadata},null,2);
    summary();buildCharts();render();renderSolve();
    $('tour-next').textContent=i===challenges.length-1?'Voltar ao primeiro →':'Próximo desafio →';
    try{const url=new URL(location.href);url.searchParams.set('cenario',cfg.challenge_id||cfg.id);url.searchParams.set('torque',cfg.torque_mode||'original');url.searchParams.set('comparacao',$('comparison-mode').value);history.replaceState(null,'',url);}catch{}
  }
  function updatePlay(){$('play').textContent=playing?'Ⅱ':'▶';$('play').setAttribute('aria-label',playing?'Pausar':'Reproduzir');}
  $('play').onclick=()=>{if(time>=scenario.config.duration)time=0;playing=!playing;lastFrame=performance.now();updatePlay();};
  $('restart').onclick=()=>{time=0;playing=false;updatePlay();render();};
  $('timeline').oninput=e=>{time=+e.target.value;render();};
  $('tour-next').onclick=()=>selectScenario((selected+1)%challenges.length);
  ['torque-mode','comparison-mode'].forEach(id=>$(id).onchange=()=>selectScenario(selected));
  ['trail','nominal','predictions'].forEach(id=>$(id).onchange=render);
  ['angle-mode','unit'].forEach(id=>$(id).onchange=()=>{buildCharts();render();});
  $('solver-select').onchange=()=>{$('solve-index').value=0;renderSolve();};$('solve-index').oninput=renderSolve;
  for(const id of ['q1','q2','v1','v2','torque','error','timing','residual']){
    $(id).addEventListener('pointerleave',()=>{hoveredPlot=null;render();});
    $(id).addEventListener('pointermove',e=>{hoveredPlot=id;if(e.pointerType==='touch'&&e.buttons===0)return;const rect=e.currentTarget.getBoundingClientRect();time=Math.max(0,Math.min(scenario.config.duration,(e.clientX-rect.left-Number(e.currentTarget.dataset.left))/Number(e.currentTarget.dataset.width)*scenario.config.duration));playing=false;updatePlay();render();});
    $(id).addEventListener('pointerdown',e=>{hoveredPlot=id;const rect=e.currentTarget.getBoundingClientRect();time=Math.max(0,Math.min(scenario.config.duration,(e.clientX-rect.left-Number(e.currentTarget.dataset.left))/Number(e.currentTarget.dataset.width)*scenario.config.duration));playing=false;updatePlay();render();});
  }
  $('csv').onclick=()=>{
    const rows=['cenario,metodo,tempo_s,theta1_rad,theta2_rad,omega1_rad_s,omega2_rad_s,torque_Nm,tempo_comando_ms'];
    modes.forEach(m=>{const r=scenario.runs[m];if(r)r.states.forEach((x,i)=>rows.push([scenario.config.id,m,r.time[i],...x,r.torque[i]??'',r.command_ms[i]??''].join(',')));});
    const a=document.createElement('a'),url=URL.createObjectURL(new Blob([rows.join('\n')],{type:'text/csv;charset=utf-8'}));a.href=url;a.download=`acrobot-${scenario.config.id}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
  $('download-json').href=URL.createObjectURL(new Blob([JSON.stringify(data)],{type:'application/json'}));
  $('download-json').download='acrobot-dados.json';
  $('provenance').textContent=`Drake ${data.metadata.drake} · Python ${data.metadata.python} · gerado em ${new Date(data.metadata.generated_at).toLocaleString('pt-BR')}. Os hashes dos controladores estão no JSON.`;
  let resize;window.addEventListener('resize',()=>{clearTimeout(resize);resize=setTimeout(()=>{render();renderSolve();},100);});
  document.addEventListener('visibilitychange',()=>{if(document.hidden){playing=false;updatePlay();}});
  function frame(now){if(playing){time=Math.min(scenario.config.duration,time+(now-lastFrame)/1000*Number($('speed').value));render();if(time>=scenario.config.duration){playing=false;updatePlay();}}lastFrame=now;requestAnimationFrame(frame);}
  const query=new URLSearchParams(location.search);
  $('torque-mode').value=['original','clipped','unclipped'].includes(query.get('torque'))?query.get('torque'):'clipped';
  $('comparison-mode').value=['focus','all','solvers'].includes(query.get('comparacao'))?query.get('comparacao'):'focus';
  renderSolverComparison();selectScenario(selected);requestAnimationFrame(frame);
})();
