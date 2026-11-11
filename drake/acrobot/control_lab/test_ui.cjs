/* Source-level interaction smoke test with a minimal DOM/canvas double.
   This does not replace browser layout or media playback QA. */
const fs=require('node:fs');const vm=require('node:vm');const assert=require('node:assert/strict');const path=require('node:path');
const standalone=process.argv.includes('--standalone');
const root=__dirname,html=fs.readFileSync(path.join(root,standalone?'acrobot_interativo.html':'index.html'),'utf8');
const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);assert.equal(new Set(ids).size,ids.length,'duplicate HTML IDs');
const elements=new Map();const context2d=new Proxy({},{get:(obj,key)=>key in obj?obj[key]:(()=>{}),set:(obj,key,val)=>(obj[key]=val,true)});
class Element{
 constructor(id=''){this.id=id;this.dataset={};this.children=[];this.checked=true;this.value='';this.max='';this.width=0;this.height=0;this.listeners={};this.style={};this.classList={toggle:()=>{}};this.textContent='';this.html='';}
 set innerHTML(v){this.html=v;for(const match of v.matchAll(/\bid="([^"]+)"/g))if(!elements.has(match[1]))elements.set(match[1],new Element(match[1]));}
 get innerHTML(){return this.html;}
 appendChild(e){this.children.push(e);}
 setAttribute(k,v){this[k]=v;}
 getBoundingClientRect(){return {width:500,height:225,left:0};}
 getContext(){return context2d;}
 addEventListener(k,f){this.listeners[k]=f;}
 click(){this.onclick?.();}
}
ids.forEach(id=>elements.set(id,new Element(id)));const el=id=>{assert(elements.has(id),`Unknown element ${id}`);return elements.get(id);};
Object.entries({'speed':'1','angle-mode':'error','unit':'rad','solver-select':'ilqr','solve-index':'0','timeline':'0'}).forEach(([id,v])=>el(id).value=v);
let raf,clock=0,blob;
class LocalURL extends URL{static createObjectURL(b){blob=b;return 'blob:mock';}static revokeObjectURL(){}}
const sandbox={console,URL:LocalURL,URLSearchParams,Blob,location:{href:'http://localhost/index.html',search:''},history:{replaceState:()=>{}},devicePixelRatio:1,performance:{now:()=>clock},requestAnimationFrame:f=>{raf=f;},setTimeout:f=>f(),clearTimeout:()=>{},document:{getElementById:el,createElement:()=>new Element(),addEventListener:()=>{}},window:{addEventListener:()=>{}}};
vm.createContext(sandbox);
if(standalone){
 assert(!/<script[^>]+src=|<link[^>]+stylesheet/.test(html));
 vm.runInContext([...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m=>m[1]).join('\n'),sandbox);
 assert.equal(Object.keys(sandbox.window.ACROBOT_MEDIA).length,99);
 assert(el('video').src.startsWith('data:video/mp4;base64,'));
}else{vm.runInContext(fs.readFileSync(path.join(root,'data.js'),'utf8'),sandbox);vm.runInContext(fs.readFileSync(path.join(root,'app.js'),'utf8'),sandbox);}
const D=sandbox.window.ACROBOT_DATA;const challenges=D.scenarios.filter(s=>s.config.torque_mode==='original');assert.equal(el('scenario-list').children.length,11);assert.equal(D.scenarios.length,33);
for(let i=0;i<11;i++){
 for(const torque of ['original','clipped','unclipped']){
  el('torque-mode').value=torque;
  for(const family of ['focus','all','linear','solvers']){
   el('comparison-mode').value=family;
   el('scenario-list').children[i].click();assert.equal(el('scenario-title').textContent,challenges[i].config.title);
   el('comparison-mode').onchange();el('torque-mode').onchange();
   const count=family==='focus'?2:family==='all'?4:family==='solvers'?3:(D.solver_comparison.show_slsqp?4:3);
   assert.equal((el('metrics').innerHTML.match(/<tr>/g)||[]).length,count);
   assert(!/NaN|undefined/.test(el('metrics').innerHTML+el('findings').innerHTML+el('torque-impact').innerHTML));
   const explanations=['scenario-note','concept-grid','exploration-notes','method-model','method-reading'].map(id=>el(id).innerHTML).join(' ');
   assert(!/NaN|undefined/.test(explanations));
   assert.equal((el('concept-grid').innerHTML.match(/<article>/g)||[]).length,count);
   assert.equal(el('solver-benchmark').hidden,family!=='linear');
   assert.equal(el('nominal-option').hidden,family!=='all');
   assert.equal(el('parameters').innerHTML.includes('LQR finito · Qf'),family==='all');
   if(family==='focus')assert(!/LQR finito|MPC · SQP|plano único/.test(el('concept-grid').innerHTML));
   if(family==='linear')assert(el('concept-grid').innerHTML.includes('Hessiana exata')||torque==='unclipped');
   if(torque==='unclipped'){
    assert(el('scenario-note').innerHTML.includes('nenhum método exibido limita o torque'));
    assert.equal(el('torque-caption').textContent,'sem limites de torque');
   }
   if(['terminal','late'].includes(challenges[i].config.id)&&family!=='all')assert(el('scenario-note').innerHTML.includes('não participa desta visão'));
   const chosen=D.scenarios.find(s=>s.config.challenge_id===challenges[i].config.id&&s.config.torque_mode===torque);
   const cards=el('concept-grid').innerHTML.match(/<article>[\s\S]*?<\/article>/g);
   const shown=family==='focus'?['infinite','mpc']:family==='all'?['finite','infinite','ilqr','mpc']:family==='solvers'?['mpc','mpc_nonlinear_sqp','mpc_nonlinear_slsqp']:['mpc_lqr','mpc_sqp',...(D.solver_comparison.show_slsqp?['mpc_slsqp']:[]),'mpc'];
   shown.forEach((mode,j)=>{
    const settling=chosen.runs[mode].metrics.settling_s;
    assert(cards[j].includes(settling===null?'não estabilizou nos 6 s':`estabilizou a partir de ${settling.toLocaleString('pt-BR',{minimumFractionDigits:2,maximumFractionDigits:2})} s`));
   });
   if(torque==='unclipped')assert(el('parameters').innerHTML.includes('Sem clipping'));
   if(torque==='clipped')assert(el('parameters').innerHTML.includes('±3 N·m'));
   el('timeline').oninput({target:{value:'3.5'}});assert(el('clock').textContent.startsWith('3,50'));
   el('solver-select').value='mpc';el('solver-select').onchange();el('solve-index').value='119';el('solve-index').oninput();assert(el('solve-time').textContent.includes('5,95'));
   if(family==='all'){el('solver-select').value='ilqr';el('solver-select').onchange();assert(el('solve-index').disabled);}
   if(family==='linear'){el('solver-select').value='mpc_sqp';el('solver-select').onchange();assert(el('solver-detail').innerHTML.includes('KKT'));assert(el('cost-caption').textContent.includes(torque==='unclipped'?'analítica':'QP linear limitado'));
    el('solver-select').value='mpc_lqr';el('solver-select').onchange();assert(el('optimization-caption').textContent.includes('sem iterações'));}
   if(family==='solvers'){
    assert(explanations.includes('AutoDiff'));assert(explanations.includes('single shooting'));
    assert(!explanations.includes('diferenças finitas'));
    assert(!el('scenario-note').innerHTML.includes('mesma solução LQ analítica'));
    for(const m of ['mpc_nonlinear_sqp','mpc_nonlinear_slsqp']){
     el('solver-select').value=m;el('solver-select').onchange();
     assert(el('solver-detail').innerHTML.includes('Auditar as linearizações'));
     assert(el('solver-detail').innerHTML.includes('Gradiente projetado'));
     assert(!el('solver-detail').innerHTML.includes('Certificado QP'));
     assert(el('optimization-caption').textContent.includes('otimização não linear'));
     assert(!/NaN|undefined/.test(el('solver-detail').innerHTML));
    }
    assert(el('video').src.includes('_nonlinear_solvers')||standalone);
   }
   el('csv').click();assert(blob.size>1000);
  }
 }
}
// Regression: the optional SLSQP curve remains inspectable when grouped by default.
el('comparison-mode').value='linear';el('torque-mode').value='clipped';el('include-slsqp').checked=true;
el('include-slsqp').onchange();
assert.equal((el('metrics').innerHTML.match(/<tr>/g)||[]).length,4);
el('solver-select').value='mpc_slsqp';el('solver-select').onchange();
assert(el('solver-detail').innerHTML.includes('Custo do plano retornado'));
assert(el('solver-detail').innerHTML.includes('Status nativo do solver'));
assert(el('cost-caption').textContent.includes('ponto final laranja'));
// A rejected low-cost iterate must not be displayed as the returned improvement.
const fixture=D.scenarios.find(s=>s.config.challenge_id===challenges[10].config.id&&s.config.torque_mode==='clipped').runs.mpc_slsqp.solves[0];
const saved={...fixture};
Object.assign(fixture,{costs:[100,10],initial_cost:100,candidate_cost:10,returned_cost:100,recovery:'initial_guess',converged:false,native_status:'synthetic fallback regression',native_success:false});
el('solve-index').value='0';el('solve-index').oninput();
assert(el('solver-detail').innerHTML.includes('Redução do custo retornado</dt><dd>0,00%'));
assert(el('solver-detail').innerHTML.includes('Fallback para o initial guess viável'));
Object.assign(fixture,saved);
el('include-slsqp').checked=false;el('include-slsqp').onchange();
el('comparison-mode').value='all';el('torque-mode').value='original';
el('scenario-list').children[0].click();el('unit').value='deg';el('unit').onchange();el('angle-mode').value='absolute';el('angle-mode').onchange();assert(el('angle2-title').textContent.includes('absoluta'));
for(const mode of ['finite','infinite','ilqr','mpc'])el('legend').listeners.change({target:{dataset:{mode},checked:false}});
for(const mode of ['finite','infinite','ilqr','mpc'])el('legend').listeners.change({target:{dataset:{mode},checked:true}});
el('restart').click();el('play').click();clock=1000;raf(clock);assert(el('clock').textContent.startsWith('1,00'));
el('speed').value='2';clock=2000;raf(clock);assert(el('clock').textContent.startsWith('3,00'));
clock=4000;raf(clock);assert(el('clock').textContent.startsWith('6,00'));assert.equal(el('play').textContent,'▶');el('restart').click();assert(el('clock').textContent.startsWith('0,00'));
for(const id of ['trail','nominal','predictions']){el(id).checked=!el(id).checked;el(id).onchange();}
console.log('PASS: 11 challenges × 3 torque options × 4 views; contextual explanations, measured outcomes, replay, units, solvers and CSV (DOM double; not browser QA).');
