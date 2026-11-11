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
 assert.equal(Object.keys(sandbox.window.ACROBOT_MEDIA).length,11);
 assert(el('video').src.startsWith('data:video/mp4;base64,'));
}else{vm.runInContext(fs.readFileSync(path.join(root,'data.js'),'utf8'),sandbox);vm.runInContext(fs.readFileSync(path.join(root,'app.js'),'utf8'),sandbox);}
const D=sandbox.window.ACROBOT_DATA;assert.equal(el('scenario-list').children.length,11);
for(let i=0;i<11;i++){
 el('scenario-list').children[i].click();assert.equal(el('scenario-title').textContent,D.scenarios[i].config.title);
 assert.equal((el('metrics').innerHTML.match(/<tr>/g)||[]).length,4);
 assert(!/NaN|undefined/.test(el('metrics').innerHTML+el('findings').innerHTML));
 el('timeline').oninput({target:{value:'3.5'}});assert(el('clock').textContent.startsWith('3,50'));
 el('solver-select').value='mpc';el('solver-select').onchange();el('solve-index').value='119';el('solve-index').oninput();assert(el('solve-time').textContent.includes('5,95'));
 el('solver-select').value='ilqr';el('solver-select').onchange();assert(el('solve-index').disabled);
 el('csv').click();assert(blob.size>1000);
}
el('scenario-list').children[0].click();el('unit').value='deg';el('unit').onchange();el('angle-mode').value='absolute';el('angle-mode').onchange();assert(el('angle2-title').textContent.includes('absoluta'));
for(const mode of ['finite','infinite','ilqr','mpc'])el('legend').listeners.change({target:{dataset:{mode},checked:false}});
for(const mode of ['finite','infinite','ilqr','mpc'])el('legend').listeners.change({target:{dataset:{mode},checked:true}});
el('restart').click();el('play').click();clock=1000;raf(clock);assert(el('clock').textContent.startsWith('1,00'));
el('speed').value='2';clock=2000;raf(clock);assert(el('clock').textContent.startsWith('3,00'));
clock=4000;raf(clock);assert(el('clock').textContent.startsWith('6,00'));assert.equal(el('play').textContent,'▶');el('restart').click();assert(el('clock').textContent.startsWith('0,00'));
for(const id of ['trail','nominal','predictions']){el(id).checked=!el(id).checked;el(id).onchange();}
console.log('PASS: 11 scenarios, replay, speed, scrubbing, units, overlays, filters, optimizer inspection and CSV (DOM double; not browser QA).');
