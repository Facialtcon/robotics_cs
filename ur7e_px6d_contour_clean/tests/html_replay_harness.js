// Run in gjs: real replay JavaScript, deterministic clock and minimal DOM.
const {Gio}=imports.gi,ByteArray=imports.byteArray;
const read=path=>ByteArray.toString(Gio.File.new_for_path(path).load_contents(null)[1]);
const page=read(ARGV[0]);
const data=JSON.parse(page.match(/<script type="application\/json" id="replay-data">([\s\S]*?)<\/script>/)[1]);
const script=page.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements={};let now=1000,callback,drawingCalls=0;
const context=new Proxy({measureText:v=>({width:String(v).length*8})},{
  get:(o,k)=>k in o?o[k]:(...args)=>{
    drawingCalls++;
    if(['moveTo','lineTo','arc','rect','fillRect','fillText'].includes(k)){
      const numbers=k==='fillText'?args.slice(1):args;
      if(numbers.some(v=>!Number.isFinite(v)))throw Error('Invalid canvas coordinates: '+k);
    }
  },set:(o,k,v)=>{o[k]=v;return true;}
});
function element(id){
  const classes=new Set(),listeners={},captured=new Set();
  const e={id,style:{setProperty(){}},children:[],width:0,height:0,hidden:id!=='replay-data',
    value:({component:'all',arrows:'both','force-scale':'1','velocity-scale':'1',rate:'.25'})[id]||'0',
    textContent:id==='replay-data'?JSON.stringify(data):'',
    classList:{add:k=>classes.add(k),remove:k=>classes.delete(k),contains:k=>classes.has(k)},
    append(...children){this.children.push(...children);},setAttribute(){},
    addEventListener(k,fn){listeners[k]=fn;},fire(k,event){listeners[k](event);},
    getContext:()=>context,
    setPointerCapture:k=>captured.add(k),hasPointerCapture:k=>captured.has(k),releasePointerCapture:k=>captured.delete(k)};
  Object.defineProperties(e,{
    clientWidth:{get:()=>elements['panel-'+id]?.classList.contains('expanded')?1300:700},
    clientHeight:{get:()=>elements['panel-'+id]?.classList.contains('expanded')?800:id==='xy'?340:235}
  });
  e.getBoundingClientRect=()=>({left:0,top:0,width:e.clientWidth,height:e.clientHeight});
  return e;
}
const document={getElementById:id=>elements[id]||(elements[id]=element(id)),
  createElement:id=>element(id),createTextNode:text=>({textContent:text}),addEventListener(){}};
const window={devicePixelRatio:1,innerWidth:1600,innerHeight:1100,addEventListener(){}};
new Function('document','window','requestAnimationFrame','performance',script)(
  document,window,cb=>callback=cb,{now:()=>now});
const api=window.experimentReplay,tests=[];
function check(ok,name){if(!ok)throw Error(name);tests.push(name);}
const copy=v=>JSON.stringify(v);
const wheel=(id,dy,shiftKey=false)=>api.wheelView(id,{clientX:350,clientY:110,deltaY:dy,shiftKey,preventDefault(){}});
check(api.rate===.25,'default speed');
check(data.samples.length===data.sample_count,'every raw sample available');
const force=copy(api.views.force),velocity=copy(api.views.velocity);
wheel('xy',-200);api.render();
check(copy(api.views.force)===force&&copy(api.views.velocity)===velocity,'independent XY zoom');
const g=api.geometry.xy;
check(Math.abs(g.x(1)-g.x(0)+g.y(1)-g.y(0))<1e-8,'equal XY millimeter scale');
wheel('force',-250);api.render();
check(copy(api.views.velocity)===velocity,'independent force time zoom');
wheel('force',-150,true);api.render();
check(api.views.force.y!==null,'force vertical zoom');
const forceAfter=copy(api.views.force);
wheel('velocity',-200);api.render();
check(copy(api.views.force)===forceAfter,'independent velocity time zoom');
const directionBefore=copy(api.views.direction);
wheel('direction',-200);api.render();
check(copy(api.views.direction)!==directionBefore,'direction zoom');
const cv=elements.force;
cv.fire('pointerdown',{clientX:350,clientY:120,button:0,pointerId:1});
cv.fire('pointermove',{clientX:320,clientY:120,pointerId:1});
cv.fire('pointerup',{clientX:320,clientY:120,pointerId:1});api.render();
check(copy(api.views.force)!==forceAfter,'drag pans');
cv.fire('pointermove',{clientX:350,clientY:120,pointerId:1});
check(!elements.tooltip.hidden&&elements.tooltip.textContent.includes('Fx / Fy'),'hover values');
const fg=api.geometry.force,middle=(fg.l+fg.w-fg.r)/2;
cv.fire('pointerdown',{clientX:middle,clientY:120,button:0,pointerId:1});
cv.fire('pointerup',{clientX:middle,clientY:120,pointerId:1});api.render();
check(Math.abs(api.time-(fg.xlim[0]+fg.xlim[1])/2)<1e-8,'curve click seeks shared time');
check(elements.clock.textContent.startsWith(api.time.toFixed(3)),'all charts share clock');
api.seek(1);elements.play.onclick();now+=1000;api.frame(now);
check(Math.abs(api.time-1.25)<1e-8,'quarter-speed timestamp playback');
const before=api.time;api.setRate(.05,now);
check(api.time===before,'speed adjustment has no jump');
now+=1000;api.frame(now);check(Math.abs(api.time-before-.05)<1e-8,'0.05x playback');
api.setRate(2,now);now+=500;api.frame(now);
check(Math.abs(api.time-before-1.05)<1e-8,'2x playback');
elements.play.onclick();const paused=api.time;now+=1000;api.frame(now);
check(api.time===paused,'pause');
elements.contact.onclick();api.render();check(api.time===data.first_contact.time_sec,'contact quick view');
elements.lost.onclick();api.render();check(api.time===4,'lost quick view');
elements.full.onclick();api.render();check(api.views.force.x[1]===data.duration,'full time reset');
for(const id of ['xy','force','velocity','direction']){
  elements[id+'-expand'].onclick();api.render();
  check(elements['panel-'+id].classList.contains('expanded'),'expand '+id);
  elements[id+'-expand'].onclick();
  elements[id+'-reset'].onclick();api.render();
}
const labels=elements['force-legend'].children,fxCheckbox=labels[0].children[0];
fxCheckbox.checked=false;fxCheckbox.onchange();api.render();check(!fxCheckbox.checked,'hide one curve');
for(const [lo,hi] of [[-1,1.7],[-.02,.09],[1.35,1.65]]){
  const ticks=api.niceTicks(lo,hi,300,45);
  const base=10**Math.floor(Math.log10(ticks.step));
  check([1,2,5,10].includes(Number((ticks.step/base).toFixed(8))),'regular ticks '+lo);
  if(lo<=0&&hi>=0)check(ticks.values.includes(0),'zero tick '+lo);
}
const rows=Array.from({length:10000},(_,i)=>[i*.001,i===5001?10:i===5002?-7:0]);
rows[5100][1]=null;
const reduced=api.peakPoints(rows,1,0,10,100);
check(reduced.some(r=>r[1]===10)&&reduced.some(r=>r[1]===-7),'display thinning retains spikes');
check(reduced.some(r=>r[1]===null),'display thinning retains gaps');
check(api.peakPoints(rows,1,5,5.01,100).length>=10,'local zoom accesses original samples');
elements['velocity-scale'].value='3';elements['velocity-scale'].oninput();api.render();
elements.arrows.value='force';elements.arrows.oninput();api.render();
elements.arrows.value='velocity';elements.arrows.oninput();api.render();
elements.restart.onclick();api.render();check(api.time===0,'restart');
print(JSON.stringify({result:'PASS',tests,drawingCalls}));
