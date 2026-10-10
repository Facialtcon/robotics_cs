'use strict';
const $ = id => document.getElementById(id);
window.addEventListener('error', e => {
  $('error').hidden = false;
  $('error').textContent = '回放错误：' + e.message;
});
const D = JSON.parse($('replay-data').textContent);
const S = D.samples, A = D.actual, C = D.commands;
const colors = ['#3975ae', '#c65050', '#16836b'], phases = ['#3975ae', '#16836b', '#e68621'];
const finite = v => typeof v === 'number' && Number.isFinite(v);
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const fmt = (v, n = 2) => finite(v) ? v.toFixed(n) : '未记录';
const names = {FIRST_THRESHOLD_STOP_REQUEST:'首次接触阈值',FIRST_CONTACT:'接触确认',
  TRACKING_ENTERED:'进入跟踪',CONTACT_LOST:'丢失接触',REACQUIRED:'恢复接触',
  EXECUTION_STOP_CONFIRMED:'停止确认',USER_STOP:'停止请求',BUDGET_STOP:'停止请求'};
const eventColor = k => k === 'CONTACT_LOST' ? '#e68621' : k === 'REACQUIRED' ? '#1964b1' : '#9b287b';
const events = D.events.filter(e => names[e.kind]).sort((a,b) => a.time-b.time);
const eventCounts={};
const eventLabelMap=new Map(events.map(e=>{
  const n=(eventCounts[e.kind]||0)+1;eventCounts[e.kind]=n;
  return [e,e.kind==='FIRST_THRESHOLD_STOP_REQUEST'?(n===1?'首次接触':'接触阈值 '+n):
    names[e.kind]+(n>1?' '+n:'')];
}));
const eventLabel=e=>eventLabelMap.get(e)||names[e.kind];
const duration = Math.max(.0001, D.duration);
let elapsed = 0, playing = false, rate = .25, lastWall = performance.now();
let dirty = true, lastRender = -Infinity, lostIndex = 0;
const views = {force:{x:[0,duration],y:null},velocity:{x:[0,duration],y:null},
  xy:{cx:0,cy:0,span:1},direction:{cx:0,cy:0,span:2.4},timeline:{x:[0,duration]}};
const cache = {}, geometry = {}, hover = {};
function lastAt(rows,t) {
  let lo=0,hi=rows.length;
  while(lo<hi){const m=(lo+hi)>>1;if(rows[m][0]<=t)lo=m+1;else hi=m;}
  return lo-1;
}
function extents(values, minimum=.02) {
  let lo=Infinity,hi=-Infinity;
  for(const v of values)if(finite(v)){lo=Math.min(lo,v);hi=Math.max(hi,v);}
  if(lo===Infinity)return [-1,1];
  const pad=Math.max((hi-lo)*.08,minimum);
  return [lo-pad,hi+pad];
}
function niceTicks(lo,hi,pixels,minSpacing=72) {
  const count=Math.max(2,Math.ceil(pixels/minSpacing));
  const raw=(hi-lo)/count,base=Math.pow(10,Math.floor(Math.log10(Math.max(raw,1e-12))));
  const step=([1,2,5,10].find(v=>v*base>=raw-1e-12)||10)*base;
  const values=[];
  for(let i=Math.ceil((lo-1e-10*step)/step);i<=Math.floor((hi+1e-10*step)/step);i++){
    values.push(Math.abs(i*step)<step*1e-8?0:i*step);
    if(values.length>100)break;
  }
  return {values,step,digits:Math.max(0,-Math.floor(Math.log10(step)))};
}
function boundedRange(center,width) {
  width=clamp(width,.0001,duration);
  const start=clamp(center-width/2,0,duration-width);
  return [start,start+width];
}
function fittedXY(points) {
  const x=extents(points.map(p=>p[0]),.5),y=extents(points.map(p=>p[1]),.5);
  return {cx:(x[0]+x[1])/2,cy:(y[0]+y[1])/2,span:Math.max(x[1]-x[0],y[1]-y[0])};
}
const allPoints=S.filter(r=>finite(r[1])&&finite(r[2])).map(r=>r.slice(1,3));
if(D.target)allPoints.push(...D.target);
if(D.stop)allPoints.push(D.stop);
const fullXY=fittedXY(allPoints);
const firstContact=D.first_contact;
const contactStart=finite(firstContact.time_sec)?firstContact.time_sec:null;
const contactPoints=contactStart===null?[]:S.filter(r=>r[0]>=contactStart&&finite(r[1])&&finite(r[2])).map(r=>r.slice(1,3));
if(contactPoints.length&&D.target)contactPoints.push(...D.target);
const contactXY=contactPoints.length?fittedXY(contactPoints):fullXY;
views.xy={...contactXY};
const forceNorm=Math.max(.2,D.deadband.reference||1.5);
const positiveSpeeds=A.map(r=>r[3]).filter(v=>finite(v)&&v>.01).sort((a,b)=>a-b);
const speedNorm=positiveSpeeds.length?Math.max(.1,positiveSpeeds[positiveSpeeds.length-1]):1;
const xyForceScale=contactXY.span*.1/forceNorm,xySpeedScale=contactXY.span*.1/speedNorm;
const defs = {
  force:[{key:'fx',name:'Fx',rows:S,col:3,color:colors[0]},
    {key:'fy',name:'Fy',rows:S,col:4,color:colors[1]},
    {key:'fxy',name:'Fxy',rows:S,col:5,color:colors[2]},
    {key:'raw',name:'滤波前 Fxy',rows:S,col:8,color:'#94a3b8'}],
  velocity:[0,1,2].flatMap(i=>[
    {key:'a'+i,name:'实际 '+['vx','vy','XY'][i],rows:A,col:i+1,color:colors[i],component:i},
    {key:'c'+i,name:'指令 '+['vx','vy','XY'][i],rows:C,col:i+1,color:colors[i],component:i,step:true}])
};
const visible=new Set(defs.force.concat(defs.velocity).map(s=>s.key));
function seriesFor(id) {
  const component=$('component').value;
  return defs[id].filter(s=>visible.has(s.key)&&(id!=='velocity'||component==='all'||s.component===Number(component)));
}
function legend(id) {
  for(const s of defs[id]){
    const label=document.createElement('label'),input=document.createElement('input');
    input.type='checkbox';input.checked=true;input.setAttribute('aria-label',s.name);
    input.onchange=()=>{input.checked?visible.add(s.key):visible.delete(s.key);dirty=true;};
    const swatch=document.createElement('i');
    swatch.className='swatch'+(s.step?' dashed':'');swatch.style.setProperty('--c',s.color);
    label.append(input,swatch,document.createTextNode(s.name));$(id+'-legend').append(label);
  }
}
legend('force');legend('velocity');
const bandLegend=document.createElement('span'),bandSwatch=document.createElement('i');
bandSwatch.className='swatch band';
bandLegend.append(bandSwatch,document.createTextNode(D.deadband.enabled?
  'Deadband '+fmt(D.deadband.lower)+'–'+fmt(D.deadband.upper)+' N（跟踪反馈）':'Deadband：'+D.deadband.reason));
$('force-legend').append(bandLegend);
function surface(id) {
  const canvas=$(id),w=canvas.clientWidth,h=canvas.clientHeight,dpr=window.devicePixelRatio||1;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){
    canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
  }
  const c=canvas.getContext('2d');
  c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);c.font='12px system-ui';
  return {c,w,h,dpr};
}
function snapshot(id) {
  const canvas=document.createElement('canvas');canvas.width=$(id).width;canvas.height=$(id).height;
  canvas.getContext('2d').drawImage($(id),0,0);return canvas;
}
function line(c,points,color,width=1.2,dash=[]) {
  c.beginPath();let active=false;
  for(const p of points){if(!p||!p.every(finite)){active=false;continue;}
    if(active)c.lineTo(...p);else{c.moveTo(...p);active=true;}}
  c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);c.stroke();c.setLineDash([]);
}
function arrow(c,x,y,dx,dy,color,dashed=false) {
  if(![x,y,dx,dy].every(finite))return;
  line(c,[[x,y],[x+dx,y+dy]],color,2.4,dashed?[5,3]:[]);
  const n=Math.hypot(dx,dy);if(n<1)return;
  const angle=Math.atan2(dy,dx),len=Math.min(10,n*.45);
  c.beginPath();c.moveTo(x+dx,y+dy);
  c.lineTo(x+dx-len*Math.cos(angle-.45),y+dy-len*Math.sin(angle-.45));
  c.lineTo(x+dx-len*Math.cos(angle+.45),y+dy-len*Math.sin(angle+.45));
  c.closePath();c.fillStyle=color;c.fill();
}
function dot(c,p,color,r=4) {
  if(!p||!p.every(finite))return;
  c.beginPath();c.arc(...p,r,0,Math.PI*2);c.fillStyle=color;c.fill();
}
function clip(c,g) {c.save();c.beginPath();c.rect(g.l,g.t,g.w-g.l-g.r,g.h-g.t-g.b);c.clip();}
function axes(c,g,xlim,ylim,xLabel,yLabel) {
  const xs=niceTicks(...xlim,g.w-g.l-g.r,86),ys=niceTicks(...ylim,g.h-g.t-g.b,28);
  c.fillStyle='#64748b';
  for(const v of xs.values){
    line(c,[[g.x(v),g.t],[g.x(v),g.h-g.b]],'#e3e9f0');
    const text=v.toFixed(xs.digits),width=c.measureText(text).width;
    c.textAlign='center';c.fillText(text,clamp(g.x(v),width/2+2,g.w-width/2-2),g.h-g.b+20);
  }
  for(const v of ys.values){
    line(c,[[g.l,g.y(v)],[g.w-g.r,g.y(v)]],v===0?'#9baabd':'#e3e9f0',v===0?1.4:.8);
    c.textAlign='right';c.fillStyle=v===0?'#243247':'#64748b';
    c.fillText(v.toFixed(ys.digits),g.l-8,clamp(g.y(v)+4,g.t+5,g.h-g.b));
  }
  c.textAlign='center';c.fillStyle='#64748b';c.fillText(xLabel,(g.l+g.w-g.r)/2,g.h-6);
  c.save();c.translate(14,(g.t+g.h-g.b)/2);c.rotate(-Math.PI/2);c.fillText(yLabel,0,0);c.restore();c.textAlign='left';
}
function peakPoints(rows,column,begin,end,pixels) {
  if(!rows.length)return [];
  const start=Math.max(0,lastAt(rows,begin)),finish=Math.min(rows.length,lastAt(rows,end)+2);
  if(finish-start<=pixels*3)return rows.slice(start,finish).map(r=>[r[0],r[column]]);
  const result=[],width=Math.max(.0001,end-begin);
  let bucket=null,items=[];
  const flush=()=>{
    if(!items.length)return;
    let low=0,high=0;
    for(let j=1;j<items.length;j++){
      if(items[j][1]<items[low][1])low=j;if(items[j][1]>items[high][1])high=j;
    }
    for(const j of [...new Set([0,low,high,items.length-1])].sort((a,b)=>a-b))result.push(items[j]);
    items=[];
  };
  for(let i=start;i<finish;i++){
    const r=rows[i],next=Math.floor((r[0]-begin)/width*pixels);
    if(!finite(r[column])){flush();result.push([r[0],null]);bucket=null;continue;}
    if(next!==bucket){flush();bucket=next;}items.push([r[0],r[column]]);
  }
  flush();return result;
}
function graphGeometry(id,w,h,series) {
  const view=views[id],values=[];
  for(const s of series){
    const begin=Math.max(0,lastAt(s.rows,view.x[0])),end=Math.min(s.rows.length,lastAt(s.rows,view.x[1])+2);
    for(let j=begin;j<end;j++)if(finite(s.rows[j][s.col]))values.push(s.rows[j][s.col]);
  }
  if(id==='force'&&view.x[0]===0&&view.x[1]===duration){
    for(const reference of [D.threshold,D.lost_threshold])if(finite(reference))values.push(reference);
    if(D.deadband.enabled&&visible.has('fxy'))values.push(D.deadband.lower,D.deadband.upper);
  }
  let y=view.y||extents(values,.02);
  const yt=niceTicks(...y,h-95,28);
  const c=$(id).getContext('2d');c.font='12px system-ui';
  const labelWidth=Math.max(35,...yt.values.map(v=>c.measureText(v.toFixed(yt.digits)).width));
  const g={w,h,l:Math.max(60,labelWidth+29),r:26,t:55,b:43,ylim:y,xlim:[...view.x]};
  g.x=t=>g.l+(t-view.x[0])/(view.x[1]-view.x[0])*(w-g.l-g.r);
  g.y=v=>g.t+(y[1]-v)/(y[1]-y[0])*(h-g.t-g.b);
  geometry[id]=g;return g;
}
function eventLabels(c,g) {
  const ends=[g.l-8,g.l-8,g.l-8];
  const positions=[];
  for(const e of events){
    if(e.time<g.xlim[0]||e.time>g.xlim[1])continue;
    const label=eventLabel(e),width=c.measureText(label).width,px=g.x(e.time);
    const left=clamp(px-width/2,g.l,g.w-g.r-width);
    const lane=ends.findIndex(end=>left>=end+7);
    if(lane<0)continue; // All events retain their line, tooltip and event-list entry.
    const baseline=13+lane*16;
    c.fillStyle=eventColor(e.kind);c.fillText(label,left,baseline);ends[lane]=left+width;
    line(c,[[px,baseline+3],[px,g.t]],eventColor(e.kind),.7);
    positions.push({left,right:left+width,top:baseline-12,bottom:baseline,event:e});
  }
  return positions;
}
function deadband(c,g) {
  if(!D.deadband.enabled||!visible.has('fxy'))return;
  for(const window of D.deadband.windows){
    const lo=Math.max(g.xlim[0],window.start),hi=Math.min(g.xlim[1],window.end);
    if(hi<=lo||window.status==='off')continue;
    const top=g.y(window.upper),bottom=g.y(window.lower);
    if(window.status==='active'){c.fillStyle='rgba(100,116,139,.12)';c.fillRect(g.x(lo),top,g.x(hi)-g.x(lo),bottom-top);}
    line(c,[[g.x(lo),top],[g.x(hi),top]],'#64748b',1,window.status==='active'?[6,4]:[2,5]);
    line(c,[[g.x(lo),bottom],[g.x(hi),bottom]],'#64748b',1,window.status==='active'?[6,4]:[2,5]);
  }
}
function graph(id) {
  const {c,w,h}=surface(id),series=seriesFor(id);
  const key=JSON.stringify([w,h,window.devicePixelRatio,views[id],series.map(s=>s.key)]);
  let g;
  if(cache[id]&&cache[id].key===key){g=cache[id].geometry;c.drawImage(cache[id].canvas,0,0,w,h);}
  else{
    g=graphGeometry(id,w,h,series);axes(c,g,g.xlim,g.ylim,'实验时间 [s]',id==='force'?'力 [N]':'速度 [mm/s]');
    clip(c,g);
    if(id==='force'){
      deadband(c,g);
      for(const [value,color] of [[D.threshold,'#9b287b'],[D.lost_threshold,'#e68621']]){
        if(finite(value))line(c,[[g.l,g.y(value)],[w-g.r,g.y(value)]],color,1.2,[5,4]);
      }
    }
    // Keep the raw magnitude visible behind the three primary filtered curves.
    for(const s of [...series].sort((a,b)=>(a.key==='raw'?-1:0)-(b.key==='raw'?-1:0))){
      const points=peakPoints(s.rows,s.col,...g.xlim,Math.floor(w-g.l-g.r)),path=[];
      let previous=null;
      for(const p of points){
        if(!finite(p[1])){path.push(null);previous=null;continue;}
        const next=[g.x(p[0]),g.y(p[1])];
        if(s.step&&previous)path.push([next[0],previous[1]]);
        path.push(next);previous=next;
      }
      c.globalAlpha=s.key==='raw'?.45:1;
      line(c,path,s.color,s.key==='raw'?.8:1.35,s.step?[6,4]:[]);
    }
    c.globalAlpha=1;
    for(const e of events)if(e.time>=g.xlim[0]&&e.time<=g.xlim[1]){
      line(c,[[g.x(e.time),g.t],[g.x(e.time),h-g.b]],eventColor(e.kind),.8,[2,4]);
    }
    c.restore();const labels=eventLabels(c,g);
    cache[id]={key,geometry:g,canvas:snapshot(id),labels};
  }
  clip(c,g);
  if(elapsed>=g.xlim[0]&&elapsed<=g.xlim[1])line(c,[[g.x(elapsed),g.t],[g.x(elapsed),h-g.b]],'#111827',1.5);
  if(finite(hover[id]))line(c,[[g.x(hover[id]),g.t],[g.x(hover[id]),h-g.b]],'#64748b',1,[3,3]);
  c.restore();
  $(id+'-view').textContent=fmt(g.xlim[0],3)+'–'+fmt(g.xlim[1],3)+' s · '+fmt(g.ylim[0])+'–'+fmt(g.ylim[1])+
    (id==='force'?' N':' mm/s')+(elapsed<g.xlim[0]||elapsed>g.xlim[1]?' · 当前时间在本图视野外':'');
}
function spatialGeometry(id,w,h) {
  const v=views[id],g={w,h,l:id==='xy'?66:25,r:24,t:17,b:id==='xy'?43:23};
  g.scale=Math.min(w-g.l-g.r,h-g.t-g.b)/v.span;
  g.x=x=>g.l+(w-g.l-g.r)/2+(x-v.cx)*g.scale;
  g.y=y=>g.t+(h-g.t-g.b)/2-(y-v.cy)*g.scale;
  g.xlim=[v.cx-(w-g.l-g.r)/(2*g.scale),v.cx+(w-g.l-g.r)/(2*g.scale)];
  g.ylim=[v.cy-(h-g.t-g.b)/(2*g.scale),v.cy+(h-g.t-g.b)/(2*g.scale)];
  geometry[id]=g;return g;
}
function markerLabels(c,g,markers) {
  const boxes=[];
  for(const m of markers){
    const [x,y]=m.point;if(x<g.l||x>g.w-g.r||y<g.t||y>g.h-g.b)continue;
    dot(c,m.point,m.color);
    const width=c.measureText(m.label).width;
    for(const [dx,dy] of [[8,-8],[8,16],[-width-8,-8],[-width-8,16],[8,-25],[8,32]]){
      const left=clamp(x+dx,g.l+2,g.w-g.r-width-2),bottom=clamp(y+dy,g.t+14,g.h-g.b-2);
      const box={left,right:left+width,top:bottom-13,bottom};
      if(boxes.some(b=>box.left<b.right+5&&box.right>b.left-5&&box.top<b.bottom+4&&box.bottom>b.top-4))continue;
      c.fillStyle=m.color;c.fillText(m.label,left,bottom);boxes.push(box);break;
    }
  }
}
function drawXY(row,actual) {
  const {c,w,h}=surface('xy'),g=spatialGeometry('xy',w,h);
  const key=JSON.stringify([w,h,window.devicePixelRatio,views.xy]);
  const project=p=>[g.x(p[0]),g.y(p[1])];
  if(!cache.xy||cache.xy.key!==key){
    axes(c,g,g.xlim,g.ylim,'Base X [mm]','Base Y [mm]');
    clip(c,g);if(D.target)line(c,D.target.map(project),'#344256',1.4,[6,4]);
    const path=[];let previous=null;
    for(const r of S){
      if(!finite(r[1])||!finite(r[2])){previous=null;continue;}
      const p=project(r.slice(1,3));
      if(!previous||Math.hypot(p[0]-previous.x,p[1]-previous.y)>.6||r[6]!==previous.phase){
        path.push({time:r[0],x:p[0],y:p[1],phase:r[6],gap:!previous});
        previous={x:p[0],y:p[1],phase:r[6]};
      }
    }
    const end=S[S.length-1];path.push({time:end[0],x:g.x(end[1]),y:g.y(end[2]),phase:end[6]});
    c.globalAlpha=.25;
    for(let phase=0;phase<3;phase++)line(c,path.map(p=>p.phase===phase?[p.x,p.y]:null),phases[phase],1.5);
    c.globalAlpha=1;c.restore();
    cache.xy={key,canvas:snapshot('xy'),path,trail:null,trailEnd:-1};
  }
  const item=cache.xy;c.drawImage(item.canvas,0,0,w,h);
  const target=lastAt(item.path.map(p=>[p.time]),elapsed);
  if(!item.trail||target<item.trailEnd){
    item.trail=document.createElement('canvas');item.trail.width=$('xy').width;item.trail.height=$('xy').height;item.trailEnd=-1;
  }
  const tc=item.trail.getContext('2d');tc.setTransform(window.devicePixelRatio||1,0,0,window.devicePixelRatio||1,0,0);
  clip(tc,g);
  for(let j=Math.max(1,item.trailEnd+1);j<=target;j++){
    const p=item.path[j],previous=item.path[j-1];
    if(!p.gap)line(tc,[[previous.x,previous.y],[p.x,p.y]],phases[p.phase],2.2);
  }
  tc.restore();item.trailEnd=target;c.drawImage(item.trail,0,0,w,h);
  clip(c,g);
  const markers=[{point:project(S[0].slice(1,3)),color:'#26384d',label:'起点'},
    {point:project(S[S.length-1].slice(1,3)),color:'#26384d',label:'最终记录'}];
  if(D.stop)markers.push({point:project(D.stop),color:'#b72a33',label:'停止确认'});
  const counts={CONTACT_LOST:0,REACQUIRED:0};
  for(const e of events){
    if(e.kind in counts)counts[e.kind]++;
    if(e.time>elapsed||!e.xy_mm.every(finite))continue;
    if(e.kind==='FIRST_CONTACT'&&events.some(v=>v.kind==='FIRST_THRESHOLD_STOP_REQUEST'))continue;
    markers.push({point:project(e.xy_mm),color:eventColor(e.kind),
      label:e.kind in counts?(e.kind==='CONTACT_LOST'?'丢':'恢复')+counts[e.kind]:eventLabel(e)});
  }
  markerLabels(c,g,markers);
  const p=project(row.slice(1,3));dot(c,p,'#111827',5);
  arrow(c,...p,row[3]*xyForceScale*g.scale,-row[4]*xyForceScale*g.scale,'#9b287b');
  if(actual)arrow(c,...p,actual[1]*xySpeedScale*g.scale,-actual[2]*xySpeedScale*g.scale,'#2563b1',true);
  c.restore();
  $('xy-view').textContent='视野 '+fmt(g.xlim[0],1)+'–'+fmt(g.xlim[1],1)+' mm × '+fmt(g.ylim[0],1)+'–'+fmt(g.ylim[1],1)+' mm · X/Y 使用相同像素比例';
}
function drawDirection(row,actual) {
  const {c,w,h}=surface('direction'),g=spatialGeometry('direction',w,h),mode=$('arrows').value;
  clip(c,g);
  line(c,[[g.l,g.y(0)],[w-g.r,g.y(0)]],'#cbd5e1');
  line(c,[[g.x(0),h-g.b],[g.x(0),g.t]],'#cbd5e1');
  const p=[g.x(0),g.y(0)];dot(c,p,'#111827',5);
  const fs=Number($('force-scale').value),vs=Number($('velocity-scale').value);
  if(mode!=='velocity')arrow(c,...p,row[3]/forceNorm*.75*fs*g.scale,-row[4]/forceNorm*.75*fs*g.scale,'#9b287b');
  if(mode!=='force'&&actual)arrow(c,...p,actual[1]/speedNorm*.75*vs*g.scale,-actual[2]/speedNorm*.75*vs*g.scale,'#2563b1',true);
  c.restore();c.fillStyle='#64748b';c.fillText('+Base X',Math.max(8,w-78),clamp(g.y(0)-6,14,h-10));
  c.fillText('+Base Y',clamp(g.x(0)+7,5,w-70),14);
  $('scales').textContent='方向图比例：力 '+fmt(.75*fs*g.scale/forceNorm)+' px/N；实际速度 '+fmt(.75*vs*g.scale/speedNorm)+' px/(mm/s)。箭头超出时可滚轮缩小。';
}
function currentValues(t) {
  return {row:S[Math.max(0,lastAt(S,t))],actual:A[lastAt(A,t)],command:C[lastAt(C,t)]};
}
function valuesText(t,id) {
  const {row,actual,command}=currentValues(t),triple=r=>r?r.slice(1,4).map(v=>fmt(v)).join(' / '):'未记录';
  let text='实验时间 '+fmt(t,4)+' s\n';
  if(!id||id==='force')text+='Fx / Fy / Fxy: '+row.slice(3,6).map(v=>fmt(v,3)).join(' / ')+' N\n力样本: '+fmt(row[0],6)+' s\n';
  if(!id||id==='velocity')text+='实际 vx / vy / XY: '+triple(actual)+' mm/s\n指令 vx / vy / XY: '+triple(command)+' mm/s\n实际样本: '+(actual?fmt(actual[0],6):'未记录')+' s\n指令发送: '+(command?fmt(command[0],6):'未记录')+' s\n';
  if(!id)text+='TCP X / Y: '+fmt(row[1])+' / '+fmt(row[2])+' mm\n状态: '+D.states[row[7]];
  return text;
}
function updateTimeline() {
  if(elapsed<views.timeline.x[0]||elapsed>views.timeline.x[1])views.timeline.x=boundedRange(elapsed,views.timeline.x[1]-views.timeline.x[0]);
  const x=views.timeline.x;
  $('time').min=x[0];$('time').max=x[1];$('time').value=clamp(elapsed,...x);
  $('clock').textContent=fmt(elapsed,3)+' / '+fmt(D.duration,3)+' s';
}
function render() {
  // Embedded viewers can report a zero-size viewport before the first layout.
  if(['xy','direction','force','velocity'].some(id=>$(id).clientWidth<90||$(id).clientHeight<90)){dirty=true;return;}
  const {row,actual}=currentValues(elapsed);
  updateTimeline();$('play').textContent=playing?'暂停':'播放';
  drawXY(row,actual);drawDirection(row,actual);graph('force');graph('velocity');
  $('values').textContent=valuesText(elapsed);dirty=false;
}
function advance(wall) {
  if(playing)elapsed=clamp(elapsed+Math.max(0,wall-lastWall)/1000*rate,0,D.duration);
  lastWall=wall;if(elapsed>=D.duration)playing=false;
}
function seek(t) {
  advance(performance.now());playing=false;elapsed=clamp(t,0,D.duration);dirty=true;
}
function setRate(value,wall=performance.now()) {
  advance(wall);rate=Math.round(clamp(Number(value),.05,2)*100)/100;
  $('rate').value=rate;$('rate-value').textContent=rate.toFixed(2)+'x';dirty=true;
}
function frame(wall) {
  const before=elapsed;
  advance(wall);
  if(playing||elapsed!==before)dirty=true;
  if(dirty&&wall-lastRender>=1000/30){render();lastRender=wall;}
  requestAnimationFrame(frame);
}
function resetView(id,full=false) {
  if(id==='xy')views.xy={...(full?fullXY:contactXY)};
  else if(id==='direction'){
    const {row,actual}=currentValues(elapsed),lengths=[1.2];
    for(const v of row.slice(3,5))if(finite(v))lengths.push(Math.abs(v)/forceNorm*.9);
    if(actual)for(const v of actual.slice(1,3))if(finite(v))lengths.push(Math.abs(v)/speedNorm*.9);
    views.direction={cx:0,cy:0,span:Math.max(...lengths)*2};$('force-scale').value=1;$('velocity-scale').value=1;$('arrows').value='both';
  }
  else views[id]={x:[0,duration],y:null};
  dirty=true;
}
function eventZoom(time) {
  seek(time);
  for(const id of ['force','velocity'])views[id]={x:boundedRange(time,2),y:null};
  views.timeline.x=boundedRange(time,2);dirty=true;
}
for(const id of ['xy','force','velocity','direction']){
  $(id+'-reset').onclick=()=>resetView(id);
  $(id+'-expand').onclick=()=>{
    const panel=$('panel-'+id),expanded=panel.classList.contains('expanded');
    for(const name of ['xy','force','velocity','direction']){
      $('panel-'+name).classList.remove('expanded');$(name+'-expand').textContent='放大显示';
    }
    if(!expanded){panel.classList.add('expanded');$(id+'-expand').textContent='返回布局';}
    dirty=true;
  };
}
document.addEventListener('keydown',e=>{
  if(e.key==='Escape')for(const id of ['xy','force','velocity','direction']){
    $('panel-'+id).classList.remove('expanded');$(id+'-expand').textContent='放大显示';dirty=true;
  }
});
$('xy-full').onclick=()=>resetView('xy',true);$('xy-contact').onclick=()=>resetView('xy');
$('xy-contact').disabled=contactStart===null;
$('direction-fit').onclick=()=>{
  const {row,actual}=currentValues(elapsed);
  views.direction={cx:0,cy:0,span:2.4};
  $('force-scale').value=finite(row[5])&&row[5]>0?clamp(forceNorm/row[5],.2,20):1;
  $('velocity-scale').value=actual&&finite(actual[3])&&actual[3]>0?clamp(speedNorm/actual[3],.2,20):1;
  dirty=true;
};
for(const id of ['force','velocity']){
  $(id+'-full').onclick=()=>resetView(id);
  $(id+'-auto').onclick=()=>{views[id].y=null;dirty=true;};
}
$('play').onclick=()=>{advance(performance.now());if(elapsed>=D.duration)elapsed=0;playing=!playing;dirty=true;};
$('restart').onclick=()=>seek(0);
$('rate').oninput=e=>setRate(e.target.value);
$('time').oninput=e=>{if(!timelineDrag)seek(Number(e.target.value));};
$('component').onchange=()=>dirty=true;
for(const id of ['arrows','force-scale','velocity-scale'])$(id).oninput=()=>dirty=true;
$('full').onclick=()=>{resetView('force');resetView('velocity');views.timeline.x=[0,duration];dirty=true;};
$('contact').disabled=contactStart===null;$('contact').onclick=()=>eventZoom(contactStart);
const lost=events.filter(e=>e.kind==='CONTACT_LOST');
$('lost').disabled=!lost.length;$('lost').onclick=()=>eventZoom(lost[lostIndex++%lost.length].time);
for(const event of events){
  const button=document.createElement('button');
  button.style.setProperty('--c',eventColor(event.kind));
  button.textContent=eventLabel(event)+' '+fmt(event.time,3)+' s';button.onclick=()=>seek(event.time);
  $('xy-events').append(button);
}
function localPoint(e,id) {const r=$(id).getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];}
function wheelView(id,e) {
  e.preventDefault();const g=geometry[id];if(!g)return;
  const [px,py]=localPoint(e,id),factor=Math.exp(clamp(e.deltaY,-400,400)*.002);
  if(id==='force'||id==='velocity'){
    const v=views[id];
    if(e.shiftKey){
      const y=v.y||g.ylim,fraction=clamp((g.h-g.b-py)/(g.h-g.t-g.b),0,1),anchor=y[0]+fraction*(y[1]-y[0]);
      const width=clamp((y[1]-y[0])*factor,1e-5,1e6);v.y=[anchor-fraction*width,anchor+(1-fraction)*width];
    }else{
      const fraction=clamp((px-g.l)/(g.w-g.l-g.r),0,1),anchor=v.x[0]+fraction*(v.x[1]-v.x[0]);
      const width=clamp((v.x[1]-v.x[0])*factor,.0001,duration);
      v.x=boundedRange(anchor+(0.5-fraction)*width,width);
    }
  }else{
    const v=views[id],anchorX=v.cx+(px-(g.l+g.w-g.r)/2)/g.scale,anchorY=v.cy-(py-(g.t+g.h-g.b)/2)/g.scale;
    const newSpan=clamp(v.span*factor,id==='xy'?.001:.001,1e7),ratio=newSpan/v.span;
    v.cx=anchorX+(v.cx-anchorX)*ratio;v.cy=anchorY+(v.cy-anchorY)*ratio;v.span=newSpan;
  }
  dirty=true;
}
function tooltip(e,text) {
  const box=$('tooltip');box.textContent=text;box.hidden=false;
  box.style.left=clamp(e.clientX+14,5,(window.innerWidth||1600)-Math.min(340,box.offsetWidth||340)-8)+'px';
  box.style.top=clamp(e.clientY+14,5,(window.innerHeight||1000)-(box.offsetHeight||180)-8)+'px';
}
for(const id of ['xy','force','velocity','direction']){
  const canvas=$(id);let drag=null;
  canvas.addEventListener('wheel',e=>wheelView(id,e),{passive:false});
  canvas.addEventListener('pointerdown',e=>{
    if(e.button!==0)return;
    drag={point:localPoint(e,id),view:JSON.parse(JSON.stringify(views[id])),y:geometry[id]&&geometry[id].ylim,moved:false};
    try{canvas.setPointerCapture(e.pointerId);}catch(error){if(error.name!=='NotFoundError')throw error;}
    delete hover[id];$('tooltip').hidden=true;
  });
  canvas.addEventListener('pointermove',e=>{
    const g=geometry[id];if(!g)return;const p=localPoint(e,id);
    if(drag){
      const dx=p[0]-drag.point[0],dy=p[1]-drag.point[1];
      if(Math.hypot(dx,dy)<3&&!drag.moved)return;drag.moved=true;
      if(id==='force'||id==='velocity'){
        const x=drag.view.x,width=x[1]-x[0];
        views[id].x=boundedRange((x[0]+x[1])/2-dx/(g.w-g.l-g.r)*width,width);
        if(Math.abs(dy)>3||drag.view.y){const y=drag.view.y||drag.y,delta=dy/(g.h-g.t-g.b)*(y[1]-y[0]);views[id].y=[y[0]+delta,y[1]+delta];}
      }else{views[id].cx=drag.view.cx-dx/g.scale;views[id].cy=drag.view.cy+dy/g.scale;}
      dirty=true;return;
    }
    if(id==='force'||id==='velocity'){
      if(p[0]<g.l||p[0]>g.w-g.r||p[1]<0||p[1]>g.h-g.b){delete hover[id];$('tooltip').hidden=true;dirty=true;return;}
      const t=g.xlim[0]+clamp((p[0]-g.l)/(g.w-g.l-g.r),0,1)*(g.xlim[1]-g.xlim[0]);
      hover[id]=t;let text=valuesText(t,id);
      const nearby=events.filter(v=>Math.abs(g.x(v.time)-p[0])<8);
      if(nearby.length)text+='\n'+nearby.map(v=>eventLabel(v)+' '+fmt(v.time,6)+' s').join('\n');
      tooltip(e,text);dirty=true;
    }else if(id==='xy'){
      const nearby=events.filter(v=>v.xy_mm.every(finite)&&Math.hypot(g.x(v.xy_mm[0])-p[0],g.y(v.xy_mm[1])-p[1])<9);
      if(nearby.length)tooltip(e,nearby.map(v=>eventLabel(v)+' '+fmt(v.time,6)+' s').join('\n'));else $('tooltip').hidden=true;
    }
  });
  canvas.addEventListener('pointerup',e=>{
    if(!drag)return;
    if(!drag.moved&&(id==='force'||id==='velocity')){
      const g=geometry[id],p=localPoint(e,id);
      const label=(cache[id].labels||[]).find(v=>p[0]>=v.left&&p[0]<=v.right&&p[1]>=v.top&&p[1]<=v.bottom);
      if(label)seek(label.event.time);
      else if(p[0]>=g.l&&p[0]<=g.w-g.r)seek(g.xlim[0]+clamp((p[0]-g.l)/(g.w-g.l-g.r),0,1)*(g.xlim[1]-g.xlim[0]));
    }
    drag=null;if(canvas.hasPointerCapture(e.pointerId))canvas.releasePointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointercancel',()=>drag=null);
  canvas.addEventListener('pointerleave',()=>{if(!drag){delete hover[id];$('tooltip').hidden=true;dirty=true;}});
}
$('time').addEventListener('wheel',e=>{
  e.preventDefault();const r=$('time').getBoundingClientRect(),fraction=clamp((e.clientX-r.left)/r.width,0,1),x=views.timeline.x;
  const anchor=x[0]+fraction*(x[1]-x[0]),width=clamp((x[1]-x[0])*Math.exp(clamp(e.deltaY,-400,400)*.002),.0001,duration);
  views.timeline.x=boundedRange(anchor+(0.5-fraction)*width,width);dirty=true;
},{passive:false});
let timelineDrag=null;
$('time').addEventListener('pointerdown',e=>{
  if(e.shiftKey){e.preventDefault();timelineDrag={px:e.clientX,x:[...views.timeline.x]};try{$('time').setPointerCapture(e.pointerId);}catch(error){if(error.name!=='NotFoundError')throw error;}}
});
$('time').addEventListener('pointermove',e=>{
  if(!timelineDrag)return;
  const x=timelineDrag.x,width=x[1]-x[0],dx=(e.clientX-timelineDrag.px)/$('time').getBoundingClientRect().width*width;
  views.timeline.x=boundedRange((x[0]+x[1])/2-dx,width);dirty=true;
});
$('time').addEventListener('pointerup',e=>{if(timelineDrag){timelineDrag=null;if($('time').hasPointerCapture(e.pointerId))$('time').releasePointerCapture(e.pointerId);}});
$('time').addEventListener('pointercancel',()=>timelineDrag=null);
$('name').textContent=(D.synthetic?'模拟数据 · ':'')+D.name;
$('thresholds').textContent='首次接触 '+fmt(D.threshold)+' N（紫色虚线） · 丢接触 '+fmt(D.lost_threshold)+' N（橙色虚线）';
$('deadband').textContent=D.deadband.enabled?
  'Deadband：误差 '+fmt(D.deadband.reference)+' − Fxy 的 ±'+fmt(D.deadband.half_width)+' N；等效 Fxy '+fmt(D.deadband.lower)+'–'+fmt(D.deadband.upper)+
  ' N。灰色阴影仅显示已记录反馈执行条件的跟踪段；灰色点线表示跟踪段执行标志缺失，不假定已执行。':'Deadband：'+D.deadband.reason;
const array=v=>v?v.map(x=>fmt(x)).join(' / '):'未记录';
if(finite(firstContact.time_sec)){
  $('collision').textContent='首次记录接触：'+fmt(firstContact.time_sec,6)+' s\n接触前指令 vx / vy / XY：'+array(firstContact.command_before_xy_mm_s)+
    ' mm/s（'+fmt(firstContact.command_before_time_sec,6)+' s）\n接触前实际 vx / vy / XY：'+array(firstContact.actual_before_xy_mm_s)+
    ' mm/s（'+fmt(firstContact.actual_before_time_sec,6)+' s）\n接触时 Fx / Fy / Fxy：'+array(firstContact.contact_force_xy_N)+' N\n'+
    (firstContact.after_contact||[]).map(p=>'接触后 '+fmt(p.sample_time_sec-firstContact.time_sec,3)+' s 实际 vx / vy / XY：'+array(p.actual_xy_mm_s)+' mm/s').join('\n')+
    (!firstContact.precontact_one_second_available?'\n本次未记录足够的接触前 1 秒实际速度。':'');
}else $('collision').textContent='没有记录首次接触事件，未推测碰撞时刻。';
$('note').textContent='内嵌全部 '+D.sample_count+' 帧和原始发送时间戳。全局显示按像素保留峰值、突变与缺失段；局部放大显示原始采样。悬停读数取不晚于所选时刻的真实样本，不插值。'+
  (firstContact.source&&firstContact.source.startsWith('legacy')?' 本次只有接触确认，物理碰撞起始未知。':' 接触阈值使用滤波信号，物理碰撞可能更早。')+
  '主时间轴滚轮缩放、Shift+拖动平移；完整实验时间按钮恢复全局。';
window.addEventListener('resize',()=>dirty=true);
render();requestAnimationFrame(frame);
window.experimentReplay={seek,render,frame,setRate,resetView,wheelView,niceTicks,peakPoints,data:D,views,geometry,
  get time(){return elapsed;},get rate(){return rate;},get playing(){return playing;}};
