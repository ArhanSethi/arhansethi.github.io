/* Portfolio interactions. No trackers, API keys, or external runtime dependencies. */
'use strict';
const $ = (id) => document.getElementById(id);

// A deterministic, 2D port of the Python Path Lab extension.
const W=1000,H=470,CLEARANCE=8,START=[60,235],GOAL=[940,235];
const canvas=$('planner-canvas'),ctx=canvas.getContext('2d');
let obstacles=[{x:310,y:0,w:48,h:305},{x:620,y:170,w:48,h:300}];
let nodes=[],algorithm='RRT*',iterations=0,running=false,drag=null,path=[],smoothed=[];
let seed=42;
function random(){seed|=0;seed=seed+0x6D2B79F5|0;let t=Math.imul(seed^seed>>>15,1|seed);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296;}
const distance=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
function free(p){return p[0]>=CLEARANCE&&p[0]<=W-CLEARANCE&&p[1]>=CLEARANCE&&p[1]<=H-CLEARANCE&&!obstacles.some(r=>p[0]>=r.x-CLEARANCE&&p[0]<=r.x+r.w+CLEARANCE&&p[1]>=r.y-CLEARANCE&&p[1]<=r.y+r.h+CLEARANCE);}
function visible(a,b){
 if(!free(a)||!free(b))return false;
 for(const r of obstacles){let lo=0,hi=1;for(const [o,d,min,max] of [[a[0],b[0]-a[0],r.x-CLEARANCE,r.x+r.w+CLEARANCE],[a[1],b[1]-a[1],r.y-CLEARANCE,r.y+r.h+CLEARANCE]]){
  if(Math.abs(d)<1e-12){if(o<min||o>max){lo=1;hi=0;break;}}
  else{const t1=(min-o)/d,t2=(max-o)/d;lo=Math.max(lo,Math.min(t1,t2));hi=Math.min(hi,Math.max(t1,t2));}
 }if(lo<=hi)return false;}return true;
}
function cost(i){let total=0;while(nodes[i].parent!==-1){const p=nodes[i].parent;total+=distance(nodes[i].p,nodes[p].p);i=p;}return total;}
function step(){
 iterations++;
 const sample=random()<.08?GOAL:[random()*W,random()*H];let nearest=0,d=Infinity;
 nodes.forEach((n,i)=>{const x=distance(n.p,sample);if(x<d){nearest=i;d=x;}});
 if(d<1e-8)return;const ratio=Math.min(1,34/d),q=nodes[nearest].p.map((v,i)=>v+(sample[i]-v)*ratio);
 if(!visible(nodes[nearest].p,q))return;
 let parent=nearest,best=cost(nearest)+distance(nodes[nearest].p,q),neighbors=[];
 if(algorithm==='RRT*')nodes.forEach((n,i)=>{if(distance(n.p,q)<90)neighbors.push(i);});
 for(const i of neighbors){const candidate=cost(i)+distance(nodes[i].p,q);if(candidate<best&&visible(nodes[i].p,q)){best=candidate;parent=i;}}
 const ni=nodes.length;nodes.push({p:q,parent});
 for(const i of neighbors){if(nodes[i].parent!==-1&&best+distance(q,nodes[i].p)+1e-9<cost(i)&&visible(q,nodes[i].p))nodes[i].parent=ni;}
}
function currentPath(){
 let best=Infinity,chosen=-1;
 nodes.forEach((n,i)=>{const d=distance(n.p,GOAL);if(d<90&&visible(n.p,GOAL)){const c=cost(i)+d;if(c<best){best=c;chosen=i;}}});
 if(chosen<0)return[];const p=[GOAL];while(chosen!==-1){p.push(nodes[chosen].p);chosen=nodes[chosen].parent;}return p.reverse();
}
function shortcut(p){if(!p.length)return[];const out=[p[0]];let i=0;while(i<p.length-1){let j=p.length-1;while(j>i+1&&!visible(p[i],p[j]))j--;out.push(p[j]);i=j;}return out;}
function pathLength(p){return p.reduce((sum,v,i)=>sum+(i?distance(p[i-1],v):0),0);}
function line(p,color,width){if(p.length<2)return;ctx.beginPath();ctx.moveTo(...p[0]);p.slice(1).forEach(v=>ctx.lineTo(...v));ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
function draw(){
 ctx.clearRect(0,0,W,H);ctx.fillStyle='#f4f4f1';ctx.fillRect(0,0,W,H);ctx.strokeStyle='#e4e4e0';ctx.lineWidth=.55;
 for(let x=20;x<W;x+=25){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();}for(let y=10;y<H;y+=25){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}
 for(const n of nodes){if(n.parent!==-1)line([n.p,nodes[n.parent].p],'#c1c1b9',.9);}
 ctx.fillStyle='#242420';for(const r of obstacles)ctx.fillRect(r.x,r.y,r.w,r.h);
 line(path,'#ada99d',2);line(smoothed,'#a53a25',3.5);
 for(const [p,label] of [[START,'A'],[GOAL,'B']]){ctx.beginPath();ctx.arc(...p,14,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='#111';ctx.lineWidth=1.5;ctx.stroke();ctx.fillStyle='#111';ctx.font='15px Helvetica,Arial,sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(label,p[0],p[1]+1);}
 if(drag){const r=dragRect();ctx.setLineDash([6,4]);ctx.strokeStyle='#a53a25';ctx.strokeRect(r.x,r.y,r.w,r.h);ctx.setLineDash([]);}
}
function sync(){path=currentPath();smoothed=shortcut(path);$('plan-nodes').textContent=nodes.length;$('plan-length').textContent=path.length?`${Math.round(pathLength(smoothed))} px`:'—';draw();}
function reset(message='Ready. Give the algorithm something to work around.'){
 running=false;seed=42;iterations=0;nodes=[{p:START,parent:-1}];path=[];smoothed=[];$('plan-run').textContent='Run planner ↗';$('plan-status').textContent=message;$('plan-nodes').textContent='1';$('plan-length').textContent='—';draw();
}
function resize(){const rect=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);if(!rect.width)return;canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.width*H/W*dpr);ctx.setTransform(canvas.width/W,0,0,canvas.height/H,0,0);draw();}
$('plan-run').onclick=()=>{if(iterations>=1600)reset();running=!running;$('plan-run').textContent=running?'Pause':'Run planner ↗';$('plan-status').textContent=running?'Finding a route. Please hold, briefly.':'Paused. The obstacles can wait.';};
$('plan-reset').onclick=()=>reset();$('plan-clear').onclick=()=>{obstacles=[];reset('Obstacles cleared. A surprisingly straightforward assignment.');};
document.querySelectorAll('[data-algorithm]').forEach(b=>b.onclick=()=>{algorithm=b.dataset.algorithm;document.querySelectorAll('[data-algorithm]').forEach(x=>{const on=x===b;x.classList.toggle('selected',on);x.setAttribute('aria-pressed',on);});reset(`${algorithm} selected. Same map. Same seed.`);});
function point(e){const r=canvas.getBoundingClientRect();return[Math.max(0,Math.min(W,(e.clientX-r.left)*W/r.width)),Math.max(0,Math.min(H,(e.clientY-r.top)*H/r.height))];}
function dragRect(){return{x:Math.min(drag.a[0],drag.b[0]),y:Math.min(drag.a[1],drag.b[1]),w:Math.abs(drag.a[0]-drag.b[0]),h:Math.abs(drag.a[1]-drag.b[1])};}
function addObstacle(r){if(r.w<8||r.h<8)return;obstacles.push(r);if(!free(START)||!free(GOAL)){obstacles.pop();reset('Keep a little space around A and B.');}else reset('Obstacle added. It’s a planning problem now.');}
canvas.oncontextmenu=e=>e.preventDefault();canvas.onpointerdown=e=>{if(e.button!==0&&e.button!==2)return;const p=point(e);if(e.button===2){obstacles=obstacles.filter(r=>!(p[0]>=r.x&&p[0]<=r.x+r.w&&p[1]>=r.y&&p[1]<=r.y+r.h));reset('Obstacle removed.');return;}reset();drag={a:p,b:p};canvas.setPointerCapture(e.pointerId);draw();};canvas.onpointermove=e=>{if(drag){drag.b=point(e);draw();}};canvas.onpointerup=()=>{if(drag){const r=dragRect();drag=null;addObstacle(r);draw();}};canvas.onpointercancel=()=>{drag=null;draw();};
let added=0;$('plan-add').onclick=()=>{const positions=[[445,185],[190,90],[750,280],[470,345],[170,330]];const[x,y]=positions[added++%positions.length];addObstacle({x,y,w:70,h:65});};
function animate(){if(running){for(let i=0;i<12&&iterations<1600;i++)step();sync();if(iterations>=1600){running=false;$('plan-run').textContent='Run again ↗';$('plan-status').textContent=path.length?'Route found. The scenic route has been declined.':'No route found in 1,600 samples. Try clearing an obstacle.';}}requestAnimationFrame(animate);}
reset();new ResizeObserver(resize).observe(canvas);requestAnimationFrame(animate);

// A labeled concept diagram, deliberately not masquerading as live CLIP output.
const svgNS='http://www.w3.org/2000/svg';
function svg(tag,attributes,text){const el=document.createElementNS(svgNS,tag);Object.entries(attributes).forEach(([k,v])=>el.setAttribute(k,v));if(text!==undefined)el.textContent=text;return el;}
function fusion(){const plot=$('fusion-plot');plot.replaceChildren();const t=Number($('text-weight').value)/100;$('fusion-weight').textContent=`${100-Math.round(t*100)} / ${Math.round(t*100)}`;
 plot.append(svg('path',{d:'M25 15V195H290',stroke:'#c6c6bf',fill:'none'}));
 for(let i=0;i<23;i++){const x=45+(Math.sin(i*18.7)+1)*108,y=35+(Math.cos(i*5.1)+1)*65;plot.append(svg('circle',{cx:x,cy:y,r:2.5,fill:'#b7b7af'}));}
 plot.append(svg('line',{x1:65,y1:145,x2:245,y2:55,stroke:'#999','stroke-dasharray':'4 5'}));
 plot.append(svg('circle',{cx:65,cy:145,r:5,fill:'#111'}),svg('circle',{cx:245,cy:55,r:5,fill:'#111'}));
 plot.append(svg('text',{x:40,y:175,'font-size':16,fill:'#555'},'image'),svg('text',{x:236,y:34,'font-size':16,fill:'#555'},'text'));
 const x=65+180*t,y=145-90*t;plot.append(svg('circle',{cx:x,cy:y,r:12,fill:'#f4f4f1',stroke:'#a53a25','stroke-width':2}),svg('circle',{cx:x,cy:y,r:3,fill:'#a53a25'}));
}
$('text-weight').oninput=fusion;fusion();

// Tiny illustrative human review: no storage or paid model calls.
document.querySelectorAll('[data-vote]').forEach(b=>b.onclick=()=>{document.querySelectorAll('[data-vote]').forEach(x=>{x.classList.toggle('selected',x===b);x.setAttribute('aria-pressed',x===b);});const pick=b.dataset.vote;$('review-result').textContent=(pick==='B'?'B is correct: halving the search interval gives logarithmic complexity.':pick==='A'?'A misses the key distinction: binary search halves the interval; it doesn’t inspect every element.':'A useful disagreement to inspect: B gives the correct logarithmic complexity.')+' This is an illustrative pair; your vote isn’t saved.';});

// Actual snapshots generated by the upstream C++ engine from synthetic events.
let snapshots=[],bookIndex=0,bookPlaying=false,lastBookTime=0;
const money=n=>(n/10000).toFixed(4);
function bookRender(){const s=snapshots[bookIndex];if(!s)return;$('book-event').textContent=String(bookIndex+1).padStart(3,'0');$('book-progress').textContent=`${bookIndex+1} / ${snapshots.length}`;$('book-seek').value=bookIndex;$('book-spread').textContent=s.bids.length&&s.asks.length?'$'+money(s.asks[0].price-s.bids[0].price):'—';for(const side of ['bids','asks']){const target=$('book-'+side);target.replaceChildren();const levels=s[side].slice(0,6),max=Math.max(1,...levels.map(x=>x.quantity));for(let i=0;i<6;i++){const row=document.createElement('div');row.className='book-row';const p=document.createElement('span'),q=document.createElement('span'),level=levels[i];p.textContent=level?money(level.price):'—';q.textContent=level?level.quantity.toLocaleString():'—';row.style.setProperty('--depth',level?`${level.quantity/max*100}%`:'0%');row.append(p,q);target.append(row);}}}
$('book-play').onclick=()=>{if(!snapshots.length)return;if(bookIndex>=snapshots.length-1)bookIndex=0;bookPlaying=!bookPlaying;lastBookTime=0;$('book-play').textContent=bookPlaying?'Pause session':'Play session ↗';bookRender();};$('book-seek').oninput=()=>{bookPlaying=false;$('book-play').textContent='Play session ↗';bookIndex=Number($('book-seek').value);bookRender();};
function bookTick(t){if(bookPlaying){if(!lastBookTime)lastBookTime=t;if(t-lastBookTime>=180){bookIndex=Math.min(bookIndex+1,snapshots.length-1);lastBookTime=t;bookRender();if(bookIndex===snapshots.length-1){bookPlaying=false;$('book-play').textContent='Replay session ↗';}}}requestAnimationFrame(bookTick);}
fetch('orderbook-data.json').then(r=>{if(!r.ok)throw Error('Session unavailable');return r.json();}).then(rows=>{snapshots=rows;$('book-seek').max=rows.length-1;bookIndex=Math.min(45,rows.length-1);bookRender();requestAnimationFrame(bookTick);}).catch(()=>{$('book-play').disabled=true;$('book-play').textContent='Session unavailable';});

// Preserve the two-frame layout while opening complete, keyboard-accessible projects.
const dialog=$('project-dialog'), projectHome=$('project-content');
let activeTile=null;
document.querySelectorAll('[data-project]').forEach(tile=>tile.addEventListener('click',()=>{
 activeTile=tile;
 const article=$(tile.dataset.project);
 dialog.querySelector('.dialog-content').append(article);
 dialog.setAttribute('aria-label',article.querySelector('h3').textContent);
 document.body.classList.add('project-open');
 dialog.showModal();dialog.scrollTop=0;requestAnimationFrame(resize);
}));
dialog.querySelector('.close-project').onclick=()=>dialog.close();
dialog.addEventListener('click',event=>{if(event.target===dialog){const r=dialog.getBoundingClientRect();if(event.clientX<r.left||event.clientX>r.right||event.clientY<r.top||event.clientY>r.bottom)dialog.close();}});
dialog.addEventListener('close',()=>{
 const article=dialog.querySelector('article');if(article)projectHome.append(article);
 running=false;bookPlaying=false;$('plan-run').textContent='Run planner ↗';$('book-play').textContent='Play session ↗';
 document.body.classList.remove('project-open');activeTile?.focus({preventScroll:true});
});
// One wheel gesture advances one frame; touch and keyboard retain native scroll snap.
let wheelLock=0,wheelTotal=0,wheelLast=0;
window.addEventListener('wheel',event=>{
 if(dialog.open||event.ctrlKey||Math.abs(event.deltaX)>Math.abs(event.deltaY))return;
 const overview=$('overview');
 if(overview.scrollHeight>overview.clientHeight+2&&window.scrollY>50)return;
 event.preventDefault();
 const now=performance.now();if(now<wheelLock)return;
 if(now-wheelLast>180)wheelTotal=0;wheelLast=now;wheelTotal+=event.deltaY;
 if(Math.abs(wheelTotal)<20)return;
 (wheelTotal>0?overview:$('opening')).scrollIntoView({behavior:matchMedia('(prefers-reduced-motion: reduce)').matches?'instant':'smooth'});
 wheelTotal=0;wheelLock=now+800;
},{passive:false});
