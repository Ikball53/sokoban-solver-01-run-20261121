import{readdirSync as R,readFileSync as F,mkdirSync as M,writeFileSync as W}from'fs'
let D=[[-1,0,'u'],[1,0,'d'],[0,-1,'l'],[0,1,'r']],Wd,H
let nb=(i,k)=>{let r=(i/Wd|0)+D[k][0],c=i%Wd+D[k][1];return r<0||c<0||r>=H||c>=Wd?-1:r*Wd+c}
let fl=(s,bs,ok)=>{let m=new Map([[s,-1]]),q=[s];for(let x of q)for(let k=0;k<4;k++){let y=nb(x,k);if(y>=0&&ok(y)&&!bs.has(y)&&!m.has(y)){m.set(y,x*4+k);q.push(y)}}return m}
let pt=(m,t)=>{let s='';for(let v;(v=m.get(t))>=0;t=v/4|0)s=D[v%4][2]+s;return s}
let so=(a,b)=>a-b
let sr=(cells,bx,gl,st,en,nx)=>{
let cs=new Set(cells),gs=new Set(gl),lv=new Set(gl),q=[...gl]
while(q.length){let x=q.pop();for(let k=0;k<4;k++){let b=nb(x,k^1),s=b<0?-1:nb(b,k^1);if(b>=0&&s>=0&&cs.has(b)&&cs.has(s)&&b!=en&&!lv.has(b)){lv.add(b);q.push(b)}}}
let nd=[{b:[...bx].sort(so),p:st,par:-1,mv:''}],sn=new Set()
let ky=n=>n.b.join(',')+'|'+Math.min(...fl(n.p,new Set(n.b),y=>cs.has(y)).keys())
sn.add(ky(nd[0]))
for(let j=0;j<nd.length&&nd.length<4e6;j++){let n=nd[j],bs=new Set(n.b),m=fl(n.p,bs,y=>cs.has(y))
if(n.b.every(b=>gs.has(b))&&(!nx||m.has(en))){let mv='',e=n;while(e.par>=0){mv=e.mv+mv;e=nd[e.par]}return{mv,p:n.p,b:n.b}}
for(let bi=0;bi<n.b.length;bi++){let b=n.b[bi];for(let k=0;k<4;k++){let ds=nb(b,k),sd=nb(b,k^1)
if(ds<0||sd<0||!cs.has(ds)||ds==en||bs.has(ds)||!lv.has(ds)||!m.has(sd))continue
let b2=[...n.b];b2[bi]=ds;b2.sort(so)
let nn={b:b2,p:b,par:j,mv:pt(m,sd)+D[k][2]},kk=ky(nn)
if(!sn.has(kk)){sn.add(kk);nd.push(nn)}}}}
return null}
let sl=t=>{
let L=t.split('\n');while(L.length&&!L.at(-1))L.pop()
H=L.length;Wd=Math.max(...L.map(s=>s.length))
let wl=new Set(),gl=new Set(),bx=[],pl=0
L.forEach((row,r)=>{for(let c=0;c<Wd;c++){let ch=row[c]||'#',i=r*Wd+c
if(ch=='#')wl.add(i);else{if(ch=='.'||ch=='*'||ch=='+')gl.add(i);if(ch=='$'||ch=='*')bx.push(i);if(ch=='@'||ch=='+')pl=i}}})
let pr=pl/Wd|0,sc=new Set(),rms=[]
for(let i=0;i<H*Wd;i++){if(wl.has(i)||(i/Wd|0)==pr||sc.has(i))continue
let cells=[],q=[i];sc.add(i)
while(q.length){let x=q.pop();cells.push(x);for(let k=0;k<4;k++){let y=nb(x,k);if(y>=0&&!wl.has(y)&&(y/Wd|0)!=pr&&!sc.has(y)){sc.add(y);q.push(y)}}}
let rb=bx.filter(b=>cells.includes(b)),rg=[...gl].filter(g=>cells.includes(g))
if(rb.length)rms.push({cells,rb,rg})}
let tot=rms.reduce((a,r)=>a+r.rb.length,0),okAll=tot==bx.length&&rms.every(r=>r.rb.length==r.rg.length)
if(okAll)for(let r of rms){r.en=-1;for(let c of r.cells)for(let k=0;k<4;k++){let y=nb(c,k);if(y>=0&&(y/Wd|0)==pr&&!wl.has(y))r.en=y}
if(r.en<0)okAll=0;r.all=[...r.cells,r.en]}
if(!okAll)return(sr([...Array(H*Wd).keys()].filter(i=>!wl.has(i)),bx,[...gl],pl,-1,0)||{mv:''}).mv
for(let r of rms){r.sol=sr(r.all,r.rb,r.rg,r.en,r.en,0);if(!r.sol)return''}
for(let r of rms)r.ex=fl(r.sol.p,new Set(r.sol.b),y=>r.all.includes(y)).has(r.en)
let bad=rms.filter(r=>!r.ex)
if(bad.length>1)for(let r of bad.slice(1)){r.sol=sr(r.all,r.rb,r.rg,r.en,r.en,1)||r.sol;r.ex=1}
rms.sort((a,b)=>b.ex-a.ex)
let cur=pl,cb=new Set(bx),out=''
for(let r of rms){out+=pt(fl(cur,cb,y=>!wl.has(y)),r.en)+r.sol.mv;cur=r.sol.p;r.rb.forEach(b=>cb.delete(b));r.sol.b.forEach(b=>cb.add(b))}
return out}
for(let id of R('/input').sort()){M('/output/'+id,{recursive:true});W('/output/'+id+'/solution.txt',sl(F('/input/'+id+'/level.txt','utf8')))}
