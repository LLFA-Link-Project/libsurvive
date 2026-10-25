/* Local recorded-pose viewer. No external assets, analytics, or runtime deps. */
'use strict';
const $ = id => document.getElementById(id);
const A_COLOR = '#58d9c0', B_COLOR = '#ffb877';
const AXIS_COLORS = ['#ff8b8b', '#8fdd97', '#83b9ff'];
const S = {sources: [], meta: {}, data: {}, generation: {A: 0, B: 0},
  windowGeneration: 0,
  time: 260, lo: 260, hi: 266, playing: false, lastFrame: 0,
  camera: {yaw: -.7, pitch: .48, zoom: 1, target: [0,0,0]}, drag: null, loading: false};
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const num = (id, fallback = 0) => Number.isFinite(+$(id).value) ? +$(id).value : fallback;
const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
const sub = (a, b) => a.map((x, i) => x-b[i]);
const add = (a, b) => a.map((x, i) => x+b[i]);
const scale = (a, s) => a.map(x => x*s);
const norm = a => Math.hypot(...a);
const dot = (a, b) => a.reduce((s, x, i) => s+x*b[i], 0);
const inv = q => [q[0], -q[1], -q[2], -q[3]];
function mul(a, b) {
  const [w,x,y,z] = a, [v,i,j,k] = b;
  return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j, w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v];
}
const rot = (q, v) => mul(mul(q, [0,...v]), inv(q)).slice(1);
function qpower(q, factor) {
  if (q[0] < 0) q = q.map(x => -x);
  const angle = Math.acos(clamp(q[0], -1, 1));
  const n = Math.hypot(...q.slice(1));
  return n < 1e-12 ? [1,0,0,0] : [Math.cos(angle*factor), ...scale(q.slice(1), Math.sin(angle*factor)/n)];
}
function slerp(a, b, t) {
  if (dot(a,b) < 0) b = scale(b,-1);
  const d = clamp(dot(a,b),-1,1);
  if (d > .9995) { const q = a.map((x,i) => x+(b[i]-x)*t); return scale(q, 1/norm(q)); }
  const angle = Math.acos(d), s = Math.sin(angle);
  return a.map((x,i) => (x*Math.sin((1-t)*angle)+b[i]*Math.sin(t*angle))/s);
}
function rotationVector(q) {
  if (q[0] < 0) q = scale(q,-1);
  const n = norm(q.slice(1));
  return n < 1e-12 ? [0,0,0] : scale(q.slice(1), 2*Math.atan2(n,q[0])/n*180/Math.PI);
}
const position = row => row.slice(3,6);
const quaternion = row => row.slice(6,10);
const valid = row => row && row[20] === 1;
function dt(a,b) {
  return a[1] !== null && b[1] !== null && a[2] > 0 && a[2] === b[2] ? (b[1]-a[1])/a[2] : b[0]-a[0];
}
const connected = (a,b) => valid(a) && valid(b) && dt(a,b) > 0 && dt(a,b) <= .02;
function lowerBound(rows, t) {
  let lo=0, hi=rows.length;
  while(lo<hi) { const m=(lo+hi)>>>1; if(rows[m][0]<t) lo=m+1; else hi=m; }
  return lo;
}
function errorMessage(error) {
  $('status').textContent = error.message || String(error);
  $('status').className = 'error';
}
const action = fn => async event => { try { await fn(event); } catch(error) { errorMessage(error); } };
async function api(path, query={}) {
  const response = await fetch(path+'?'+new URLSearchParams(query));
  const data = await response.json();
  if (!response.ok || data.error) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}
function status(message) { $('status').textContent = message; $('status').className = ''; }
function sourceOptions(side) {
  const select = $('source'+side), previous = select.value;
  const filter = $('search').value.toLowerCase();
  select.replaceChildren();
  const groups = new Map();
  for (const source of S.sources) {
    if (!source.label.toLowerCase().includes(filter) && source.id !== previous) continue;
    if (!groups.has(source.group)) {
      const group = document.createElement('optgroup'); group.label = source.group;
      select.append(group); groups.set(source.group,group);
    }
    const option = new Option(source.label, source.id);
    groups.get(source.group).append(option);
  }
  if ([...select.options].some(o=>o.value===previous)) select.value=previous;
}
function describe(meta, side) {
  const div = $('meta'+side);
  div.replaceChildren();
  const addLine = text => { const p=document.createElement('div'); p.textContent=text; div.append(p); };
  addLine(`${meta.source.manifest_status} · ${(meta.source_bytes/1e6).toFixed(1)} MB`);
  addLine(`ストリーム: ${meta.health.stream_integrity} · 解析 / 成分エラー: ${meta.health.damaged_rows} 行`);
  if (meta.health.stream_error) addLine(meta.health.stream_error);
  for(const warning of meta.health.metadata_warnings||[])addLine(warning);
  if (meta.source.revision) addLine(`core: ${meta.source.revision.slice(0,12)}`);
  const track = selectedTrack(side);
  if (track) { addLine(`${track.frame} / ${track.reference}`); addLine(track.clock); addLine(`無効姿勢: ${track.invalid} サンプル`); }
  addLine(`SHA-256: ${meta.source_sha256.slice(0,16)}…`);
}
const selectedTrack = side => S.meta[side]?.tracks.find(x=>x.id===$('track'+side).value);
function updateTrack(side) {
  describe(S.meta[side],side);
  if (side==='A') {
    const track=selectedTrack('A');
    if (!track) return;
    $('overview').min=track.start; $('overview').max=track.end; $('overview').value=num('lo');
    $('extent').textContent=`${track.start.toFixed(3)} – ${track.end.toFixed(3)} 秒 · ${track.count.toLocaleString()} サンプル`;
  }
}
async function prepare(side, preferred) {
  const id=$('source'+side).value, generation=++S.generation[side];
  ++S.windowGeneration;pause();
  $('track'+side).replaceChildren(new Option('インデックスを準備中…',''));
  S.data[side]=null;
  S.plotCache=null;
  for (;;) {
    const meta=await api('/api/source',{id});
    if (generation!==S.generation[side]) return false;
    if (meta.state==='error') throw new Error(meta.error);
    if (meta.state==='ready') {
      S.meta[side]=meta;
      const select=$('track'+side); select.replaceChildren();
      for (const track of meta.tracks) select.append(new Option(track.label,track.id));
      const preferredTrack=meta.tracks.find(t=>t.id===preferred) || meta.tracks.find(t=>t.serial===preferred && t.stage==='POSE_STATE') ||
        meta.tracks.find(t=>t.serial==='LHR-C6F86C46' && t.stage==='POSE_STATE') ||
        meta.tracks.find(t=>t.serial==='LHR-C6F86C46' && t.stage==='backend_received') || meta.tracks[0];
      if (!preferredTrack) throw new Error('このストリームに表示可能な姿勢がありません。');
      select.value=preferredTrack.id;
      updateTrack(side); updateMarkers(); updateCompareNotice();
      return true;
    }
    status(`${side}: ${meta.queued?'順番待ち':'原記録を読み取り、検索用キャッシュを作成中'}。初回だけ記録の長さに応じて時間がかかります。`);
    await sleep(1200);
  }
}
function updateMarkers() {
  const select=$('markers'); select.replaceChildren(new Option('マーカーを選択…',''));
  for(const m of S.meta.A?.markers_raw || []) {
    if (Number.isFinite(m.viewer_time_s)) select.append(new Option(`${m.viewer_time_s.toFixed(3)} s · ${m.label || m.name || m.event || 'marker'}`,m.viewer_time_s));
  }
}
function updateCompareNotice() {
  const a=S.meta.A?.source,b=S.meta.B?.source;
  if (!a||!b) return;
  $('compareNotice').textContent=a.input_group && a.input_group===b.input_group ?
    '同じ入力記録の受信秒で再生します。位置はそれぞれの区間先頭を原点とします。再較正した出力では world の向きが異なる場合があります。' :
    '別の記録または別の時計です。B の時刻 = A + 時刻差。各先頭を原点にして動きの特徴を見比べます。同じ物理動作・座標・真値の比較とは扱いません。';
}
function toggleCompare() {
  const enabled=$('compare').checked;
  $('compareControls').hidden=!enabled; $('panelB').hidden=!enabled;
  $('scenes').classList.toggle('comparing',enabled);
}
async function loadWindow({selection=true, time=null, fit=true}={}) {
  const generation=++S.windowGeneration;
  const lo=num('lo'),hi=num('hi');
  if (!(hi>lo&&hi-lo<=60)) throw new Error('開始 < 終了、区間の長さは最大 60 秒にしてください。');
  pause(); S.loading=true; $('load').disabled=true;
  const sides=$('compare').checked?['A','B']:['A'];
  try {
    for (const side of sides) if (!selectedTrack(side)) throw new Error(`${side} のデバイスを選択してください。`);
    const data=await Promise.all(sides.map(side=>{
      const shift=side==='B'?num('offsetB'):0;
      return api('/api/window',{id:$('source'+side).value,track:$('track'+side).value,lo:lo+shift,hi:hi+shift});
    }));
    if(generation!==S.windowGeneration)return;
    sides.forEach((side,i)=>{
      if(data[i].state!=='ready') throw new Error('インデックス作成が終わるまで待ってください。');
      const d=data[i]; d.offset=side==='B'?num('offsetB'):0;
      d.sourceId=$('source'+side).value;
      d.anchor=d.rows.find(valid); d.label=S.meta[side].source.label;
      S.data[side]=d;
    });
    S.lo=lo; S.hi=hi; S.time=clamp(time??S.data.A.anchor?.[0]??lo,lo,hi);
    $('scrub').min=lo; $('scrub').max=hi; $('scrub').value=S.time;
    $('overview').value=lo;
    if(selection){$('selectLo').value=lo; $('selectHi').value=hi;}
    $('windowTitle').textContent=`${lo.toFixed(3)} – ${hi.toFixed(3)} 秒`;
    $('sourceSummary').textContent=sides.map(side=>{const s=S.meta[side].source;return `${side}: ${s.label.includes(s.session)?s.label:s.session+' · '+s.label}`;}).join(' / ');
    $('sampleBadge').textContent=`間引きなし · ${data.reduce((s,d)=>s+d.rows.length,0).toLocaleString()} samples`;
    const warnings=[];
    for(const side of sides) {
      const meta=S.meta[side], d=S.data[side];
      $('title'+side).textContent=`${side} · ${d.track.serial} · ${d.track.stage}`;
      $('title'+side).title=d.label;
      if (!d.rows.length) warnings.push(`${side}: この範囲には姿勢がありません`);
      if(meta.health.stream_integrity!=='complete'||meta.health.damaged_rows) warnings.push(`${side}: 元記録の不完全 / 解析・成分エラー情報あり`);
      if(meta.source.manifest_status!=='complete'&&meta.source.group==='recordings') warnings.push(`${side}: manifest ${meta.source.manifest_status}`);
    }
    status(warnings.length?warnings.join(' · '):'原記録のサンプルを表示しています。選んだ区間を低速再生し、グラフの範囲をコピーできます。');
    if(fit)fitCamera();
    rebuild(); updateShare();
  } finally { if(generation===S.windowGeneration){S.loading=false; $('load').disabled=false;} }
}
function basisVector(v,d) { return $('basis').value==='body'?rot(inv(quaternion(d.anchor)),v):v; }
function displacement(row,d) {return basisVector(sub(position(row),position(d.anchor)),d);}
function angleVector(row,d) {return basisVector(rotationVector(mul(quaternion(row),inv(quaternion(d.anchor)))),d);}
function sampleAt(d,t) {
  if(!d?.rows.length) return null;
  const target=t+d.offset, i=lowerBound(d.rows,target);
  if(i<d.rows.length&&d.rows[i][0]===target) return d.rows[i];
  const a=d.rows[i-1], b=d.rows[i];
  if(!valid(a)||target-a[0]>.02) return null;
  if($('interpolation').value==='linear'&&b&&connected(a,b)&&b[0]>a[0]) {
    const f=(target-a[0])/(b[0]-a[0]), row=a.slice();
    row.splice(3,3,...position(a).map((x,j)=>x+(b[3+j]-x)*f));
    row.splice(6,4,...slerp(quaternion(a),quaternion(b),f));
    return row;
  }
  return a;
}
function canvasContext(canvas) {
  const rect=canvas.getBoundingClientRect(), ratio=Math.min(devicePixelRatio||1,2);
  const w=Math.max(1,rect.width),h=Math.max(1,rect.height);
  if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)) {canvas.width=Math.round(w*ratio); canvas.height=Math.round(h*ratio);}
  const ctx=canvas.getContext('2d'); ctx.setTransform(ratio,0,0,ratio,0,0);
  return {ctx,w,h};
}
function visualVector(v,d) {
  if($('basis').value==='world'&&/Y-up|y_up/.test(d.track.frame)) return [v[0],-v[2],v[1]];
  return v;
}
function fitCamera() {
  const min=[0,0,0],max=[0,0,0];
  for(const side of ($('compare').checked?['A','B']:['A'])) {
    const d=S.data[side];if(!d?.anchor)continue;
    for(const row of d.rows)if(valid(row)) {
      const p=visualVector(scale(displacement(row,d),num('gain',30)),d);
      p.forEach((x,i)=>{min[i]=Math.min(min[i],x);max[i]=Math.max(max[i],x);});
    }
  }
  S.camera.target=min.map((x,i)=>(x+max[i])/2);
  const radius=norm(sub(max,min))/2;
  const span=$('view').value==='micro'?.12:3;
  S.camera.zoom=clamp(span/Math.max(span,radius*2.6),1e-6,20);
}
function drawScene(side) {
  const canvas=$('scene'+side), {ctx,w,h}=canvasContext(canvas),d=S.data[side];
  ctx.fillStyle='#0e1823';ctx.fillRect(0,0,w,h);
  if(!d?.anchor) {ctx.fillStyle='#91a5b9';ctx.font='13px system-ui';ctx.fillText('この区間に有効な姿勢がありません',24,h/2);return;}
  const gain=num('gain',30),micro=$('view').value==='micro';
  const span=micro?.12:3, pixels=Math.min(w,h)/span*S.camera.zoom;
  const {yaw,pitch}=S.camera;
  const project=v=>{
    const [x,y,z]=sub(visualVector(v,d),S.camera.target),u=x*Math.cos(yaw)-y*Math.sin(yaw),depth=x*Math.sin(yaw)+y*Math.cos(yaw);
    return [w/2+u*pixels,h*.53+(depth*Math.sin(pitch)-z*Math.cos(pitch))*pixels];
  };
  function line(points,color,width=1,dashed=false) {
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dashed?[4,5]:[]);ctx.beginPath();
    points.forEach((v,i)=>{const p=project(v);i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.stroke();ctx.setLineDash([]);
  }
  const idealGrid=span/S.camera.zoom/12,grid=10**Math.floor(Math.log10(idealGrid))*Math.ceil(idealGrid/10**Math.floor(Math.log10(idealGrid)));
  const gridExtent=grid*12,cx=Math.round(S.camera.target[0]/grid)*grid,cy=Math.round(S.camera.target[1]/grid)*grid;
  const fromVisual=v=>$('basis').value==='world'&&/Y-up|y_up/.test(d.track.frame)?[v[0],v[2],-v[1]]:v;
  for(let i=-12;i<=12;i++) {
    const t=i*grid;
    line([fromVisual([cx-gridExtent,cy+t,0]),fromVisual([cx+gridExtent,cy+t,0])],'#1d3040');
    line([fromVisual([cx+t,cy-gridExtent,0]),fromVisual([cx+t,cy+gridExtent,0])],'#1d3040');
  }
  const axisLength=micro?.025/S.camera.zoom:.35;
  ['X','Y','Z'].forEach((label,i)=>{const v=[0,0,0];v[i]=axisLength;line([[0,0,0],v],AXIS_COLORS[i],1.5);ctx.fillStyle=AXIS_COLORS[i];ctx.font='11px ui-monospace,monospace';ctx.fillText(label,...project(scale(v,1.1)));});
  const trail=num('trail',1),start=S.time-trail+d.offset,end=S.time+d.offset;
  let previous=null;
  ctx.strokeStyle=side==='A'?A_COLOR:B_COLOR;ctx.lineWidth=1.2;ctx.globalAlpha=.6;ctx.beginPath();
  for(let i=lowerBound(d.rows,start);i<d.rows.length&&d.rows[i][0]<=end;i++) {
    const row=d.rows[i]; if(!valid(row)){previous=null;continue;}
    const p=project(scale(displacement(row,d),gain));
    if(previous&&connected(previous,row))ctx.lineTo(...p);else ctx.moveTo(...p);
    previous=row;
  }
  ctx.stroke();ctx.globalAlpha=1;
  const drawBox=(p,q,color,ghost=false)=>{
    const size=micro?.009/S.camera.zoom:.045;
    const vertices=[];
    for(let i=0;i<8;i++) {const v=[i&1?size:-size,i&2?size*.7:-size*.7,i&4?size*.45:-size*.45];vertices.push(add(p,basisVector(rot(q,v),d)));}
    for(let i=0;i<8;i++)for(const bit of [1,2,4])if(!(i&bit))line([vertices[i],vertices[i|bit]],color,ghost?1:2,ghost);
    if(!ghost) {
      for(let i=0;i<3;i++){const v=[0,0,0];v[i]=size*2;line([p,add(p,basisVector(rot(q,v),d))],AXIS_COLORS[i],2);}
      const screen=project(p);ctx.fillStyle=color;ctx.beginPath();ctx.arc(...screen,3.5,0,Math.PI*2);ctx.fill();
    }
  };
  drawBox([0,0,0],quaternion(d.anchor),'#546477',true);
  const row=sampleAt(d,S.time);
  if(row) {
    const q0=quaternion(d.anchor),q=mul(q0,qpower(mul(inv(q0),quaternion(row)),num('rotationGain',1)));
    drawBox(scale(displacement(row,d),gain),q,side==='A'?A_COLOR:B_COLOR);
    const delta=norm(displacement(row,d))*1000;
    $('readout'+side).textContent=`Δp ${delta.toFixed(3)} mm · ${norm(angleVector(row,d)).toFixed(3)}°${row[10]===null?'':` · 静止判定 ${(row[10]*1000).toFixed(0)} ms`}`;
  } else { $('readout'+side).textContent='欠測 / 無効姿勢';ctx.fillStyle='#ffbca3';ctx.font='12px system-ui';ctx.fillText('欠測 / 無効姿勢',18,25); }
  ctx.font='11px ui-monospace,monospace';ctx.fillStyle='#91a5b9';
  ctx.fillText(`位置 ${gain}×  /  回転 ${num('rotationGain',1)}×`,16,h-37);
  ctx.fillText(`grid ${(grid/gain*1000).toPrecision(3)} mm (元の変位)`,16,h-19);
  const bar=50, original=bar/pixels/gain*1000;
  ctx.strokeStyle='#d0dfed';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(w-78,h-24);ctx.lineTo(w-28,h-24);ctx.stroke();
  ctx.textAlign='right';ctx.fillText(`${original.toPrecision(3)} mm`,w-23,h-35);ctx.textAlign='left';
}
function rebuild() {
  S.plotCache=null;
  const sides=$('compare').checked?['A','B']:['A'];
  for(const side of sides) {
    const d=S.data[side];if(!d?.anchor)continue;
    d.points=d.rows.map((row,i)=>{
      const p=valid(row)?scale(displacement(row,d),1000):[null,null,null];
      const a=valid(row)?angleVector(row,d):[null,null,null];
      const step=i&&connected(d.rows[i-1],row)?norm(sub(position(row),position(d.rows[i-1])))*1000:null;
      const gyro=row.slice(14,20).every(x=>x!==null)?norm(sub(row.slice(17,20),row.slice(14,17))):null;
      return {t:row[0]-d.offset, p,a,step,gyro,stationary:row[10]};
    });
  }
  const stats=$('stats');stats.replaceChildren();
  for(const side of sides) {
    const d=S.data[side];if(!d?.points)continue;
    const steps=d.points.map(p=>p.step).filter(x=>x!==null).sort((a,b)=>a-b);
    const gaps=d.rows.reduce((n,r,i)=>n+(i&&!connected(d.rows[i-1],r)?1:0),0);
    const addStat=(label,value)=>{const item=document.createElement('div');item.textContent=label;const b=document.createElement('b');b.textContent=value;item.append(b);stats.append(item);};
    addStat(`${side} · サンプル数`,d.rows.length.toLocaleString());
    addStat(`${side} · 最大サンプル間変位`,steps.length?`${steps.at(-1).toFixed(3)} mm`:'—');
    addStat(`${side} · p95 サンプル間変位`,steps.length?`${steps[Math.floor((steps.length-1)*.95)].toFixed(3)} mm`:'—');
    addStat(`${side} · 接続しない区間`,String(gaps));
  }
  $('frameLabel').textContent=`各区間の先頭位置を原点として固定 · ${$('basis').value==='body'?'先頭デバイス軸':'world 軸'} · 灰色: 先頭姿勢`;
  drawAll();
}
function drawPlots() {
  const {ctx,w,h}=canvasContext($('plots'));
  if(S.plotCache&&S.plotCache.w===w&&S.plotCache.h===h) {
    ctx.drawImage(S.plotCache.canvas,0,0,w,h);drawPlotOverlay(ctx,w,h);return;
  }
  ctx.fillStyle='#0e1823';ctx.fillRect(0,0,w,h);
  const left=64,right=16,top=28,bottom=23,plotW=w-left-right;
  const panels=[{key:'p',label:'位置 Δ mm',dim:3},{key:'a',label:'回転 Δ deg',dim:3},
    {key:'step',label:'step mm',dim:1},{key:'gyro',label:'|gyro − bias| rad/s',dim:1}];
  const sides=$('compare').checked?['A','B']:['A'];
  const panelH=(h-top-bottom)/panels.length;
  const x=t=>left+(t-S.lo)/(S.hi-S.lo)*plotW;
  ctx.font='10px ui-monospace,monospace';
  panels.forEach((panel,index)=>{
    let min=panel.key==='p'||panel.key==='a'?Infinity:0,max=-Infinity,available=false;
    for(const side of sides)for(const point of S.data[side]?.points||[]) {
      const values=panel.dim===3?point[panel.key]:[point[panel.key]];
      for(const v of values)if(v!==null&&Number.isFinite(v)){min=Math.min(min,v);max=Math.max(max,v);available=true;}
    }
    if(!Number.isFinite(min)||!Number.isFinite(max)){min=-1;max=1;}
    if(max-min<1e-6){min-=.001;max+=.001;}
    const pad=(max-min)*.08;min-=pad;max+=pad;
    const y0=top+index*panelH,y=v=>y0+panelH-10-(v-min)/(max-min)*(panelH-22);
    ctx.strokeStyle='#263847';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(left,y0+panelH-5);ctx.lineTo(w-right,y0+panelH-5);ctx.stroke();
    ctx.fillStyle='#91a5b9';ctx.textAlign='right';ctx.fillText(max.toPrecision(3),left-8,y0+12);ctx.fillText(min.toPrecision(3),left-8,y0+panelH-10);ctx.textAlign='left';ctx.fillText(panel.label,left+4,y0+10);
    if(!available)ctx.fillText('このストリームでは未記録',left+plotW/2-65,y0+panelH/2);
    for(const side of sides) {
      const d=S.data[side];if(!d?.points)continue;
      for(let component=0;component<panel.dim;component++) {
        ctx.strokeStyle=panel.dim===3?AXIS_COLORS[component]:side==='A'?A_COLOR:B_COLOR;
        ctx.globalAlpha=side==='A'?1:.7;ctx.setLineDash(side==='B'?[4,3]:[]);ctx.lineWidth=1;ctx.beginPath();
        let previous=false;
        for(let i=0;i<d.points.length;i++) {
          const point=d.points[i],v=panel.dim===3?point[panel.key][component]:point[panel.key];
          if(v===null||!Number.isFinite(v)){previous=false;continue;}
          if(previous&&connected(d.rows[i-1],d.rows[i]))ctx.lineTo(x(point.t),y(v));else ctx.moveTo(x(point.t),y(v));
          previous=true;
        }
        ctx.stroke();ctx.globalAlpha=1;ctx.setLineDash([]);
      }
    }
  });
  ctx.fillStyle='#91a5b9';
  for(let i=0;i<=5;i++){const t=S.lo+(S.hi-S.lo)*i/5;ctx.fillText(t.toFixed(3),clamp(x(t)-20,5,w-55),h-6);}
  ctx.fillStyle='#aebfd0';ctx.fillText('X 赤 · Y 緑 · Z 青   /   A 実線 · B 破線   /   倍率なし',left,15);
  const saved=document.createElement('canvas');saved.width=$('plots').width;saved.height=$('plots').height;
  saved.getContext('2d').drawImage($('plots'),0,0);S.plotCache={canvas:saved,w,h};
  drawPlotOverlay(ctx,w,h);
}
function drawPlotOverlay(ctx,w,h) {
  const left=64,right=16,top=28,bottom=23,plotW=w-left-right;
  const x=t=>left+(t-S.lo)/(S.hi-S.lo)*plotW;
  const lo=num('selectLo',S.lo),hi=num('selectHi',S.hi);
  ctx.fillStyle='#58d9c014';ctx.fillRect(x(clamp(lo,S.lo,S.hi)),top,(clamp(hi,S.lo,S.hi)-clamp(lo,S.lo,S.hi))/(S.hi-S.lo)*plotW,h-top-bottom);
  ctx.strokeStyle='#e5f1fb';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x(S.time),top);ctx.lineTo(x(S.time),h-bottom);ctx.stroke();
}
function drawAll() {
  drawScene('A');if($('compare').checked)drawScene('B');drawPlots();
  $('time').textContent=`${S.time.toFixed(6)} s`;$('scrub').value=S.time;
}
function pause() {S.playing=false;$('play').textContent='▶ 再生';}
function animate(ts) {
  if(S.playing&&!document.hidden&&!S.loading) {
    const delta=Math.min((ts-S.lastFrame)/1000,.15)*num('speed',.25);
    S.time+=delta;
    if(S.time>S.hi) {if($('loop').checked)S.time=S.lo+(S.time-S.lo)%(S.hi-S.lo);else{S.time=S.hi;pause();}}
    drawAll();
  }
  S.lastFrame=ts;requestAnimationFrame(animate);
}
function snapshot() {
  const state={v:1,a:$('sourceA').value,ta:$('trackA').value,b:$('compare').checked?$('sourceB').value:'',tb:$('compare').checked?$('trackB').value:'',
    shaA:S.meta.A?.source_sha256,shaB:$('compare').checked?S.meta.B?.source_sha256:null,
    offset:num('offsetB'),lo:num('selectLo',S.lo),hi:num('selectHi',S.hi),windowLo:S.lo,windowHi:S.hi,time:S.time,
    gain:num('gain'),rotation:num('rotationGain'),basis:$('basis').value,view:$('view').value,
    speed:num('speed'),trail:num('trail'),interpolation:$('interpolation').value,camera:{...S.camera},comment:$('comment').value};
  return state;
}
function shareLink(state) {
  return `${location.origin}${location.pathname}#${new URLSearchParams({review:JSON.stringify(state)})}`;
}
function updateShare() {
  const state=snapshot(),lines=['Lighthouse motion review v1'];
  for(const side of (state.b?['A','B']:['A'])) {
    const meta=S.meta[side],track=selectedTrack(side);if(!meta||!track)continue;
    const shift=side==='B'?state.offset:0;
    const rows=S.data[side]?.rows||[];
    const selected=rows.filter(r=>r[0]>=state.lo+shift&&r[0]<=state.hi+shift);
    lines.push(`${side}: ${meta.source.session}`,`source: ${meta.source.id}`,`stream: ${meta.source.label}`,
      `SHA-256: ${meta.source_sha256}`,`device: ${track.serial} / ${track.stage} / ${track.id}`,
      `frame: ${track.frame}; reference: ${track.reference}`,`clock: ${meta.time_basis}`,
      `range: ${(state.lo+shift).toFixed(6)} – ${(state.hi+shift).toFixed(6)} s`,
      `integrity: ${meta.health.stream_integrity}; damaged rows: ${meta.health.damaged_rows}; manifest: ${meta.source.manifest_status}`);
    if(selected.length&&selected[0][1]!==null) lines.push(`sensor ticks in loaded selection: ${selected[0][1]} – ${selected.at(-1)[1]} @ ${selected[0][2]} Hz`);
    if(state.lo<S.lo||state.hi>S.hi)lines.push('NOTE: selection extends beyond loaded window; reload selection to include its sample ticks.');
  }
  lines.push(`display: position ${state.gain}x; rotation ${state.rotation}x; basis ${state.basis}; ${state.interpolation}; speed ${state.speed}x`,
    `cursor: ${state.time.toFixed(6)} s`,`comment: ${state.comment || '(未記入)'}`,`reopen: ${shareLink(state)}`,
    `review-state: ${JSON.stringify(state)}`);
  $('shareText').value=lines.join('\n');
  return {state,text:$('shareText').value};
}
async function copyText(text) {
  try {
    if(navigator.clipboard&&isSecureContext){await navigator.clipboard.writeText(text);return true;}
  } catch (_) { /* LAN HTTP usually lacks the secure Clipboard API. */ }
  const area=document.createElement('textarea');area.value=text;area.style.position='fixed';area.style.left='-9999px';document.body.append(area);area.select();
  let copied=false;try{copied=document.execCommand('copy');}finally{area.remove();}
  return copied;
}
async function copy(linkOnly=false) {
  for(const side of ($('compare').checked?['A','B']:['A'])) {
    if(S.loading||S.data[side]?.sourceId!==$('source'+side).value||S.data[side]?.track.id!==$('track'+side).value)
      throw new Error('選択中の記録・デバイスの区間を開いてからコピーしてください。');
  }
  const {state,text}=updateShare();
  if(!(state.hi>state.lo&&state.hi-state.lo<=60))throw new Error('共有する範囲は開始 < 終了、最大 60 秒にしてください。');
  const copied=await copyText(linkOnly?shareLink(state):text);
  history.replaceState(null,'',shareLink(state));
  $('copied').textContent=copied?'コピーしました':'下の欄を選択して手動でコピーしてください';
  if(!copied){$('shareDetails').open=true;$('shareText').focus();$('shareText').select();}
}
async function choose(id,lo,hi,comparison='') {
  $('search').value='';sourceOptions('A');sourceOptions('B');
  if(!S.sources.some(s=>s.id===id))throw new Error('この記録は現在のカタログにありません。');
  $('sourceA').value=id;$('compare').checked=!!comparison;toggleCompare();
  await prepare('A');
  if(comparison){$('sourceB').value=comparison;$('offsetB').value=0;await prepare('B');}
  $('lo').value=lo;$('hi').value=hi;await loadWindow();
}
function restoreControls(state) {
  const map={gain:'gain',rotationGain:'rotation',basis:'basis',view:'view',speed:'speed',trail:'trail',interpolation:'interpolation'};
  for(const [id,key] of Object.entries(map))if(state[key]!==undefined) {
    const value=String(state[key]);if([...$(id).options].some(o=>o.value===value))$(id).value=value;
  }
  if(state.camera&&['yaw','pitch','zoom'].every(k=>Number.isFinite(state.camera[k]))) {
    S.camera={yaw:state.camera.yaw,pitch:clamp(state.camera.pitch,-1.4,1.4),zoom:clamp(state.camera.zoom,1e-6,20),
      target:Array.isArray(state.camera.target)&&state.camera.target.length===3&&state.camera.target.every(Number.isFinite)?state.camera.target:[0,0,0]};
  }
  $('offsetB').value=Number.isFinite(state.offset)?state.offset:0;$('comment').value=state.comment||'';
}
async function restore(state) {
  if(state.v!==1)throw new Error('未対応の共有データです。');
  if(!S.sources.some(s=>s.id===state.a))throw new Error('共有リンクの記録が見つかりません。');
  $('sourceA').value=state.a;$('compare').checked=!!state.b;toggleCompare();restoreControls(state);
  await prepare('A',state.ta);
  if($('trackA').value!==state.ta)throw new Error('共有リンクのデバイス A が見つかりません。');
  if(state.shaA&&state.shaA!==S.meta.A.source_sha256)throw new Error('記録 A の SHA-256 が共有時と異なります。元ファイルを確認してください。');
  if(state.b) {
    if(!S.sources.some(s=>s.id===state.b))throw new Error('共有リンクの比較記録 B が見つかりません。');
    $('sourceB').value=state.b;await prepare('B',state.tb);
    if($('trackB').value!==state.tb)throw new Error('共有リンクのデバイス B が見つかりません。');
    if(state.shaB&&state.shaB!==S.meta.B.source_sha256)throw new Error('記録 B の SHA-256 が共有時と異なります。');
  }
  $('lo').value=Number.isFinite(state.windowLo)?state.windowLo:state.lo;
  $('hi').value=Number.isFinite(state.windowHi)?state.windowHi:state.hi;
  await loadWindow({time:state.time,fit:false});
  $('selectLo').value=state.lo;$('selectHi').value=state.hi;drawAll();updateShare();
}
function bind() {
  $('refreshSources').onclick=action(async()=>{await refreshSources();status('記録一覧を更新しました。');});
  $('search').oninput=()=>{sourceOptions('A');sourceOptions('B');};
  for(const side of ['A','B']) {
    $('source'+side).onchange=action(async()=>{
      pause();if(!await prepare(side))return;
      if(side==='A'){
        const t=selectedTrack(side),lo=clamp(num('lo'),t.start,Math.max(t.start,t.end-6));
        $('lo').value=lo.toFixed(6);$('hi').value=Math.min(t.end,lo+6).toFixed(6);
      }
      await loadWindow();
    });
    $('track'+side).onchange=action(async()=>{updateTrack(side);await loadWindow();});
  }
  $('compare').onchange=action(async()=>{toggleCompare();if($('compare').checked)await prepare('B');await loadWindow();});
  $('load').onclick=action(()=>loadWindow());$('offsetB').onchange=action(()=>loadWindow());
  $('markers').onchange=action(async()=>{if(!$('markers').value)return;const t=+$('markers').value;$('lo').value=Math.max(0,t-1).toFixed(6);$('hi').value=(t+5).toFixed(6);await loadWindow();});
  $('overview').onchange=action(async()=>{const lo=num('overview'),track=selectedTrack('A');$('lo').value=lo.toFixed(6);$('hi').value=Math.min(track.end,lo+6).toFixed(6);await loadWindow();});
  $('play').onclick=()=>{if(S.loading)return;S.playing=!S.playing;$('play').textContent=S.playing?'Ⅱ 一時停止':'▶ 再生';if(S.playing&&S.time>=S.hi)S.time=S.lo;};
  $('scrub').oninput=()=>{pause();S.time=num('scrub');drawAll();};
  for(const [id,direction] of [['prev',-1],['next',1]])$(id).onclick=()=>{
    pause();const rows=S.data.A?.rows;if(!rows?.length)return;
    let i=lowerBound(rows,S.time);if(direction<0)i--;else if(rows[i]?.[0]<=S.time+1e-9)i++;
    S.time=clamp(rows[clamp(i,0,rows.length-1)][0],S.lo,S.hi);drawAll();
  };
  for(const id of ['gain','rotationGain','basis','trail','interpolation'])$(id).onchange=()=>{if(id==='basis')fitCamera();rebuild();updateShare();};
  $('view').onchange=()=>{$('gain').value=$('view').value==='room'?'1':'30';fitCamera();rebuild();updateShare();};
  $('fitCamera').onclick=()=>{fitCamera();drawAll();};
  $('resetCamera').onclick=()=>{S.camera={yaw:-.7,pitch:.48,zoom:1,target:[0,0,0]};fitCamera();drawAll();};
  for(const side of ['A','B']) {
    const canvas=$('scene'+side);
    canvas.onpointerdown=e=>{S.drag={x:e.clientX,y:e.clientY,yaw:S.camera.yaw,pitch:S.camera.pitch};canvas.setPointerCapture(e.pointerId);};
    canvas.onpointermove=e=>{if(!S.drag)return;S.camera.yaw=S.drag.yaw+(e.clientX-S.drag.x)*.008;S.camera.pitch=clamp(S.drag.pitch+(e.clientY-S.drag.y)*.008,-1.4,1.4);drawAll();};
    canvas.onpointerup=()=>{S.drag=null;};canvas.onpointercancel=()=>{S.drag=null;};
    canvas.onwheel=e=>{e.preventDefault();S.camera.zoom=clamp(S.camera.zoom*Math.exp(-e.deltaY*.001),1e-6,20);drawAll();};
    canvas.ondblclick=()=>$('resetCamera').click();
  }
  const chartTime=e=>{const r=$('plots').getBoundingClientRect();return clamp(S.lo+(e.clientX-r.left-64)/(r.width-80)*(S.hi-S.lo),S.lo,S.hi);};
  let chartStart=null;
  $('plots').onpointerdown=e=>{pause();chartStart={t:chartTime(e),x:e.clientX};$('plots').setPointerCapture(e.pointerId);};
  $('plots').onpointermove=e=>{if(!chartStart)return;const t=chartTime(e);$('selectLo').value=Math.min(t,chartStart.t).toFixed(6);$('selectHi').value=Math.max(t,chartStart.t).toFixed(6);S.time=t;drawAll();};
  $('plots').onpointerup=e=>{if(!chartStart)return;S.time=chartTime(e);if(Math.abs(e.clientX-chartStart.x)<4){$('selectLo').value=S.lo;$('selectHi').value=S.hi;}chartStart=null;drawAll();updateShare();};
  $('plots').onpointercancel=()=>{chartStart=null;};
  $('markLo').onclick=()=>{$('selectLo').value=S.time.toFixed(6);drawAll();updateShare();};
  $('markHi').onclick=()=>{$('selectHi').value=S.time.toFixed(6);drawAll();updateShare();};
  $('zoomSelection').onclick=action(async()=>{$('lo').value=$('selectLo').value;$('hi').value=$('selectHi').value;await loadWindow();});
  for(const id of ['selectLo','selectHi','comment'])$(id).oninput=()=>{updateShare();drawPlots();};
  $('copy').onclick=action(()=>copy());$('copyLink').onclick=action(()=>copy(true));
  const a='2026-10-25_211424_libsurvive_158643023_1366788';
  $('presetA').onclick=action(()=>choose(`dump:${a}:native`,260,266));
  $('presetCompare').onclick=action(()=>choose('replay:handheld:baseline',260,266,'replay:handheld:tracker-final'));
  $('presetLatest').onclick=action(async()=>{const id=S.sources.find(s=>s.group==='recordings'&&s.kind==='native')?.id||S.sources[0].id;await choose(id,0,6);const t=selectedTrack('A');$('lo').value=t.start.toFixed(6);$('hi').value=Math.min(t.start+6,t.end).toFixed(6);await loadWindow();});
  addEventListener('resize',drawAll);
  addEventListener('hashchange',action(async()=>{const value=new URLSearchParams(location.hash.slice(1)).get('review');if(value)await restore(JSON.parse(value));}));
  document.addEventListener('keydown',e=>{if(['INPUT','TEXTAREA','SELECT','BUTTON'].includes(e.target.tagName))return;if(e.code==='Space'){e.preventDefault();$('play').click();}if(e.code==='ArrowRight')$('next').click();if(e.code==='ArrowLeft')$('prev').click();});
}
async function refreshSources() {
  const result=await api('/api/sources');S.sources=result.sources;
  const sessions=new Set(S.sources.filter(s=>s.group==='recordings').map(s=>s.session));
  $('catalogStatus').textContent=`${sessions.size} ダンプ · ${S.sources.length} ストリーム · ローカル再生`;
  sourceOptions('A');sourceOptions('B');
}
async function init() {
  bind();requestAnimationFrame(animate);
  await refreshSources();
  const hash=new URLSearchParams(location.hash.slice(1)).get('review');
  if(hash)await restore(JSON.parse(hash));
  else {
    const preferred='dump:2026-10-25_211424_libsurvive_158643023_1366788:native';
    if(S.sources.some(s=>s.id===preferred))await choose(preferred,260,266);
    else if(S.sources.length) $('presetLatest').click();
    else status('LH-Dump に記録が見つかりません。--dumps で保存先を指定してください。');
  }
}
init().catch(errorMessage);
