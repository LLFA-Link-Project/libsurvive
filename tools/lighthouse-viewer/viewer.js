/* Local recorded-pose viewer. No external assets, analytics, or runtime deps. */
'use strict';
const $ = id => document.getElementById(id);
const A_COLOR = '#58d9c0', B_COLOR = '#ffb877';
const OPTICAL_COLOR = '#d997ff';
const AXIS_COLORS = ['#ff8b8b', '#8fdd97', '#83b9ff'];
const S = {sources: [], meta: {}, data: {}, generation: {A: 0, B: 0},
  windowGeneration: 0, full: null,
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
    $('extent').textContent=`${track.start.toFixed(3)} – ${track.end.toFixed(3)} 秒 · ${track.count.toLocaleString()} サンプル`;
    syncRanges();
  }
}
async function prepare(side, preferred) {
  endFull();
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
      for (const track of meta.tracks.filter(t=>!['LH_POSE','OPTICAL'].includes(t.stage))) select.append(new Option(track.label,track.id));
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
async function fetchWindow(lo,hi,anchors={}) {
  const sides=$('compare').checked?['A','B']:['A'];
  return Promise.all(sides.map(async side=>{
    const shift=side==='B'?num('offsetB'):0;
    const query={id:$('source'+side).value,track:$('track'+side).value,lo:lo+shift,hi:hi+shift};
    if(Number.isFinite(anchors[side])||anchors[side]==='first')query.anchor=anchors[side];
    let data=await api('/api/window',query);
    while(data.optical?.state==='indexing') {
      status(`${side}: 光学診断のキャッシュを準備中…`);
      await sleep(1000);data=await api('/api/window',query);
    }
    return data;
  }));
}
async function loadWindow({selection=true, time=null, fit=true, full=false, anchors={}, buffer=null}={}) {
  const generation=++S.windowGeneration;
  const lo=num('lo'),hi=num('hi');
  if (!(hi>lo&&hi-lo<=60)) throw new Error('開始 < 終了、区間の長さは最大 60 秒にしてください。');
  if(!full)endFull();
  pause(); S.loading=true; $('load').disabled=true;
  const sides=$('compare').checked?['A','B']:['A'];
  try {
    for (const side of sides) if (!selectedTrack(side)) throw new Error(`${side} のデバイスを選択してください。`);
    const data=buffer||await fetchWindow(lo,hi,anchors);
    if(generation!==S.windowGeneration)return;
    sides.forEach((side,i)=>{
      if(data[i].state!=='ready') throw new Error('インデックス作成が終わるまで待ってください。');
      const d=data[i]; d.offset=side==='B'?num('offsetB'):0;
      d.sourceId=$('source'+side).value;
      d.anchor=d.anchor||d.rows.find(valid); d.label=S.meta[side].source.label;
      d.context=(d.context||[]).map(c=>({...c,offset:d.offset}));
      if(d.optical)d.optical.offset=d.offset;
      S.data[side]=d;
    });
    S.lo=lo; S.hi=hi; S.time=clamp(time??S.data.A.anchor?.[0]??lo,lo,hi);
    $('scrub').min=lo; $('scrub').max=hi; $('scrub').value=S.time;
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
    const descriptions=sides.map(side=>{
      const o=S.data[side].optical;
      return `${side}: ${o?.state==='ready'?(o.origin==='reconstructed'||o.provenance?'再生で求めた光学解（元の実行とは処理順が異なります）':'記録済みの光学解'):(o?.reason||'光学解の表示にはサーバーの再起動が必要です')}`;
    });
    $('opticalNotice').textContent=descriptions.join(' / ');
    if(fit)fitCamera();
    rebuild(); updateShare();
  } finally { if(generation===S.windowGeneration){S.loading=false; $('load').disabled=false;} }
}
function basisVector(v,d) { return $('basis').value==='body'?rot(inv(quaternion(d.anchor)),v):v; }
function displacement(row,d) {return basisVector(sub(position(row),position(d.anchor)),d);}
function angleVector(row,d) {return basisVector(rotationVector(mul(quaternion(row),inv(quaternion(d.anchor)))),d);}
function sampleAt(d,t,maxAge=.02,interpolate=true) {
  if(!d) return null;
  const target=t+d.offset, i=lowerBound(d.rows,target);
  if(i<d.rows.length&&d.rows[i][0]===target) return d.rows[i];
  const a=d.rows[i-1]||(i===0?d.previous:null), b=d.rows[i];
  if(!valid(a)||target<a[0]||target-a[0]>maxAge) return null;
  if(interpolate&&$('interpolation').value==='linear'&&b&&connected(a,b)&&b[0]>a[0]) {
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
function scenePosition(row,d,magnified=false) {
  return scale(displacement(row,d),magnified?num('gain',30):1);
}
function deviceKind(track) {
  const name=`${track.model||''} ${track.role||''}`.toLowerCase();
  if(/base|lighthouse|trackingreference/.test(name)||track.role==='4')return 'base';
  if(/tracker|tundra|generic/.test(name)||track.role==='3')return 'tracker';
  if(/knuckle|index|controller|hand/.test(name)||track.role==='2')return 'controller';
  if(/head|hmd/.test(name)||track.role==='1')return 'hmd';
  return 'unknown';
}
function deviceName(track) {
  const model=track.model||({controller:'Controller',tracker:'Tracker',base:'BS',hmd:'HMD',unknown:'Device'}[deviceKind(track)]);
  return `${model}${/left/.test(`${track.role}`)?' L':/right/.test(`${track.role}`)?' R':''} · ${track.serial}`;
}
// Wire schematics distinguish device classes; they are not CAD/grip models.
function modelLines(track) {
  const lines=[],kind=deviceKind(track);
  const ring=(center,radii,plane='xy')=>{
    const points=[];
    for(let i=0;i<=24;i++) {
      const a=i/24*Math.PI*2,c=Math.cos(a),s=Math.sin(a);
      points.push(add(center,plane==='xy'?[radii[0]*c,radii[1]*s,0]:[radii[0]*c,0,radii[1]*s]));
    }
    lines.push(points);
  };
  const box=(x,y,z)=>{
    const vertices=Array.from({length:8},(_,i)=>[i&1?x:-x,i&2?y:-y,i&4?z:-z]);
    for(let i=0;i<8;i++)for(const bit of [1,2,4])if(!(i&bit))lines.push([vertices[i],vertices[i|bit]]);
  };
  if(kind==='tracker') {
    ring([0,0,.25],[.55,.55]);ring([0,0,-.2],[.55,.55]);
    for(let i=0;i<3;i++) {
      const a=i*Math.PI*2/3;
      const local=[[.4,-.3,.25],[1.25,-.3,.12],[1.45,0,0],[1.25,.3,.12],[.4,.3,.25]];
      lines.push(local.map(([x,y,z])=>[x*Math.cos(a)-y*Math.sin(a),x*Math.sin(a)+y*Math.cos(a),z]));
    }
  } else if(kind==='controller') {
    box(.33,.95,.3);
    ring([0,-.75,.25],[.65,.5]);
    const side=/left/i.test(`${track.model} ${track.role}`)?-1:1;
    lines.push([[side*.3,-.85,.25],[side*1.05,-.7,.3],[side*1.25,.45,.15],[side*.3,.95,.1]]);
    ring([0,-.85,.38],[.22,.17]);
  } else if(kind==='base') {
    box(.7,.55,.65);
    lines.push([[-.55,-.6,-.5],[.55,-.6,-.5],[.55,-.6,.5],[-.55,-.6,.5],[-.55,-.6,-.5]]);
    lines.push([[0,-.6,0],[0,-1.7,0]]);
  } else if(kind==='hmd') {box(1,.5,.5);ring([0,.1,0],[1,.9]);}
  else box(1,.7,.45);
  return lines;
}
function makeProjection(w,h,d) {
  const span=$('view').value==='micro'?.12:3;
  const focal=Math.min(w,h)/(2*Math.tan(42*Math.PI/360));
  const distance=focal/(Math.min(w,h)/span*S.camera.zoom),near=Math.max(1e-8,distance*.001);
  const {yaw,pitch}=S.camera;
  const camera=v=>{
    const [x,y,z]=sub(visualVector(v,d),S.camera.target);
    const depth=x*Math.sin(yaw)+y*Math.cos(yaw);
    return [x*Math.cos(yaw)-y*Math.sin(yaw),depth*Math.sin(pitch)-z*Math.cos(pitch),distance-depth*Math.cos(pitch)-z*Math.sin(pitch)];
  };
  const screen=v=>[w/2+v[0]*focal/v[2],h*.53+v[1]*focal/v[2],v[2]];
  return {camera,near,distance,focal,project:v=>{const p=camera(v);return p[2]>=near?screen(p):null;},
    segment:(a,b)=>{
      a=camera(a);b=camera(b);
      if(a[2]<near&&b[2]<near)return null;
      if(a[2]<near||b[2]<near) {
        const f=(near-a[2])/(b[2]-a[2]),cut=a.map((x,i)=>x+f*(b[i]-x));
        cut[2]=near;if(a[2]<near)a=cut;else b=cut;
      }
      return [screen(a),screen(b)];
    }};
}
function fitCamera(includeContext=false) {
  const min=[0,0,0],max=[0,0,0];
  const extend=p=>p.forEach((x,i)=>{min[i]=Math.min(min[i],x);max[i]=Math.max(max[i],x);});
  for(const side of ($('compare').checked?['A','B']:['A'])) {
    const d=S.data[side];if(!d?.anchor)continue;
    for(const row of d.rows)if(valid(row))extend(visualVector(scenePosition(row,d,true),d));
    if(includeContext&&$('showContext').checked)for(const other of d.context||[])
      for(const row of other.rows)if(valid(row))extend(visualVector(scenePosition(row,d),d));
    if(includeContext&&$('showOptical').checked)for(const row of d.optical?.rows||[])
      if(valid(row))extend(visualVector(scenePosition(row,d,true),d));
  }
  S.camera.target=min.map((x,i)=>(x+max[i])/2);
  const radius=norm(sub(max,min))/2,span=$('view').value==='micro'?.12:3;
  S.camera.zoom=clamp(span/Math.max(span,radius*3.3),1e-6,20);
}
function drawScene(side) {
  const canvas=$('scene'+side),{ctx,w,h}=canvasContext(canvas),d=S.data[side];
  ctx.fillStyle='#0e1823';ctx.fillRect(0,0,w,h);
  if(!d?.anchor) {ctx.fillStyle='#91a5b9';ctx.font='13px system-ui';ctx.fillText('この区間に有効な姿勢がありません',24,h/2);return;}
  const projection=makeProjection(w,h,d),{project}=projection,micro=$('view').value==='micro';
  const span=micro?.12:3;
  function line(points,color,width=1,dashed=false) {
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dashed?[4,5]:[]);ctx.beginPath();
    for(let i=1;i<points.length;i++) {
      const edge=projection.segment(points[i-1],points[i]);
      if(edge){ctx.moveTo(...edge[0].slice(0,2));ctx.lineTo(...edge[1].slice(0,2));}
    }
    ctx.stroke();ctx.setLineDash([]);
  }
  const idealGrid=span/S.camera.zoom/12,base=10**Math.floor(Math.log10(idealGrid)),grid=base*Math.ceil(idealGrid/base);
  const extent=grid*12,cx=Math.round(S.camera.target[0]/grid)*grid,cy=Math.round(S.camera.target[1]/grid)*grid;
  const fromVisual=v=>$('basis').value==='world'&&/Y-up|y_up/.test(d.track.frame)?[v[0],v[2],-v[1]]:v;
  for(let i=-12;i<=12;i++) {
    line([fromVisual([cx-extent,cy+i*grid,0]),fromVisual([cx+extent,cy+i*grid,0])],'#1d3040');
    line([fromVisual([cx+i*grid,cy-extent,0]),fromVisual([cx+i*grid,cy+extent,0])],'#1d3040');
  }
  const axisLength=micro?.025/S.camera.zoom:.35;
  ['X','Y','Z'].forEach((label,i)=>{
    const v=[0,0,0];v[i]=axisLength;line([[0,0,0],v],AXIS_COLORS[i],1.5);
    const p=project(scale(v,1.1));if(p){ctx.fillStyle=AXIS_COLORS[i];ctx.font='11px ui-monospace,monospace';ctx.fillText(label,p[0],p[1]);}
  });
  const trail=(rows,color,magnified)=>{
    let previous=null;
    for(let i=lowerBound(rows,S.time-num('trail',1)+d.offset);i<rows.length&&rows[i][0]<=S.time+d.offset;i++) {
      const row=rows[i];
      if(previous&&connected(previous,row))line([scenePosition(previous,d,magnified),scenePosition(row,d,magnified)],color,1.25);
      previous=valid(row)?row:null;
    }
  };
  ctx.globalAlpha=.65;trail(d.rows,side==='A'?A_COLOR:B_COLOR,true);
  if($('showOptical').checked&&d.optical?.state==='ready')trail(d.optical.rows,OPTICAL_COLOR,true);
  ctx.globalAlpha=1;
  const objects=[],edgeLabels=[];
  const addObject=(row,track,color,kind,magnified=false)=>{
    if(!row)return;
    const p=scenePosition(row,d,magnified),q0=quaternion(d.anchor);
    const q=magnified?mul(q0,qpower(mul(inv(q0),quaternion(row)),num('rotationGain',1))):quaternion(row);
    objects.push({p,q,track,color,kind,depth:projection.camera(p)[2]});
  };
  if($('showContext').checked)for(const other of d.context||[]) {
    const row=sampleAt(other,S.time,other.track.persistent||deviceKind(other.track)==='base'?Infinity:.02,false);
    addObject(row,other.track,'#8395a9','context');
  }
  if(S.time+d.offset>=d.anchor[0])addObject(d.anchor,d.track,'#4c5c6d','ghost',true);
  const row=sampleAt(d,S.time),optical=$('showOptical').checked&&d.optical?.state==='ready'?sampleAt(d.optical,S.time,.05,false):null;
  addObject(row,d.track,side==='A'?A_COLOR:B_COLOR,'selected',true);
  addObject(optical,d.track,OPTICAL_COLOR,'optical',true);
  if(row&&optical)line([scenePosition(row,d,true),scenePosition(optical,d,true)],OPTICAL_COLOR,1,true);
  objects.sort((a,b)=>b.depth-a.depth);
  for(const obj of objects) {
    const {p,q,track,color,kind}=obj;
    const size=micro&&kind!=='context'?.009/S.camera.zoom:
      Math.max(.045,obj.depth/projection.focal*(kind==='context'?7:10));
    const transform=v=>add(p,basisVector(rot(q,scale(v,size)),d));
    ctx.globalAlpha=kind==='context'?.36:kind==='ghost'?.4:kind==='optical'?.8:1;
    if(kind==='optical') {
      for(let i=0;i<3;i++){const v=[0,0,0];v[i]=size*.65;line([sub(p,v),add(p,v)],color,2);}
    } else for(const points of modelLines(track))line(points.map(transform),color,kind==='selected'?2:1,kind==='ghost');
    const screen=project(p);
    if(kind==='selected')for(let i=0;i<3;i++) {
      const axis=[0,0,0];axis[i]=2*size;line([p,add(p,basisVector(rot(q,axis),d))],AXIS_COLORS[i],1.5);
    }
    if(kind!=='ghost') {
      const name=kind==='optical'?'光学解':deviceName(track);
      if(screen&&screen[0]>12&&screen[0]<w-12&&screen[1]>30&&screen[1]<h-30) {
        ctx.fillStyle=color;ctx.beginPath();ctx.arc(screen[0],screen[1],kind==='selected'?3.5:2.5,0,Math.PI*2);ctx.fill();
        ctx.font='11px system-ui';ctx.fillText(name,clamp(screen[0]+10,8,Math.max(8,w-ctx.measureText(name).width-8)),screen[1]-12);
      } else if(kind!=='selected') {
        const c=projection.camera(p),sign=c[2]<projection.near?-1:1;
        const x=c[0]*sign,y=c[1]*sign,r=Math.max(Math.abs(x)/(w/2-18),Math.abs(y)/(h/2-35),1e-12);
        const edge=[w/2+x/r,h/2+y/r];
        ctx.fillStyle=color;ctx.beginPath();ctx.arc(edge[0],edge[1],3,0,Math.PI*2);ctx.fill();
        edgeLabels.push({name,color});
      }
    }
    ctx.globalAlpha=1;
  }
  ctx.font='11px system-ui';
  edgeLabels.forEach((item,i)=>{ctx.fillStyle=item.color;ctx.globalAlpha=.65;ctx.fillText(`画面外 · ${item.name}`,12,48+i*17);});
  ctx.globalAlpha=1;ctx.fillStyle='#91a5b9';ctx.font='11px ui-monospace,monospace';
  ctx.fillText(`透視投影 · 位置 ${num('gain')}× / 回転 ${num('rotationGain')}× · ${$('interpolation').value==='sample'?'実サンプル':'補間'}`,14,22);
  ctx.fillText(`grid ${grid>=1?grid.toPrecision(2)+' m':(grid*1000).toPrecision(2)+' mm'} · 模式図`,14,h-15);
  if(row) {
    const diff=optical?` · ${d.optical.origin==='reconstructed'||d.optical.provenance?'再生光学':'光学'}との差 ${(norm(sub(position(optical),position(row)))*1000).toFixed(2)} mm (${((S.time+d.offset-optical[0])*1000).toFixed(0)} ms 前)`:'';
    $('readout'+side).textContent=`Δp ${(norm(displacement(row,d))*1000).toFixed(3)} mm · ${norm(angleVector(row,d)).toFixed(3)}°${diff}`;
  } else $('readout'+side).textContent='欠測 / 無効姿勢';
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
  $('frameLabel').textContent=`原点 ${S.data.A?.anchor?.[0].toFixed(6)??'—'} s · ${$('basis').value==='body'?'先頭デバイス軸':'world 軸'} · 灰色: 先頭姿勢 · 紫: 光学解`;
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
function captureExtent() {
  const selected=selectedTrack('A');
  const tracks=(S.meta.A?.tracks||[]).filter(t=>t.frame===selected?.frame&&
    (t.stage===selected.stage||t.stage==='LH_POSE'));
  return {start:Math.min(0,...tracks.map(t=>t.start)),end:Math.max(0,...tracks.map(t=>t.end))};
}
function endFull() {
  if(S.full)S.full.cancelled=true;
  S.full=null;
  $('playFull').textContent='記録全体を再生';$('fullStatus').textContent='';
}
async function startFull() {
  if(S.full){pause();endFull();return;}
  const track=selectedTrack('A');if(!track||S.loading)return;
  const full={...captureExtent(),anchors:{A:'first',B:'first'},cancelled:false,next:null,switching:false};
  S.full=full;$('playFull').textContent='全体再生を終了';
  $('view').value='room';$('gain').value='1';$('rotationGain').value='1';
  $('lo').value=full.start;$('hi').value=Math.min(full.start+30,full.end);
  await loadWindow({full:true,anchors:full.anchors,time:full.start});
  if(S.full!==full)return;
  for(const side of ($('compare').checked?['A','B']:['A']))full.anchors[side]=S.data[side].anchor?.[0];
  fitCamera(true);drawAll();S.playing=true;$('play').textContent='Ⅱ 一時停止';
  prefetchFull();
}
function prefetchFull() {
  const full=S.full;
  if(!full||full.next||S.hi>=full.end)return;
  // Include the boundary sample in both windows. Selection and sample stepping
  // stay local; playback retains the original anchor across every window.
  const lo=S.hi,hi=Math.min(lo+30,full.end);
  full.next={lo,hi,promise:fetchWindow(lo,hi,full.anchors).then(data=>({data}),error=>({error}))};
}
async function advanceFull(time) {
  const full=S.full;if(!full||full.switching)return;
  prefetchFull();if(!full.next)return;
  full.switching=true;pause();
  $('fullStatus').textContent='次の区間を読み込み中…';
  try {
    const next=full.next,result=await next.promise;
    if(S.full!==full||full.cancelled)return;
    if(result.error)throw result.error;
    $('lo').value=next.lo;$('hi').value=next.hi;full.next=null;
    await loadWindow({full:true,fit:false,time,anchors:full.anchors,buffer:result.data});
    if(S.full!==full)return;
    S.playing=true;$('play').textContent='Ⅱ 一時停止';prefetchFull();
  } catch(error) {endFull();errorMessage(error);}
  finally {full.switching=false;}
}
function animate(ts) {
  if(S.playing&&!document.hidden&&!S.loading) {
    const delta=Math.min((ts-S.lastFrame)/1000,.15)*num('speed',.25);
    S.time+=delta;
    if(S.full) {
      $('fullStatus').textContent=`全体再生 ${S.full.start.toFixed(3)} – ${S.full.end.toFixed(3)} s · 原点固定 · 読込 ${S.lo.toFixed(3)} – ${S.hi.toFixed(3)} s`;
      if(S.time>S.hi) {
        if(S.hi<S.full.end){const time=S.time;S.time=S.hi;advanceFull(time);}
        else if($('loop').checked){endFull();startFull().catch(errorMessage);}
        else {S.time=S.hi;pause();endFull();}
      }
    } else if(S.time>S.hi) {if($('loop').checked)S.time=S.lo+(S.time-S.lo)%(S.hi-S.lo);else{S.time=S.hi;pause();}}
    drawAll();
  }
  S.lastFrame=ts;requestAnimationFrame(animate);
}
function snapshot() {
  const state={v:1,a:$('sourceA').value,ta:$('trackA').value,b:$('compare').checked?$('sourceB').value:'',tb:$('compare').checked?$('trackB').value:'',
    shaA:S.meta.A?.source_sha256,shaB:$('compare').checked?S.meta.B?.source_sha256:null,
    offset:num('offsetB'),lo:num('selectLo',S.lo),hi:num('selectHi',S.hi),windowLo:S.lo,windowHi:S.hi,time:S.time,
    gain:num('gain'),rotation:num('rotationGain'),basis:$('basis').value,view:$('view').value,
    speed:num('speed'),trail:num('trail'),interpolation:$('interpolation').value,camera:{...S.camera},comment:$('comment').value,
    projection:'perspective',context:$('showContext').checked,optical:$('showOptical').checked,
    anchorA:S.data.A?.anchor?.[0]??null,anchorB:$('compare').checked?S.data.B?.anchor?.[0]??null:null,
    opticalShaA:$('showOptical').checked?S.data.A?.optical?.source_sha256??null:null,
    opticalShaB:$('compare').checked&&$('showOptical').checked?S.data.B?.optical?.source_sha256??null:null};
  return state;
}
function shareLink(state) {
  return `${location.origin}${location.pathname}#${new URLSearchParams({review:JSON.stringify(state)})}`;
}
function updateShare() {
  syncRanges();
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
    if(S.data[side]?.anchor)lines.push(`fixed anchor: ${S.data[side].anchor[0].toFixed(6)} s`);
    const optical=S.data[side]?.optical;
    if(state.optical&&optical?.state==='ready') {
      lines.push(`optical: ${optical.origin}; source: ${optical.source_id}; SHA-256: ${optical.source_sha256}`);
      if(optical.provenance)lines.push(`optical provenance: ${JSON.stringify(optical.provenance)}`);
    }
    if(state.lo<S.lo||state.hi>S.hi)lines.push('NOTE: selection extends beyond loaded window; reload selection to include its sample ticks.');
  }
  lines.push(`display: perspective; position ${state.gain}x; rotation ${state.rotation}x; basis ${state.basis}; ${state.interpolation}; speed ${state.speed}x; context ${state.context}; optical ${state.optical}`,
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
  $('showContext').checked=state.context!==false;$('showOptical').checked=state.optical!==false;
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
  await loadWindow({time:state.time,fit:false,anchors:{A:state.anchorA,B:state.anchorB}});
  for(const side of (state.b?['A','B']:['A'])) {
    const expected=state['opticalSha'+side];
    if(expected&&expected!==S.data[side].optical?.source_sha256)throw new Error(`光学解 ${side} の SHA-256 が共有時と異なるか、見つかりません。`);
  }
  $('selectLo').value=state.lo;$('selectHi').value=state.hi;drawAll();updateShare();
}
function syncRange(prefix,lo,hi,min,max) {
  for(const [suffix,value] of [['Lo',lo],['Hi',hi]]) {
    const input=$(prefix+suffix);input.min=min;input.max=max;input.value=clamp(value,min,max);
    input.setAttribute('aria-valuetext',`${value.toFixed(6)} 秒`);
  }
  const span=max-min||1,range=$(prefix+'Range');
  range.style.setProperty('--start',`${clamp((lo-min)/span*100,0,100)}%`);
  range.style.setProperty('--end',`${clamp((hi-min)/span*100,0,100)}%`);
  $(prefix+'Label').textContent=`${lo.toFixed(3)} – ${hi.toFixed(3)} s`;
}
function syncRanges() {
  const track=selectedTrack('A');
  if(track){const extent=captureExtent();syncRange('overview',num('lo'),num('hi'),extent.start,extent.end);}
  syncRange('selection',num('selectLo',S.lo),num('selectHi',S.hi),S.lo,S.hi);
}
function bindRange(prefix,loId,hiId) {
  for(const endpoint of ['Lo','Hi'])$(prefix+endpoint).oninput=()=>{
    pause();
    const input=$(prefix+endpoint),min=+input.min,max=+input.max,epsilon=Math.min(.000001,(max-min)/2);
    let lo=num(prefix+'Lo'),hi=num(prefix+'Hi');
    if(endpoint==='Lo') {lo=clamp(lo,min,hi-epsilon);if(hi-lo>60)hi=lo+60;}
    else {hi=clamp(hi,lo+epsilon,max);if(hi-lo>60)lo=hi-60;}
    $(loId).value=lo.toFixed(6);$(hiId).value=hi.toFixed(6);
    syncRanges();updateShare();drawPlots();
  };
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
  bindRange('overview','lo','hi');bindRange('selection','selectLo','selectHi');
  for(const id of ['lo','hi'])$(id).oninput=syncRanges;
  $('playFull').onclick=action(startFull);
  $('play').onclick=()=>{if(S.loading)return;S.playing=!S.playing;$('play').textContent=S.playing?'Ⅱ 一時停止':'▶ 再生';if(S.playing&&S.time>=S.hi)S.time=S.lo;};
  $('scrub').oninput=()=>{pause();S.time=num('scrub');drawAll();};
  for(const [id,direction] of [['prev',-1],['next',1]])$(id).onclick=()=>{
    pause();const rows=S.data.A?.rows;if(!rows?.length)return;
    let i=lowerBound(rows,S.time);if(direction<0)i--;else if(rows[i]?.[0]<=S.time+1e-9)i++;
    S.time=clamp(rows[clamp(i,0,rows.length-1)][0],S.lo,S.hi);drawAll();
  };
  for(const id of ['gain','rotationGain','basis','trail','interpolation'])$(id).onchange=()=>{if(id==='basis')fitCamera();rebuild();updateShare();};
  for(const id of ['showContext','showOptical'])$(id).onchange=()=>{drawAll();updateShare();};
  $('view').onchange=()=>{$('gain').value=$('view').value==='room'?'1':'30';fitCamera();rebuild();updateShare();};
  $('fitCamera').onclick=()=>{fitCamera();drawAll();};
  $('fitRoom').onclick=()=>{fitCamera(true);drawAll();};
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
