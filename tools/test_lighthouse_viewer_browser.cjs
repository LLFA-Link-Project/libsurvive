/* Integration check against a running server and the existing A/replay corpus.
 * npm install --prefix /tmp/lh-viewer-test playwright
 * PLAYWRIGHT_MODULE=/tmp/lh-viewer-test/node_modules/playwright node tools/test_lighthouse_viewer_browser.cjs
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const base = process.env.LH_VIEWER_URL || 'http://127.0.0.1:8765';
const output = process.env.LH_VIEWER_TEST_OUTPUT || '/tmp/lh-viewer-browser';

(async () => {
  fs.mkdirSync(output, {recursive: true});
  const browser = await chromium.launch({headless: true});
  const errors = [];
  try {
    const context = await browser.newContext({viewport: {width: 1600, height: 1100}});
    context.setDefaultTimeout(240000);
    const page = await context.newPage();
    page.on('pageerror', e => errors.push(e.message));
    await page.goto(base);
    await page.waitForFunction(() => !!S.data.A?.points, {timeout: 240000});
    assert.equal(await page.evaluate(() => S.data.A.rows.length), 1499);
    assert.match(await page.locator('#metaA').innerText(), /readable-prefix/);
    assert.equal(await page.evaluate(() => S.meta.A.source_sha256), 'b3ec6f2a0b4d2fb76cc72eeccf04041801986223fe74e589e363ba1b48329f01');
    const canvasHash = () => page.locator('#sceneA').evaluate(e => e.toDataURL());
    const initialCanvas = await canvasHash();
    await page.locator('#scrub').evaluate(el => {el.value='263.552427';el.dispatchEvent(new Event('input'));});
    assert.notEqual(await canvasHash(), initialCanvas);
    assert.match(await page.locator('#readoutA').innerText(), /Δp/);
    await page.screenshot({path: output+'/A.png', fullPage: true});

    // Actual CONFIG models, contemporaneous devices/BS, and the separately
    // reconstructed optical candidates all share the selected world origin.
    assert.deepEqual(await page.evaluate(()=>S.data.A.context.map(c=>deviceKind(c.track)).sort()),
      ['base','base','base','controller','controller']);
    assert.equal(await page.evaluate(()=>deviceKind(S.data.A.track)), 'tracker');
    assert.match(await page.locator('#opticalNotice').innerText(), /再生で求めた/);
    assert.equal(await page.evaluate(()=>S.data.A.optical.state), 'ready');
    assert.equal(await page.evaluate(()=>S.data.A.optical.origin), 'reconstructed');
    assert.equal(await page.evaluate(()=>S.data.A.optical.provenance.target_sha256===S.meta.A.source_sha256),true);
    assert.equal(await page.evaluate(()=>S.data.A.optical.rows.length>100),true);
    const projection=await page.evaluate(()=>{
      const d=S.data.A,p=makeProjection(600,400,d),{yaw,pitch,target}=S.camera;
      const u=[Math.cos(yaw),-Math.sin(yaw),0];
      const eye=[Math.sin(yaw)*Math.cos(pitch),Math.cos(yaw)*Math.cos(pitch),Math.sin(pitch)];
      const near=add(add(target,scale(eye,p.distance*.5)),scale(u,.01));
      const far=add(add(target,scale(eye,-p.distance*.5)),scale(u,.01));
      const behind=add(target,scale(eye,p.distance*2));
      const crossing=p.segment(target,behind);
      return {near:p.project(near)[0]-300,far:p.project(far)[0]-300,
        behind:p.project(behind),hidden:p.segment(behind,add(behind,u)),finite:crossing.flat().every(Number.isFinite)};
    });
    assert.ok(projection.near>projection.far*2.9);
    assert.equal(projection.behind,null);assert.equal(projection.hidden,null);assert.equal(projection.finite,true);
    assert.deepEqual(await page.evaluate(()=>{
      const d=S.data.A,c=d.context.find(x=>deviceKind(x.track)==='controller'),r=c.rows[0];
      return scenePosition(r,d);
    }),await page.evaluate(()=>{
      const d=S.data.A,c=d.context.find(x=>deviceKind(x.track)==='controller');
      return sub(position(c.rows[0]),position(d.anchor));
    }));
    const withOptical=await canvasHash();
    await page.locator('#showOptical').uncheck();assert.notEqual(await canvasHash(),withOptical);
    await page.locator('#showOptical').check();

    // Both endpoints are independent, accessible native range thumbs. Exercise
    // real mouse dragging as well as keyboard adjustment, not just API values.
    await page.locator('#selectionLo').focus();await page.keyboard.press('ArrowRight');
    assert.ok(+(await page.locator('#selectLo').inputValue())>260);
    const range=await page.locator('#selectionLo').boundingBox();
    await page.mouse.move(range.x+8,range.y+15);await page.mouse.down();
    await page.mouse.move(range.x+8+(range.width-16)*.25,range.y+15,{steps:12});await page.mouse.up();
    const selectedLo=+(await page.locator('#selectLo').inputValue());
    assert.ok(selectedLo>261&&selectedLo<262);
    assert.equal(+(await page.locator('#selectHi').inputValue()),266);
    await page.locator('#overviewHi').focus();await page.keyboard.press('ArrowLeft');
    assert.equal(+(await page.locator('#hi').inputValue()),265.999);

    await page.locator('#lo').fill('234.176186');await page.locator('#hi').fill('240.176186');
    await page.locator('#load').click();
    await page.waitForFunction(()=>S.lo===234.176186&&!S.loading);
    assert.equal(await page.evaluate(()=>S.data.A.rows.length),1498);
    assert.equal(await page.evaluate(()=>S.data.A.optical.rows.length>100),true);
    await page.locator('#view').selectOption('room');
    await page.locator('#scrub').evaluate(el=>{el.value='238.97409';el.dispatchEvent(new Event('input'));});
    await page.locator('#fitRoom').click();
    await page.screenshot({path:output+'/optical-room.png',fullPage:true});

    // Load *every* sample via whole-record playback. Jump only the playback
    // cursor to boundaries to avoid waiting five minutes in this smoke test.
    await page.locator('#loop').uncheck();await page.locator('#speed').selectOption('8');
    await page.locator('#playFull').click();
    await page.waitForFunction(()=>!!S.full&&S.lo===0&&S.playing&&!S.loading);
    const fullAnchor=await page.evaluate(()=>S.data.A.anchor);
    const fullCamera=await page.evaluate(()=>({...S.camera}));
    assert.equal(await page.evaluate(()=>S.time<1),true); // Do not skip the first chunk before the tracker appears.
    const seen=new Set(),windows=[];
    for(let chunk=0;chunk<20;chunk++) {
      const state=await page.evaluate(()=>({lo:S.lo,hi:S.hi,end:S.full?.end,
        ticks:S.data.A.rows.map(r=>r[1]),anchor:S.data.A.anchor,camera:S.camera}));
      windows.push([state.lo,state.hi]);for(const tick of state.ticks)seen.add(tick);
      assert.deepEqual(state.anchor,fullAnchor);assert.deepEqual(state.camera,fullCamera);
      await page.evaluate(()=>{S.time=S.hi-.000001;});
      if(state.hi===state.end) {await page.waitForFunction(()=>!S.full&&!S.playing);break;}
      await page.waitForFunction(lo=>S.lo>lo&&S.playing&&!S.loading,state.lo);
    }
    assert.equal(seen.size,await page.evaluate(()=>selectedTrack('A').count));
    assert.equal(windows.length,11);
    // Copy/restoring a late window also retains its original global anchor.
    const wholeState=await page.evaluate(()=>snapshot());
    const whole=await context.newPage();whole.on('pageerror',e=>errors.push(e.message));
    await whole.goto(base+'/#'+new URLSearchParams({review:JSON.stringify(wholeState)}));
    await whole.waitForFunction(()=>!!S.data.A?.points&&!S.loading);
    assert.deepEqual(await whole.evaluate(()=>snapshot()),wholeState);
    await whole.close();

    // Sensor tick arithmetic and q/-q hemisphere changes must not create gaps
    // or spurious rotations. Explicit invalid samples must break interpolation.
    assert.equal(await page.evaluate(() => connected([0,4294967200,48000000,...Array(17).fill(0),1], [.004,4295159200,48000000,...Array(17).fill(0),1])), true);
    assert.deepEqual(await page.evaluate(() => slerp([1,0,0,0],[-1,0,0,0],.5)), [1,0,0,0]);
    assert.equal(await page.evaluate(() => {const a=S.data.A.rows[0].slice(),b=S.data.A.rows[1].slice();b[20]=0;return connected(a,b);}), false);
    assert.equal(await page.evaluate(() => {const a=S.data.A.rows[0].slice(),b=S.data.A.rows[1].slice();b[1]=a[1]+a[2]*.03;return connected(a,b);}), false);
    assert.equal(await page.evaluate(()=>{
      const d=S.data.A,t=d.rows[0][0],previous=d.rows[0].slice();previous[0]=t-.004;
      return sampleAt({rows:d.rows,previous,offset:0},t-.002)===previous;
    }),true);

    await page.locator('#presetCompare').click();
    await page.waitForFunction(() => !!S.data.B?.points, {timeout: 240000});
    assert.equal(await page.evaluate(() => S.data.B.rows.length), 1499);
    await page.locator('#basis').selectOption('body');
    await page.locator('#rotationGain').selectOption('5');
    await page.locator('#scrub').evaluate(el => {el.value='263.552427';el.dispatchEvent(new Event('input'));});
    await page.locator('#next').click();
    assert.equal(await page.evaluate(() => S.data.A.rows.some(r=>r[0]===S.time)), true);
    await page.locator('#selectLo').fill('263.5');
    await page.locator('#selectHi').fill('265.5');
    await page.locator('#comment').fill('同じ方向へ滑って戻る / test');
    await page.locator('#copy').click();
    await page.waitForFunction(()=>!!document.querySelector('#copied').textContent);
    assert.match(await page.locator('#copied').innerText(), /コピーしました|手動/);
    const text=await page.locator('#shareText').inputValue();
    assert.match(text, /range: 263.500000 – 265.500000 s/);
    assert.match(text, /sensor ticks/);
    assert.match(text, /SHA-256: [a-f0-9]{64}/);
    const shared=page.url();
    const state=await page.evaluate(() => snapshot());
    const restored=await context.newPage();
    restored.on('pageerror', e=>errors.push(e.message));
    await restored.goto(shared);
    await restored.waitForFunction(() => !!S.data.B?.points && document.querySelector('#selectLo').value==='263.5', {timeout:240000});
    assert.deepEqual(await restored.evaluate(() => snapshot()), state);
    await restored.screenshot({path:output+'/comparison.png',fullPage:true});

    // JSON state is rendered as text; source IDs cannot become file paths.
    const bad=await context.request.get(base+'/api/source?id=../../etc/passwd');
    assert.equal(bad.status(),400);
    assert.equal((await context.request.get(base+'/../../etc/passwd')).status(),404);
    const tooLong=await context.request.get(base+'/api/window?'+new URLSearchParams({id:state.a,track:state.ta,lo:0,hi:61}));
    assert.equal(tooLong.status(),400);
    const mismatch={...state,shaA:'0'.repeat(64)};
    await restored.goto(base+'/#'+new URLSearchParams({review:JSON.stringify(mismatch)}));
    await restored.waitForFunction(()=>document.querySelector('#status').className==='error');
    assert.match(await restored.locator('#status').innerText(),/SHA-256/);

    // Actual playback progresses; static charts are cached without decimation.
    const before=await page.evaluate(()=>S.time);
    await page.locator('#play').click();
    await page.waitForFunction(t=>S.time>t+.1, before);
    await page.locator('#play').click();
    const perf=await page.evaluate(()=>{
      const start=performance.now();for(let i=0;i<30;i++)drawAll();
      return {mean_draw_ms:(performance.now()-start)/30,samples:S.data.A.rows.length,decimation:S.data.A.decimation};
    });
    assert.equal(perf.decimation,false);
    const mobile=await context.newPage();
    await mobile.setViewportSize({width:390,height:844});
    await mobile.goto(shared);
    await mobile.waitForFunction(()=>!!S.data.B?.points,{timeout:240000});
    assert.equal(await mobile.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    await mobile.screenshot({path:output+'/mobile.png',fullPage:true});
    assert.deepEqual(errors,[]);
    console.log(JSON.stringify({result:'PASS',base,perf,whole_record:{windows:windows.length,unique_samples:seen.size},screenshots:output,checks:'raw samples, gap/tick/quaternion handling, perspective and clipping, device schematics, other devices/BS, reconstructed optical provenance, dual endpoints by mouse/keyboard, all-sample whole playback with fixed anchor, late-window link roundtrip, comparison, sample step, copy on HTTP, SHA mismatch, path restrictions, window limits, playback, mobile'},null,2));
  } finally { await browser.close(); }
})().catch(error=>{console.error(error);process.exitCode=1;});
