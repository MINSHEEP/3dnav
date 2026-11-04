/* Real editor and patrol DOM, all HTTP/WebSocket mocked. No robot traffic. */
'use strict';
const assert=require('node:assert/strict'),fs=require('fs'),path=require('path');
const {chromium}=require('playwright');
const web=path.resolve(__dirname,'../car_vel/web_frontend');
(async()=>{
 const browser=await chromium.launch({headless:true,...(process.env.NAV_TEST_BROWSER?{executablePath:process.env.NAV_TEST_BROWSER}:{})});
 try {
  const wire=[],errors=[];let cloudFail=false,emptyKept=false,noChange=false,groundOnly=false;
  const page=await browser.newPage({viewport:{width:1440,height:900}});
  page.on('pageerror',error=>errors.push(error.message));
  page.on('dialog',dialog=>dialog.accept());
  await page.routeWebSocket('**/*',socket=>socket.onMessage(raw=>{
   const m=JSON.parse(String(raw));wire.push(m);
   if(m.op!=='call_service')return;
   let values={success:true,message:'mock success'};
   if(m.service==='/get_map_list')values.map_names=['fixture'];
   if(m.service==='/get_edited_map')Object.assign(values,{width:10,height:10,resolution:.1,
     origin:[0,0,0],image_data:Array(100).fill(254),has_changes:false});
   if(m.service==='/edit_map_cloud'){
    values.success=!cloudFail;values.message=cloudFail?'等待停止确认失败':'mock cleanup';
    values.result_json=JSON.stringify({preview_id:noChange?'':'preview-1',committable:!noChange,
     delete_count:(noChange||groundOnly)?0:2,total_count:3000,
     reason_counts:{ground:2,height:1,terrain:0,ground_unknown:0,no_floor:0,not_black:0},relative_height_range_m:[0,.9],
     protected_points:3,cleared_pixels:noChange?0:1,removed_xyz:(noChange||groundOnly)?[]:[.15,.15,.4,.16,.15,.4],
     kept_xyz:emptyKept?[]:[.15,.15,0,.16,.15,0,.17,.15,0],backup_path:'/mock/backup'});
   }
   socket.send(JSON.stringify({op:'service_response',id:m.id,service:m.service,result:true,values}));
  }));
  await page.route('**/*',async route=>{
   const name=path.basename(new URL(route.request().url()).pathname)||'map_editor.html';
   const file=path.join(web,name);
   if(!fs.existsSync(file))return route.fulfill({status:404,body:''});
   let body=fs.readFileSync(file);
   if(name==='map_editor.html')body=Buffer.from(body.toString().replace("    log('修图页面已加载",`    window.__editFixture={select:cells=>cloudEdit.select(cells),state:()=>({token:cloudEdit.token,busy:serviceBusy,shape:mapCanvas.editorInteraction?.shape,cells:cloudEdit.cells,map:Array.from(mapCanvas.mapData||[])}),point:(x,y)=>{const p=mapCanvas.mapToCanvas(x,y),r=mapCanvas.canvas.getBoundingClientRect();return {x:r.x+p.x,y:r.y+p.y};},stopped:()=>coreStopped};\n    log('修图页面已加载`));
   await route.fulfill({body,contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.png')?'image/png':'text/html'});
  });
  await page.goto('http://fixture/map_editor.html');
  await page.waitForFunction(()=>window.__editFixture?.stopped());
  await page.selectOption('#map-selector','fixture');await page.click('#btn-open-map');
  await page.waitForFunction(()=>!document.querySelector('#cloud-edit-enabled').disabled);
  await page.check('#cloud-edit-enabled');
  assert.equal(await page.locator('#btn-shape-brush').isEnabled(),true,'linked eraser supports brush');
  await page.evaluate(()=>{const r=document.querySelector('#brush-radius');r.value=0;r.dispatchEvent(new Event('input'));});
  const originalMap=(await page.evaluate(()=>window.__editFixture.state())).map;
  for(const [x,y] of [[.55,.55],[.75,.75]]){
    const p=await page.evaluate(([x,y])=>window.__editFixture.point(x,y),[x,y]);
    assert.equal(await page.evaluate(p=>document.elementFromPoint(p.x,p.y)?.id,p),'map-canvas','pointer must be on visible canvas');
    await page.mouse.click(p.x,p.y);
  }
  let state=await page.evaluate(()=>window.__editFixture.state());
  assert.deepEqual(state.cells.map(c=>c.serviceIndex).sort(),[27,45],'real mouse strokes use exact selected pixels');
  assert.deepEqual(state.map,originalMap,'linked mouse strokes never repaint source PGM');
  await page.click('#btn-cloud-clear');
  await page.click('#btn-shape-rectangle');
  assert.equal((await page.evaluate(()=>window.__editFixture.state())).shape,'rectangle');
  await page.evaluate(()=>window.__editFixture.select([{serviceIndex:81,dataIndex:11}]));
  await page.evaluate(()=>window.__editFixture.select([{serviceIndex:82,dataIndex:12},{serviceIndex:81,dataIndex:11}]));
  assert.equal(wire.filter(m=>m.service==='/set_map_pixel_indices').length,0);
  await page.click('#btn-cloud-preview');
  await page.waitForFunction(()=>window.__editFixture.state().token==='preview-1');
  const linked=wire.filter(m=>m.service==='/edit_map_cloud').at(-1);
  assert.equal(linked.args.action,'preview_linked');
  assert.deepEqual(linked.args.indices.slice().sort(),[81,82],'strokes union, not rectangle bounding box');
  assert.ok((await page.locator('#cloud-edit-reasons').textContent()).includes('地面保留 2'));
  await page.click('#btn-cloud-undo');
  assert.equal(await page.locator('#btn-cloud-commit').isDisabled(),true,'undo invalidates preview');
  await page.click('#btn-cloud-preview');
  await page.waitForFunction(()=>window.__editFixture.state().token==='preview-1');
  assert.deepEqual(wire.filter(m=>m.service==='/edit_map_cloud').at(-1).args.indices,[81]);
  assert.equal(await page.locator('#btn-cloud-commit').isDisabled(),true,'confirmation required');
  await page.check('#cloud-edit-confirm');assert.equal(await page.locator('#btn-cloud-commit').isEnabled(),true);
  await page.fill('#cloud-edit-max','0.9');
  assert.equal(await page.locator('#btn-cloud-commit').isDisabled(),true,'height change invalidates token');
  emptyKept=true;
  await page.click('#btn-cloud-preview');await page.waitForFunction(()=>window.__editFixture.state().token==='preview-1');
  assert.ok((await page.locator('#cloud-edit-summary').textContent()).includes('框内无保留点显示'));
  await page.check('#cloud-edit-confirm');await page.click('#btn-cloud-commit');
  await page.waitForFunction(()=>!window.__editFixture.state().busy);
  assert.equal(wire.filter(m=>m.service==='/edit_map_cloud'&&m.args.action==='commit').length,1);
  await page.click('#btn-cloud-restore');await page.waitForFunction(()=>!window.__editFixture.state().busy);
  assert.equal(wire.filter(m=>m.service==='/edit_map_cloud'&&m.args.action==='restore').length,1);
  await page.evaluate(()=>window.__editFixture.select([{serviceIndex:81,dataIndex:11}]));
  noChange=true;await page.click('#btn-cloud-preview');await page.waitForFunction(()=>!window.__editFixture.state().busy);
  assert.equal(await page.locator('#cloud-edit-preview').isVisible(),true,'zero-delete preview explains retained points');
  await page.check('#cloud-edit-confirm');assert.equal(await page.locator('#btn-cloud-commit').isDisabled(),true);
  noChange=false;groundOnly=true;await page.click('#btn-cloud-preview');
  await page.waitForFunction(()=>window.__editFixture.state().token==='preview-1');
  await page.check('#cloud-edit-confirm');assert.equal(await page.locator('#btn-cloud-commit').isEnabled(),true,'2D-only mislabel correction can commit');
  const editorLayout=await page.evaluate(()=>({height:document.documentElement.scrollHeight,map:document.querySelector('#map-wrapper').getBoundingClientRect().toJSON()}));
  assert.ok(editorLayout.height<=900,'desktop editor must not grow with preview/sidebar');
  assert.ok(editorLayout.map.bottom<=901,'full canvas stays inside viewport');
  await page.locator('#cloud-edit-summary').scrollIntoViewIfNeeded();
  await page.screenshot({path:path.resolve(__dirname,'../../../../../tmp/map-linked-erase-editor.png')});
  await page.click('#btn-cloud-clear');assert.equal(await page.locator('#btn-cloud-preview').isDisabled(),true);
  cloudFail=true;await page.evaluate(()=>window.__editFixture.select([{serviceIndex:81,dataIndex:11}]));
  await page.click('#btn-cloud-preview');await page.waitForFunction(()=>!window.__editFixture.state().busy);
  assert.equal(await page.locator('#btn-cloud-commit').isDisabled(),true);
  cloudFail=false;groundOnly=false;
  await page.selectOption('#cloud-edit-scope','region');
  assert.equal(await page.locator('#btn-cloud-commit').isDisabled(),true,'scope change invalidates preview');
  await page.click('#btn-cloud-preview');await page.waitForFunction(()=>window.__editFixture.state().token==='preview-1');
  assert.equal(wire.filter(m=>m.service==='/edit_map_cloud').at(-1).args.action,'preview','legacy whole-selection tool remains accessible');
  await page.uncheck('#cloud-edit-enabled');await page.click('#btn-tool-free');await page.click('#btn-shape-brush');
  const p=await page.evaluate(()=>window.__editFixture.point(.55,.55));await page.mouse.click(p.x,p.y);
  assert.equal(await page.evaluate(p=>document.elementFromPoint(p.x,p.y)?.id,p),'map-canvas');
  await page.waitForFunction(()=>!window.__editFixture.state().busy);
  const normal=wire.filter(m=>m.service==='/set_map_pixel_indices');
  assert.equal(normal.length,1,'ordinary brush still uses original 2D service');
  assert.equal(normal[0].args.pixel_value,254);
  assert.equal(wire.filter(m=>m.op==='publish'||m.service==='/start_route').length,0);
  assert.deepEqual(errors,[]);
  console.log('PASS editor: real brush strokes/rectangle exact union, undo/clear, reasons and ground-only preview, explicit confirm, stale height, commit/restore, fail-closed and original 2D brush; no motion');
  await page.close();

  const patrol=await browser.newPage();
  await patrol.routeWebSocket('**/*',socket=>socket.onMessage(raw=>{
    const m=JSON.parse(String(raw));
    if(m.op==='call_service')socket.send(JSON.stringify({op:'service_response',id:m.id,service:m.service,result:true,
      values:{success:true,message:'mock',map_names:[],routes:[],tasks:[],history:[],result_json:'{}'}}));
  }));
  await patrol.route('**/*',async route=>{
    const name=path.basename(new URL(route.request().url()).pathname)||'inspection.html',file=path.join(web,name);
    if(!fs.existsSync(file))return route.fulfill({status:404,body:''});
    await route.fulfill({body:fs.readFileSync(file),contentType:name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':name.endsWith('.png')?'image/png':'text/html'});
  });
  await patrol.goto('http://fixture/inspection.html');
  for(const [width,height] of [[1890,870],[1366,768],[1024,768],[390,844]]){
    await patrol.setViewportSize({width,height});
    await patrol.evaluate(()=>{
      document.querySelector('#inspection-3d-toolbar').hidden=false;
      document.querySelector('#inspection-3d-action-status').textContent='静止定位待更新：可以选点，预览前会自动刷新并再次校验，不会自动启动。';
      document.querySelector('#connection-status').textContent='已连接 ROS2（ws://100.109.124.106:9090）';
      document.querySelector('#camera-message').textContent='正在连接YOLO结果流，请等待视频服务返回第一帧。';
    });
    await patrol.waitForTimeout(100);
    const measurements=await patrol.evaluate(()=>{
      const rect=s=>document.querySelector(s).getBoundingClientRect().toJSON();
      return {header:rect('.topbar'),toolbar:rect('#inspection-3d-toolbar'),legend:rect('.inspection-legend'),
       map:rect('#map-wrapper'),bodyWidth:document.documentElement.scrollWidth,
       links:[...document.querySelectorAll('.page-nav a')].map(a=>{
          const r=document.createRange();r.selectNodeContents(a);return {text:a.textContent,height:r.getBoundingClientRect().height};})};
    });
    assert.ok(measurements.bodyWidth<=width+1,`no page horizontal overflow at ${width}`);
    assert.ok(measurements.legend.top>=measurements.toolbar.bottom,`legend below toolbar at ${width}`);
    assert.ok(measurements.map.top>=measurements.legend.bottom,`map below legend at ${width}`);
    for(const a of measurements.links)assert.ok(a.height<35,`nav label must not wrap: ${width} ${a.text}`);
    if(width>1050)assert.ok(measurements.header.height<140,`compact desktop header at ${width}`);
    if(width===1890)await patrol.screenshot({path:path.resolve(__dirname,'../../../../../tmp/inspection-layout-1890.png')});
    console.log('PASS inspection layout',width,height,JSON.stringify(measurements.header));
  }
  await patrol.close();
 } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
