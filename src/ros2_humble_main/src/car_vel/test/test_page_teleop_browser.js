/* One takeover scenario on both pages; every ROS/HTTP response is offline. */
const assert=require('node:assert/strict'), fs=require('node:fs'), path=require('node:path');
const {chromium}=require('playwright');
const web=path.resolve(__dirname,'../car_vel/web_frontend');
(async()=>{
 const browser=await chromium.launch({headless:true,executablePath:process.env.NAV_TEST_BROWSER||'/usr/bin/google-chrome',args:['--no-sandbox']});
 try {for(const name of ['navigation','inspection']) {
  const page=await browser.newPage({viewport:{width:1440,height:1000}}), wire=[], errors=[];
  let socket, failCancel=true, timer;
  const mode={mode:'map_2d',inspection_active:true,campus_mission_active:false};
  const task={active:true,state:'navigating',paused:false};
  const dog={body_id:'xingji',connected:true,remote_active:false,stale:false,motion_mode:2,emergency_latched:false};
  function send(topic,data,json=true){socket?.send(JSON.stringify({op:'publish',topic,msg:{data:json?JSON.stringify(data):data}}));}
  function status(){send('/navigation/mode',mode);send('/inspection/status',task);send('/dog_status',dog);}
  page.on('pageerror',e=>errors.push(e.message));
  await page.routeWebSocket('**/*',mock=>{
   socket=mock;timer=setInterval(status,200);timer.unref();
   mock.onMessage(raw=>{
    const m=JSON.parse(String(raw));wire.push(m);
    if(m.op==='subscribe') {
     status();
     if(m.topic==='/navigation_active')send(m.topic,true,false);
     if(m.topic==='/relocalization/status')send(m.topic,{state:'idle',terminal:false});
    }
    if(m.op==='publish') {
     if(m.topic==='/cancel_navigation')setTimeout(()=>send('/navigation_active',false,false),150);
     if(m.topic==='/dog_remote/enable'){dog.remote_active=m.msg.data;status();}
     if(m.topic==='/motion_stop'){dog.emergency_latched=true;status();}
     if(m.topic==='/clear_emergency_stop'){dog.emergency_latched=false;status();}
    }
    if(m.op!=='call_service')return;
    let values={success:true,message:'offline',map_names:[]};
    if(m.service==='/inspection/control') {
     if(m.args.command==='pause'&&failCancel){values={success:false,message:'pause test failure'};failCancel=false;}
     else if(m.args.command==='pause'){task.active=true;task.state='paused';task.paused=true;status();}
     else if(m.args.command==='resume'){task.state='navigating';task.paused=false;status();}
    }
    if(m.service.endsWith('/get_parameters'))values={values:[{type:3,double_value:.5}]};
    mock.send(JSON.stringify({op:'service_response',id:m.id,service:m.service,result:true,values}));
   });
  });
  await page.route('**/*',route=>{
   const url=new URL(route.request().url()),file=path.resolve(web,'.'+url.pathname);
   if(url.hostname!=='takeover.test'||!file.startsWith(web+path.sep)||!fs.existsSync(file))return route.abort();
   const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'};
   return route.fulfill({contentType:types[path.extname(file)]||'application/octet-stream',body:fs.readFileSync(file)});
  });
  await page.goto(`http://takeover.test/${name}.html`);
  const root=page.locator(`#${name}-page-teleop`), take=root.locator('[data-manual=take]');
  await take.waitFor();await page.waitForTimeout(250);
  await page.keyboard.press('q');
  assert(!wire.some(m=>m.op==='publish'&&m.topic==='/dog_remote/cmd_vel'),'inactive page must not send manual commands');
  await take.click();await root.locator('[data-manual=status]').filter({hasText:'接管失败'}).waitFor();
  assert(!wire.some(m=>m.topic==='/dog_remote/enable'&&m.msg?.data===true),'failed automatic stop must not grant control');
  await take.click();await root.locator('[data-drive=q]').waitFor();
  await page.waitForFunction(id=>!document.querySelector(`#${id}-page-teleop [data-drive=q]`).disabled,name);
  const enableIndex=wire.findIndex(m=>m.topic==='/dog_remote/enable'&&m.msg?.data===true);
  assert(wire.slice(0,enableIndex).some(m=>m.op==='call_service'&&m.service==='/inspection/control'&&m.args.command==='pause'));
  assert(!wire.some(m=>m.op==='publish'&&m.topic==='/cancel_navigation'),'takeover must preserve the paused route');
  await page.keyboard.down('q');await page.waitForTimeout(140);await page.keyboard.up('q');await page.waitForTimeout(80);
  const motion=wire.filter(m=>m.op==='publish'&&m.topic==='/dog_remote/cmd_vel');
  assert(motion.some(m=>m.msg.angular.z>=.39));
  assert.equal(motion.at(-1).msg.angular.z,0);
  await root.locator('[data-manual=lock]').click();
  await page.waitForFunction(id=>document.querySelector(`#${id}-page-teleop [data-drive=q]`).disabled,name);
  await root.locator('[data-manual=reset]').click();
  await page.evaluate(()=>window.dispatchEvent(new Event('blur')));
  await page.waitForFunction(id=>document.querySelector(`#${id}-page-teleop [data-manual=controls]`).hidden,name);
  assert(wire.some(m=>m.topic==='/dog_remote/enable'&&m.msg?.data===false));
  assert(!wire.some(m=>m.op==='call_service'&&m.args?.command==='resume'));
  await root.locator('[data-manual=resume]').click();
  await page.waitForFunction(id=>document.querySelector(`#${id}-page-teleop [data-manual=status]`).textContent.includes('从当前位置继续'),name);
  assert(wire.some(m=>m.op==='call_service'&&m.service==='/inspection/control'&&m.args.command==='resume'));
  assert(!wire.some(m=>m.op==='call_service'&&m.args?.command==='cancel'));
  assert(!wire.some(m=>m.op==='call_service'&&m.service==='/control_launch'&&m.args.action==='stop'),'takeover must keep map/navigation nodes alive');
  assert.deepEqual(errors,[]);
  clearInterval(timer);await page.close();console.log('PASS:',name,'takeover pause failure, manual turn, stop, lock/reset and focus-loss release');
 }} finally {await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
