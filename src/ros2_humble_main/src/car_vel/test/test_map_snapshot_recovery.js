// Production recovery helper, no ROS/network/robot I/O.
'use strict';
const assert=require('node:assert/strict'),fs=require('fs'),vm=require('vm'),path=require('path');
const window={location:{href:'http://test/navigation.html',hostname:'test'},setTimeout,clearTimeout};
const document={addEventListener(){},removeEventListener(){}};
vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../car_vel/web_frontend/robot_ui_common.js'),'utf8'),
    {window,document,URL,console,ROSLIB:{}});
const Recovery=window.RobotUi.MapSnapshotRecovery;
assert.equal(typeof Recovery,'function','missing reliable map snapshot recovery');
const map={header:{frame_id:'map'},info:{width:2,height:2,resolution:.05,
    origin:{position:{x:0,y:0,z:0},orientation:{w:1}}},data:[0,0,100,-1]};
(async()=>{
    let resolve,calls=0,accepted=[],valid=true;
    const session={connected:true,call(name,type,request,timeout){
        calls++;assert.equal(name,'/map_server/map');assert.equal(type,'nav_msgs/srv/GetMap');
        assert(timeout>0&&timeout<=25000);return new Promise(done=>{resolve=done;});}};
    const recovery=new Recovery(session,msg=>accepted.push(msg),()=>{});
    const first=recovery.request('load-a',()=>valid);
    recovery.request('load-a',()=>valid);assert.equal(calls,1,'single in-flight request per load');
    resolve({map});await first;assert.equal(accepted.length,1);
    const stale=recovery.request('load-b',()=>valid);
    recovery.cancel();resolve({map});await stale;assert.equal(accepted.length,1,'late cancelled response ignored');
    valid=false;await recovery.request('load-c',()=>valid);assert.equal(calls,2,'no request for obsolete identity');
    valid=true;
    const invalid=recovery.request('load-d',()=>valid);resolve({map:{...map,data:[0]}});
    await invalid;assert.equal(accepted.length,1,'incomplete raster cannot mark map ready');recovery.cancel();
    const offline=recovery.request('load-e',()=>valid);session.connected=false;resolve({map});await offline;
    assert.equal(accepted.length,1,'disconnected session cannot deliver late raster');recovery.cancel();
    session.connected=true;
    const obsolete=recovery.request('load-f',()=>valid);valid=false;resolve({map});await obsolete;
    assert.equal(accepted.length,1,'changed load identity ignored even before explicit cancel');recovery.cancel();
    const timers=[];
    window.setTimeout=(callback,delay)=>{const timer={callback,delay};timers.push(timer);return timer;};
    window.clearTimeout=()=>{};
    let attempts=0;
    const failed=new Recovery({connected:true,call:async()=>{attempts++;throw new Error('fixture missing map');}},()=>{},()=>{});
    await failed.request('retry',()=>true);
    await timers[0].callback();await timers[1].callback();
    assert.equal(attempts,3);assert.equal(timers.length,2,'retry must be bounded');
    failed.cancel();
    console.log('PASS: reliable GetMap snapshot, duplicate/cancel/stale/incomplete/disconnected guards');
})().catch(error=>{console.error(error);process.exitCode=1;});
