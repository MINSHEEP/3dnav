const assert = require('assert'), fs = require('fs'), path = require('path'), vm = require('vm');
function fixture() {
    const elements = {};
    for (const id of ['ip','host-ip','save','refresh','form','state','result']) elements['lidar-'+id] = {
        value:'',disabled:false,textContent:'',events:{},
        addEventListener(event,fn){this.events[event]=fn;},reportValidity(){return true;},
    };
    let now=10000, receive;
    const calls=[], window={RobotUi:{TYPES:{string:'std_msgs/msg/String'}},setInterval(fn){this.tick=fn;}};
    vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../car_vel/web_frontend/lidar_settings.js'),'utf8'),
        {window,document:{getElementById:id=>elements[id]},Date:{now:()=>now}});
    const state={config:{version:0,lidar_ip:'192.168.1.130',host_ip:'192.168.1.50'},error:'',warning:''};
    const session={subscribe(topic,type,fn){receive=fn;},async call(...args){
        calls.push(args);return {success:true,message:'saved',status_json:JSON.stringify(state)};
    }};
    const app=window.LidarSettings.create(()=>{});app.attach(session);app.connection(true);
    return {app,session,state,calls,get:id=>elements['lidar-'+id],emit:()=>receive({data:JSON.stringify(state)}),
        advance(){now+=7000;window.tick();}};
}
(async()=>{
    const f=fixture();assert(f.get('save').disabled);f.emit();assert(!f.get('save').disabled);
    assert.equal(f.get('ip').value,'192.168.1.130');
    f.get('ip').value='192.168.1.131';f.get('ip').events.input();
    f.state.config.version=1;f.emit();assert.equal(f.get('ip').value,'192.168.1.131');
    await f.get('save').events.click();assert.equal(f.calls[0][2].expected_version,0,'dirty editor preserves CAS version');
    assert.equal(f.calls[0][1],'lzp_interface/srv/SetLidarNetwork');
    assert.equal(f.calls[0][2].lidar_ip,'192.168.1.131');
    f.advance();assert(f.get('save').disabled);f.emit();f.app.connection(false);assert(f.get('save').disabled);
    f.app.connection(true);f.emit();f.state.config=null;f.state.error='broken';f.emit();assert(f.get('save').disabled);
    const g=fixture();g.emit();let finish;
    g.session.call=()=>new Promise(resolve=>{finish=resolve;});
    const pending=g.get('save').events.click();assert(g.get('ip').disabled);assert(g.get('refresh').disabled);
    finish({success:false,message:'reject',status_json:JSON.stringify(g.state)});await pending;
    assert(!g.get('ip').disabled);assert.equal(g.get('result').textContent,'reject');
    g.session.call=async()=>{throw Error('timeout');};await g.get('save').events.click();
    assert(g.get('result').textContent.includes('刷新'));assert(!g.get('ip').disabled);
    console.log('LiDAR settings: status, dirty/CAS, offline/stale, errors, busy and timeout PASS');
})().catch(error=>{console.error(error);process.exitCode=1;});
