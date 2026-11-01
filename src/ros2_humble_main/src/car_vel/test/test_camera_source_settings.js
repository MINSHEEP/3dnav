const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function fixture() {
    const elements = {};
    for (const id of ['source','device','path','url','width','height','fps','refresh','save',
        'usb-fields','body-fields','state','form','result']) elements['ordinary-camera-'+id] = {
        value:'', disabled:false, hidden:false, textContent:'', children:[], events:{},
        addEventListener(type,fn){this.events[type]=fn;},
        replaceChildren(){this.children=[];this.value='';},
        appendChild(option){this.children.push(option);},reportValidity(){return true;}
    };
    let now=10000, offline=false, postWait=null;
    const status={success:true,config:{version:0,source:'usb',usb_device:'/dev/v4l/by-id/normal-video-index0',
        usb_width:1280,usb_height:720,usb_fps:30,body_url:'http://127.0.0.1:8081/camera'},
        devices:[{device:'/dev/v4l/by-id/normal-video-index0',node:'/dev/video2',name:'USB Camera',aliases:[]}],
        locked_reason:'',switching:false};
    const calls=[];
    const window={location:{hostname:'192.168.66.55'},fetch:async()=>{},
        setTimeout(){return 1;},clearTimeout(){},setInterval(fn){this.tick=fn;return 1;},clearInterval(){}};
    const fetch=async(url,options)=>{
        calls.push({url,options});
        if(offline)throw Error('offline');
        if(options.method==='POST'){
            if(postWait)await postWait;
            const payload=JSON.parse(options.body);
            if(payload.expected_version!==status.config.version)return {ok:false,status:400,json:async()=>({error:'配置已更新'})};
            status.config={...payload.config,version:status.config.version+1};
        }
        return {ok:true,json:async()=>JSON.parse(JSON.stringify(status))};
    };
    vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../car_vel/web_frontend/camera_source_settings.js'),'utf8'),
        {window,document:{getElementById:id=>elements[id],createElement:()=>({})},
            Date:{now:()=>now},AbortController:class{constructor(){this.signal={};}abort(){}}});
    const app=window.CameraSourceSettings.create(()=>{},{fetch});
    const get=name=>elements['ordinary-camera-'+name];
    return {app,get,status,calls,window,offline(v){offline=v;},advance(ms){now+=ms;},
        deferPost(){let resolve;postWait=new Promise(r=>resolve=r);return resolve;}};
}

(async()=>{
    let f=fixture(); assert(f.get('save').disabled);
    await f.app.refresh(); assert(!f.get('save').disabled);
    assert.equal(f.get('path').value,f.status.config.usb_device);
    assert.equal(f.calls[0].url,'http://192.168.66.55:8766/api/camera/source');
    f.get('source').value='body';f.get('source').events.change();
    assert(f.get('usb-fields').hidden && !f.get('body-fields').hidden);
    assert(f.get('path').disabled && !f.get('url').disabled,'inactive inputs cannot block validity');
    await f.app.save();
    let payload=JSON.parse(f.calls[1].options.body);
    assert.equal(payload.expected_version,0); assert.equal(payload.config.source,'body');
    assert.equal(payload.config.body_url,'http://127.0.0.1:8081/camera');

    f=fixture();await f.app.refresh();
    f.get('path').value='/dev/video8';f.get('path').events.input();
    f.status.config.version=3;await f.app.refresh();
    assert.equal(f.get('path').value,'/dev/video8','poll preserves dirty edit');
    await f.app.save();
    assert.equal(JSON.parse(f.calls.at(-1).options.body).expected_version,0,'dirty version cannot silently authorize an overwrite');
    assert(f.get('result').textContent.includes('配置已更新'));
    await f.app.refresh(true);assert.equal(f.get('path').value,f.status.config.usb_device);

    f=fixture();await f.app.refresh();f.status.locked_reason='请先结束巡检任务';await f.app.refresh();
    assert(f.get('save').disabled && f.get('state').textContent.includes('巡检'));
    f.status.locked_reason='';await f.app.refresh();f.advance(7000);f.offline(true);await f.app.refresh();
    assert(f.get('save').disabled,'offline or stale state does not authorize source changes');

    f=fixture();await f.app.refresh();const finish=f.deferPost();const pending=f.app.save();
    assert(f.get('save').disabled && f.get('source').disabled && f.get('refresh').disabled);
    const count=f.calls.length;await f.app.refresh();assert.equal(f.calls.length,count,'no conflicting refresh while saving');
    finish();await pending;assert(!f.get('save').disabled);
    f.app.stop();assert(f.get('save').disabled);
    console.log('PASS: camera settings defaults, body/USB fields, save, dirty/stale versions, activity, offline and in-flight exclusion');
})().catch(error=>{console.error(error);process.exitCode=1;});
