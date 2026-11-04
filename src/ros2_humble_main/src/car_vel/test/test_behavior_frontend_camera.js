#!/usr/bin/env node
'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const web = path.resolve(__dirname, '../car_vel/web_frontend');
const html = fs.readFileSync(path.join(web, 'camera.html'), 'utf8');
const script = html.slice(html.lastIndexOf('<script>') + 8, html.lastIndexOf('</script>'));
const requests = [], timers = new Map(), intervals = new Map(), images = [];
let timerId = 0, time = 10000, mode = 'stopped', released = false;
class Node {
    constructor(id) {
        this.id = id; this.handlers = new Map(); this.children = new Map(); this.textContent = ''; this.hidden = false;
        this.attributes = new Map(); this.classList = {add(){},remove(){}};
    }
    addEventListener(name, fn) { if (!this.handlers.has(name)) this.handlers.set(name,new Set()); this.handlers.get(name).add(fn); }
    removeEventListener(name, fn) { this.handlers.get(name)?.delete(fn); }
    emit(name, data={}) { for(const fn of this.handlers.get(name)||[]) fn(data); }
    set src(value) { this.attributes.set('src',value); images.push({id:this.id,src:value}); }
    get src() { return this.attributes.get('src') || ''; }
    removeAttribute(key) { this.attributes.delete(key); }
    getAttribute(key) { return this.attributes.get(key) ?? null; }
    querySelector(key) { if(!this.children.has(key))this.children.set(key,new Node(key));return this.children.get(key); }
}
const document = new Node('document'), window = new Node('window');
const elements = new Map([...html.matchAll(/\bid="([^"]+)"/g)].map(match => [match[1], new Node(match[1])]));
document.hidden=false; document.activeElement=null;
document.querySelector = selector => elements.get(selector.slice(1));
const get = id => elements.get(id);
const behavior = () => ({running:mode!=='stopped',ready:mode==='ready'||mode==='failed',fresh:mode==='ready'||mode==='failed',
    source_epoch:1,stamp_ns:1793111111000000000,frame_stamp_ns:'1793111111000000017',frame_age_seconds:0,max_result_age_seconds:5,
    enabled:{fall:true,fight:true},branches:{fall:{enabled:true,state:mode==='starting'?'starting':'valid',ready:mode!=='starting'},
        fight:{enabled:true,state:mode==='starting'?'warming':'valid',ready:mode!=='starting'}}});
const detection = () => ({running:mode!=='stopped',ready:mode==='ready',
    yolo:{running:mode==='ready'||mode==='starting',ready:mode==='ready',error:mode==='failed'?'exited':null},
    thermal:{ready:true,manual_detection_enabled:mode!=='stopped',detection_enabled:mode!=='stopped'},behavior:behavior()});
Object.assign(window, {document,location:{protocol:'http:',hostname:'robot',reload(){}},AbortController,
    URL:{createObjectURL:()=>`blob:frame-${time}`,revokeObjectURL(){}},
    setTimeout(fn,delay){const id=++timerId;timers.set(id,{fn,due:time+delay});return id;},
    clearTimeout(id){timers.delete(id);},
    setInterval(fn,delay){const id=++timerId;intervals.set(id,{fn,delay});return id;},clearInterval(id){intervals.delete(id);},
    open(){},fetch:async(url,options={})=>{
        requests.push({url,options}); let body;
        if(url.includes('/api/behavior/status')) body=behavior();
        else if(url.includes('/api/behavior/frame.jpg')) return {ok:true,blob:async()=>({}),headers:{get:key=>key==='X-Behavior-Source-Epoch'?'1':'1793111111000000017'}};
        else if(url.endsWith('/api/detection')) { mode=mode==='stopped'?'starting':'stopped';body=detection(); }
        else if(url.endsWith('/api/status')) body={...detection(),detection:detection(),connected:true,
            camera_source:{generation:1,label:'USB摄像头'},video:{device:'/dev/video0'}};
        else throw new Error(`unexpected request ${url}`);
        return {ok:true,json:async()=>body};
    },RobotUi:{
        previewUrl:url=>url,previewAllowed:()=>!document.hidden,previewControls(){},
        setBatteryDisconnected(){},subscribeDogStatus(){},
        RosSession:class{connect(){}stop(){released=true;}}
    }});
class ClockDate extends Date {static now(){return time;}}
const context=vm.createContext({window,document,fetch:window.fetch,Date:ClockDate,console});
vm.runInContext(fs.readFileSync(path.join(web,'behavior_widget.js'),'utf8'),context);
vm.runInContext(script,context);
async function flush(){for(let i=0;i<16;i++)await Promise.resolve();}
async function advance(ms){const end=time+ms;for(let i=0;i<100;i++){
    const next=[...timers.entries()].filter(([,value])=>value.due<=end).sort((a,b)=>a[1].due-b[1].due)[0];
    if(!next)break;time=next[1].due;timers.delete(next[0]);next[1].fn();await flush();}time=end;await flush();}
(async()=>{
    await flush();await advance(0);
    assert.ok(requests.every(r=>!r.options.method),'opening camera page does not start/configure recognition');
    assert.ok(get('camera-feed').src.includes('/api/stream'));
    assert.ok(get('thermal-feed').src.includes('/api/thermal/stream.mjpg'));
    const thermalSrc=get('thermal-feed').src;
    get('btn-yolo').emit('click');await flush();await advance(100);
    assert.ok(get('camera-log').textContent.includes('识别启动中'));
    assert.ok(!get('camera-log').textContent.includes('已就绪'));
    mode='ready';for(const interval of intervals.values())interval.fn();await flush();
    assert.ok(get('camera-log').textContent.includes('已就绪'));
    get('btn-behavior-view').emit('click');await flush();await advance(1500);
    assert.ok(get('camera-feed').src.startsWith('blob:'),'behavior selection shows the exact annotated inference image');
    const selectedAt=images.length;
    await advance(1500);
    assert.ok(!images.slice(selectedAt).some(image=>image.id==='camera-feed'&&image.src.includes('stream')),
        'ordinary MJPEG never restarts while behavior view is selected');
    assert.strictEqual(get('thermal-feed').src,thermalSrc,'selecting behavior does not reconnect thermal');
    mode='failed';for(const interval of intervals.values())interval.fn();await flush();
    assert.ok(get('camera-log').textContent.trim().split('\n').at(-1).includes('部分不可用'),
        'later YOLO subprocess exit replaces the all-ready message');
    assert.ok(get('camera-feed').src.startsWith('blob:'),'independent behavior preview survives visual failure');
    get('btn-yolo').emit('click');await flush();await advance(100);
    assert.strictEqual(get('camera-feed').src,'','stop recognition immediately clears behavior overlay');
    get('btn-behavior-view').emit('click');await advance(0);
    assert.ok(get('camera-feed').src.includes('/api/stream'),'returning to ordinary view restores raw stream');
    window.emit('pagehide');await flush();
    assert.strictEqual(get('camera-feed').src,'');assert.strictEqual(get('thermal-feed').src,'');
    assert.strictEqual(intervals.size,0);assert.ok(released);
    const count=requests.length;await advance(10000);assert.strictEqual(requests.length,count,'closed page creates no further work');
    assert.ok(requests.filter(r=>r.options.method==='POST').every(r=>r.url.endsWith('/api/detection')),
        'page loading/view switches never mutate configuration or publish robot motion');
    console.log('camera behavior integration: initial GET-only, startup/readiness/failure, preview switch, thermal isolation and stop/pagehide: OK');
})().catch(error=>{console.error(error);process.exitCode=1;});
