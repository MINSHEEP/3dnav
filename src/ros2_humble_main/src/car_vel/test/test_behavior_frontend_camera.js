#!/usr/bin/env node
'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const web = path.resolve(__dirname, '../car_vel/web_frontend');
const html = fs.readFileSync(path.join(web, 'camera.html'), 'utf8');
const script = html.slice(html.lastIndexOf('<script>') + 8, html.lastIndexOf('</script>'));
const requests = [], timers = new Map(), intervals = new Map(), opened = [];
let timerId = 0, now = 10000, released = false, hangYolo = false, yolo = false, behaviorRunning = false;
let holdStatus = false, releaseStatus;
let enabled = {fall:true, fight:false, smoking:true}, epoch = 1;
class Node {
    constructor(id) {
        this.id = id; this.handlers = new Map(); this.children = new Map(); this.textContent = ''; this.hidden = false;
        this.attributes = new Map(); this.disabled = false;
        this.classList = {add(){}, remove(){}, contains(){return false;}};
    }
    addEventListener(name, fn) { if (!this.handlers.has(name)) this.handlers.set(name,new Set()); this.handlers.get(name).add(fn); }
    removeEventListener(name, fn) { this.handlers.get(name)?.delete(fn); }
    emit(name, data={}) { for (const fn of this.handlers.get(name)||[]) fn(data); }
    set src(value) { this.attributes.set('src',value); }
    get src() { return this.attributes.get('src') || ''; }
    removeAttribute(key) { this.attributes.delete(key); }
    getAttribute(key) { return this.attributes.get(key) ?? null; }
    querySelector(key) { if (!this.children.has(key)) this.children.set(key,new Node(key)); return this.children.get(key); }
    set innerHTML(value) { this.markup = value; this.children.clear(); }
}
const document = new Node('document'), window = new Node('window');
const elements = new Map([...html.matchAll(/\bid="([^"]+)"/g)].map(match => [match[1], new Node(match[1])]));
document.hidden = false; document.activeElement = null;
document.querySelector = selector => elements.get(selector.slice(1));
const get = id => elements.get(id), saved = new Map();
const behavior = () => ({running:behaviorRunning, ready:behaviorRunning, fresh:behaviorRunning, error:null,
    source_epoch:epoch, stamp_ns:1000000001, frame_stamp_ns:'1000000001', frame_age_seconds:.01, max_result_age_seconds:2,
    enabled:{...enabled}, branches:Object.fromEntries(Object.keys(enabled).map(key => [key,
        {enabled:enabled[key], state:enabled[key]?'valid':'disabled', ready:enabled[key], label:'other'}]))});
const detection = () => ({success:true, running:yolo||behaviorRunning, ready:yolo||behaviorRunning,
    yolo:{running:yolo,ready:yolo,suspended_by_behavior:behaviorRunning,error:null},
    thermal:{ready:true,manual_detection_enabled:false,detection_enabled:false},behavior:behavior()});
Object.assign(window, {document,location:{protocol:'http:',hostname:'robot',reload(){}}, AbortController,
    sessionStorage:{getItem:k=>saved.get(k)||null,setItem:(k,v)=>saved.set(k,v)},
    URL:{createObjectURL:()=>`blob:frame-${now}`,revokeObjectURL(){}},
    setTimeout(fn,delay){const id=++timerId;timers.set(id,{fn,due:now+delay,delay});return id;},
    clearTimeout(id){timers.delete(id);},
    setInterval(fn,delay){const id=++timerId;intervals.set(id,{fn,delay});return id;},
    clearInterval(id){intervals.delete(id);},
    open(url){opened.push(url);},
    fetch:async(url,options={})=>{
        requests.push({url,options}); const u = new URL(url); let body;
        if (u.pathname === '/api/behavior/status') body=behavior();
        else if (u.pathname === '/api/behavior/frame.jpg') return {ok:true,blob:async()=>({}),headers:{get:key=>key==='X-Behavior-Source-Epoch'?String(epoch):'1000000001'}};
        else if (u.pathname === '/api/detection' && !options.method) body=detection();
        else if (u.pathname === '/api/detection') {
            const op=JSON.parse(options.body).operation;
            if (op === 'toggle' && (yolo||behaviorRunning)) {yolo=false;behaviorRunning=false;}
            else if (Object.values(enabled).some(Boolean)) {yolo=false;behaviorRunning=true;}
            else {yolo=true;behaviorRunning=false;}
            epoch++; body=detection();
        } else if (u.pathname === '/api/yolo') {
            if (hangYolo) return new Promise((resolve,reject)=>options.signal.addEventListener('abort',()=>reject(Object.assign(new Error('aborted'),{name:'AbortError'}))));
            enabled={fall:false,fight:false,smoking:false};behaviorRunning=false;yolo=true;epoch++;body={running:true,ready:true};
        } else if (u.pathname === '/api/behavior/config') {
            const v=JSON.parse(options.body);enabled={fall:v.fall_enabled,fight:v.fight_enabled,smoking:v.smoking_enabled};
            if (yolo && Object.values(enabled).some(Boolean)) {yolo=false;behaviorRunning=true;}
            epoch++;body=behavior();
        } else if (u.pathname === '/api/video') body={success:true};
        else if (u.pathname === '/api/status') {
            body={...detection(),detection:detection(),connected:true,
                camera_source:{generation:1,label:'C12'},video:{device:'rtsp://camera'}};
            if (holdStatus) {
                holdStatus = false;
                return new Promise(resolve => { releaseStatus = () => resolve({ok:true,json:async()=>body}); });
            }
        }
        else throw new Error(`unexpected request ${url}`);
        return {ok:true,json:async()=>body};
    },RobotUi:{previewUrl:url=>url,previewAllowed:()=>!document.hidden,previewControls(){},
        temperatureText:()=> '-- ℃',setBatteryDisconnected(){},subscribeDogStatus(){},RosSession:class{connect(){}stop(){released=true;}}}
});
class ClockDate extends Date {static now(){return now;}}
const context=vm.createContext({window,document,fetch:window.fetch,Date:ClockDate,console});
vm.runInContext(fs.readFileSync(path.join(web,'behavior_widget.js'),'utf8'),context);
vm.runInContext(script,context);
async function flush(){for(let i=0;i<40;i++)await Promise.resolve();}
async function advance(ms){const end=now+ms;for(let i=0;i<500;i++){
    const next=[...timers.entries()].filter(([,v])=>v.due<=end).sort((a,b)=>a[1].due-b[1].due)[0];
    if(!next)break;now=next[1].due;timers.delete(next[0]);next[1].fn();await flush();}now=end;await flush();}
async function click(id){get(id).emit('click');await flush();await advance(0);await flush();}
(async()=>{
    await flush();await advance(0);
    assert.ok(requests.every(r=>!r.options.method),'opening page is read-only');
    assert.ok(get('camera-feed').src.includes('/api/stream'));
    const thermalSrc=get('thermal-feed').src;
    await click('btn-yolo');
    assert.equal(behaviorRunning,true);assert.equal(yolo,false);
    assert.equal(get('btn-yolo').textContent,'停止识别');
    await click('btn-behavior-view');await advance(200);
    assert.equal(get('feed-badge').textContent,'行为识别 · 对应推理帧');
    assert.ok(get('camera-feed').src.startsWith('blob:'),'behavior view loads an inference frame');
    await click('btn-frame');assert.ok(opened.at(-1).includes('/api/behavior/frame.jpg'));
    holdStatus=true;for(const interval of intervals.values()) interval.fn();await flush();
    assert.ok(releaseStatus,'old status request is pending');
    await click('btn-view');
    assert.equal(yolo,true);assert.equal(behaviorRunning,false);
    assert.ok(get('camera-feed').src.includes('/api/yolo/stream.mjpg'));
    releaseStatus();await flush();
    assert.equal(get('feed-badge').textContent,'YOLO 标注画面','late status cannot undo the new view');
    await click('btn-frame');assert.ok(opened.at(-1).includes('/api/yolo/frame.jpg'));
    await click('btn-behavior-view');await advance(200);
    assert.equal(behaviorRunning,true);assert.equal(yolo,false);
    assert.deepEqual(enabled,{fall:true,fight:false,smoking:true},'switching back restores selected branches');
    assert.equal(get('thermal-feed').src,thermalSrc,'visual switching does not reconnect thermal');
    await click('btn-yolo');assert.equal(behaviorRunning,false);
    await click('btn-yolo');assert.equal(behaviorRunning,true,'start after stop restores selected mode');
    hangYolo=true;get('btn-view').emit('click');await flush();
    assert.equal(get('btn-view').disabled,true);
    const timeout=[...timers.entries()].find(([,v])=>v.delay===30000);
    assert.ok(timeout,'control request has a deadline');timers.delete(timeout[0]);timeout[1].fn();await flush();
    assert.equal(get('btn-view').disabled,false,'timeout releases controls');
    assert.ok(get('camera-log').textContent.includes('超时'));
    assert.equal(behaviorRunning,true);hangYolo=false;
    await click('btn-behavior-view');
    assert.equal(get('feed-badge').textContent,'原始画面');
    await click('btn-reconnect');
    assert.ok(requests.some(r=>r.url.endsWith('/api/video') && r.options.method==='POST'));
    await click('btn-frame');assert.ok(opened.at(-1).includes('/api/snapshot'));
    saved.clear();enabled={fall:false,fight:false,smoking:false};behaviorRunning=false;yolo=true;
    await click('btn-behavior-view');
    assert.ok(get('camera-log').textContent.includes('请先勾选'));
    assert.equal(behaviorRunning,false,'view switch must not enable unchecked behavior categories');
    assert.equal(yolo,true);
    window.emit('pagehide');await flush();assert.ok(released);
    console.log('camera page: start/stop, behavior-YOLO switching, choices, snapshots, reconnect and timeout OK');
})().catch(error=>{console.error(error);process.exitCode=1;});
