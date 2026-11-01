'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const frontend = path.resolve(__dirname, '../car_vel/web_frontend');
let now = 1000, timerId = 0;
const timers = new Map(), callbacks = [], events = [];
class Element {
    constructor() { this.hidden = false; this.textContent = ''; this.children = new Map(); this.dataset = {}; this.classList = {add(){},toggle(){}}; }
    set innerHTML(value) { this.markup = value; this.children.clear(); }
    querySelector(key) { if (!this.children.has(key)) this.children.set(key, new Element()); return this.children.get(key); }
}
const root = {setInterval(fn){timers.set(++timerId,fn);return timerId;},clearInterval(id){timers.delete(id);}};
vm.runInNewContext(fs.readFileSync(path.join(frontend,'velocity_guard_widget.js'),'utf8'), {window:root, Date:{now:()=>now}, WeakMap});
const session = {subscribe(topic,type,callback){assert.equal(topic,'/navigation/velocity_guard');callbacks.push(callback);}};
const reason = new Element(), motion = new Element();
root.VelocityGuardWidget.attach(session, reason, 'std_msgs/msg/String', {motionElement:motion,onEvent:text=>events.push(text)});
const send = (changes={}) => callbacks.at(-1)({data:JSON.stringify({schema_version:2,body_id:'xingji',navigation_mode:'terrain_3d',
    terrain_guard_enabled:true,navigation_active:true,source:'nav',blocked:false,warning:false,input_fresh:true,
    input_age_sec:.03,request_velocity:[.25,0,.1],commanded_velocity:[.25,0,.1],speed_limit_mps:.25,
    speed_policy:'nav2_body_limits',speed_limit_source:'nav2_body_limits',terrain_type:'curb',reason_code:'forwarded',...changes})});
const text = key => motion.querySelector(`[data-speed-${key}]`).textContent;
send();
assert.equal(motion.hidden,false,'confirmed Xingji 3D speed snapshot must be visible');
assert.equal(text('request'),'0.25 m/s');
assert.equal(text('command'),'0.25 m/s');
assert.equal(text('limit'),'0.25 m/s');
assert.match(text('policy'),/不按可通行地形额外减速/);
assert.match(text('state'),/下发/);
assert.match(text('terrain'),/低台阶/);
assert.match(motion.markup,/下发指令不等于本体实测速度/);
send({input_fresh:false,input_age_sec:.56,request_velocity:null,source:'zero',commanded_velocity:[0,0,0],reason_code:'upstream_stale',
    stream_reason:'原始控制器没有新速度',controller_stream:{seen:true,age_sec:.56,receive_rate_hz:2,request_velocity:[.25,0,0]}});
assert.equal(text('request'),'--');
assert.equal(text('command'),'0.00 m/s');
assert.match(reason.textContent,/速度消息.*过期/,'upstream loss must be clear even if old blocked field is false');
assert.match(text('state'),/零速停车/);
assert.match(reason.textContent,/原始控制器没有新速度/);
assert.match(text('stream'),/样本已过期/);
assert(!text('stream').includes('0.25'),'expired raw controller requests cannot look current');
send({input_fresh:false,input_age_sec:.9,request_velocity:null,source:'zero',commanded_velocity:[0,0,0],reason_code:'upstream_stale'});
assert.equal(events.filter(t=>t.includes('速度消息')).length,1,'repeated ages must not flood operation log');
send({speed_limit_source:'restricted_map',speed_limit_mps:.1});
assert.match(text('policy'),/受限场地/);
assert.equal(text('limit'),'0.10 m/s');
send({blocked:true,reason:'检测到坑洞',reason_code:'terrain_unsafe',speed_limit_source:'safety_stop',speed_limit_mps:0,commanded_velocity:[0,0,0]});
assert.equal(reason.textContent,'检测到坑洞');
assert.equal(text('command'),'0.00 m/s');
send({blocked:true,reason:'导航速度过期；原始控制器断流',reason_code:'upstream_stale',stream_reason:'原始控制器断流',commanded_velocity:[0,0,0]});
assert.equal(reason.textContent,'导航速度过期；原始控制器断流','raw stream explanation must not be appended twice');
send();
now += 2100; for (const timer of timers.values()) timer();
for (const key of ['request','command','limit']) assert.equal(text(key),'--','stale snapshots must clear every number');
assert.match(text('state'),/过期/);
assert.equal(text('terrain'),'实时地形 --');
send({request_velocity:[NaN,0,0],speed_limit_mps:null});
assert.equal(text('request'),'--','invalid or null velocity is unknown, not zero');
assert.equal(text('limit'),'--');
send({navigation_mode:'map_2d',terrain_guard_enabled:false,speed_policy:'suspended_2d'});
assert.equal(motion.hidden,true,'2D must keep its existing display behavior');
send({body_id:'gangben_zsl1',speed_policy:'terrain_adaptive'});
assert.equal(motion.hidden,true,'Gangben must keep its existing display behavior');
send(); callbacks.at(-1)({data:'broken'});
for (const key of ['request','command','limit']) assert.equal(text(key),'--','malformed telemetry cannot retain numeric commands');
for (const page of ['navigation.html','campus_navigation.html','inspection.html']) {
    const html=fs.readFileSync(path.join(frontend,page),'utf8');
    assert(html.includes('id="navigation-motion-status"'),`${page} needs a motion snapshot container`);
    assert(html.includes('motionElement:'),`${page} must attach the read-only snapshot`);
}
console.log('navigation speed status: request/command/cap, reason, expiry, scope and no motion side effects OK');
