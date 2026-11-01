'use strict';
const fs = require('fs'), vm = require('vm'), path = require('path'), assert = require('assert');
const source = fs.readFileSync(path.join(__dirname,'../car_vel/web_frontend/robot_ui_common.js'),'utf8');
const ros = [], topics = [], events = new Map(), storage = new Map(), tab = new Map();
const store = map => ({getItem:key=>map.get(key)||null,setItem:(k,v)=>map.set(k,v),removeItem:k=>map.delete(k)});
class Ros {
    constructor(options) { this.options=options;this.handlers={};ros.push(this); }
    on(name,callback) { this.handlers[name]=callback; }
    close() { this.closed=true; this.handlers.close?.(); }
}
class Topic {
    constructor(options) { this.options=options;this.callbacks=new Set();topics.push(this); }
    subscribe(callback) { this.callbacks.add(callback); }
    unsubscribe(callback) { callback ? this.callbacks.delete(callback) : this.callbacks.clear(); }
    publish(message) { this.last=message; }
}
const document = { hidden:false,addEventListener:(name,cb)=>events.set(name,cb),
    removeEventListener:(name,cb)=>{if(events.get(name)===cb)events.delete(name);},
    querySelector:()=>null };
const window = {document,location:{href:'http://100.109.124.106:8080/camera.html',hostname:'100.109.124.106'},
    localStorage:store(storage),sessionStorage:store(tab),ROSLIB:{Ros,Topic,Message:class{constructor(v){Object.assign(this,v);}}},
    setTimeout:()=>1,clearTimeout:()=>{}};
const context={window,document,URL,ROSLIB:window.ROSLIB,Event:class{constructor(type){this.type=type;}},console};
vm.createContext(context);vm.runInContext(source,context);
const ui=window.RobotUi;
assert.strictEqual(ui.previewProfile(),'remote');
for(const [url,kind] of [['/api/stream','raw'],['/api/yolo/stream.mjpg','yolo'],['/api/thermal/stream.mjpg','thermal']]) {
    const u=new URL(ui.previewUrl('http://100.109.124.106:8766'+url+'?t=1'));
    assert.strictEqual(u.pathname,'/api/preview/stream');
    assert.strictEqual(u.searchParams.get('source'),kind);assert.strictEqual(u.searchParams.get('profile'),'remote');
}
assert.strictEqual(ui.previewUrl('/api/snapshot'),'/api/snapshot','explicit full-resolution still is unchanged');
assert.strictEqual(ui.previewAllowed('ordinary'),true);assert.strictEqual(ui.previewAllowed('thermal'),false);
tab.set('robot.webPreview.streams','both');assert.strictEqual(ui.previewAllowed('thermal'),true);
document.hidden=true;assert.strictEqual(ui.previewAllowed('ordinary'),false);document.hidden=false;
storage.set('robot.webPreview.profile','lan');assert.strictEqual(ui.previewProfile(),'lan');
storage.set('robot.webPreview.profile','remote');
const session=new ui.RosSession({}); session.connect();ros[0].handlers.connection();
const callback=()=>{};
const cloud=session.subscribe('/global_localization_map',ui.TYPES.pointCloud2,callback,{queueLength:8});
assert.strictEqual(ros.length,2,'point cloud uses a separate socket');
ros[1].handlers.connection();
assert.strictEqual(topics[0].options.name,'/web_preview/remote/global_localization_map');
assert.strictEqual(topics[0].options.queueLength,1);
session.subscribe('/dog_status',ui.TYPES.string,()=>{});
assert.strictEqual(topics[1].options.ros,ros[0],'status stays on control connection');
session.publish('/cmd_vel',ui.TYPES.twist,{linear:{x:0}});
assert.strictEqual(topics[2].options.name,'/cmd_vel');assert.strictEqual(topics[2].options.ros,ros[0]);
document.hidden=true;events.get('visibilitychange')();
assert.strictEqual(ros[1].closed,true);assert.notStrictEqual(ros[0].closed,true);
document.hidden=false;events.get('visibilitychange')();assert.strictEqual(ros.length,3);
ros[2].handlers.connection();
session.setCloudPreviewVisible(false);assert.strictEqual(ros[2].closed,true);
assert.notStrictEqual(ros[0].closed,true,'hiding cloud never closes control');
const oldCount=ros.length;document.hidden=false;events.get('visibilitychange')();
assert.strictEqual(ros.length,oldCount,'showing tab does not reopen a hidden cloud view');
session.setCloudPreviewVisible(true);ros[3].handlers.connection();
cloud.unsubscribe();assert.strictEqual(ros[3].closed,true);
assert.strictEqual(session.cloudPreviews.size,0);
const another=session.subscribe('/global_mapping_cloud',ui.TYPES.pointCloud2,callback);
ros[4].handlers.connection();session.stop();assert.strictEqual(ros[4].closed,true);
assert.strictEqual(ros[0].closed,true);assert.strictEqual(session.cloudPreviews.size,0);
console.log('preview profile, 3 video paths, full local still, hidden/off, independent cloud sockets and cleanup: OK');
