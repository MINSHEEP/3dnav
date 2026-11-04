const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function fixture() {
    const elements = {};
    const ids = ['select','state','apply','ip','local-ip','port','max-vx','max-vy',
        'max-yaw','navigation-verified','sdk-options','port-label','note','form'];
    for (const id of ids) elements['robot-body-'+id] = {
        value: id === 'select' ? 'xingji' : '', checked:false, disabled:false,
        type: id === 'navigation-verified' ? 'checkbox'
            : ['port','max-vx','max-vy','max-yaw'].includes(id) ? 'number' : 'text',
        events:{}, textContent:'', hidden:false,
        addEventListener(event, callback) { this.events[event] = callback; },
        reportValidity() { return true; },
    };
    const window = {RobotUi:{TYPES:{string:'std_msgs/msg/String'}},
        confirm: () => true, setInterval: callback => { window.tick = callback; }};
    let now = 100000, receiver;
    const calls = [], logs = [];
    const session = {subscribe: (topic, type, callback) => { receiver = callback; },
        call: async (...args) => { calls.push(args); return {success:true,message:'ok'}; }};
    const context = {window,document:{getElementById:id=>elements[id]},
        Date:{now:()=>now}, console};
    vm.runInNewContext(fs.readFileSync(path.join(__dirname,
        '../car_vel/web_frontend/robot_body_settings.js'),'utf8'), context);
    const app = window.RobotBodySettings.create(value => logs.push(value));
    app.attach(session);
    const profile = {robot_ip:'192.168.96.2',local_ip:'',port:8080,
        max_vx:0.5,max_vy:0.2,max_yaw:0.6,navigation_verified:true};
    const payload = {config:{version:1,selected:'xingji',profiles:{xingji:profile,
        gangben_zsl1:{...profile,robot_ip:'192.168.168.168',port:43988,navigation_verified:false}}},
        connected:true,active_body:'xingji',names:{xingji:'星玑',gangben_zsl1:'钢镚'},locked:false};
    const emit = value => receiver({data:JSON.stringify(value || payload)});
    const get = id => elements['robot-body-'+id];
    return {app,get,payload,emit,calls,logs,window,session,advance:ms=>{now+=ms;window.tick();}};
}

(async () => {
    let f=fixture();
    f.app.connection(true);
    assert.equal(f.get('apply').disabled,true,'no backend status must remain disabled');
    f.emit();
    assert.equal(f.get('apply').disabled,false);
    assert.equal(f.get('ip').value,'192.168.96.2');
    f.advance(3100);
    assert.equal(f.get('apply').disabled,true,'stale heartbeat disables apply');
    f.emit(); f.app.connection(false);
    assert.equal(f.get('apply').disabled,true);

    f=fixture(); f.app.connection(true); f.emit();
    f.payload.locked=true; f.emit();
    assert.equal(f.get('apply').disabled,false,'confirmed switch stops active tasks');
    f.payload.locked=false; f.emit();
    f.get('select').value='gangben_zsl1'; f.get('select').events.change();
    assert.equal(f.get('sdk-options').hidden,false);
    assert.equal(f.get('navigation-verified').checked,false);
    f.get('local-ip').value='192.168.168.15'; f.get('local-ip').events.input();
    f.emit();
    assert.equal(f.get('local-ip').value,'192.168.168.15','heartbeat preserves user edits');
    await f.get('apply').events.click();
    assert.equal(f.calls.length,1);
    assert.equal(f.calls[0][0],'/robot_body/select');
    assert.equal(f.calls[0][2].body_id,'gangben_zsl1');
    assert.equal(f.calls[0][2].expected_version,1);
    const saved=JSON.parse(f.calls[0][2].profile_json);
    assert.equal(saved.local_ip,'192.168.168.15');
    assert.equal(saved.max_vx,0.5);
    assert.equal(saved.navigation_verified,false);

    f=fixture(); f.app.connection(true); f.payload.connected=false; f.emit();
    f.get('select').value='gangben_zsl1'; f.get('select').events.change();
    const prompts=[]; f.window.confirm=text=>{prompts.push(text); return true;};
    await f.get('apply').events.click();
    assert.equal(prompts.length,1);
    assert.equal(JSON.parse(f.calls[0][2].profile_json).switch_confirmed,true);

    f=fixture(); f.app.connection(true); f.payload.connected=false; f.emit();
    f.get('select').value='gangben_zsl1'; f.get('select').events.change();
    f.window.confirm=()=>false;
    await f.get('apply').events.click(); assert.equal(f.calls.length,0);

    f=fixture(); f.app.connection(true); f.emit();
    f.get('ip').events.input();
    f.payload.config.version=2; f.emit();
    await f.get('apply').events.click();
    assert.equal(f.calls[0][2].expected_version,1,'dirty form cannot silently overwrite another tab');

    f=fixture(); f.app.connection(true); f.emit(); f.window.confirm=()=>false;
    await f.get('apply').events.click(); assert.equal(f.calls.length,0);

    f=fixture(); f.app.connection(true); f.emit();
    f.session.call=async()=>{throw new Error('timeout');};
    await f.get('apply').events.click();
    assert.ok(f.logs.some(text=>text.includes('未确认切换结果')));
    console.log('robot body settings: connection, stale, locks, profile, CAS, cancel, timeout PASS');
})().catch(error=>{console.error(error);process.exit(1);});
