const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function fixture() {
    const elements = {};
    const ids = ['mode','device','port','baudrate','address','confirm','refresh','test','save',
        'connection-state','connection-form','device-detail','result'];
    for (const id of ids) elements['m701-'+id] = {
        value:'', checked:false, disabled:false, textContent:'', events:{}, children:[],
        addEventListener(event, callback) { this.events[event] = callback; },
        replaceChildren() { this.children=[]; this.value=''; },
        appendChild(child) { this.children.push(child); },
        reportValidity() { return true; },
    };
    let now=10000, receiver;
    const calls=[];
    const window={RobotUi:{TYPES:{string:'std_msgs/msg/String'}}, setInterval(fn){this.tick=fn;}};
    const document={getElementById:id=>elements[id], createElement:()=>({})};
    vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../car_vel/web_frontend/m701_settings.js'),'utf8'),
        {window,document,Date:{now:()=>now}});
    const session={subscribe(topic,type,fn){receiver=fn;},
        async call(...args){calls.push(args);return {success:true,message:'ok'};}};
    const app=window.M701Settings.create(()=>{}); app.attach(session); app.connection(true);
    const binding={vid:'1a86',pid:'7523',serial:'unique',interface:'00',channel:'0'};
    const item={...binding,port:'/dev/ttyUSB0',aliases:['/dev/serial/by-id/unique'],
        location:'usb-port1',driver:'ch341',product:'CH340',identity:{...binding,location:'usb-port1'}};
    const status={config:{version:3,mode:'auto',port:item.port,baudrate:9600,address:1,binding},
        devices:[item],error:{code:'ok',message:'数据正常'},active_port:item.port};
    const emit=()=>receiver({data:JSON.stringify(status)});
    const get=id=>elements['m701-'+id];
    const confirm=()=>{get('confirm').checked=true;get('confirm').events.change();};
    return {app,get,status,item,emit,confirm,calls,session,advance(ms){now+=ms;window.tick();}};
}

(async()=>{
    let f=fixture();
    assert(f.get('test').disabled); f.emit();
    assert(f.get('test').disabled, 'explicit confirmation required');
    f.confirm(); assert(!f.get('test').disabled);
    await f.get('test').events.click();
    assert.equal(f.calls[0][0], '/air_quality/m701/configure');
    assert.equal(f.calls[0][2].expected_version,3);
    assert.equal(JSON.parse(f.calls[0][2].config_json).identity.location,'usb-port1');
    f.advance(6100); assert(f.get('save').disabled, 'stale status');
    f.emit(); f.app.connection(false); assert(f.get('save').disabled);

    f=fixture();f.emit();f.confirm();
    f.get('address').value='2'; f.get('address').events.input();
    f.emit(); assert.equal(f.get('address').value,'2','preserve dirty form');
    assert(!f.get('confirm').checked,'parameter edit clears confirmation');
    f.confirm(); f.item.identity.serial='replacement'; f.item.serial='replacement'; f.emit();
    assert(f.get('test').disabled,'reused tty with other identity must not authorize');

    f=fixture();f.emit();f.confirm();
    f.status.devices=[];f.emit();assert(f.get('save').disabled,'unplugged');
    await f.get('refresh').events.click();
    assert.equal(f.calls[0][2].operation,'list');

    f=fixture();
    f.status.config={...f.status.config, mode:'manual', binding:null,
        port:'/dev/serial/by-id/old-missing-device'};
    f.item.port='/dev/ttyCH341USB0'; f.item.aliases=[];
    f.status.active_port='';
    f.status.diagnostic={code:'selection_required', message:'旧串口不存在；请确认候选设备',
        suggested_port:f.item.port, confirmation_required:true};
    f.emit();
    assert.equal(f.get('device').value, '/dev/ttyCH341USB0', 'offer unique replacement candidate');
    assert.equal(f.get('mode').value, 'auto', 'recommend identity-based reconnect for repaired path');
    assert(!f.get('confirm').checked && f.get('test').disabled && f.get('save').disabled);
    assert.equal(f.calls.length,0,'recommendation does not open or probe any port');
    assert(f.get('connection-state').textContent.includes('旧串口不存在'));
    f.confirm(); assert(!f.get('test').disabled);
    f.item.serial='replacement'; f.item.identity={...f.item.identity,serial:'replacement'};
    f.emit(); assert(!f.get('confirm').checked && f.get('test').disabled,
        'changed recommended identity must revoke prior confirmation');

    f=fixture(); f.status.devices=[]; f.status.active_port='';
    f.status.diagnostic={code:'driver_missing', message:'USB 1a86:7523 已识别但没有生成串口',
        suggested_port:'',confirmation_required:false};
    f.emit(); assert(f.get('connection-state').textContent.includes('没有生成串口'));
    assert(f.get('test').disabled && f.get('save').disabled);

    f=fixture(); f.status.config={...f.status.config,mode:'manual',port:'/dev/ttyUSB9'};
    f.status.active_port='';
    f.status.diagnostic={code:'path_changed',message:'同一设备的固定路径已变化',
        suggested_port:f.item.port,confirmation_required:true};
    f.emit();
    assert.equal(f.get('mode').value,'auto','manual alias recovery must actually switch form mode');
    assert.equal(f.get('port').value,f.item.port);
    assert(f.get('test').disabled,'same-identity recovery still requires confirmation');

    f=fixture();f.emit();f.confirm();
    let finish;
    f.session.call=()=>new Promise(resolve=>{finish=resolve;});
    const pending=f.get('save').events.click();
    assert(f.get('test').disabled && f.get('save').disabled,'one operation at a time');
    finish({success:true,message:'saved',status_json:JSON.stringify({...f.status,
        config:{...f.status.config,version:4}})});
    await pending;
    assert(!f.get('confirm').checked);
    assert.equal(f.get('result').textContent,'saved');
    console.log('M701 settings: confirmation, identity, stale, dirty, unplug, CAS and busy PASS');
})().catch(error=>{console.error(error);process.exitCode=1;});
