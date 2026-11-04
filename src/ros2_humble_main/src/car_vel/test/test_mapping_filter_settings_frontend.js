const assert = require('assert'), fs = require('fs'), path = require('path'), vm = require('vm');
async function fixture() {
    const elements = {};
    for (const id of ['enabled','save','refresh','state','state-value','detail','result']) {
        elements['mapping-filter-'+id] = {checked:false,indeterminate:false,disabled:false,textContent:'',events:{},
            classList:{toggle(){}}, addEventListener(event, fn){this.events[event]=fn;}};
    }
    let now = 10000, receiver;
    const calls = [], state = {schema_version:1,mode:'protected',enabled:true,version:0,
        load_error:'',mapping_running:false,active_mode:null,pending:false,apply_policy:'next_mapping_start'};
    const window = {RobotUi:{TYPES:{string:'std_msgs/msg/String'}},
        setInterval(fn){this.tick=fn;return 1;},clearInterval(){this.cleared=true;}};
    vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../car_vel/web_frontend/mapping_filter_settings.js'),'utf8'),
        {window,document:{getElementById:id=>elements[id]},Date:{now:()=>now},JSON,Number});
    const session = {subscribe(topic,type,fn){assert.equal(topic,'/mapping/filter/status');receiver=fn;},
        async call(...args){calls.push(args);
            if(args[0]==='/mapping/filter/set_enabled') {
                state.enabled=args[2].data;state.mode=state.enabled?'protected':'bypass';state.version++;
                state.pending=state.mapping_running&&state.mode!==state.active_mode;
            }
            return {success:true,message:JSON.stringify(state)};
        }};
    const app = window.MappingFilterSettings.create(()=>{});
    app.connection(true);app.attach(session);await new Promise(resolve=>setImmediate(resolve));
    return {app,state,session,calls,window,get:id=>elements['mapping-filter-'+id],
        emit:()=>receiver({data:JSON.stringify(state)}),advance(){now+=7000;window.tick();},
        choose(value){elements['mapping-filter-enabled'].checked=value;elements['mapping-filter-enabled'].events.change();}};
}
(async()=>{
    const f=await fixture();
    assert(f.get('enabled').checked);assert(f.get('save').disabled);
    f.choose(false);assert(!f.get('save').disabled);
    await f.get('save').events.click();
    assert.equal(f.calls.find(item=>item[0]==='/mapping/filter/set_enabled')[1],'std_srvs/srv/SetBool');
    assert.strictEqual(f.calls.find(item=>item[0]==='/mapping/filter/set_enabled')[2].data,false);
    assert(f.get('save').disabled);assert(f.get('detail').textContent.includes('关闭'));
    assert(!f.calls.some(item=>item[0]==='/control_launch'));
    f.state.mapping_running=true;f.state.active_mode='bypass';f.emit();f.choose(true);
    await f.get('save').events.click();
    assert(f.get('state-value').textContent.includes('下次建图'));
    assert(f.get('detail').textContent.includes('当前建图：关闭'));
    f.choose(false);f.emit();assert.strictEqual(f.get('enabled').checked,false,'heartbeat must not overwrite unsaved input');
    f.advance();assert(f.get('save').disabled);assert(f.get('enabled').disabled);
    f.emit();f.app.connection(false);assert(f.get('save').disabled);assert(f.get('enabled').indeterminate);

    const g=await fixture();g.state.mode='strict';g.emit();assert(g.get('enabled').checked);
    assert(g.get('detail').textContent.includes('严格'));
    g.state.load_error='corrupt file';g.emit();assert(g.get('enabled').disabled);assert(g.get('state-value').textContent.includes('损坏'));

    const h=await fixture();h.choose(false);
    h.session.call=async()=>{throw Error('timeout');};await h.get('save').events.click();
    assert(h.get('result').textContent.includes('未确认'));assert(!h.get('enabled').disabled);
    let finish;h.session.call=()=>new Promise(resolve=>{finish=resolve;});
    const pending=h.get('save').events.click();assert(h.get('enabled').disabled);
    h.app.connection(false);finish({success:true,message:JSON.stringify({...h.state,mode:'bypass',enabled:false,version:1})});
    await pending;assert(h.get('save').disabled);assert(h.get('enabled').indeterminate,'late response cannot revive disconnected controls');
    h.app.stop();assert(h.window.cleared);
    console.log('PASS: mapping filter toggle, SetBool, actual/pending mode, dirty input, stale/offline, corrupt, timeout and late response');
})().catch(error=>{console.error(error);process.exitCode=1;});
