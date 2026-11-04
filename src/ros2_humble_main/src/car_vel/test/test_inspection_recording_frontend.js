const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
let callback, resolveOld, requests = [];
const button = {disabled:false, textContent:'', addEventListener:(_, fn) => { callback = fn; }};
const status = {textContent:''};
const controls = {hidden:true};
const result = data => ({ok:true, json:async () => data});
const fetch = (url, options = {}) => {
    if (options.method === 'POST') {
        requests.push(JSON.parse(options.body).operation);
        return Promise.resolve(result({state: requests.at(-1) === 'start' ? 'recording' : 'stopping'}));
    }
    return new Promise(resolve => { resolveOld = resolve; });
};
const context = {document:{getElementById: id => id === 'inspection-record-button' ? button : id === 'inspection-recording-controls' ? controls : status},
    window:{RecordingVisibility:{get:()=>false,key:'visibility'},addEventListener:()=>{}},
    location:{hostname:'robot'}, fetch, setInterval:fn=>{context.poll=fn;}};
vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/inspection_recording.js'), 'utf8'), context);
(async () => {
    await callback();
    resolveOld(result({state:'idle'}));
    await new Promise(resolve => setImmediate(resolve));
    assert.equal(button.textContent, '停止录像');
    assert.equal(controls.hidden, false, 'active recording keeps stop visible');
    await callback();
    assert.deepEqual(requests, ['start', 'stop']);
    assert.equal(controls.hidden, false, 'saving keeps controls visible');
    assert.equal(button.disabled, true, 'saving cannot start another recording');
    assert.equal(button.textContent, '正在保存录像…');
    const poll = context.poll();
    resolveOld(result({state:'idle'}));
    await poll;
    assert.equal(button.disabled, false);
    assert.equal(controls.hidden, true, 'idle recording follows hidden setting');
    console.log('录像状态过期响应不会覆盖停止按钮');
})().catch(error => {console.error(error);process.exitCode=1;});
