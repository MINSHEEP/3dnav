const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const values = new Map();
const localStorage = {getItem:k => values.get(k) ?? null, setItem:(k,v) => values.set(k,v)};
let onChange;
const setting = {checked:false, addEventListener:(_,fn) => {onChange=fn;}};
const window = {addEventListener:()=>{}};
vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/recording_visibility.js'), 'utf8'),
    {window,localStorage,document:{getElementById:()=>setting}});
assert.equal(window.RecordingVisibility.get(), false);
setting.checked = true; onChange();
assert.equal(window.RecordingVisibility.get(), true);
setting.checked = false; onChange();
assert.equal(window.RecordingVisibility.get(), false);
console.log('录像显示设置默认关闭，可保存和关闭');
