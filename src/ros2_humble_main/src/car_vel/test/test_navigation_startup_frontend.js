const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const page = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/navigation.html'), 'utf8');
for (const match of page.matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g)) {
    if (match[1].trim()) new vm.Script(match[1]);
}
const start = page.indexOf('    async function setNavigationMode(');
const end = page.indexOf('    async function deleteSelectedMap()', start);
assert(start > 0 && end > start);
function harness() {
    const pending = [], modes = [], logs = [];
    const context = {navigationConnectionEpoch: 1, navigationStartRequest: null,
        nav2Ready: false, UI: {TYPES: {}},
        session: {connected: true, call: (...args) => new Promise((resolve, reject) => pending.push({resolve, reject, args}))},
        setMode: m => modes.push(m), refreshModeFromLocalization: () => modes.push('ready'),
        log: m => logs.push(m), renderNavigationStartup() {}, updateMapReadyUi() {},
        document: {getElementById() { throw new Error('stale map response touched DOM'); }}};
    vm.createContext(context);
    vm.runInContext(page.slice(start, end), context);
    return {context, pending, modes, logs};
}
(async () => {
    let h = harness();
    let first = h.context.setNavigationMode();
    let second = h.context.setNavigationMode();
    assert.equal(h.pending.length, 1, 'coalesce duplicate start on same connection');
    h.context.nav2Ready = true;
    h.pending[0].resolve({success: true});
    await Promise.all([first, second]);
    assert.equal(h.modes.at(-1), 'ready', 'late launch result must not revert readiness');

    h = harness();
    first = h.context.setNavigationMode();
    h.context.navigationConnectionEpoch++;
    h.context.navigationStartRequest = null;
    second = h.context.setNavigationMode();
    h.pending[0].reject(new Error('old connection failed'));
    await first;
    assert.equal(h.logs.length, 0);
    assert(h.context.navigationStartRequest, 'old finally must not erase new request');
    h.pending[1].resolve({success: true});
    await second;
    assert.equal(h.logs.length, 1);

    h = harness();
    first = h.context.refreshMaps();
    h.context.navigationConnectionEpoch++;
    h.pending[0].resolve({success:true, map_names:['old-map']});
    await first; // DOM stub throws if a stale response is applied.
    assert.equal(h.logs.length, 0);
    console.log('PASS: deduplicated start, late-ready, stale failure/finally and stale map response');
})().catch(error => { console.error(error); process.exitCode = 1; });
