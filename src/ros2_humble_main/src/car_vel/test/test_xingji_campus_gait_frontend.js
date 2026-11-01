'use strict';
// Execute the production campus async start chain with fake ROS only.
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const html = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/campus_navigation.html'), 'utf8');
const code = ['ensureCampusXingjiGait', 'executeCampusPlan'].map(name =>
    html.match(new RegExp(`async function ${name}\\(\\) \\{[^]*?\\n\\}`))[0]).join('\n');

function fixture() {
    const calls = [], logs = [];
    const context = {
        campusBodyStatus: { active_body: 'xingji', generation: 1,
            connected: true, config: { selected: 'xingji' } },
        campusBodyStatusAt: 10000, Date: { now: () => 10000 },
        activePlan: {}, campusStartPending: false,
        UI: { TYPES: { trigger: 'trigger', startCampusMission: 'start' } },
        document: { getElementById: id => ({ value: id }) },
        availableModes: () => ['obstacle'], updatePlanningControls() {},
        log: value => logs.push(value),
    };
    context.session = { connected: true, call: async name => {
        calls.push(name);
        return { success: true, message: 'ok' };
    } };
    vm.createContext(context);
    vm.runInContext(code, context);
    return { context, calls, logs };
}

(async () => {
    let f = fixture();
    await f.context.executeCampusPlan();
    assert.deepStrictEqual(f.calls, ['/dog_remote/ensure_standing', '/start_campus_mission']);
    assert.strictEqual(f.context.campusStartPending, false);

    f = fixture();
    f.context.session.call = async name => { f.calls.push(name); return { success: false }; };
    await f.context.executeCampusPlan();
    assert.deepStrictEqual(f.calls, ['/dog_remote/ensure_standing']);

    f = fixture();
    f.context.session.call = async name => { f.calls.push(name); throw new Error('timeout'); };
    await f.context.executeCampusPlan();
    assert.deepStrictEqual(f.calls, ['/dog_remote/ensure_standing']);

    f = fixture();
    f.context.campusBodyStatus.active_body = 'gangben_zsl1';
    await f.context.executeCampusPlan();
    assert.deepStrictEqual(f.calls, ['/start_campus_mission']);

    f = fixture();
    f.context.campusBodyStatusAt = 0;
    await f.context.executeCampusPlan();
    assert.deepStrictEqual(f.calls, []);

    for (const mutation of [
        c => { c.campusBodyStatus = { ...c.campusBodyStatus, generation: 2 }; },
        c => { c.session.connected = false; },
        c => { c.campusBodyStatus = { ...c.campusBodyStatus, error: 'fault' }; },
        c => { c.activePlan = {}; },
    ]) {
        f = fixture();
        f.context.session.call = async name => {
            f.calls.push(name); mutation(f.context); return { success: true };
        };
        await f.context.executeCampusPlan();
        assert.deepStrictEqual(f.calls, ['/dog_remote/ensure_standing']);
    }

    f = fixture();
    await Promise.all([f.context.executeCampusPlan(), f.context.executeCampusPlan()]);
    assert.deepStrictEqual(f.calls, ['/dog_remote/ensure_standing', '/start_campus_mission']);
    console.log('Xingji campus confirmed gait / failures / async switch / Gangben isolation: 10 cases OK');
})().catch(error => { console.error(error); process.exitCode = 1; });
