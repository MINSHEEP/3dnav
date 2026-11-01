'use strict';
// Actual page functions, fake clock/ROS: preview refresh must never send motion.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const page = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/navigation.html'), 'utf8');
function method(name) {
    const match = page.match(new RegExp('(?:async )?function ' + name + '\\([^]*?\\n    }'));
    assert(match, name);
    return match[0];
}
function context() {
    const timers = [], calls = [], logs = [], boxes = {};
    const ctx = {
        now: 1000, timers, calls, logs, boxes,
        document: {getElementById: id => boxes[id] ||= {
            value: '1', checked: false, replaceChildren() {}}},
        window: {setTimeout(fn) {timers.push(fn); return timers.length;}, clearTimeout() {}},
        routeDraft: {id: 8, name: 'route', map_name: 'garden', points: [{x: 1, y: 2}]},
        routeDraftDirty: false, routeEditing: false, routeServiceBusy: false,
        navigationUiMode: 'multi', routeExecutionStatus: {mode: 'idle'},
        loadedMapName: 'garden', currentMapLoadId: 'load-1',
        localizationGeneration: 1, threeDRobotBodyGeneration: 1, singleGoalRefreshState: '',
        threeDRoutePreview: null, threeDRoutePreviewGeneration: 0, threeDRoutePickGeneration: 0,
        threeDRoutePoseTimer: null, threeDRoutePoseLease: 0,
        threeDRoutePreviewIssue: () => '', isXingjiThreeD: () => true,
        routeIsActive: () => false, goalReadinessIssue: () => '',
        threeDNavigationMode: 'terrain_3d',
        UI: {TYPES: {}}, updateRouteUi() {}, beginMotionLocalizationWatch() {},
        expectGlobalPlan() {}, boundedGlobalPlanPoints: x => x,
        navigationPointCloudViewer: {clearPreviewPath() {}, setPreviewPath() {}},
        log: s => logs.push(s),
    };
    ctx.Date = {now: () => ctx.now};
    ctx.session = {connected: true, call: async (name) => {
        calls.push(name);
        if (name === '/start_route') return {success: false, message: '地图已变化，请重新预览'};
        return {valid: true, preview_id: 'preview-1', map_name: 'garden',
            path: {header: {frame_id: 'map'}, poses: [{}, {}]}};
    }};
    ctx.requestNoMotionUpdate = async () => {calls.push('/request_nomotion_update'); return true;};
    vm.createContext(ctx);
    for (const name of ['clearThreeDRoutePreview', 'stopThreeDRoutePoseRefresh',
        'maintainThreeDRoutePoseFreshness', 'threeDRoutePreviewFresh',
        'previewThreeDRoute', 'startRouteNavigation']) vm.runInContext(method(name), ctx);
    return ctx;
}
(async () => {
    const ctx = context();
    await vm.runInContext('previewThreeDRoute()', ctx);
    assert(ctx.threeDRoutePreview);
    assert.equal(ctx.timers.length, 1, 'successful preview schedules stationary refresh');
    for (let i = 0; ctx.timers.length && i < 25; i++) {
        ctx.now += 1000;
        await ctx.timers.shift()();
    }
    assert(ctx.calls.filter(x => x === '/request_nomotion_update').length > 1);
    assert.equal(ctx.now, 19000, 'maintenance ends at 18-second preview expiry');
    assert(!ctx.calls.includes('/start_route'), 'preview and its timer never start navigation');

    for (const change of ['clear', 'disconnect', 'map', 'body', 'localization', 'busy', 'failure']) {
        const c = context();
        await vm.runInContext('previewThreeDRoute()', c);
        if (change === 'clear') vm.runInContext('clearThreeDRoutePreview()', c);
        if (change === 'disconnect') c.session.connected = false;
        if (change === 'map') c.currentMapLoadId = 'new-load';
        if (change === 'body') c.threeDRobotBodyGeneration++;
        if (change === 'localization') c.localizationGeneration++;
        if (change === 'busy') c.routeServiceBusy = true;
        if (change === 'failure') c.requestNoMotionUpdate = async () => false;
        await c.timers.shift()();
        assert.equal(c.timers.length, 0, change + ' must stop maintenance');
        assert(!c.calls.includes('/start_route'));
    }
    const late = context();
    await vm.runInContext('previewThreeDRoute()', late);
    let finish;
    late.requestNoMotionUpdate = () => new Promise(resolve => {finish = resolve;});
    const tick = late.timers.shift()();
    vm.runInContext('clearThreeDRoutePreview()', late);
    finish(true); await tick;
    assert.equal(late.timers.length, 0, 'late reply cannot revive canceled maintenance');

    const start = context();
    await vm.runInContext('previewThreeDRoute()', start);
    await vm.runInContext('startRouteNavigation()', start);
    assert.equal(start.calls.filter(x => x === '/start_route').length, 1);
    assert.match(start.threeDRouteNotice, /地图已变化/, 'show backend rejection beside button');
    await start.timers.shift()();
    assert.equal(start.timers.length, 0);
    assert.equal(start.threeDRoutePreview, null);

    const changed = context();
    const call = changed.session.call;
    changed.session.call = async (...args) => {
        const result = await call(...args);
        changed.threeDRoutePreviewIssue = () => '本体状态已失联';
        return result;
    };
    await vm.runInContext('previewThreeDRoute()', changed);
    assert.match(changed.threeDRouteNotice, /本体状态已失联/);
    assert.equal(changed.threeDRoutePreview, null);
    console.log('Route preview maintenance, fences, manual Start and rejection details: OK');
})().catch(error => {console.error(error); process.exitCode = 1;});
