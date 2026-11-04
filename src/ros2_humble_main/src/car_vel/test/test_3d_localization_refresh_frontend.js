'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const page = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/navigation.html'), 'utf8');
function method(name) {
    const match = page.match(new RegExp('(?:async )?function ' + name + '\\([^]*?\\n    }'));
    assert(match, `${name} missing`);
    return match[0];
}
function context() {
    const box = {};
    const ctx = {document: {getElementById: () => box}, navigationActive: false,
        routeActive: false, navigationMapReady: true, relocalizationActive: false,
        goalReadinessIssue: () => '', singleGoalRefreshState: '',
        navigationMapView: '3d', threeDNavigationMode: 'terrain_3d',
        threeDLocalizationRefreshState: 'idle', latestAmclReceivedAtMs: Date.now() - 5100,
        threeDPoseFreshnessLimitMs: 3000, amclDataFresh: () => false,
        threeDDraftGeneration: 1, currentMapLoadId: 'load-1',
        threeDInteractionIssue: () => '', updateThreeDPoseUi() {}, updateMapReadyUi() {},
        stopThreeDPreviewPoseRefresh() {}, maintainThreeDPreviewPoseFreshness() {},
        threeDNotice: '', UI: {TYPES: {}, goalPoseMessage: () => ({})},
        navigationPointCloudViewer: {clearPreviewPath() {}, setPreviewPath() {}},
        boundedGlobalPlanPoints: values => values, log() {}, box};
    ctx.threeDPoseDraft = {kind: 'goal', groundValid: true, busy: false, x: 1, y: 2, yaw: 0,
        mapName: 'garden', mapLoadId: 'load-1', previewId: 'old', previewedAtMs: Date.now()};
    vm.createContext(ctx);
    return ctx;
}
const block = page.slice(page.indexOf('        const readinessBox ='), page.indexOf('        const resetEmergencyButton ='));
function readiness(ctx) {vm.runInContext(`{${block}}`, ctx); return ctx.box.textContent;}

(async () => {
    const ctx = context();
    assert.match(readiness(ctx), /预览前.*刷新|刷新.*预览/, 'stale 3D must not claim localization is ready');
    ctx.threeDLocalizationRefreshState = 'refreshing';
    assert.match(readiness(ctx), /刷新中/);
    ctx.threeDLocalizationRefreshState = 'repreview';
    assert.match(readiness(ctx), /重新预览/);
    ctx.navigationMapView = '2d';
    assert.match(readiness(ctx), /定位与导航已就绪/, '2D readiness behavior must be retained');

    const preview = context();
    const calls = [];
    preview.session = {call: async (...args) => {
        calls.push(args[0]);
        return {valid: false, code: 'localization_refresh_timeout', message: '等待新定位超时'};
    }};
    vm.runInContext(method('previewThreeDGoal'), preview);
    await vm.runInContext('previewThreeDGoal()', preview);
    assert.match(preview.threeDNotice, /无法预览路径/);
    assert(!preview.threeDNotice.includes('无法启动导航'));
    assert.deepEqual(calls, ['/navigation/preview_3d_goal']);

    const start = context();
    start.session = {call: async (...args) => {
        calls.push(args[0]);
        return {success: false, code: 'localization_refreshed_repreview', message: '定位已刷新，请重新预览'};
    }};
    vm.runInContext(method('startThreeDPreviewedGoal'), start);
    await vm.runInContext('startThreeDPreviewedGoal()', start);
    assert.equal(start.threeDPoseDraft.previewId, '');
    assert.equal(start.threeDLocalizationRefreshState, 'repreview');
    assert.equal(calls.length, 2, 'refresh response must not auto-call Preview or Start again');

    const superseded = context();
    let reject;
    superseded.session = {call: () => new Promise((_resolve, fail) => {reject = fail;})};
    vm.runInContext(method('previewThreeDGoal'), superseded);
    const pending = vm.runInContext('previewThreeDGoal()', superseded);
    superseded.threeDDraftGeneration += 1;
    superseded.threeDPoseDraft = {kind: 'goal', previewId: 'new-user-draft'};
    superseded.threeDNotice = 'new candidate';
    reject(new Error('late old refresh failure'));
    await pending;
    assert.equal(superseded.threeDNotice, 'new candidate', 'old RPC failure must not overwrite a newer draft');
    assert(!page.includes('maintainThreeDPreviewPoseFreshness'),
        'waiting for a human must not require an AMCL keepalive timer');
    console.log('3D pose refresh readiness, preview-only errors and manual restart semantics: OK');
})().catch(error => {console.error(error); process.exitCode = 1;});
