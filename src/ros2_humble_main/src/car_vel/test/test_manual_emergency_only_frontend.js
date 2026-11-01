/* Actual navigation callbacks, fake transports only. */
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const file = path.join(__dirname, '../car_vel/web_frontend/navigation.html');
const source = fs.readFileSync(file, 'utf8');
function fn(name, next) {
    const start = source.indexOf(`    function ${name}(`);
    const end = source.indexOf(`\n    function ${next}(`, start);
    assert(start > 0 && end > start);
    return source.slice(start, end);
}
function context(mode = 'single') {
    const sent = [], handlers = {};
    const ctx = { sent, session: {connected: true, publish: (topic, type, message) => sent.push({topic, message})},
        UI: {TYPES: {empty: 'std_msgs/Empty', twist: 'geometry_msgs/Twist'}},
        window: {addEventListener: (name, callback) => handlers[name] = callback, clearTimeout() {}, clearInterval() {}},
        routeExecutionStatus: {mode}, latestGlobalPlanPoints: [],
        mapSnapshotRecovery: {cancel() {ctx.snapshotCancelled=true;}},
        savedMapCloud: {dispose() {ctx.fullMapDisposed=true;}},
        stopThreeDPreviewPoseRefresh() {}, navigationSafetyRetryTimer: null, localizationWatchdogTimer: null,
        automaticTerrainWatchdogTimer: null,
        clearRelocalizationWatchdog() {}, clearNoMotionUpdateRequests() {}, requestRelocalizationCancel() {},
        clearGlobalPlan() {}, setGlobalPlanStatus() {}, mapCanvas: {clearGoalPose() {}},
        localizationReady: true, localizationHealth: 'good', localizationHealthReason: '',
        LOCALIZATION_HEALTH: {LOST: 'lost'}, localizationSafetyMotionActive: () => true,
        clearInitialPoseConfirmation() {}, setMode() {}, updateMapReadyUi() {}, log() {}};
    vm.createContext(ctx);
    vm.runInContext(fn('cancelNavigation', 'clearEmergencyStop'), ctx);
    return {ctx, sent, handlers};
}
{
    const {ctx, sent, handlers} = context();
    const begin = source.indexOf("    window.addEventListener('beforeunload', () => {");
    const end = source.indexOf('\n    });', begin) + '\n    });'.length;
    vm.runInContext(source.slice(begin, end), ctx);
    handlers.beforeunload();
    assert(ctx.snapshotCancelled,'page exit must cancel pending map recovery');
    assert(ctx.fullMapDisposed,'page exit must dispose complete-map transfer');
    assert(sent.some(item => item.topic === '/cancel_navigation'));
    assert(sent.some(item => item.topic === '/cmd_vel_nav' && item.message.linear.x === 0));
    assert(!sent.some(item => item.topic === '/emergency_stop'), 'page exit must not hard-stop');
}
{
    const {ctx, sent} = context();
    vm.runInContext(fn('latchLocalizationLoss', 'extendEvidenceWindow'), ctx);
    ctx.latchLocalizationLoss({positionStep: 1, yawStep: 1});
    assert(sent.some(item => item.topic === '/motion_stop'), 'confirmed localization loss must latch zero stop');
    assert(!sent.some(item => item.topic === '/emergency_stop'));
    assert.equal(ctx.localizationHealth, 'lost');
}
const remote = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/remote_control.html'), 'utf8');
assert(remote.includes('data-command="emergency"'));
assert(remote.includes("t: 'emergency'"));
assert(remote.includes("session.publish('/dog_remote/command'"));
console.log('PASS: normal page exit zero-cancels; LOST soft-latches; remote button/T emergency retained');
