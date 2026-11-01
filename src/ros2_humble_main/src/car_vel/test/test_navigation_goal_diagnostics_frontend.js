'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const page = fs.readFileSync(path.join(__dirname, '..', 'car_vel', 'web_frontend', 'navigation.html'), 'utf8');

assert(page.includes('id="navigation-goal-readiness"'), 'readiness needs a persistent visible message');
assert(page.includes('id="btn-reset-navigation-emergency"'), 'hard reset must be a separate explicit operation');
const start = page.indexOf('    const appendNavigationLog = ');
const end = page.indexOf('\n    const ', start + 8);
assert(start >= 0 && end > start, 'navigation log wrapper is missing');
const sent = [];
const local = [];
const context = {
  document: {getElementById() {return {}; }},
  UI: {createLogger() {return (text) => local.push(text); }, TYPES: {string: 'std_msgs/msg/String'}},
  session: {connected: true, publish(...args) {sent.push(args); }},
  loadedMapName: 'campus',
};
vm.runInNewContext(page.slice(start, end), context);
vm.runInNewContext('log("暂不能发送导航目标：初始定位尚未通过确认");', context);
assert.strictEqual(sent.length, 1);
assert.strictEqual(sent[0][0], '/navigation/goal_ui_event');
assert(JSON.parse(sent[0][2].data).reason.includes('初始定位'));
assert.strictEqual(local.length, 1);
assert(!sent.some(([topic]) => topic === '/web_goal_pose' || topic === '/clear_emergency_stop'));
const calls = [...page.matchAll(/clearEmergencyStop\(\);/g)];
assert.strictEqual(calls.length, 1, 'only the dedicated confirmed reset button may clear hard emergency');
const resetAt = page.indexOf("'btn-reset-navigation-emergency').addEventListener");
assert(resetAt >= 0);
const reset = page.slice(resetAt, page.indexOf('\n    });', resetAt) + 8);
assert(reset.includes('window.confirm('), 'operator confirmation is required');
assert(reset.includes('navigationActive') && reset.includes('routeIsActive()'), 'running tasks must be stopped first');
assert(reset.includes('clearEmergencyStop();'));
let resetHandler;
const resetContext = {
  document: {getElementById() {return {addEventListener(_event, callback) {resetHandler = callback; }}; }},
  session: {connected: true}, navigationActive: false, routeIsActive: () => false,
  relocalizationActive: false, threeDRobotBodyStatus: {connected: true, status: {emergency_latched: true}},
  threeDRobotBodyAtMs: Date.now(), THREE_D_STATUS_MAX_AGE_MS: 3000,
  navigationSafetyPending: false, navigationSafetyReady: true,
  navigationSafetyManualHold: false, navigationSafetyRetryTimer: null,
  window: {confirm: () => false, clearTimeout() {}}, log() {}, updateMapReadyUi() {},
  clearEmergencyStop: () => sent.push(['explicit-reset']),
};
const resetStart = page.lastIndexOf('    document.getElementById(', resetAt);
vm.runInNewContext(page.slice(resetStart, page.indexOf('\n    });', resetAt) + 8), resetContext);
const beforeReset = sent.length;
resetHandler();
assert.strictEqual(sent.length, beforeReset, 'canceling confirmation must not reset');
resetContext.window.confirm = () => true;
resetContext.navigationActive = true;
resetHandler();
assert.strictEqual(sent.length, beforeReset, 'an active task cannot be reset');
resetContext.navigationActive = false;
resetContext.threeDRobotBodyAtMs = 0;
resetHandler();
assert.strictEqual(sent.length, beforeReset, 'stale body data cannot authorize reset');
resetContext.threeDRobotBodyAtMs = Date.now();
resetHandler();
assert.strictEqual(sent.length, beforeReset + 1, 'only a separate confirmed operation can reset');
assert.strictEqual(resetContext.navigationSafetyManualHold, true,
    'reset must also suspend page-entry automatic standing retries');
const bodySubscription = page.slice(page.indexOf("activeSession.subscribe('/robot_body/status'"),
    page.indexOf('            ensureNavigationSafety(activeSession);'));
assert(!bodySubscription.includes("if (navigationMapView === '3d') updateMapReadyUi();"),
    '2D must also refresh the explicit reset state after a body heartbeat');
console.log('Navigation decision telemetry and explicit emergency reset: OK');
