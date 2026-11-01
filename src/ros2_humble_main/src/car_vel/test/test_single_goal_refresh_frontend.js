#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const page = fs.readFileSync(path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'navigation.html'), 'utf8');
const statusStart = page.indexOf('    const goalStatusText = Object.freeze({');
const statusEnd = page.indexOf('\n    function setMode(', statusStart);
const callbackStart = page.indexOf(
  "activeSession.subscribe('/goal_pose_status', UI.TYPES.string, (message) => {");
const callbackEnd = page.indexOf(
  "\n            activeSession.subscribe('/nav2_ready'", callbackStart);
const activeStart = page.indexOf(
  "activeSession.subscribe('/navigation_active', UI.TYPES.bool, (message) => {");
const activeEnd = page.indexOf(
  "\n            activeSession.subscribe(\n                '/route_execution_status'", activeStart);
const markerStart = page.indexOf('    function restorePreviousSingleGoalMarker()');
const markerEnd = page.indexOf('\n    function selectedMap()', markerStart);
const readinessStart = page.indexOf('    function goalReadinessIssue(');
const readinessEnd = page.indexOf('\n    function lioFaultDiagnostics(', readinessStart);
assert(statusStart >= 0 && statusEnd > statusStart);
assert(callbackStart >= 0 && callbackEnd > callbackStart);
assert(activeStart >= 0 && activeEnd > activeStart);
assert(markerStart >= 0 && markerEnd > markerStart);
assert(readinessStart >= 0 && readinessEnd > readinessStart);
const statusSource = page.slice(statusStart, statusEnd);
const callbackSource = page.slice(callbackStart, callbackEnd);
const activeSource = page.slice(activeStart, activeEnd);
const markerSource = page.slice(markerStart, markerEnd);
const readinessSource = page.slice(readinessStart, readinessEnd);

function exercise(statuses, initialActive, pendingRequest = true, hydrated = true) {
  const events = { logs: [], modes: [], clearMarkers: 0, clearPlans: [], updates: 0 };
  const callbacks = {};
  const activeSession = {
    connected: true,
    subscribe(topic, _type, handler) { callbacks[topic] = handler; },
  };
  const context = {
    activeSession,
    session: activeSession,
    UI: { TYPES: { string: 'std_msgs/msg/String', bool: 'std_msgs/msg/Bool' } },
    log(value) { events.logs.push(value); },
    setMode(value) { events.modes.push(value); },
    nav2Ready: true,
    navigationActive: initialActive,
    navigationActiveKnown: true,
    singleGoalRefreshState: '',
    singleGoalRequestPending: pendingRequest,
    previousSingleGoalMarker: initialActive ? { x: 1, y: 2, yaw: 0.3 } : null,
    singleGoalHadActiveNavigation: initialActive,
    goalStatusHydrated: hydrated,
    localizationReady: true,
    localizationHealth: 'GOOD',
    LOCALIZATION_HEALTH: { GOOD: 'GOOD' },
    refreshModeFromLocalization() {},
    updateMapReadyUi() { events.updates += 1; },
    expectGlobalPlan() { throw new Error('refresh must not expect a plan'); },
    latestGlobalPlanPoints: [],
    globalPlanExpected: false,
    setGlobalPlanStatus() {},
    clearGlobalPlan(value) { events.clearPlans.push(value); },
    resolveCancelWaiters() {},
    mapCanvas: {
      goalPose: { x: 9, y: 8, yaw: -0.4 },
      setGoalPose(x, y, yaw) { this.goalPose = { x, y, yaw }; },
      clearGoalPose() { this.goalPose = null; events.clearMarkers += 1; },
    },
  };
  vm.runInNewContext(
    `${statusSource}\n${markerSource}\n${callbackSource}\n${activeSource}`, context);
  for (const status of statuses) callbacks['/goal_pose_status']({ data: status });
  return { events, context, callbacks };
}

{
  const { events, context } = exercise(['goal_refreshing_localization'], false);
  assert.strictEqual(events.clearMarkers, 1,
    'stale click must remove its provisional goal marker');
  assert.strictEqual(context.navigationActive, false,
    'refresh is not proof Nav2 accepted a goal');
  assert(events.modes.some((mode) => mode.includes('定位刷新')));
  assert(events.logs.some((line) => line.includes('不会自动启动')));
  assert(!events.clearPlans.length,
    'refresh must not hide an in-flight cancellation path');
}

{
  const { events, context } = exercise([
    'goal_refreshing_localization', 'goal_localization_refreshed',
  ], false);
  assert.strictEqual(context.navigationActive, false);
  assert(events.modes.at(-1).includes('重新设置目标'));
  assert(events.logs.at(-1).includes('重新'));
  assert(events.clearMarkers >= 1);
}

{
  const { events, context } = exercise([
    'goal_refreshing_localization', 'goal_localization_refresh_failed',
  ], false);
  assert.strictEqual(context.navigationActive, false);
  assert(events.modes.at(-1).includes('定位'));
  assert(events.logs.at(-1).includes('重新定位'));
}

{
  const { events, context, callbacks } = exercise([
    'goal_refreshing_localization', 'goal_localization_refresh_failed',
  ], true);
  assert.strictEqual(context.navigationActive, true,
    'refresh timeout must not conceal an old goal whose cancel is unconfirmed');
  assert(!events.clearPlans.length);
  assert(events.modes.at(-1).includes('停止待确认'));
  assert(events.logs.at(-1).includes('旧导航可能仍在执行'));
  assert.strictEqual(context.mapCanvas.goalPose.x, 1,
    'old active goal marker must be restored instead of showing rejected new goal');
  // The old goal can finish canceling after the refresh timeout. Its terminal
  // status must not erase the failed-refresh instruction to re-localize.
  callbacks['/navigation_active']({ data: false });
  assert(events.modes.at(-1).includes('定位刷新失败'));
  callbacks['/goal_pose_status']({ data: 'goal_canceled' });
  assert.strictEqual(context.singleGoalRefreshState, 'failed');
  assert.strictEqual(context.navigationActive, false);
  assert.strictEqual(context.mapCanvas.goalPose, null);
  assert(events.modes.at(-1).includes('定位刷新失败'));
}

{
  const { events, context, callbacks } = exercise(['goal_refreshing_localization'], true);
  assert.strictEqual(context.navigationActive, true,
    'old Nav2 goal remains active until backend confirms its cancellation');
  assert(!events.clearPlans.length,
    'do not hide an old active Nav2 path before cancellation is confirmed');
  assert.strictEqual(context.mapCanvas.goalPose.x, 1);
  callbacks['/navigation_active']({ data: false });
  assert.strictEqual(context.navigationActive, false);
  assert.strictEqual(context.mapCanvas.goalPose, null);
  callbacks['/goal_pose_status']({ data: 'goal_localization_refreshed' });
  assert.strictEqual(context.singleGoalRefreshState, 'refreshed');
  assert(events.modes.at(-1).includes('重新设置目标'));
}

{
  const { events, context } = exercise(['goal_rejected_navigation_stopping'], true);
  assert.strictEqual(context.navigationActive, true,
    'new-goal rejection cannot declare an old unconfirmed Nav2 goal stopped');
  assert(!events.clearPlans.length,
    'old active global path must remain visible until backend confirms stop');
  assert.strictEqual(context.mapCanvas.goalPose.x, 1,
    'rejecting the new click must restore the old target marker');
  assert(events.logs.some((line) => line.includes('旧导航')));
}

{
  const { events, context } = exercise(['goal_rejected_goal_blocked'], true);
  assert.strictEqual(context.navigationActive, true,
    'any rejected replacement must leave the old active goal visible');
  assert(!events.clearPlans.length);
  assert.strictEqual(context.mapCanvas.goalPose.x, 1);
}

{
  const { context } = exercise(['goal_rejected_goal_blocked'], false);
  assert.strictEqual(context.navigationActive, false);
  assert.strictEqual(context.mapCanvas.goalPose, null,
    'rejected first goal must not leave a target marker');
}

{
  const { events, context } = exercise(['goal_localization_refreshed'], false, false, false);
  assert.strictEqual(context.singleGoalRefreshState, '',
    'a retained status from an earlier browser session must not change this page');
  assert.strictEqual(events.clearMarkers, 0);
  assert.strictEqual(events.modes.length, 0);
}

{
  const { events, context } = exercise(['goal_refreshing_localization'], false, false, false);
  assert.strictEqual(context.singleGoalRefreshState, '',
    'a retained refresh-start status must not block an unrelated browser session');
  assert.strictEqual(events.clearMarkers, 0);
}

{
  const { events, context } = exercise(['goal_rejected_localization_stale'], false, false, false);
  assert.strictEqual(context.singleGoalRefreshState, '',
    'a retained legacy stale rejection must not disable goals after reconnect');
  assert.strictEqual(events.clearMarkers, 0);
}

{
  const { events, context } = exercise(['goal_rejected_navigation_stopping'], false, false, false);
  assert.strictEqual(context.navigationActive, false,
    'a retained old stop warning must not fabricate a current active goal');
  assert.strictEqual(events.clearMarkers, 0);
}

function readiness(state, active, hydrated = true) {
  return vm.runInNewContext(`${readinessSource}\ngoalReadinessIssue()`, {
    singleGoalRefreshState: state,
    navigationActive: active,
    goalStatusHydrated: hydrated,
    routeIsActive: () => false,
    lioFaultLatched: false,
    nav2Ready: true,
    localizationReady: true,
    localizationHealth: 'GOOD',
    localizationHealthReason: '',
    LOCALIZATION_HEALTH: { GOOD: 'GOOD' },
    amclDataFresh: () => true,
    idleStaleLocalizationAllowed: () => false,
  });
}
assert(readiness('refreshing', false).includes('等待完成'));
assert(readiness('failed', false).includes('重新设置初始点'));
assert(readiness('refreshed', true).includes('等待旧导航停止'));
assert.strictEqual(readiness('refreshed', false), '');
assert(readiness('', false, false).includes('正在同步导航目标状态'),
  'the first retained status must be handled before a browser may send a goal');

{
  const { context, events } = exercise(['ready'], false, false, false);
  assert.strictEqual(context.goalStatusHydrated, true);
  assert(events.updates > 0, 'the goal control should re-enable after status hydration');
}

{
  const { context } = exercise(['goal_rejected_localization_stale'], false, false, false);
  assert.strictEqual(context.goalStatusHydrated, true);
  assert.strictEqual(context.singleGoalRefreshState, '',
    'first retained stale result hydrates status but cannot claim a new request');
}

assert(!callbackSource.includes("session.publish('/web_goal_pose'"),
  'backend refresh status must never re-publish the rejected target');

assert(page.includes('singleGoalRefreshState'),
  'refresh UI needs state independent of frontend AMCL freshness');
assert(page.includes("singleGoalRefreshState === 'refreshing'"),
  'map interaction must be blocked during refresh');
assert(page.includes("singleGoalRefreshState === 'failed'"),
  'failed refresh must require re-localization');

console.log('single-goal manual localization-refresh frontend: ok');
