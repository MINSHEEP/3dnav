#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const frontend = path.join(__dirname, '..', 'car_vel', 'web_frontend');
const page = fs.readFileSync(path.join(frontend, 'navigation.html'), 'utf8');
const common = fs.readFileSync(path.join(frontend, 'robot_ui_common.js'), 'utf8');
const script = page.split('<script>')[1].split('</script>')[0];
new Function(script);

for (const [name, type] of [
  ['check3dGround', 'Check3DGround'],
  ['preview3dGoal', 'Preview3DGoal'],
  ['startPreviewed3dGoal', 'StartPreviewed3DGoal'],
]) {
  assert(common.includes(`${name}: 'lzp_interface/srv/${type}'`),
    `${name} ROS service type is unavailable to the page`);
}
for (const required of [
  'id="three-d-pose-panel"', 'id="btn-three-d-confirm-initial"',
  'id="btn-three-d-preview-goal"', 'id="btn-three-d-start-goal"',
  "subscribe('/navigation/mode'", "subscribe('/robot_body/status'",
  "subscribe('/global_localization_map/status'",
  'currentMapLoadId',
  'latestNavigationGlobalCloudIdentity.load_id !== currentMapLoadId',
  'function tryAcceptNavigationGlobalCloud()',
  'pendingNavigationGlobalCloudStampNs',
  'latestNavigationGlobalCloudIdentity.map_name !== loadedMapName',
  'latestNavigationGlobalCloudIdentity.stamp_ns !== cloudStampNs',
  "'/terrain/check_3d_ground'", "'/navigation/preview_3d_goal'",
  "'/navigation/start_previewed_3d_goal'",
  'map_load_id: draft.mapLoadId',
  'navigationPointCloudViewer.setGroundPoseInteraction(',
  'navigationPointCloudViewer.setPreviewPath(',
  'navigationPointCloudViewer.setDraftPose(',
]) {
  assert(page.includes(required), `3D navigation interaction is missing: ${required}`);
}
assert(/identity\.load_id !== currentMapLoadId/.test(page),
  'point cloud status must match the loaded map generation');
assert(/message\.load_id[\s\S]*?clearNavigationGlobalCloud/.test(page),
  'a confirmed same-name map reload must invalidate the old cloud');

const start = script.match(/async function startThreeDPreviewedGoal\([^]*?\n    }/);
assert(start, '3D start handler must be explicit');
assert(!start[0].includes("session.publish('/web_goal_pose'"),
  '3D start must not bypass backend preview validation');
assert(script.includes("session.publish('/web_goal_pose'"),
  'existing 2D direct goal flow must remain unchanged');

const updateUi = script.match(/function updateThreeDPoseUi\(\) \{[^]*?\n    \}/);
assert(updateUi, '3D interaction status updater must exist');
const controls = Object.fromEntries([
  'three-d-pose-panel', 'three-d-pose-message',
  'btn-three-d-confirm-initial', 'btn-three-d-preview-goal',
  'btn-three-d-start-goal', 'btn-three-d-clear',
].map((id) => [id, {}]));
let previewCleared = false;
const context = {
  document: { getElementById: (id) => controls[id] },
  navigationMapView: '3d',
  clickMode: 'goal',
  navigationPointCloudViewer: {
    clearDraftPose() {},
    clearPreviewPath() { previewCleared = true; },
    setGroundPoseInteraction() {},
  },
  threeDInteractionIssue: () => '',
  goalReadinessIssue: () => '定位正在刷新',
  singleGoalRefreshState: '',
  handleThreeDGroundPose() {},
  threeDNotice: '旧路径预览',
  threeDDraftGeneration: 0,
  threeDPoseDraft: {
    kind: 'goal', groundValid: true, busy: false,
    previewId: 'obsolete-preview', previewedAtMs: Date.now(), refreshState: '',
  },
};
vm.createContext(context);
vm.runInContext(`function clearThreeDPoseDraft(message = '') {
  threeDDraftGeneration += 1;
  threeDPoseDraft = null;
  threeDNotice = message;
  navigationPointCloudViewer.clearDraftPose();
  navigationPointCloudViewer.clearPreviewPath();
}\n${updateUi[0]}\nupdateThreeDPoseUi();`, context);
assert(context.threeDPoseDraft,
  'temporary readiness loss must retain evidence while preventing Start');
assert(!previewCleared, 'a temporary data wait must not erase the preview');
assert.strictEqual(controls['btn-three-d-start-goal'].disabled, true,
  'a stale preview must not leave Start enabled');
previewCleared = false;
context.threeDPoseDraft = {
  kind: 'goal', groundValid: true, busy: false,
  previewId: 'refreshed-preview', previewedAtMs: Date.now(), refreshState: '',
};
context.goalReadinessIssue = () => '';
context.singleGoalRefreshState = 'refreshed';
vm.runInContext('updateThreeDPoseUi();', context);
assert(context.threeDPoseDraft,
  'a same-identity stationary refresh must retain the preview');
assert(!previewCleared, 'refresh alone must not erase the preview');
context.threeDPoseDraft = {
  kind: 'goal', groundValid: true, busy: false,
  previewId: 'new-preview', previewedAtMs: Date.now(), refreshState: 'refreshed',
};
previewCleared = false;
vm.runInContext('updateThreeDPoseUi();', context);
assert.strictEqual(context.threeDPoseDraft.previewId, 'new-preview',
  'a new user-selected goal after refresh must be allowed to preview');
assert.strictEqual(controls['btn-three-d-start-goal'].disabled, false,
  'an explicitly previewed new goal may start after refresh');

const acceptCloud = script.match(/function tryAcceptNavigationGlobalCloud\(\) \{[^]*?\n    \}/);
const trackMap = script.match(/function trackCurrentMapLoadIdentity\(message\) \{[^]*?\n    \}/);
const invalidateCloud = script.match(/function invalidateDisplayedNavigationCloud\(reason\) \{[^]*?\n    \}/);
assert(acceptCloud && trackMap && invalidateCloud,
  '3D map generation matching must be testable as isolated functions');
const cloud = { header: { frame_id: 'map', stamp: { sec: 1, nanosec: 2 } } };
const cloudStatus = { className: '', textContent: '' };
let cloudRenders = 0;
let cloudClears = 0;
const identityContext = {
  savedMapCloud: null,
  recoverActiveRouteMap() {},
  document: { getElementById: () => cloudStatus },
  navigationPointCloudViewer: {
    setPointCloud() { cloudRenders += 1; return { renderedPoints: 3, sourcePoints: 4 }; },
    clear() { cloudClears += 1; },
  },
  navigationMapReady: true,
  loadedMapName: 'ceshi3_remap1',
  pendingMapName: '',
  pendingNavigationGlobalCloud: cloud,
  pendingNavigationGlobalCloudStampNs: '1000000002',
  latestNavigationGlobalCloud: null,
  latestNavigationGlobalCloudIdentity: {
    map_name: 'ceshi3_remap1', load_id: 'old-load', stamp_ns: '1000000002',
  },
  currentMapLoadName: 'ceshi3_remap1',
  currentMapLoadId: 'new-load',
  terrainReloadRequired: null,
  navigationMapView: '3d',
  clearThreeDPoseDraft() {},
  clearThreeDRoutePreview() {},
  updateMapReadyUi() {},
};
vm.createContext(identityContext);
vm.runInContext(`${invalidateCloud[0]}\n${acceptCloud[0]}\n${trackMap[0]}`, identityContext);
vm.runInContext('tryAcceptNavigationGlobalCloud();', identityContext);
assert.strictEqual(cloudRenders, 0,
  'same map name and stamp are insufficient when load_id differs');
identityContext.latestNavigationGlobalCloudIdentity.load_id = 'new-load';
vm.runInContext('tryAcceptNavigationGlobalCloud();', identityContext);
assert.strictEqual(cloudRenders, 1, 'matching cloud identity should render once');
assert.strictEqual(identityContext.latestNavigationGlobalCloud, cloud);
identityContext.pointCloudStampNanoseconds = () => '1000000002';
vm.runInContext(`trackCurrentMapLoadIdentity({
  success: true, map_name: "ceshi3_remap1", load_id: "newer-load"
});`, identityContext);
assert.strictEqual(identityContext.currentMapLoadId, 'newer-load');
assert.strictEqual(identityContext.latestNavigationGlobalCloud, null,
  'a confirmed same-name reload must hide the old cloud immediately');
assert.strictEqual(cloudClears, 1);

const mapPoseHandler = script.match(/async function handleMapPose\(x, y, yaw\) \{[^]*?\n    \}/);
const initialPosePublisher = script.match(/async function publishInitialPose\([^]*?\n    \}/);
assert(mapPoseHandler && initialPosePublisher,
  '2D map click and shared initial-pose publisher must be testable');

async function verifyViewAndMapGenerationGuards() {
  async function clickGoal(navigationMode, gangben = false) {
    const published = [];
    const logs = [];
    let clearedEmergency = 0;
    const context = {
      navigationMapReady: true, session: {
        connected: true,
        publish(topic) { published.push(topic); },
      },
      clickMode: 'goal', navigationUiMode: 'single',
      threeDNavigationMode: navigationMode,
      threeDNavigationModeAtMs: Date.now(), THREE_D_STATUS_MAX_AGE_MS: 3000,
      isGangbenThreeD: () => gangben,
      log(value) { logs.push(value); },
      goalReadinessIssue: () => '', localizationGeneration: 4,
      localizationHealthyForControl: () => true,
      navigationActive: false, latestGlobalPlanPoints: [],
      mapCanvas: { goalPose: null, setGoalPose() {} },
      clearEmergencyStop() { clearedEmergency += 1; },
      beginMotionLocalizationWatch() {}, expectGlobalPlan() {},
      updateMapReadyUi() {}, UI: {
        TYPES: { poseStamped: 'geometry_msgs/msg/PoseStamped' },
        goalPoseMessage: () => ({}),
      },
    };
    vm.createContext(context);
    await vm.runInContext(`${mapPoseHandler[0]}\nhandleMapPose(1, 2, 0);`, context);
    return { published, logs, clearedEmergency };
  }
  const terrain3d = await clickGoal('terrain_3d');
  assert.deepStrictEqual(terrain3d.published, [],
    '2D-view goal clicks in terrain_3d must not bypass preview/start');
  assert.strictEqual(terrain3d.clearedEmergency, 0,
    'blocked 3D goal click must not clear the emergency stop');
  assert(terrain3d.logs.some((line) => line.includes('三维')),
    'the page should direct the user to the 3D preview/start flow');
  const map2d = await clickGoal('map_2d');
  assert.deepStrictEqual(map2d.published, ['/web_goal_pose'],
    'map_2d direct goal behavior must remain unchanged');
  const gangben3d = await clickGoal('terrain_3d', true);
  assert.deepStrictEqual(gangben3d.published, ['/web_goal_pose'],
    'confirmed Gangben 3D must preserve the legacy direct-goal path');

  let resolveCancellation;
  const cancellation = new Promise((resolve) => { resolveCancellation = resolve; });
  const initialPublishes = [];
  const initialContext = {
    lioFaultLatched: false, relocalizationActive: false,
    latestOdometryStamp: { sec: 1, nanosec: 0 },
    localizationGeneration: 0, loadedMapName: 'ceshi3_remap1',
    currentMapLoadId: 'load-before', navigationMapReady: true,
    session: { connected: true, publish(topic) { initialPublishes.push(topic); } },
    LOCALIZATION_HEALTH: { DEGRADED: 'DEGRADED' },
    clearRelocalizationWatchdog() {}, clearInitialPoseConfirmation() {},
    clearThreeDRoutePreview() {},
    clearNoMotionUpdateRequests() {}, clearTrackingEvidence() {},
    initialPoseTimeout: null, window: { clearTimeout() {} },
    mapCanvas: { setRobotPose() {}, clearGoalPose() {}, clearRobotPose() {} },
    navigationPointCloudViewer: null,
    setRelocalizationUi() {}, updateMapReadyUi() {},
    cancelNavigation: () => cancellation, log() {},
  };
  vm.createContext(initialContext);
  vm.runInContext(initialPosePublisher[0], initialContext);
  const initialPromise = vm.runInContext(
    'publishInitialPose(1, 2, 0, "load-before")', initialContext);
  initialContext.currentMapLoadId = 'load-after';
  resolveCancellation(true);
  assert.strictEqual(await initialPromise, false,
    '3D initial-pose publication must abort after same-name map reload');
  assert.deepStrictEqual(initialPublishes, []);
  assert.strictEqual(initialContext.relocalizationActive, false,
    'aborted 3D initial pose must not leave localization stuck active');
}

verifyViewAndMapGenerationGuards().then(() => {
  console.log('Xingji-only 3D candidate, preview and explicit start contract: ok');
}).catch((error) => { console.error(error); process.exitCode = 1; });
