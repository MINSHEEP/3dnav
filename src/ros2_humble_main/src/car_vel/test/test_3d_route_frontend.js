#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const frontend = path.join(__dirname, '..', 'car_vel', 'web_frontend');
const page = fs.readFileSync(path.join(frontend, 'navigation.html'), 'utf8');
const viewerSource = fs.readFileSync(path.join(frontend, 'pointcloud_viewer.js'), 'utf8');
const script = page.split('<script>')[1].split('</script>')[0];
new Function(script);

for (const required of [
  'id="btn-three-d-preview-route"',
  "'/navigation/preview_3d_route'",
  'map_load_id: requestLoadId',
  'loop_count: loopCount',
  'preview_id: previewId',
  'navigationPointCloudViewer.setRoutePoints(',
  'navigationPointCloudViewer.setGroundPoseInteraction(',
  'threeDRoutePreviewFresh()',
  "threeDNavigationMode === 'terrain_3d'",
]) {
  assert(page.includes(required), `missing 3D route contract: ${required}`);
}
const start = script.match(/async function startRouteNavigation\(\) \{[^]*?\n    \}/);
assert(start && start[0].includes('if (xingji3d && !threeDRoutePreviewFresh())'),
  'Xingji 3D route may not start without a fresh whole-route preview');
assert(start[0].includes('preview_id: previewId'),
  '3D route start must send the backend-issued preview token');
assert(start[0].includes('navigation_mode: requestNavigationMode'),
  'route start must freeze the selected mode against a delayed mode switch');
assert(start[0].includes('!isGangbenThreeD()')
  && start[0].includes("threeDNavigationMode !== 'map_2d'"),
  'stale navigation mode must not fall back to the old 2D start flow');
const mapPose = script.match(/async function handleMapPose\(x, y, yaw\) \{[^]*?\n    \}/);
assert(mapPose && mapPose[0].includes('const gangbenMapInteraction = isGangbenThreeD()')
  && mapPose[0].includes("threeDNavigationMode !== 'map_2d' && !gangbenMapInteraction"),
  '2D canvas must remain read-only in Xingji 3D but preserve Gangben initial/route editing');

const bodySelection = script.match(/function confirmedThreeDBody\(nowMs = Date\.now\(\)\) \{[^]*?\n    \}/);
const xingjiSelection = script.match(/function isXingjiThreeD\(nowMs = Date\.now\(\)\) \{[^]*?\n    \}/);
const gangbenSelection = script.match(/function isGangbenThreeD\(nowMs = Date\.now\(\)\) \{[^]*?\n    \}/);
assert(bodySelection && xingjiSelection && gangbenSelection,
  '3D route policy must distinguish confirmed robot bodies');
const bodyContext = {
  threeDNavigationMode: 'terrain_3d', threeDNavigationModeAtMs: Date.now(),
  threeDRobotBodyAtMs: Date.now(), THREE_D_STATUS_MAX_AGE_MS: 3000,
  threeDRobotBodyStatus: {
    connected: true, switching: false, error: '', active_body: 'gangben_zsl1',
    config: { selected: 'gangben_zsl1', profiles: {
      gangben_zsl1: { navigation_verified: true },
      xingji: { navigation_verified: true },
    } },
  },
};
vm.createContext(bodyContext);
for (const declaration of [bodySelection[0], xingjiSelection[0], gangbenSelection[0]]) {
  vm.runInContext(declaration, bodyContext);
}
assert.strictEqual(vm.runInContext('isGangbenThreeD()', bodyContext), true);
assert.strictEqual(vm.runInContext('isXingjiThreeD()', bodyContext), false,
  'Gangben must not be forced into the Xingji-only 3D preview gate');
bodyContext.threeDRobotBodyStatus.active_body = 'xingji';
bodyContext.threeDRobotBodyStatus.config.selected = 'xingji';
assert.strictEqual(vm.runInContext('isXingjiThreeD()', bodyContext), true);
bodyContext.threeDRobotBodyAtMs -= 5000;
assert.strictEqual(vm.runInContext('isXingjiThreeD()', bodyContext), false,
  'stale body status must not silently select either route policy');

const previewFresh = script.match(/function threeDRoutePreviewFresh\(\) \{[^]*?\n    \}/);
assert(previewFresh, '3D route freshness guard must be testable');
const previewContext = {
  threeDRoutePreviewIssue: () => '',
  threeDRoutePreview: {
    previewId: 'token', previewedAtMs: Date.now(), routeId: 7,
    mapName: 'map_a', mapLoadId: 'load_a', loopCount: 2,
    continuous: false, bodyGeneration: 8, localizationGeneration: 3,
    refreshState: '',
  },
  loadedMapName: 'map_a', currentMapLoadId: 'load_a', routeDraft: { id: 7 },
  threeDRobotBodyGeneration: 8, localizationGeneration: 3,
  singleGoalRefreshState: '',
  document: { getElementById: (id) => ({
    value: id === 'route-loop-count' ? '2' : undefined,
    checked: false,
  }) },
};
vm.createContext(previewContext);
vm.runInContext(page.match(/function threeDRoutePreviewMatches\([^]*?\n    }/)[0]+'\n'+previewFresh[0], previewContext);
assert.strictEqual(vm.runInContext('threeDRoutePreviewFresh()', previewContext), true);
previewContext.currentMapLoadId = 'load_b';
assert.strictEqual(vm.runInContext('threeDRoutePreviewFresh()', previewContext), false,
  'a same-name map reload must revoke the old route preview');
previewContext.currentMapLoadId = 'load_a';
previewContext.threeDRobotBodyGeneration = 9;
assert.strictEqual(vm.runInContext('threeDRoutePreviewFresh()', previewContext), false,
  'switching robot body must revoke the old route preview');
previewContext.threeDRobotBodyGeneration = 8;
previewContext.threeDRoutePreview.previewedAtMs -= 20000;
assert.strictEqual(vm.runInContext('threeDRoutePreviewFresh()', previewContext), true,
  'unchanged route preview must not expire from human waiting');

function makeViewer() {
  let next = 0;
  const draws = [];
  const gl = {
    VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, COMPILE_STATUS: 3, LINK_STATUS: 4,
    DEPTH_TEST: 5, LEQUAL: 6, ARRAY_BUFFER: 7, DYNAMIC_DRAW: 8,
    COLOR_BUFFER_BIT: 16, DEPTH_BUFFER_BIT: 32, FLOAT: 9,
    POINTS: 10, TRIANGLES: 11, TRIANGLE_STRIP: 12,
    createShader: () => ++next, shaderSource() {}, compileShader() {},
    getShaderParameter: () => true, getShaderInfoLog: () => '',
    createProgram: () => ++next, attachShader() {}, linkProgram() {},
    getProgramParameter: () => true, getProgramInfoLog: () => '',
    createBuffer: () => ++next, enable() {}, disable() {}, depthFunc() {},
    bindBuffer() {}, bufferData() {}, viewport() {}, clearColor() {}, clear() {},
    useProgram() {}, getAttribLocation: () => 0, enableVertexAttribArray() {},
    vertexAttribPointer() {}, getUniformLocation: () => 0,
    uniform1f() {}, uniform2f() {},
    drawArrays: (mode, _first, count) => draws.push({ mode, count }),
  };
  const container = { getBoundingClientRect: () => ({
    left: 0, top: 0, width: 800, height: 600,
  }) };
  const canvas = {
    hidden: false, width: 0, height: 0, parentElement: container,
    addEventListener() {}, getContext: () => gl,
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 600 }),
  };
  const browser = { devicePixelRatio: 1, addEventListener() {} };
  vm.runInNewContext(viewerSource, { window: browser, console });
  const viewer = new browser.PointCloudViewer(canvas, container);
  viewer.cloudTransform = { centerX: 0, centerY: 0, centerZ: 0, radius: 2 };
  viewer.setTopView();
  return { viewer, draws, gl };
}

const { viewer, draws, gl } = makeViewer();
viewer.setRoutePoints([
  { id: 42, name: '首点', x: 0, y: 0, z: 0, yaw: 0 },
  { id: 43, name: '末点', x: 1, y: 0, z: 0, yaw: 0 },
]);
assert.strictEqual(viewer.routePathVertexCount, 4,
  'a two-point 3D route must display its connector');
assert.strictEqual(viewer.pickRoutePoint(400, 300).id, 42,
  'route marker picking must preserve the saved point id');
assert(draws.some((entry) => entry.mode === gl.TRIANGLE_STRIP && entry.count === 4),
  'saved route connector must be rendered over the cloud');
viewer.setCheckpointPoints([{ x: 1, y: 0, z: 0, name: '检测点' }]);
assert.strictEqual(viewer.checkpointPoints.length, 1,
  'inspection checkpoints share the same point-cloud marker API');
viewer.clearRoutePoints();
assert.strictEqual(viewer.pickRoutePoint(400, 300), null,
  'deleted route points must no longer be selectable');

const routePicker = script.match(/async function handleThreeDRoutePose\(x, y, yaw\) \{[^]*?\n    \}/);
assert(routePicker, '3D route point handler must be testable');

async function verifyGroundCheckedEditing() {
  const logs = [];
  let ground = { valid: false, message: '未知地面', ground_z: 0 };
  const draft = { id: 4, map_name: 'map_a', points: [] };
  const context = {
    threeDInteractionIssue: () => '', navigationUiMode: 'multi',
    routeDraft: draft, routeEditing: true, routeServiceBusy: false,
    routeIsActive: () => false,
    clearThreeDRoutePreview() { this.threeDRoutePickGeneration += 1; },
    threeDRoutePickGeneration: 0, routeReplaceIndex: null,
    loadedMapName: 'map_a', currentMapLoadId: 'load_a',
    UI: { TYPES: { check3dGround: 'lzp_interface/srv/Check3DGround' } },
    session: { connected: true, call: async () => ground },
    clickMode: 'route', updateRouteUi() {},
    markRouteDirty() { this.routeDraftDirty = true; },
    log: (line) => logs.push(line),
  };
  // Methods are called as free functions in the browser, not as object methods.
  context.clearThreeDRoutePreview = () => { context.threeDRoutePickGeneration += 1; };
  context.markRouteDirty = () => { context.routeDraftDirty = true; };
  vm.createContext(context);
  vm.runInContext(routePicker[0], context);
  await vm.runInContext('handleThreeDRoutePose(1, 2, 0)', context);
  assert.strictEqual(draft.points.length, 0,
    'unknown ground must not be persisted as a 3D route point');
  ground = { valid: true, ground_z: 0.35, message: '地面有效' };
  await vm.runInContext('handleThreeDRoutePose(1, 2, 0)', context);
  assert.strictEqual(draft.points.length, 1, `3D pick failed: ${logs.join(' | ')}`);
  assert.strictEqual(draft.points[0].z, 0.35,
    '3D route height must be the validated map ground height');
  context.routeReplaceIndex = 0;
  ground = { valid: true, ground_z: 0.7, message: '地面有效' };
  await vm.runInContext('handleThreeDRoutePose(3, 4, 1)', context);
  assert.strictEqual(draft.points.length, 1);
  assert.strictEqual(draft.points[0].x, 3);
  assert.strictEqual(draft.points[0].z, 0.7);
  assert.strictEqual(context.routeReplaceIndex, null);
  assert.strictEqual(context.routeDraftDirty, true,
    'point replacement must invalidate the old saved-route preview');
  assert(logs.some((line) => line.includes('未修改')),
    'rejected pick should explain that the route was not changed');
}

verifyGroundCheckedEditing().then(() => {
  console.log('3D route ground-checked editing, markers and preview contract: ok');
}).catch((error) => { console.error(error); process.exitCode = 1; });
