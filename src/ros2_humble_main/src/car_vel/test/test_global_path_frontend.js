#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const viewerSource = fs.readFileSync(path.join(frontend, 'pointcloud_viewer.js'), 'utf8');
const common = fs.readFileSync(path.join(frontend, 'robot_ui_common.js'), 'utf8');
const navigation = fs.readFileSync(path.join(frontend, 'navigation.html'), 'utf8');

const scriptStart = navigation.lastIndexOf('<script>');
const scriptEnd = navigation.indexOf('</script>', scriptStart);
new Function(navigation.slice(scriptStart + '<script>'.length, scriptEnd));

for (const token of [
  "path: 'nav_msgs/msg/Path'",
  'setGlobalPath(points)',
  'clearGlobalPath()',
  'drawGlobalPath(ctx)',
  "'#45e39b'",
  "'#ffb454'",
  'ctx.setLineDash([])',
]) {
  if (!common.includes(token)) {
    throw new Error(`2D global-plan contract is missing: ${token}`);
  }
}

for (const token of [
  'setGlobalPath(points)',
  'clearGlobalPath()',
  'this.pathBuffer',
  'gl.TRIANGLE_STRIP',
]) {
  if (!viewerSource.includes(token)) {
    throw new Error(`3D global-plan contract is missing: ${token}`);
  }
}

for (const token of [
  'id="global-plan-status"',
  "activeSession.subscribe('/navigation/global_path', UI.TYPES.path",
  "activeSession.subscribe('/navigation/global_path/status', UI.TYPES.string",
  'mapCanvas.setGlobalPath(points)',
  'navigationPointCloudViewer.setGlobalPath(points)',
  'function clearGlobalPlan(',
  'mapCanvas.clearGlobalPath()',
  'navigationPointCloudViewer.clearGlobalPath()',
  'if (!globalPlanExpected) return;',
  "setGlobalPlanStatus('当前没有执行中的全局规划', 'waiting')",
  "clearGlobalPlan('ROS2 已断开')",
  "clearNavigationGlobalCloud('ROS2 已断开，等待重连后的当前地图')",
  "clearGlobalPlan('地图选择已改变，等待加载后重新规划')",
  "clearNavigationGlobalCloud('等待所选地图的全局三维点云')",
  "['pause', 'skip', 'stop'].includes(command)",
  'if (!navigationPointCloudViewer) return;',
]) {
  if (!navigation.includes(token)) {
    throw new Error(`navigation global-plan lifecycle is missing: ${token}`);
  }
}

if (navigation.includes('if (status.navigation_active === true) globalPlanExpected = true;')) {
  throw new Error('broad navigation_active status must not revive a paused stale path');
}

if (/activeSession\.subscribe\(\s*['"]\/plan['"]/.test(navigation)) {
  throw new Error('navigation page must consume the terrain-elevated bridge, not raw /plan');
}
if (/activeSession\.subscribe\(\s*['"]\/terrain_grid['"]/.test(navigation)) {
  throw new Error('navigation page must not send the full terrain grid through rosbridge');
}
if (!navigation.includes("'goal_succeeded', 'goal_canceled', 'goal_aborted'")) {
  throw new Error('terminal Nav2 states must clear the displayed global plan');
}
if (!navigation.includes("'dwelling', 'pausing', 'paused_navigation', 'paused_dwell'")) {
  throw new Error('paused/dwelling route states must clear the displayed global plan');
}

function verify2dPathStyle() {
  const browser = {
    location: { hostname: '127.0.0.1', protocol: 'http:', port: '8000' },
    setTimeout: () => 0, clearTimeout: () => {}, addEventListener: () => {},
  };
  vm.runInNewContext(common, {
    window: browser,
    document: { createElement: () => ({ textContent: '', innerHTML: '' }) },
    console,
  });
  const canvas = Object.create(browser.RobotUi.MapCanvas.prototype);
  canvas.mapInfo = {
    width: 20, height: 20, resolution: 1,
    origin: { position: { x: 0, y: 0 } },
  };
  canvas.zoom = 10;
  canvas.offsetX = 0;
  canvas.offsetY = 0;
  const strokes = [];
  let dash = null;
  const ctx = {
    strokeStyle: '', lineWidth: 0, lineCap: '', lineJoin: '',
    save: () => {}, restore: () => {}, beginPath: () => {},
    moveTo: () => {}, lineTo: () => {},
    setLineDash: (value) => { dash = Array.from(value); },
    stroke() { strokes.push({ color: this.strokeStyle, width: this.lineWidth, dash }); },
  };
  canvas.setGlobalPath([
    { x: 1, y: 1, z: 0.1 },
    { x: 5, y: 4, z: 0.5 },
  ]);
  canvas.drawGlobalPath(ctx);
  if (!strokes.some((entry) => entry.color === '#45e39b'
      && entry.width === 3.4 && entry.dash.length === 0)) {
    throw new Error('2D global path is not a solid green planner line');
  }
  strokes.length = 0;
  canvas.setGlobalPath([
    { x: 1, y: 1, z: 0.1 },
    { x: 5, y: 4, z: 0.5 },
  ], { warning: true });
  canvas.drawGlobalPath(ctx);
  if (!strokes.some((entry) => entry.color === '#ffb454'
      && entry.width === 3.4 && entry.dash.length === 0)) {
    throw new Error('2D fallback path is not a solid amber warning line');
  }
}

function verify3dPathLifecycle() {
  const drawCalls = [];
  const uploads = new Map();
  let nextObjectId = 1;
  let boundBuffer = null;
  const gl = {
    VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, COMPILE_STATUS: 3, LINK_STATUS: 4,
    DEPTH_TEST: 5, LEQUAL: 6, ARRAY_BUFFER: 7, DYNAMIC_DRAW: 8,
    COLOR_BUFFER_BIT: 16, DEPTH_BUFFER_BIT: 32, FLOAT: 9,
    POINTS: 10, TRIANGLES: 11, TRIANGLE_STRIP: 12,
    createShader: () => ({ id: nextObjectId++ }), shaderSource: () => {},
    compileShader: () => {}, getShaderParameter: () => true,
    getShaderInfoLog: () => '', createProgram: () => ({ id: nextObjectId++ }),
    attachShader: () => {}, linkProgram: () => {}, getProgramParameter: () => true,
    getProgramInfoLog: () => '', createBuffer: () => ({ id: nextObjectId++ }),
    enable: () => {}, disable: () => {}, depthFunc: () => {},
    bindBuffer: (_target, buffer) => { boundBuffer = buffer; },
    bufferData: (_target, data) => {
      uploads.set(boundBuffer, data && typeof data.length === 'number'
        ? new Float32Array(Array.from(data)) : data);
    },
    viewport: () => {}, clearColor: () => {}, clear: () => {},
    useProgram: () => {}, getAttribLocation: (_program, name) => name === 'aPosition' ? 0 : 1,
    enableVertexAttribArray: () => {}, vertexAttribPointer: () => {},
    getUniformLocation: () => ({}), uniform1f: () => {}, uniform2f: () => {},
    lineWidth: () => {},
    drawArrays: (mode, first, count) => drawCalls.push({ mode, first, count }),
  };
  const container = { getBoundingClientRect: () => ({ width: 800, height: 600 }) };
  const canvas = {
    hidden: false, width: 0, height: 0, parentElement: container,
    classList: { add: () => {}, remove: () => {} },
    addEventListener: () => {}, getContext: () => gl,
  };
  const browser = { devicePixelRatio: 1, addEventListener: () => {}, atob: global.atob };
  vm.runInNewContext(viewerSource, { window: browser, console });
  const bytes = new ArrayBuffer(24);
  const view = new DataView(bytes);
  [[0, 0, 0], [2, 2, 1]].forEach((point, index) => {
    point.forEach((value, axis) => view.setFloat32(index * 12 + axis * 4, value, true));
  });
  const cloud = {
    width: 2, height: 1, point_step: 12, row_step: 24, is_bigendian: false,
    fields: [
      { name: 'x', offset: 0, datatype: 7 },
      { name: 'y', offset: 4, datatype: 7 },
      { name: 'z', offset: 8, datatype: 7 },
    ],
    data: Array.from(new Uint8Array(bytes)),
  };
  const instance = new browser.PointCloudViewer(canvas, container, { maxPoints: 1000 });
  instance.setPointCloud(cloud);
  drawCalls.length = 0;
  instance.setGlobalPath([
    { x: 0, y: 0, z: 0.08 },
    { x: 1, y: 1, z: 0.40 },
    { x: 2, y: 2, z: 1.08 },
  ]);
  if (!drawCalls.some((call) => call.mode === gl.TRIANGLE_STRIP && call.count === 6)) {
    throw new Error('3D viewer did not draw the terrain-elevated global path');
  }
  const elevatedVertices = uploads.get(instance.pathBuffer);
  if (!(elevatedVertices instanceof Float32Array)
      || !(Math.abs(elevatedVertices[2] - elevatedVertices[26]) > 0.1)) {
    throw new Error('3D path ribbon flattened the backend-provided z values');
  }
  const expectedFirstZ = (0.08 - instance.cloudTransform.centerZ)
    / instance.cloudTransform.radius;
  if (Math.abs(elevatedVertices[2] - expectedFirstZ) > 1e-5) {
    throw new Error('3D viewer lifted the backend-elevated path a second time');
  }

  drawCalls.length = 0;
  instance.setGlobalPath([
    { x: 0, y: 0, z: 0.08 },
    { x: 2, y: 2, z: 1.08 },
  ], { warning: true });
  if (!drawCalls.some((call) => call.mode === gl.TRIANGLE_STRIP && call.count === 4)) {
    throw new Error('3D viewer appended a replacement path instead of replacing it');
  }
  const warningVertices = uploads.get(instance.pathBuffer);
  if (!warningVertices || Math.abs(warningVertices[3] - 1) > 1e-6
      || Math.abs(warningVertices[4] - 0.56) > 1e-6) {
    throw new Error('3D fallback path did not use the amber warning ribbon');
  }

  drawCalls.length = 0;
  instance.clearGlobalPath();
  if (drawCalls.some((call) => call.mode === gl.TRIANGLE_STRIP)) {
    throw new Error('3D viewer kept a stale global path after clearing');
  }

  // A latched path may be delivered before the large fragmented PCD. Keep it
  // in world coordinates and build the ribbon as soon as the cloud transform
  // becomes available.
  const lateCloudInstance = new browser.PointCloudViewer(canvas, container, { maxPoints: 1000 });
  drawCalls.length = 0;
  lateCloudInstance.setGlobalPath([
    { x: 0, y: 0, z: 0.08 },
    { x: 2, y: 2, z: 1.08 },
  ]);
  if (drawCalls.some((call) => call.mode === gl.TRIANGLE_STRIP)) {
    throw new Error('3D viewer drew a path before it had a cloud transform');
  }
  lateCloudInstance.setPointCloud(cloud);
  if (!drawCalls.some((call) => call.mode === gl.TRIANGLE_STRIP && call.count === 4)) {
    throw new Error('3D viewer lost a path that arrived before the point cloud');
  }
}

verify2dPathStyle();
verify3dPathLifecycle();
console.log('2D/3D terrain global-plan frontend contract: ok');
