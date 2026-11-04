#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const viewer = fs.readFileSync(path.join(frontend, 'pointcloud_viewer.js'), 'utf8');
const mapping = fs.readFileSync(path.join(frontend, 'mapping.html'), 'utf8');
const navigation = fs.readFileSync(path.join(frontend, 'navigation.html'), 'utf8');

for (const required of [
  'setRobotPose(x, y, z, yaw)',
  'clearRobotPose()',
  'this.robotPose',
  'this.cloudTransform',
  'gl.TRIANGLES',
]) {
  if (!viewer.includes(required)) {
    throw new Error(`3D viewer is missing robot-pose rendering contract: ${required}`);
  }
}

if (!/pointCloudViewer\.setRobotPose\(x, y, z, yaw\)/.test(mapping)) {
  throw new Error('mapping 3D view does not receive the odom-frame robot pose');
}

if (!/function setNavigationRobotPose\(x, y, z, yaw\)/.test(navigation)
    || !/navigationPointCloudViewer\.setRobotPose\(x, y, safeZ, yaw\)/.test(navigation)
    || !/setNavigationRobotPose\(\s*pose\.position\.x, pose\.position\.y, pose\.position\.z, yaw\)/.test(navigation)) {
  throw new Error('navigation 3D view does not receive the map-frame AMCL robot pose');
}
for (const [pageName, page] of [['mapping', mapping], ['navigation', navigation]]) {
  const version=pageName==='mapping'?'20260820-global-path':'20261023-draft-marker-scale';
  if (!page.includes('pointcloud_viewer.js?v='+version)) {
    throw new Error(`${pageName} page can reuse a cached 3D viewer without the robot marker`);
  }
}
if (!mapping.includes('{ maxPoints: 120000, markerScale: 0.25 }')) {
  throw new Error('mapping page does not use the requested smaller 3D robot marker');
}
if (!navigation.includes('{ maxPoints: 120000, fullResolution: true, markerScale: 0.25 }')) {
  throw new Error('terrain navigation must use the smaller robot marker without reducing saved-map points');
}

function verifyMarkerDrawLifecycle() {
  const drawCalls = [];
  let lastBuffer = null;
  let nextObjectId = 1;
  const gl = {
    VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, COMPILE_STATUS: 3, LINK_STATUS: 4,
    DEPTH_TEST: 5, LEQUAL: 6, ARRAY_BUFFER: 7, DYNAMIC_DRAW: 8,
    COLOR_BUFFER_BIT: 16, DEPTH_BUFFER_BIT: 32, FLOAT: 9,
    POINTS: 10, TRIANGLES: 11,
    createShader: () => ({ id: nextObjectId++ }), shaderSource: () => {},
    compileShader: () => {}, getShaderParameter: () => true,
    getShaderInfoLog: () => '', createProgram: () => ({ id: nextObjectId++ }),
    attachShader: () => {}, linkProgram: () => {}, getProgramParameter: () => true,
    getProgramInfoLog: () => '', createBuffer: () => ({ id: nextObjectId++ }),
    enable: () => {}, disable: () => {}, depthFunc: () => {}, bindBuffer: () => {},
    bufferData: (_target, data) => { lastBuffer = data; }, viewport: () => {}, clearColor: () => {}, clear: () => {},
    useProgram: () => {}, getAttribLocation: (_program, name) => name === 'aPosition' ? 0 : 1,
    enableVertexAttribArray: () => {}, vertexAttribPointer: () => {},
    getUniformLocation: () => ({}), uniform1f: () => {}, uniform2f: () => {},
    drawArrays: (mode, first, count) => drawCalls.push({ mode, first, count }),
  };
  const container = { getBoundingClientRect: () => ({ width: 800, height: 600 }) };
  const canvas = {
    hidden: false, width: 0, height: 0, parentElement: container,
    classList: { add: () => {}, remove: () => {} },
    addEventListener: () => {}, getContext: () => gl,
  };
  const browser = {
    devicePixelRatio: 1, addEventListener: () => {}, atob: global.atob,
  };
  vm.runInNewContext(viewer, { window: browser, console });
  const pointBytes = new ArrayBuffer(24);
  const pointView = new DataView(pointBytes);
  [[0, 0, 0], [2, 1, 0.4]].forEach((point, index) => {
    point.forEach((value, axis) => pointView.setFloat32(index * 12 + axis * 4, value, true));
  });
  const pointCloud = {
    width: 2, height: 1, point_step: 12, row_step: 24, is_bigendian: false,
    fields: [
      { name: 'x', offset: 0, datatype: 7 },
      { name: 'y', offset: 4, datatype: 7 },
      { name: 'z', offset: 8, datatype: 7 },
    ],
    data: Array.from(new Uint8Array(pointBytes)),
  };
  const instance = new browser.PointCloudViewer(canvas, container, { maxPoints: 1000 });
  instance.setPointCloud(pointCloud);
  drawCalls.length = 0;
  instance.setRobotPose(1, 0.5, 0.2, Math.PI / 2);
  if (!drawCalls.some((call) => call.mode === gl.TRIANGLES && call.count === 3)) {
    throw new Error('3D viewer did not draw the directional robot triangle');
  }
  instance._updateRobotBuffer();
  const fullLength = lastBuffer[1] - lastBuffer[7];
  instance.markerScale = 0.25;
  instance._updateRobotBuffer();
  const smallLength = lastBuffer[1] - lastBuffer[7];
  if (Math.abs(smallLength / fullLength - 0.25) > 1e-5) {
    throw new Error('small navigation marker must be one quarter of its original length');
  }
  instance.setDraftPose(1, 0.5, 0.2, Math.PI / 2);
  instance._updateDraftBuffer();
  const draftLength = lastBuffer[1] - lastBuffer[7];
  if (Math.abs(draftLength - smallLength) > 1e-6) {
    throw new Error('initial/goal draft arrow must match the confirmed small robot arrow');
  }
  instance.clearDraftPose();
  drawCalls.length = 0;
  instance.clearRobotPose();
  if (drawCalls.some((call) => call.mode === gl.TRIANGLES)) {
    throw new Error('3D viewer kept drawing a stale robot marker after reset');
  }
}

verifyMarkerDrawLifecycle();

console.log('3D mapping/navigation robot-pose marker contract: ok');
