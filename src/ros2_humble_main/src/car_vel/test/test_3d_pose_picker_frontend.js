#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const source = fs.readFileSync(path.join(
  __dirname, '..', 'car_vel', 'web_frontend', 'pointcloud_viewer.js'), 'utf8');

function viewerFixture() {
  const canvasEvents = {};
  const windowEvents = {};
  const drawCalls = [];
  let nextId = 0;
  const gl = {
    VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, COMPILE_STATUS: 3, LINK_STATUS: 4,
    DEPTH_TEST: 5, LEQUAL: 6, ARRAY_BUFFER: 7, DYNAMIC_DRAW: 8,
    COLOR_BUFFER_BIT: 16, DEPTH_BUFFER_BIT: 32, FLOAT: 9,
    POINTS: 10, TRIANGLES: 11, TRIANGLE_STRIP: 12,
    createShader: () => ++nextId, shaderSource: () => {}, compileShader: () => {},
    getShaderParameter: () => true, getShaderInfoLog: () => '',
    createProgram: () => ++nextId, attachShader: () => {}, linkProgram: () => {},
    getProgramParameter: () => true, getProgramInfoLog: () => '',
    createBuffer: () => ++nextId, enable: () => {}, disable: () => {},
    depthFunc: () => {}, bindBuffer: () => {}, bufferData: () => {},
    viewport: () => {}, clearColor: () => {}, clear: () => {},
    useProgram: () => {}, getAttribLocation: () => 0,
    enableVertexAttribArray: () => {}, vertexAttribPointer: () => {},
    getUniformLocation: () => 0, uniform1f: () => {}, uniform2f: () => {},
    drawArrays: (mode, _first, count) => drawCalls.push({ mode, count }),
  };
  const container = { getBoundingClientRect: () => ({ width: 800, height: 600 }) };
  const canvas = {
    hidden: false, width: 0, height: 0, parentElement: container,
    classList: { add: () => {}, remove: () => {} },
    addEventListener(name, callback) { canvasEvents[name] = callback; },
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 600 }),
    getContext: () => gl,
  };
  const browser = {
    devicePixelRatio: 1, requestAnimationFrame: callback => callback(), atob: global.atob,
    addEventListener(name, callback) { windowEvents[name] = callback; },
  };
  vm.runInNewContext(source, { window: browser, console });
  const viewer = new browser.PointCloudViewer(canvas, container);
  const bytes = new ArrayBuffer(48);
  const values = new DataView(bytes);
  [[-2, -2, 0], [2, -2, 0], [-2, 2, 0], [2, 2, 0]].forEach((point, index) => {
    point.forEach((value, axis) => values.setFloat32(index * 12 + axis * 4, value, true));
  });
  viewer.setPointCloud({
    width: 4, height: 1, point_step: 12, row_step: 48, is_bigendian: false,
    fields: [
      { name: 'x', offset: 0, datatype: 7 },
      { name: 'y', offset: 4, datatype: 7 },
      { name: 'z', offset: 8, datatype: 7 },
    ],
    data: Array.from(new Uint8Array(bytes)),
  });
  return { viewer, canvasEvents, windowEvents, drawCalls, gl };
}

function drag(fixture, start, end) {
  fixture.canvasEvents.mousedown({ button: 0, clientX: start.x, clientY: start.y });
  fixture.windowEvents.mousemove({ clientX: end.x, clientY: end.y });
  fixture.windowEvents.mouseup({ button: 0, clientX: end.x, clientY: end.y });
}

const fixture = viewerFixture();
const picks = [];
drag(fixture, { x: 400, y: 300 }, { x: 400, y: 250 });
assert.strictEqual(picks.length, 0, 'read-only point cloud must not select a pose');

fixture.viewer.setTopView();
fixture.viewer.setGroundPoseInteraction(true, (...pose) => picks.push(pose));
drag(fixture, { x: 400, y: 300 }, { x: 400, y: 250 });
assert.strictEqual(picks.length, 1, 'one top-view drag should select one pose');
assert(Math.abs(picks[0][0]) < 1e-6 && Math.abs(picks[0][1]) < 1e-6,
  'screen center must map to cloud center, not an arbitrary point height');
assert(Math.abs(picks[0][2] - Math.PI / 2) < 1e-6,
  'upward drag should face positive map Y');
drag(fixture, { x: 500, y: 300 }, { x: 500, y: 300 });
assert(Math.abs(picks[1][0] - 0.910569) < 1e-4
  && Math.abs(picks[1][1]) < 1e-6,
  'off-center pick must invert the WebGL projection into map coordinates');

fixture.viewer.resetView();
drag(fixture, { x: 400, y: 300 }, { x: 450, y: 300 });
assert.strictEqual(picks.length, 2,
  'tilted/rotated point cloud must not allow ambiguous ground picking');

fixture.viewer.setDraftPose(0.5, -0.5, 0, 0);
fixture.viewer.setPreviewPath([
  { x: 0, y: 0, z: 0.1 }, { x: 1, y: 1, z: 0.2 },
]);
fixture.drawCalls.length = 0;
fixture.viewer.draw();
assert(fixture.drawCalls.some((call) => call.mode === fixture.gl.TRIANGLES),
  'draft goal arrow should remain visible over point-cloud returns');
assert(fixture.drawCalls.some((call) => call.mode === fixture.gl.TRIANGLE_STRIP
  && call.count === 4), 'preview path must have its own visible overlay');
fixture.viewer.clearDraftPose();
fixture.viewer.clearPreviewPath();
fixture.drawCalls.length = 0;
fixture.viewer.draw();
assert(!fixture.drawCalls.some((call) => call.mode === fixture.gl.TRIANGLES),
  'invalidating a draft must remove its marker');
assert(!fixture.drawCalls.some((call) => call.mode === fixture.gl.TRIANGLE_STRIP),
  'invalidating a preview must remove its path');

console.log('3D top-view ground pose picker: ok');
