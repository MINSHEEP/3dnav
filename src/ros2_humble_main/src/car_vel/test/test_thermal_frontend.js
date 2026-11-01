#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const webRoot = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');

function read(name) {
  return fs.readFileSync(path.join(webRoot, name), 'utf8');
}

function validateInlineScript(name, html) {
  const start = html.lastIndexOf('<script>');
  const end = html.indexOf('</script>', start);
  if (start < 0 || end < 0) throw new Error(`${name} has no inline script`);
  new Function(html.slice(start + '<script>'.length, end));
  const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
  const duplicates = ids.filter((id, index) => ids.indexOf(id) !== index);
  if (duplicates.length) {
    throw new Error(`${name} duplicate ids: ${[...new Set(duplicates)].join(', ')}`);
  }
}

const camera = read('camera.html');
const remote = read('remote_control.html');
const inspection = read('inspection.html');
const reports = read('inspection_reports.html');
const commonCss = read('robot_ui.css');
const inspectionCss = read('inspection.css');

for (const [name, html] of [
  ['camera.html', camera], ['remote_control.html', remote],
  ['inspection.html', inspection], ['inspection_reports.html', reports],
]) validateInlineScript(name, html);

for (const [name, html] of [
  ['camera.html', camera], ['remote_control.html', remote],
  ['inspection.html', inspection],
]) {
  if (!html.includes('/api/thermal/stream.mjpg')) {
    throw new Error(`${name} does not consume the shared thermal stream`);
  }
}

if (!camera.includes('启动识别') || !camera.includes('查看识别画面')) {
  throw new Error('camera page does not expose the renamed recognition controls');
}
if (!camera.includes("post('/api/detection'") || !camera.includes("scope:'global'")) {
  throw new Error('camera page does not coordinate recognition and global threshold');
}
if (!camera.includes('thermalBecameReady')
    || !camera.includes('if (thermalBecameReady) connectThermalFeed(0)')) {
  throw new Error('camera page does not reconnect the thermal stream after backend recovery');
}
if (!camera.includes("viewMode === 'yolo' ? '/api/yolo/stream.mjpg' : '/api/stream'")) {
  throw new Error('view recognition no longer preserves the raw/YOLO visible-camera toggle');
}
if (!remote.includes('thermal-camera') && !remote.includes('thermal-stream')) {
  throw new Error('remote page has no independent thermal image');
}
if (!remote.includes("const VIDEO_URL = cameraApiUrl('/api/stream')")
    || remote.includes('8081/camera')) {
  throw new Error('remote visible camera does not use the local USB camera gateway');
}
if (!remote.includes("const THERMAL_VIDEO_URL = cameraApiUrl('/api/thermal/stream.mjpg')")) {
  throw new Error('remote thermal camera is not isolated on the camera gateway');
}
if (!remote.includes('普通摄像头和遥控不受影响')) {
  throw new Error('remote page does not communicate thermal partial failure');
}
if (!inspection.includes("scope:activeTask ? 'task' : 'global'")) {
  throw new Error('inspection page has no task-scoped threshold override');
}
if (!inspection.includes('新阈值只影响后续温度帧')) {
  throw new Error('inspection page does not confirm an in-flight threshold change');
}
if (!inspection.includes("alert.measurement_type === 'temperature'")
    || !inspection.includes('alert.thermal_annotated_image')) {
  throw new Error('inspection alert list does not render typed temperature evidence');
}
if (!reports.includes("alert.measurement_type === 'temperature'")
    || !reports.includes('threshold.toFixed(1)')
    || !reports.includes('item.thermal_annotated_image')) {
  throw new Error('inspection report does not render temperature measurements and thermal evidence');
}
if (!commonCss.includes('.remote-video-grid')
    || !inspectionCss.includes('.inspection-camera-grid')) {
  throw new Error('thermal layouts are missing responsive grids');
}
if (!commonCss.includes('object-fit: contain')
    || !inspectionCss.includes('object-fit: contain')) {
  throw new Error('camera images may be stretched instead of contained');
}

console.log('thermal frontend dual-view, threshold and lifecycle contracts: ok');
