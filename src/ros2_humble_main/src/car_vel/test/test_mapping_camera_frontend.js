#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const html = fs.readFileSync(path.join(frontend, 'mapping.html'), 'utf8');
const css = fs.readFileSync(path.join(frontend, 'robot_ui.css'), 'utf8');
const elementIds = [...html.matchAll(/\sid="([^"]+)"/g)].map((match) => match[1]);
const duplicateIds = [...new Set(elementIds.filter(
  (id, index) => elementIds.indexOf(id) !== index))];
if (duplicateIds.length) {
  throw new Error(`mapping page has duplicate element IDs: ${duplicateIds.join(', ')}`);
}

for (const required of [
  'id="camera-toggle"',
  'id="camera-display"',
  'id="mapping-camera-panel"',
  'id="mapping-camera-stream"',
  'id="mapping-camera-status"',
  'id="btn-camera-close"',
  'aria-pressed="false"',
  'alt="USB相机实时画面"',
]) {
  if (!html.includes(required)) {
    throw new Error(`mapping camera UI contract missing: ${required}`);
  }
}

for (const required of [
  "'/api/stream'",
  'function setCameraEnabled(',
  'function connectCameraStream(',
  'function suspendCameraStream(',
  "removeAttribute('src')",
  'document.hidden',
  "host.includes(':')",
  "window.addEventListener('pagehide'",
  "window.addEventListener('pageshow'",
  'window.clearTimeout(cameraRetryTimer)',
]) {
  if (!html.includes(required)) {
    throw new Error(`mapping camera lifecycle contract missing: ${required}`);
  }
}
if (html.includes('/api/video')) {
  throw new Error('mapping page must not restart the shared camera capture service');
}
if (!/id="camera-toggle"[\s\S]*id="mapping-camera-panel"/.test(html)) {
  throw new Error('mapping camera preview must expand inside its sidebar section');
}

if (!/<img\s+id="mapping-camera-stream"[^>]*alt="USB相机实时画面"(?![^>]*\ssrc=)[^>]*>/s.test(html)) {
  throw new Error('mapping camera must be disabled by default without an image src');
}
if (!/<div\s+id="mapping-camera-panel"[^>]*hidden/s.test(html)) {
  throw new Error('mapping camera panel must be hidden by default');
}

for (const required of [
  '.mapping-camera-panel',
  '.mapping-camera-frame',
  '.mapping-camera-stream',
  '.mapping-camera-status.error',
  '@media (max-width: 520px)',
]) {
  if (!css.includes(required)) {
    throw new Error(`mapping camera responsive style missing: ${required}`);
  }
}
if (!/@media \(max-width: 520px\)[\s\S]*\.mapping-camera-header/.test(css)) {
  throw new Error('mapping camera needs a narrow-screen header adjustment');
}

const inlineScripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)];
if (!inlineScripts.length) throw new Error('mapping page controller is missing');
inlineScripts.forEach((match) => new Function(match[1])); // eslint-disable-line no-new-func

console.log('mapping camera toggle, cleanup and responsive contracts: ok');
