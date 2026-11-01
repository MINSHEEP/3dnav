#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const webRoot = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const pagePath = path.join(webRoot, 'inspection.html');
const draftModulePath = path.join(webRoot, 'inspection_draft_state.js');

assert.ok(
  fs.existsSync(draftModulePath),
  'inspection draft state module is required for page re-entry');

const { createInspectionDraftStore } = require(draftModulePath);

function memoryStorage() {
  const values = new Map();
  return {
    getItem(key) { return values.has(key) ? values.get(key) : null; },
    setItem(key, value) { values.set(key, String(value)); },
    removeItem(key) { values.delete(key); },
  };
}

// This is the reported sequence: configure an idle inspection page, leave it,
// then create a fresh page instance backed by the same tab session storage.
const storage = memoryStorage();
const firstPage = createInspectionDraftStore(storage);
firstPage.save({
  mapName: 'ceshi2',
  routeId: 17,
  taskName: '夜间巡检',
  loopCount: 2,
  loopInterval: 30,
  bidirectional: true,
  checkpoints: [
    { route_point_id: 8, directions: ['current'], zoom_ratio: null, save_images: true },
    { route_point_id: 3, directions: ['left', 'invalid'], zoom_ratio: 1.5, save_images: false },
  ],
});

const returnedPage = createInspectionDraftStore(storage);
assert.deepStrictEqual(returnedPage.load(), {
  mapName: 'ceshi2',
  routeId: 17,
  taskName: '夜间巡检',
  loopCount: 2,
  loopInterval: 30,
  bidirectional: true,
  checkpoints: [
    { route_point_id: 8, directions: ['current'], zoom_ratio: null, save_images: true },
    { route_point_id: 3, directions: ['left'], zoom_ratio: 1.5, save_images: false },
  ],
});

const malformed = firstPage.save({
  mapName: 'ceshi2', routeId: 17, checkpoints: [
    { route_point_id: 0 },
    { route_point_id: 4, directions: [], zoom_ratio: -10, save_images: 1 },
    { route_point_id: 4, directions: ['right'], zoom_ratio: 100 },
  ],
});
assert.deepStrictEqual(malformed.checkpoints, [
  { route_point_id: 4, directions: ['current'], zoom_ratio: null, save_images: true },
]);

returnedPage.clear();
assert.strictEqual(createInspectionDraftStore(storage).load(), null);

const html = fs.readFileSync(pagePath, 'utf8');
assert.ok(html.includes('inspection_draft_state.js'), 'inspection page must load the draft module');
assert.ok(html.includes('function saveInspectionDraft('), 'inspection page must save its idle setup');
assert.ok(html.includes('function restoreInspectionDraft('), 'inspection page must restore its idle setup');
assert.ok(
  /saveInspectionDraft\(\)[\s\S]*const checkpoints[\s\S]*checkpoints\s*\n?\s*}\);/.test(html),
  'idle draft must include unsaved route checkpoint configuration');
assert.ok(
  /function selectRoute\([\s\S]*inspectionDraft\.checkpoints/.test(html),
  'route restore must apply draft checkpoints after loading the persisted route template');
assert.ok(
  /if \(nav2Ready && !wasReady\)[\s\S]*restoreInspectionDraft/.test(html),
  'draft restoration must wait until Nav2 is ready');
assert.ok(
  /function releaseInspectionPage\(\)[\s\S]*saveInspectionDraft\(\)/.test(html),
  'leaving the page must save the current idle setup');
assert.ok(
  html.includes("addEventListener('beforeunload', releaseInspectionPage)")
    && html.includes("addEventListener('pagehide', releaseInspectionPage)"),
  'both unload and page-cache navigation must release the inspection page');
assert.ok(
  /function releaseInspectionPage\(\)[\s\S]*clearPendingInspectionMapLoad\(\)[\s\S]*cameraImage\.removeAttribute\('src'\)/.test(html),
  'page release must cancel pending map work and stop the MJPEG request');
assert.ok(
  /addEventListener\('pageshow',[\s\S]*event\.persisted[\s\S]*location\.reload/.test(html),
  'returning from the browser page cache must create a clean ROS/camera page instance');

console.log('inspection page leave/re-entry draft restoration: ok');
