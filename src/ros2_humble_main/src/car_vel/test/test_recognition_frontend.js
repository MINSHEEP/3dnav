#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const root = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const pagePath = path.join(root, 'recognition_records.html');
assert.ok(fs.existsSync(pagePath), 'dedicated recognition management page is required');

const page = fs.readFileSync(pagePath, 'utf8');
const css = fs.readFileSync(path.join(root, 'recognition_records.css'), 'utf8');
const elementIds = [...page.matchAll(/\sid="([^"]+)"/g)].map((match) => match[1]);
const duplicateIds = [...new Set(elementIds.filter((id, index) => elementIds.indexOf(id) !== index))];
assert.deepStrictEqual(duplicateIds, [], `recognition page has duplicate element IDs: ${duplicateIds.join(', ')}`);
const inlineScripts = [...page.matchAll(/<script>([\s\S]*?)<\/script>/g)];
assert.ok(inlineScripts.length, 'recognition page must contain its controller');
inlineScripts.forEach((match) => new Function(match[1])); // eslint-disable-line no-new-func
assert.ok(page.includes('/api/recognition'), 'recognition API base is missing');
[
  '/status', '/people', '/vehicles', '/events', '/audit',
  '/person/upsert', '/person/delete', '/vehicle/upsert', '/vehicle/delete',
  '/event/review', '/cleanup',
].forEach((endpoint) => assert.ok(page.includes(endpoint), `${endpoint} UI contract missing`));
assert.ok(page.includes('sample_images_base64'), 'person registration must upload face samples');
assert.ok(page.includes('registered') && page.includes('unregistered') && page.includes('uncertain'));
assert.ok(page.includes('yellow_black') && page.includes('white_black')
  && page.includes('blue_white') && page.includes('green_black'));

for (const navigationToken of [
  'href="map_editor.html">修图',
  'href="navigation.html">地形导航',
  'href="campus_navigation.html">园区导航',
  'class="topbar-statuses"',
]) {
  assert.ok(page.includes(navigationToken), `shared navigation contract missing: ${navigationToken}`);
}
assert.ok(page.includes('class="warning"'), 'retention cleanup must use the shared warning action style');
assert.ok(page.includes('class="danger compact"'), 'destructive row actions need explicit danger styling');
assert.ok(page.includes('function recognitionStatusLabel('), 'event states need localized labels');
assert.ok(page.includes('function plateColorLabel('), 'plate colors need localized labels');
assert.ok(page.includes('role="status"'), 'operation feedback needs status semantics');

for (const cssToken of [
  'background: var(--bg)',
  'background: var(--panel)',
  'background: var(--panel-2)',
  'color: var(--text)',
  '@media (max-width: 720px)',
  '@media (max-width: 480px)',
]) {
  assert.ok(css.includes(cssToken), `shared visual token missing: ${cssToken}`);
}
assert.ok(!css.includes('background: #f3f6f8'), 'recognition page must not force a light-only background');
assert.ok(!css.includes('background: #fff'), 'recognition panels must use the shared dark theme');

const inspection = fs.readFileSync(path.join(root, 'inspection.html'), 'utf8');
assert.ok(inspection.includes('recognition_records.html'), 'inspection page must link recognition records');
assert.ok(inspection.includes('id="recognition-latest"'), 'inspection page must show latest recognition');
assert.ok(
  /api\/recognition\/events[\s\S]*task_id[\s\S]*limit[\s\S]*5/.test(inspection),
  'inspection page must request the latest five records for the current task');

const reports = fs.readFileSync(path.join(root, 'inspection_reports.html'), 'utf8');
assert.ok(reports.includes('/api/recognition/summary'), 'report must load recognition summary');
assert.ok(reports.includes('识别记录（不影响任务异常结论）'));
assert.ok(reports.includes('recognition_records.html?task_id='));

console.log('recognition management, latest-five and report summary contracts: ok');
