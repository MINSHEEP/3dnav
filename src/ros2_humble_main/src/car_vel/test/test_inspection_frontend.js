#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const htmlPath = path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'inspection.html');
const html = fs.readFileSync(htmlPath, 'utf8');
const commonJs = fs.readFileSync(path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'robot_ui_common.js'), 'utf8');
const scriptStart = html.lastIndexOf('<script>');
const scriptEnd = html.indexOf('</script>', scriptStart);

if (scriptStart < 0 || scriptEnd < 0) {
  throw new Error('inspection.html is missing its inline script');
}

// Compilation catches syntax errors without starting ROS bridge or a browser.
new Function(html.slice(scriptStart + '<script>'.length, scriptEnd));

const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
const duplicates = ids.filter((id, index) => ids.indexOf(id) !== index);
if (duplicates.length) {
  throw new Error(`duplicate element ids: ${[...new Set(duplicates)].join(', ')}`);
}

const usedTypes = [...html.matchAll(/UI\.TYPES\.([A-Za-z0-9_]+)/g)]
  .map(match => match[1]);
const missingTypes = [...new Set(usedTypes)]
  .filter(key => !new RegExp(`\\b${key}\\s*:`).test(commonJs));
if (missingTypes.length) {
  throw new Error(`missing RobotUi service types: ${missingTypes.join(', ')}`);
}

if (!html.includes("'/dog_remote/ensure_standing'")) {
  throw new Error('inspection.html does not request standing confirmation');
}
if (!html.includes('UI.TYPES.trigger')) {
  throw new Error('inspection.html does not use the Trigger service type');
}
if (!html.includes("'/control_launch'")
    || !html.includes("launch_type: 'car_vel'")) {
  throw new Error('inspection.html does not start the navigation stack');
}
if (!html.includes("subscribe('/nav2_ready'")) {
  throw new Error('inspection.html does not subscribe to Nav2 readiness');
}
if (!/\blet nav2Ready\s*=\s*false/.test(html)
    || !/\blet localizationReady\s*=\s*false/.test(html)) {
  throw new Error('inspection.html is missing Nav2/localization readiness state');
}
if (!html.includes('!nav2Ready || !localizationReady')) {
  throw new Error('inspection start button is not guarded by Nav2 and localization readiness');
}
if (!html.includes("const SKIPPABLE_TASK_STATES = new Set(['navigating', 'inspecting'])")) {
  throw new Error('inspection skip control is not restricted to a current route point');
}

const pageNames = [
  'mapping.html', 'map_editor.html', 'navigation.html',
  'campus_navigation.html', 'inspection.html', 'inspection_reports.html',
  'remote_control.html', 'camera.html',
];
for (const pageName of pageNames) {
  const page = fs.readFileSync(path.resolve(
    __dirname, '..', 'car_vel', 'web_frontend', pageName), 'utf8');
  if (pageName !== 'inspection.html' && !page.includes('href="inspection.html"')) {
    throw new Error(`${pageName} is missing the inspection navigation link`);
  }
  if (pageName !== 'campus_navigation.html'
      && !page.includes('href="campus_navigation.html"')) {
    throw new Error(`${pageName} is missing the campus navigation link`);
  }
  if (!page.includes('<div class="brand">四足机器人巡检</div>')) {
    throw new Error(`${pageName} is missing the unified inspection brand`);
  }
  if (page.includes('四足机器人导航')) {
    throw new Error(`${pageName} still contains the old navigation brand`);
  }
}

for (const field of [
  'z', 'area_id', 'floor_id', 'terrain_type', 'traversal_mode',
  'transition_id', 'speed_limit_mps',
]) {
  if (!new RegExp(`\\b${field}\\s*:`).test(html)) {
    throw new Error(`inspection route adapter is missing ${field}`);
  }
}

const brandAssetPath = path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'guochuang-brand.png');
if (!fs.existsSync(brandAssetPath) || fs.statSync(brandAssetPath).size === 0) {
  throw new Error('the PDF-derived Guochuang brand asset is missing');
}

console.log('inspection frontend syntax, standing guard and branding: ok');
