#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const html = fs.readFileSync(path.join(frontend, 'campus_navigation.html'), 'utf8');
const css = fs.readFileSync(path.join(frontend, 'campus_navigation.css'), 'utf8');
const common = fs.readFileSync(path.join(frontend, 'robot_ui_common.js'), 'utf8');
const scriptStart = html.lastIndexOf('<script>');
const scriptEnd = html.indexOf('</script>', scriptStart);

if (scriptStart < 0 || scriptEnd < 0) {
  throw new Error('campus_navigation.html is missing its inline script');
}
new Function(html.slice(scriptStart + '<script>'.length, scriptEnd));

const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
const duplicates = ids.filter((id, index) => ids.indexOf(id) !== index);
if (duplicates.length) {
  throw new Error(`duplicate element ids: ${[...new Set(duplicates)].join(', ')}`);
}

for (const type of [
  'manageCampusTopology', 'planCampusRoute', 'evaluateTerrain',
  'startCampusMission', 'controlCampusMission',
]) {
  if (!html.includes(`UI.TYPES.${type}`) || !new RegExp(`\\b${type}\\s*:`).test(common)) {
    throw new Error(`campus page is missing service type ${type}`);
  }
}

for (const selector of [
  'source-area', 'target-area', 'terrain-type', 'btn-plan', 'btn-evaluate',
  'btn-execute-plan', 'btn-stop-plan', 'mission-status',
  'topology-view', 'perception-indicator',
]) {
  if (!ids.includes(selector)) throw new Error(`campus page is missing #${selector}`);
}

if (!html.includes('当前不是实时点云画面')
    || !html.includes('实时地形 / 可通行性感知')) {
  throw new Error('campus page must distinguish configured topology from live perception');
}
if (!html.includes('segments.every((segment) => segment.executable)')
    || !html.includes("'/campus_mission/status'")) {
  throw new Error('campus executor must require calibrated endpoints and expose mission status');
}
if (!css.includes('@media (max-width: 1080px)')
    || !css.includes('@media (max-width: 620px)')) {
  throw new Error('campus page is missing responsive layouts');
}

console.log('campus navigation frontend syntax, safety labels and responsiveness: ok');
