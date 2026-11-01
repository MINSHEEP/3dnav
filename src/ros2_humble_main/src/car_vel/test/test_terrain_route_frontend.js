#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const htmlPath = path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'navigation.html');
const html = fs.readFileSync(htmlPath, 'utf8');
const scriptStart = html.lastIndexOf('<script>');
const scriptEnd = html.indexOf('</script>', scriptStart);

if (scriptStart < 0 || scriptEnd < 0) {
  throw new Error('navigation.html is missing its inline script');
}

// Compile the real inline page code without requiring ROS or a browser.
new Function(html.slice(scriptStart + '<script>'.length, scriptEnd));

for (const field of [
  'z', 'area_id', 'floor_id', 'terrain_type', 'traversal_mode',
  'transition_id', 'speed_limit_mps',
]) {
  if (!new RegExp(`\\b${field}\\s*:`).test(html)) {
    throw new Error(`navigation route payload is missing ${field}`);
  }
}

if (!html.includes("selectField('地形', 'terrain_type'")) {
  throw new Error('route editor terrain field is not a controlled selector');
}
if (!html.includes("selectField('执行模式', 'traversal_mode'")) {
  throw new Error('route editor traversal mode is not a controlled selector');
}

console.log('terrain-aware route frontend syntax and payload: ok');
