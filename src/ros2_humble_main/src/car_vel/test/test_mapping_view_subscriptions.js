#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const mapping = fs.readFileSync(
  path.resolve(__dirname, '..', 'car_vel', 'web_frontend', 'mapping.html'),
  'utf8',
);

const scriptStart = mapping.lastIndexOf('<script>');
const scriptEnd = mapping.indexOf('</script>', scriptStart);
new Function(mapping.slice(scriptStart + '<script>'.length, scriptEnd));

for (const token of [
  'function syncLargeMappingSubscription(',
  'function unsubscribeLargeMappingSubscription(',
  'activeLargeMappingTopic.unsubscribe()',
  "document.addEventListener('visibilitychange'",
  'if (document.hidden)',
]) {
  if (!mapping.includes(token)) {
    throw new Error(`view-aware mapping subscription contract is missing: ${token}`);
  }
}

for (const topic of [
  '/global_mapping_cloud',
  '/cloud_mapping_dynamic',
  '/octomap_point_cloud_centers',
  '/projected_map',
]) {
  const matches = mapping.match(new RegExp(`subscribe\\('${topic.replaceAll('/', '\\/')}'`, 'g')) || [];
  if (matches.length !== 1) {
    throw new Error(`${topic} must have exactly one view-aware subscription definition; got ${matches.length}`);
  }
}

if (!mapping.includes('if (mappingStarted && inputPoints > 0) mappingDirty = true;')) {
  throw new Error('small mapping-filter status must keep the unsaved 3D-map guard active');
}

if (!mapping.includes('地形/可通行性网格')) {
  throw new Error('map-save copy does not describe the persisted terrain artifact');
}

console.log('mapping view-aware large-topic subscription contract: ok');
