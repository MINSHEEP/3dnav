#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const carVel = path.resolve(__dirname, '..');
const navigation = fs.readFileSync(
  path.join(carVel, 'car_vel', 'web_frontend', 'navigation.html'), 'utf8');
const common = fs.readFileSync(
  path.join(carVel, 'car_vel', 'web_frontend', 'robot_ui_common.js'), 'utf8');
const backend = fs.readFileSync(
  path.resolve(carVel, '..', 'nav_service', 'lzp_service', 'lzp_service',
    'map_switcher_service.py'), 'utf8');

const scriptStart = navigation.lastIndexOf('<script>');
const scriptEnd = navigation.indexOf('</script>', scriptStart);
new Function(navigation.slice(scriptStart + '<script>'.length, scriptEnd));

for (const token of [
  'id="btn-delete-map"',
  '统一删除地图',
  '二维地图、三维 PCD、地形/可通行性网格、定位元数据及关联路线',
  'function deleteSelectedMap()',
  "session.call(\n                '/delete_map', UI.TYPES.deleteMap, { map_name: mapName })",
  "document.getElementById('btn-delete-map').addEventListener('click', deleteSelectedMap)",
  'selectedMapName === loadedMapName',
]) {
  if (!navigation.includes(token)) {
    throw new Error(`navigation page is missing unified map deletion contract: ${token}`);
  }
}

if (!/deleteMap\s*:\s*'lzp_interface\/srv\/DeleteMap'/.test(common)) {
  throw new Error('RobotUi does not expose the DeleteMap service type');
}
for (const token of [
  'self.delete_map_files(map_name)',
  'self.delete_pcd_files(map_name)',
  "f'{map_name}.terrain.npz'",
  'self.route_database.delete_routes_by_map(',
  'Cannot delete current map',
]) {
  if (!backend.includes(token)) {
    throw new Error(`delete-map backend is incomplete: ${token}`);
  }
}

console.log('Unified 2D/3D map and associated-route deletion frontend contract: ok');
