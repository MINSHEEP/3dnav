#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const navigation = fs.readFileSync(path.join(frontend, 'navigation.html'), 'utf8');
const inspection = fs.readFileSync(path.join(frontend, 'inspection.html'), 'utf8');
const rosbridge = fs.readFileSync(path.join(frontend, 'rosbridge_client.js'), 'utf8');

function compileInlineScript(source, pageName) {
  const start = source.lastIndexOf('<script>');
  const end = source.indexOf('</script>', start);
  if (start < 0 || end < 0) throw new Error(`${pageName} is missing its inline script`);
  new Function(source.slice(start + '<script>'.length, end));
}

compileInlineScript(navigation, 'navigation.html');
compileInlineScript(inspection, 'inspection.html');

function functionBody(source, name, nextName) {
  const start = source.indexOf(`function ${name}`);
  if (start < 0) throw new Error(`missing function ${name}`);
  const end = source.indexOf(`function ${nextName}`, start + 1);
  if (end < 0) throw new Error(`missing function after ${name}: ${nextName}`);
  return source.slice(start, end);
}

if (!rosbridge.includes('FRAGMENT_IDLE_TIMEOUT_MS = 30000')) {
  throw new Error('rosbridge fragments do not use the 30-second idle timeout');
}
if (!rosbridge.includes('this.armFragmentIdleTimeout(id, buffer)')) {
  throw new Error('rosbridge fragment progress does not renew its idle timeout');
}

for (const pageName of [
  'mapping.html', 'map_editor.html', 'navigation.html', 'inspection.html',
  'inspection_reports.html', 'remote_control.html', 'camera.html',
  'robot_control.html',
]) {
  const page = fs.readFileSync(path.join(frontend, pageName), 'utf8');
  if (!page.includes('rosbridge_client.js?v=20260804-map-idle-timeout')) {
    throw new Error(`${pageName} still references the cached rosbridge client`);
  }
}

for (const token of [
  'pendingMapStatusConfirmed',
  'pendingMapRasterReady',
  'tryCompletePendingMapLoad',
]) {
  if (!navigation.includes(token)) {
    throw new Error(`navigation map load gate is missing ${token}`);
  }
}
const navigationConfirmation = functionBody(
  navigation, 'confirmMapLoaded', 'handleRelocalizationStatus');
if (navigationConfirmation.includes('navigationMapReady = true')) {
  throw new Error('navigation marks the map ready from status before raster data');
}
const navigationCompletion = functionBody(
  navigation, 'tryCompletePendingMapLoad', 'confirmMapLoaded');
for (const condition of [
  'pendingMapRequestAccepted',
  'pendingMapStatusConfirmed',
  'pendingMapRasterReady',
]) {
  if (!navigationCompletion.includes(condition)) {
    throw new Error(`navigation completion does not require ${condition}`);
  }
}

for (const token of [
  'pendingMapRequestAccepted',
  'pendingMapStatusConfirmed',
  'pendingMapRasterReady',
  'tryCompleteInspectionMapLoad',
]) {
  if (!inspection.includes(token)) {
    throw new Error(`inspection map load gate is missing ${token}`);
  }
}
const inspectionLoad = functionBody(inspection, 'loadMap', 'routeFromMessage');
if (inspectionLoad.includes('mapReady = true')) {
  throw new Error('inspection marks the map ready from switch service response');
}
const inspectionCompletion = functionBody(
  inspection, 'tryCompleteInspectionMapLoad', 'loadMap');
for (const condition of [
  'pendingMapRequestAccepted',
  'pendingMapStatusConfirmed',
  'pendingMapRasterReady',
]) {
  if (!inspectionCompletion.includes(condition)) {
    throw new Error(`inspection completion does not require ${condition}`);
  }
}

process.stdout.write('large-map page readiness contracts passed\n');
