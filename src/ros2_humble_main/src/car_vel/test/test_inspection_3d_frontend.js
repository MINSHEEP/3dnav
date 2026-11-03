#!/usr/bin/env node
'use strict';

const fs = require('fs');
const assert = require('assert');
const path = require('path');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const html = fs.readFileSync(path.join(frontend, 'inspection.html'), 'utf8');
const common = fs.readFileSync(path.join(frontend, 'robot_ui_common.js'), 'utf8');
const viewer = fs.readFileSync(path.join(frontend, 'pointcloud_viewer.js'), 'utf8');
const inline = html.slice(html.lastIndexOf('<script>') + '<script>'.length,
  html.indexOf('</script>', html.lastIndexOf('<script>')));
new Function(inline);

function requireMatch(source, pattern, description) {
  if (!pattern.test(source)) throw new Error(description);
}

requireMatch(html, /id="inspection-pointcloud-canvas"/, '3D inspection canvas missing');
requireMatch(html, /pointcloud_viewer\.js\?v=/, 'point cloud renderer not loaded');
requireMatch(html, /id="btn-inspection-preview"/, 'no-motion preview button missing');
requireMatch(html, /在三维导航页编辑路线/, '3D route editing entry missing');
requireMatch(inline, /inspectionMapView = view;[\s\S]*?'map-canvas'\)\.hidden = is3d/,
  'Xingji 3D map view does not hide the 2D canvas');
requireMatch(inline, /function isConfirmedMap2D[\s\S]*?navigationModeSeen[\s\S]*?selectedModeAtMs <= THREE_D_STATUS_MAX_AGE_MS[\s\S]*?=== 'map_2d'/,
  '2D initial-pose picking is not gated on a fresh confirmed map_2d mode');
requireMatch(inline, /mapCanvas\.setPoseInteraction\(\s*\(interactionMode === 'map_2d'[\s\S]*?interactionMode === 'gangben_legacy_3d'\)/,
  '2D map interaction is not isolated to map_2d and confirmed Gangben legacy 3D');
requireMatch(inline, /function handleInitialPose\(x, y, yaw, source = '2d'\)[\s\S]*?: !\(isConfirmedMap2D\(\) \|\| isGangbenTerrain3D\(\)\)/,
  'a stale or unconfirmed 2D callback can publish an initial pose');
requireMatch(inline, /cloudIdentity\.load_id !== currentMapLoadId/,
  'point cloud identity is not matched to map load');
requireMatch(inline, /cloudIdentity\.stamp_ns !== cloudStampNs\(acceptedGlobalCloud\)/,
  'point cloud stamp is not matched to identity');
requireMatch(inline, /selected !== 'xingji'[\s\S]*?profile\.navigation_verified !== true/,
  'unverified robot body may use 3D inspection');
requireMatch(inline, /pickRoutePoint\([\s\S]*?Number\(picked\.id\)/,
  '3D marker does not select a stable route point ID');
requireMatch(inline, /if \(checkpointFor\(pointId\)\) removeCheckpoint\(pointId\);[\s\S]*?ensureCheckpoint\(pointId\)/,
  '3D route marker cannot toggle detection checkpoint');
requireMatch(inline, /'\/terrain\/check_3d_ground'[\s\S]*?draft\.valid = true/,
  '3D initial pose is not ground-checked');
requireMatch(inline, /'\/navigation\/preview_3d_route'[\s\S]*?UI\.TYPES\.preview3dRoute/,
  'inspection route preview service not used');
requireMatch(inline, /result\.path && result\.path\.header[\s\S]*?frame_id[\s\S]*?!== 'map'/,
  'preview path is not constrained to the map frame');
requireMatch(inline, /session !== requestSession \|\| !requestSession\.connected[\s\S]*?currentMapLoadId !== mapLoadId/,
  'asynchronous preview result may survive session or map change');
requireMatch(inline, /const bidirectional = document\.getElementById\('bidirectional'\)\.checked;[\s\S]*?loop_count: loopCount, continuous: false, bidirectional/,
  'finite inspection preview must send the selected round-trip mode');
assert(!inline.includes('3D 巡检暂不支持往返'), '3D round-trip checkbox blocks preview');
requireMatch(inline, /const preview = terrain3d \? currentThreeDPreview\(\) : null;[\s\S]*?if \(terrain3d && !preview\)/,
  '3D start can bypass explicit route preview');
requireMatch(inline, /preview_id: terrain3d \? preview\.previewId : ''/,
  'inspection start does not submit preview token');
assert(!inline.includes('THREE_D_PREVIEW_MAX_AGE_MS'),
  'human preview evidence must not have a countdown');
requireMatch(common, /preview3dRoute:\s*'lzp_interface\/srv\/Preview3DRoute'/,
  'RobotUi Preview3DRoute type missing');
for (const method of ['setRoutePoints', 'setCheckpointPoints', 'pickRoutePoint']) {
  requireMatch(viewer, new RegExp(`\\b${method}\\(`), `${method} viewer API missing`);
}

console.log('inspection 3D point-cloud interaction and fail-closed preview contract: ok');
