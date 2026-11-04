#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const html = fs.readFileSync(path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'inspection.html'), 'utf8');
const script = html.slice(html.lastIndexOf('<script>') + '<script>'.length,
  html.indexOf('</script>', html.lastIndexOf('<script>')));
new Function(script);

function extract(name) {
  const start = script.indexOf(`function ${name}(`);
  assert(start >= 0, `${name} helper missing`);
  const open = script.indexOf('{', start);
  let depth = 0;
  for (let index = open; index < script.length; index += 1) {
    if (script[index] === '{') depth += 1;
    if (script[index] === '}') depth -= 1;
    if (depth === 0) return script.slice(start, index + 1);
  }
  throw new Error(`${name} helper is unterminated`);
}

const makeConfirmed = new Function('session', 'navigationModeSeen',
  'selectedModeAtMs', 'THREE_D_STATUS_MAX_AGE_MS', 'robotBodyStatus',
  'robotBodyAtMs', `return (${extract('confirmedRobotBody')});`);
function bodyId(body, options = {}) {
  const confirmed = makeConfirmed(
    {connected: options.connected !== false},
    options.modeSeen !== false,
    options.modeAt ?? 1000,
    6000,
    body,
    options.bodyAt ?? 1000);
  return confirmed(1000);
}
const body = (id) => ({
  config: {selected: id, profiles: {[id]: {navigation_verified: true}}},
  active_body: id,
  connected: true, switching: false, status: {}
});
assert.strictEqual(bodyId(body('xingji')), 'xingji');
assert.strictEqual(bodyId(body('gangben_zsl1')), 'gangben_zsl1');
assert.strictEqual(bodyId(body('xingji'), {bodyAt: -6000}), '');
assert.strictEqual(bodyId(body('xingji'), {connected: false}), '');
assert.strictEqual(bodyId({...body('xingji'), active_body: 'gangben_zsl1'}), '');
assert.strictEqual(bodyId({...body('gangben_zsl1'), switching: true}), '');
assert.strictEqual(bodyId({...body('gangben_zsl1'), status: {emergency_latched: true}}), '');
const unverified = body('gangben_zsl1');
unverified.config.profiles.gangben_zsl1.navigation_verified = false;
assert.strictEqual(bodyId(unverified), '', 'unverified robot body must not enable navigation');

const makePolicy = new Function('isConfirmedMap2D', 'isTerrain3D',
  'confirmedRobotBody', `return (${extract('inspectionInteractionMode')});`);
const policy = (mode, robot = '') => makePolicy(
  () => mode === 'map_2d', () => mode === 'terrain_3d', () => robot)(1000);
assert.strictEqual(policy('map_2d'), 'map_2d');
assert.strictEqual(policy('terrain_3d', 'xingji'), 'xingji_3d');
assert.strictEqual(policy('terrain_3d', 'gangben_zsl1'), 'gangben_legacy_3d');
assert.strictEqual(policy('terrain_3d', ''), 'blocked');
assert.strictEqual(policy('unknown', 'xingji'), 'blocked');

assert(/const view = isXingjiTerrain3D\(\) \? '3d' : '2d'/.test(script),
  'Gangben 3D workflow must retain its legacy map view');
assert(/interactionMode === 'blocked'[\s\S]*?interactionMode === 'xingji_3d' && !currentThreeDPreview\(\)/.test(script),
  'start button must block unknown body but require preview only for Xingji 3D');
assert(/const terrain3d = interactionMode === 'xingji_3d'/.test(script),
  'Gangben 3D start must not require the Xingji preview token');
assert(/interactionMode === 'gangben_legacy_3d'[\s\S]*?mapReady && initialPoseMode/.test(script),
  'Gangben 3D legacy map interaction was removed');

console.log('inspection Xingji/Gangben navigation-mode isolation frontend: ok');
