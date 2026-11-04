#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const assert = require('assert');
const vm = require('vm');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const html = fs.readFileSync(path.join(frontend, 'navigation.html'), 'utf8');
const campusHtml = fs.readFileSync(path.join(frontend, 'campus_navigation.html'), 'utf8');

for (const token of [
  'id="automatic-terrain-status"',
  'id="automatic-terrain-mode"',
  'id="automatic-terrain-speed"',
  "subscribe('/automatic_terrain_navigation/status'",
  '自动地形导航', '连续楼梯仍需人工确认',
]) {
  if (!html.includes(token)) {
    throw new Error(`missing automatic terrain frontend contract: ${token}`);
  }
}

for (const token of [
  'id="automatic-navigation-enabled"',
  'data-automatic-mode="rough_ground"',
  'data-automatic-mode="obstacle"',
  'data-automatic-mode="ramp"',
  'collectAutomaticNavigationForm',
  'topologyDraft.automatic_navigation',
  'id="restricted-flat-site"',
]) {
  if (!campusHtml.includes(token)) {
    throw new Error(`missing automatic terrain config contract: ${token}`);
  }
}

const collect = campusHtml.match(/function collectAutomaticNavigationForm\(\) \{[^]*?\n\}/)[0];
const restricted = { checked: false };
const context = { document: {
  getElementById(id) {
    return id === 'restricted-flat-site' ? restricted : { checked: true };
  },
  querySelectorAll() { return [{ dataset: { automaticMode: 'ramp' } }]; },
} };
vm.createContext(context);
vm.runInContext(collect, context);
assert.strictEqual(context.collectAutomaticNavigationForm().restricted_flat_site, false);
restricted.checked = true;
assert.strictEqual(context.collectAutomaticNavigationForm().restricted_flat_site, true);
assert.deepStrictEqual(Array.from(context.collectAutomaticNavigationForm().enabled_modes), ['ramp']);

console.log('automatic same-map terrain navigation frontend contract: ok');
