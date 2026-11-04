#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const handlers = new Map();
const on = (type, callback) => handlers.set(type, [...(handlers.get(type) || []), callback]);
const document = { addEventListener: on };
let interval;
const window = {
  addEventListener: on,
  setInterval: (callback) => { interval = callback; return 1; },
  clearInterval: () => { interval = null; },
};
const source = fs.readFileSync(path.resolve(__dirname, '..', 'car_vel',
  'web_frontend', 'robot_ui_common.js'), 'utf8');
vm.runInNewContext(source, { window, document });

const published = [];
let releases = 0;
const teleop = new window.RobotUi.KeyboardTeleop({
  session: { connected: true, publish: (...args) => published.push(args) },
  onNeutral: () => { releases += 1; },
});
teleop.setEnabled(true);
const key = (type, value) => {
  for (const callback of handlers.get(type) || []) {
    callback({ key: value, target: null, preventDefault() {} });
  }
};

key('keydown', 'w');
key('keydown', 'q');
if (interval) interval();
key('keyup', 'w');
assert.strictEqual(releases, 0, 'partial release must not rearm movement');
key('keyup', 'q');
assert.strictEqual(releases, 1, 'all keys up must emit an explicit release');
assert.strictEqual(published.at(-1)[0], '/cmd_vel');

teleop.publishZero();
teleop.stopAndZero();
assert.strictEqual(releases, 1, 'synthetic zero and blur must not count as key release');

teleop.setEnabled(false);
key('keyup', 'w');
assert.strictEqual(releases, 1, 'disabled teleop must not confirm a release');

console.log('mapping teleop explicit key-release behavior: ok');
