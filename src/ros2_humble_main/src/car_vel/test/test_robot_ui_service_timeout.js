#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

let clockMs = 0;
let nextTimerId = 1;
const timers = new Map();
function setTimeoutFake(callback, delay = 0) {
  const id = nextTimerId++;
  timers.set(id, { callback, due: clockMs + Number(delay || 0) });
  return id;
}
function clearTimeoutFake(id) {
  timers.delete(id);
}
function advanceTime(milliseconds) {
  clockMs += milliseconds;
  for (const [id, timer] of [...timers.entries()]) {
    if (timer.due > clockMs) continue;
    timers.delete(id);
    timer.callback();
  }
}

class HangingService {
  callService() {}
}

const browser = {
  console,
  document: {},
  location: { hostname: 'robot' },
  setTimeout: setTimeoutFake,
  clearTimeout: clearTimeoutFake,
  ROSLIB: {
    Message: class Message { constructor(value) { Object.assign(this, value); } },
    ServiceRequest: class ServiceRequest { constructor(value) { Object.assign(this, value); } },
    Service: HangingService,
  },
};
browser.window = browser;
const context = vm.createContext(browser);
const commonPath = path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'robot_ui_common.js');
vm.runInContext(fs.readFileSync(commonPath, 'utf8'), context, {
  filename: commonPath,
});

(async () => {
  const session = new browser.RobotUi.RosSession({});
  session.ros = {};
  session.connected = true;
  let rejection = null;
  session.call('/hung_service', 'example/srv/Hang', {}, 1000)
    .catch((error) => { rejection = error; });

  advanceTime(1001);
  await Promise.resolve();

  assert(rejection, 'hung ROS service promise did not reject after its deadline');
  assert.match(rejection.message, /超时/);
  assert.strictEqual(timers.size, 0);
  process.stdout.write('RobotUi service timeout regression passed\n');
})().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
