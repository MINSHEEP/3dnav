'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const frontend = path.join(__dirname, '..', 'car_vel', 'web_frontend');
let now = 1000;
const timers = new Map();
let next = 0;
const root = {
    setInterval(fn) { timers.set(++next, fn); return next; },
    clearInterval(id) { timers.delete(id); },
};
const context = { window: root, Date: { now: () => now }, WeakMap };
vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(frontend, 'velocity_guard_widget.js'), 'utf8'), context);
const callbacks = [];
const session = { subscribe(topic, type, fn) {
    assert.strictEqual(topic, '/navigation/velocity_guard'); callbacks.push(fn);
} };
const element = { hidden: false, textContent: '' };
root.VelocityGuardWidget.attach(session, element, 'std_msgs/msg/String');
callbacks[0]({ data: JSON.stringify({ blocked: true, reason: '禁止自动倒退' }) });
assert.strictEqual(element.hidden, false);
assert.strictEqual(element.textContent, '禁止自动倒退');
now += 2100;
for (const timer of timers.values()) timer();
assert(element.textContent.includes('已过期'));
root.VelocityGuardWidget.attach(session, element, 'std_msgs/msg/String');
assert.strictEqual(timers.size, 1);
callbacks[1]({ data: JSON.stringify({ blocked: false, reason: '' }) });
callbacks[0]({ data: JSON.stringify({ blocked: true, reason: '旧会话' }) });
assert.strictEqual(element.hidden, true, 'old session cannot overwrite new state');
callbacks[1]({ data: 'broken' });
assert(element.textContent.includes('无法解析'));
const events = [];
root.VelocityGuardWidget.attach(session, element, 'std_msgs/msg/String', {
    onEvent: (text) => events.push(text),
});
const warn = (changes = {}) => callbacks[2]({ data: JSON.stringify({
    blocked: false, reason: '', warning: true,
    warning_code: 'no_motion_observed',
    warning_reason: '已持续输出非零导航速度，但里程计未检测到明显移动',
    ...changes,
}) });
warn();
assert.strictEqual(element.hidden, false, 'unblocked motion warning must be visible');
assert(element.textContent.includes('未检测到明显移动'));
assert.strictEqual(events.length, 1);
for (let i = 0; i < 20; ++i) {
    now += 100;
    warn({ warning_reason: `持续 ${5 + i} 秒，未检测到明显移动` });
}
assert.strictEqual(events.length, 1, 'elapsed values must not flood the operation log');
warn({ blocked: true, reason: '地形保护停车' });
assert.strictEqual(element.textContent, '地形保护停车', 'stop reason takes precedence');
warn({ blocked: false });
now += 2100;
for (const timer of timers.values()) timer();
assert(element.textContent.includes('已过期'), 'stale warning cannot remain current');
callbacks[2]({ data: JSON.stringify({ blocked: false, warning: false }) });
assert.strictEqual(element.hidden, true);
assert(events.some(text => text.includes('解除')));
const eventCount = events.length;
callbacks[2]({ data: JSON.stringify({ blocked: false, warning: false }) });
assert.strictEqual(events.length, eventCount, 'clear state must not repeat logs');
root.VelocityGuardWidget.attach(session, element, 'std_msgs/msg/String', {
    onEvent: () => { throw new Error('logger unavailable'); },
});
callbacks[3]({ data: JSON.stringify({ blocked: false, warning: true,
    warning_code: 'odometry_unavailable', warning_reason: '里程计不可用，无法判断是否移动' }) });
assert(element.textContent.includes('无法判断'));
warn();
assert(element.textContent.includes('无法判断'), 'old callback cannot restore old warning');
for (const page of ['navigation.html', 'inspection.html']) {
    const html = fs.readFileSync(path.join(frontend, page), 'utf8');
    assert(html.includes('velocity_guard_widget.js'));
    assert(html.includes('VelocityGuardWidget.attach(activeSession'));
    assert(html.includes('id="velocity-guard-reason"'));
    assert(/onEvent:\s*log/.test(html), 'motion warning is recorded on both pages');
}
console.log('final velocity guard display, expiry and reconnect: ok');
