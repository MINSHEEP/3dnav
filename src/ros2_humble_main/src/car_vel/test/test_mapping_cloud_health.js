#!/usr/bin/env node
'use strict';

const assert = require('assert');
const { GlobalCloudHealthMonitor } = require(
  '../car_vel/web_frontend/mapping_cloud_health.js');

class FakeClock {
  constructor() {
    this.time = 0;
    this.nextId = 1;
    this.tasks = new Map();
  }

  now = () => this.time;

  setTimeout = (callback, delay) => this.add(callback, delay, 0);

  clearTimeout = (id) => this.tasks.delete(id);

  setInterval = (callback, delay) => this.add(callback, delay, delay);

  clearInterval = (id) => this.tasks.delete(id);

  add(callback, delay, repeat) {
    const id = this.nextId++;
    this.tasks.set(id, { callback, due: this.time + delay, repeat });
    return id;
  }

  advance(milliseconds) {
    const target = this.time + milliseconds;
    while (true) {
      const next = [...this.tasks.entries()]
        .filter(([, task]) => task.due <= target)
        .sort((left, right) => left[1].due - right[1].due)[0];
      if (!next) break;
      const [id, task] = next;
      this.time = task.due;
      if (task.repeat) task.due += task.repeat;
      else this.tasks.delete(id);
      task.callback();
    }
    this.time = target;
  }
}

function fixture() {
  const clock = new FakeClock();
  const events = [];
  const monitor = new GlobalCloudHealthMonitor({
    initialTimeoutMs: 45000,
    staleTimeoutMs: 15000,
    checkIntervalMs: 1000,
    now: clock.now,
    setTimeoutFn: clock.setTimeout,
    clearTimeoutFn: clock.clearTimeout,
    setIntervalFn: clock.setInterval,
    clearIntervalFn: clock.clearInterval,
    onState: (event) => events.push(event),
  });
  return { clock, events, monitor };
}

{
  const { clock, events, monitor } = fixture();
  monitor.setConnected(true);
  monitor.start();
  clock.advance(44999);
  assert.equal(events.at(-1).state, 'waiting');
  clock.advance(1);
  assert.equal(events.at(-1).state, 'missing');
}

{
  const { clock, events, monitor } = fixture();
  monitor.setConnected(true);
  monitor.start();
  clock.advance(5000);
  assert.equal(monitor.markValidFrame(), true);
  assert.equal(events.at(-1).state, 'ready');
  clock.advance(14999);
  assert.equal(events.at(-1).state, 'ready');
  clock.advance(1);
  assert.equal(events.at(-1).state, 'stale');
  monitor.markValidFrame();
  assert.deepEqual(events.at(-1), { state: 'ready', recovered: true });
}

{
  const { clock, events, monitor } = fixture();
  monitor.setConnected(true);
  monitor.start();
  monitor.markValidFrame();
  monitor.stop();
  clock.advance(60000);
  assert.equal(events.at(-1).state, 'stopped');
  monitor.start();
  clock.advance(45000);
  assert.equal(events.at(-1).state, 'missing');
}

{
  const { clock, events, monitor } = fixture();
  monitor.setConnected(true);
  monitor.start();
  monitor.markValidFrame();
  monitor.setConnected(false);
  assert.equal(events.at(-1).state, 'disconnected');
  monitor.setConnected(true);
  clock.advance(45000);
  assert.equal(events.at(-1).state, 'missing');
}

console.log('Mapping cloud session-health behavior: ok');
