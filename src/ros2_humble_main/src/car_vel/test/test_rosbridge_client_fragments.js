const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

class FakeWebSocket {
  static OPEN = 1;
  static CLOSING = 2;

  constructor(url) {
    this.url = url;
    this.readyState = FakeWebSocket.OPEN;
    this.listeners = new Map();
    this.sent = [];
  }

  addEventListener(name, callback) {
    this.listeners.set(name, callback);
  }

  send(payload) {
    this.sent.push(JSON.parse(payload));
  }

  close() {
    this.readyState = 3;
  }
}

let clockMs = 0;
let nextTimerId = 1;
const fakeTimers = new Map();

function fakeSetTimeout(callback, delay = 0) {
  const id = nextTimerId++;
  fakeTimers.set(id, { callback, due: clockMs + Number(delay || 0) });
  return id;
}

function fakeClearTimeout(id) {
  fakeTimers.delete(id);
}

function advanceTime(milliseconds) {
  clockMs += milliseconds;
  [...fakeTimers.entries()]
    .filter(([, timer]) => timer.due <= clockMs)
    .forEach(([id, timer]) => {
      fakeTimers.delete(id);
      timer.callback();
    });
}

const browser = {
  WebSocket: FakeWebSocket,
  clearTimeout: fakeClearTimeout,
  console,
  setTimeout: fakeSetTimeout,
};
browser.window = browser;
const context = vm.createContext(browser);
const clientPath = path.join(
  __dirname,
  '..',
  'car_vel',
  'web_frontend',
  'rosbridge_client.js'
);
vm.runInContext(fs.readFileSync(clientPath, 'utf8'), context, {
  filename: clientPath,
});

const ros = new browser.ROSLIB.Ros({ url: 'ws://test:9090' });
const rosErrors = [];
ros.on('error', (error) => rosErrors.push(error));
let receivedMap = null;
ros.addTopicCallback('/projected_map', (message) => {
  receivedMap = message;
});

// The captured field failure starts once a projected map exceeds rosbridge's
// one-million-character WebSocket fragment size. Recreate that scale here and
// deliver the fragments out of order.
const mapData = new Array(450000).fill(-1);
mapData[449999] = 100;
const serialized = JSON.stringify({
  op: 'publish',
  topic: '/projected_map',
  msg: {
    info: { width: 842, height: 534, resolution: 0.05 },
    data: mapData,
  },
});
assert(serialized.length > 1000000);

const fragmentSize = 1000000;
const fragments = [];
for (let offset = 0; offset < serialized.length; offset += fragmentSize) {
  fragments.push(serialized.slice(offset, offset + fragmentSize));
}
const order = [...fragments.keys()].reverse();
order.forEach((num) => {
  ros.handleMessage(JSON.stringify({
    op: 'fragment',
    id: 'projected-map-1',
    num,
    total: fragments.length,
    data: fragments[num],
  }));
});

assert(receivedMap);
assert.strictEqual(receivedMap.info.width, 842);
assert.strictEqual(receivedMap.info.height, 534);
assert.strictEqual(receivedMap.data.length, 450000);
assert.strictEqual(receivedMap.data[449999], 100);
assert.strictEqual(ros.fragmentBuffers.size, 0);

// A large map can take longer than ten seconds in total while fragments are
// still arriving regularly. The timeout must measure inactivity, not elapsed
// time since the first fragment, otherwise the first load is blank and a fast
// replay after refreshing the page succeeds.
let receivedSlowMap = null;
ros.addTopicCallback('/map', (message) => {
  receivedSlowMap = message;
});
const slowMapData = new Array(900000).fill(-1);
slowMapData[899999] = 100;
const slowSerialized = JSON.stringify({
  op: 'publish',
  topic: '/map',
  msg: {
    info: { width: 1200, height: 750, resolution: 0.05 },
    data: slowMapData,
  },
});
const slowFragments = [];
for (let offset = 0; offset < slowSerialized.length; offset += fragmentSize) {
  slowFragments.push(slowSerialized.slice(offset, offset + fragmentSize));
}
assert(slowFragments.length >= 3);
slowFragments.forEach((data, num) => {
  if (num > 0) advanceTime(6000);
  ros.handleMessage(JSON.stringify({
    op: 'fragment',
    id: 'slow-map-with-progress',
    num,
    total: slowFragments.length,
    data,
  }));
});
assert(receivedSlowMap, 'large map was discarded despite regular fragment progress');
assert.strictEqual(receivedSlowMap.data[899999], 100);
assert.strictEqual(ros.fragmentBuffers.size, 0);

const mapTopic = new browser.ROSLIB.Topic({
  ros,
  name: '/projected_map',
  messageType: 'nav_msgs/msg/OccupancyGrid',
  queueLength: 1,
  throttleRate: 250,
});
mapTopic.subscribe(() => {});
const subscription = ros.socket.sent.at(-1);
assert.strictEqual(subscription.op, 'subscribe');
assert.strictEqual(subscription.queue_length, 1);
assert.strictEqual(subscription.throttle_rate, 250);

ros.handleMessage(JSON.stringify({
  op: 'fragment',
  id: 'stalled-map',
  num: 0,
  total: 2,
  data: '{"op":"publish",',
}));
assert.strictEqual(ros.fragmentBuffers.size, 1);
advanceTime(30001);
assert.strictEqual(ros.fragmentBuffers.size, 0);
assert(rosErrors.some((error) => (
  error.message === 'Timed out waiting for rosbridge fragments: stalled-map'
)));

ros.close();
assert.strictEqual(ros.fragmentBuffers.size, 0);
assert.strictEqual(fakeTimers.size, 0);
process.stdout.write('rosbridge fragment and slow-map regression tests passed\n');
