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

const browser = {
  WebSocket: FakeWebSocket,
  clearTimeout,
  console,
  setTimeout,
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

ros.close();
assert.strictEqual(ros.fragmentBuffers.size, 0);
process.stdout.write('rosbridge fragment regression test passed\n');
