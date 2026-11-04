const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/navigation.html'), 'utf8');
const start = source.indexOf('    function armRelocalizationWatchdog(');
const end = source.indexOf('\n    function ', start + 10);
const duration = Number(source.match(/const RELOCALIZATION_WATCHDOG_MS = (\d+)/)[1]);
let timer, delay;
const sent = [];
const context = {
  RELOCALIZATION_WATCHDOG_MS: duration,
  window: {setTimeout: (fn, ms) => { timer = fn; delay = ms; return 1; }},
  clearRelocalizationWatchdog() {}, armRelocalizationCancelWatchdog() {},
  relocalizationActive: true, localizationGeneration: 3,
  relocalizationRequestMapName: 'map1', relocalizationRequestStampKey: 'stamp',
  relocalizationRequestId: 'request1', localizationReady: true,
  LOCALIZATION_HEALTH: {DEGRADED: 'degraded'},
  session: {connected: true, publish: (...args) => sent.push(args)},
  RELOCALIZATION_CANCEL_TOPIC: '/relocalization/cancel', UI: {TYPES: {empty: 'Empty'}},
  setMode() {}, setRelocalizationUi() {}, log() {}, updateMapReadyUi() {},
};
vm.createContext(context);
vm.runInContext(source.slice(start, end), context);
context.armRelocalizationWatchdog(3, 'map1', 'stamp', 'request1');
assert(delay > (5 + 30 + 1.5 + 12 + 10) * 1000);
assert.strictEqual(sent.length, 0);
timer();
assert.strictEqual(sent.length, 1);
assert.strictEqual(sent[0][0], '/relocalization/cancel');
assert.strictEqual(context.localizationReady, false);
// A completed or replaced request must not be canceled by its old timer.
context.relocalizationActive = false;
timer();
assert.strictEqual(sent.length, 1);
console.log('PASS: expanded registration budget, timeout cancellation, completed request fencing');
