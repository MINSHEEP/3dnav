'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const frontend = path.join(__dirname, '..', 'car_vel', 'web_frontend');
const script = fs.readFileSync(path.join(frontend, 'voice_debug.js'), 'utf8');

function fixture() {
    const elements = new Map();
    const listeners = {};
    const calls = [];
    const element = id => {
        if (!elements.has(id)) elements.set(id, {
            id, disabled: false, hidden: false, open: false, dataset: {}, children: [],
            classList: { add() {}, remove() {}, toggle() {} },
            textContent: '', append(...children) { this.children.push(...children); },
            appendChild(child) { this.children.push(child); }, querySelector() { return null; },
            addEventListener(type, callback) { listeners[`${id}:${type}`] = callback; },
            setAttribute() {}, scrollIntoView() {}, replaceChildren() {},
            getBoundingClientRect() { return { left: 0, bottom: 200, width: 200, height: 200 }; },
            getContext() { return { putImageData() {}, save() {}, restore() {}, translate() {},
                rotate() {}, beginPath() {}, arc() {}, fill() {}, moveTo() {}, lineTo() {},
                stroke() {}, closePath() {} }; }, setPointerCapture() {},
        });
        return elements.get(id);
    };
    const groups = {
        '[data-direction]': ['forward', 'backward', 'left', 'right', 'turn_left', 'turn_right'].map(direction => {
            const button = element(direction); button.dataset.direction = direction; return button;
        }),
        '[data-posture]': [], '[data-phrase]': [],
        '[data-navigation]': ['prepare', 'route1', 'route2'].map(navigation => {
            const button = element(navigation); button.dataset.navigation = navigation; return button;
        }),
    };
    const nav = { state: 'idle', ready: false, busy: false, active: false, anchor: null,
        map_available: true, map_revision: 'saved-map', can_set_anchor: true,
        message: '点击开始导航。' };
    const status = { ready: true, preview: false, voice_connected: true, robot_online: true,
        posture: '站立', posture_busy: false, microphone_enabled: false, events: [], navigation: nav };
    const map = { width: 2, height: 2, resolution: 1, revision: 'saved-map',
        origin: { position: { x: 0, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
        pixels: Buffer.from([255, 0, 110, 255]).toString('base64') };
    const document = {
        hidden: false, body: { dataset: { servicePort: '8768' } },
        getElementById: element, querySelectorAll: selector => groups[selector] || [],
        addEventListener(type, callback) { listeners[`document:${type}`] = callback; },
        createElement: () => ({ className: '', append() {}, remove() {} }),
        createTextNode: text => text,
    };
    const window = {
        location: { href: 'http://127.0.0.1:8768/voice_debug.html' },
        setTimeout, clearTimeout, setInterval: () => 1, clearInterval() {},
        addEventListener(type, callback) { listeners[`window:${type}`] = callback; },
    };
    vm.runInNewContext(script, {
        window, document, URL, AbortController, console, atob: global.atob,
        ImageData: class { constructor(width, height) { this.data = new Uint8ClampedArray(width * height * 4); } },
        fetch: async (url, options) => {
            calls.push({ path: url.pathname, data: options.body ? JSON.parse(options.body) : null });
            const response = url.pathname.endsWith('/navigation/map') ? map
                : url.pathname.endsWith('/status') ? status
                : url.pathname.endsWith('/session') ? { session: 'test-session' } : { ok: true };
            return { ok: true, json: async () => response };
        },
    });
    return { element, listeners, calls, status, nav, document };
}

async function flush() { await new Promise(resolve => setImmediate(resolve)); }

(async () => {
    const f = fixture();
    await flush();
    assert.equal(f.element('anchor-picker').hidden, false, 'first-use map must be visible before ROS starts');
    assert.equal(f.element('anchor-picker').open, true, 'first-use map must automatically expand');
    assert.equal(f.element('anchor-map').width, 2, 'saved map must be drawn without clicking Start');
    f.listeners['anchor-map:pointerdown']({ button: 0, pointerId: 3, clientX: 100,
        clientY: 100, preventDefault() {} });
    f.listeners['anchor-map:pointerup']({ pointerId: 3 });
    assert.equal(f.element('save-anchor').disabled, false, 'area a must be selectable before navigation startup');
    assert.equal(f.calls.some(call => call.path.endsWith('/navigation/prepare')), false);
    f.document.hidden = true;
    f.listeners['document:visibilitychange']();
    await flush();
    assert.equal(f.calls.find(call => call.path.endsWith('/release')).data.reason, 'hidden',
        'a navigation cancellation caused by hiding the page must report that reason');
    console.log('Voice debug first-use map and session-release tests passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
