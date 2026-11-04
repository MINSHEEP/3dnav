#!/usr/bin/env node
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/inspection.html'), 'utf8');
const source = html.slice(html.indexOf('    async function cameraApi('),
    html.indexOf("    document.getElementById('btn-inspection-recognition').addEventListener"));
const elements = new Map();
const document = { getElementById(id) {
    if (!elements.has(id)) elements.set(id, { disabled: false, hidden: true, textContent: '',
        addEventListener(name, fn) { this[name] = fn; } });
    return elements.get(id);
} };
const storage = new Map(), opened = [];
let expireRequests = false;
const window = { AbortController, setTimeout:(fn, ms) => setTimeout(fn, expireRequests ? 0 : ms), clearTimeout,
    open:url=>opened.push(url), sessionStorage: {
    setItem: (key, value) => storage.set(key, value),
    getItem: key => storage.get(key) || null
} };
let enabled = { fall: true, fight: false, smoking: true };
let behaviorRunning = false, yoloRunning = false, failYolo = false, hangRead = false;
const writes = [], logs = [];
async function fetch(url, options = {}) {
    const endpoint = new URL(url).pathname;
    let result;
    if (hangRead && !options.method) return new Promise((resolve, reject) =>
        options.signal.addEventListener('abort', () => reject(Object.assign(new Error('aborted'), {name:'AbortError'}))));
    if (!options.method) {
        result = { behavior: { running: behaviorRunning, enabled: { ...enabled } }, yolo: { running: yoloRunning } };
    } else {
        const body = JSON.parse(options.body);
        writes.push({ endpoint, body });
        if (endpoint === '/api/yolo') {
            if (failYolo) return { ok: false, status: 503, json: async () => ({ error: 'YOLO 启动失败' }) };
            enabled = { fall: false, fight: false, smoking: false };
            behaviorRunning = false; yoloRunning = true;
            result = { running: true };
        } else if (endpoint === '/api/behavior/config') {
            enabled = { fall: body.fall_enabled, fight: body.fight_enabled, smoking: body.smoking_enabled };
            yoloRunning = false;
            result = { running: behaviorRunning };
        } else if (endpoint === '/api/detection') {
            if (body.operation === 'toggle' && (behaviorRunning || yoloRunning)) {
                behaviorRunning = false; yoloRunning = false;
            } else { behaviorRunning = true; yoloRunning = false; }
            result = { success: true, running: behaviorRunning || yoloRunning,
                behavior: { running: behaviorRunning }, yolo: { running: yoloRunning } };
        } else throw new Error('unexpected POST ' + endpoint);
    }
    return { ok: true, json: async () => result };
}
let reconnects = 0;
const context = vm.createContext({ document, window, fetch, log: text => logs.push(text),
    CAMERA_BASE: 'http://camera.test:8766', behaviorWidget: { refreshNow() {} },
    reconnectCamera() { reconnects++; }, refreshCameraState() {}, recognitionBusy:false, cameraView: 'raw' });
vm.runInContext(source, context);

(async () => {
    await context.switchInspectionView('behavior');
    assert.equal(behaviorRunning, true);
    assert.equal(writes.at(-1).endpoint, '/api/detection');
    const count = writes.length;
    await context.switchInspectionView('behavior');
    await context.switchInspectionView('raw');
    assert.equal(writes.length, count, 'running behavior and raw view must not restart detection');
    await context.switchInspectionView('yolo');
    assert.equal(yoloRunning, true);
    assert.equal(behaviorRunning, false);
    await context.switchInspectionView('behavior');
    assert.deepEqual(enabled, { fall: true, fight: false, smoking: true });
    assert.equal(behaviorRunning, true);
    assert.equal(yoloRunning, false);
    await context.toggleInspectionRecognition();
    assert.equal(behaviorRunning, false);
    assert.equal(context.cameraView, 'raw');
    await context.toggleInspectionRecognition();
    assert.equal(behaviorRunning, true);
    assert.equal(context.cameraView, 'behavior');
    let ordinaryFullscreen = 0, thermalFullscreen = 0;
    document.getElementById('camera-frame').requestFullscreen = () => ordinaryFullscreen++;
    document.getElementById('thermal-camera-frame').requestFullscreen = () => thermalFullscreen++;
    const controls = html.slice(html.indexOf("    document.getElementById('btn-camera-reconnect').addEventListener"),
        html.indexOf("    document.getElementById('btn-camera-snapshot').addEventListener"));
    vm.runInContext(controls, context);
    const previousReconnects = reconnects;
    document.getElementById('btn-camera-reconnect').click();
    document.getElementById('btn-camera-fullscreen').click();
    document.getElementById('btn-thermal-fullscreen').click();
    assert.equal(reconnects, previousReconnects + 1);
    assert.equal(ordinaryFullscreen, 1);
    assert.equal(thermalFullscreen, 1);
    const snapshot = html.slice(html.indexOf("    document.getElementById('btn-camera-snapshot').addEventListener"),
        html.indexOf("    document.getElementById('inspection-thermal-config').addEventListener"));
    vm.runInContext(snapshot, context);
    for (const [view, expected] of [['raw','/api/snapshot'], ['yolo','/api/yolo/frame.jpg'], ['behavior','/api/behavior/frame.jpg']]) {
        context.cameraView = view;
        document.getElementById('btn-camera-snapshot').click();
        assert.ok(opened.at(-1).includes(expected), `snapshot follows ${view} view`);
    }
    failYolo = true;
    await context.switchInspectionView('yolo');
    assert.equal(context.cameraView, 'behavior');
    assert.match(logs.at(-1), /YOLO 启动失败/);
    assert.equal(document.getElementById('btn-camera-view').disabled, false);
    hangRead = true; expireRequests = true;
    await context.switchInspectionView('yolo');
    assert.match(logs.at(-1), /请求超时/);
    assert.equal(document.getElementById('btn-inspection-recognition').disabled, false);
    hangRead = false; expireRequests = false; storage.clear();
    enabled = {fall:false, fight:false, smoking:false}; behaviorRunning = false; yoloRunning = true;
    await context.switchInspectionView('behavior');
    assert.match(logs.at(-1), /请先勾选/);
    assert.equal(behaviorRunning, false, 'unchecked categories must stay disabled');
    assert.equal(yoloRunning, true);
    console.log('inspection view: auto-start, mutually exclusive switching, restore selection, raw and startup failure OK');
})().catch(error => { console.error(error); process.exitCode = 1; });
