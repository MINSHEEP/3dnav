#!/usr/bin/env node
'use strict';
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const web = path.resolve(__dirname, '../car_vel/web_frontend');
const source = fs.readFileSync(path.join(web, 'behavior_widget.js'), 'utf8');

class Element {
    constructor() {
        this.handlers = new Map(); this.attributes = new Map(); this.children = new Map();
        this.textContent = ''; this.hidden = false; this.checked = false; this.disabled = false;
        const classes = new Set();
        this.classList = { add: v => classes.add(v), remove: v => classes.delete(v), contains: v => classes.has(v) };
    }
    addEventListener(name, fn) { if (!this.handlers.has(name)) this.handlers.set(name, new Set()); this.handlers.get(name).add(fn); }
    removeEventListener(name, fn) { this.handlers.get(name)?.delete(fn); }
    emit(name, event = {}) { for (const fn of [...(this.handlers.get(name) || [])]) fn(event); }
    querySelector(selector) { if (!this.children.has(selector)) this.children.set(selector, new Element()); return this.children.get(selector); }
    set innerHTML(value) { this.markup = value; this.children.clear(); }
    get innerHTML() { return this.markup || ''; }
    set src(value) { this.attributes.set('src', value); }
    get src() { return this.attributes.get('src') || ''; }
    getAttribute(name) { return this.attributes.get(name) ?? null; }
    removeAttribute(name) { this.attributes.delete(name); }
}
const flush = async () => { for (let i = 0; i < 12; i++) await Promise.resolve(); };
function harness() {
    let now = 10000, id = 0, urls = 0;
    const timers = new Map(), requests = [], revoked = [];
    const window = new Element(), document = new Element();
    document.hidden = false;
    Object.assign(window, {
        document, AbortController,
        setTimeout(fn, delay) { const key = ++id; timers.set(key, { fn, due: now + delay }); return key; },
        clearTimeout(key) { timers.delete(key); },
        URL: { createObjectURL: () => `blob:behavior-${++urls}`, revokeObjectURL: value => revoked.push(value) },
        fetch(url, options = {}) {
            return new Promise((resolve, reject) => {
                const item = { url, options, resolve, reject, done: false };
                options.signal?.addEventListener('abort', () => { item.aborted = true; });
                requests.push(item);
            });
        }
    });
    class ClockDate extends Date { static now() { return now; } }
    const context = vm.createContext({ window, Date: ClockDate, console });
    vm.runInContext(source, context);
    const root = new Element(), image = new Element(), message = new Element();
    const attach = () => window.RobotBehavior.attach(root, { baseUrl: 'http://robot:8766', image, message });
    const pending = part => requests.filter(r => !r.done && !r.aborted && r.url.includes(part));
    async function respond(part, body, headers = {}, ok = true) {
        const req = pending(part)[0]; assert.ok(req, `pending request ${part}`);
        req.done = true;
        req.resolve({ ok, status: ok ? 200 : 503, json: async () => body, blob: async () => body,
            headers: { get: key => headers[key] ?? null } });
        await flush(); return req;
    }
    async function advance(ms) {
        const end = now + ms;
        for (let i = 0; i < 100; i++) {
            const next = [...timers.entries()].filter(([, v]) => v.due <= end).sort((a,b) => a[1].due-b[1].due)[0];
            if (!next) break;
            now = next[1].due; timers.delete(next[0]); next[1].fn(); await flush();
        }
        now = end; await flush();
    }
    return { window, document, root, image, message, attach, requests, timers, pending, respond, advance, revoked };
}
function status(overrides = {}) {
    return { success: true, running: true, ready: true, fresh: true, error: null,
        source_epoch: 4, stamp_ns: 1793111111000000000, frame_stamp_ns: '1793111111000000123',
        max_result_age_seconds: 5, frame_age_seconds: 0.5,
        enabled: { fall: true, fight: true, smoking: true },
        branches: { fall: { enabled: true, state: 'valid', ready: true, label: 'fall' },
            fight: { enabled: true, state: 'valid', ready: true, label: 'normal' },
            smoking: { enabled: true, state: 'valid', ready: true, label: 'smoking' } }, ...overrides };
}
function headers(value) { return { 'X-Behavior-Source-Epoch': String(value.source_epoch),
    'X-Behavior-Stamp-Ns': value.frame_stamp_ns }; }
const branch = (h, key) => h.root.querySelector(`[data-behavior-branch="${key}"]`).textContent;

(async () => {
    const h = harness(); let widget = h.attach();
    assert.strictEqual(h.requests.length, 1);
    assert.ok(h.requests.every(r => !r.options.method || r.options.method === 'GET'), 'attaching only reads status');
    const warming = status({ ready: false, branches: {
        fall: { enabled: true, state: 'warming', ready: false, label: 'fall' },
        fight: { enabled: true, state: 'unknown', ready: false, label: 'normal' }
    }});
    await h.respond('/status', warming);
    assert.strictEqual(branch(h, 'fall'), '等待连续观测');
    assert.strictEqual(branch(h, 'fight'), '等待连续观测');
    assert.ok(!branch(h, 'fight').includes('正常'), 'unknown result cannot display normal');
    assert.strictEqual(h.requests.length, 1, 'no hidden annotated preview request');
    widget.setPreviewActive(true); await flush();
    assert.strictEqual(h.pending('/frame.jpg').length, 1);
    assert.ok(h.pending('/frame.jpg')[0].url.includes(warming.frame_stamp_ns), 'nanosecond timestamp keeps all digits');
    widget.setPreviewActive(true); widget.setPreviewActive(true);
    assert.strictEqual(h.pending('/frame.jpg').length, 1, 'repeated selection does not duplicate inflight image');
    await h.respond('/frame.jpg', {}, headers(warming)); h.image.emit('load');
    assert.ok(h.image.src.startsWith('blob:')); assert.strictEqual(h.message.hidden, true);
    const frameCount = h.requests.filter(r => r.url.includes('/frame.jpg')).length;
    await h.advance(1500); await h.respond('/status', status());
    assert.strictEqual(branch(h, 'fall'), '就绪 · 疑似跌倒');
    assert.strictEqual(h.requests.filter(r => r.url.includes('/frame.jpg')).length, frameCount, 'same inference frame is not downloaded repeatedly');
    const fallToggle = h.root.querySelector('[data-behavior-toggle="fall"]');
    fallToggle.checked = false; fallToggle.emit('change'); await flush();
    const config = h.pending('/config')[0];
    assert.deepStrictEqual(JSON.parse(config.options.body), { fall_enabled: false, fight_enabled: true, smoking_enabled: true }, 'one switch preserves the other branch');
    assert.strictEqual(h.image.src, '', 'config clears previous temporal evidence immediately');
    const stoppedFall = status({ source_epoch: 5, fresh: false, ready: false,
        enabled: {fall: false, fight: true}, branches: {fall: {enabled: false, state:'disabled'}, fight:{enabled:true,state:'warming'}} });
    await h.respond('/config', stoppedFall); await h.respond('/status', stoppedFall);
    assert.strictEqual(branch(h, 'fall'), '已停止（未启用）');
    assert.strictEqual(branch(h, 'fight'), '等待连续观测');
    assert.ok(h.requests.every(r => /\/api\/behavior\/(status|config|frame\.jpg)/.test(r.url)), 'no robot motion or camera control request');
    widget.dispose(); assert.strictEqual(h.timers.size, 0);

    // A frame that completes after a source change cannot repaint the new source.
    const race = harness(); widget = race.attach();
    await race.respond('/status', status()); widget.observeSource(1); widget.setPreviewActive(true);
    const oldFrame = race.pending('/frame.jpg')[0];
    widget.observeSource(2); await flush();
    assert.ok(oldFrame.aborted); assert.strictEqual(race.image.src, '');
    oldFrame.resolve({ok:true,blob:async()=>({}),headers:{get:key=>headers(status())[key]}}); await flush();
    assert.strictEqual(race.image.src, '', 'late old-source image is ignored');
    const newer = status({source_epoch: 5, frame_stamp_ns: '1793111112000000765'});
    await race.respond('/status', newer); await race.advance(1500);
    await race.respond('/status', newer);
    await race.respond('/frame.jpg', {}, { ...headers(newer), 'X-Behavior-Source-Epoch': '4' });
    assert.strictEqual(race.image.src, '', 'mismatched frame provenance is rejected');
    await race.advance(1500); await race.respond('/status', newer);
    await race.respond('/frame.jpg', {}, headers(newer)); race.image.emit('load');
    assert.ok(race.image.src);
    const runningFrame = race.image.src;
    await race.advance(1500); await race.respond('/status', status({running:false, fresh:false, ready:false}));
    assert.strictEqual(race.image.src, '', 'stop removes the last annotated image');
    assert.ok(race.revoked.includes(runningFrame));
    assert.strictEqual(branch(race,'fall'), '已停止');
    widget.dispose();

    // Inference may advance between status polling and JPEG delivery.
    const advancing = harness(); widget = advancing.attach();
    const first = status({frame_age_seconds: 0});
    await advancing.respond('/status', first);
    widget.setPreviewActive(true);
    await advancing.respond('/frame.jpg', {}, headers(first)); advancing.image.emit('load');
    const original = advancing.image.src;
    await advancing.advance(1500);
    const second = status({frame_stamp_ns: '1793111112000000123'});
    await advancing.respond('/status', second);
    await advancing.respond('/frame.jpg', {}, headers(status({frame_stamp_ns: '1793111112000000124'})));
    assert.strictEqual(advancing.image.src, original, 'an updated frame must not blank the last still-fresh preview');
    assert.strictEqual(advancing.message.hidden, true);
    await advancing.advance(3500);
    assert.strictEqual(advancing.image.src, '', 'the retained preview still expires at its own deadline');
    widget.dispose();

    // Lost status and body timeouts do not leave a convincing stale image visible.
    const stale = harness(); widget = stale.attach();
    await stale.respond('/status', status({frame_age_seconds:4.5})); widget.setPreviewActive(true);
    await stale.respond('/frame.jpg', {}, headers(status())); stale.image.emit('load');
    await stale.advance(500);
    assert.strictEqual(stale.image.src, '', 'local frame age expires without waiting for another response');
    assert.strictEqual(branch(stale, 'fall'), '等待连续观测');
    await stale.advance(1000); await stale.respond('/status', {}, {}, false);
    assert.strictEqual(branch(stale, 'fight'), '不可用');
    assert.strictEqual(stale.image.src, '');
    await stale.advance(1500); assert.strictEqual(stale.pending('/status').length, 1, 'failed status retries after bounded delay');
    const pendingStatus = stale.pending('/status')[0];
    stale.document.hidden = true; stale.document.emit('visibilitychange');
    assert.ok(pendingStatus.aborted); assert.strictEqual(stale.timers.size, 1, 'only abort deadline remains until fetch settles');
    const count = stale.requests.length; await stale.advance(6000);
    assert.strictEqual(stale.requests.length, count, 'hidden page does not poll');
    stale.document.hidden = false; stale.document.emit('visibilitychange'); await flush();
    assert.strictEqual(stale.requests.length, count + 1);
    const replacement = stale.attach();
    assert.strictEqual(stale.window.handlers.get('pagehide').size, 1, 'reattach removes prior global handlers');
    assert.strictEqual(stale.document.handlers.get('visibilitychange').size, 1);
    stale.window.emit('pagehide'); await flush();
    assert.ok(stale.pending('/status').length === 0, 'pagehide aborts outstanding requests');
    const afterHide = stale.requests.length;
    await stale.advance(7000); stale.document.emit('visibilitychange'); replacement.refresh();
    assert.strictEqual(stale.requests.length, afterHide, 'disposed widget cannot restart');

    const labels = h.window.RobotBehavior;
    const split = harness(); widget = split.attach();
    await split.respond('/status', status({preview_branch:'fall', branches:{
        fall:{enabled:true,state:'valid',ready:true,fresh:true,label:'no_person'},
        fight:{enabled:true,state:'warming',ready:false,fresh:false,
            capture_window:{samples:5,required_samples:8}}
    }}));
    assert.strictEqual(branch(split,'fight'), '采集连续画面 5/8');
    await split.advance(500);
    await split.respond('/status', status({branches:{
        fall:{enabled:true,state:'valid',ready:true,fresh:true,label:'no_person'},
        fight:{enabled:true,state:'warming',ready:false,fresh:false,stale_results:3}
    }}));
    assert.strictEqual(branch(split,'fight'), '推理结果已过期（处理过慢）');
    await split.advance(500);
    await split.respond('/status', status({fresh:false, ready:false, branches:{
        fall:{enabled:true,state:'valid',ready:true,fresh:true,label:'no_person'},
        fight:{enabled:true,state:'valid',ready:true,fresh:true,label:'other'}
    }}));
    assert.ok(!branch(split,'fall').startsWith('就绪'), 'preview expiration cannot keep a cached branch ready');
    widget.dispose();
    assert.strictEqual(labels.categoryLabel('lying'), '躺卧姿态（未确认跌倒）');
    assert.strictEqual(labels.categoryLabel('smoking'), '疑似吸烟');
    assert.strictEqual(labels.evidenceLabel({category:'smoking', confidence:.4}), '人物关联检测分数 40%（非行为概率）');
    assert.ok(source.includes('data-behavior-toggle="smoking"'));
    assert.strictEqual(labels.evidenceLabel({category:'fall', confidence:null}), '连续姿态规则证据');
    assert.strictEqual(labels.evidenceLabel({category:'fight',confidence:0.89}), '模型分数 89%');
    const ready = {running:true, yolo:{running:true,ready:true}, thermal:{ready:true},behavior:status()};
    assert.ok(labels.recognitionLabel(ready).includes('已就绪'));
    assert.ok(labels.recognitionLabel({...ready, yolo:{running:true,ready:false}}).includes('启动中'));
    assert.ok(labels.recognitionLabel({...ready, yolo:{running:false,error:'failed'}}).includes('部分不可用'), 'later visual-process failure cannot advertise everything ready');
    assert.ok(labels.recognitionLabel({...ready, behavior:warming}).includes('启动中'));

    for (const name of ['camera.html','inspection.html']) {
        const html = fs.readFileSync(path.join(web,name),'utf8');
        assert.ok(html.includes('behavior_widget.js') && html.includes('btn-behavior-view'));
        assert.ok(html.includes("=== 'behavior') return;"), `${name} ordinary MJPEG path is bypassed during behavior view`);
        assert.ok(html.includes('behaviorWidget.dispose()'));
        assert.ok(html.includes('/api/thermal/stream.mjpg'), 'thermal preview remains independent');
        new Function(html.slice(html.lastIndexOf('<script>') + 8, html.lastIndexOf('</script>')));
    }
    console.log('behavior frontend: independent controls, honest readiness, exact frames, bounded retry, source races, cleanup and no motion: OK');
})().catch(error => { console.error(error); process.exitCode = 1; });
