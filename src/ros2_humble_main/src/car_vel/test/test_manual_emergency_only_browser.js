/* Full navigation-to-remote lifecycle. All HTTP/WS are in-memory mocks. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const web = path.resolve(__dirname, '../car_vel/web_frontend');

(async () => {
    const browser = await chromium.launch({headless: true,
        ...(process.env.NAV_TEST_BROWSER ? {executablePath: process.env.NAV_TEST_BROWSER} : {})});
    try {
        const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
        page.setDefaultTimeout(10000);
        const wire = [], errors = [];
        let socket;
        page.on('pageerror', error => errors.push(error.message));
        const state = {connected: true, stale: false, battery: 85, motion_mode: 2,
            emergency_latched: false, hardware_emergency_latched: false, hardware_error: false};
        const body = {active_body: 'xingji', generation: 1, connected: true,
            config: {selected: 'xingji', profiles: {xingji: {navigation_verified: true}}}, status: state};
        const publish = (topic, msg) => socket.send(JSON.stringify({op: 'publish', topic, msg}));
        await page.routeWebSocket('**/*', mock => {
            mock.onMessage(raw => {
                const message = JSON.parse(String(raw));
                // Display clouds now use their own socket. Publish fake
                // control/status updates on the control socket, not the last
                // opened (possibly display-only) connection.
                if (!String(message.topic || '').startsWith('/web_preview/')) socket = mock;
                wire.push(message);
                if (message.op === 'call_service') {
                    mock.send(JSON.stringify({op: 'service_response', id: message.id,
                        service: message.service, result: true,
                        values: {success: true, message: 'offline fixture', map_names: []}}));
                }
                if (message.op === 'subscribe') {
                    const initial = {
                        '/navigation/mode': {data: JSON.stringify({mode: 'terrain_3d'})},
                        '/robot_body/status': {data: JSON.stringify(body)},
                        '/dog_status': {data: JSON.stringify(state)},
                        '/navigation_active': {data: false},
                        '/nav2_ready': {data: true}, '/goal_pose_status': {data: 'ready'},
                    };
                    if (initial[message.topic]) publish(message.topic, initial[message.topic]);
                }
            });
        });
        await page.route('**/*', async route => {
            const url = new URL(route.request().url());
            if (url.hostname !== 'navigation.test') return route.abort();
            const filename = path.resolve(web, '.' + url.pathname);
            if (!filename.startsWith(web + path.sep) || !fs.existsSync(filename)) return route.fulfill({status: 404});
            const types = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
                '.css': 'text/css; charset=utf-8', '.png': 'image/png'};
            return route.fulfill({contentType: types[path.extname(filename)] || 'application/octet-stream',
                body: fs.readFileSync(filename)});
        });
        await page.goto('http://navigation.test/navigation.html');
        await page.waitForFunction(() => document.querySelector('#log-area').textContent.includes('ROS2'));
        await page.waitForFunction(() => document.querySelector('#navigation-goal-readiness').textContent.includes('请先加载地图'));
        publish('/route_execution_status', {mode: 'single', state: 'navigating', active: true,
            paused: false, point_index: 0, point_count: 1});
        publish('/navigation_active', {data: true});
        await page.waitForFunction(() => document.querySelector('#navigation-goal-readiness').textContent.includes('任务正在执行或保持中'));
        const before = wire.length;
        await page.locator('a[href$="remote_control.html"]').first().click();
        await page.waitForFunction(() => document.querySelector('#log-area')?.textContent.includes('独立遥控模式已开启'));
        const switchWire = wire.slice(before);
        assert(switchWire.some(item => item.op === 'publish' && item.topic === '/cancel_navigation'));
        assert(switchWire.some(item => item.op === 'publish' && item.topic === '/cmd_vel_nav'
            && item.msg.linear.x === 0 && item.msg.angular.z === 0));
        assert(!switchWire.some(item => item.op === 'publish' && item.topic === '/emergency_stop'));
        assert(!switchWire.some(item => item.op === 'publish' && item.topic === '/motion_stop'),
            'ordinary page handoff must not require a fault reset');
        const manual = () => wire.filter(item => item.op === 'publish' && item.topic === '/dog_remote/command'
            && JSON.parse(item.msg.data).command === 'emergency');
        assert.equal(manual().length, 0);
        const stopCore = switchWire.findIndex(item => item.op === 'call_service' && item.service === '/control_launch');
        const enable = switchWire.findIndex(item => item.op === 'publish' && item.topic === '/dog_remote/enable' && item.msg.data);
        assert(stopCore >= 0 && enable > stopCore, 'remote authority still follows backend stop acknowledgement');
        await page.keyboard.press('t');
        await page.waitForTimeout(80);
        assert.equal(manual().length, 1, 'T must retain the explicit native emergency request');
        await page.locator('button[data-command="emergency"]').click();
        await page.waitForTimeout(80);
        assert.equal(manual().length, 2, 'the emergency button must retain the same request');
        await page.keyboard.press('x');
        assert.equal(manual().length, 2, 'normal X stop must not become hardware emergency');
        publish('/dog_status', {data: JSON.stringify({...state, emergency_latched: true,
            hardware_emergency_latched: false, stop_kind: 'zero_speed_latched'})});
        await page.waitForFunction(() => document.querySelector('#hardware-detail').textContent.includes('零速停车已锁定'));
        assert(!await page.locator('#hardware-detail').textContent().then(text => text.includes('人工硬急停已锁定')));
        const legacy = {...state, emergency_latched: true};
        delete legacy.hardware_emergency_latched;
        publish('/dog_status', {data: JSON.stringify(legacy)});
        await page.waitForFunction(() => document.querySelector('#hardware-detail').textContent.includes('停止类型未报告'));
        assert(!await page.locator('#hardware-detail').textContent().then(text => text.includes('未发送硬急停')));
        assert.deepEqual(errors, []);
        if (process.env.NAV_TEST_SCREENSHOT) await page.screenshot({path: process.env.NAV_TEST_SCREENSHOT});
        console.log('PASS: full single-navigation page handoff zero-cancels; only remote T/button emit emergency; X and soft status verified; no real I/O');
    } finally {
        await browser.close();
    }
})().catch(error => {console.error(error); process.exitCode = 1;});
