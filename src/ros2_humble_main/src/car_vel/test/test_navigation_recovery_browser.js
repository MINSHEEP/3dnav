/* Full-page offline smoke test: all HTTP and WebSocket traffic is mocked.
 * Requires Playwright and an installed browser. Never contacts a robot.
 */
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
        const errors = [], wire = [], subscriptions = new Set();
        let socket;
        page.on('pageerror', error => errors.push(error.message));
        const body = {generation: 1, active_body: 'xingji', connected: true,
            config: {selected: 'xingji', profiles: {xingji: {navigation_verified: true}}},
            status: {stale: false, emergency_latched: true, hardware_error: false}};
        const publish = (topic, msg) => socket.send(JSON.stringify({op: 'publish', topic, msg}));
        await page.routeWebSocket('**/*', mock => {
            socket = mock;
            mock.onMessage(raw => {
                const message = JSON.parse(String(raw));
                wire.push(message);
                if (message.op === 'call_service') {
                    const values = message.service === '/dog_remote/ensure_standing'
                        ? {success: false, message: '离线测试：急停锁定，拒绝自动站立'}
                        : {success: true, message: 'offline fixture', map_names: ['fixture_map']};
                    mock.send(JSON.stringify({op: 'service_response', id: message.id,
                        service: message.service, result: true, values}));
                }
                if (message.op === 'subscribe') {
                    subscriptions.add(message.topic);
                    const initial = {
                        '/navigation/mode': {data: JSON.stringify({mode: 'map_2d'})},
                        '/robot_body/status': {data: JSON.stringify(body)},
                        '/navigation_active': {data: false},
                        '/nav2_ready': {data: true},
                        '/goal_pose_status': {data: 'ready'},
                    };
                    if (initial[message.topic]) publish(message.topic, initial[message.topic]);
                }
            });
        });
        await page.route('**/*', async route => {
            const url = new URL(route.request().url());
            if (url.hostname !== 'navigation.test') return route.abort();
            const filename = path.resolve(web, '.' + url.pathname);
            if (!filename.startsWith(web + path.sep) || !fs.existsSync(filename)) {
                return route.fulfill({status: 404});
            }
            const types = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
                '.css': 'text/css; charset=utf-8', '.png': 'image/png'};
            return route.fulfill({contentType: types[path.extname(filename)] || 'application/octet-stream',
                body: fs.readFileSync(filename)});
        });
        await page.goto('http://navigation.test/navigation.html');
        const reset = page.locator('#btn-reset-navigation-emergency');
        await page.waitForFunction(() => !document.querySelector('#btn-reset-navigation-emergency').disabled);
        await page.locator('#btn-nav-view-2d').click();
        assert(await page.locator('#navigation-goal-readiness').isVisible());
        assert.match(await page.locator('#navigation-goal-readiness').innerText(), /请先加载地图/);
        body.status.emergency_latched = false;
        publish('/robot_body/status', {data: JSON.stringify(body)});
        await page.waitForFunction(() => document.querySelector('#btn-reset-navigation-emergency').disabled);
        body.status.emergency_latched = true;
        publish('/robot_body/status', {data: JSON.stringify(body)});
        await page.waitForFunction(() => !document.querySelector('#btn-reset-navigation-emergency').disabled);
        publish('/navigation_active', {data: true});
        await page.waitForFunction(() => document.querySelector('#btn-reset-navigation-emergency').disabled);
        assert.match(await page.locator('#navigation-goal-readiness').innerText(), /任务正在执行或保持中/);
        publish('/navigation_active', {data: false});
        publish('/robot_body/status', {data: JSON.stringify(body)});
        await page.waitForFunction(() => !document.querySelector('#btn-reset-navigation-emergency').disabled);
        const resetCount = () => wire.filter(item => item.op === 'publish'
            && item.topic === '/clear_emergency_stop').length;
        page.once('dialog', dialog => dialog.dismiss());
        await reset.click();
        assert.equal(resetCount(), 0, 'cancel must not reset the body');
        const ensureCount = () => wire.filter(item => item.op === 'call_service'
            && item.service === '/dog_remote/ensure_standing').length;
        const oldEnsureCount = ensureCount();
        page.once('dialog', dialog => dialog.accept());
        await reset.click();
        await page.waitForFunction(() => document.querySelector('#log-area').textContent.includes('本页不会自动站立'));
        assert.equal(resetCount(), 1);
        body.status.emergency_latched = false;
        publish('/robot_body/status', {data: JSON.stringify(body)});
        await page.waitForTimeout(3200);
        assert.equal(ensureCount(), oldEnsureCount, 'reset must not trigger delayed automatic standing');
        assert(!wire.some(item => item.op === 'publish'
            && ['/web_goal_pose', '/emergency_stop', '/cmd_vel_nav'].includes(item.topic)));
        assert(subscriptions.has('/robot_body/status'));
        assert.deepEqual(errors, []);
        if (process.env.NAV_TEST_SCREENSHOT) {
            await page.locator('#navigation-goal-readiness').scrollIntoViewIfNeeded();
            await page.screenshot({path: process.env.NAV_TEST_SCREENSHOT});
        }
        console.log('PASS: full navigation page, 2D body updates, active-task lock, confirmed reset and no automatic standing; no real network');
    } finally {
        await browser.close();
    }
})().catch(error => {console.error(error); process.exitCode = 1;});
