/* Headless UI integration with mocked camera APIs; no robot or external access. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const web = path.resolve(__dirname, '../car_vel/web_frontend');
const png = Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aY1sAAAAASUVORK5CYII=', 'base64');

(async () => {
    const browser = await chromium.launch({headless:true,
        ...(process.env.CAMERA_TEST_BROWSER ? {executablePath:process.env.CAMERA_TEST_BROWSER} : {})});
    try {
        const page = await browser.newPage({viewport:{width:1366,height:900}});
        const errors = [], writes = [];
        page.on('pageerror', error => errors.push(error.message));
        await page.routeWebSocket('**/*', socket => socket.close());
        const enabled = {fall:true, fight:true};
        const behavior = () => ({success:true,running:true,ready:true,fresh:true,source_epoch:1,
            stamp_ns:1000000001,frame_stamp_ns:'1000000001',frame_age_seconds:0.01,max_result_age_seconds:1,
            enabled:{...enabled},branches:Object.fromEntries(['fall','fight'].map(key => [key,{
                enabled:enabled[key],state:enabled[key]?'valid':'disabled',ready:enabled[key],
                label:key==='fall'?'no_person':'other'}])),speech:{ready:false,error:'测试：未安装本地语音'}});
        const thermal = {ready:true,max_temperature_c:25,frame_age_seconds:0.01,manual_detection_enabled:true,
            config:{threshold_c:60,minimum_c:-40,maximum_c:550,version:1}};
        const yolo = {running:true,ready:true};
        await page.route('**/*', async route => {
            const url = new URL(route.request().url());
            if (url.hostname !== 'camera.test') return route.abort();
            if (route.request().method() === 'POST') {
                writes.push({path:url.pathname,body:route.request().postDataJSON()});
                assert.equal(url.pathname, '/api/behavior/config', 'UI test must never trigger robot control');
                const value = writes.at(-1).body;
                enabled.fall = value.fall_enabled; enabled.fight = value.fight_enabled;
                return route.fulfill({json:behavior()});
            }
            if (url.pathname === '/api/behavior/status') return route.fulfill({json:behavior()});
            if (url.pathname === '/api/behavior/frame.jpg') return route.fulfill({contentType:'image/png',body:png,
                headers:{'X-Behavior-Source-Epoch':'1','X-Behavior-Stamp-Ns':'1000000001','Access-Control-Allow-Origin':'*',
                    'Access-Control-Expose-Headers':'X-Behavior-Source-Epoch, X-Behavior-Stamp-Ns'}});
            if (url.pathname === '/api/status' || url.pathname === '/api/inspection/state') {
                return route.fulfill({json:{success:true,connected:true,running:true,ready:true,mode:'manual',
                    camera_source:{generation:1,label:'USB摄像头'},video:{device:'/dev/video2',backend:'v4l2',width:640,height:480},
                    yolo,thermal,behavior:behavior(),detection:{running:true,ready:true,yolo,thermal,behavior:behavior()},
                    recognition:{},fixed_view:true}});
            }
            if (url.pathname.startsWith('/api/preview/') || /stream|snapshot/.test(url.pathname)) {
                return route.fulfill({contentType:'image/png',body:png});
            }
            if (url.pathname.startsWith('/api/')) return route.fulfill({json:{success:true,events:[]}});
            const file = path.resolve(web, '.' + url.pathname);
            if (!file.startsWith(web + path.sep) || !fs.existsSync(file)) return route.fulfill({status:404});
            const types = {'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'};
            return route.fulfill({contentType:types[path.extname(file)] || 'application/octet-stream',body:fs.readFileSync(file)});
        });
        for (const [filename, id] of [['camera.html','camera-behavior'],['inspection.html','inspection-behavior']]) {
            await page.goto('http://camera.test/' + filename);
            const card = page.locator('#' + id);
            await page.waitForFunction(id => document.querySelector('#'+id+' [data-behavior-state]').textContent === '就绪', id);
            await card.scrollIntoViewIfNeeded();
            assert(await card.isVisible());
            assert(await card.evaluate(el => el.scrollWidth <= el.clientWidth + 1), 'behavior controls overflow');
            assert((await card.innerText()).includes('未安装本地语音'));
            const fall = card.locator('[data-behavior-toggle="fall"]');
            await fall.setChecked(false);
            await page.waitForFunction(id => document.querySelector('#'+id+' [data-behavior-branch="fall"]').textContent.includes('未启用'), id);
            await fall.setChecked(true);
            await page.locator('#btn-behavior-view').click();
            const image = page.locator(filename === 'camera.html' ? '#camera-feed' : '#camera-image');
            await page.waitForFunction(selector => document.querySelector(selector)?.getAttribute('src')?.startsWith('blob:'),
                filename === 'camera.html' ? '#camera-feed' : '#camera-image');
            if (process.env.CAMERA_TEST_SCREENSHOTS) {
                fs.mkdirSync(process.env.CAMERA_TEST_SCREENSHOTS,{recursive:true});
                await card.screenshot({path:path.join(process.env.CAMERA_TEST_SCREENSHOTS,filename.replace('.html','')+'-behavior.png')});
            }
            assert(await image.getAttribute('src'));
        }
        assert.equal(writes.length,4);
        assert.deepEqual(errors,[]);
        console.log('behavior browser: both pages, switches, speech notice, exact preview and desktop layout OK');
    } finally { await browser.close(); }
})().catch(error => {console.error(error); process.exitCode=1;});
