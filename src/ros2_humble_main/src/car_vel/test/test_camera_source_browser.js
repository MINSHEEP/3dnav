/* Optional headless UI smoke test. Requires Playwright; no robot/network access.
 * CAMERA_TEST_BROWSER can select an already installed Chrome/Chromium.
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const web = path.resolve(__dirname, '../car_vel/web_frontend');

(async () => {
    const browser = await chromium.launch({headless:true,args:['--no-proxy-server'],
        ...(process.env.CAMERA_TEST_BROWSER ? {executablePath:process.env.CAMERA_TEST_BROWSER} : {})});
    try {
        const page = await browser.newPage({viewport:{width:1366,height:900}});
        page.setDefaultTimeout(10000);
        const errors = [];
        page.on('pageerror', error => errors.push(error.message));
        page.on('console', message => { if (message.type() === 'error') console.error('browser:',message.text()); });
        // No ROS, UDP, real device or external website is contacted.
        await page.routeWebSocket('**/*', socket => socket.close());
        const config = {version:0,pipeline:'legacy',source:'usb',usb_device:'/dev/v4l/by-id/Sonix-video-index0',
            usb_width:1280,usb_height:720,usb_fps:30,body_url:'http://127.0.0.1:8081/camera'};
        let locked = '', rawRequests = 0, thermalRequests = 0;
        const display = () => ({pipeline:config.pipeline, generation:config.version,
            label:config.pipeline==='c12'?'C12 双光相机':'原有相机链路',
            visible_label:config.pipeline==='c12'?'C12 可见光':'普通摄像头',
            infrared_label:config.pipeline==='c12'?'C12 红外':'海康热成像',
            thermal:config.pipeline==='c12'?{ready:true,supports_temperature:false,max_temperature_c:null}
                :{ready:true,supports_temperature:true,max_temperature_c:25}});

        await page.route('**/*', async route => {
            const url = new URL(route.request().url());
            if (url.hostname !== 'camera.test') return route.abort();
            if (url.pathname === '/api/camera/source') {
                if (route.request().method() === 'POST') {
                    const payload = route.request().postDataJSON();
                    assert.equal(payload.expected_version, config.version);
                    Object.assign(config, payload.config, {version:config.version+1});
                }
                return route.fulfill({json:{success:true,config,devices:[{
                    device:config.usb_device,node:'/dev/video2',aliases:[],name:'Sonix USB Camera'}],
                    locked_reason:locked,switching:false,generation:config.version}});
            }
            if (url.pathname === '/api/display/status') return route.fulfill({json:display()});
            if (url.pathname === '/api/status') return route.fulfill({json:{
                connected:true,display:display(),camera_type:config.source,camera_source:{generation:config.version,
                    label:config.pipeline==='c12'?'C12可见光相机':config.source==='body'?'狗本体摄像头':'USB摄像头'},
                video:{device:config.source==='body'?config.body_url:config.usb_device,
                    backend:config.source==='body'?'http':'v4l2',width:640,height:480,error:null},
                yolo:{running:false},thermal:{ready:true,max_temperature_c:25,frame_age_seconds:0.02,
                    config:{threshold_c:60,minimum_c:-40,maximum_c:550,version:1}},detection:{running:false}}});
            if (url.pathname === '/api/display/stream') {
                assert.equal(url.searchParams.get('profile'),'remote');
                if (url.searchParams.get('source') === 'raw') rawRequests++; else thermalRequests++;
                return route.fulfill({contentType:'image/png',body:Buffer.from(
                    'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aY1sAAAAASUVORK5CYII=', 'base64')});
            }
            const filename = path.resolve(web, '.'+url.pathname);
            if (!filename.startsWith(web+path.sep) || !fs.existsSync(filename)) return route.fulfill({status:404});
            const types = {'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.png':'image/png'};
            return route.fulfill({contentType:types[path.extname(filename)]||'application/octet-stream',body:fs.readFileSync(filename)});
        });
        await page.goto('http://camera.test/settings.html');
        const save = page.locator('#ordinary-camera-save');
        await page.waitForFunction(() => !document.querySelector('#ordinary-camera-save').disabled);
        assert.match(await page.locator('#ordinary-camera-state').innerText(), /USB/);
        await page.locator('#ordinary-camera-source').selectOption('body');
        assert(await page.locator('#ordinary-camera-usb-fields').isHidden());
        assert(await page.locator('#ordinary-camera-body-fields').isVisible());
        assert.equal(await page.locator('#ordinary-camera-url').inputValue(), config.body_url);
        await save.click();
        await page.waitForFunction(() => document.querySelector('#ordinary-camera-state').textContent.includes('狗本体摄像头'));
        assert.equal(config.source, 'body');
        const previousBodyUrl = config.body_url, previousUsbDevice = config.usb_device;
        await page.locator('#ordinary-camera-pipeline').selectOption('c12');
        assert(await page.locator('#ordinary-camera-legacy-fields').isHidden());
        assert(await page.locator('#ordinary-camera-c12-fields').isVisible());
        await save.click();
        await page.waitForFunction(() => document.querySelector('#ordinary-camera-state').textContent.includes('C12'));
        assert.equal(config.pipeline,'c12');
        assert.equal(config.source,'body');
        assert.equal(config.body_url,previousBodyUrl);
        assert.equal(config.usb_device,previousUsbDevice);
        await page.locator('#ordinary-camera-pipeline').selectOption('legacy');
        await save.click();
        await page.waitForFunction(() => document.querySelector('#ordinary-camera-state').textContent.includes('狗本体摄像头'));
        assert.equal(config.pipeline,'legacy');
        const card = page.locator('section[aria-labelledby="ordinary-camera-title"]');
        if (process.env.CAMERA_TEST_SCREENSHOTS) await card.screenshot({path:path.join(process.env.CAMERA_TEST_SCREENSHOTS,'camera-settings-desktop.png')});
        await page.setViewportSize({width:390,height:844});
        assert(await page.locator('#ordinary-camera-url').isVisible());
        assert(await card.evaluate(el => el.scrollWidth <= el.clientWidth+1), 'camera card must not overflow on mobile');
        if (process.env.CAMERA_TEST_SCREENSHOTS) await card.screenshot({path:path.join(process.env.CAMERA_TEST_SCREENSHOTS,'camera-settings-mobile.png')});
        await page.locator('#ordinary-camera-source').selectOption('usb');
        await save.click();
        await page.waitForFunction(() => document.querySelector('#ordinary-camera-state').textContent.includes('USB'));
        assert.equal(config.source,'usb');
        locked = '请先结束巡检任务';
        await page.waitForFunction(() => document.querySelector('#ordinary-camera-save').disabled);
        await page.goto('http://camera.test/camera.html');
        await page.waitForFunction(() => document.querySelector('#camera-top-status').textContent.includes('USB摄像头在线')).catch(async error => {
            console.error('camera diagnostics:',await page.locator('#camera-top-status').innerText(),errors);
            throw error;
        });
        await page.waitForTimeout(300);
        assert(rawRequests>0, '5G default ordinary preview must connect');
        assert.equal(thermalRequests,0,'5G default must not open a second video');
        await page.locator('#web-preview-streams').selectOption('both');
        await page.waitForFunction(() => document.querySelector('#thermal-feed').getAttribute('src')?.includes('source=thermal'));
        await page.waitForTimeout(100);
        const oldRawRequests = rawRequests, oldThermalRequests = thermalRequests;
        config.source='body';config.version++;
        await page.waitForFunction(() => document.querySelector('#camera-top-status').textContent.includes('狗本体摄像头在线'));
        await page.waitForFunction(() => document.querySelector('#camera-log').textContent.includes('来源已更新'));
        await page.waitForTimeout(150);
        assert(rawRequests>oldRawRequests, 'source generation refreshes ordinary preview');
        assert.equal(thermalRequests,oldThermalRequests, 'ordinary source update must not reconnect thermal preview');
        const beforeC12Raw = rawRequests, beforeC12Thermal = thermalRequests;
        config.pipeline='c12';config.version++;
        await page.waitForFunction(() => document.querySelector('#camera-top-status').textContent.includes('C12可见光相机在线'));
        await page.waitForTimeout(150);
        assert(rawRequests>beforeC12Raw,'C12 selection reconnects the ordinary preview');
        await page.waitForFunction(() => document.querySelector('[data-camera-label=thermal]').textContent==='C12 红外');
        await page.waitForTimeout(200);
        assert(thermalRequests>beforeC12Thermal,'C12 selection refreshes infrared preview');
        assert.match(await page.locator('#thermal-feed-badge').textContent(), /测温数据未接入/);
        await page.locator('#web-preview-streams').selectOption('off');
        await page.waitForFunction(() => !document.querySelector('#camera-feed').hasAttribute('src')
            && !document.querySelector('#thermal-feed').hasAttribute('src'));
        await page.setViewportSize({width:1366,height:900});
        for (const name of ['remote_control.html','mapping.html','inspection.html']) {
            await page.evaluate(() => localStorage.setItem('robot.webPreview.streams','ordinary'));
            await page.goto('http://camera.test/'+name);
            if (name==='mapping.html') await page.locator('#camera-toggle').click();
            await page.waitForFunction(() => document.querySelectorAll('img[src*="/api/display/stream"]').length===1);
            assert.equal(await page.locator('img[src*="source=thermal"]').count(),0);
            await page.evaluate(() => {
                Object.defineProperty(document,'hidden',{configurable:true,value:true});
                document.dispatchEvent(new Event('visibilitychange'));
            });
            await page.waitForFunction(() => document.querySelectorAll('img[src*="/api/display/stream"]').length===0);
            await page.evaluate(() => {
                Object.defineProperty(document,'hidden',{configurable:true,value:false});
                document.dispatchEvent(new Event('visibilitychange'));
            });
            await page.waitForFunction(() => document.querySelectorAll('img[src*="/api/display/stream"]').length===1);
            await page.locator('#web-preview-streams').selectOption('off');
            await page.waitForFunction(() => document.querySelectorAll('img[src*="/api/display/stream"]').length===0);
            await page.waitForTimeout(2600);
            assert.equal(await page.locator('img[src*="/api/display/stream"]').count(),0,
                name+' must not reconnect an explicitly disabled stream');
        }
        assert.deepEqual(errors,[]);
        console.log('PASS: original/C12 roundtrip preserves USB/body; responsive layout; four video pages and source refresh; thermal isolated');
    } finally { await browser.close(); }
})().catch(error=>{console.error(error);process.exitCode=1;});
