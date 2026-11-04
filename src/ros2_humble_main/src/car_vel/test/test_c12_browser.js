/* Unified video layout, shared controls and lifecycle; no robot I/O. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const root = path.resolve(__dirname, '../car_vel/web_frontend');
const png = Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+a9foAAAAASUVORK5CYII=', 'base64');

(async () => {
    const browser = await chromium.launch({headless:true,args:['--no-proxy-server'],
        ...(process.env.CAMERA_TEST_BROWSER ? {executablePath:process.env.CAMERA_TEST_BROWSER} : {})});
    try {
        const context = await browser.newContext({viewport:{width:1440,height:1000}});
        await context.addInitScript(() => Object.defineProperty(document,'hidden',{configurable:true,value:false}));
        let pipeline = 'legacy', offline = false, requests = [];
        const errors = [];
        await context.route('**/*', async route => {
            const url = new URL(route.request().url());
            assert.equal(url.hostname, 'c12.test');
            assert.equal(route.request().method(),'GET');
            if (url.pathname === '/api/display/status') return route.fulfill({json:{pipeline,
                label:pipeline==='c12'?'C12 双光相机':'原有相机链路',
                visible_label:pipeline==='c12'?'C12 可见光':'普通摄像头',
                infrared_label:pipeline==='c12'?'C12 红外':'海康热成像',
                thermal:{supports_temperature:pipeline!=='c12',max_temperature_c:pipeline==='c12'?null:65}}});
            if (url.pathname === '/api/display/stream') {
                requests.push(url.searchParams.get('source'));
                if (offline) return route.fulfill({status:503,json:{error:'camera offline'}});
                return route.fulfill({contentType:'image/png',body:png});
            }
            const file = path.join(root,url.pathname);
            if (!fs.existsSync(file)) return route.fulfill({status:404,body:''});
            if (url.pathname.endsWith('.js') && !['robot_ui_common.js','c12_camera.js'].some(name=>url.pathname.endsWith('/'+name))) {
                return route.fulfill({contentType:'application/javascript',body:''});
            }
            if (file.endsWith('.html')) return route.fulfill({contentType:'text/html',
                body:fs.readFileSync(file,'utf8').replace(/<script>([\s\S]*?)<\/script>/g,'')});
            return route.fulfill({path:file});
        });
        const page = await context.newPage();
        page.on('pageerror',error=>errors.push(error.message));
        for (const [file,original] of [['camera.html','#camera-feed'],['remote_control.html','#camera-stream'],
                ['inspection.html','#camera-image'],['mapping.html','#mapping-camera-stream']]) {
            await page.goto(`http://c12.test/${file}`);
            await page.evaluate(()=>window.RobotUi.previewControls());
            assert.equal(await page.locator(original).count(),1);
            assert.equal(await page.locator('[data-c12-camera],.c12-camera').count(),0);
            assert.equal(await page.locator('[data-camera-channel]').count(),2);
            await page.locator('#web-preview-streams').selectOption('thermal');
            assert(await page.locator('[data-camera-channel=ordinary]').isHidden());
            // Mapping retains its own explicit camera on/off toggle.
            if(file!=='mapping.html') assert(await page.locator('[data-camera-channel=thermal]').isVisible());
            assert.equal(await page.locator('a[href="settings.html#ordinary-camera-title"]').count(),1);
        }
        await page.goto('http://c12.test/c12.html');
        await page.waitForFunction(()=>document.querySelectorAll('img.ready').length===1);
        assert.equal(await page.locator('img[src*="source=raw"]').count(),0);
        assert.equal(await page.locator('img[src*="source=thermal"]').count(),1);
        await page.locator('#web-preview-streams').selectOption('both');
        await page.waitForFunction(()=>document.querySelectorAll('img.ready').length===2);
        assert(requests.includes('raw') && requests.includes('thermal'));
        const second = await context.newPage();
        second.on('pageerror',error=>errors.push(error.message));
        await second.goto('http://c12.test/c12.html');
        await second.waitForFunction(()=>document.querySelectorAll('img.ready').length===2);
        await second.locator('#web-preview-streams').selectOption('thermal');
        await page.waitForFunction(()=>document.querySelector('#web-preview-streams').value==='thermal'
            && !document.querySelector('#standalone-visible').hasAttribute('src'));
        pipeline='c12';
        for (const target of [page,second]) {
            await target.waitForFunction(()=>document.querySelector('[data-camera-label=thermal]').textContent==='C12 红外');
            assert.match(await target.locator('#infrared-measurement').innerText(),/测温数据未接入/);
            assert.doesNotMatch(await target.locator('#infrared-measurement').innerText(),/65|0\.0/);
        }
        await page.setViewportSize({width:390,height:844});
        assert(await page.locator('main').evaluate(el=>el.scrollWidth<=el.clientWidth+1));
        offline=true;
        await page.locator('#web-preview-streams').selectOption('both');
        await page.waitForFunction(()=>document.querySelector('#infrared-message').textContent.includes('暂不可用'));
        offline=false;
        await page.waitForFunction(()=>document.querySelectorAll('img.ready').length===2);
        await page.evaluate(()=>window.dispatchEvent(new Event('pagehide')));
        assert.equal(await page.locator('main img[src]').count(),0);
        await page.evaluate(()=>window.dispatchEvent(new Event('pageshow')));
        await page.waitForFunction(()=>document.querySelectorAll('img.ready').length===2);
        await page.locator('#web-preview-streams').selectOption('off');
        await second.waitForFunction(()=>document.querySelector('select').value==='off');
        const before=requests.length;
        await page.waitForTimeout(2300);
        assert.equal(requests.length,before);
        assert.equal(await second.locator('main img[src]').count(),0);
        assert.deepEqual(errors,[]);
        console.log('PASS: four original layouts; shared source/channel controls; C12 null temperatures; reconnect/mobile/lifecycle');
    } finally { await browser.close(); }
})().catch(error=>{console.error(error);process.exitCode=1;});
