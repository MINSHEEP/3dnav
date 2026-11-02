/* Actual settings/camera pages; all HTTP/ROS are mocked, no hardware or motion. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const web = path.resolve(__dirname, '../car_vel/web_frontend');
const keys = ['fire','smoke','face','plate','fall','fight','smoking','thermal','m701','co2_alarm'];
(async () => {
    const browser = await chromium.launch({headless:true,
        ...(process.env.CAMERA_TEST_BROWSER ? {executablePath:process.env.CAMERA_TEST_BROWSER} : {})});
    try {
        const context = await browser.newContext({viewport:{width:1440,height:1000}});
        const errors = [], writes = [];
        const state = {schema:1,version:0,enabled:Object.fromEntries(keys.map(k=>[k,true]))};
        let offline = false;
        const policy = () => ({success:true,...state,effective:{...state.enabled,co2_alarm:state.enabled.co2_alarm&&state.enabled.m701}});
        const behavior = () => ({success:true,enabled:Object.fromEntries(['fall','fight','smoking'].map(k=>[k,state.enabled[k]])),
            settings_version:state.version, running:['fall','fight','smoking'].some(k=>state.enabled[k]),
            ready:true,fresh:true,source_epoch:'pose:1',stamp_ns:1000000001,frame_stamp_ns:'1000000001',
            max_result_age_seconds:5,frame_age_seconds:0,
            branches:Object.fromEntries(['fall','fight','smoking'].map(k=>[k,{enabled:state.enabled[k],
                ready:state.enabled[k],state:state.enabled[k]?'valid':'disabled',label:'other'}]))});
        await context.routeWebSocket('**/*', socket=>socket.close());
        await context.route('**/*', async route => {
            const url = new URL(route.request().url());
            if (url.hostname !== 'settings.test') return route.abort();
            if (url.pathname === '/api/detection/settings' || url.pathname === '/api/behavior/config') {
                if (offline) return route.fulfill({status:503,json:{error:'offline fixture'}});
                if (route.request().method() === 'POST') {
                    const value=route.request().postDataJSON(); writes.push({path:url.pathname,value});
                    if (value.expected_version !== state.version) return route.fulfill({status:409,json:{error:'配置已变化，请刷新'}});
                    const changes=url.pathname.endsWith('/config')
                        ? Object.fromEntries(['fall','fight','smoking'].map(k=>[k,value[k+'_enabled']])) : value.changes;
                    assert(Object.keys(changes).every(k=>keys.includes(k)));
                    assert(Object.values(changes).every(v=>typeof v==='boolean'));
                    Object.assign(state.enabled,changes);state.version++;
                    return route.fulfill({json:url.pathname.endsWith('/config')?behavior():policy()});
                }
                return route.fulfill({json:policy()});
            }
            if (route.request().method() === 'POST') throw new Error('Unexpected write '+url.pathname);
            if (url.pathname === '/api/behavior/status') return route.fulfill({json:behavior()});
            const thermal={ready:true,configured_enabled:state.enabled.thermal,max_temperature_c:25,
                config:{threshold_c:60,minimum_c:-40,maximum_c:550,version:1}};
            const yolo={running:true,ready:true};
            if (url.pathname === '/api/status') return route.fulfill({json:{success:true,connected:true,
                video:{device:'/dev/video2',backend:'v4l2',width:640,height:480},camera_source:{generation:1,label:'USB'},
                yolo,thermal,behavior:behavior(),recognition:{},detection:{running:true,ready:true,yolo,thermal,
                    behavior:behavior(),requirements:{yolo:keys.slice(0,4).some(k=>state.enabled[k]),thermal:state.enabled.thermal}}}});
            if (url.pathname === '/api/thermal/status') return route.fulfill({json:thermal});
            const file=path.resolve(web,'.'+url.pathname);
            if (!file.startsWith(web+path.sep) || !fs.existsSync(file)) return route.fulfill({status:404});
            const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'};
            return route.fulfill({contentType:types[path.extname(file)]||'application/octet-stream',body:fs.readFileSync(file)});
        });
        const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));
        await page.goto('http://settings.test/settings.html');
        await page.waitForFunction(()=>!document.querySelector('#detection-switch-fire').disabled);
        assert.equal(await page.locator('#detection-settings-list input').count(),10);
        assert.equal(writes.length,0,'page load is read-only');
        for (const key of keys) {
            const before={...state.enabled};
            await page.locator('#detection-switch-'+key).uncheck();
            await page.waitForFunction(key=>!document.querySelector('#detection-switch-'+key).disabled,key);
            assert.equal(state.enabled[key],false);
            for (const other of keys.filter(k=>k!==key)) assert.equal(state.enabled[other],before[other]);
        }
        assert.equal(writes.length,10);
        await page.reload();
        await page.waitForFunction(()=>!document.querySelector('#detection-switch-fire').disabled);
        for (const key of keys) assert(!(await page.locator('#detection-switch-'+key).isChecked()),'saved '+key+' survives reload');

        // Another editor changes the revision between this page's read/write.
        state.enabled.fall=true;state.version++;
        await page.locator('#detection-switch-fire').check();
        await page.waitForFunction(()=>document.querySelector('#detection-settings-result').textContent.includes('未确认'));
        await page.waitForFunction(()=>!document.querySelector('#detection-switch-fire').disabled);
        assert(!(await page.locator('#detection-switch-fire').isChecked()),'conflicting write was not applied');
        assert(await page.locator('#detection-switch-fall').isChecked());

        offline=true;
        await page.locator('#detection-settings-refresh').click();
        await page.waitForFunction(()=>document.querySelector('#detection-switch-fire').disabled);
        assert.equal(await page.locator('#detection-settings-list input:disabled').count(),10);
        offline=false;await page.locator('#detection-settings-refresh').click();
        await page.waitForFunction(()=>!document.querySelector('#detection-switch-fire').disabled);
        assert(!(await page.locator('#detection-settings-result').innerText()).includes('读取失败'),'recovery clears stale error');

        // Existing behavior controls write the same policy; the settings page follows.
        const camera=await context.newPage();camera.on('pageerror',e=>errors.push(e.message));
        await camera.goto('http://settings.test/camera.html');
        await camera.waitForFunction(()=>!document.querySelector('[data-behavior-toggle="fall"]').disabled);
        await camera.locator('[data-behavior-toggle="fall"]').uncheck();
        await camera.waitForFunction(()=>!document.querySelector('[data-behavior-toggle="fall"]').disabled);
        assert.equal(state.enabled.fall,false);
        await page.bringToFront();await page.locator('#detection-settings-refresh').click();
        await page.waitForFunction(()=>!document.querySelector('#detection-switch-fall').checked);
        const card=page.locator('section[aria-labelledby="detection-settings-title"]');
        await card.scrollIntoViewIfNeeded();
        assert(await card.evaluate(el=>el.scrollWidth<=el.clientWidth+1));
        if(process.env.DETECTION_SCREENSHOTS){fs.mkdirSync(process.env.DETECTION_SCREENSHOTS,{recursive:true});
            await card.screenshot({path:path.join(process.env.DETECTION_SCREENSHOTS,'detection-settings-desktop.png')});}
        await page.setViewportSize({width:390,height:900});await card.scrollIntoViewIfNeeded();
        assert(await card.evaluate(el=>el.scrollWidth<=el.clientWidth+1),'mobile controls do not overflow');
        if(process.env.DETECTION_SCREENSHOTS)await card.screenshot({path:path.join(process.env.DETECTION_SCREENSHOTS,'detection-settings-mobile.png')});
        assert.deepEqual(errors,[]);
        assert(writes.every(item=>['/api/detection/settings','/api/behavior/config'].includes(item.path)),'no robot/camera/power actions');
        console.log('Detection switches: 10 independent saves, reload, conflicts, offline recovery, cross-page sync and desktop/mobile OK');
    } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
