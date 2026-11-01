/* Real browser rendering against fully mocked HTTP/rosbridge, no hardware. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const web = path.resolve(__dirname, '../car_vel/web_frontend');
const body = {generation:1,active_body:'xingji',connected:true,
    config:{selected:'xingji',profiles:{xingji:{navigation_verified:true}}},
    status:{stale:false,emergency_latched:false,hardware_error:false}};
const sample = changes => ({schema_version:2,body_id:'xingji',navigation_mode:'terrain_3d',terrain_guard_enabled:true,
    navigation_active:true,source:'nav',blocked:false,warning:false,input_fresh:true,input_age_sec:.02,
    request_velocity:[.25,0,.1],commanded_velocity:[.25,0,.1],speed_limit_mps:.25,
    speed_policy:'nav2_body_limits',speed_limit_source:'nav2_body_limits',terrain_type:'curb',reason_code:'forwarded',...changes});
(async () => {
    const browser = await chromium.launch({headless:true,
        ...(process.env.NAV_TEST_BROWSER ? {executablePath:process.env.NAV_TEST_BROWSER} : {})});
    try {
        for (const filename of ['navigation.html','campus_navigation.html','inspection.html']) {
            const page = await browser.newPage({viewport:{width:1366,height:900}});
            page.setDefaultTimeout(10000);
            const sockets = [], errors = [];
            page.on('pageerror', error => errors.push(error.message));
            await page.routeWebSocket('**/*', socket => {
                socket.onMessage(raw => {
                    const request = JSON.parse(String(raw));
                    const send = (topic,msg) => socket.send(JSON.stringify({op:'publish',topic,msg}));
                    if (request.op === 'subscribe') {
                        if (request.topic === '/navigation/velocity_guard') {
                            sockets.push(socket); send(request.topic,{data:JSON.stringify(sample())});
                        }
                        if (request.topic === '/navigation/mode') send(request.topic,{data:JSON.stringify({mode:'terrain_3d'})});
                        if (request.topic === '/robot_body/status') send(request.topic,{data:JSON.stringify(body)});
                        if (request.topic === '/automatic_terrain_navigation/status') send(request.topic,{data:JSON.stringify({safe:true,state:'ready',current_mode:'obstacle',
                            speed_limit_mps:.25,speed_policy:'nav2_body_limits',execution_policy:'fixed_obstacle',navigation_mode:'terrain_3d',
                            reason:'星玑保持越障模式；速度由Nav2与本体上限控制'})});
                    }
                    if (request.op === 'call_service') socket.send(JSON.stringify({op:'service_response',id:request.id,
                        service:request.service,result:true,values:{success:false,message:'Read-only UI fixture; motion disabled'}}));
                });
            });
            await page.route('**/*', route => {
                const url = new URL(route.request().url());
                if (url.hostname !== 'nav.test') return route.abort();
                if (url.pathname.startsWith('/api/')) return route.fulfill({json:{success:true,running:false,ready:false,enabled:{fall:false,fight:false},branches:{},thermal:{ready:false},events:[]}});
                const file = path.resolve(web,'.'+url.pathname);
                if (!file.startsWith(web+path.sep) || !fs.existsSync(file)) return route.fulfill({status:404});
                const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'};
                return route.fulfill({contentType:types[path.extname(file)]||'application/octet-stream',body:fs.readFileSync(file)});
            });
            await page.goto('http://nav.test/'+filename);
            if (filename === 'inspection.html') await page.locator('[data-tab="progress"]').click();
            const card=page.locator('#navigation-motion-status');
            await page.waitForFunction(() => document.querySelector('[data-speed-command]')?.textContent==='0.25 m/s');
            await card.scrollIntoViewIfNeeded();
            assert(await card.isVisible());
            assert(await card.evaluate(el=>el.scrollWidth<=el.clientWidth+1),'speed card overflows panel');
            assert((await card.innerText()).includes('下发指令不等于本体实测速度'));
            assert((await card.locator('[data-speed-terrain]').innerText()).includes('低台阶'));
            const publish = value => sockets.forEach(socket=>socket.send(JSON.stringify({op:'publish',topic:'/navigation/velocity_guard',msg:{data:JSON.stringify(value)}})));
            publish(sample({input_fresh:false,request_velocity:null,source:'zero',commanded_velocity:[0,0,0],reason_code:'upstream_stale',stream_reason:'原始Nav2控制器断流'}));
            await page.waitForFunction(()=>document.querySelector('#velocity-guard-reason').textContent.includes('原始Nav2控制器断流'));
            assert.equal(await card.locator('[data-speed-request]').innerText(),'--');
            assert.equal(await card.locator('[data-speed-command]').innerText(),'0.00 m/s');
            if (process.env.NAV_TEST_SCREENSHOTS) {
                fs.mkdirSync(process.env.NAV_TEST_SCREENSHOTS,{recursive:true});
                await card.screenshot({path:path.join(process.env.NAV_TEST_SCREENSHOTS,filename.replace('.html','')+'-speed.png')});
            }
            await page.waitForFunction(()=>document.querySelector('[data-speed-state]').textContent.startsWith('速度状态已过期'));
            for (const key of ['request','command','limit']) assert.equal(await card.locator(`[data-speed-${key}]`).innerText(),'--');
            publish(sample({body_id:'gangben_zsl1',speed_policy:'terrain_adaptive'}));
            await page.waitForFunction(()=>document.querySelector('#navigation-motion-status').hidden);
            assert.deepEqual(errors,[],filename+' JS errors');
            await page.close();
        }
        console.log('navigation speed browser: all three pages, explicit stop causes, expiry and scoped desktop layout OK');
    } finally { await browser.close(); }
})().catch(error=>{console.error(error);process.exitCode=1;});
