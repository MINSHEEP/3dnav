/* Complete settings page; all ROS/WebSocket/HTTP responses are mocked. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const web = path.resolve(__dirname, '../car_vel/web_frontend');

(async () => {
    const browser = await chromium.launch({headless:true,
        ...(process.env.NAV_TEST_BROWSER ? {executablePath:process.env.NAV_TEST_BROWSER} : {})});
    try {
        const page = await browser.newPage({viewport:{width:1440,height:1000}});
        page.setDefaultTimeout(10000);
        const errors = [], wire = [];
        let socket;
        const state = {schema_version:1,mode:'protected',enabled:true,version:0,load_error:'',
            mapping_running:true,active_mode:'protected',pending:false,apply_policy:'next_mapping_start'};
        const publish = () => socket.send(JSON.stringify({op:'publish',topic:'/mapping/filter/status',
            msg:{data:JSON.stringify(state)}}));
        page.on('pageerror',error=>errors.push(error.message));
        await page.routeWebSocket('**/*',mock=>{
            socket=mock;
            mock.onMessage(raw=>{
                const message=JSON.parse(String(raw));wire.push(message);
                if(message.op==='subscribe'&&message.topic==='/mapping/filter/status')publish();
                if(message.op!=='call_service')return;
                let values={success:true,message:'offline unrelated module',status_json:'{}'};
                if(message.service==='/mapping/filter/get')values={success:true,message:JSON.stringify(state)};
                if(message.service==='/mapping/filter/set_enabled'){
                    assert.strictEqual(typeof message.args.data,'boolean');
                    state.enabled=message.args.data;state.mode=state.enabled?'protected':'bypass';state.version++;
                    state.pending=state.mapping_running&&state.active_mode!==state.mode;
                    publish();values={success:true,message:JSON.stringify(state)};
                }
                mock.send(JSON.stringify({op:'service_response',id:message.id,
                    service:message.service,result:true,values}));
            });
        });
        await page.route('**/*',async route=>{
            const url=new URL(route.request().url());
            if(url.hostname!=='settings.test')return route.abort();
            const filename=path.resolve(web,'.'+url.pathname);
            if(!filename.startsWith(web+path.sep)||!fs.existsSync(filename))return route.fulfill({status:404});
            const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.png':'image/png'};
            return route.fulfill({contentType:types[path.extname(filename)]||'application/octet-stream',body:fs.readFileSync(filename)});
        });
        await page.goto('http://settings.test/settings.html');
        const toggle=page.locator('#mapping-filter-enabled'),save=page.locator('#mapping-filter-save');
        const card=page.locator('section[aria-labelledby="mapping-filter-title"]');
        await page.waitForFunction(()=>!document.querySelector('#mapping-filter-enabled').disabled);
        assert(await toggle.isChecked());assert(await save.isDisabled());
        await toggle.uncheck();assert(await save.isEnabled());await save.click();
        await page.waitForFunction(()=>document.querySelector('#mapping-filter-state-value').textContent.includes('下次建图'));
        assert.match(await page.locator('#mapping-filter-detail').innerText(),/已保存：关闭.*当前建图：开启/);
        assert(!wire.some(message=>message.op==='call_service'&&message.service==='/control_launch'));
        await page.reload();
        await page.waitForFunction(()=>!document.querySelector('#mapping-filter-enabled').disabled);
        assert(!(await toggle.isChecked()),'saved choice survives a full page reload');
        assert.match(await page.locator('#mapping-filter-state-value').innerText(),/下次建图/);
        await card.scrollIntoViewIfNeeded();
        if(process.env.MAPPING_FILTER_SCREENSHOTS)await card.screenshot({path:path.join(process.env.MAPPING_FILTER_SCREENSHOTS,'mapping-filter-desktop.png')});
        await page.setViewportSize({width:390,height:900});
        await card.scrollIntoViewIfNeeded();
        assert(await card.evaluate(element=>element.scrollWidth<=element.clientWidth+1),'new card must not overflow on mobile');
        if(process.env.MAPPING_FILTER_SCREENSHOTS)await card.screenshot({path:path.join(process.env.MAPPING_FILTER_SCREENSHOTS,'mapping-filter-mobile.png')});
        state.mapping_running=false;state.active_mode=null;state.pending=false;publish();
        await page.waitForFunction(()=>document.querySelector('#mapping-filter-state-value').textContent==='已保存：关闭');
        state.mapping_running=true;state.active_mode='bypass';publish();
        await page.waitForFunction(()=>document.querySelector('#mapping-filter-state-value').textContent==='当前建图：关闭');
        socket.close();
        await page.waitForFunction(()=>document.querySelector('#mapping-filter-enabled').disabled);
        assert(await save.isDisabled());
        assert.deepEqual(errors,[]);
        console.log('PASS: complete settings page, save-only next-start switch, reload persistence, current/pending state, offline and desktop/mobile layout; no real I/O');
    }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
