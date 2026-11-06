/* Real inspection page with offline ROS replies; no connection to hardware. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const web = path.resolve(__dirname, '../car_vel/web_frontend');
(async () => {
    const browser = await chromium.launch({headless: true, executablePath: process.env.NAV_TEST_BROWSER || '/usr/bin/google-chrome', args:['--no-sandbox']});
    try {
        const page = await browser.newPage({viewport:{width:1440,height:1000}});
        const errors=[], wire=[]; let socket, ratio=.5;
        page.on('pageerror', error=>errors.push(error.message));
        await page.routeWebSocket('**/*', mock=>{
            socket=mock;
            mock.onMessage(raw=>{
                const m=JSON.parse(String(raw)); wire.push(m);
                if (m.op!=='call_service') return;
                let values={success:true,message:'offline fixture',map_names:[]};
                if(m.service.endsWith('/set_parameters')) {
                    ratio=m.args.parameters[0].value.double_value;
                    values={results:[{successful:true,reason:''}]};
                }
                if(m.service.endsWith('/get_parameters')) values={values:[{type:3,double_value:ratio}]};
                mock.send(JSON.stringify({op:'service_response',id:m.id,service:m.service,result:true,values}));
            });
        });
        await page.route('**/*', async route=>{
            const url=new URL(route.request().url());
            if(url.hostname!=='inspection.test') return route.abort();
            const file=path.resolve(web,'.'+url.pathname);
            if(!file.startsWith(web+path.sep)||!fs.existsSync(file)) return route.fulfill({status:404});
            let body=fs.readFileSync(file);
            if(url.pathname==='/inspection.html') {
                // Set map/idle state without starting a real navigation backend.
                body=body.toString().replace(/\}\)\(\);\s*<\/script>/, `
                    window.scanTest={prepare(){
                        selectedNavigationMode='map_2d';navigationModeSeen=true;selectedModeAtMs=Date.now();
                        loadedMapName='test';currentMapLoadName='test';currentMapLoadId='load-1';
                        mapReady=true;nav2Ready=true;serviceBusy=false;standingConfirmed=true;
                        latestOdometryStamp={sec:10,nanosec:0};
                        mapCanvas.setMap({width:100,height:100,resolution:.1,origin:{position:{x:-5,y:-5,z:0},orientation:{x:0,y:0,z:0,w:1}}},new Array(10000).fill(0));
                        updateUi();
                    }, point(){return mapCanvas.mapToCanvas(1,1)},
                    busy(){taskState={state:'paused',active:true,paused:true};updateUi()},
                    state(){return {draft:inspectionInitialDraft,scan:mapCanvas.scanOverlay}}};
                })();</script>`);
            }
            const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'};
            return route.fulfill({contentType:types[path.extname(file)]||'application/octet-stream',body});
        });
        await page.goto('http://inspection.test/inspection.html');
        await page.waitForFunction(()=>window.scanTest && document.getElementById('connection-status').textContent.includes('已连接'));
        await page.evaluate(()=>window.scanTest.prepare());
        await page.click('#btn-inspection-match-read');
        await page.waitForFunction(()=>document.querySelector('[data-pose-tool="match-hint"]').textContent.includes('50%'));
        await page.fill('#inspection-match-percent','40'); await page.click('#btn-inspection-match-apply');
        await page.waitForFunction(()=>document.querySelector('[data-pose-tool="match-hint"]').textContent.includes('40%'));
        await page.click('#btn-initial-mode');
        await page.waitForTimeout(100);
        socket.send(JSON.stringify({op:'publish',topic:'/localization/scan_preview',msg:{data:JSON.stringify({points:[[1,0],[1,.2]],frame_id:'base_footprint'})}}));
        await page.waitForFunction(()=>window.scanTest.state().scan?.points.length===2);
        const point=await page.evaluate(()=>window.scanTest.point());
        const rect=await page.locator('#map-canvas').boundingBox();
        await page.mouse.move(rect.x+point.x,rect.y+point.y);await page.mouse.down();
        await page.mouse.move(rect.x+point.x+60,rect.y+point.y-30);await page.mouse.up();
        await page.waitForFunction(()=>window.scanTest.state().draft!==null);
        assert(!wire.some(m=>m.op==='publish'&&m.topic==='/relocalization/request'),'drag must remain a preview');
        assert.equal(await page.locator('#btn-inspection-confirm-2d').isEnabled(),true);
        await page.screenshot({path:'/tmp/inspection-scan-preview.png'});
        await page.click('#btn-inspection-confirm-2d');
        await page.waitForFunction(()=>document.getElementById('inspection-initial-hint').textContent.includes('已提交'));
        assert(wire.some(m=>m.op==='publish'&&m.topic==='/relocalization/request'),'confirm must submit the existing coordinator request');
        await page.evaluate(()=>window.scanTest.busy());
        assert.equal(await page.locator('#btn-initial-mode').isDisabled(),true);
        assert.equal(await page.locator('#btn-inspection-match-apply').isDisabled(),true);
        assert.equal(await page.locator('#btn-inspection-confirm-2d').isDisabled(),true);
        await page.setViewportSize({width:1280,height:900});
        await page.waitForTimeout(100);
        assert(await page.locator('#inspection-initial-tools').evaluate(el=>el.scrollWidth<=el.clientWidth+1),'localization controls must fit a narrower desktop');
        assert.deepEqual(errors,[]);
        console.log('PASS: inspection localization toolbar, threshold read/apply, scan alignment, explicit confirmation and paused-task locks');
    } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
