/* Real browser UI + mocked HTTP/rosbridge. Never contacts real hardware.
 * A test-only closure hook supplies a confirmed map/localization fixture;
 * the production point picker, RPC handlers, status callbacks and DOM run.
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {chromium} = require('playwright');
const web = path.resolve(__dirname, '../car_vel/web_frontend');
const hook = `
window.__refreshFixture = {
    ready: () => session && session.connected && nav2Ready && goalStatusHydrated,
    inspect: () => ({issue:threeDInteractionIssue(),readiness:goalReadinessIssue(),draft:threeDPoseDraft}),
    age: () => { latestAmclReceivedAtMs = Date.now()-6000; updateMapReadyUi(); },
    prepare: async () => {
        loadedMapName='fixture_map'; navigationMapReady=true; pendingMapName='';
        currentMapLoadName=loadedMapName; currentMapLoadId='load-1';
        navigationSafetyReady=true; localizationReady=true;
        localizationHealth=LOCALIZATION_HEALTH.GOOD;
        lioFaultLatched=false; navigationActive=false; relocalizationActive=false;
        latestAmclStamp=Date.now()/1000-6; latestAmclReceivedAtMs=Date.now()-6000;
        latestAmclPose={x:1.2,y:1.2,z:0,yaw:0,stamp:latestAmclStamp};
        const xyz=new Float32Array([0,0,0, 2,0,0, 4,0,0, 0,2,0, 2,2,0, 4,2,0]);
        const cloud={completeSavedMap:true,header:{frame_id:'map',stamp:{sec:1,nanosec:0}},height:1,width:6,
            fields:['x','y','z'].map((name,i)=>({name,offset:i*4,datatype:7,count:1})),
            is_bigendian:false,point_step:12,row_step:72,is_dense:true,
            data:btoa(String.fromCharCode(...new Uint8Array(xyz.buffer)))};
        latestNavigationGlobalCloud=cloud;
        latestNavigationGlobalCloudIdentity={map_name:loadedMapName,load_id:'load-1',stamp_ns:'1000000000'};
        navigationPointCloudViewer.setPointCloud(cloud);
        setNavigationMapView('3d');
        navigationPointCloudViewer.setTopView();
        setClickMode('goal');
        await handleThreeDGroundPose(3.2,1.2,0);
        updateMapReadyUi();
    }
};
`;

(async () => {
    const browser = await chromium.launch({headless:true,
        ...(process.env.NAV_TEST_BROWSER ? {executablePath:process.env.NAV_TEST_BROWSER} : {})});
    let heartbeat;
    try {
        const page = await browser.newPage({viewport:{width:1440,height:1000}});
        page.setDefaultTimeout(10000);
        const errors=[], wire=[];
        let socket, pendingPreview, previews=0, starts=0, sampleIndex=0;
        page.on('pageerror', error=>errors.push(error.message));
        const body={generation:1,active_body:'xingji',connected:true,
            config:{selected:'xingji',profiles:{xingji:{navigation_verified:true}}},
            status:{stale:false,emergency_latched:false,hardware_error:false}};
        const publish=(topic,msg)=>socket.send(JSON.stringify({op:'publish',topic,msg}));
        const status=value=>publish('/goal_pose_status',{data:value});
        const reply=(request,values)=>socket.send(JSON.stringify({op:'service_response',
            id:request.id,service:request.service,result:true,values}));
        const emitSample=()=>{
            sampleIndex++;
            const covariance=Array(36).fill(0);covariance[0]=covariance[7]=covariance[35]=0.0001;
            publish('/amcl_pose',{header:{frame_id:'map',stamp:{sec:Math.floor(Date.now()/1000),nanosec:sampleIndex*10000000}},
                pose:{pose:{position:{x:1.2,y:1.2,z:0},orientation:{x:0,y:0,z:0,w:1}},covariance}});
        };
        await page.routeWebSocket('**/*',mock=>{
            mock.onMessage(raw=>{
                const message=JSON.parse(String(raw));wire.push(message);
                if(!String(message.topic||'').startsWith('/web_preview/'))socket=mock;
                if(message.op==='subscribe'){
                    const initial={
                        '/navigation/mode':{data:JSON.stringify({mode:'terrain_3d',goal_preflight_pose_max_age_sec:3})},
                        '/robot_body/status':{data:JSON.stringify(body)},'/navigation_active':{data:false},
                        '/nav2_ready':{data:true},'/goal_pose_status':{data:'ready'}};
                    if(initial[message.topic])publish(message.topic,initial[message.topic]);
                }
                if(message.op!=='call_service')return;
                if(message.service==='/request_nomotion_update'){
                    emitSample();return reply(message,{});
                }
                if(message.service==='/terrain/check_3d_ground')return reply(message,{valid:true,ground_z:0,message:'fixture ground'});
                if(message.service==='/navigation/preview_3d_goal'){
                    previews++;pendingPreview=message;status('goal_refreshing_3d_localization');return;
                }
                if(message.service==='/navigation/start_previewed_3d_goal'){
                    starts++;
                    if(starts===1){
                        status('goal_refreshing_3d_localization');emitSample();status('goal_3d_localization_refreshed');
                        return reply(message,{success:false,code:'localization_refreshed_repreview',message:'定位已刷新，旧预览已作废；请重新预览并再次人工确认开始'});
                    }
                    return reply(message,{success:true,code:'accepted',message:'fixture accepted after explicit click'});
                }
                reply(message,{success:true,message:'offline fixture',map_names:['fixture_map']});
            });
        });
        await page.route('**/*',async route=>{
            const url=new URL(route.request().url());
            if(url.hostname!=='navigation.test')return route.abort();
            const filename=path.resolve(web,'.'+url.pathname);
            if(!filename.startsWith(web+path.sep)||!fs.existsSync(filename))return route.fulfill({status:404});
            const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.png':'image/png'};
            let content=fs.readFileSync(filename);
            if(url.pathname==='/navigation.html'){
                const original=content.toString('utf8');
                content=original.replace(/\n\}\)\(\);\s*<\/script>/,`\n${hook}\n})();\n</script>`);
                assert.notEqual(content,original,'test-only hook insertion failed');
            }
            return route.fulfill({contentType:types[path.extname(filename)]||'application/octet-stream',body:content});
        });
        await page.goto('http://navigation.test/navigation.html');
        await page.waitForFunction(()=>window.__refreshFixture&&window.__refreshFixture.ready());
        heartbeat=setInterval(()=>{
            publish('/robot_body/status',{data:JSON.stringify(body)});
            publish('/navigation/mode',{data:JSON.stringify({mode:'terrain_3d',goal_preflight_pose_max_age_sec:3})});
        },500);
        await page.evaluate(()=>window.__refreshFixture.prepare());
        assert.match(await page.locator('#navigation-goal-readiness').innerText(),/预览前.*刷新/);
        const preview=page.locator('#btn-three-d-preview-goal'), start=page.locator('#btn-three-d-start-goal');
        assert(!(await preview.isDisabled()), JSON.stringify(await page.evaluate(()=>window.__refreshFixture.inspect())));
        await preview.click();
        await page.waitForFunction(()=>document.querySelector('#navigation-goal-readiness').textContent.includes('3D定位刷新中'));
        assert.equal(starts,0);
        assert(await start.isDisabled());
        const completePreview=()=>{
            emitSample();status('goal_3d_localization_refreshed');
            reply(pendingPreview,{valid:true,code:'ready',preview_id:'preview-'+previews,map_name:'fixture_map',
                message:'无运动路径预览已生成',path:{header:{frame_id:'map'},poses:[
                    {pose:{position:{x:1.2,y:1.2,z:0}}},{pose:{position:{x:3.2,y:1.2,z:0}}}]}});
        };
        completePreview();
        await page.waitForFunction(()=>!document.querySelector('#btn-three-d-start-goal').disabled);
        assert.equal(starts,0,'preview must never start navigation');
        await page.waitForTimeout(3200);
        assert(wire.some(message=>message.op==='call_service'&&message.service==='/request_nomotion_update'),
            'reading a preview must maintain localization without issuing navigation');
        assert(await start.isEnabled());
        assert.equal(starts,0);
        await page.evaluate(()=>window.__refreshFixture.age());
        await start.click();
        await page.waitForFunction(()=>document.querySelector('#navigation-goal-readiness').textContent.includes('旧预览已作废'));
        assert(await start.isDisabled());
        assert.equal(starts,1);
        assert.equal(previews,1,'a refreshed Start must require another Preview click');
        if(process.env.NAV_TEST_SCREENSHOT){
            await page.locator('#three-d-pose-panel').scrollIntoViewIfNeeded();
            await page.screenshot({path:process.env.NAV_TEST_SCREENSHOT});
        }
        await preview.click();
        await page.waitForFunction(()=>document.querySelector('#navigation-goal-readiness').textContent.includes('3D定位刷新中'));
        completePreview();
        await page.waitForFunction(()=>!document.querySelector('#btn-three-d-start-goal').disabled);
        assert.equal(starts,1);
        await start.click();
        await page.waitForFunction(()=>document.querySelector('#log-area').textContent.includes('fixture accepted'));
        assert.equal(starts,2,'only the second explicit Start click may be accepted');
        const guardStatus = (value) => publish('/navigation/velocity_guard', {data:JSON.stringify(value)});
        const warning = {blocked:false, reason:'', warning:true, warning_code:'no_motion_observed',
            warning_reason:'已持续输出非零导航速度 5.0 秒，但里程计未检测到明显移动'};
        guardStatus(warning);
        await page.waitForFunction(()=>!document.querySelector('#velocity-guard-reason').hidden
            && document.querySelector('#velocity-guard-reason').textContent.includes('未检测到明显移动'));
        for (let i=0;i<5;i++) guardStatus({...warning, warning_reason:`持续 ${6+i} 秒，但里程计未检测到明显移动`});
        await page.waitForTimeout(300);
        assert.equal((await page.locator('#log-area').textContent()).split('运动诊断：').length-1,1,
            'motion warning is logged once, not once per update');
        guardStatus({...warning, blocked:true, reason:'测试：地形保护停车'});
        await page.waitForFunction(()=>document.querySelector('#velocity-guard-reason').textContent==='测试：地形保护停车');
        guardStatus({blocked:false, warning:false});
        await page.waitForFunction(()=>document.querySelector('#velocity-guard-reason').hidden);
        assert.equal(starts,2,'read-only diagnostic updates cannot start another goal');
        assert(!wire.some(message=>message.op==='publish'&&['/web_goal_pose','/emergency_stop','/clear_emergency_stop','/cmd_vel_nav'].includes(message.topic)));
        assert.deepEqual(errors,[]);
        console.log('PASS: real 3D refresh/manual Start plus read-only motion warning, log deduplication and guard priority; all I/O mocked');
    }finally{clearInterval(heartbeat);await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
