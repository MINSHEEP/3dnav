/* Real navigation/inspection DOM, production callbacks and GetMap recovery.
 * All HTTP/WebSocket traffic intercepted; never contacts ROS or robot hardware.
 */
'use strict';
const assert=require('node:assert/strict'),fs=require('fs'),path=require('path'),crypto=require('crypto');
const {chromium}=require('playwright');
const web=path.resolve(__dirname,'../car_vel/web_frontend');
(async()=>{
    const browser=await chromium.launch({headless:true,...(process.env.NAV_TEST_BROWSER
        ?{executablePath:process.env.NAV_TEST_BROWSER}:{})});
    const timers=[];
    const pointCount=150001,bodyBytes=Buffer.alloc(pointCount*16);
    for(let i=0;i<pointCount;i++){
        bodyBytes.writeFloatLE(i%500*.05,i*16);bodyBytes.writeFloatLE(Math.floor(i/500)*.05,i*16+4);
        bodyBytes.writeFloatLE(0,i*16+8);bodyBytes.writeFloatLE(9,i*16+12);
    }
    const pcd=Buffer.concat([Buffer.from(`FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ${pointCount}\nHEIGHT 1\nPOINTS ${pointCount}\nDATA binary\n`),bodyBytes]);
    const digest=crypto.createHash('sha256').update(pcd).digest('hex');
    let fileDownloads=0,afterNavigation=0;
    const browserContext=await browser.newContext({viewport:{width:1440,height:1000}});
    try{
        for(const filename of ['navigation.html','inspection.html']){
            const page=await browserContext.newPage();
            const navigation=filename==='navigation.html',errors=[],wire=[],snapshots=[];
            page.setDefaultTimeout(12000);page.on('pageerror',error=>errors.push(error.message));
            const hook=`\nwindow.__mapTest={ready:()=>session&&session.connected&&nav2Ready,
                state:()=>({ready:${navigation?'navigationMapReady':'mapReady'},name:loadedMapName,pending:pendingMapName,
                    cloudReady:Boolean(${navigation?'latestNavigationGlobalCloud':'acceptedGlobalCloud'}),
                    points:${navigation?'navigationPointCloudViewer':'inspectionPointCloudViewer'}?.pointCount}),
                show3D:()=>{${navigation?"setNavigationMapView('3d')":"syncInspectionMapView()"};},
                prepareRoute:()=>{
                    const r={id:8,name:'fixture_route',map_name:loadedMapName,
                        points:[{id:1,x:1,y:1,z:0,yaw:0,dwell_seconds:0},
                                {id:2,x:2,y:1,z:0,yaw:0,dwell_seconds:0}]};
                    ${navigation?`navigationUiMode='multi';routeDraft=r;routeEditing=false;routeDraftDirty=false;
                        localizationReady=true;localizationHealth=LOCALIZATION_HEALTH.GOOD;
                        latestAmclReceivedAtMs=Date.now()-50000;navigationSafetyReady=true;
                        navigationPointCloudViewer.setTopView();`:
                        `route=r;routes=[r];localizationReady=true;localizedMapName=loadedMapName;
                         standingConfirmed=true;document.getElementById('bidirectional').checked=false;
                         inspectionPointCloudViewer.setTopView();`}
                    return ${navigation?'threeDRoutePreviewIssue()':'threeDRouteIssue()'};
                },
                previewRoute:()=>previewThreeDRoute(),
                routeState:()=>({preview:Boolean(threeDRoutePreview),
                    count:${navigation?'routeDraft':'route'}.points.length}),
                cancel:()=>{${navigation?'clearPendingMapLoad':'clearPendingInspectionMapLoad'}();
                    ${navigation?'updateMapReadyUi':'updateUi'}();}};\n`;
            const body={generation:1,active_body:'xingji',connected:true,
                config:{selected:'xingji',profiles:{xingji:{navigation_verified:true}}},
                status:{stale:false,emergency_latched:false,hardware_error:false}};
            const send=(socket,topic,msg)=>socket.send(JSON.stringify({op:'publish',topic,msg}));
            const reply=(socket,request,values)=>socket.send(JSON.stringify({op:'service_response',
                id:request.id,service:request.service,result:true,values}));
            let generation=0,controlSocket,currentName='',corruptManifest=false,routeRefreshFails=false;
            const savedCloud=()=>{
                // Intentionally no ROS cloud/status payload: the HTTP manifest
                // must bootstrap the exact current stamp for a late browser.
            };
            await page.routeWebSocket('**/*',socket=>socket.onMessage(raw=>{
                const message=JSON.parse(String(raw));wire.push(message);
                if(!String(message.topic||'').startsWith('/web_preview/remote/'))controlSocket=socket;
                if(message.op==='subscribe'){
                    const initial={'/nav2_ready':{data:true},'/navigation_active':{data:false},
                        '/goal_pose_status':{data:'ready'},'/robot_body/status':{data:JSON.stringify(body)},
                        '/navigation/mode':{data:JSON.stringify({mode:'terrain_3d'})}};
                    if(initial[message.topic])send(socket,message.topic,initial[message.topic]);
                }
                if(message.op==='publish'&&message.topic==='/cancel_navigation'){
                    send(socket,'/navigation_active',{data:false});send(socket,'/goal_pose_status',{data:'goal_canceled'});
                }
                if(message.op!=='call_service')return;
                controlSocket=socket;
                if(message.service==='/map_server/map'){snapshots.push({socket,message});return;}
                if(message.service==='/switch_map'){
                    generation++;
                    currentName=message.args.map_name;
                    reply(socket,message,{success:true,message:'confirmed load'});
                    send(socket,'/current_map_info',{success:true,map_name:message.args.map_name,
                        load_id:'load-'+generation,message:'confirmed'});savedCloud();return;
                }
                if(message.service==='/dog_remote/ensure_standing'){
                    return reply(socket,message,{success:true,message:'offline standing confirmation only'});
                }
                if(message.service==='/navigation/preview_3d_route'){
                    send(socket,'/goal_pose_status',{data:'goal_refreshing_3d_localization'});
                    send(socket,'/goal_pose_status',{data:routeRefreshFails?
                        'goal_3d_localization_refresh_failed':'goal_3d_localization_refreshed'});
                    return reply(socket,message,routeRefreshFails?
                        {valid:false,code:'localization_refresh_timeout',message:'等待新定位超时；未启动导航'}:
                        {valid:true,preview_id:'route-preview',map_name:currentName,message:'定位刷新后路线已预检',
                            path:{header:{frame_id:'map'},poses:[{pose:{position:{x:1,y:1,z:0}}},
                                {pose:{position:{x:2,y:1,z:0}}}]}});
                }
                reply(socket,message,{success:true,message:'offline fixture',
                    map_names:['map_a','map_b'],routes:[],tasks:[]});
            }));
            await page.route('**/*',async route=>{
                const url=new URL(route.request().url());
                if(url.hostname!=='map.test')return route.abort();
                if(url.port==='8769'){
                    const identity={map_name:currentName,load_id:'load-'+generation,stamp_ns:String(generation*1000000000)};
                    const headers={'Access-Control-Allow-Origin':'http://map.test'};
                    if(Object.keys(identity).some(key=>key==='stamp_ns'&&url.pathname.endsWith('/manifest')
                        &&!url.searchParams.has(key)?false:url.searchParams.get(key)!==identity[key]))
                        return route.fulfill({status:409,headers,contentType:'application/json',body:'{"error":"stale fixture map"}'});
                    if(url.pathname.endsWith('/manifest'))return route.fulfill({headers,contentType:'application/json',body:JSON.stringify({
                        ...identity,complete:true,frame_id:'map',point_count:pointCount,size_bytes:pcd.length,
                        sha256:corruptManifest?'0'.repeat(64):digest})});
                    fileDownloads++;
                    return route.fulfill({headers:{...headers,'Content-Length':String(pcd.length)},contentType:'application/octet-stream',body:pcd});
                }
                const file=path.resolve(web,'.'+url.pathname);
                if(!file.startsWith(web+path.sep)||!fs.existsSync(file))return route.fulfill({status:404});
                let data=fs.readFileSync(file);
                if(url.pathname==='/'+filename){
                    const original=data.toString('utf8');
                    data=original.replace(/\n\}\)\(\);\s*<\/script>/,hook+'})();\n</script>');
                    assert.notEqual(data,original,'test hook insertion failed');
                }
                const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8',
                    '.css':'text/css; charset=utf-8','.png':'image/png'};
                return route.fulfill({contentType:types[path.extname(file)]||'application/octet-stream',body:data});
            });
            await page.goto('http://map.test/'+filename);
            const heartbeat=setInterval(()=>{
                if(!controlSocket)return;
                send(controlSocket,'/robot_body/status',{data:JSON.stringify(body)});
                send(controlSocket,'/navigation/mode',{data:JSON.stringify({mode:'terrain_3d'})});
            },500);
            timers.push(heartbeat);
            await page.waitForFunction(()=>window.__mapTest&&window.__mapTest.ready());
            await page.waitForFunction(()=>document.querySelector('#map-selector').options.length>=3);
            await page.selectOption('#map-selector','map_a');await page.locator('#btn-load-map').click();
            await page.waitForFunction(()=>window.__mapTest.state().pending==='map_a');
            await waitFor(()=>snapshots.length===1);
            assert.equal((await page.evaluate(()=>window.__mapTest.state())).ready,false,
                'service + status cannot bypass missing raster');
            const stale=snapshots[0];
            await page.evaluate(()=>window.__mapTest.cancel());
            await page.selectOption('#map-selector','map_b');await page.locator('#btn-load-map').click();
            await waitFor(()=>snapshots.length===2);
            const map={header:{frame_id:'map'},info:{width:1880,height:1568,resolution:.05,
                origin:{position:{x:-79.35,y:-48.1,z:0},orientation:{x:0,y:0,z:0,w:1}}},
                data:Array(1880*1568).fill(-1)};
            reply(stale.socket,stale.message,{map});
            await page.waitForTimeout(100);
            assert.equal((await page.evaluate(()=>window.__mapTest.state())).ready,false,
                'late old-map response cannot mark new map ready');
            const current=snapshots[1];reply(current.socket,current.message,{map});
            await page.waitForFunction(()=>window.__mapTest.state().ready);
            const state=await page.evaluate(()=>window.__mapTest.state());
            assert.equal(state.name,'map_b');assert.equal(state.pending,'');
            await page.evaluate(()=>window.__mapTest.show3D());
            savedCloud();
            await page.waitForFunction(()=>window.__mapTest.state().cloudReady);
            assert.equal((await page.evaluate(()=>window.__mapTest.state())).points,pointCount,'all saved points must render past 120000');
            assert(!wire.some(m=>m.op==='subscribe'&&(m.topic==='/global_localization_map'
                ||m.topic==='/web_preview/remote/global_localization_map')),'saved map must not use reduced ROS topic');
            assert(!wire.some(m=>m.op==='publish'&&['/web_goal_pose','/emergency_stop','/clear_emergency_stop'].includes(m.topic)));
            assert.deepEqual(errors,[]);
            assert.equal(await page.evaluate(()=>window.__mapTest.prepareRoute()),'',filename+' route fixture ready');
            await page.evaluate(()=>window.__mapTest.previewRoute());
            assert.equal((await page.evaluate(()=>window.__mapTest.routeState())).preview,true,
                'stationary route preview must accept refresh success without starting');
            routeRefreshFails=true;
            await page.evaluate(()=>window.__mapTest.previewRoute());
            const failedRoute=await page.evaluate(()=>window.__mapTest.routeState());
            assert.equal(failedRoute.preview,false,'refresh failure must not retain old executable preview');
            assert.equal(failedRoute.count,2,'refresh failure must retain saved route points');
            assert(!wire.some(m=>m.op==='call_service'&&['/start_route','/start_inspection_task'].includes(m.service)));
            assert.deepEqual(errors,[]);
            await page.waitForFunction(async expected=>new Promise(resolve=>{
                const open=indexedDB.open('robot.complete-saved-map',1);open.onerror=()=>resolve(false);
                open.onsuccess=()=>{const db=open.result;const tx=db.transaction('pcd','readonly');
                    const read=tx.objectStore('pcd').get('latest');read.onsuccess=()=>resolve(read.result?.digest===expected);
                    read.onerror=()=>resolve(false);tx.oncomplete=()=>db.close();};
            }),digest);
            if(!navigation)assert.equal(fileDownloads,afterNavigation,'inspection reuses verified disk cache without full redownload');
            corruptManifest=true;
            await page.selectOption('#map-selector','map_a');await page.locator('#btn-load-map').click();
            await waitFor(()=>snapshots.length===3);
            reply(snapshots[2].socket,snapshots[2].message,{map});
            const statusId=navigation?'navigation-pointcloud-status':'inspection-pointcloud-status';
            await page.waitForFunction(id=>document.getElementById(id).textContent.includes('SHA256'),statusId);
            assert.equal((await page.evaluate(()=>window.__mapTest.state())).cloudReady,false,
                'checksum failure must clear full cloud rather than fall back to reduced cloud');
            if(navigation)afterNavigation=fileDownloads;
            console.log('PASS: '+filename+' full PCD/cache/identity; route refresh success/failure retains points, no automatic motion');
            clearInterval(heartbeat);
            await page.close();
        }
    }finally{timers.forEach(clearInterval);await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
async function waitFor(predicate){
    const end=Date.now()+10000;
    while(!predicate()){if(Date.now()>end)throw new Error('mock GetMap request not received');
        await new Promise(resolve=>setTimeout(resolve,20));}
}
