/* Manual takeover while keeping the navigation/map processes alive. */
(function (global) {
    'use strict';
    function attach({root, getSession, log, onChange}) {
        const UI=global.RobotUi;
        root.innerHTML=`<div class="page-teleop-row"><strong>人工遥控</strong>
            <button data-manual="take" type="button">接管（暂停自动执行）</button>
            <button data-manual="release" type="button" disabled>退出接管</button>
            <button data-manual="resume" type="button" disabled>继续原任务</button>
            <button data-manual="prepare" type="button" disabled>准备运动／站立</button>
            <button data-manual="reset" type="button" disabled>复位停车锁 R</button>
            <button data-manual="lock" type="button" class="danger" disabled>零速锁定停车</button></div>
            <p data-manual="status" class="hint" aria-live="polite">接管先暂停自动执行，保留地图、原目标和巡检进度；调整后退出接管，再点“继续原任务”。</p>
            <div data-manual="controls" class="page-teleop-controls" hidden>
                <div class="page-teleop-pad"><button type="button" data-drive="q">Q 左转</button><button type="button" data-drive="w">W 前进</button><button type="button" data-drive="e">E 右转</button>
                <span></span><button type="button" data-manual="stop">X 停止</button><span></span>
                <span></span><button type="button" data-drive="s">S 后退</button><span></span></div>
                <label>遥控速度档 <input data-manual="speed" type="range" min="0.1" max="1" step="0.05" value="0.5"><output data-manual="speed-label">50%</output></label>
                <span class="hint">按住移动，松开停止。失焦、切页或断连即退出；需要调整定位时先退出接管。</span>
            </div>`;
        const find=name=>root.querySelector(`[data-manual="${name}"]`);
        let active=false,busy=false,generation=0,session=null,subscriptions=[],timer=null;
        let mode=null,modeAt=0,task=null,taskAt=0,navActive=null,navAt=0,dog=null,dogAt=0,localization=null,localizationAt=0,route=null,needsPreparation=false,releaseAt=0;
        function publish(topic,type,message) {return session?.connected&&session.publish(topic,type,message);}
        function command(value) {if(active) publish('/dog_remote/command',UI.TYPES.string,{data:JSON.stringify({command:value})});}
        const teleop=new UI.KeyboardTeleop({session:{connected:false},cmdVelTopic:'/dog_remote/cmd_vel',
            acceptInput:()=>active, maxLinear:.35,maxAngular:1.0,speedScale:.5,log,
            onSpeed:value=>{find('speed').value=value;find('speed-label').textContent=Math.round(value*100)+'%';},
            onReset:()=>reset(),onEmergencyStop:()=>command('stop')});
        function ready() {
            return active&&session?.connected&&Date.now()-dogAt<1500&&dog?.connected===true
                &&dog.remote_active===true&&!dog.stale&&!dog.emergency_latched&&!dog.hardware_error&&!dog.error
                &&!dog.active_action&&!dog.mapping_lio_paused
                &&(typeof dog.motion_authorized==='boolean'?dog.motion_authorized:dog.motion_mode===2);
        }
        function render(message) {
            const movable=ready()&&!busy;
            if(teleop.enabled!==movable) teleop.setEnabled(movable);
            find('take').disabled=active||busy||!session?.connected;
            find('release').disabled=!active&&!busy;
            find('resume').disabled=active||busy||!session?.connected||!(task?.active&&task.state==='paused'||route?.paused);
            find('prepare').disabled=!active||busy;
            find('reset').disabled=!active||busy||!(dog?.emergency_latched||dog?.error);
            find('lock').disabled=!active;
            find('controls').hidden=!active;
            root.querySelectorAll('[data-drive]').forEach(b=>{b.disabled=!movable;});
            if(message) find('status').textContent=message;
            else if(active&&!busy) find('status').textContent=movable
                ?'已接管：W/S 前后、Q/E 转向；松键或 X/T 停止。自动任务不会自行恢复。'
                :'已接管，移动尚未放行：'+(dog?.error||'请检查连接与姿态；有停车锁时先 R，再点击“准备运动／站立”。');
        }
        function release(reason='已退出接管；原任务保持暂停，点击“继续原任务”后从当前位置重新规划。') {
            generation++;
            if(active) {teleop.stopAndZero();releaseAt=Date.now();publish('/dog_remote/enable',UI.TYPES.bool,{data:false});}
            active=false;busy=false;
            if(timer) global.clearInterval(timer);timer=null;
            render(reason);onChange();
        }
        async function waitFor(check,ticket,message) {
            const end=Date.now()+15000;
            while(!check()) {
                if(ticket!==generation||!session?.connected) throw new Error('接管已取消或连接中断');
                if(Date.now()>end) throw new Error(message);
                await new Promise(resolve=>global.setTimeout(resolve,50));
            }
            if(ticket!==generation) throw new Error('接管已取消');
        }
        async function take() {
            if(active||busy||!getSession()?.connected) return;
            const ticket=++generation;busy=true;session=getSession();teleop.session=session;
            render('正在暂停自动执行，保留原任务和目标；尚未开放遥控…');onChange();
            try {
                await waitFor(()=>mode&&Date.now()-modeAt<2500&&task&&Date.now()-taskAt<2500&&localization,ticket,'任务状态过期，未开放遥控');
                if(mode.campus_mission_active) throw new Error('园区跨段任务尚不支持保留进度接管，请先在园区页停止该任务');
                if(mode.inspection_active||task.active) {
                    if(task.state!=='paused') {
                        const r=await session.call('/inspection/control',UI.TYPES.controlInspectionTask,{command:'pause'},30000);
                        if(!r.success) throw new Error(r.message||'巡检暂停失败');
                    }
                    await waitFor(()=>task.active===true&&task.state==='paused',ticket,'巡检未确认暂停');
                } else if(route?.active) {
                    const r=await session.call('/control_route',UI.TYPES.controlRoute,{command:'pause',owner_token:'',authorization_generation:0},30000);
                    if(!r.success) throw new Error(r.message||'导航暂停失败');
                    await waitFor(()=>route.paused===true,ticket,'导航未确认暂停');
                } else {
                    await waitFor(()=>navActive===false&&Date.now()-navAt<2500,ticket,'未确认自动导航已停止');
                }
                if(localization.state!=='idle'&&!localization.terminal
                        &&!['succeeded','fallback_succeeded','failed','canceled'].includes(localization.state)) {
                    const cancelLocalizationAt=Date.now();
                    publish('/relocalization/cancel',UI.TYPES.empty,{});
                    await waitFor(()=>localizationAt>=cancelLocalizationAt&&localization.terminal===true,
                        ticket,'初始定位取消未确认，未开放遥控');
                }
                if(ticket!==generation||document.hidden) throw new Error('页面已离开，未开放遥控');
                publish('/dog_remote/enable',UI.TYPES.bool,{data:true});
                active=true;busy=false;needsPreparation=true;
                publish('/dog_remote/heartbeat',UI.TYPES.empty,{});
                timer=global.setInterval(()=>{
                    if(!session?.connected||document.hidden) {release('页面或连接不可用，已退出接管。');return;}
                    publish('/dog_remote/heartbeat',UI.TYPES.empty,{});render();
                },100);
                render();onChange();
            } catch(error) {
                if(ticket===generation) {busy=false;render('接管失败：'+(error.message||error));onChange();}
                log('遥控接管：'+(error.message||error));
            }
        }
        async function resume() {
            if(active||busy||!session?.connected) return;
            const ticket=++generation;busy=true;render('正在确认运动准备，随后从当前位置继续原任务…');onChange();
            try {
                await waitFor(()=>dog?.remote_active===false&&dogAt>=releaseAt&&Date.now()-dogAt<1500,ticket,'遥控退出尚未确认');
                const prepared=await session.call('/dog_remote/ensure_standing',UI.TYPES.trigger,{},30000);
                if(ticket!==generation) return;
                if(!prepared.success) throw new Error(prepared.message||'运动准备未确认');
                const r=task?.active&&task.state==='paused'
                    ? await session.call('/inspection/control',UI.TYPES.controlInspectionTask,{command:'resume'},30000)
                    : await session.call('/control_route',UI.TYPES.controlRoute,{command:'resume',owner_token:'',authorization_generation:0},30000);
                if(ticket!==generation) return;
                if(!r.success) throw new Error(r.message||'恢复失败');
                needsPreparation=false;busy=false;render('已请求从当前位置继续原任务，原点位进度保留。');onChange();
            } catch(error) {if(ticket===generation) {busy=false;render('继续失败：'+(error.message||error));onChange();}}
        }
        async function prepare() {
            if(!active||busy) return;
            const ticket=generation;teleop.stopAndZero();busy=true;render('正在确认站立和运动模式，请等待…');
            try {
                const r=await session.call('/dog_remote/ensure_standing',UI.TYPES.trigger,{},30000);
                if(ticket!==generation) return;
                if(!r.success) throw new Error(r.message||'运动准备未确认');
                busy=false;render();
            } catch(error) {if(ticket===generation) {busy=false;render('运动准备失败：'+(error.message||error));}}
        }
        function reset() {
            if(!active||busy) return;
            if(!(dog?.emergency_latched||dog?.error)) {log('当前无停车锁，无需复位。');return;}
            teleop.stopAndZero();publish('/clear_emergency_stop',UI.TYPES.empty,{});render('已请求复位；确认解除锁定后点击“准备运动／站立”。');
        }
        function connect(next) {
            if(active||busy) release('连接已更新，已退出接管。');
            subscriptions.forEach(s=>s.unsubscribe());subscriptions=[];
            session=next;teleop.session=next;mode=task=dog=localization=route=null;navActive=null;
            subscriptions.push(session.subscribe('/navigation/mode',UI.TYPES.string,m=>{try{mode=JSON.parse(m.data);modeAt=Date.now();}catch(_){mode=null;}}, {queueLength:1}));
            subscriptions.push(session.subscribe('/inspection/status',UI.TYPES.string,m=>{try{task=JSON.parse(m.data);taskAt=Date.now();}catch(_){task=null;}}, {queueLength:1}));
            subscriptions.push(session.subscribe('/navigation_active',UI.TYPES.bool,m=>{navActive=m.data;navAt=Date.now();}, {queueLength:1}));
            subscriptions.push(session.subscribe('/relocalization/status',UI.TYPES.string,m=>{try{localization=JSON.parse(m.data);localizationAt=Date.now();}catch(_){localization=null;}}, {queueLength:1}));
            subscriptions.push(session.subscribe('/route_execution_status',UI.TYPES.routeExecutionStatus,m=>{route=m;render();}, {queueLength:1}));
            subscriptions.push(session.subscribe('/dog_status',UI.TYPES.string,m=>{try{dog=JSON.parse(m.data);dogAt=Date.now();}catch(_){dog=null;}render();}, {queueLength:1}));
            render();
        }
        find('take').addEventListener('click',take);find('release').addEventListener('click',()=>release());
        find('resume').addEventListener('click',resume);find('prepare').addEventListener('click',prepare);find('reset').addEventListener('click',reset);
        find('stop').addEventListener('click',()=>{teleop.stopAndZero();command('stop');});
        find('lock').addEventListener('click',()=>{teleop.stopAndZero();publish('/motion_stop',UI.TYPES.empty,{});});
        find('speed').addEventListener('input',event=>teleop.setSpeed(event.target.value));
        root.querySelectorAll('[data-drive]').forEach(button=>{
            button.addEventListener('pointerdown',event=>{if(!ready()||busy)return;event.preventDefault();button.setPointerCapture(event.pointerId);teleop.keys[button.dataset.drive]=true;teleop.start();});
            const up=()=>{teleop.keys[button.dataset.drive]=false;if(!Object.values(teleop.keys).some(Boolean))teleop.stopAndZero();};
            button.addEventListener('pointerup',up);button.addEventListener('pointercancel',up);button.addEventListener('lostpointercapture',up);
        });
        global.addEventListener('blur',()=>{if(active||busy)release('窗口已失焦，已退出接管。');});
        document.addEventListener('visibilitychange',()=>{if(document.hidden&&(active||busy))release('页面已转入后台，已退出接管。');});
        global.addEventListener('pagehide',()=>{if(active||busy)release('页面已离开，已退出接管。');subscriptions.forEach(s=>s.unsubscribe());});
        return {get active(){return active||busy;},get needsPreparation(){return needsPreparation;},connect,release,resume};
    }
    global.PageTeleop={attach};
})(window);
