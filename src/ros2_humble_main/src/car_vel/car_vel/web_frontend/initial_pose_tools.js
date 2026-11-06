/* Shared read-only scan overlay and 2-D matching parameters for navigation/inspection. */
(function (global) {
    'use strict';
    function attach(options) {
        const {root, toggle, canvas, getSession, getMode, isBusy, isInitialMode, onToggle, log} = options;
        const find = role => root.querySelector(`[data-pose-tool="${role}"]`);
        const input=find('percent'), hint=find('match-hint'), scanHint=find('scan-hint');
        const readButton=find('read'), applyButton=find('apply');
        let enabled=false, subscription=null, previousMode='';
        function reconnect() {
            subscription?.unsubscribe(); subscription=null; canvas.scanOverlay=null;
            const session=getSession();
            if (!enabled || !session?.connected) return;
            subscription=session.subscribe('/localization/scan_preview', global.RobotUi.TYPES.string, message=>{
                let data; try {data=JSON.parse(message.data);} catch (_) {return;}
                const points=(data.points||[]).filter(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite));
                canvas.scanOverlay={points,receivedAt:Date.now(),followDrag:isInitialMode()};
                scanHint.textContent=data.error||(points.length
                    ? `实时激光 ${points.length} 点；红点贴合墙体后确认初始位置。`
                    : '暂无有效激光点，请检查雷达和 LIO。');
            });
        }
        function setEnabled(value) {
            enabled=Boolean(value);
            toggle.setAttribute('aria-pressed',String(enabled));
            toggle.classList.toggle('active',enabled);
            toggle.textContent=enabled?'隐藏二维点云':'显示二维点云';
            scanHint.textContent=enabled?'等待实时激光点；请加载地图并启动导航定位。':'二维点云已关闭';
            onToggle(enabled); reconnect();
        }
        async function readThreshold() {
            try {
                if (!getSession()?.connected) throw new Error('ROS2 尚未连接');
                const response=await getSession().call('/relocalization_coordinator/get_parameters','rcl_interfaces/srv/GetParameters',
                    {names:['initial_scan_match_ratio_2d']});
                if(response.values?.[0]?.type!==3) throw new Error('后端未提供二维阈值接口，请更新并重启导航');
                const percent=Math.round(response.values[0].double_value*1000)/10;
                input.value=percent;
                hint.textContent=`当前阈值 ${percent}%；导航重启后恢复默认值。`;
            } catch(error) {hint.textContent='读取失败：'+(error.message||error);}
        }
        function syncMode() {
            const mode=getMode(), connected=Boolean(getSession()?.connected);
            toggle.disabled=mode!=='map_2d'||!connected;
            input.disabled=mode!=='map_2d'||!connected||isBusy();
            readButton.disabled=mode!=='map_2d'||!connected;
            applyButton.disabled=mode!=='map_2d'||!connected||isBusy();
            if(mode!=='map_2d'&&enabled) setEnabled(false);
            if(mode!=='map_2d') scanHint.textContent=mode==='terrain_3d'?'当前三维模式，请使用三维初始点工具。':'等待导航模式确认。';
            if(mode==='map_2d'&&previousMode!==mode&&connected) readThreshold();
            previousMode=mode;
        }
        toggle.addEventListener('click',()=>{
            if(getMode()!=='map_2d') {log('请先切换到二维导航模式。');return;}
            setEnabled(!enabled);
        });
        readButton.addEventListener('click',readThreshold);
        applyButton.addEventListener('click',async()=>{
            const value=Number(input.value);
            if(!Number.isFinite(value)||value<1||value>100) {log('匹配阈值请输入 1–100%。');return;}
            if(getMode()!=='map_2d'||isBusy()) {log('请停止任务并等待当前初始定位完成后调整阈值。');return;}
            applyButton.disabled=true;
            try {
                const response=await getSession().call('/relocalization_coordinator/set_parameters','rcl_interfaces/srv/SetParameters',
                    {parameters:[{name:'initial_scan_match_ratio_2d',value:{type:3,double_value:value/100}}]});
                if(!response.results?.[0]?.successful) throw new Error(response.results?.[0]?.reason||'后端拒绝设置');
                await readThreshold();
            } catch(error) {hint.textContent='应用失败：'+(error.message||error);}
            finally {syncMode();}
        });
        const timer=global.setInterval(()=>{
            if(enabled&&canvas.scanOverlay&&Date.now()-canvas.scanOverlay.receivedAt>1500)
                scanHint.textContent='激光点已过期，请检查雷达和定位节点；旧点已隐藏。';
        },1000);
        global.addEventListener('pagehide',()=>{global.clearInterval(timer);subscription?.unsubscribe();canvas.scanOverlay=null;});
        return {get enabled(){return enabled;},setEnabled,reconnect,syncMode,readThreshold};
    }
    global.InitialPoseTools={attach};
})(window);
