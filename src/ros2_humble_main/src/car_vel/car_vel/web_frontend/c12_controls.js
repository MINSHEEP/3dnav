/* C12 controls share the selected camera pipeline; no robot motion commands. */
(() => {
    const host = location.hostname.includes(':') ? `[${location.hostname}]` : location.hostname;
    const base = `http://${host}:8766`;
    const panel = document.createElement('section');
    panel.className = 'section c12-panel'; panel.hidden = true;
    const style=document.createElement('style');
    style.textContent=`
    .c12-panel{padding:20px!important;margin:16px 0!important;border:1px solid #475569;border-radius:14px;background:#1b2438;color:#e7edf8;font:14px/1.6 system-ui,sans-serif;min-width:0;box-sizing:border-box}
    .c12-panel h2{font-size:20px;line-height:1.4;margin:0 0 8px;font-weight:650}
    .c12-panel>p{color:#a9b6cd;margin:0 0 18px;font-size:13px}
    .c12-panel .c12-layout{display:grid;grid-template-columns:minmax(180px,230px) minmax(0,1fr);gap:24px;align-items:start}
    .c12-panel .c12-directions{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin:0 0 18px}
    .c12-panel button{margin:0;min-height:42px;border-radius:8px;padding:9px 12px;font:600 14px/1.4 system-ui,sans-serif;white-space:nowrap}
    .c12-panel .c12-stop{background:#794052}.c12-panel .c12-home{background:#34445f}
    .c12-panel .c12-speed{display:grid;gap:8px;color:#c8d4e7;font-size:13px}
    .c12-panel .c12-speed-head{display:flex;justify-content:space-between;gap:8px}
    .c12-panel output{color:#7dc8ff;font-weight:650;font-variant-numeric:tabular-nums}
    .c12-panel input[type=range]{width:100%;min-width:0;accent-color:#5bb4f2;padding:0;margin:4px 0}
    .c12-panel .c12-settings{display:grid;gap:12px;min-width:0}
    .c12-panel .c12-fields{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}
    .c12-panel label.c12-field{display:grid;gap:5px;color:#b7c5da;font-size:13px;min-width:0}
    .c12-panel select,.c12-panel input:not([type=range]){width:100%;box-sizing:border-box;min-height:40px;padding:8px 10px;border:1px solid #51617b;border-radius:8px;background:#0f1729;color:#e7edf8;font:14px/1.4 system-ui,sans-serif;min-width:0}
    .c12-panel .c12-actions{display:flex;flex-wrap:wrap;gap:10px;margin-top:4px}
    .c12-panel .c12-actions button:first-child{background:#327db4}
    .c12-panel .c12-feedback{margin:18px 0 0;padding:12px 14px;background:#111a2b;border-radius:8px;color:#bdd4ed;font-size:13px;overflow-wrap:anywhere;min-height:20px}
    .inspection-column .c12-panel .c12-layout{grid-template-columns:1fr;gap:20px}
    @media(max-width:600px){.c12-panel .c12-layout{grid-template-columns:1fr;gap:20px}.c12-panel .c12-directions{max-width:260px;margin-left:auto;margin-right:auto}}
    `;
    panel.append(style);
    const title = document.createElement('h2'); title.textContent = 'C12 云台与画面控制';panel.append(title);
    const note = document.createElement('p');note.textContent='按住方向按钮持续转动，松开停止；离开页面或断连会停止。设置由相机固件决定是否支持；未收到回复不代表成功。';panel.append(note);
    const status=document.createElement('p');status.className='c12-feedback';status.textContent='就绪 · 按住方向按钮转动';status.setAttribute('role','status');
    let busy=false, catalog;
    async function send(payload) {
        const response=await fetch(base+'/api/c12/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
        const result=await response.json();
        if(!response.ok) throw new Error(result.error || '指令失败');
        status.textContent=result.message+(result.data ? `；回复：${result.data}` : '')+(result.angles ? `；航向/俯仰/横滚：${result.angles.join(' / ')}°` : '');
        return result;
    }
    async function act(fn) {
        await release();
        if(busy)return;busy=true;status.textContent='正在发送…';
        try {await fn();} catch(e){status.textContent=e.message;} finally {busy=false;}
    }
    let held = null;
    async function motion(payload) {
        const response = await fetch(base+'/api/c12/motion', {
            method:'POST', headers:{'Content-Type':'application/json'},
            body:JSON.stringify(payload), keepalive:payload.operation==='stop'
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || '云台控制失败');
        return result;
    }
    async function release() {
        const previous = held;
        held = null;
        if (!previous) return;
        clearInterval(previous.timer);
        previous.button.classList.remove('choice-active');
        try {
            await motion({operation:'stop',token:previous.token});
            status.textContent='已发送停止，请观察云台';
        } catch(e) { status.textContent='停止请求未确认，后台将在心跳超时后停止：'+e.message; }
    }
    async function press(direction, button) {
        release();
        const current={token:Date.now().toString(36)+'-'+Math.random().toString(36).slice(2),button};
        held=current;
        button.classList.add('choice-active');
        try {
            await motion({operation:'start',token:current.token,direction,speed:Number(speedSlider.value)});
            if(held!==current) {
                await motion({operation:'stop',token:current.token});
                return;
            }
            status.textContent='正在转动，松开停止';
            current.timer=setInterval(async()=>{
                if(held!==current || current.pending)return;
                current.pending=true;
                try {await motion({operation:'keepalive',token:current.token});}
                catch(e){status.textContent=e.message;if(held===current)await release();}
                finally{current.pending=false;}
            },150);
        } catch(e) { if(held===current){await release();status.textContent=e.message;} }
    }
    const layout=document.createElement('div');layout.className='c12-layout';
    const movement=document.createElement('div'),settings=document.createElement('div');settings.className='c12-settings';
    panel.append(layout);layout.append(movement,settings);
    const directions=document.createElement('div');directions.className='c12-directions';movement.append(directions);
    const speedLabel=document.createElement('label');speedLabel.className='c12-speed';
    const speedHead=document.createElement('span');speedHead.className='c12-speed-head';
    const speedTitle=document.createElement('span');speedTitle.textContent='转动速度';
    const speedOutput=document.createElement('output');
    const speedSlider=document.createElement('input');speedSlider.type='range';speedSlider.min='1';speedSlider.max='127';speedSlider.step='1';speedSlider.value='8';speedSlider.setAttribute('aria-label','云台转动速度');
    function showSpeed(){speedOutput.textContent=(Number(speedSlider.value)*.5).toFixed(1)+' °/s';}
    speedSlider.addEventListener('input',showSpeed);showSpeed();
    speedHead.append(speedTitle,speedOutput);speedLabel.append(speedHead,speedSlider);
    const speedHint=document.createElement('span');speedHint.textContent='0.5–63.5 °/s · 下次按住时生效';speedLabel.append(speedHint);movement.append(speedLabel);
    for(const [label,direction] of [['左','left'],['右','right'],['上','up'],['下','down']]){
        const b=document.createElement('button');b.type='button';b.textContent=label;
        b.style.cssText='touch-action:none;user-select:none';
        b.style.gridArea={left:'2 / 1',right:'2 / 3',up:'1 / 2',down:'3 / 2'}[direction];
        b.addEventListener('pointerdown',event=>{
            if(event.button!==0)return;
            event.preventDefault();b.setPointerCapture(event.pointerId);press(direction,b);
        });
        for(const event of ['pointerup','pointercancel','lostpointercapture'])b.addEventListener(event,release);
        b.addEventListener('keydown',event=>{
            if(![' ','Enter'].includes(event.key))return;
            event.preventDefault();if(!event.repeat)press(direction,b);
        });
        b.addEventListener('keyup',event=>{if([' ','Enter'].includes(event.key)){event.preventDefault();release();}});
        b.addEventListener('blur',release);
        directions.append(b);
    }
    window.addEventListener('blur',release);
    window.addEventListener('pagehide',release);
    document.addEventListener('visibilitychange',()=>{if(document.hidden)release();});
    for(const [label,value] of [['停止','00'],['回中','05']]){
        const b=document.createElement('button');b.type='button';b.textContent=label;
        b.className=value==='00'?'c12-stop':'c12-home';b.style.gridArea=value==='00'?'2 / 2':'3 / 3';
        b.onclick=async()=>{await release();await act(()=>send({command:'PTZ',value}));};directions.append(b);
    }
    const select=document.createElement('select'),form=document.createElement('div');
    const apply=document.createElement('button'),read=document.createElement('button');
    apply.type=read.type='button';apply.textContent='应用设置';read.textContent='读取相机';
    const category=document.createElement('label');category.className='c12-field';category.textContent='功能选择';category.append(select);
    form.className='c12-fields';const actions=document.createElement('div');actions.className='c12-actions';actions.append(apply,read);
    settings.append(category,form,actions);panel.append(status);
    let inputs={};
    function render(){
        form.replaceChildren();inputs={};const spec=catalog[select.value];
        apply.disabled=!!spec.readonly;read.disabled=!(spec.read || spec.readonly);
        if(spec.options){const input=document.createElement('select');for(const [v,l] of Object.entries(spec.options)){const o=document.createElement('option');o.value=v;o.textContent=l;input.append(o);}form.append(input);inputs.value=input;}
        for(const f of spec.fields||[]){const label=document.createElement('label');label.className='c12-field';label.textContent=f.label+' ';const input=document.createElement('input');input.type='number';input.min=f.min;input.max=f.max;input.step=['angle','yaw','pitch'].includes(f.name)?'.01':'1';input.value=f.default;label.append(input);form.append(label);inputs[f.name]=input;}
        if(spec.network){const input=document.createElement('input');input.placeholder='IPv4 地址';form.append(input);inputs.value=input;}
    }
    select.onchange=render;
    apply.onclick=()=>act(async()=>{
        const spec=catalog[select.value],payload={command:select.value};
        for(const [key,input] of Object.entries(inputs)){if(input.type==='number' && (!input.value.trim() || !input.checkValidity()))throw new Error('请检查参数范围');payload[key]=input.value;}
        if(spec.network){if(!confirm('修改网络参数可能使视频和控制断开，确认已经记录新地址并准备同步网络配置？'))return;payload.confirm=true;}
        if((select.value==='PTZ' && ['0A','0B'].includes(payload.value)) || (select.value==='EXT' && payload.calibrate==='1')){if(!confirm('该操作会改变安装模式或进行校准，请确认云台已固定、周围有运动空间。'))return;}
        await send(payload);
    });
    read.onclick=()=>act(()=>send({command:select.value,read:true}));
    const inspectionControls = document.querySelector('.inspection-controls');
    const container = document.querySelector('aside') || document.querySelector('main') || document.body;
    if (inspectionControls) inspectionControls.after(panel); else container.append(panel);
    async function refresh(){
        if(document.hidden)return;
        try{
            const response=await fetch(base+'/api/camera/source');const state=await response.json();
            panel.hidden=state.config?.pipeline!=='c12';
            if(panel.hidden)release();
            if(!panel.hidden && !catalog){const r=await fetch(base+'/api/c12/commands');if(!r.ok)throw new Error('C12 控制接口尚未加载');catalog=await r.json();for(const [tag,spec] of Object.entries(catalog)){const o=document.createElement('option');o.value=tag;o.textContent=spec.label;select.append(o);}render();}
        }catch(e){status.textContent=e.message;}
    }
    refresh();setInterval(refresh,3000);
})();
