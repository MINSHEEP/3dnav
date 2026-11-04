(function (root) {
    'use strict';
    function create(log, options = {}) {
        const get = name => document.getElementById('ordinary-camera-' + name);
        const source = get('source'), device = get('device'), path = get('path');
        const url = get('url'), width = get('width'), height = get('height'), fps = get('fps');
        const c12Visible = get('c12-visible'), c12Infrared = get('c12-infrared');
        const refreshButton = get('refresh'), saveButton = get('save');
        let host = root.location.hostname || '192.168.66.103';
        if (host.includes(':') && !host.startsWith('[')) host = `[${host}]`;
        const base = options.base || `http://${host}:8766`;
        let latest = null, seenAt = 0, version = 0, dirty = false, busy = false;
        let polling = false, timer = null, stopped = false;
        const fetcher = options.fetch || root.fetch.bind(root);
        async function request(payload) {
            const controller = new AbortController();
            const timeout = root.setTimeout(() => controller.abort(), payload ? 30000 : 5000);
            try {
                const response = await fetcher(base + '/api/camera/source', {
                    method: payload ? 'POST' : 'GET', cache: 'no-store', signal: controller.signal,
                    ...(payload ? {headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)} : {})
                });
                const result = await response.json();
                if (!response.ok || result.success === false) throw new Error(result.error || result.message || `HTTP ${response.status}`);
                return result;
            } finally { root.clearTimeout(timeout); }
        }
        function render() {
            const fresh = latest && Date.now() - seenAt < 6000 && !stopped;
            const reason = fresh ? latest.locked_reason : '普通摄像头服务离线或状态已过期';
            refreshButton.disabled = busy || polling || stopped;
            saveButton.disabled = !fresh || busy || Boolean(reason) || latest?.switching;
            source.disabled = busy;
            [device, path, width, height, fps].forEach(el => { el.disabled = busy || source.value !== 'usb'; });
            url.disabled = busy || source.value !== 'body';
            get('usb-fields').hidden = source.value !== 'usb';
            get('body-fields').hidden = source.value !== 'body';
            get('c12-fields').hidden = source.value !== 'c12';
            c12Visible.disabled = c12Infrared.disabled = busy || source.value !== 'c12';
            const current = {body:'狗本体摄像头', usb:'USB 摄像头', c12:'C12 双光相机'}[latest?.config?.source] || '--';
            get('state').textContent = !fresh ? reason : `${busy || latest.switching ? '正在切换；' : ''}已保存来源：${current}。${reason || latest.config_error || latest.error || latest.video?.error || ''}`;
        }
        function receive(value) {
            if (!value?.config || !Array.isArray(value.devices)) throw new Error('摄像头配置返回格式不正确');
            if (latest && value.config.version < latest.config.version) return;
            latest = value; seenAt = Date.now();
            if (!dirty && !busy) {
                const config = value.config;
                source.value = config.source; path.value = config.usb_device; url.value = config.body_url;
                c12Visible.value = config.c12_visible_url || '';
                c12Infrared.value = config.c12_infrared_url || '';
                width.value = String(config.usb_width); height.value = String(config.usb_height); fps.value = String(config.usb_fps);
                version = config.version;
            }
            const previous = device.value;
            device.replaceChildren();
            const manual = document.createElement('option');
            manual.value = ''; manual.textContent = '选择设备，或填写下方路径'; device.appendChild(manual);
            for (const item of value.devices) {
                const option = document.createElement('option'); option.value = item.device;
                option.textContent = `${item.name} — ${item.device}`; device.appendChild(option);
            }
            const selected = value.devices.find(item => [item.device, item.node, ...(item.aliases || [])].includes(path.value));
            device.value = selected?.device || (dirty ? previous : '');
            render();
        }
        async function refresh(explicit = false) {
            if (polling || busy || stopped) return;
            if (explicit) dirty = false;
            polling = true; render();
            try { const value = await request(); if (!stopped) receive(value); }
            catch (error) {
                if (!stopped) { seenAt = 0; get('result').textContent = `读取失败：${error.message}`; }
            } finally { polling = false; render(); }
        }
        async function save() {
            if (saveButton.disabled || !get('form').reportValidity()) return;
            const config = {...latest.config, source:source.value};
            if (source.value === 'body') config.body_url = url.value.trim();
            else if (source.value === 'c12') Object.assign(config, {
                c12_visible_url:c12Visible.value.trim(), c12_infrared_url:c12Infrared.value.trim()
            });
            else Object.assign(config, {usb_device:path.value.trim(), usb_width:Number(width.value),
                usb_height:Number(height.value), usb_fps:Number(fps.value)});
            busy = true; render();
            get('result').textContent = '正在释放旧来源并确认新来源首帧；失败将恢复原来源…';
            try {
                const result = await request({config, expected_version:version});
                if (!stopped) {
                    busy = false; dirty = false; receive(result);
                    get('result').textContent = result.message || '已保存并切换普通摄像头';
                    log(get('result').textContent);
                }
            } catch (error) {
                if (!stopped) {
                    get('result').textContent = `切换未成功或结果未确认：${error.message}。请刷新核对当前配置。`;
                    log(get('result').textContent);
                }
            } finally { busy = false; render(); }
        }
        [source, path, url, width, height, fps, c12Visible, c12Infrared].forEach(el => el.addEventListener('input', () => { dirty = true; render(); }));
        source.addEventListener('change', () => { dirty = true; render(); });
        device.addEventListener('change', () => { if (device.value) path.value = device.value; dirty = true; render(); });
        refreshButton.addEventListener('click', () => refresh(true));
        saveButton.addEventListener('click', save);
        render();
        return {
            refresh, save,
            start() { stopped = false; refresh(); timer = root.setInterval(() => { render(); refresh(); }, 2000); },
            stop() { stopped = true; if (timer !== null) root.clearInterval(timer); render(); }
        };
    }
    root.CameraSourceSettings = {create};
}(window));
