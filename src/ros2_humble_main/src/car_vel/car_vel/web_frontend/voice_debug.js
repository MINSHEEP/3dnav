(function () {
    'use strict';

    const byId = id => document.getElementById(id);
    const directions = Array.from(document.querySelectorAll('[data-direction]'));
    const postures = Array.from(document.querySelectorAll('[data-posture]'));
    const phrases = Array.from(document.querySelectorAll('[data-phrase]'));
    const navigationButtons = Array.from(document.querySelectorAll('[data-navigation]'));
    const keys = { w: 'forward', s: 'backward', a: 'left', d: 'right', q: 'turn_left', e: 'turn_right' };
    const service = new URL(window.location.href);
    service.port = document.body.dataset.servicePort || '8768';
    service.pathname = '/';
    service.search = '';
    service.hash = '';
    let session = '';
    let lastStatus = null;
    let polling = false;
    let connecting = false;
    let epoch = 0;
    let motionSequence = 0;
    let heldDirection = '';
    let heldPointer = null;
    let heldKey = '';
    let moveTimer = null;
    let movePending = false;
    let micPending = false;
    let micVersion = 0;
    let posturePending = false;
    let eventCursor = 0;
    const phrasePending = new Set();
    let navigationPending = false;
    let anchorMap = null;
    let anchorPixels = null;
    let anchorDraft = null;
    let anchorPointer = null;
    let mapPending = false;
    let mapError = '';

    function log(message) {
        const area = byId('log');
        const placeholder = area.querySelector('.log-placeholder');
        if (placeholder) placeholder.remove();
        const row = document.createElement('p');
        const stamp = document.createElement('span');
        stamp.className = 'log-time';
        stamp.textContent = new Date().toLocaleTimeString();
        row.append(stamp, document.createTextNode(String(message)));
        area.appendChild(row);
        while (area.children.length > 80) area.firstElementChild.remove();
        area.scrollTop = area.scrollHeight;
    }

    function message(text, error = false) {
        byId('page-message').textContent = text;
        byId('page-message').classList.toggle('error', error);
    }

    async function request(path, payload, keepalive = false) {
        const controller = new AbortController();
        const timer = window.setTimeout(() => controller.abort(), 2500);
        try {
            const options = { cache: 'no-store', signal: controller.signal };
            if (payload !== undefined) {
                options.method = 'POST';
                options.headers = { 'Content-Type': 'application/json' };
                options.body = JSON.stringify(payload);
                options.keepalive = keepalive;
            }
            const response = await fetch(new URL(`api/voice-debug/${path}`, service), options);
            const result = await response.json();
            if (!response.ok) {
                const error = new Error(result.error || '调试请求失败。');
                error.status = response.status;
                throw error;
            }
            return result;
        } finally {
            window.clearTimeout(timer);
        }
    }

    function updateControls() {
        const ready = Boolean(session && lastStatus && lastStatus.ready);
        const movingReady = ready && lastStatus.robot_online && lastStatus.posture === '站立' && !lastStatus.posture_busy && !posturePending;
        directions.forEach(button => { button.disabled = !movingReady; });
        postures.forEach(button => { button.disabled = !ready || lastStatus.posture_busy || posturePending; });
        byId('stop').disabled = !ready;
        // The microphone flag deliberately does not participate in this condition.
        phrases.forEach(button => { button.disabled = !ready || !lastStatus.voice_connected || phrasePending.has(button.dataset.phrase); });
        byId('microphone').disabled = !ready || micPending;
        byId('microphone').setAttribute('aria-checked', String(Boolean(ready && lastStatus.microphone_enabled)));
        byId('microphone-label').textContent = ready && lastStatus.microphone_enabled ? '关闭麦克风' : '开启麦克风';
        byId('microphone-description').textContent = ready && lastStatus.microphone_enabled
            ? '已开启，麦克风音频允许传入语音程序。'
            : '已关闭，音频不会传入语音程序。';
        const nav = lastStatus && lastStatus.navigation;
        navigationButtons.forEach(button => {
            button.disabled = !ready || !nav || nav.busy || navigationPending || posturePending
                || lastStatus.posture_busy || Boolean(heldDirection)
                || (button.dataset.navigation !== 'prepare' && (!nav.ready || !lastStatus.voice_connected));
        });
        byId('save-anchor').disabled = !ready || !nav || !nav.can_set_anchor
            || nav.busy || navigationPending || !anchorDraft;
        byId('reset-anchor').disabled = !ready || !nav || !nav.map_available || nav.busy || navigationPending;
    }

    function render(status) {
        lastStatus = status;
        const connected = Boolean(session && status.ready);
        byId('service-status').textContent = status.preview ? '页面预览' : connected ? '调试服务已连接' : '调试会话未连接';
        byId('service-status').classList.toggle('online', connected);
        byId('robot-status').textContent = status.robot_online ? '机器人在线' : '机器人未连接';
        byId('robot-status').classList.toggle('online', status.robot_online);
        byId('voice-status').textContent = status.voice_connected ? '语音程序已连接' : '语音程序未连接';
        byId('voice-status').classList.toggle('online', status.voice_connected);
        byId('posture').textContent = status.posture_busy ? '切换中…' : status.posture;
        byId('last-input').textContent = status.last_input || '等待语音或按钮输入';
        byId('last-reply').textContent = status.last_reply || '等待回复，语音由小主机扬声器播放';
        const nav = status.navigation;
        const states = { idle: '尚未启动', preparing: '准备中', awaiting_anchor: '请设置区域a', waiting_odometry: '等待雷达定位',
            localizing: '定位校正中', ready: '导航就绪', navigating: '导航中', dwelling: '停留中',
            completed: '已到达', interrupted: '已接管', error: '未完成' };
        byId('navigation-status').textContent = nav ? states[nav.state] || nav.state : '未连接';
        byId('navigation-status').classList.toggle('online', Boolean(nav && nav.ready && nav.state !== 'error' && nav.state !== 'interrupted'));
        byId('navigation-status').classList.toggle('error', Boolean(nav && nav.state === 'error'));
        byId('navigation-message').textContent = nav ? nav.message : '连接机器人后可使用导航。';
        byId('anchor-picker').hidden = false;
        if (nav && (!nav.anchor || nav.state === 'awaiting_anchor')) byId('anchor-picker').open = true;
        const mapCurrent = Boolean(anchorMap && nav && anchorMap.revision === nav.map_revision);
        byId('anchor-map-wrap').hidden = !anchorPixels;
        byId('anchor-map-message').textContent = !nav ? '连接调试服务后读取已保存的地图。'
            : nav.map_available ? mapError || (mapCurrent
                ? '已显示 shiyanshi1 地图。地图可见不代表导航已就绪；请先设置区域a的位置和朝向。'
                : '正在读取已保存的地图…')
                : nav.map_error || '正在等待选点地图；导航启动失败时，这里会显示具体原因。';
        if (nav && nav.map_available && !mapCurrent && !mapPending) loadAnchorMap();
        if (nav && !nav.map_available) { anchorMap = null; anchorPixels = null; anchorDraft = null; }
        if (status.preview) message('当前为页面预览，尚未连接机器人和语音程序。');
        else if (status.error) message(status.error, true);
        else if (connected) message(status.voice_connected
            ? '调试已就绪。麦克风开关控制音频输入，两句文本可随时单独发送。'
            : '遥控服务已连接，语音程序正在连接；请稍候。');
        for (const event of status.events || []) {
            if (event.id > eventCursor) log(event.text);
            eventCursor = Math.max(eventCursor, event.id);
        }
        updateControls();
        if (heldDirection && (!connected || !status.robot_online || status.posture_busy || status.posture !== '站立')) stopMotion();
    }

    function clearHeld() {
        heldDirection = '';
        heldPointer = null;
        heldKey = '';
        if (moveTimer !== null) window.clearInterval(moveTimer);
        moveTimer = null;
        directions.forEach(button => button.classList.remove('held'));
    }

    async function loadAnchorMap() {
        mapPending = true;
        const version = epoch;
        const revision = lastStatus && lastStatus.navigation && lastStatus.navigation.map_revision;
        try {
            const result = await request('navigation/map');
            if (version !== epoch || !lastStatus.navigation || revision !== lastStatus.navigation.map_revision
                || result.revision !== revision) return;
            mapError = '';
            anchorMap = result;
            const canvas = byId('anchor-map');
            canvas.width = result.width;
            canvas.height = result.height;
            const gray = atob(result.pixels);
            anchorPixels = new ImageData(result.width, result.height);
            for (let y = 0; y < result.height; y += 1) {
                for (let x = 0; x < result.width; x += 1) {
                    const index = ((result.height - 1 - y) * result.width + x) * 4;
                    const value = gray.charCodeAt(y * result.width + x);
                    anchorPixels.data.set([value, value, value, 255], index);
                }
            }
            anchorDraft = lastStatus.navigation.anchor && { ...lastStatus.navigation.anchor };
            drawAnchor();
            byId('anchor-map-wrap').hidden = false;
        } catch (error) {
            mapError = `地图读取失败：${error.message}`;
            byId('anchor-map-message').textContent = mapError;
            log(mapError);
        }
        finally { mapPending = false; updateControls(); }
    }

    function originYaw() {
        const q = anchorMap.origin.orientation;
        return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }

    function mapPosition(event) {
        const rect = byId('anchor-map').getBoundingClientRect();
        const x = Math.max(0, Math.min(anchorMap.width, (event.clientX - rect.left) / rect.width * anchorMap.width));
        const y = Math.max(0, Math.min(anchorMap.height, (rect.bottom - event.clientY) / rect.height * anchorMap.height));
        const a = originYaw();
        return { x: anchorMap.origin.position.x + anchorMap.resolution * (x * Math.cos(a) - y * Math.sin(a)),
            y: anchorMap.origin.position.y + anchorMap.resolution * (x * Math.sin(a) + y * Math.cos(a)) };
    }

    function drawAnchor() {
        if (!anchorPixels) return;
        const canvas = byId('anchor-map');
        const ctx = canvas.getContext('2d');
        ctx.putImageData(anchorPixels, 0, 0);
        if (anchorDraft) {
            const a = originYaw();
            const dx = anchorDraft.x - anchorMap.origin.position.x;
            const dy = anchorDraft.y - anchorMap.origin.position.y;
            const x = (dx * Math.cos(a) + dy * Math.sin(a)) / anchorMap.resolution;
            const y = anchorMap.height - (-dx * Math.sin(a) + dy * Math.cos(a)) / anchorMap.resolution;
            const yaw = anchorDraft.yaw - a;
            const length = Math.max(24, canvas.width / 25);
            ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw);
            ctx.strokeStyle = '#e12a46'; ctx.fillStyle = '#e12a46'; ctx.lineWidth = 4;
            ctx.beginPath(); ctx.arc(0, 0, 7, 0, Math.PI * 2); ctx.fill();
            ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(length, 0); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(length, 0); ctx.lineTo(length - 12, -7); ctx.lineTo(length - 12, 7); ctx.closePath(); ctx.fill();
            ctx.restore();
            byId('anchor-description').textContent = `区域a：(${anchorDraft.x.toFixed(2)}, ${anchorDraft.y.toFixed(2)})，朝向 ${(anchorDraft.yaw * 180 / Math.PI).toFixed(0)}°`;
        } else byId('anchor-description').textContent = '在区域a附近按住并拖动，设置位置和朝向。';
        updateControls();
    }

    async function navigationAction(action, payload = {}) {
        if (!session || navigationPending) return;
        navigationPending = true;
        updateControls();
        try {
            await request(`navigation/${action}`, { ...payload, session, sequence: ++motionSequence });
            if (action === 'prepare' && lastStatus.navigation && !lastStatus.navigation.anchor) {
                byId('anchor-picker').open = true;
                byId('anchor-picker').scrollIntoView({ block: 'start', behavior: 'smooth' });
            }
            await poll();
        } catch (error) { log(error.message); message(error.message, true); }
        finally { navigationPending = false; updateControls(); }
    }

    function stopMotion() {
        clearHeld();
        if (session) {
            return request('stop', { session, sequence: ++motionSequence }, true).then(() => true).catch(error => {
                log(`停止请求未确认：${error.message}；服务会在遥控超时后停止。`);
                return false;
            });
        }
        return Promise.resolve(false);
    }

    function disconnected(error) {
        epoch += 1;
        clearHeld();
        session = '';
        byId('service-status').textContent = '调试服务连接中断';
        byId('service-status').classList.remove('online');
        updateControls();
        message(`连接中断：${error.message || '无法访问调试服务'}。麦克风和运动将在会话超时后关闭，请重新连接。`, true);
    }

    async function sendMove() {
        if (!heldDirection || !session || movePending) return;
        movePending = true;
        const version = epoch;
        try {
            await request('move', { session, direction: heldDirection,
                speed: Number(byId('speed').value), sequence: ++motionSequence });
        } catch (error) {
            if (version !== epoch) return;
            stopMotion();
            log(error.message);
            message(error.message, true);
        } finally {
            movePending = false;
        }
    }

    function startMove(direction, pointer, key) {
        const button = directions.find(item => item.dataset.direction === direction);
        if (!button || button.disabled) return;
        clearHeld();
        heldDirection = direction;
        heldPointer = pointer;
        heldKey = key || '';
        button.classList.add('held');
        sendMove();
        moveTimer = window.setInterval(sendMove, 100);
    }

    async function poll() {
        if (polling || !session || document.hidden) return;
        polling = true;
        const version = epoch;
        const inputVersion = micVersion;
        try {
            await request('heartbeat', { session });
            const status = await request('status');
            if (version === epoch) {
                if (inputVersion !== micVersion && lastStatus) status.microphone_enabled = lastStatus.microphone_enabled;
                render(status);
            }
        } catch (error) {
            if (version === epoch) disconnected(error);
        } finally {
            polling = false;
        }
    }

    async function connect() {
        if (connecting || session || document.hidden) return;
        connecting = true;
        const version = ++epoch;
        eventCursor = 0;
        byId('reconnect').disabled = true;
        try {
            const status = await request('status');
            if (version !== epoch) return;
            render(status);
            if (!status.ready) return;
            const result = await request('session', {});
            if (version !== epoch || document.hidden) {
                request('release', { session: result.session, reason: 'hidden' }, true).catch(() => {});
                return;
            }
            session = result.session;
            motionSequence = 0;
            await poll();
        } catch (error) {
            if (version === epoch) disconnected(error);
        } finally {
            connecting = false;
            byId('reconnect').disabled = false;
        }
    }

    function release(reason = 'pagehide') {
        epoch += 1;
        clearHeld();
        const previous = session;
        session = '';
        updateControls();
        if (previous) request('release', { session: previous, reason }, true).catch(() => {});
    }

    directions.forEach(button => {
        button.addEventListener('pointerdown', event => {
            if (event.button !== 0 || heldPointer !== null) return;
            event.preventDefault();
            button.setPointerCapture(event.pointerId);
            startMove(button.dataset.direction, event.pointerId, '');
        });
        const up = event => {
            if (event.pointerId !== heldPointer) return;
            event.preventDefault();
            stopMotion();
        };
        button.addEventListener('pointerup', up);
        button.addEventListener('pointercancel', up);
        button.addEventListener('lostpointercapture', up);
        button.addEventListener('contextmenu', event => event.preventDefault());
    });
    postures.forEach(button => button.addEventListener('click', async () => {
        if (!session || posturePending) return;
        const version = epoch;
        posturePending = true;
        updateControls();
        try {
            if (!await stopMotion() || version !== epoch) return;
            await request('posture', { session, command: button.dataset.posture });
            log(button.dataset.posture === 'stand' ? '已请求站立。' : '已请求趴下。');
            await poll();
        } catch (error) { log(error.message); message(error.message, true); }
        finally { posturePending = false; updateControls(); }
    }));
    phrases.forEach(button => button.addEventListener('click', async () => {
        if (!session || button.disabled) return;
        const phrase = button.dataset.phrase;
        phrasePending.add(phrase);
        updateControls();
        try {
            await request('phrase', { session, phrase });
        } catch (error) { log(error.message); message(error.message, true); }
        finally { phrasePending.delete(phrase); updateControls(); }
    }));
    byId('microphone').addEventListener('click', async () => {
        if (!session || micPending) return;
        const version = epoch;
        const enabled = !lastStatus.microphone_enabled;
        micPending = true;
        micVersion += 1;
        updateControls();
        try {
            await request('microphone', { session, enabled });
            if (version === epoch) lastStatus.microphone_enabled = enabled;
        } catch (error) {
            log(`麦克风切换未确认：${error.message}`);
            // Losing an OFF response cannot leave an unknown live microphone.
            if (version === epoch) { release(); disconnected(error); }
        } finally { micPending = false; micVersion += 1; updateControls(); }
    });
    byId('stop').addEventListener('click', stopMotion);
    navigationButtons.forEach(button => button.addEventListener('click', () => navigationAction(button.dataset.navigation)));
    byId('save-anchor').addEventListener('click', () => { if (anchorDraft) navigationAction('anchor', anchorDraft); });
    byId('reset-anchor').addEventListener('click', () => navigationAction('reset-anchor'));
    byId('anchor-map').addEventListener('pointerdown', event => {
        if (event.button !== 0 || !anchorMap || !session || !lastStatus.navigation.can_set_anchor) return;
        event.preventDefault();
        anchorPointer = event.pointerId;
        byId('anchor-map').setPointerCapture(event.pointerId);
        anchorDraft = { ...mapPosition(event), yaw: anchorDraft ? anchorDraft.yaw : 0 };
        drawAnchor();
    });
    byId('anchor-map').addEventListener('pointermove', event => {
        if (anchorPointer !== event.pointerId || !anchorDraft) return;
        const position = mapPosition(event);
        if (Math.hypot(position.x - anchorDraft.x, position.y - anchorDraft.y) > 0.08) {
            anchorDraft.yaw = Math.atan2(position.y - anchorDraft.y, position.x - anchorDraft.x);
            drawAnchor();
        }
    });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(type => byId('anchor-map').addEventListener(type, event => {
        if (anchorPointer === event.pointerId) anchorPointer = null;
    }));
    byId('speed').addEventListener('input', () => { byId('speed-value').textContent = Number(byId('speed').value).toFixed(1); });
    byId('clear-log').addEventListener('click', () => { byId('log').replaceChildren(); });
    byId('reconnect').addEventListener('click', async () => {
        if (session) { await poll(); return; }
        await connect();
    });
    document.addEventListener('keydown', event => {
        if (event.target && (['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target.tagName) || event.target.isContentEditable)) return;
        if (event.target && event.target.tagName === 'BUTTON' && [' ', 'Enter'].includes(event.key)) return;
        const key = event.key.toLowerCase();
        if (event.key === ' ' || event.key === 'Escape') { event.preventDefault(); stopMotion(); return; }
        if (keys[key]) {
            event.preventDefault();
            if (!event.repeat) startMove(keys[key], null, key);
        }
    });
    document.addEventListener('keyup', event => {
        if (event.key.toLowerCase() === heldKey) { event.preventDefault(); stopMotion(); }
    });
    window.addEventListener('blur', () => { if (heldDirection) stopMotion(); });
    window.addEventListener('pagehide', () => release('pagehide'));
    window.addEventListener('beforeunload', () => release('unload'));
    document.addEventListener('visibilitychange', () => {
        if (document.hidden) release('hidden');
        else connect();
    });
    window.setInterval(poll, 400);
    updateControls();
    connect();
})();
