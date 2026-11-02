/* Shared M701 air-quality telemetry widget for inspection and remote pages. */
(function (global) {
    'use strict';

    const TOPIC = '/air_quality/m701';
    const MESSAGE_TYPE = 'std_msgs/msg/String';
    const STALE_AFTER_MS = 3500;
    const bindings = new WeakMap();
    const FIELDS = Object.freeze([
        ['co2_ppm', 'CO₂', 'ppm', 0],
        ['formaldehyde_ug_m3', '甲醛', 'μg/m³', 0],
        ['tvoc_ug_m3', 'TVOC', 'μg/m³', 0],
        ['pm25_ug_m3', 'PM2.5', 'μg/m³', 0],
        ['pm10_ug_m3', 'PM10', 'μg/m³', 0],
        ['temperature_c', '温度', '℃', 1],
        ['humidity_rh', '湿度', '%RH', 1],
    ]);

    function parsePayload(text) {
        try {
            const payload = JSON.parse(String(text || ''));
            if (!payload || typeof payload !== 'object') return null;
            const values = payload.values && typeof payload.values === 'object'
                ? payload.values : null;
            const normalized = { ...payload, values: {} };
            let complete = Boolean(payload.online && payload.valid && values);
            FIELDS.forEach(([key]) => {
                const value = values ? Number(values[key]) : NaN;
                normalized.values[key] = Number.isFinite(value) ? value : null;
                if (!Number.isFinite(value)) complete = false;
            });
            normalized.valid = complete;
            return normalized;
        } catch (_error) {
            return null;
        }
    }

    function ensureMarkup(root) {
        if (!root || root.dataset.airQualityReady === 'true') return;
        root.dataset.airQualityReady = 'true';
        const cards = FIELDS.map(([key, label, unit]) => (
            `<div class="air-quality-metric" data-field="${key}">`
            + `<span class="label">${label}</span>`
            + '<span class="reading"><strong>--</strong>'
            + `<small>${unit}</small></span></div>`
        )).join('');
        root.innerHTML = '<div class="air-quality-head">'
            + '<span class="air-quality-state stale" data-air-quality-state>等待传感器</span>'
            + '<span class="air-quality-meta" data-air-quality-meta>M701 · 9600 8N1</span>'
            + `</div><div class="air-quality-grid">${cards}</div>`
            + '<div class="air-quality-diagnostic" data-air-quality-diagnostic>'
            + '尚未收到 /air_quality/m701 数据。</div>';
    }

    function clearValues(root) {
        FIELDS.forEach(([key]) => {
            const value = root.querySelector(`[data-field="${key}"] strong`);
            if (value) value.textContent = '--';
        });
    }

    function setState(root, state, text, diagnostic) {
        ensureMarkup(root);
        const element = root.querySelector('[data-air-quality-state]');
        element.className = `air-quality-state ${state}`;
        element.textContent = text;
        root.querySelector('[data-air-quality-diagnostic]').textContent = diagnostic;
        root.classList.toggle('stale', state === 'stale' || state === 'error');
    }

    function setDisconnected(root, reason) {
        if (!root) return;
        clearValues(root);
        setState(root, 'stale', '数据失联', reason || 'ROS2 连接已断开。');
    }

    function render(root, payload) {
        ensureMarkup(root);
        if (!payload) {
            setDisconnected(root, '收到无法解析的传感器状态。');
            return;
        }
        const meta = root.querySelector('[data-air-quality-meta]');
        meta.textContent = `M701 · 地址 ${payload.address ?? '--'} · ${payload.port || '串口未知'}`;
        if (payload.enabled === false) {
            clearValues(root);
            setState(root, 'disabled', '检测已关闭', 'M701 气体/环境检测已在设置页关闭，串口轮询已停止。');
            return;
        }
        if (!payload.valid) {
            clearValues(root);
            setState(
                root,
                payload.online ? 'error' : 'stale',
                payload.online ? '数据无效' : '传感器离线',
                payload.error || '串口未连接或响应未通过完整帧校验。'
            );
            return;
        }
        FIELDS.forEach(([key, _label, _unit, digits]) => {
            const value = root.querySelector(`[data-field="${key}"] strong`);
            value.textContent = Number(payload.values[key]).toFixed(digits);
        });
        const warmup = payload.warming_up === true;
        const remaining = Math.max(0, Number(payload.warmup_remaining_sec) || 0);
        setState(
            root,
            warmup ? 'warming' : 'online',
            warmup ? '传感器预热中' : '实时数据',
            warmup
                ? `首次上电按说明书保守预热，预计还需 ${Math.ceil(remaining)} 秒；当前数值仅供观察。`
                : '已通过地址、功能码、字节长度和 CRC16 校验。'
        );
    }

    function bind(session, root) {
        if (!session || !root) return null;
        ensureMarkup(root);
        const previous = bindings.get(root);
        if (previous && previous.timer) global.clearInterval(previous.timer);
        const state = { lastMessageAt: 0, startedAt: Date.now(), timer: null };
        session.subscribe(TOPIC, MESSAGE_TYPE, (message) => {
            state.lastMessageAt = Date.now();
            render(root, parsePayload(message && message.data));
        });
        state.timer = global.setInterval(() => {
            const reference = state.lastMessageAt || state.startedAt;
            if (Date.now() - reference > STALE_AFTER_MS) {
                setDisconnected(root, '超过 3.5 秒未收到新数据，已清空旧读数。');
            }
        }, 1000);
        bindings.set(root, state);
        setState(root, 'stale', '等待传感器', '正在订阅 /air_quality/m701。');
        return state;
    }

    global.RobotAirQuality = Object.freeze({
        TOPIC,
        bind,
        parsePayload,
        render,
        setDisconnected,
    });
})(window);
