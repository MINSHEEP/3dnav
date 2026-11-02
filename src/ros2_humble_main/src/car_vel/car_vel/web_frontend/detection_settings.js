/* One persistent policy for manual recognition, inspection and environment data. */
(function (root) {
    'use strict';
    const ITEMS = [
        ['fire', '火焰检测', '检测火焰并产生告警。'],
        ['smoke', '烟雾检测', '检测烟雾并产生告警。'],
        ['face', '人脸识别', '人脸检测、登记人员匹配及识别记录。'],
        ['plate', '车牌识别', '车牌检测、号码读取、车辆匹配及记录。'],
        ['fall', '疑似跌倒', '连续姿态规则；单纯躺卧不按跌倒告警。'],
        ['fight', '疑似打架', '连续视频场景判断。'],
        ['smoking', '疑似吸烟', '香烟候选与人物嘴部、手腕关联判断。'],
        ['thermal', '热成像高温检测', '关闭后停止高温告警，热成像画面仍可查看。'],
        ['m701', 'M701 气体/环境检测', 'CO₂、甲醛、TVOC、颗粒物、温湿度；关闭后释放串口并停止轮询。'],
        ['co2_alarm', '二氧化碳超限告警', '巡检期间按设定阈值告警；关闭不影响 M701 其他读数。']
    ];
    function create(log = () => {}, options = {}) {
        const panel = document.getElementById('detection-settings-list');
        if (!panel) return {start() {}, stop() {}};
        const status = document.getElementById('detection-settings-state');
        const note = document.getElementById('detection-settings-result');
        const refreshButton = document.getElementById('detection-settings-refresh');
        const elements = new Map();
        panel.replaceChildren();
        for (const [key, label, detail] of ITEMS) {
            const row = document.createElement('label');
            row.className = 'detection-switch-row';
            const copy = document.createElement('span');
            const title = document.createElement('strong'); title.textContent = label;
            const hint = document.createElement('small'); hint.textContent = detail;
            copy.append(title, hint);
            const state = document.createElement('span'); state.className = 'detection-switch-value';
            const input = document.createElement('input');
            input.type = 'checkbox'; input.id = 'detection-switch-' + key;
            input.setAttribute('role', 'switch'); input.setAttribute('aria-label', label);
            input.disabled = true;
            row.append(copy, state, input); panel.appendChild(row);
            elements.set(key, {input, state});
            input.addEventListener('change', () => change(key, input.checked));
        }
        let host = root.location.hostname || '127.0.0.1';
        if (host.includes(':') && !host.startsWith('[')) host = '[' + host + ']';
        const base = options.base || `http://${host}:8766`;
        const fetcher = options.fetch || root.fetch.bind(root);
        let latest = null, seenAt = 0, stopped = false, busy = false, pending = null;
        let controller = null, interval = null, generation = 0, readError = false;
        const fresh = () => latest && Date.now() - seenAt < 6500 && !stopped;
        function render() {
            const available = fresh();
            status.textContent = !available ? '检测配置服务离线或正在连接…'
                : (busy ? '正在保存…' : latest.error || `已连接 · 配置版本 ${latest.version}`);
            for (const [key, {input, state}] of elements) {
                const enabled = pending?.key === key ? pending.value : latest?.enabled?.[key];
                if (typeof enabled === 'boolean') input.checked = enabled;
                input.disabled = !available || busy;
                state.textContent = !available ? '等待配置' : (pending?.key === key ? '保存中'
                    : !enabled ? '已关闭' : key === 'co2_alarm' && latest.enabled.m701 === false
                        ? '随 M701 暂停' : '已开启');
            }
            refreshButton.disabled = stopped || busy || Boolean(controller);
        }
        function receive(value) {
            if (!value || !Number.isInteger(value.version) || !value.enabled
                || ITEMS.some(([key]) => typeof value.enabled[key] !== 'boolean')) {
                throw new Error('检测开关返回格式无效');
            }
            if (latest && value.version < latest.version && !value.error) return;
            latest = value; seenAt = Date.now();
        }
        async function request(payload) {
            const own = new root.AbortController(); controller = own;
            const timeout = root.setTimeout(() => own.abort(), payload ? 45000 : 5000);
            try {
                const response = await fetcher(base + '/api/detection/settings', {
                    method: payload ? 'POST' : 'GET', cache: 'no-store', signal: own.signal,
                    ...(payload ? {headers: {'Content-Type': 'application/json'}, body: JSON.stringify(payload)} : {})
                });
                const value = await response.json();
                if (!response.ok || value.success === false) throw new Error(value.error || `HTTP ${response.status}`);
                return value;
            } finally {
                root.clearTimeout(timeout);
                if (controller === own) controller = null;
            }
        }
        async function refresh() {
            if (stopped || busy || controller || document.hidden) return;
            const own = generation;
            try {
                const value = await request();
                if (!stopped && own === generation) {
                    receive(value);
                    if (readError) note.textContent = '检测配置连接已恢复。';
                    readError = false;
                }
            }
            catch (error) {
                if (!stopped && own === generation) { seenAt = 0; readError = true; note.textContent = `读取失败：${error.message}`; }
            } finally { render(); }
        }
        async function change(key, value) {
            if (!fresh() || busy) { render(); return; }
            generation += 1; controller?.abort(); controller = null;
            const own = generation;
            readError = false;
            pending = {key, value}; busy = true; render();
            try {
                const saved = await request({changes: {[key]: value}, expected_version: latest.version});
                if (stopped || own !== generation) return;
                receive(saved);
                const label = ITEMS.find(item => item[0] === key)[1];
                note.textContent = saved.apply_warning || `${label}已${value ? '开启' : '关闭'}并保存；后台从下一轮处理应用。`;
                log(note.textContent);
            } catch (error) {
                if (!stopped && own === generation) {
                    seenAt = 0;
                    note.textContent = `保存结果未确认：${error.message}。正在重新读取实际设置。`;
                    log(note.textContent);
                }
            } finally {
                if (own === generation) { pending = null; busy = false; render(); refresh(); }
            }
        }
        refreshButton.addEventListener('click', refresh);
        const visibility = () => { if (!document.hidden) refresh(); };
        document.addEventListener('visibilitychange', visibility);
        render();
        return {
            refresh,
            start() {
                stopped = false;
                if (interval !== null) root.clearInterval(interval);
                refresh(); interval = root.setInterval(() => { render(); refresh(); }, 2000);
            },
            stop() {
                stopped = true; generation += 1; controller?.abort(); controller = null;
                if (interval !== null) root.clearInterval(interval);
                interval = null; document.removeEventListener('visibilitychange', visibility); render();
            }
        };
    }
    root.DetectionSettingsUI = Object.freeze({create});
}(window));
