/* Shared, read-only-on-attach behavior status and exact-inference-frame preview. */
(function (global) {
    'use strict';
    const bindings = new WeakMap();
    const NAMES = Object.freeze({ fall: '疑似跌倒', lying: '躺卧姿态（未确认跌倒）', fight: '疑似打架', smoking: '疑似吸烟' });
    const BRANCHES = ['fall', 'fight', 'smoking'];
    // Fight evidence expires after one second; poll sooner so a healthy stream
    // does not repeatedly disappear between ordinary 1.5-second status polls.
    const POLL_MS = 500;
    const REQUEST_TIMEOUT_MS = 5000;

    function categoryLabel(category) { return NAMES[category] || category || '行为告警'; }
    function evidenceLabel(alert) {
        if (alert.category === 'fall' || alert.category === 'lying'
            || alert.score_kind === 'rule_evidence') return '连续姿态规则证据';
        if (alert.category === 'fight') {
            const value = alert.confidence;
            return value != null && Number.isFinite(Number(value))
                ? `模型分数 ${Math.round(Number(value) * 100)}%` : '场景行为证据';
        }
        if (alert.category === 'smoking') {
            const score = alert.confidence;
            return score != null && Number.isFinite(Number(score))
                ? `人物关联检测分数 ${Math.round(Number(score) * 100)}%（非行为概率）` : '人物与吸烟动作关联证据';
        }
        return `${Math.round(Number(alert.confidence || 0) * 100)}%`;
    }
    function branchLabel(status, key) {
        const branch = status?.branches?.[key] || {};
        if (branch.enabled === false || status?.enabled?.[key] === false) return '已停止（未启用）';
        if (['error', 'unavailable'].includes(branch.state) || (!status?.running && status?.error)) return '不可用';
        if (branch.state === 'starting') return '启动中';
        if (!status?.running || branch.state === 'stopped') return '已停止';
        if (branch.state === 'valid' && branch.ready === true && status.fresh === true && branch.fresh !== false) {
            return NAMES[branch.label] ? `就绪 · ${NAMES[branch.label]}` : '就绪';
        }
        if (branch.state === 'stale') return '等待连续观测（画面已过期）';
        if (branch.stale_results > 0 && branch.fresh === false) return '推理结果已过期（处理过慢）';
        const window = branch.window || branch.capture_window;
        if (key === 'fight' && window && Number.isFinite(window.samples)) {
            return `采集连续画面 ${window.samples}/${window.required_samples || 8}`;
        }
        return '等待连续观测';
    }
    function recognitionLabel(detection) {
        const yolo = detection?.yolo || {};
        const thermal = detection?.thermal || {};
        const behavior = detection?.behavior || {};
        const enabled = BRANCHES.some(key => behavior.enabled?.[key] === true);
        const required = detection?.requirements || {yolo: true, thermal: true, behavior: enabled};
        if (!required.yolo && !required.thermal && !enabled) return '识别项目已全部关闭，可在设置页开启。';
        const running = Boolean(detection?.running || yolo.running || thermal.manual_detection_enabled || behavior.running);
        if (!running) return '识别已停止。';
        if ((required.yolo && (yolo.error || !yolo.running)) || (required.thermal && thermal.error)
            || (enabled && (!behavior.running || behavior.error
                || Object.values(behavior.branches || {}).some(branch => branch.enabled && branch.state === 'error')))) {
            return '识别部分不可用，请查看各项状态。';
        }
        if ((required.yolo && !yolo.ready) || (required.thermal && !thermal.ready) || (enabled && !behavior.ready)) return '识别启动中，等待各项就绪。';
        return '视觉识别、高温检测及已启用行为识别已就绪。';
    }

    function attach(root, options = {}) {
        if (!root) return null;
        bindings.get(root)?.dispose();
        root.innerHTML = '<div class="behavior-head"><strong>人员行为识别</strong>'
            + '<span data-behavior-state aria-live="polite">检查中</span></div>'
            + '<div class="behavior-branches">'
            + '<label><input type="checkbox" data-behavior-toggle="fall" disabled> 疑似跌倒'
            + '<span data-behavior-branch="fall">检查中</span></label>'
            + '<label><input type="checkbox" data-behavior-toggle="fight" disabled> 疑似打架'
            + '<span data-behavior-branch="fight">检查中</span></label>'
            + '<label><input type="checkbox" data-behavior-toggle="smoking" disabled> 疑似吸烟'
            + '<span data-behavior-branch="smoking">检查中</span></label></div>'
            + '<p class="hint" data-behavior-note>正在读取状态；开关仅设置识别项目，启动识别或巡检后生效。</p>';
        const inputs = BRANCHES.map(key => root.querySelector(`[data-behavior-toggle="${key}"]`));
        const image = options.image;
        const message = options.message;
        const baseUrl = options.baseUrl || '';
        const fetcher = options.fetch || global.fetch.bind(global);
        let status = null, disposed = false, previewActive = false, busy = false;
        let statusTimer = null, expiryTimer = null, statusRequest = null, frameRequest = null, configRequest = null;
        let statusGeneration = 0, frameGeneration = 0, objectUrl = null, lastFrameKey = '';
        let lastAttemptAt = 0, sourceGeneration;
        const listeners = [];
        const listen = (target, name, fn) => {
            target.addEventListener(name, fn);
            listeners.push(() => target.removeEventListener(name, fn));
        };
        const allowed = () => !disposed && !global.document.hidden
            && (!options.isPreviewAllowed || options.isPreviewAllowed());
        function clearPreview(text = '等待当前来源的行为识别画面…') {
            frameGeneration += 1;
            frameRequest?.abort();
            frameRequest = null;
            lastFrameKey = '';
            if (objectUrl) global.URL.revokeObjectURL(objectUrl);
            objectUrl = null;
            if (image && previewActive) {
                image.removeAttribute('src');
                image.classList.remove('ready');
            }
            if (message && previewActive) { message.hidden = false; message.textContent = text; }
        }
        async function request(path, options = {}, kind) {
            const controller = new global.AbortController();
            if (kind === 'status') statusRequest = controller;
            if (kind === 'frame') frameRequest = controller;
            if (kind === 'config') configRequest = controller;
            const timeout = global.setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS);
            try {
                const response = await fetcher(`${baseUrl}${path}`, {
                    cache: 'no-store', ...options, signal: controller.signal
                });
                if (!response.ok) throw new Error(`HTTP ${response.status}`);
                // Body consumption remains covered by the request deadline.
                return kind === 'frame'
                    ? { blob: await response.blob(), headers: response.headers }
                    : await response.json();
            } finally {
                global.clearTimeout(timeout);
                if (kind === 'status' && statusRequest === controller) statusRequest = null;
                if (kind === 'frame' && frameRequest === controller) frameRequest = null;
                if (kind === 'config' && configRequest === controller) configRequest = null;
            }
        }
        function render() {
            if (disposed) return;
            const stateNode = root.querySelector('[data-behavior-state]');
            if (!status) stateNode.textContent = '不可用';
            else if (status.error) stateNode.textContent = '不可用';
            else if (!status.running) stateNode.textContent = '已停止';
            else if (status.ready && status.fresh) stateNode.textContent = '就绪';
            else if (Object.values(status.branches || {}).some(branch => branch.state === 'starting')) stateNode.textContent = '启动中';
            else stateNode.textContent = '等待连续观测';
            BRANCHES.forEach((key, index) => {
                const known = typeof status?.enabled?.[key] === 'boolean';
                inputs[index].disabled = busy || !known;
                if (known) inputs[index].checked = status.enabled[key];
                root.querySelector(`[data-behavior-branch="${key}"]`).textContent = status ? branchLabel(status, key) : '不可用';
            });
            root.querySelector('[data-behavior-note]').textContent = !status
                ? '行为识别服务暂不可用，正在重试。'
                : (status.error ? `${status.error}。` : '')
                    + '开关仅设置识别项目，启动识别或巡检后生效。疑似事件需人工核实；等待观测不代表正常。'
                    + (status.branches?.fight?.timing_ms?.fight != null
                        ? ` 打架推理 ${Math.round(status.branches.fight.timing_ms.fight)} 毫秒。` : '')
                    + (status.preview_branch ? ` 当前画面：${status.preview_branch === 'fight' ? '打架' : '人物姿态与吸烟'}检测结果。` : '')
                    + (status.speech?.error ? ` 语音：${status.speech.error}` : '');
            options.onStatus?.(status);
        }
        async function refreshFrame() {
            if (!previewActive || !allowed() || frameRequest) return;
            if (!status?.running || status.fresh !== true || !status.stamp_ns) {
                clearPreview(status?.running ? '等待连续观测，当前没有可用行为画面。' : '行为识别已停止，请启动识别或巡检。');
                return;
            }
            const epoch = String(status.source_epoch), stamp = String(status.frame_stamp_ns || status.stamp_ns);
            const key = `${epoch}:${stamp}`;
            if (lastFrameKey === key || Date.now() - lastAttemptAt < POLL_MS) return;
            lastAttemptAt = Date.now();
            const generation = frameGeneration;
            try {
                const frame = await request(`/api/behavior/frame.jpg?epoch=${encodeURIComponent(epoch)}&stamp=${encodeURIComponent(stamp)}`, {}, 'frame');
                if (disposed || generation !== frameGeneration || !previewActive || !allowed()) return;
                if (String(frame.headers.get('X-Behavior-Source-Epoch')) !== epoch
                    || String(frame.headers.get('X-Behavior-Stamp-Ns')) !== stamp
                    || String(status?.source_epoch) !== epoch
                    || String(status?.frame_stamp_ns || status?.stamp_ns) !== stamp || !status?.fresh || !status?.running) {
                    clearPreview('画面已更新，正在获取当前识别结果…');
                    return;
                }
                const previous = objectUrl;
                objectUrl = global.URL.createObjectURL(frame.blob);
                image.src = objectUrl;
                lastFrameKey = key;
                if (previous) global.URL.revokeObjectURL(previous);
            } catch (_error) {
                if (disposed || generation !== frameGeneration) return;
                clearPreview('行为画面暂不可用，正在重试…');
            }
        }
        function schedule() {
            global.clearTimeout(statusTimer);
            if (!disposed && !global.document.hidden) statusTimer = global.setTimeout(refresh, POLL_MS);
        }
        async function refresh() {
            if (disposed || global.document.hidden || busy || statusRequest) return;
            global.clearTimeout(statusTimer);
            const generation = statusGeneration;
            try {
                const incoming = await request('/api/behavior/status', {}, 'status');
                if (disposed || generation !== statusGeneration) return;
                if (status && String(status.source_epoch) !== String(incoming.source_epoch)) clearPreview();
                status = incoming;
                global.clearTimeout(expiryTimer);
                if (status.fresh) {
                    const remaining = Math.max(0, (Number(status.max_result_age_seconds || 5)
                        - Number(status.frame_age_seconds || 0)) * 1000);
                    expiryTimer = global.setTimeout(() => {
                        if (disposed || status !== incoming) return;
                        status = { ...status, fresh: false, ready: false };
                        clearPreview('行为画面已过期，等待新的连续观测。');
                        render();
                    }, remaining);
                }
                render();
                if (!status.running || !status.fresh) clearPreview();
                await refreshFrame();
            } catch (_error) {
                if (disposed || generation !== statusGeneration) return;
                status = null;
                clearPreview('行为识别服务失联，正在重试…');
                render();
            } finally { if (generation === statusGeneration) schedule(); }
        }
        function invalidate() {
            statusGeneration += 1;
            statusRequest?.abort(); statusRequest = null;
            global.clearTimeout(statusTimer);
            global.clearTimeout(expiryTimer);
            clearPreview();
        }
        async function configure(key, input) {
            if (disposed || busy) return;
            const previous = status;
            const enabled = { ...status.enabled, [key]: Boolean(input.checked) };
            busy = true;
            invalidate();
            status = { ...status, enabled, fresh: false, ready: false };
            render();
            try {
                const incoming = await request('/api/behavior/config', {
                    method: 'POST', headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ fall_enabled: enabled.fall, fight_enabled: enabled.fight, smoking_enabled: enabled.smoking,
                        ...(Number.isInteger(status.settings_version) ? {expected_version: status.settings_version} : {}) })
                }, 'config');
                if (!disposed) status = incoming;
            } catch (_error) {
                if (!disposed) {
                    status = { ...previous, fresh: false, ready: false };
                    options.onError?.('行为识别设置未保存，请检查相机服务后重试。');
                }
            } finally {
                if (!disposed) { busy = false; render(); refresh(); }
            }
        }
        BRANCHES.forEach((key, index) => listen(inputs[index], 'change', () => configure(key, inputs[index])));
        if (image) {
            listen(image, 'load', () => {
                if (!previewActive || !objectUrl || image.getAttribute('src') !== objectUrl || !allowed()) return;
                image.classList.add('ready');
                if (message) message.hidden = true;
            });
            listen(image, 'error', () => { if (previewActive) clearPreview('行为画面暂不可用，正在重试…'); });
        }
        function visibilityChanged() {
            invalidate();
            if (!global.document.hidden) refresh();
        }
        listen(global.document, 'visibilitychange', visibilityChanged);
        listen(global, 'robot-preview-change', () => { clearPreview(); refreshFrame(); });
        function dispose() {
            if (disposed) return;
            disposed = true;
            invalidate();
            configRequest?.abort(); configRequest = null;
            listeners.splice(0).forEach(remove => remove());
            if (bindings.get(root) === handle) bindings.delete(root);
        }
        listen(global, 'pagehide', dispose);
        const handle = {
            refresh,
            refreshNow() { invalidate(); status = null; render(); refresh(); },
            get status() { return status; },
            setPreviewActive(active) {
                const changed = previewActive !== Boolean(active);
                if (changed || !allowed()) clearPreview();
                previewActive = Boolean(active);
                if (previewActive) {
                    if (changed && image) { image.removeAttribute('src'); image.classList.remove('ready'); }
                    if (!allowed()) clearPreview('普通画面预览已暂停。');
                    else refreshFrame();
                }
            },
            observeSource(generation) {
                if (generation == null || generation === sourceGeneration) return;
                const hadSource = sourceGeneration !== undefined;
                sourceGeneration = generation;
                if (hadSource) handle.refreshNow();
            },
            dispose
        };
        bindings.set(root, handle);
        refresh();
        return handle;
    }
    global.RobotBehavior = Object.freeze({ attach, branchLabel, categoryLabel, evidenceLabel, recognitionLabel });
})(window);
