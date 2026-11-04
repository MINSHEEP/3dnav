(function (root) {
    'use strict';
    function create(log) {
        const el = name => document.getElementById('mapping-filter-' + name);
        const toggle = el('enabled');
        const modes = ['protected', 'bypass', 'strict'];
        let session, online = false, busy = false, dirty = false;
        let status = null, seen = 0, generation = 0, readError = '';
        function label(mode) {
            return mode === 'strict' ? '开启（严格模式）'
                : mode === 'protected' ? '开启（保留地面）' : mode === 'bypass' ? '关闭' : '模式未报告';
        }
        function fresh() { return status && Date.now() - seen >= 0 && Date.now() - seen <= 6000; }
        function render() {
            const usable = online && fresh() && !status.load_error;
            toggle.disabled = !usable || busy;
            toggle.indeterminate = !online || !fresh();
            el('save').disabled = !usable || busy || !dirty;
            el('refresh').disabled = !online || busy;
            const failed = !online || !fresh() || Boolean(status && status.load_error);
            el('state').classList.toggle('failed', failed);
            el('state').classList.toggle('pending', !failed && status.pending);
            el('state').classList.toggle('ready', !failed && !status.pending);
            if (!online) {
                el('state-value').textContent = 'ROS2 已断开';
            } else if (!fresh()) {
                el('state-value').textContent = readError || '过滤配置未连接或状态已过期';
            } else if (status.load_error) {
                el('state-value').textContent = '配置损坏，已回退开启';
            } else if (status.pending) {
                el('state-value').textContent = '已保存，待下次建图生效';
            } else {
                el('state-value').textContent = status.mapping_running
                    ? `当前建图：${label(status.active_mode)}`
                    : `${status.version ? '已保存' : '启动默认'}：${label(status.mode)}`;
            }
            el('detail').textContent = status && online
                ? `${status.version ? '已保存' : '启动默认'}：${label(status.mode)}；`
                    + `当前建图：${status.mapping_running ? label(status.active_mode) : '未运行'}。`
                    + (status.mapping_running ? ' 当前模式按本次启动参数显示。' : '')
                    + (status.load_error ? ` 配置读取失败：${status.load_error}，请检查小电脑配置文件。`
                        : ' 保存只影响下一次启动建图，不会中断当前建图。')
                : '等待后台确认；此处未连接时不能判断过滤是否开启。';
        }
        function decode(raw) {
            const value = typeof raw === 'string' ? JSON.parse(raw) : raw;
            if (!value || value.schema_version !== 1 || !modes.includes(value.mode)
                || typeof value.enabled !== 'boolean' || value.enabled !== (value.mode !== 'bypass')
                || !Number.isInteger(value.version) || value.version < 0
                || typeof value.mapping_running !== 'boolean' || typeof value.pending !== 'boolean'
                || !(value.active_mode === null || modes.includes(value.active_mode))
                || value.apply_policy !== 'next_mapping_start') throw Error('配置状态格式或版本不匹配');
            return value;
        }
        function receive(raw) {
            try {
                const value = decode(raw);
                if (status && value.version < status.version) return false;
                status = value; seen = Date.now(); readError = '';
                if (!dirty && !busy) toggle.checked = value.enabled;
                render(); return true;
            } catch (error) {
                status = null; seen = 0; readError = error.message; render(); return false;
            }
        }
        async function refresh() {
            if (!online || busy || !session) return;
            const epoch = generation;
            busy = true; render();
            try {
                const response = await session.call('/mapping/filter/get', 'std_srvs/srv/Trigger', {}, 8000);
                if (epoch !== generation) return;
                if (!response.success) throw Error(response.message || '读取过滤配置失败');
                busy = false;
                if (!receive(response.message)) throw Error(readError || '未收到有效配置');
                el('result').textContent = dirty ? '后台状态已刷新；未保存的选择已保留。' : '';
            } catch (error) {
                if (epoch !== generation) return;
                el('result').textContent = `读取失败：${error.message}；请确认后台已更新，或稍后刷新状态。`;
            } finally {
                if (epoch === generation) { busy = false; render(); }
            }
        }
        async function save() {
            if (el('save').disabled || !session) return;
            const epoch = generation, wanted = toggle.checked;
            busy = true; render();
            try {
                const response = await session.call('/mapping/filter/set_enabled', 'std_srvs/srv/SetBool', {data: wanted}, 8000);
                if (epoch !== generation) return;
                if (!response.success) throw Error(response.message || '后台拒绝保存');
                const saved = decode(response.message);
                dirty = false; busy = false;
                if (status && saved.version < status.version) {
                    toggle.checked = status.enabled;
                    el('result').textContent = '保存后的配置已被其他连接更新，请核对最新状态。';
                } else {
                    receive(saved); toggle.checked = saved.enabled;
                    el('result').textContent = `已保存：${label(saved.mode)}。`
                        + (saved.pending ? '当前建图保持原模式，下次启动建图生效。' : '下次启动建图使用此设置。');
                }
                log(`建图动态障碍过滤：${el('result').textContent}`);
            } catch (error) {
                if (epoch !== generation) return;
                el('result').textContent = `未确认保存结果：${error.message}；请刷新核对后台配置，勿重复启动建图。`;
            } finally {
                if (epoch === generation) { busy = false; render(); }
            }
        }
        toggle.addEventListener('change', () => {
            dirty = !status || toggle.checked !== status.enabled;
            el('result').textContent = dirty ? '选择尚未保存；当前建图不会因此改变。' : '';
            render();
        });
        el('save').addEventListener('click', save);
        el('refresh').addEventListener('click', refresh);
        const timer = root.setInterval(render, 1000);
        render();
        return {
            connection(value) {
                online = Boolean(value);
                if (!online) { generation++; status = null; seen = 0; busy = false; dirty = false; }
                render();
            },
            attach(value) {
                session = value;
                const epoch = ++generation;
                session.subscribe('/mapping/filter/status', root.RobotUi.TYPES.string,
                    message => { if (epoch === generation && online) receive(message.data); }, {queue_length: 1});
                return refresh();
            },
            stop() {
                root.clearInterval(timer); generation++; online = false; status = null; seen = 0;
                busy = false; render();
            },
        };
    }
    root.MappingFilterSettings = {create};
})(window);
