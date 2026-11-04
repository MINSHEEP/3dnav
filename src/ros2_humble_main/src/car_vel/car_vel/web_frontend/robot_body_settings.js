(function (root) {
    'use strict';
    function create(log) {
        const get = id => document.getElementById(id);
        const model = get('robot-body-select');
        const state = get('robot-body-state');
        const apply = get('robot-body-apply');
        const inputs = {
            robot_ip: get('robot-body-ip'), local_ip: get('robot-body-local-ip'),
            port: get('robot-body-port'), max_vx: get('robot-body-max-vx'),
            max_vy: get('robot-body-max-vy'), max_yaw: get('robot-body-max-yaw'),
            navigation_verified: get('robot-body-navigation-verified'),
        };
        let session, latest = null, online = false, busy = false;
        let dirty = false, formVersion = 0, seenAt = 0;

        function populate() {
            if (!latest || !latest.config) return;
            const profile = latest.config.profiles[model.value];
            if (!profile) return;
            Object.entries(inputs).forEach(([key, input]) => {
                if (input.type === 'checkbox') input.checked = profile[key] === true;
                else input.value = String(profile[key]);
            });
            formVersion = latest.config.version;
            const sdk = model.value === 'gangben_zsl1';
            get('robot-body-sdk-options').hidden = !sdk;
            get('robot-body-port-label').textContent = sdk ? '小电脑 SDK 接收端口' : '星玑控制端口';
        }

        function render() {
            const fresh = online && Date.now() - seenAt < 3000;
            apply.disabled = Boolean(busy || !fresh || !latest || !latest.config
                || latest.locked !== false || latest.switching);
            model.disabled = busy;
            Object.values(inputs).forEach(input => { input.disabled = busy; });
            if (!fresh) {
                state.textContent = '未收到本体管理状态，切换不可用';
                return;
            }
            const selected = latest.config && latest.config.selected;
            const active = latest.active_body;
            const names = latest.names || {};
            state.textContent = `已保存：${names[selected] || '未配置'}；当前连接：`
                + (latest.connected ? (names[active] || '未知') : '未连接')
                + (latest.switching ? '；正在切换' : '');
            get('robot-body-note').textContent = latest.error || latest.lock_reason
                || '切换前让旧本体趴下、退出遥控，并停止建图、导航和巡检程序。';
        }

        function receive(message) {
            try {
                const data = typeof message === 'string' ? JSON.parse(message) : message;
                if (!data || typeof data !== 'object') return;
                const first = !latest;
                latest = data;
                seenAt = Date.now();
                if (first && latest.config) model.value = latest.config.selected;
                if (!dirty && !busy) populate();
                render();
            } catch (_error) { /* a malformed status must not enable switching */ }
        }

        model.addEventListener('change', () => { populate(); dirty = true; });
        Object.values(inputs).forEach(input => input.addEventListener('input', () => { dirty = true; }));
        apply.addEventListener('click', async () => {
            if (apply.disabled || !session) return;
            const profile = {};
            Object.entries(inputs).forEach(([key, input]) => {
                profile[key] = input.type === 'checkbox' ? input.checked
                    : input.type === 'number' ? Number(input.value) : input.value.trim();
            });
            if (!get('robot-body-form').reportValidity()) return;
            if (!root.confirm('确认应用所选本体？旧本体必须已安全趴下。切换将重新建立底盘连接，不会自动站立或恢复任务。钢镚 SDK 占用期间手柄不能正常接管。')) return;
            busy = true;
            render();
            log('正在检查旧本体并应用本体配置…');
            try {
                const response = await session.call('/robot_body/select',
                    'lzp_interface/srv/SetRobotBody', {
                        body_id: model.value, profile_json: JSON.stringify(profile),
                        expected_version: formVersion,
                    }, 25000);
                log(response.message || (response.success ? '本体切换完成' : '本体切换失败'));
                dirty = !response.success;
                if (response.status_json) receive(response.status_json);
            } catch (error) {
                log(`未确认切换结果：${error.message}。请查看“已保存/当前连接”状态，不要重复启动控制程序。`);
            } finally {
                busy = false;
                if (!dirty) populate();
                render();
            }
        });
        root.setInterval(render, 1000);
        return {
            connection(connected) { online = connected; if (!connected) seenAt = 0; render(); },
            attach(value) {
                session = value;
                session.subscribe('/robot_body/status', root.RobotUi.TYPES.string,
                    message => receive(message.data), {queue_length: 1});
            },
        };
    }
    root.RobotBodySettings = {create};
})(window);
