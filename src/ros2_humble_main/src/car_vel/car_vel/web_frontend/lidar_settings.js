(function (root) {
    'use strict';
    function create(log) {
        const el = name => document.getElementById('lidar-' + name);
        let session, online = false, busy = false, dirty = false, config = null, seen = 0, version = 0;
        function render() {
            el('save').disabled = !online || busy || !config || Date.now() - seen > 6000;
            el('refresh').disabled = !online || busy;
            el('ip').disabled = el('host-ip').disabled = busy;
            if (!online || Date.now() - seen > 6000) el('state').textContent = '雷达配置服务离线或状态过期';
        }
        function receive(data) {
            try {
                const status = typeof data === 'string' ? JSON.parse(data) : data;
                if (!status || !('config' in status)) return;
                config = status.config; seen = Date.now();
                if (config && !dirty && !busy) {
                    el('ip').value = config.lidar_ip; el('host-ip').value = config.host_ip;
                    version = config.version;
                }
                el('state').textContent = status.error ? `配置读取失败：${status.error}`
                    : `已保存配置 v${config.version}；下次启动雷达生效，不代表当前运行地址。${status.warning || ''}`;
                render();
            } catch (_error) { /* Do not enable saving on malformed status. */ }
        }
        ['ip', 'host-ip'].forEach(name => el(name).addEventListener('input', () => { dirty = true; }));
        async function run(operation) {
            if (el(operation === 'save' ? 'save' : 'refresh').disabled) return;
            if (operation === 'save' && !el('form').reportValidity()) return;
            const request = {operation, lidar_ip: el('ip').value.trim(),
                host_ip: el('host-ip').value.trim(), expected_version: version};
            busy = true; render();
            try {
                const response = await session.call('/lidar/set_network', 'lzp_interface/srv/SetLidarNetwork', request, 8000);
                busy = false;
                if (response.success) dirty = false;
                receive(response.status_json);
                el('result').textContent = response.message; log(`雷达网络：${response.message}`);
            } catch (error) {
                el('result').textContent = `未确认保存结果：${error.message}；请刷新核对配置。`;
            } finally { busy = false; render(); }
        }
        el('save').addEventListener('click', () => run('save'));
        el('refresh').addEventListener('click', () => run('get'));
        root.setInterval(render, 1000);
        return {
            connection(value) { online = value; if (!value) seen = 0; render(); },
            attach(value) {
                session = value;
                session.subscribe('/lidar/network_settings', root.RobotUi.TYPES.string,
                    message => receive(message.data), {queue_length: 1});
            },
        };
    }
    root.LidarSettings = {create};
})(window);
