(function (root) {
    'use strict';
    function create(log) {
        const get = id => document.getElementById('m701-' + id);
        const mode = get('mode'), device = get('device'), port = get('port');
        const baud = get('baudrate'), address = get('address'), confirm = get('confirm');
        const refresh = get('refresh'), test = get('test'), save = get('save');
        let session, latest = null, online = false, busy = false, dirty = false;
        let seenAt = 0, version = 0, selectedIdentity = null;
        const equal = (a, b) => a && b && Object.keys(a).length === Object.keys(b).length
            && Object.keys(a).every(key => a[key] === b[key]);
        function selected() {
            return (latest?.devices || []).find(d => d.port === device.value
                && equal(d.identity, selectedIdentity));
        }
        function render() {
            const fresh = online && latest && Date.now() - seenAt < 6000;
            refresh.disabled = !online || busy;
            test.disabled = save.disabled = !fresh || busy || !confirm.checked || !selected();
            [mode, device, baud, address, confirm].forEach(el => { el.disabled = busy; });
            port.disabled = busy || mode.value === 'auto';
            const detail = selected();
            get('device-detail').textContent = detail
                ? `USB ${detail.vid}:${detail.pid}；驱动 ${detail.driver || '未知'}；序列号 ${detail.serial || '无（绑定 USB 插口）'}；位置 ${detail.location}`
                : '请选择设备；设备更换后需重新确认。';
            get('connection-state').textContent = !fresh ? '连接服务离线或状态已过期'
                : latest.config_error || `当前串口：${latest.active_port || '未连接'}；${latest.error?.message || ''}`;
            if (fresh && !latest.config_error && latest.diagnostic?.code !== 'configured'
                    && latest.diagnostic?.message && latest.diagnostic.message !== latest.error?.message) {
                get('connection-state').textContent += `；${latest.diagnostic.message}`;
            } else if (fresh && !latest.diagnostic && latest.usb_without_serial?.length) {
                get('connection-state').textContent += '；发现 WCH USB 设备但未发现对应串口，请核对芯片型号及驱动';
            }
        }
        function receive(data) {
            try {
                const value = typeof data === 'string' ? JSON.parse(data) : data;
                if (!value?.config || !Array.isArray(value.devices)) return;
                const previousPort = device.value;
                const previousIdentity = selectedIdentity;
                latest = value; seenAt = Date.now();
                device.replaceChildren();
                const placeholder = document.createElement('option');
                placeholder.value = ''; placeholder.textContent = '请选择并确认设备'; device.appendChild(placeholder);
                for (const item of value.devices) {
                    const option = document.createElement('option');
                    option.value = item.port;
                    option.textContent = `${item.port} — ${item.product} (${item.vid}:${item.pid}, ${item.driver || '未知驱动'})`;
                    device.appendChild(option);
                }
                if (!dirty && !busy) {
                    mode.value = value.config.mode; port.value = value.config.port;
                    baud.value = String(value.config.baudrate); address.value = String(value.config.address);
                    version = value.config.version;
                    const matches = value.devices.filter(d => value.config.binding
                        ? Object.entries(value.config.binding).every(([key, v]) => d[key] === v)
                        : [d.port, ...(d.aliases || [])].includes(value.config.port));
                    let choice = matches.length === 1 ? matches[0] : null;
                    if (!value.config_error && value.diagnostic?.suggested_port) {
                        const suggestions = value.devices.filter(d => d.port === value.diagnostic.suggested_port);
                        if (suggestions.length === 1) {
                            choice = suggestions[0];
                            mode.value = 'auto';
                            port.value = choice.port;
                        }
                    }
                    device.value = choice?.port || '';
                    selectedIdentity = choice?.identity || null;
                } else {
                    device.value = previousPort;
                }
                if (!selected() || !equal(previousIdentity, selectedIdentity)) confirm.checked = false;
                render();
            } catch (_error) { /* Malformed packets never authorize an operation. */ }
        }
        device.addEventListener('change', () => {
            const value = latest?.devices.find(d => d.port === device.value);
            selectedIdentity = value?.identity || null;
            port.value = value ? (value.aliases?.[0] || value.port) : '';
            dirty = true; confirm.checked = false; render();
        });
        [mode, port, baud, address].forEach(el => el.addEventListener('input', () => {
            dirty = true; confirm.checked = false; render();
        }));
        confirm.addEventListener('change', render);
        async function run(operation) {
            const button = {list: refresh, test, save}[operation];
            if (button.disabled || !session) return;
            if (operation !== 'list' && !get('connection-form').reportValidity()) return;
            const chosen = selected();
            const config = operation === 'list' ? {} : {
                mode: mode.value, port: mode.value === 'auto' ? chosen.port : port.value.trim(),
                baudrate: Number(baud.value), address: Number(address.value),
                identity: chosen.identity, confirmed: confirm.checked,
            };
            busy = true; render();
            get('result').textContent = operation === 'list' ? '正在枚举设备…' : '正在操作已确认设备；采样可能短暂中断…';
            try {
                const response = await session.call('/air_quality/m701/configure',
                    'lzp_interface/srv/ConfigureM701', {
                        operation, config_json: JSON.stringify(config), expected_version: version,
                    }, 8000);
                busy = false;
                if (operation === 'save' && response.success) { dirty = false; confirm.checked = false; }
                if (response.status_json) receive(response.status_json);
                get('result').textContent = response.message;
                log(`气体传感器：${response.message}`);
            } catch (error) {
                get('result').textContent = `未确认操作结果：${error.message}。请刷新核对已保存配置。`;
            } finally { busy = false; render(); }
        }
        refresh.addEventListener('click', () => {
            dirty = false; confirm.checked = false; run('list');
        });
        test.addEventListener('click', () => run('test'));
        save.addEventListener('click', () => run('save'));
        root.setInterval(render, 1000);
        return {
            connection(value) { online = value; if (!value) { seenAt = 0; confirm.checked = false; } render(); },
            attach(value) {
                session = value;
                session.subscribe('/air_quality/m701/connection', root.RobotUi.TYPES.string,
                    message => receive(message.data), {queue_length: 1});
            },
        };
    }
    root.M701Settings = {create};
})(window);
