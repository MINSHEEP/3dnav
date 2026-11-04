/* Standalone viewer for the globally selected camera chain; no robot control. */
(function () {
    'use strict';
    const UI = window.RobotUi;
    let host = window.location.hostname || '127.0.0.1';
    if (host.includes(':')) host = `[${host}]`;
    const base = `http://${host}:8766`;
    const channels = [
        {name:'ordinary', image:document.getElementById('standalone-visible'),
            message:document.getElementById('visible-message'), path:'/api/stream'},
        {name:'thermal', image:document.getElementById('standalone-infrared'),
            message:document.getElementById('infrared-message'), path:'/api/thermal/stream.mjpg'}
    ];
    let released = false;
    function stop(channel) {
        window.clearTimeout(channel.timer);
        channel.image.removeAttribute('src');
        channel.image.classList.remove('ready');
    }
    function connect(channel) {
        stop(channel);
        if (released || !UI.previewAllowed(channel.name)) return;
        channel.message.hidden = false;
        channel.message.textContent = '正在连接当前相机链路…';
        channel.image.src = UI.previewUrl(`${base}${channel.path}?t=${Date.now()}`);
    }
    function sync() { channels.forEach(connect); }
    for (const channel of channels) {
        channel.image.addEventListener('load', () => {
            channel.image.classList.add('ready');channel.message.hidden = true;
        });
        channel.image.addEventListener('error', () => {
            if (released || !UI.previewAllowed(channel.name)) return;
            channel.message.hidden = false;
            channel.message.textContent = '所选相机链路暂不可用，正在重连；可在相机设置中核对来源。';
            window.clearTimeout(channel.timer);
            channel.timer = window.setTimeout(() => connect(channel), 2000);
        });
    }
    UI.previewControls();
    window.addEventListener('robot-preview-change', sync);
    document.addEventListener('visibilitychange', sync);
    window.addEventListener('robot-camera-display', () => {
        const thermal = UI.cameraDisplay()?.thermal || {};
        document.getElementById('infrared-measurement').textContent = thermal.supports_temperature === false
            ? '红外视频 · 测温数据未接入' : `最高温 ${UI.temperatureText(thermal.max_temperature_c)}`;
    });
    window.addEventListener('pagehide', () => {released = true;channels.forEach(stop);});
    window.addEventListener('pageshow', () => {released = false;sync();});
    sync();
}());
