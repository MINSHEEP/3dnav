(() => {
    const button = document.getElementById('inspection-record-button');
    const message = document.getElementById('inspection-record-status');
    const controls = document.getElementById('inspection-recording-controls');
    const host = location.hostname.includes(':') ? `[${location.hostname}]` : location.hostname;
    const url = `http://${host}:8766/api/c12/recording`;
    let state = 'idle', busy = false, revision = 0;

    function render(data) {
        state = data.state;
        const active = state === 'starting' || state === 'recording';
        controls.hidden = !window.RecordingVisibility.get() && !active && state !== 'stopping';
        button.textContent = state === 'stopping' ? '正在保存录像…' : active ? '停止录像' : '开始双路录像';
        button.disabled = busy || state === 'stopping';
        const files = data.files || {};
        const paths = files.visible && files.infrared
            ? `可见光：${files.visible}；热成像：${files.infrared}` : '';
        const text = {idle: '未录像', starting: '正在连接两路视频…',
                      recording: '双路录像中', stopping: '正在保存录像…', error: '录像已停止'};
        message.textContent = [text[state] || state, data.error, paths].filter(Boolean).join('。');
    }
    async function refresh() {
        if (busy) return;
        const current = revision;
        try {
            const response = await fetch(url, {cache:'no-store'});
            if (!response.ok) throw new Error('录像服务不可用');
            const data = await response.json();
            if (current === revision) render(data);
        } catch (error) {
            if (current === revision) message.textContent = `录像状态读取失败：${error.message}`;
        }
    }
    button.addEventListener('click', async () => {
        if (busy) return;
        busy = true;
        revision += 1;
        button.disabled = true;
        try {
            const response = await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'},
                body:JSON.stringify({operation: state === 'starting' || state === 'recording' ? 'stop' : 'start'})});
            const data = await response.json();
            if (!response.ok) throw new Error(data.error || '录像操作失败');
            render(data);
        } catch (error) {
            message.textContent = `录像操作失败：${error.message}`;
        } finally {
            busy = false;
            button.disabled = state === 'stopping';
        }
    });
    window.addEventListener('storage', event => {
        if (event.key === window.RecordingVisibility.key) refresh();
    });
    refresh();
    setInterval(refresh, 3000);
})();
