(() => {
    const key = 'robot.inspection.recording.visible';
    function get() {
        try { return localStorage.getItem(key) === 'true'; }
        catch (_) { return false; }
    }
    window.RecordingVisibility = {key, get};
    const setting = document.getElementById('recording-visible-setting');
    if (setting) {
        setting.checked = get();
        setting.addEventListener('change', () => {
            try { localStorage.setItem(key, String(setting.checked)); }
            catch (_) { setting.checked = false; }
        });
        window.addEventListener('storage', event => {
            if (event.key === key) setting.checked = get();
        });
    }
})();
