/* Shared browser-side helpers for all quadruped robot web pages. */
(function () {
    'use strict';

    const TYPES = Object.freeze({
        twist: 'geometry_msgs/Twist',
        poseStamped: 'geometry_msgs/PoseStamped',
        poseWithCovariance: 'geometry_msgs/PoseWithCovarianceStamped',
        relocalizationRequest: 'lzp_interface/msg/RelocalizationRequest',
        odometry: 'nav_msgs/msg/Odometry',
        terrainObservation: 'lzp_interface/msg/TerrainObservation',
        empty: 'std_msgs/msg/Empty',
        bool: 'std_msgs/msg/Bool',
        string: 'std_msgs/msg/String',
        trigger: 'std_srvs/srv/Trigger',
        occupancyGrid: 'nav_msgs/msg/OccupancyGrid',
        path: 'nav_msgs/msg/Path',
        pointCloud2: 'sensor_msgs/msg/PointCloud2',
        mapSave: 'lzp_interface/srv/MapSave',
        rebuildTerrain: 'lzp_interface/srv/RebuildTerrain',
        controlLaunch: 'lzp_interface/srv/ControlLaunch',
        mapList: 'lzp_interface/srv/GetMapList',
        switchMap: 'lzp_interface/srv/SwitchMap',
        currentMap: 'lzp_interface/msg/CurrentMap',
        deleteMap: 'lzp_interface/srv/DeleteMap',
        editedMap: 'lzp_interface/srv/GetEditedMap',
        setMapPixels: 'lzp_interface/srv/SetMapPixelIndices',
        routePoint: 'lzp_interface/msg/RoutePoint',
        route: 'lzp_interface/msg/Route',
        routeExecutionStatus: 'lzp_interface/msg/RouteExecutionStatus',
        saveRoute: 'lzp_interface/srv/SaveRoute',
        listRoutes: 'lzp_interface/srv/ListRoutes',
        getRoute: 'lzp_interface/srv/GetRoute',
        deleteRoute: 'lzp_interface/srv/DeleteRoute',
        startRoute: 'lzp_interface/srv/StartRoute',
        controlRoute: 'lzp_interface/srv/ControlRoute',
        getRouteExecutionState: 'lzp_interface/srv/GetRouteExecutionState',
        saveInspectionTemplate: 'lzp_interface/srv/SaveInspectionTemplate',
        getInspectionTemplate: 'lzp_interface/srv/GetInspectionTemplate',
        startInspectionTask: 'lzp_interface/srv/StartInspectionTask',
        controlInspectionTask: 'lzp_interface/srv/ControlInspectionTask',
        getInspectionState: 'lzp_interface/srv/GetInspectionState',
        listInspectionHistory: 'lzp_interface/srv/ListInspectionHistory',
        getInspectionHistory: 'lzp_interface/srv/GetInspectionHistory',
        deleteInspectionHistory: 'lzp_interface/srv/DeleteInspectionHistory',
        acknowledgeInspectionAlert: 'lzp_interface/srv/AcknowledgeInspectionAlert',
        queryInspectionReports: 'lzp_interface/srv/QueryInspectionReports',
        getInspectionReport: 'lzp_interface/srv/GetInspectionReport',
        reviewInspectionReport: 'lzp_interface/srv/ReviewInspectionReport',
        evaluateTerrain: 'lzp_interface/srv/EvaluateTerrain',
        planCampusRoute: 'lzp_interface/srv/PlanCampusRoute',
        getCampusTopology: 'lzp_interface/srv/GetCampusTopology',
        manageCampusTopology: 'lzp_interface/srv/ManageCampusTopology',
        startCampusMission: 'lzp_interface/srv/StartCampusMission',
        controlCampusMission: 'lzp_interface/srv/ControlCampusMission',
        setNavigationMode: 'lzp_interface/srv/SetNavigationMode',
        setNavigationSpeed: 'lzp_interface/srv/SetNavigationSpeed',
        check3dGround: 'lzp_interface/srv/Check3DGround',
        preview3dGoal: 'lzp_interface/srv/Preview3DGoal',
        startPreviewed3dGoal: 'lzp_interface/srv/StartPreviewed3DGoal',
        preview3dRoute: 'lzp_interface/srv/Preview3DRoute',
        setInflationRadius: 'lzp_interface/srv/SetInflationRadius',
        setInspectionEnvironmentConfig:
            'lzp_interface/srv/SetInspectionEnvironmentConfig'
    });

    function getRosbridgeUrl() {
        // The browser normally runs on the robot's HTTP server.  Reuse that host
        // instead of assuming that rosbridge is on the user's own computer.
        let host = window.location.hostname || 'localhost';
        if (host.includes(':') && !host.startsWith('[')) host = `[${host}]`;
        return `ws://${host}:9090`;
    }

    function createLogger(element) {
        return function log(message) {
            if (!element) return;
            const row = document.createElement('div');
            const stamp = document.createElement('span');
            stamp.className = 'log-time';
            stamp.textContent = `[${new Date().toLocaleTimeString()}] `;
            row.append(stamp, document.createTextNode(String(message)));
            element.appendChild(row);
            while (element.childNodes.length > 120) element.removeChild(element.firstChild);
            element.scrollTop = element.scrollHeight;
        };
    }

    const PREVIEW_KEY = 'robot.webPreview.profile';
    const PREVIEW_STREAMS_KEY = 'robot.webPreview.streams';
    let previewSelection = null;
    let cameraDisplay = null;
    const DISPLAY_CLOUDS = new Set(['/global_mapping_cloud', '/global_localization_map',
        '/cloud_mapping_dynamic', '/octomap_point_cloud_centers']);
    function previewProfile() {
        try { return window.localStorage.getItem(PREVIEW_KEY) === 'lan' ? 'lan' : 'remote'; }
        catch (_error) { return 'remote'; }
    }
    function previewAllowed(source) {
        if (document.hidden) return false;
        let selection = previewProfile() === 'lan' ? 'both' : 'ordinary';
        try { selection = window.localStorage.getItem(PREVIEW_STREAMS_KEY)
            || window.sessionStorage.getItem(PREVIEW_STREAMS_KEY) || selection; }
        catch (_error) { /* Storage may be disabled. */ }
        selection = previewSelection || selection;
        return selection === 'both' || selection === source;
    }
    function previewUrl(value) {
        const url = new URL(value, window.location.href);
        const source = {'/api/stream': 'raw', '/api/yolo/stream.mjpg': 'yolo',
                        '/api/thermal/stream.mjpg': 'thermal'}[url.pathname];
        if (!source) return value;
        url.pathname = '/api/display/stream';
        url.searchParams.set('source', source);
        url.searchParams.set('profile', previewProfile());
        return url.href;
    }
    function temperatureText(value) {
        return value !== null && value !== undefined && value !== '' && typeof value !== 'boolean'
            && Number.isFinite(Number(value)) ? `${Number(value).toFixed(1)} ℃` : '-- ℃';
    }
    function syncPreviewVisibility() {
        document.querySelectorAll('[data-camera-channel]').forEach(element => {
            element.hidden = !previewAllowed(element.dataset.cameraChannel);
        });
        document.querySelectorAll('[data-camera-grid]').forEach(grid => {
            const count = [...grid.querySelectorAll('[data-camera-channel]')].filter(el => !el.hidden).length;
            grid.hidden = count === 0;
            grid.classList.toggle('camera-single-view', count === 1);
        });
    }
    function previewControls({thermal = true} = {}) {
        const parent = document.querySelector('.topbar-statuses');
        if (!parent || document.getElementById('web-preview-streams')) return;
        const label = document.createElement('label');
        label.className = 'web-preview-controls';
        const caption = document.createElement('span');
        caption.textContent = `${previewProfile() === 'remote' ? '5G低带宽' : '局域网高清'} · 预览 `;
        label.appendChild(caption);
        const select = document.createElement('select');
        select.id = 'web-preview-streams';
        for (const [value, title] of [['ordinary', '普通画面'], ['thermal', '热成像'],
                                      ['both', '双路（流量增加）'], ['off', '全部关闭']]) {
            if (!thermal && (value === 'thermal' || value === 'both')) continue;
            const option = document.createElement('option');
            option.value = value; option.textContent = title; select.appendChild(option);
        }
        try { select.value = window.localStorage.getItem(PREVIEW_STREAMS_KEY)
            || window.sessionStorage.getItem(PREVIEW_STREAMS_KEY)
            || (previewProfile() === 'lan' ? 'both' : 'ordinary'); }
        catch (_error) { select.value = 'ordinary'; }
        if (!select.value) { select.value = 'ordinary'; previewSelection = 'ordinary'; }
        select.addEventListener('change', () => {
            previewSelection = select.value;
            try { window.localStorage.setItem(PREVIEW_STREAMS_KEY, select.value);
                window.sessionStorage.setItem(PREVIEW_STREAMS_KEY, select.value); }
            catch (_error) { /* Keep page controls functional without storage. */ }
            syncPreviewVisibility();
            window.dispatchEvent(new Event('robot-preview-change'));
        });
        label.appendChild(select); parent.appendChild(label);
        const settings = document.createElement('a');
        settings.href = 'settings.html#ordinary-camera-title';
        settings.textContent = '相机设置';
        parent.appendChild(settings);
        window.addEventListener('storage', event => {
            if (event.key !== PREVIEW_STREAMS_KEY) return;
            previewSelection = event.newValue || null;
            select.value = event.newValue || (previewProfile() === 'lan' ? 'both' : 'ordinary');
            syncPreviewVisibility();
            window.dispatchEvent(new Event('robot-preview-change'));
        });
        document.addEventListener('visibilitychange', syncPreviewVisibility);
        syncPreviewVisibility();
        let released = false, fetching = false, timer;
        async function refreshDisplay() {
            if (released || fetching || document.hidden || typeof window.fetch !== 'function') return;
            fetching = true;
            const controller = new AbortController();
            const timeout = window.setTimeout(() => controller.abort(), 4000);
            try {
                let host = window.location.hostname || '127.0.0.1';
                if (host.includes(':')) host = `[${host}]`;
                const response = await window.fetch(`http://${host}:8766/api/display/status`,
                    {cache:'no-store', signal:controller.signal});
                if (!response.ok) return;
                const status = await response.json();
                if (released) return;
                const changed = cameraDisplay && cameraDisplay.pipeline !== status.pipeline;
                cameraDisplay = status;
                caption.textContent = `${status.label} · ${previewProfile() === 'remote' ? '5G低带宽' : '局域网高清'} · 预览 `;
                for (const option of select.options) {
                    if (option.value === 'ordinary') option.textContent = status.pipeline === 'c12' ? '可见光' : '普通画面';
                    if (option.value === 'thermal') option.textContent = status.pipeline === 'c12' ? '红外' : '热成像';
                }
                document.querySelectorAll('[data-camera-label]').forEach(element => {
                    element.textContent = element.dataset.cameraLabel === 'ordinary'
                        ? status.visible_label : status.infrared_label;
                });
                document.querySelectorAll('[data-camera-pipeline-label]').forEach(element => {
                    element.textContent = status.label;
                });
                window.dispatchEvent(new Event('robot-camera-display'));
                if (changed) window.dispatchEvent(new Event('robot-preview-change'));
            } catch (_error) { /* Existing stream handlers display connection errors. */ }
            finally { fetching = false; window.clearTimeout(timeout); }
        }
        function start() { released = false; refreshDisplay(); timer = window.setInterval(refreshDisplay, 2000); }
        window.addEventListener('pagehide', () => { released = true; window.clearInterval(timer); });
        window.addEventListener('pageshow', event => { if (event.persisted) start(); });
        start();
    }

    class PreviewCloudSubscription {
        constructor(session, name, type, callback, options) {
            this.session = session; this.callbacks = new Set([callback]);
            this.name = '/web_preview/' + previewProfile() + name;
            this.type = type; this.options = options || {}; this.closed = false;
            this.connect();
        }
        connect() {
            if (this.closed || document.hidden || !this.session.cloudPreviewVisible
                || !this.session.connected || this.ros) return;
            const ros = new ROSLIB.Ros({url: this.session.url});
            this.ros = ros;
            ros.on('connection', () => {
                if (this.ros !== ros || this.closed) return;
                this.topic = new ROSLIB.Topic({...this.options, ros, name: this.name,
                    messageType: this.type, queueLength: 1});
                this.callbacks.forEach(callback => this.topic.subscribe(callback));
            });
            ros.on('error', () => {});
            ros.on('close', () => {
                if (this.ros !== ros) return;
                this.ros = null; this.topic = null;
                if (!this.closed && this.session.connected && !document.hidden) {
                    this.timer = window.setTimeout(() => this.connect(), 3000);
                }
            });
        }
        suspend() {
            window.clearTimeout(this.timer);
            const ros = this.ros; this.ros = null; this.topic = null;
            if (ros) ros.close();
        }
        unsubscribe(callback) {
            if (callback) {
                this.callbacks.delete(callback);
                if (this.topic) this.topic.unsubscribe(callback);
            } else this.callbacks.clear();
            if (!this.callbacks.size) {
                this.closed = true; this.suspend();
                this.session.cloudPreviews.delete(this);
            }
        }
    }

    // Map topics can have been sent before rosbridge discovered map_server.
    // GetMap uses reliable request/response QoS without relying on version-
    // specific rosbridge subscription extensions. The owner supplies a load
    // identity predicate; a cancelled/obsolete response never reaches the UI.
    class MapSnapshotRecovery {
        constructor(session, accept, log) {
            this.session = session; this.accept = accept; this.log = log || (() => {});
            this.generation = 0; this.key = ''; this.pending = null; this.timer = null;
        }
        cancel() {
            this.generation += 1; this.key = ''; this.pending = null;
            window.clearTimeout(this.timer); this.timer = null;
        }
        request(key, isCurrent) {
            if (!key || !isCurrent() || !this.session.connected) return Promise.resolve();
            if (this.key === key) return this.pending || Promise.resolve();
            this.cancel(); this.key = key;
            const generation = this.generation;
            const active = () => generation === this.generation && this.session.connected
                && !this.session.stopped && isCurrent();
            let attempts = 0;
            const run = async () => {
                if (!active()) return;
                attempts += 1;
                try {
                    const result = await this.session.call('/map_server/map', 'nav_msgs/srv/GetMap', {}, 20000);
                    if (!active()) return;
                    const message = result && result.map;
                    const info = message && message.info;
                    const cells = info && Number(info.width) * Number(info.height);
                    if (!info || !Number.isInteger(info.width) || !Number.isInteger(info.height)
                        || info.width < 1 || info.height < 1 || cells > 20000000
                        || !Number.isFinite(info.resolution) || info.resolution <= 0
                        || !message.header || message.header.frame_id !== 'map'
                        || !message.data || message.data.length !== cells) {
                        throw new Error('地图快照为空、尺寸不完整或坐标系无效');
                    }
                    if (this.accept(message) === false) throw new Error('地图栅格渲染未完成');
                    this.log(`已通过可靠地图快照补收二维栅格：${info.width} × ${info.height}。`);
                } catch (error) {
                    if (!active()) return;
                    this.log(`地图快照补收失败（${attempts}/3）：${error.message || error}`);
                    if (attempts < 3) this.timer = window.setTimeout(run, attempts * 1000);
                }
            };
            this.pending = run();
            return this.pending;
        }
    }

    function subscribePreviewHealth(session, log) {
        let previous = '';
        return session.subscribe('/web_preview/status', TYPES.string, message => {
            try {
                const state = JSON.parse(message.data);
                const cloud = state.topics && state.topics['/global_localization_map'];
                if (!cloud) return;
                const viewing = Object.values(cloud.viewers || {}).some(count => count > 0);
                const text = cloud.error ? `三维预览处理失败：${cloud.error}`
                    : !viewing ? '' : !cloud.input_subscribed ? '三维预览未连接源地图，请检查预览服务状态。'
                    : !cloud.has_frame ? '三维预览已订阅源地图，正在等待可靠地图缓存。' : '三维预览已收到源地图，可检查地图身份匹配和网页接收。';
                if (text && text !== previous) log(text);
                previous = text;
            } catch (_error) { /* Invalid diagnostic data cannot change navigation readiness. */ }
        }, {queueLength: 1, throttleRate: 2000});
    }

    class RosSession {
        constructor(options) {
            this.log = options.log || (() => {});
            this.onReady = options.onReady || (() => {});
            this.onState = options.onState || (() => {});
            this.url = options.url || getRosbridgeUrl();
            this.ros = null;
            this.connected = false;
            this.reconnectTimer = null;
            this.serviceCache = new Map();
            this.topicCache = new Map();
            this.stopped = false;
            this.cloudPreviews = new Set();
            this.cloudPreviewVisible = true;
            this.previewVisibility = () => this.cloudPreviews.forEach(topic =>
                document.hidden || !this.cloudPreviewVisible ? topic.suspend() : topic.connect());
            document.addEventListener?.('visibilitychange', this.previewVisibility);
        }

        connect() {
            this.stopped = false;
            if (this.ros) return;
            if (!window.ROSLIB) {
                this.log('ROS JavaScript 库未加载，无法连接 ROS2。');
                this.onState(false, 'ROS JavaScript 库未加载');
                return;
            }
            const ros = new ROSLIB.Ros({ url: this.url });
            this.ros = ros;
            ros.on('connection', () => {
                if (this.stopped || this.ros !== ros) return;
                this.connected = true;
                this.serviceCache.clear();
                this.topicCache.clear();
                this.onState(true, `已连接 ROS2（${this.url}）`);
                this.log(`已连接 ROS2：${this.url}`);
                this.onReady(this);
            });
            ros.on('error', (error) => {
                this.onState(false, 'ROS2 连接错误');
                this.log(`ROS2 连接错误：${error && error.message ? error.message : error}`);
            });
            ros.on('close', () => {
                const wasConnected = this.connected;
                this.connected = false;
                this.cloudPreviews.forEach(topic => topic.unsubscribe());
                this.ros = null;
                this.serviceCache.clear();
                this.topicCache.clear();
                this.onState(false, 'ROS2 已断开，正在重连…');
                if (wasConnected) this.log('ROS2 已断开，3 秒后自动重连。');
                if (!this.stopped) {
                    window.clearTimeout(this.reconnectTimer);
                    this.reconnectTimer = window.setTimeout(() => this.connect(), 3000);
                }
            });
        }

        stop() {
            this.stopped = true;
            this.cloudPreviews.forEach(topic => topic.unsubscribe());
            document.removeEventListener?.('visibilitychange', this.previewVisibility);
            window.clearTimeout(this.reconnectTimer);
            if (this.ros) this.ros.close();
            this.ros = null;
            this.connected = false;
        }

        setCloudPreviewVisible(visible) {
            this.cloudPreviewVisible = Boolean(visible);
            this.previewVisibility();
        }

        topic(name, messageType, options) {
            if (!this.ros) throw new Error('ROS2 未连接');
            const key = `${name}|${messageType}`;
            if (!this.topicCache.has(key)) {
                this.topicCache.set(key, new ROSLIB.Topic({
                    ros: this.ros,
                    name,
                    messageType,
                    ...(options || {})
                }));
            }
            return this.topicCache.get(key);
        }

        subscribe(name, messageType, callback, options) {
            if (messageType === TYPES.pointCloud2 && DISPLAY_CLOUDS.has(name)) {
                const topic = new PreviewCloudSubscription(this, name, messageType, callback, options);
                this.cloudPreviews.add(topic);
                return topic;
            }
            const topic = this.topic(name, messageType, options);
            topic.subscribe(callback);
            return topic;
        }

        publish(name, messageType, message) {
            if (!this.connected) return false;
            this.topic(name, messageType).publish(new ROSLIB.Message(message));
            return true;
        }

        service(name, serviceType) {
            if (!this.ros) throw new Error('ROS2 未连接');
            const key = `${name}|${serviceType}`;
            if (!this.serviceCache.has(key)) {
                this.serviceCache.set(key, new ROSLIB.Service({ ros: this.ros, name, serviceType }));
            }
            return this.serviceCache.get(key);
        }

        call(name, serviceType, request, timeoutMs = 15000) {
            if (!this.connected) return Promise.reject(new Error('ROS2 未连接'));
            return new Promise((resolve, reject) => {
                let settled = false;
                const deadline = Math.max(1000, Number(timeoutMs) || 15000);
                const finish = (callback) => (value) => {
                    if (settled) return;
                    settled = true;
                    window.clearTimeout(timer);
                    callback(value);
                };
                const timer = window.setTimeout(() => {
                    if (settled) return;
                    settled = true;
                    reject(new Error(`ROS2 服务 ${name} 调用超时（${Math.round(deadline / 1000)} 秒）`));
                }, deadline);
                try {
                    this.service(name, serviceType).callService(
                        new ROSLIB.ServiceRequest(request),
                        finish(resolve), finish(reject));
                } catch (error) {
                    finish(reject)(error);
                }
            });
        }
    }

    function quaternionFromYaw(yaw) {
        const half = yaw / 2;
        return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
    }

    function yawFromQuaternion(q) {
        return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }

    function initialPoseMessage(x, y, yaw, odometryStamp) {
        const q = quaternionFromYaw(yaw);
        const stamp = odometryStamp
            ? { sec: odometryStamp.sec, nanosec: odometryStamp.nanosec }
            : { sec: 0, nanosec: 0 };
        return {
            header: { frame_id: 'map', stamp },
            pose: {
                pose: { position: { x, y, z: 0 }, orientation: q },
                covariance: [
                    0.04, 0, 0, 0, 0, 0,
                    0, 0.04, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.03
                ]
            }
        };
    }

    function goalPoseMessage(x, y, yaw) {
        return {
            header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
            pose: { position: { x, y, z: 0 }, orientation: quaternionFromYaw(yaw) }
        };
    }

    class KeyboardTeleop {
        constructor(options) {
            this.session = options.session;
            this.log = options.log || (() => {});
            this.acceptInput = options.acceptInput || (() => true);
            this.onSpeed = options.onSpeed || (() => {});
            this.onReset = options.onReset || (() => {});
            this.onEmergencyStop = options.onEmergencyStop || (() => {});
            this.onNeutral = options.onNeutral || (() => {});
            this.cmdVelTopic = options.cmdVelTopic || '/cmd_vel';
            this.maxLinear = options.maxLinear || 0.35;
            this.maxAngular = options.maxAngular || 0.8;
            this.allowLateral = Boolean(options.allowLateral);
            this.speedScale = options.speedScale || 0.5;
            this.enabled = false;
            this.keys = { w: false, a: false, s: false, d: false, q: false, e: false };
            this.timer = null;
            this.bind();
        }

        bind() {
            document.addEventListener('keydown', (event) => {
                if (!this.acceptInput()) return;
                if (this.isTextInput(event.target)) return;
                const key = event.key.toLowerCase();
                if (Object.prototype.hasOwnProperty.call(this.keys, key)) {
                    event.preventDefault();
                    if (!this.enabled) {
                        if (!event.repeat) this.log('请先开启遥控模式。');
                        return;
                    }
                    this.keys[key] = true;
                    this.start();
                    return;
                }
                if (key === 'r') {
                    event.preventDefault();
                    this.onReset();
                } else if (key === 't' || key === 'x') {
                    event.preventDefault();
                    this.stopAndZero();
                    this.onEmergencyStop();
                } else if (key === '+' || key === '=') {
                    event.preventDefault();
                    this.setSpeed(this.speedScale + 0.05);
                } else if (key === '-') {
                    event.preventDefault();
                    this.setSpeed(this.speedScale - 0.05);
                }
            });
            document.addEventListener('keyup', (event) => {
                if (!this.acceptInput()) return;
                const key = event.key.toLowerCase();
                if (!Object.prototype.hasOwnProperty.call(this.keys, key)) return;
                event.preventDefault();
                this.keys[key] = false;
                if (!Object.values(this.keys).some(Boolean)) {
                    this.publishZero();
                    if (this.enabled && this.session.connected) this.onNeutral();
                }
            });
            window.addEventListener('blur', () => { if (this.acceptInput()) this.stopAndZero(); });
            document.addEventListener('visibilitychange', () => { if (document.hidden && this.acceptInput()) this.stopAndZero(); });
        }

        isTextInput(target) {
            if (!target) return false;
            return target.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName);
        }

        setEnabled(enabled) {
            this.enabled = Boolean(enabled);
            if (!this.enabled) this.stopAndZero();
        }

        setSpeed(value) {
            this.speedScale = Math.max(0.1, Math.min(1.0, Number(value)));
            this.onSpeed(this.speedScale);
        }

        start() {
            if (this.timer) return;
            this.timer = window.setInterval(() => this.publishMotion(), 50);
        }

        publishMotion() {
            if (!this.enabled || !this.session.connected) return;
            const scale = this.speedScale * 0.8;
            let x = 0, y = 0, z = 0;
            if (this.keys.w) x += this.maxLinear * scale;
            if (this.keys.s) x -= this.maxLinear * scale;
            if (this.allowLateral && this.keys.a) y += this.maxLinear * scale;
            if (this.allowLateral && this.keys.d) y -= this.maxLinear * scale;
            if (this.keys.q) z += this.maxAngular * scale;
            if (this.keys.e) z -= this.maxAngular * scale;
            this.session.publish(this.cmdVelTopic, TYPES.twist, {
                linear: { x, y, z: 0 }, angular: { x: 0, y: 0, z }
            });
        }

        publishZero() {
            if (this.session.connected) {
                this.session.publish(this.cmdVelTopic, TYPES.twist, {
                    linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }
                });
            }
        }

        stopAndZero() {
            Object.keys(this.keys).forEach((key) => { this.keys[key] = false; });
            if (this.timer) window.clearInterval(this.timer);
            this.timer = null;
            this.publishZero();
        }
    }

    class MapCanvas {
        constructor(canvas, container) {
            this.canvas = canvas;
            this.container = container;
            this.ctx = canvas.getContext('2d');
            this.offscreen = document.createElement('canvas');
            this.offCtx = this.offscreen.getContext('2d');
            this.mapInfo = null;
            this.mapData = null;
            this.mapSignature = '';
            this.width = 1;
            this.height = 1;
            this.pixelRatio = 1;
            this.baseZoom = 1;
            this.zoom = 1;
            this.offsetX = 0;
            this.offsetY = 0;
            this.minZoomRatio = 0.02;
            this.maxZoomRatio = 24;
            this.robotPose = null;
            this.scanOverlay = null;
            this.goalPose = null;
            this.globalPath = [];
            this.globalPathWarning = false;
            this.routePoints = [];
            this.activeRoutePointIndex = -1;
            this.routeExecutionState = 'idle';
            this.poseInteraction = { enabled: false, callback: null };
            this.editorInteraction = null;
            this.pan = null;
            this.gesture = null;
            this.dragPose = null;
            this.editRect = null;
            this.onMapChanged = null;
            this.resize();
            this.bindEvents();
            this.renderLoop();
        }

        resize() {
            const rect = this.container.getBoundingClientRect();
            this.width = Math.max(1, rect.width);
            this.height = Math.max(1, rect.height);
            this.pixelRatio = Math.max(1, window.devicePixelRatio || 1);
            this.canvas.width = Math.round(this.width * this.pixelRatio);
            this.canvas.height = Math.round(this.height * this.pixelRatio);
            this.ctx.setTransform(this.pixelRatio, 0, 0, this.pixelRatio, 0, 0);
            if (this.mapInfo) this.resetView();
        }

        bindEvents() {
            this.canvas.addEventListener('contextmenu', (event) => event.preventDefault());
            this.canvas.addEventListener('wheel', (event) => this.handleWheel(event), { passive: false });
            this.canvas.addEventListener('mousedown', (event) => this.handleMouseDown(event));
            window.addEventListener('mousemove', (event) => this.handleMouseMove(event));
            window.addEventListener('mouseup', (event) => this.handleMouseUp(event));
            window.addEventListener('resize', () => this.resize());
            if (typeof window.ResizeObserver === 'function') {
                this.resizeObserver = new window.ResizeObserver(() => this.resize());
                this.resizeObserver.observe(this.container);
            }
        }

        handleWheel(event) {
            event.preventDefault();
            if (!this.mapInfo) return;
            const point = this.eventPoint(event);
            const worldX = (point.x - this.offsetX) / this.zoom;
            const worldY = (point.y - this.offsetY) / this.zoom;
            const delta = event.deltaY > 0 ? -0.10 : 0.10;
            const ratio = Math.max(this.minZoomRatio, Math.min(this.maxZoomRatio, this.zoom / this.baseZoom + delta));
            this.zoom = ratio * this.baseZoom;
            this.offsetX = point.x - worldX * this.zoom;
            this.offsetY = point.y - worldY * this.zoom;
        }

        handleMouseDown(event) {
            if (!this.mapInfo) return;
            if (event.button === 2) {
                this.pan = { x: event.clientX, y: event.clientY, offsetX: this.offsetX, offsetY: this.offsetY };
                this.canvas.style.cursor = 'grabbing';
                return;
            }
            if (event.button !== 0) return;
            const point = this.eventPoint(event);
            const mapPoint = this.canvasToMap(point.x, point.y);
            if (!mapPoint) return;
            if (this.editorInteraction && this.editorInteraction.enabled) {
                this.beginEdit(mapPoint);
            } else if (this.poseInteraction.enabled) {
                this.dragPose = { start: mapPoint, startScreen: point, endScreen: point };
            }
        }

        handleMouseMove(event) {
            if (this.pan) {
                this.offsetX = this.pan.offsetX + event.clientX - this.pan.x;
                this.offsetY = this.pan.offsetY + event.clientY - this.pan.y;
                return;
            }
            if (!this.gesture && !this.dragPose) return;
            const point = this.eventPoint(event);
            if (this.dragPose) {
                this.dragPose.endScreen = point;
                return;
            }
            const mapPoint = this.canvasToMap(point.x, point.y, true);
            if (mapPoint) this.updateEdit(mapPoint);
        }

        handleMouseUp(event) {
            if (event.button === 2 && this.pan) {
                this.pan = null;
                this.canvas.style.cursor = 'crosshair';
                return;
            }
            if (event.button !== 0) return;
            if (this.dragPose) {
                const drag = this.dragPose;
                this.dragPose = null;
                const dx = drag.endScreen.x - drag.startScreen.x;
                const dy = drag.endScreen.y - drag.startScreen.y;
                // Screen Y grows downward, whereas map Y grows upward.
                const yaw = Math.hypot(dx, dy) > 5 ? Math.atan2(-dy, dx) : 0;
                if (this.poseInteraction.callback) this.poseInteraction.callback(drag.start.x, drag.start.y, yaw);
                return;
            }
            if (this.gesture) this.commitEdit();
        }

        beginEdit(mapPoint) {
            const cfg = this.editorInteraction;
            this.gesture = { start: mapPoint, last: mapPoint, cells: new Map() };
            if (cfg.shape === 'rectangle') {
                this.editRect = { start: mapPoint, end: mapPoint };
            } else {
                this.addBrushCells(mapPoint, this.gesture.cells);
                this.previewCells(this.gesture.cells);
            }
        }

        updateEdit(mapPoint) {
            const cfg = this.editorInteraction;
            if (cfg.shape === 'rectangle') {
                this.gesture.last = mapPoint;
                this.editRect = { start: this.gesture.start, end: mapPoint };
                return;
            }
            this.addBrushLine(this.gesture.last, mapPoint, this.gesture.cells);
            this.gesture.last = mapPoint;
            this.previewCells(this.gesture.cells);
        }

        commitEdit() {
            const cfg = this.editorInteraction;
            const gesture = this.gesture;
            this.gesture = null;
            if (cfg.shape === 'rectangle') {
                this.addRectangleCells(gesture.start, gesture.last, gesture.cells);
                this.editRect = null;
                this.previewCells(gesture.cells);
            }
            const cells = [...gesture.cells.values()];
            if (cells.length && cfg.onCommit) cfg.onCommit(cells);
        }

        previewCells(cells) {
            const cfg = this.editorInteraction;
            if (!cfg || !cfg.onPreview) return;
            cfg.onPreview([...cells.values()]);
        }

        addBrushLine(from, to, target) {
            const distance = Math.hypot(to.x - from.x, to.y - from.y);
            const interval = Math.max(this.mapInfo.resolution * 0.45, 0.001);
            const steps = Math.max(1, Math.ceil(distance / interval));
            for (let i = 0; i <= steps; i += 1) {
                this.addBrushCells({
                    x: from.x + (to.x - from.x) * i / steps,
                    y: from.y + (to.y - from.y) * i / steps
                }, target);
            }
        }

        addBrushCells(point, target) {
            const center = this.mapToGrid(point.x, point.y);
            if (!center) return;
            const radius = Math.max(0, Math.floor(this.editorInteraction.brushRadius || 0));
            for (let dy = -radius; dy <= radius; dy += 1) {
                for (let dx = -radius; dx <= radius; dx += 1) {
                    if (dx * dx + dy * dy > radius * radius) continue;
                    this.addCell(center.x + dx, center.y + dy, target);
                }
            }
        }

        addRectangleCells(start, end, target) {
            const a = this.mapToGrid(start.x, start.y);
            const b = this.mapToGrid(end.x, end.y);
            if (!a || !b) return;
            const minX = Math.min(a.x, b.x), maxX = Math.max(a.x, b.x);
            const minY = Math.min(a.y, b.y), maxY = Math.max(a.y, b.y);
            for (let y = minY; y <= maxY; y += 1) {
                for (let x = minX; x <= maxX; x += 1) this.addCell(x, y, target);
            }
        }

        addCell(x, y, target) {
            const info = this.mapInfo;
            if (x < 0 || y < 0 || x >= info.width || y >= info.height) return;
            const pgmRow = info.height - 1 - y;
            const serviceIndex = pgmRow * info.width + x;
            if (!target.has(serviceIndex)) target.set(serviceIndex, { serviceIndex, dataIndex: y * info.width + x });
        }

        eventPoint(event) {
            const rect = this.canvas.getBoundingClientRect();
            return { x: event.clientX - rect.left, y: event.clientY - rect.top };
        }

        canvasToMap(screenX, screenY, clamp) {
            if (!this.mapInfo) return null;
            const info = this.mapInfo;
            let x = info.origin.position.x + (screenX - this.offsetX) / this.zoom;
            let y = info.origin.position.y + info.height * info.resolution - (screenY - this.offsetY) / this.zoom;
            const maxX = info.origin.position.x + info.width * info.resolution;
            const maxY = info.origin.position.y + info.height * info.resolution;
            if (clamp) {
                x = Math.max(info.origin.position.x, Math.min(maxX - 1e-9, x));
                y = Math.max(info.origin.position.y, Math.min(maxY - 1e-9, y));
            }
            if (x < info.origin.position.x || y < info.origin.position.y || x >= maxX || y >= maxY) return null;
            return { x, y };
        }

        mapToGrid(x, y) {
            if (!this.mapInfo) return null;
            const info = this.mapInfo;
            const gx = Math.floor((x - info.origin.position.x) / info.resolution);
            const gy = Math.floor((y - info.origin.position.y) / info.resolution);
            if (gx < 0 || gy < 0 || gx >= info.width || gy >= info.height) return null;
            return { x: gx, y: gy };
        }

        mapToCanvas(x, y) {
            if (!this.mapInfo) return null;
            const info = this.mapInfo;
            return {
                x: this.offsetX + (x - info.origin.position.x) * this.zoom,
                y: this.offsetY + (info.height * info.resolution - (y - info.origin.position.y)) * this.zoom
            };
        }

        setMap(info, data, options) {
            if (!info || !data) return;
            const signature = `${info.width}|${info.height}|${info.resolution}|${info.origin && info.origin.position ? info.origin.position.x : 0}|${info.origin && info.origin.position ? info.origin.position.y : 0}`;
            const reset = !this.mapInfo || this.mapSignature !== signature || (options && options.reset);
            this.mapInfo = info;
            // Occupancy grids arrive as very large JSON arrays. A compact
            // typed array avoids retaining hundreds of thousands of boxed
            // JavaScript numbers after every map update. Int16 also preserves
            // the 0..255 values used by the map editor.
            this.mapData = Int16Array.from(data);
            this.mapSignature = signature;
            this.rebuildRaster();
            if (reset) this.resetView();
            if (this.onMapChanged) this.onMapChanged(info, this.mapData);
        }

        applyOccupancyCells(cells, value) {
            if (!this.mapData) return;
            cells.forEach((cell) => { this.mapData[cell.dataIndex] = value; });
            this.rebuildRaster();
        }

        rebuildRaster() {
            if (!this.mapInfo || !this.mapData) return;
            const info = this.mapInfo;
            this.offscreen.width = info.width;
            this.offscreen.height = info.height;
            const image = this.offCtx.createImageData(info.width, info.height);
            for (let gridY = 0; gridY < info.height; gridY += 1) {
                for (let x = 0; x < info.width; x += 1) {
                    const value = Number(this.mapData[gridY * info.width + x]);
                    const pixel = ((info.height - 1 - gridY) * info.width + x) * 4;
                    let red = 76;
                    let green = 82;
                    let blue = 96;
                    if (Number.isFinite(value) && value >= 0) {
                        const shade = value === 0
                            ? 250
                            : (value >= 100
                                ? 26
                                : Math.max(44, Math.round(250 - value * 1.8)));
                        red = shade;
                        green = shade;
                        blue = shade;
                    }
                    image.data[pixel] = red;
                    image.data[pixel + 1] = green;
                    image.data[pixel + 2] = blue;
                    image.data[pixel + 3] = 255;
                }
            }
            this.offCtx.putImageData(image, 0, 0);
        }

        resetView() {
            if (!this.mapInfo) return;
            const mapWidth = this.mapInfo.width * this.mapInfo.resolution;
            const mapHeight = this.mapInfo.height * this.mapInfo.resolution;
            this.baseZoom = Math.min(this.width * 0.84 / mapWidth, this.height * 0.84 / mapHeight);
            this.baseZoom = Number.isFinite(this.baseZoom) && this.baseZoom > 0 ? this.baseZoom : 1;
            this.zoom = this.baseZoom;
            this.offsetX = (this.width - mapWidth * this.zoom) / 2;
            this.offsetY = (this.height - mapHeight * this.zoom) / 2;
        }

        zoomBy(amount) {
            if (!this.mapInfo) return;
            const point = { x: this.width / 2, y: this.height / 2 };
            const localX = (point.x - this.offsetX) / this.zoom;
            const localY = (point.y - this.offsetY) / this.zoom;
            const ratio = Math.max(this.minZoomRatio, Math.min(this.maxZoomRatio, this.zoom / this.baseZoom + amount));
            this.zoom = ratio * this.baseZoom;
            this.offsetX = point.x - localX * this.zoom;
            this.offsetY = point.y - localY * this.zoom;
        }

        setRobotPose(x, y, yaw) { this.robotPose = { x, y, yaw }; }
        clearRobotPose() { this.robotPose = null; }
        setGoalPose(x, y, yaw) { this.goalPose = { x, y, yaw }; }
        clearGoalPose() { this.goalPose = null; }
        setGlobalPath(points) {
            const options = arguments[1] || {};
            this.globalPath = (points || []).map((point) => ({
                x: Number(point && point.x),
                y: Number(point && point.y),
                z: Number(point && point.z)
            })).filter((point) => (
                Number.isFinite(point.x)
                && Number.isFinite(point.y)
                && Number.isFinite(point.z)
            ));
            this.globalPathWarning = Boolean(options.warning);
        }
        clearGlobalPath() {
            this.globalPath = [];
            this.globalPathWarning = false;
        }
        setRoutePoints(points, activeIndex, executionState) {
            this.routePoints = (points || []).map((point) => ({
                x: Number(point.x),
                y: Number(point.y),
                yaw: Number(point.yaw),
                name: String(point.name || ''),
                id: Number(point.id) || 0,
                sequence: Number(point.sequence) || 0,
                inspection: Boolean(point.inspection),
                inspectionStatus: String(point.inspectionStatus || ''),
                alerted: Boolean(point.alerted)
            })).filter((point) => (
                Number.isFinite(point.x)
                && Number.isFinite(point.y)
                && Number.isFinite(point.yaw)
            ));
            this.activeRoutePointIndex = Number.isInteger(activeIndex)
                ? activeIndex : -1;
            this.routeExecutionState = executionState || 'idle';
        }
        clearRoutePoints() {
            this.routePoints = [];
            this.activeRoutePointIndex = -1;
            this.routeExecutionState = 'idle';
        }

        setPoseInteraction(enabled, callback) {
            this.poseInteraction = { enabled: Boolean(enabled), callback };
            if (enabled) this.editorInteraction = null;
        }

        setEditorInteraction(config) {
            this.editorInteraction = config ? {
                enabled: true,
                shape: config.shape || 'brush',
                brushRadius: config.brushRadius || 0,
                onPreview: config.onPreview,
                onCommit: config.onCommit
            } : null;
            if (config) this.poseInteraction.enabled = false;
        }

        renderLoop() {
            this.render();
            window.requestAnimationFrame(() => this.renderLoop());
        }

        render() {
            const ctx = this.ctx;
            ctx.clearRect(0, 0, this.width, this.height);
            ctx.fillStyle = '#0d1428';
            ctx.fillRect(0, 0, this.width, this.height);
            if (!this.mapInfo || !this.mapData) {
                ctx.fillStyle = '#a9badb';
                ctx.font = '16px sans-serif';
                ctx.textAlign = 'center';
                ctx.fillText('等待地图数据…', this.width / 2, this.height / 2);
                return;
            }
            const info = this.mapInfo;
            ctx.imageSmoothingEnabled = false;
            ctx.drawImage(this.offscreen, this.offsetX, this.offsetY,
                info.width * info.resolution * this.zoom, info.height * info.resolution * this.zoom);
            this.drawScanOverlay(ctx);
            this.drawGlobalPath(ctx);
            this.drawPoseDrag(ctx);
            this.drawEditSelection(ctx);
            this.drawEditRectangle(ctx);
            this.drawRoute(ctx);
            this.drawRobot(ctx);
            this.drawGoal(ctx);
            ctx.fillStyle = 'rgba(6, 10, 21, .68)';
            ctx.fillRect(8, 8, 184, 25);
            ctx.fillStyle = '#d9e6fb';
            ctx.font = '12px sans-serif';
            ctx.textAlign = 'left';
            ctx.fillText(`缩放 ${Math.round(this.zoom / this.baseZoom * 100)}%  ·  ${info.width} × ${info.height}`, 15, 25);
        }

        drawScanOverlay(ctx) {
            const scan = this.scanOverlay;
            if (!scan || Date.now() - scan.receivedAt > 1500) return;
            let pose = this.robotPose;
            if (scan.followDrag && this.dragPose) {
                const drag = this.dragPose;
                const dx = drag.endScreen.x - drag.startScreen.x;
                const dy = drag.endScreen.y - drag.startScreen.y;
                pose = { x: drag.start.x, y: drag.start.y,
                    yaw: Math.hypot(dx, dy) > 5 ? Math.atan2(-dy, dx) : 0 };
            }
            if (!pose) return;
            const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
            ctx.save();
            ctx.fillStyle = '#ff694f';
            for (const [x, y] of scan.points) {
                const p = this.mapToCanvas(pose.x + c*x - s*y, pose.y + s*x + c*y);
                if (p) ctx.fillRect(p.x - 1.5, p.y - 1.5, 3, 3);
            }
            ctx.restore();
        }

        drawPoseDrag(ctx) {
            if (!this.dragPose) return;
            const drag = this.dragPose;
            ctx.save();
            ctx.strokeStyle = '#ffca62';
            ctx.lineWidth = 2;
            ctx.setLineDash([7, 5]);
            ctx.beginPath();
            ctx.moveTo(drag.startScreen.x, drag.startScreen.y);
            ctx.lineTo(drag.endScreen.x, drag.endScreen.y);
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.fillStyle = '#ffca62';
            ctx.beginPath();
            ctx.arc(drag.startScreen.x, drag.startScreen.y, 5, 0, Math.PI * 2);
            ctx.fill();
            ctx.restore();
        }

        drawEditSelection(ctx) {
            if (!this.selectionCells?.length || !this.mapInfo) return;
            const info=this.mapInfo, r=info.resolution, size=r*this.zoom;
            ctx.save(); ctx.fillStyle='rgba(255, 202, 98, .48)';
            for (const cell of this.selectionCells) {
                const x=cell.dataIndex%info.width, y=Math.floor(cell.dataIndex/info.width);
                const p=this.mapToCanvas(info.origin.position.x+x*r,info.origin.position.y+y*r);
                if (p) ctx.fillRect(p.x,p.y-size,size,size);
            }
            ctx.restore();
        }

        drawEditRectangle(ctx) {
            const rectangle = this.editRect || this.selectionRect;
            if (!rectangle) return;
            const a = this.mapToCanvas(rectangle.start.x, rectangle.start.y);
            const b = this.mapToCanvas(rectangle.end.x, rectangle.end.y);
            if (!a || !b) return;
            ctx.save();
            ctx.setLineDash([7, 4]);
            ctx.lineWidth = 2;
            ctx.strokeStyle = '#ffca62';
            ctx.fillStyle = 'rgba(255, 202, 98, .12)';
            const x = Math.min(a.x, b.x), y = Math.min(a.y, b.y);
            const w = Math.abs(a.x - b.x), h = Math.abs(a.y - b.y);
            ctx.fillRect(x, y, w, h);
            ctx.strokeRect(x, y, w, h);
            ctx.restore();
        }

        drawRobot(ctx) {
            if (!this.robotPose) return;
            const p = this.mapToCanvas(this.robotPose.x, this.robotPose.y);
            if (!p) return;
            const size = Math.max(7, Math.min(15, 8 * Math.sqrt(this.zoom / this.baseZoom)));
            ctx.save();
            ctx.translate(p.x, p.y);
            ctx.rotate(-this.robotPose.yaw);
            ctx.fillStyle = '#e75c73';
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 1.4;
            ctx.beginPath();
            ctx.moveTo(size * 1.25, 0);
            ctx.lineTo(-size * .7, -size * .65);
            ctx.lineTo(-size * .7, size * .65);
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
            ctx.restore();
        }

        drawGlobalPath(ctx) {
            if (this.globalPath.length < 2) return;
            const color = this.globalPathWarning ? '#ffb454' : '#45e39b';
            const drawSegments = (strokeStyle, lineWidth) => {
                let segmentOpen = false;
                ctx.strokeStyle = strokeStyle;
                ctx.lineWidth = lineWidth;
                ctx.beginPath();
                this.globalPath.forEach((point) => {
                    // Do not bridge across an invalid/out-of-map section. A
                    // straight chord there would falsely look like a valid
                    // planner shortcut through unknown space.
                    if (!this.mapToGrid(point.x, point.y)) {
                        if (segmentOpen) ctx.stroke();
                        ctx.beginPath();
                        segmentOpen = false;
                        return;
                    }
                    const screen = this.mapToCanvas(point.x, point.y);
                    if (!screen) return;
                    if (!segmentOpen) {
                        ctx.moveTo(screen.x, screen.y);
                        segmentOpen = true;
                    } else {
                        ctx.lineTo(screen.x, screen.y);
                    }
                });
                if (segmentOpen) ctx.stroke();
            };

            ctx.save();
            ctx.lineCap = 'round';
            ctx.lineJoin = 'round';
            ctx.setLineDash([]);
            // A dark underlay keeps the actual Nav2 plan readable over both
            // white free-space cells and dark occupied/unknown cells.
            ctx.strokeStyle = 'rgba(4, 14, 22, .86)';
            ctx.lineWidth = 6.4;
            drawSegments(ctx.strokeStyle, ctx.lineWidth);
            ctx.strokeStyle = color;
            ctx.lineWidth = 3.4;
            drawSegments(ctx.strokeStyle, ctx.lineWidth);
            ctx.restore();
        }

        drawRoute(ctx) {
            if (!this.routePoints.length) return;
            const screenPoints = this.routePoints.map((point) => ({
                point,
                screen: this.mapToCanvas(point.x, point.y)
            })).filter((entry) => entry.screen);
            if (!screenPoints.length) return;

            ctx.save();
            ctx.lineWidth = 2.4;
            ctx.strokeStyle = 'rgba(84, 205, 255, .76)';
            ctx.setLineDash([9, 5]);
            ctx.beginPath();
            screenPoints.forEach((entry, index) => {
                if (index === 0) ctx.moveTo(entry.screen.x, entry.screen.y);
                else ctx.lineTo(entry.screen.x, entry.screen.y);
            });
            ctx.stroke();
            ctx.setLineDash([]);

            screenPoints.forEach((entry, screenIndex) => {
                const routeIndex = this.routePoints.indexOf(entry.point);
                const active = routeIndex === this.activeRoutePointIndex;
                const radius = active ? 15 : 13;
                const status = entry.point.inspectionStatus;
                const color = active
                    ? '#ffca62'
                    : (entry.point.alerted
                        ? '#ff9f43'
                        : (status === 'completed'
                            ? '#45e39b'
                            : (status === 'failed' || status === 'skipped'
                                ? '#e85b73'
                                : (entry.point.inspection ? '#b98cff' : '#4dc5ff'))));
                ctx.save();
                ctx.translate(entry.screen.x, entry.screen.y);
                ctx.rotate(-entry.point.yaw);
                ctx.strokeStyle = color;
                ctx.fillStyle = color;
                ctx.lineWidth = active ? 3.2 : 2.2;
                ctx.beginPath();
                ctx.moveTo(radius + 13, 0);
                ctx.lineTo(radius + 3, -5);
                ctx.lineTo(radius + 3, 5);
                ctx.closePath();
                ctx.fill();
                ctx.beginPath();
                ctx.moveTo(radius - 1, 0);
                ctx.lineTo(radius + 13, 0);
                ctx.stroke();
                ctx.rotate(entry.point.yaw);
                ctx.fillStyle = active ? 'rgba(10, 25, 34, .96)' : 'rgba(12, 31, 50, .94)';
                ctx.beginPath();
                if (entry.point.inspection) {
                    ctx.moveTo(0, -radius);
                    ctx.lineTo(radius, 0);
                    ctx.lineTo(0, radius);
                    ctx.lineTo(-radius, 0);
                    ctx.closePath();
                } else {
                    ctx.arc(0, 0, radius, 0, Math.PI * 2);
                }
                ctx.fill();
                ctx.strokeStyle = color;
                ctx.stroke();
                ctx.fillStyle = '#ffffff';
                ctx.font = `700 ${routeIndex + 1 >= 100 ? 10 : 12}px sans-serif`;
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(String(routeIndex + 1), 0, .5);
                ctx.restore();
            });
            ctx.restore();
        }

        drawGoal(ctx) {
            if (!this.goalPose) return;
            const p = this.mapToCanvas(this.goalPose.x, this.goalPose.y);
            if (!p) return;
            ctx.save();
            ctx.strokeStyle = '#4dc5ff';
            ctx.fillStyle = '#4dc5ff';
            ctx.lineWidth = 2.5;
            ctx.beginPath();
            ctx.arc(p.x, p.y, 11, 0, Math.PI * 2);
            ctx.stroke();
            ctx.beginPath();
            ctx.arc(p.x, p.y, 3.5, 0, Math.PI * 2);
            ctx.fill();
            ctx.restore();
        }
    }

    function setConnectionStatus(element, connected, text) {
        if (!element) return;
        element.textContent = connected ? `● ${text}` : `○ ${text}`;
        element.className = `page-status ${connected ? 'connected' : 'error'}`;
    }

    function setBatteryDisconnected(element) {
        if (!element) return;
        const lastBattery = element.dataset.lastBattery;
        element.textContent = lastBattery
            ? `电量 ${lastBattery}%（状态失联）`
            : '电量 --（状态失联）';
        element.className = 'battery-status stale';
        element.title = 'ROS2 或机械狗状态数据已中断；保留的是最后一次有效电量。';
    }

    function updateBatteryStatus(element, status) {
        if (!element || !status) return;
        const hasBattery = status.battery !== null
            && status.battery !== undefined
            && status.battery !== '';
        const value = hasBattery ? Number(status.battery) : NaN;
        if (Number.isFinite(value)) {
            const rounded = String(Math.round(Math.max(0, Math.min(100, value))));
            element.dataset.lastBattery = rounded;
        }
        const lastBattery = element.dataset.lastBattery;
        const fresh = status.connected === true && status.stale !== true;
        element.textContent = lastBattery
            ? `电量 ${lastBattery}%${fresh ? '' : '（状态失联）'}`
            : `电量 --${fresh ? '' : '（状态失联）'}`;
        const low = lastBattery && Number(lastBattery) <= 20;
        element.className = `battery-status${fresh ? '' : ' stale'}${low ? ' low' : ''}`;
        element.title = fresh
            ? '机械狗实时电量'
            : '机械狗状态数据已中断；保留的是最后一次有效电量。';
    }

    function subscribeDogStatus(session, batteryElement, onStatus) {
        return session.subscribe('/dog_status', TYPES.string, (message) => {
            let status;
            try {
                status = JSON.parse(message.data);
            } catch (_error) {
                return;
            }
            updateBatteryStatus(batteryElement, status);
            if (batteryElement && status.body_id) {
                batteryElement.title = '当前本体：' + ({xingji: '星玑', gangben_zsl1: '钢镚 ZSL-1 点足', xuanji: '璇玑轮式四足'}[status.body_id] || '未知本体');
            }
            if (onStatus) onStatus(status);
        });
    }

    function bindZoomButtons(mapCanvas) {
        const inButton = document.getElementById('btn-zoom-in');
        const outButton = document.getElementById('btn-zoom-out');
        const resetButton = document.getElementById('btn-reset-view');
        if (inButton) inButton.addEventListener('click', () => mapCanvas.zoomBy(.15));
        if (outButton) outButton.addEventListener('click', () => mapCanvas.zoomBy(-.15));
        if (resetButton) resetButton.addEventListener('click', () => mapCanvas.resetView());
    }

    function escapeText(value) {
        const span = document.createElement('span');
        span.textContent = String(value ?? '');
        return span.innerHTML;
    }

    window.RobotUi = Object.freeze({
        TYPES,
        getRosbridgeUrl,
        previewProfile, previewUrl, previewAllowed, previewControls,
        temperatureText,
        cameraDisplay: () => cameraDisplay,
        createLogger,
        RosSession,
        MapSnapshotRecovery,
        subscribePreviewHealth,
        KeyboardTeleop,
        MapCanvas,
        quaternionFromYaw,
        yawFromQuaternion,
        initialPoseMessage,
        goalPoseMessage,
        setConnectionStatus,
        setBatteryDisconnected,
        updateBatteryStatus,
        subscribeDogStatus,
        bindZoomButtons,
        escapeText
    });
}());
