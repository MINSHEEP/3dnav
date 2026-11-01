/* Lightweight WebGL viewer for ROS2 sensor_msgs/PointCloud2 messages. */
(function (global) {
    'use strict';

    const FLOAT32 = 7;
    const FLOAT64 = 8;

    function decodeBytes(data) {
        if (data instanceof Uint8Array) return data;
        if (Array.isArray(data)) return Uint8Array.from(data);
        if (data && data.buffer instanceof ArrayBuffer) {
            return new Uint8Array(data.buffer, data.byteOffset || 0, data.byteLength);
        }
        if (typeof data !== 'string') throw new Error('PointCloud2.data 不是有效字节数组');
        const binary = global.atob(data);
        const bytes = new Uint8Array(binary.length);
        for (let offset = 0; offset < binary.length; offset += 1) {
            bytes[offset] = binary.charCodeAt(offset);
        }
        return bytes;
    }

    function coordinateReader(view, field, littleEndian) {
        if (!field) throw new Error('PointCloud2 缺少 x/y/z 字段');
        const offset = Number(field.offset);
        if (Number(field.datatype) === FLOAT32) {
            return (base) => view.getFloat32(base + offset, littleEndian);
        }
        if (Number(field.datatype) === FLOAT64) {
            return (base) => view.getFloat64(base + offset, littleEndian);
        }
        throw new Error(`不支持的坐标字段类型：${field.datatype}`);
    }

    function compileShader(gl, type, source) {
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
            throw new Error(gl.getShaderInfoLog(shader) || 'WebGL shader 编译失败');
        }
        return shader;
    }

    function createProgram(gl) {
        const vertexSource = `
            attribute vec3 aPosition;
            attribute vec3 aColor;
            uniform float uYaw;
            uniform float uPitch;
            uniform float uZoom;
            uniform float uAspect;
            uniform vec2 uPan;
            varying vec3 vColor;
            void main() {
                vec3 p = aPosition + vec3(uPan, 0.0);
                float cy = cos(uYaw), sy = sin(uYaw);
                float cp = cos(uPitch), sp = sin(uPitch);
                vec3 yawed = vec3(cy * p.x - sy * p.y, sy * p.x + cy * p.y, p.z);
                vec3 viewed = vec3(yawed.x, cp * yawed.y - sp * yawed.z, sp * yawed.y + cp * yawed.z);
                gl_Position = vec4(viewed.x * uZoom / uAspect, viewed.y * uZoom, viewed.z * 0.08, 1.0);
                gl_PointSize = 2.2;
                vColor = aColor;
            }
        `;
        const fragmentSource = `
            precision mediump float;
            varying vec3 vColor;
            void main() {
                vec2 center = gl_PointCoord - vec2(0.5);
                if (dot(center, center) > 0.25) discard;
                gl_FragColor = vec4(vColor, 0.94);
            }
        `;
        const program = gl.createProgram();
        gl.attachShader(program, compileShader(gl, gl.VERTEX_SHADER, vertexSource));
        gl.attachShader(program, compileShader(gl, gl.FRAGMENT_SHADER, fragmentSource));
        gl.linkProgram(program);
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            throw new Error(gl.getProgramInfoLog(program) || 'WebGL program 链接失败');
        }
        return program;
    }

    function createMarkerProgram(gl) {
        const vertexSource = `
            attribute vec3 aPosition;
            attribute vec3 aColor;
            uniform float uYaw;
            uniform float uPitch;
            uniform float uZoom;
            uniform float uAspect;
            uniform vec2 uPan;
            varying vec3 vColor;
            void main() {
                vec3 p = aPosition + vec3(uPan, 0.0);
                float cy = cos(uYaw), sy = sin(uYaw);
                float cp = cos(uPitch), sp = sin(uPitch);
                vec3 yawed = vec3(cy * p.x - sy * p.y, sy * p.x + cy * p.y, p.z);
                vec3 viewed = vec3(yawed.x, cp * yawed.y - sp * yawed.z, sp * yawed.y + cp * yawed.z);
                gl_Position = vec4(viewed.x * uZoom / uAspect, viewed.y * uZoom, viewed.z * 0.08, 1.0);
                vColor = aColor;
            }
        `;
        const fragmentSource = `
            precision mediump float;
            varying vec3 vColor;
            void main() {
                gl_FragColor = vec4(vColor, 1.0);
            }
        `;
        const program = gl.createProgram();
        gl.attachShader(program, compileShader(gl, gl.VERTEX_SHADER, vertexSource));
        gl.attachShader(program, compileShader(gl, gl.FRAGMENT_SHADER, fragmentSource));
        gl.linkProgram(program);
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            throw new Error(gl.getProgramInfoLog(program) || 'WebGL marker program link failed');
        }
        return program;
    }

    class PointCloudViewer {
        constructor(canvas, container, options = {}) {
            this.canvas = canvas;
            this.container = container || canvas.parentElement;
            this.maxPoints = Math.max(1000, Number(options.maxPoints) || 120000);
            this.fullResolution = Boolean(options.fullResolution);
            this.pointColor = Array.isArray(options.pointColor) && options.pointColor.length === 3
                ? options.pointColor.map(value => Math.max(0, Math.min(1, Number(value) || 0))) : null;
            this.markerScale = Math.max(0.1, Math.min(2, Number(options.markerScale) || 1));
            this.gl = canvas.getContext('webgl', {
                alpha: false, antialias: false, depth: true,
                powerPreference: 'high-performance'
            });
            if (!this.gl) throw new Error('浏览器不支持 WebGL，无法显示三维点云');
            this.gl.enable(this.gl.DEPTH_TEST);
            this.gl.depthFunc(this.gl.LEQUAL);
            this.program = createProgram(this.gl);
            this.markerProgram = createMarkerProgram(this.gl);
            this.buffer = this.gl.createBuffer();
            this.markerBuffer = this.gl.createBuffer();
            this.pathBuffer = this.gl.createBuffer();
            this.draftBuffer = this.gl.createBuffer();
            this.previewPathBuffer = this.gl.createBuffer();
            this.routePathBuffer = this.gl.createBuffer();
            this.pointCount = 0;
            this.markerVertexCount = 0;
            this.pathVertexCount = 0;
            this.draftVertexCount = 0;
            this.previewPathVertexCount = 0;
            this.routePathVertexCount = 0;
            this.robotPose = null;
            this.draftPose = null;
            this.globalPath = [];
            this.previewPath = [];
            this.routePoints = [];
            this.routeActiveIndex = -1;
            this.checkpointPoints = [];
            this.globalPathWarning = false;
            this.cloudTransform = null;
            this.yaw = -0.72;
            this.pitch = -0.88;
            this.zoom = 0.82;
            this.panX = 0;
            this.panY = 0;
            this.drag = null;
            this.groundPoseInteraction = { enabled: false, callback: null };
            this.pixelRatio = 1;
            // Numbered markers remain readable over dense point returns. The
            // layer does not take pointer input; all picking uses map-frame
            // coordinates and the same projection as the WebGL canvas.
            this.markerLayer = null;
            if (global.document && typeof this.container.appendChild === 'function') {
                this.markerLayer = global.document.createElement('div');
                Object.assign(this.markerLayer.style, {
                    position: 'absolute', inset: '0', zIndex: '1',
                    pointerEvents: 'none', overflow: 'hidden'
                });
                this.container.appendChild(this.markerLayer);
            }
            this.bindEvents();
            this.resize();
            this.draw();
        }

        bindEvents() {
            this.canvas.addEventListener('contextmenu', (event) => event.preventDefault());
            this.canvas.addEventListener('mousedown', (event) => {
                if (event.button !== 0 && event.button !== 2) return;
                if (event.button === 0 && this.groundPoseInteraction.enabled
                        && this._isTopView()) {
                    const start = this._screenToGroundMapPoint(event.clientX, event.clientY);
                    if (!start) return;
                    this.drag = {
                        button: 0, groundPose: true, start,
                        x: event.clientX, y: event.clientY
                    };
                    this.canvas.classList.add('dragging');
                    return;
                }
                this.drag = {
                    button: event.button, x: event.clientX, y: event.clientY,
                    yaw: this.yaw, pitch: this.pitch,
                    panX: this.panX, panY: this.panY
                };
                this.canvas.classList.add('dragging');
            });
            global.addEventListener('mousemove', (event) => {
                if (!this.drag) return;
                if (this.drag.groundPose) return;
                const dx = event.clientX - this.drag.x;
                const dy = event.clientY - this.drag.y;
                if (this.drag.button === 0) {
                    this.yaw = this.drag.yaw + dx * 0.008;
                    this.pitch = Math.max(-1.5, Math.min(1.5, this.drag.pitch + dy * 0.008));
                } else {
                    this.panX = this.drag.panX + dx / Math.max(180, this.width) * 2 / this.zoom;
                    this.panY = this.drag.panY - dy / Math.max(180, this.height) * 2 / this.zoom;
                }
                this.draw();
            });
            global.addEventListener('mouseup', (event) => {
                const drag = this.drag;
                this.drag = null;
                this.canvas.classList.remove('dragging');
                if (!drag || !drag.groundPose || event.button !== 0
                        || !this.groundPoseInteraction.enabled || !this._isTopView()) return;
                const end = this._screenToGroundMapPoint(event.clientX, event.clientY);
                if (!end) return;
                const distance = Math.hypot(event.clientX - drag.x, event.clientY - drag.y);
                const yaw = distance > 5
                    ? Math.atan2(end.y - drag.start.y, end.x - drag.start.x)
                    : 0;
                if (this.groundPoseInteraction.callback) {
                    this.groundPoseInteraction.callback(drag.start.x, drag.start.y, yaw);
                }
            });
            this.canvas.addEventListener('wheel', (event) => {
                event.preventDefault();
                const factor = event.deltaY > 0 ? 0.88 : 1.14;
                this.zoom = Math.max(0.18, Math.min(8, this.zoom * factor));
                this.draw();
            }, { passive: false });
            global.addEventListener('resize', () => this.resize());
            if (typeof global.ResizeObserver === 'function') {
                this.resizeObserver = new global.ResizeObserver(() => this.resize());
                this.resizeObserver.observe(this.container);
            }
        }

        resize() {
            if (this.canvas.hidden) return;
            const rect = this.container.getBoundingClientRect();
            this.width = Math.max(1, Math.floor(rect.width));
            this.height = Math.max(1, Math.floor(rect.height));
            this.pixelRatio = Math.min(2, global.devicePixelRatio || 1);
            const width = Math.round(this.width * this.pixelRatio);
            const height = Math.round(this.height * this.pixelRatio);
            if (this.canvas.width !== width || this.canvas.height !== height) {
                this.canvas.width = width;
                this.canvas.height = height;
            }
            this.draw();
        }

        decodePointCloud(message) {
            const width = Math.max(0, Number(message.width) || 0);
            const height = Math.max(1, Number(message.height) || 1);
            const sourcePoints = width * height;
            const pointStep = Number(message.point_step);
            const rowStep = Number(message.row_step) || width * pointStep;
            if (!sourcePoints || !Number.isFinite(pointStep) || pointStep < 12) {
                throw new Error('PointCloud2 尺寸或 point_step 无效');
            }
            const bytes = decodeBytes(message.data);
            if (!Number.isFinite(rowStep) || rowStep < width * pointStep
                    || bytes.byteLength < rowStep * height) {
                throw new Error('PointCloud2 data / row_step is incomplete');
            }
            const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
            const fields = new Map((message.fields || []).map((field) => [field.name, field]));
            const littleEndian = !Boolean(message.is_bigendian);
            const readX = coordinateReader(view, fields.get('x'), littleEndian);
            const readY = coordinateReader(view, fields.get('y'), littleEndian);
            const readZ = coordinateReader(view, fields.get('z'), littleEndian);
            const stride = this.fullResolution ? 1 : Math.max(1, Math.ceil(sourcePoints / this.maxPoints));
            if (this.fullResolution && sourcePoints > 4000000) {
                throw new Error('完整地图超过400万点资源限额；未自动抽稀');
            }
            let renderedPoints = 0, examinedPoints = 0;
            let minX = Infinity, minY = Infinity, minZ = Infinity;
            let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
            for (let index = 0; index < sourcePoints; index += stride) {
                examinedPoints += 1;
                const row = Math.floor(index / width);
                const column = index % width;
                const base = row * rowStep + column * pointStep;
                const x = readX(base), y = readY(base), z = readZ(base);
                if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
                renderedPoints += 1;
                minX = Math.min(minX, x); maxX = Math.max(maxX, x);
                minY = Math.min(minY, y); maxY = Math.max(maxY, y);
                minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
            }
            if (!renderedPoints) throw new Error('点云没有有限坐标点');
            const centerX = (minX + maxX) / 2;
            const centerY = (minY + maxY) / 2;
            const centerZ = (minZ + maxZ) / 2;
            const radius = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 0.1) * 0.56;
            const zRange = Math.max(0.05, maxZ - minZ);
            if (![centerX, centerY, centerZ, radius, zRange].every(Number.isFinite)) {
                throw new Error('点云坐标范围不可绘制；未裁剪地图');
            }
            const vertices = new Float32Array(renderedPoints * 6);
            // Two passes avoid a large boxed JS array of all XYZ values.
            let point = 0;
            for (let index = 0; index < sourcePoints; index += stride) {
                const row = Math.floor(index / width), column = index % width;
                const base = row * rowStep + column * pointStep;
                const x = readX(base), y = readY(base), z = readZ(base);
                if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
                const target = point * 6;
                const heightRatio = Math.max(0, Math.min(1, (z - minZ) / zRange));
                vertices[target] = (x - centerX) / radius;
                vertices[target + 1] = (y - centerY) / radius;
                vertices[target + 2] = (z - centerZ) / radius;
                vertices[target + 3] = this.pointColor ? this.pointColor[0] : 0.16 + 0.30 * heightRatio;
                vertices[target + 4] = this.pointColor ? this.pointColor[1] : 0.82 - 0.20 * heightRatio;
                vertices[target + 5] = this.pointColor ? this.pointColor[2] : 0.96 - 0.06 * heightRatio;
                point += 1;
            }
            return {
                vertices,
                transform: { centerX, centerY, centerZ, radius },
                stats: {
                    sourcePoints,
                    renderedPoints,
                    invalidPoints: examinedPoints - renderedPoints,
                    fullResolution: this.fullResolution,
                    bounds: { minX, minY, minZ, maxX, maxY, maxZ }
                }
            };
        }

        validatePointCloud(message) {
            return this.decodePointCloud(message).stats;
        }

        setPointCloud(message) {
            const decoded = this.decodePointCloud(message);
            const gl = this.gl;
            gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
            gl.bufferData(gl.ARRAY_BUFFER, decoded.vertices, gl.DYNAMIC_DRAW);
            if (this.fullResolution && gl.getError && gl.getError() !== gl.NO_ERROR) {
                throw new Error('显存不足或完整点云上传失败；未退回简化点云');
            }
            this.pointCount = decoded.stats.renderedPoints;
            this.cloudTransform = decoded.transform;
            this._updatePathBuffer();
            this._updatePreviewPathBuffer();
            this._updateRobotBuffer();
            this._updateDraftBuffer();
            this._updateRoutePathBuffer();
            this.draw();
            return decoded.stats;
        }

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
            this._updatePathBuffer();
            this.draw();
            return this.globalPath.length;
        }

        clearGlobalPath() {
            this.globalPath = [];
            this.globalPathWarning = false;
            this._updatePathBuffer();
            this.draw();
        }

        setPreviewPath(points) {
            this.previewPath = (points || []).map((point) => ({
                x: Number(point && point.x),
                y: Number(point && point.y),
                z: Number(point && point.z)
            })).filter((point) => Number.isFinite(point.x)
                && Number.isFinite(point.y) && Number.isFinite(point.z));
            this._updatePreviewPathBuffer();
            this.draw();
        }

        clearPreviewPath() {
            this.previewPath = [];
            this._updatePreviewPathBuffer();
            this.draw();
        }

        setRoutePoints(points, activeIndex = -1, _executionState = 'idle') {
            this.routePoints = (points || []).filter((point) => point
                && point.x != null && point.y != null).map((point, index) => ({
                ...point,
                x: Number(point && point.x), y: Number(point && point.y),
                z: Number(point && point.z != null ? point.z : 0),
                yaw: Number(point && point.yaw != null ? point.yaw : 0),
                sequence: index
            })).filter((point) => Number.isFinite(point.x)
                && Number.isFinite(point.y) && Number.isFinite(point.z));
            this.routeActiveIndex = Number.isInteger(activeIndex) ? activeIndex : -1;
            this._updateRoutePathBuffer();
            this.draw();
        }

        clearRoutePoints() {
            this.routePoints = [];
            this.routeActiveIndex = -1;
            this._updateRoutePathBuffer();
            this.draw();
        }

        setCheckpointPoints(points) {
            this.checkpointPoints = (points || []).filter((point) => point
                && point.x != null && point.y != null).map((point) => ({
                ...point,
                x: Number(point && point.x), y: Number(point && point.y),
                z: Number(point && point.z != null ? point.z : 0)
            })).filter((point) => Number.isFinite(point.x)
                && Number.isFinite(point.y) && Number.isFinite(point.z));
            this.draw();
        }

        clearCheckpointPoints() {
            this.checkpointPoints = [];
            this.draw();
        }

        pickRoutePoint(clientX, clientY, maxPixelDistance = 18) {
            if (!this.cloudTransform || this.canvas.hidden || !this._isTopView()) return null;
            const rect = this.canvas.getBoundingClientRect();
            if (clientX < rect.left || clientX > rect.left + rect.width
                    || clientY < rect.top || clientY > rect.top + rect.height) return null;
            const maxDistance = Math.max(1, Number(maxPixelDistance) || 18);
            let nearest = null;
            let distance = maxDistance;
            this.routePoints.forEach((point) => {
                const pixel = this._projectMapPoint(point);
                if (!pixel) return;
                const candidate = Math.hypot(clientX - pixel.x, clientY - pixel.y);
                if (candidate <= distance) {
                    nearest = point;
                    distance = candidate;
                }
            });
            return nearest;
        }

        setDraftPose(x, y, z, yaw) {
            const pose = { x: Number(x), y: Number(y), z: Number(z), yaw: Number(yaw) };
            if (Object.values(pose).some((value) => !Number.isFinite(value))) return false;
            this.draftPose = pose;
            this._updateDraftBuffer();
            this.draw();
            return true;
        }

        clearDraftPose() {
            this.draftPose = null;
            this._updateDraftBuffer();
            this.draw();
        }

        setRobotPose(x, y, z, yaw) {
            const pose = {
                x: Number(x),
                y: Number(y),
                z: Number(z),
                yaw: Number(yaw)
            };
            if (!Number.isFinite(pose.x) || !Number.isFinite(pose.y)
                    || !Number.isFinite(pose.z) || !Number.isFinite(pose.yaw)) return false;
            this.robotPose = pose;
            this._updateRobotBuffer();
            this.draw();
            return true;
        }

        clearRobotPose() {
            this.robotPose = null;
            this._updateRobotBuffer();
            this.draw();
        }

        _updatePathBuffer() {
            const gl = this.gl;
            let vertices = new Float32Array(0);
            this.pathVertexCount = 0;
            if (this.globalPath.length >= 2 && this.cloudTransform) {
                const transform = this.cloudTransform;
                const radius = transform.radius;
                // A world-space ribbon is reliable on WebGL 1 / Intel GPUs,
                // where lineWidth values above one pixel are commonly ignored.
                const halfWidth = Math.max(0.0018, Math.min(0.008, 0.07 / radius));
                const color = this.globalPathWarning
                    ? [1.00, 0.56, 0.24]
                    : [0.27, 0.89, 0.61];
                const values = [];
                let lastTangent = { x: 1, y: 0 };
                this.globalPath.forEach((point, index) => {
                    const previous = this.globalPath[Math.max(0, index - 1)];
                    const next = this.globalPath[Math.min(this.globalPath.length - 1, index + 1)];
                    const tangentX = next.x - previous.x;
                    const tangentY = next.y - previous.y;
                    const tangentLength = Math.hypot(tangentX, tangentY);
                    if (tangentLength > 1e-6) {
                        lastTangent = {
                            x: tangentX / tangentLength,
                            y: tangentY / tangentLength
                        };
                    }
                    const normalX = -lastTangent.y;
                    const normalY = lastTangent.x;
                    const x = (point.x - transform.centerX) / radius;
                    const y = (point.y - transform.centerY) / radius;
                    // The display bridge already applies its configured terrain
                    // clearance. Preserve that world z instead of lifting twice.
                    const z = (point.z - transform.centerZ) / radius;
                    values.push(
                        x + normalX * halfWidth, y + normalY * halfWidth, z, ...color,
                        x - normalX * halfWidth, y - normalY * halfWidth, z, ...color
                    );
                });
                vertices = new Float32Array(values);
                this.pathVertexCount = this.globalPath.length * 2;
            }
            gl.bindBuffer(gl.ARRAY_BUFFER, this.pathBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.DYNAMIC_DRAW);
        }

        _updatePreviewPathBuffer() {
            const gl = this.gl;
            const values = [];
            this.previewPathVertexCount = 0;
            if (this.previewPath.length >= 2 && this.cloudTransform) {
                const transform = this.cloudTransform;
                const halfWidth = Math.max(0.0018, Math.min(0.008, 0.07 / transform.radius));
                let lastTangent = { x: 1, y: 0 };
                this.previewPath.forEach((point, index) => {
                    const previous = this.previewPath[Math.max(0, index - 1)];
                    const next = this.previewPath[Math.min(this.previewPath.length - 1, index + 1)];
                    const length = Math.hypot(next.x - previous.x, next.y - previous.y);
                    if (length > 1e-6) {
                        lastTangent = {
                            x: (next.x - previous.x) / length,
                            y: (next.y - previous.y) / length
                        };
                    }
                    const nx = -lastTangent.y;
                    const ny = lastTangent.x;
                    const x = (point.x - transform.centerX) / transform.radius;
                    const y = (point.y - transform.centerY) / transform.radius;
                    const z = (point.z - transform.centerZ) / transform.radius;
                    const color = [0.45, 0.72, 1.0];
                    values.push(
                        x + nx * halfWidth, y + ny * halfWidth, z, ...color,
                        x - nx * halfWidth, y - ny * halfWidth, z, ...color
                    );
                });
                this.previewPathVertexCount = this.previewPath.length * 2;
            }
            gl.bindBuffer(gl.ARRAY_BUFFER, this.previewPathBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(values), gl.DYNAMIC_DRAW);
        }

        _updateRoutePathBuffer() {
            const gl = this.gl;
            const values = [];
            this.routePathVertexCount = 0;
            if (this.routePoints.length >= 2 && this.cloudTransform) {
                const transform = this.cloudTransform;
                const halfWidth = Math.max(0.0015, Math.min(0.006, 0.05 / transform.radius));
                let lastTangent = { x: 1, y: 0 };
                this.routePoints.forEach((point, index) => {
                    const previous = this.routePoints[Math.max(0, index - 1)];
                    const next = this.routePoints[Math.min(this.routePoints.length - 1, index + 1)];
                    const length = Math.hypot(next.x - previous.x, next.y - previous.y);
                    if (length > 1e-6) {
                        lastTangent = {
                            x: (next.x - previous.x) / length,
                            y: (next.y - previous.y) / length
                        };
                    }
                    const nx = -lastTangent.y;
                    const ny = lastTangent.x;
                    const x = (point.x - transform.centerX) / transform.radius;
                    const y = (point.y - transform.centerY) / transform.radius;
                    const z = (point.z + 0.08 - transform.centerZ) / transform.radius;
                    const color = [0.94, 0.80, 0.28];
                    values.push(
                        x + nx * halfWidth, y + ny * halfWidth, z, ...color,
                        x - nx * halfWidth, y - ny * halfWidth, z, ...color
                    );
                });
                this.routePathVertexCount = this.routePoints.length * 2;
            }
            gl.bindBuffer(gl.ARRAY_BUFFER, this.routePathBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(values), gl.DYNAMIC_DRAW);
        }

        _updateDraftBuffer() {
            const gl = this.gl;
            const values = [];
            this.draftVertexCount = 0;
            if (this.draftPose && this.cloudTransform) {
                const pose = this.draftPose;
                const transform = this.cloudTransform;
                // Use the same scale before and after initial/goal confirmation.
                const size = Math.max(0.035, Math.min(0.11, 0.45 / transform.radius))
                    * this.markerScale;
                const x = (pose.x - transform.centerX) / transform.radius;
                const y = (pose.y - transform.centerY) / transform.radius;
                const z = (pose.z + 0.12 - transform.centerZ) / transform.radius;
                const dx = Math.cos(pose.yaw);
                const dy = Math.sin(pose.yaw);
                const px = -dy;
                const py = dx;
                const color = [1.0, 0.70, 0.18];
                values.push(
                    x + dx * size * 1.55, y + dy * size * 1.55, z, ...color,
                    x - dx * size * 0.55 + px * size * 0.72,
                    y - dy * size * 0.55 + py * size * 0.72, z, ...color,
                    x - dx * size * 0.55 - px * size * 0.72,
                    y - dy * size * 0.55 - py * size * 0.72, z, ...color
                );
                this.draftVertexCount = 3;
            }
            gl.bindBuffer(gl.ARRAY_BUFFER, this.draftBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(values), gl.DYNAMIC_DRAW);
        }

        _updateRobotBuffer() {
            const gl = this.gl;
            let vertices = new Float32Array(0);
            this.markerVertexCount = 0;
            if (this.robotPose && this.cloudTransform) {
                const pose = this.robotPose;
                const transform = this.cloudTransform;
                const radius = transform.radius;
                const baseX = (pose.x - transform.centerX) / radius;
                const baseY = (pose.y - transform.centerY) / radius;
                // Lift the marker slightly above the robot origin so floor points
                // cannot visually swallow it. Marker size remains legible on both
                // room-scale and campus-scale maps.
                const baseZ = (pose.z + 0.12 - transform.centerZ) / radius;
                const size = Math.max(0.035, Math.min(0.11, 0.45 / radius))
                    * this.markerScale;
                const dx = Math.cos(pose.yaw);
                const dy = Math.sin(pose.yaw);
                const px = -dy;
                const py = dx;
                const tipX = baseX + dx * size * 1.55;
                const tipY = baseY + dy * size * 1.55;
                const rearX = baseX - dx * size * 0.55;
                const rearY = baseY - dy * size * 0.55;
                vertices = new Float32Array([
                    tipX, tipY, baseZ, 1.00, 0.25, 0.12,
                    rearX + px * size * 0.72, rearY + py * size * 0.72, baseZ, 1.00, 0.82, 0.12,
                    rearX - px * size * 0.72, rearY - py * size * 0.72, baseZ, 1.00, 0.82, 0.12
                ]);
                this.markerVertexCount = 3;
            }
            gl.bindBuffer(gl.ARRAY_BUFFER, this.markerBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.DYNAMIC_DRAW);
        }

        clear() {
            this.pointCount = 0;
            this.cloudTransform = null;
            this.globalPath = [];
            this.globalPathWarning = false;
            this.previewPath = [];
            this.routePoints = [];
            this.checkpointPoints = [];
            this.routeActiveIndex = -1;
            this.draftPose = null;
            this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.buffer);
            this.gl.bufferData(
                this.gl.ARRAY_BUFFER, new Float32Array(0), this.gl.DYNAMIC_DRAW);
            this._updatePathBuffer();
            this._updatePreviewPathBuffer();
            this._updateRobotBuffer();
            this._updateDraftBuffer();
            this._updateRoutePathBuffer();
            this.draw();
        }

        resetView() {
            this.yaw = -0.72;
            this.pitch = -0.88;
            this.zoom = 0.82;
            this.panX = 0;
            this.panY = 0;
            this.draw();
        }

        setTopView() {
            this.yaw = 0;
            this.pitch = 0;
            this.zoom = 0.82;
            this.panX = 0;
            this.panY = 0;
            this.draw();
        }

        setGroundPoseInteraction(enabled, callback) {
            this.groundPoseInteraction = {
                enabled: Boolean(enabled), callback: typeof callback === 'function' ? callback : null
            };
            if (!enabled) this.drag = null;
        }

        isTopView() {
            return this._isTopView();
        }

        _isTopView() {
            return Math.abs(this.yaw) < 1e-6 && Math.abs(this.pitch) < 1e-6;
        }

        _screenToGroundMapPoint(clientX, clientY) {
            if (!this.cloudTransform || !this._isTopView()) return null;
            const rect = this.canvas.getBoundingClientRect();
            if (!rect.width || !rect.height
                    || clientX < rect.left || clientX > rect.left + rect.width
                    || clientY < rect.top || clientY > rect.top + rect.height) return null;
            const ndcX = 2 * (clientX - rect.left) / rect.width - 1;
            const ndcY = 1 - 2 * (clientY - rect.top) / rect.height;
            const aspect = Math.max(0.2, this.width / this.height);
            const transform = this.cloudTransform;
            const x = (ndcX * aspect / this.zoom - this.panX) * transform.radius
                + transform.centerX;
            const y = (ndcY / this.zoom - this.panY) * transform.radius
                + transform.centerY;
            return Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null;
        }

        _projectMapPoint(point) {
            if (!this.cloudTransform || this.canvas.hidden) return null;
            const rect = this.canvas.getBoundingClientRect();
            if (!rect.width || !rect.height) return null;
            const t = this.cloudTransform;
            const px = (point.x - t.centerX) / t.radius + this.panX;
            const py = (point.y - t.centerY) / t.radius + this.panY;
            const pz = (point.z - t.centerZ) / t.radius;
            const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw);
            const cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
            const yawX = cy * px - sy * py;
            const yawY = sy * px + cy * py;
            const ndcX = yawX * this.zoom / Math.max(0.2, this.width / this.height);
            const ndcY = (cp * yawY - sp * pz) * this.zoom;
            if (Math.abs(ndcX) > 1.1 || Math.abs(ndcY) > 1.1) return null;
            return {
                x: rect.left + (ndcX + 1) * rect.width / 2,
                y: rect.top + (1 - ndcY) * rect.height / 2
            };
        }

        _syncMarkerLayer() {
            if (!this.markerLayer) return;
            this.markerLayer.style.display = this.canvas.hidden || !this.cloudTransform
                ? 'none' : '';
            this.markerLayer.replaceChildren();
            if (this.canvas.hidden || !this.cloudTransform) return;
            const parentRect = this.container.getBoundingClientRect();
            const addBadge = (point, label, color, title, offsetX = 0) => {
                const pixel = this._projectMapPoint(point);
                if (!pixel) return;
                const badge = global.document.createElement('span');
                badge.textContent = label;
                badge.title = title;
                Object.assign(badge.style, {
                    position: 'absolute', left: `${pixel.x - parentRect.left + offsetX}px`,
                    top: `${pixel.y - parentRect.top}px`,
                    transform: 'translate(-50%, -50%)',
                    minWidth: '24px', height: '24px', lineHeight: '24px',
                    padding: '0 3px', textAlign: 'center', borderRadius: '12px',
                    border: '2px solid #07101c', background: color,
                    color: '#07101c', font: 'bold 12px sans-serif',
                    boxShadow: '0 0 0 1px rgba(255,255,255,.8)'
                });
                this.markerLayer.appendChild(badge);
            };
            this.routePoints.forEach((point, index) => {
                addBadge(point, String(index + 1),
                    index === this.routeActiveIndex ? '#ff8f68' : '#f3cc46',
                    point.name || `路径点 ${index + 1}`);
            });
            this.checkpointPoints.forEach((point) => {
                addBadge(point, '检', '#75e0ff', point.name || '巡检检测点', 22);
            });
        }

        _bindProgram(program, buffer) {
            const gl = this.gl;
            gl.useProgram(program);
            gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
            const stride = 6 * Float32Array.BYTES_PER_ELEMENT;
            const position = gl.getAttribLocation(program, 'aPosition');
            const color = gl.getAttribLocation(program, 'aColor');
            gl.enableVertexAttribArray(position);
            gl.vertexAttribPointer(position, 3, gl.FLOAT, false, stride, 0);
            gl.enableVertexAttribArray(color);
            gl.vertexAttribPointer(color, 3, gl.FLOAT, false, stride, 12);
            gl.uniform1f(gl.getUniformLocation(program, 'uYaw'), this.yaw);
            gl.uniform1f(gl.getUniformLocation(program, 'uPitch'), this.pitch);
            gl.uniform1f(gl.getUniformLocation(program, 'uZoom'), this.zoom);
            gl.uniform1f(gl.getUniformLocation(program, 'uAspect'), Math.max(0.2, this.width / this.height));
            gl.uniform2f(gl.getUniformLocation(program, 'uPan'), this.panX, this.panY);
        }

        draw() {
            if (!this.gl) return;
            this._syncMarkerLayer();
            if (this.canvas.hidden) return;
            const gl = this.gl;
            gl.viewport(0, 0, this.canvas.width, this.canvas.height);
            gl.clearColor(0.025, 0.050, 0.090, 1);
            gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
            if (this.pointCount) {
                this._bindProgram(this.program, this.buffer);
                gl.drawArrays(gl.POINTS, 0, this.pointCount);
            }
            if (this.pathVertexCount) {
                // This is an operational overlay. Keep the ribbon readable
                // through dense returns, while its terrain z still conveys the
                // planned slope/step profile.
                gl.disable(gl.DEPTH_TEST);
                this._bindProgram(this.markerProgram, this.pathBuffer);
                gl.drawArrays(gl.TRIANGLE_STRIP, 0, this.pathVertexCount);
                gl.enable(gl.DEPTH_TEST);
            }
            if (this.previewPathVertexCount) {
                gl.disable(gl.DEPTH_TEST);
                this._bindProgram(this.markerProgram, this.previewPathBuffer);
                gl.drawArrays(gl.TRIANGLE_STRIP, 0, this.previewPathVertexCount);
                gl.enable(gl.DEPTH_TEST);
            }
            if (this.routePathVertexCount) {
                gl.disable(gl.DEPTH_TEST);
                this._bindProgram(this.markerProgram, this.routePathBuffer);
                gl.drawArrays(gl.TRIANGLE_STRIP, 0, this.routePathVertexCount);
                gl.enable(gl.DEPTH_TEST);
            }
            if (this.markerVertexCount) {
                // The pose indicator is operational information, so keep it
                // visible even when dense points lie at the same height.
                gl.disable(gl.DEPTH_TEST);
                this._bindProgram(this.markerProgram, this.markerBuffer);
                gl.drawArrays(gl.TRIANGLES, 0, this.markerVertexCount);
                gl.enable(gl.DEPTH_TEST);
            }
            if (this.draftVertexCount) {
                gl.disable(gl.DEPTH_TEST);
                this._bindProgram(this.markerProgram, this.draftBuffer);
                gl.drawArrays(gl.TRIANGLES, 0, this.draftVertexCount);
                gl.enable(gl.DEPTH_TEST);
            }
        }
    }

    global.PointCloudViewer = PointCloudViewer;
}(window));
