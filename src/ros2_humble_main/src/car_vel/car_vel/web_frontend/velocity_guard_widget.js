/* Read-only final velocity protection reasons; never sends motion commands. */
(function (root) {
    'use strict';
    const slots = new WeakMap();
    const REPEATING_STOP_CODES = new Set([
        'global_localization_recovering', 'upstream_discarded', 'upstream_zero',
        'upstream_stale', 'lio_paused_or_stale', 'navigation_paused_or_stale'
    ]);
    const STOP_TEXT = Object.freeze({
        upstream_stale: '导航速度消息已过期，已零速停车',
        upstream_zero: '导航控制器当前给出零速',
        terrain_unsafe: '地形保护停车：等待有效且可通行的地形证据',
        direction_not_authorized: '当前方向缺少地形授权，已零速停车'
    });
    const TERRAIN_TEXT = Object.freeze({flat:'平地',curb:'低台阶 / 路缘',ramp:'坡道',rough:'粗糙地面',
        grass:'草地',gravel:'碎石',stairs_up:'上楼梯',stairs_down:'下楼梯',unknown:'未知',blocked:'不可通行',negative_obstacle:'坑洞 / 临边'});
    function vector(value) {
        return Array.isArray(value) && value.length === 3
            && value.every(number => typeof number === 'number' && Number.isFinite(number)) ? value : null;
    }
    function speed(value) { return typeof value === 'number' && Number.isFinite(value) && value >= 0 ? `${value.toFixed(2)} m/s` : '--'; }
    function reasonText(value) {
        if (value.blocked) return String(value.reason || STOP_TEXT[value.reason_code] || '最终速度保护已停车');
        if (value.schema_version >= 2 && value.navigation_active === true) {
            return STOP_TEXT[value.reason_code] || '';
        }
        return '';
    }
    function attach(session, element, messageType, options = {}) {
        const previous = slots.get(element);
        if (previous) root.clearInterval(previous.timer);
        const slot = { at: 0, blocked: false, warning: false, warningCode: '', reasonCode: '',
            stoppedLogs: new Map(), timer: null };
        const motion = options.motionElement;
        if (motion) {
            motion.hidden = true;
            motion.classList.add('navigation-motion-status');
            motion.innerHTML = '<div class="navigation-motion-head"><strong>导航速度与停车原因</strong><span data-speed-state>等待状态</span></div>'
                + '<div class="navigation-motion-grid">'
                + '<div><span>导航前进请求</span><strong data-speed-request>--</strong></div>'
                + '<div><span>最终前进指令</span><strong data-speed-command>--</strong></div>'
                + '<div><span>导航系统上限</span><strong data-speed-limit>--</strong></div></div>'
                + '<p class="hint" data-speed-turn>转向请求 -- · 下发 -- rad/s</p>'
                + '<p class="hint" data-speed-stream hidden></p>'
                + '<p class="hint" data-speed-terrain>实时地形 --</p>'
                + '<p class="hint" data-speed-policy>等待有效速度策略。</p>'
                + '<p class="hint">下发指令不等于本体实测速度。</p>';
        }
        function clearMotion(text) {
            if (!motion || motion.hidden) return;
            for (const key of ['request', 'command', 'limit']) motion.querySelector(`[data-speed-${key}]`).textContent = '--';
            motion.querySelector('[data-speed-turn]').textContent = '转向请求 -- · 下发 -- rad/s';
            motion.querySelector('[data-speed-stream]').textContent = '原始控制器状态已过期';
            motion.querySelector('[data-speed-terrain]').textContent = '实时地形 --';
            motion.querySelector('[data-speed-state]').textContent = text;
        }
        function renderMotion(value, stop) {
            if (!motion) return;
            const scoped = value.schema_version >= 2 && value.body_id === 'xingji'
                && (value.navigation_mode === 'terrain_3d' || value.terrain_guard_enabled === true)
                && value.speed_policy === 'nav2_body_limits';
            motion.hidden = !scoped;
            if (!scoped) return;
            const requested = value.input_fresh === true ? vector(value.request_velocity) : null;
            const commanded = vector(value.commanded_velocity);
            motion.querySelector('[data-speed-request]').textContent = requested ? `${requested[0].toFixed(2)} m/s` : '--';
            motion.querySelector('[data-speed-command]').textContent = commanded ? `${commanded[0].toFixed(2)} m/s` : '--';
            motion.querySelector('[data-speed-limit]').textContent = speed(value.speed_limit_mps);
            motion.querySelector('[data-speed-turn]').textContent = `转向请求 ${requested ? requested[2].toFixed(2) : '--'} · 下发 ${commanded ? commanded[2].toFixed(2) : '--'} rad/s`;
            motion.querySelector('[data-speed-terrain]').textContent = `实时地形 ${TERRAIN_TEXT[value.terrain_type] || '--'}`;
            const stream = value.controller_stream;
            const streamNode = motion.querySelector('[data-speed-stream]');
            streamNode.hidden = !stream || typeof stream !== 'object';
            if (!streamNode.hidden) {
                const age = stream.age_sec;
                const rate = stream.receive_rate_hz;
                const raw = vector(stream.request_velocity);
                const current = stream.seen === true && typeof age === 'number' && Number.isFinite(age)
                    && age >= 0 && age <= (typeof value.command_timeout_sec === 'number' ? value.command_timeout_sec : .3);
                streamNode.textContent = current && raw ? `原始控制器前进请求 ${raw[0].toFixed(2)} m/s`
                    : (stream.seen ? '原始控制器样本已过期' : '尚未收到原始控制器速度');
                if (typeof rate === 'number' && Number.isFinite(rate) && rate >= 0) streamNode.textContent += ` · 最近窗口 ${rate.toFixed(1)} Hz`;
            }
            const sources = {
                restricted_map: '受限场地地图先验仍使用独立低速上限。',
                unanchored_observation: '地面高度证据尚未充分确认，保留独立低速上限。',
                safety_stop: '危险或必要状态无效，已零速停车。',
                campus_connector: '园区连接段独占控制，速度遵循连接段约束。'
            };
            motion.querySelector('[data-speed-policy]').textContent = sources[value.speed_limit_source]
                || '星玑固定越障步态；不按可通行地形额外减速，速度由导航与本体上限控制。';
            motion.querySelector('[data-speed-state]').textContent = stop || (
                value.navigation_active !== true ? '当前无导航任务'
                    : value.source === 'teleop' ? '遥控正在接管；此上限仅适用于导航'
                    : value.source === 'campus' ? '园区连接段正在接管'
                    : commanded ? '已下发导航指令' : '等待有效指令');
        }
        function record(text) {
            // Display/diagnostics must not depend on the operation log sink.
            try { if (typeof options.onEvent === 'function') options.onEvent(text); } catch (_) {}
        }
        slots.set(element, slot);
        element.hidden = true;
        session.subscribe('/navigation/velocity_guard', messageType, (message) => {
            if (slots.get(element) !== slot) return;
            slot.at = Date.now();
            try {
                const value = JSON.parse(message.data);
                if (typeof value.blocked !== 'boolean') throw new Error('invalid status');
                if (value.warning !== undefined && typeof value.warning !== 'boolean') {
                    throw new Error('invalid warning status');
                }
                const warning = value.warning === true;
                const warningCode = warning ? String(value.warning_code || 'motion_warning') : '';
                const warningText = String(value.warning_reason || '导航运动诊断需要检查');
                if (warning && warningCode !== slot.warningCode) {
                    record(`运动诊断：${warningText}`);
                } else if (!warning && slot.warning) {
                    record('当前运动诊断提示已解除；请以任务状态和现场运动为准。');
                }
                slot.blocked = value.blocked;
                slot.warning = warning;
                slot.warningCode = warningCode;
                const baseReason = reasonText(value);
                const streamReason = String(value.stream_reason || '');
                const stop = baseReason && streamReason && !baseReason.includes(streamReason)
                    ? `${baseReason}；${streamReason}` : baseReason;
                const reasonCode = stop ? String(value.reason_code || value.reason || 'blocked') : '';
                const command = vector(value.commanded_velocity);
                if (value.navigation_active === false || (!value.blocked && command
                        && command.some(number => Math.abs(number) > 1e-6))) {
                    slot.stoppedLogs.clear();
                }
                if (value.schema_version >= 2 && value.navigation_active === true
                        && stop && reasonCode !== slot.reasonCode) {
                    const last = slot.stoppedLogs.get(reasonCode);
                    if (!REPEATING_STOP_CODES.has(reasonCode) || last === undefined
                            || slot.at-last >= 10000) {
                        record(`零速原因：${stop}`);
                        if (REPEATING_STOP_CODES.has(reasonCode)) slot.stoppedLogs.set(reasonCode, slot.at);
                    }
                }
                slot.reasonCode = reasonCode;
                element.hidden = !(stop || warning);
                element.textContent = stop || (warning ? warningText : '');
                renderMotion(value, stop);
            } catch (_) {
                slot.blocked = true;
                element.hidden = false;
                element.textContent = '最终速度保护状态无法解析，请检查后端服务。';
                clearMotion('速度状态无法解析');
            }
        }, { queueLength: 1, throttleRate: 100 });
        slot.timer = root.setInterval(() => {
            if (Date.now()-slot.at > 2000) clearMotion('速度状态已过期，等待后端更新');
            if ((slot.blocked || slot.warning) && Date.now()-slot.at > 2000) {
                element.hidden = false;
                element.textContent = slot.blocked ? '最终速度保护状态已过期，请检查后端服务。'
                    : '运动诊断状态已过期，请检查后端服务，不能据此判断当前是否移动。';
            }
        }, 500);
    }
    root.VelocityGuardWidget = { attach };
})(window);
