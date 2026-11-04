/* Session-aware health monitor for the global mapping PointCloud2 stream. */
(function (global, factory) {
    'use strict';
    const api = factory(global);
    if (typeof module === 'object' && module.exports) module.exports = api;
    global.MappingCloudHealth = api;
}(typeof window !== 'undefined' ? window :
    (typeof globalThis !== 'undefined' ? globalThis : global), function (global) {
    'use strict';

    class GlobalCloudHealthMonitor {
        constructor(options = {}) {
            this.initialTimeoutMs = Math.max(
                1000, Number(options.initialTimeoutMs) || 45000);
            this.staleTimeoutMs = Math.max(
                1000, Number(options.staleTimeoutMs) || 15000);
            this.checkIntervalMs = Math.max(
                250, Number(options.checkIntervalMs) || 1000);
            this.onState = typeof options.onState === 'function'
                ? options.onState : () => {};
            this.now = typeof options.now === 'function'
                ? options.now : () => Date.now();
            this.setTimeoutFn = options.setTimeoutFn || global.setTimeout.bind(global);
            this.clearTimeoutFn = options.clearTimeoutFn || global.clearTimeout.bind(global);
            this.setIntervalFn = options.setIntervalFn || global.setInterval.bind(global);
            this.clearIntervalFn = options.clearIntervalFn || global.clearInterval.bind(global);
            this.connected = false;
            this.mappingActive = false;
            this.lastValidAt = null;
            this.state = 'stopped';
            this.generation = 0;
            this.initialTimer = null;
            this.staleTimer = null;
        }

        setConnected(connected) {
            const next = Boolean(connected);
            if (next === this.connected) return;
            this.connected = next;
            this.generation += 1;
            this._clearTimers();
            this.lastValidAt = null;
            if (!next) {
                this._emit('disconnected');
            } else if (this.mappingActive) {
                this._emit('waiting');
                this._armInitialTimeout();
            }
        }

        start() {
            this.mappingActive = true;
            this.generation += 1;
            this._clearTimers();
            this.lastValidAt = null;
            this._emit(this.connected ? 'waiting' : 'disconnected');
            if (this.connected) this._armInitialTimeout();
        }

        stop() {
            this.mappingActive = false;
            this.generation += 1;
            this._clearTimers();
            this.lastValidAt = null;
            this._emit('stopped');
        }

        markValidFrame() {
            if (!this.connected || !this.mappingActive) return false;
            const recovered = this.state === 'missing' || this.state === 'stale';
            if (this.initialTimer !== null) {
                this.clearTimeoutFn(this.initialTimer);
                this.initialTimer = null;
            }
            this.lastValidAt = this.now();
            if (this.staleTimer === null) {
                const generation = this.generation;
                this.staleTimer = this.setIntervalFn(() => {
                    if (generation !== this.generation || !this.mappingActive ||
                        !this.connected || this.lastValidAt === null) return;
                    const ageMs = Math.max(0, this.now() - this.lastValidAt);
                    if (ageMs >= this.staleTimeoutMs) {
                        this._emit('stale', { ageMs });
                    }
                }, this.checkIntervalMs);
            }
            this._emit('ready', { recovered });
            return true;
        }

        destroy() {
            this.generation += 1;
            this._clearTimers();
        }

        _armInitialTimeout() {
            const generation = this.generation;
            this.initialTimer = this.setTimeoutFn(() => {
                this.initialTimer = null;
                if (generation !== this.generation || !this.mappingActive ||
                    !this.connected || this.lastValidAt !== null) return;
                this._emit('missing');
            }, this.initialTimeoutMs);
        }

        _clearTimers() {
            if (this.initialTimer !== null) {
                this.clearTimeoutFn(this.initialTimer);
                this.initialTimer = null;
            }
            if (this.staleTimer !== null) {
                this.clearIntervalFn(this.staleTimer);
                this.staleTimer = null;
            }
        }

        _emit(state, details = {}) {
            const changed = state !== this.state;
            this.state = state;
            if (changed || details.recovered) {
                this.onState({ state, ...details });
            }
        }
    }

    return { GlobalCloudHealthMonitor };
}));
