/*
 * Minimal, dependency-free ROSLIB-compatible client for this project.
 *
 * It implements only the rosbridge operations used by the four web pages:
 * topic publish/subscribe and service calls.  Keeping this file beside the
 * pages makes the UI work when the robot Wi-Fi has no Internet access.
 */
(function (global) {
    'use strict';

    let nextId = 1;
    const FRAGMENT_TIMEOUT_MS = 10000;
    const MAX_FRAGMENT_COUNT = 256;
    const MAX_FRAGMENTED_MESSAGE_CHARACTERS = 32 * 1024 * 1024;

    function makeId(prefix) {
        const id = nextId;
        nextId += 1;
        return `${prefix}:${id}`;
    }

    class Ros {
        constructor(options) {
            this.url = options && options.url;
            this.socket = null;
            this.isConnected = false;
            this.handlers = new Map();
            this.topicCallbacks = new Map();
            this.serviceCallbacks = new Map();
            this.fragmentBuffers = new Map();
            this.connect(this.url);
        }

        on(event, callback) {
            if (!this.handlers.has(event)) this.handlers.set(event, new Set());
            this.handlers.get(event).add(callback);
            return this;
        }

        off(event, callback) {
            const callbacks = this.handlers.get(event);
            if (callbacks) callbacks.delete(callback);
            return this;
        }

        emit(event, value) {
            const callbacks = this.handlers.get(event);
            if (!callbacks) return;
            callbacks.forEach((callback) => {
                try { callback(value); } catch (error) { window.setTimeout(() => { throw error; }, 0); }
            });
        }

        connect(url) {
            if (!url) throw new Error('rosbridge WebSocket URL is required');
            if (this.socket) this.close();
            this.url = url;
            const socket = new WebSocket(url);
            this.socket = socket;
            socket.addEventListener('open', () => {
                this.isConnected = true;
                this.emit('connection');
            });
            socket.addEventListener('error', () => this.emit(
                'error', new Error(`无法连接 rosbridge WebSocket：${this.url}`)));
            socket.addEventListener('close', (event) => {
                this.isConnected = false;
                if (this.socket === socket) this.socket = null;
                this.clearFragmentBuffers();
                this.serviceCallbacks.forEach(({ failed, timer }) => {
                    window.clearTimeout(timer);
                    if (failed) failed(new Error('rosbridge connection closed'));
                });
                this.serviceCallbacks.clear();
                this.emit('close', event);
            });
            socket.addEventListener('message', (event) => this.handleMessage(event.data));
        }

        close() {
            const socket = this.socket;
            this.socket = null;
            this.isConnected = false;
            this.clearFragmentBuffers();
            if (socket && socket.readyState < WebSocket.CLOSING) socket.close();
        }

        send(message) {
            if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
                throw new Error('rosbridge WebSocket is not connected');
            }
            this.socket.send(JSON.stringify(message));
        }

        handleMessage(data) {
            let message;
            try {
                message = JSON.parse(data);
            } catch (error) {
                this.emit('error', new Error(`Invalid rosbridge JSON: ${error.message}`));
                return;
            }

            if (message.op === 'fragment') {
                this.handleFragment(message);
                return;
            }

            if (message.op === 'publish') {
                const callbacks = this.topicCallbacks.get(message.topic);
                if (callbacks) callbacks.forEach((callback) => callback(message.msg));
                return;
            }

            if (message.op === 'service_response' && message.id) {
                const pending = this.serviceCallbacks.get(message.id);
                if (!pending) return;
                this.serviceCallbacks.delete(message.id);
                window.clearTimeout(pending.timer);
                if (message.result === false) {
                    if (pending.failed) pending.failed(message.values || message);
                } else {
                    pending.succeeded(message.values || {});
                }
                return;
            }

            if (message.op === 'status' && message.level === 'error') {
                this.emit('error', new Error(message.msg || 'rosbridge reported an error'));
            }
        }

        handleFragment(message) {
            const id = typeof message.id === 'string' ? message.id : '';
            const num = Number(message.num);
            const total = Number(message.total);
            const data = message.data;
            if (!id ||
                !Number.isInteger(num) ||
                !Number.isInteger(total) ||
                num < 0 ||
                total < 1 ||
                num >= total ||
                total > MAX_FRAGMENT_COUNT ||
                typeof data !== 'string') {
                this.emit('error', new Error('Invalid rosbridge fragment metadata'));
                return;
            }

            let buffer = this.fragmentBuffers.get(id);
            if (!buffer) {
                buffer = {
                    total,
                    parts: new Array(total),
                    received: 0,
                    characters: 0,
                    timer: window.setTimeout(() => {
                        if (!this.fragmentBuffers.has(id)) return;
                        this.discardFragmentBuffer(id);
                        this.emit(
                            'error',
                            new Error(`Timed out waiting for rosbridge fragments: ${id}`)
                        );
                    }, FRAGMENT_TIMEOUT_MS)
                };
                this.fragmentBuffers.set(id, buffer);
            } else if (buffer.total !== total) {
                this.discardFragmentBuffer(id);
                this.emit(
                    'error',
                    new Error(`Inconsistent rosbridge fragment count: ${id}`)
                );
                return;
            }

            const existing = buffer.parts[num];
            if (existing !== undefined) {
                if (existing !== data) {
                    this.discardFragmentBuffer(id);
                    this.emit(
                        'error',
                        new Error(`Conflicting duplicate rosbridge fragment: ${id}/${num}`)
                    );
                }
                return;
            }

            buffer.parts[num] = data;
            buffer.received += 1;
            buffer.characters += data.length;
            if (buffer.characters > MAX_FRAGMENTED_MESSAGE_CHARACTERS) {
                this.discardFragmentBuffer(id);
                this.emit(
                    'error',
                    new Error(`Rosbridge fragmented message is too large: ${id}`)
                );
                return;
            }
            if (buffer.received !== buffer.total) return;

            const serializedMessage = buffer.parts.join('');
            this.discardFragmentBuffer(id);
            this.handleMessage(serializedMessage);
        }

        discardFragmentBuffer(id) {
            const buffer = this.fragmentBuffers.get(id);
            if (!buffer) return;
            window.clearTimeout(buffer.timer);
            this.fragmentBuffers.delete(id);
        }

        clearFragmentBuffers() {
            this.fragmentBuffers.forEach((buffer) => {
                window.clearTimeout(buffer.timer);
            });
            this.fragmentBuffers.clear();
        }

        addTopicCallback(topic, callback) {
            if (!this.topicCallbacks.has(topic)) this.topicCallbacks.set(topic, new Set());
            this.topicCallbacks.get(topic).add(callback);
        }

        removeTopicCallback(topic, callback) {
            const callbacks = this.topicCallbacks.get(topic);
            if (!callbacks) return 0;
            if (callback) callbacks.delete(callback); else callbacks.clear();
            if (!callbacks.size) this.topicCallbacks.delete(topic);
            return callbacks.size;
        }
    }

    class Topic {
        constructor(options) {
            this.ros = options.ros;
            this.name = options.name;
            this.messageType = options.messageType;
            this.compression = options.compression || 'none';
            this.throttleRate = Math.max(
                0, Number(options.throttleRate || options.throttle_rate) || 0);
            this.queueLength = Math.max(
                0, Number(options.queueLength || options.queue_length) || 0);
            this.fragmentSize = Math.max(
                0, Number(options.fragmentSize || options.fragment_size) || 0);
            this.subscriptionId = null;
            this.advertiseId = null;
            this.callbacks = new Set();
        }

        subscribe(callback) {
            if (this.callbacks.has(callback)) return;
            this.callbacks.add(callback);
            this.ros.addTopicCallback(this.name, callback);
            if (!this.subscriptionId) {
                this.subscriptionId = makeId('subscribe');
                const subscribeMessage = {
                    op: 'subscribe',
                    id: this.subscriptionId,
                    topic: this.name,
                    type: this.messageType,
                    compression: this.compression,
                    throttle_rate: this.throttleRate,
                    queue_length: this.queueLength
                };
                if (this.fragmentSize > 0) {
                    subscribeMessage.fragment_size = this.fragmentSize;
                }
                this.ros.send(subscribeMessage);
            }
        }

        unsubscribe(callback) {
            if (callback) {
                this.callbacks.delete(callback);
                this.ros.removeTopicCallback(this.name, callback);
            } else {
                this.callbacks.forEach((item) => this.ros.removeTopicCallback(this.name, item));
                this.callbacks.clear();
            }
            if (!this.callbacks.size && this.subscriptionId) {
                this.ros.send({ op: 'unsubscribe', id: this.subscriptionId, topic: this.name });
                this.subscriptionId = null;
            }
        }

        advertise() {
            if (this.advertiseId) return;
            this.advertiseId = makeId('advertise');
            this.ros.send({
                op: 'advertise', id: this.advertiseId,
                topic: this.name, type: this.messageType
            });
        }

        unadvertise() {
            if (!this.advertiseId) return;
            this.ros.send({ op: 'unadvertise', id: this.advertiseId, topic: this.name });
            this.advertiseId = null;
        }

        publish(message) {
            this.advertise();
            this.ros.send({ op: 'publish', topic: this.name, msg: message });
        }
    }

    class Service {
        constructor(options) {
            this.ros = options.ros;
            this.name = options.name;
            this.serviceType = options.serviceType;
        }

        callService(request, succeeded, failed) {
            const id = makeId('service');
            const pending = {
                succeeded: succeeded || (() => {}),
                failed: failed || (() => {}),
                timer: null
            };
            pending.timer = window.setTimeout(() => {
                if (!this.ros.serviceCallbacks.delete(id)) return;
                pending.failed(new Error(`ROS service call timed out: ${this.name}`));
            }, 30000);
            this.ros.serviceCallbacks.set(id, pending);
            try {
                this.ros.send({
                    op: 'call_service', id, service: this.name,
                    type: this.serviceType, args: request || {}
                });
            } catch (error) {
                this.ros.serviceCallbacks.delete(id);
                window.clearTimeout(pending.timer);
                if (failed) failed(error); else throw error;
            }
        }
    }

    class Message { constructor(values) { Object.assign(this, values || {}); } }
    class ServiceRequest { constructor(values) { Object.assign(this, values || {}); } }

    global.ROSLIB = Object.freeze({ Ros, Topic, Service, Message, ServiceRequest });
}(window));
