#!/usr/bin/env node
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/inspection.html'), 'utf8');
const functions = html.slice(html.indexOf('    async function refreshRoutes('),
    html.indexOf('    async function saveTemplate('));
const subscription = html.slice(html.indexOf("            activeSession.subscribe('/inspection/status'"),
    html.indexOf("            activeSession.subscribe('/inspection/alerts'"));
const reentry = html.slice(html.indexOf('            if (taskState.active) clearInspectionDraft();\n            else applyInspectionDraftForm'),
    html.indexOf('            refreshHistory();'));
const oldRoute = {id: 2, name: '旧路线', points: []};
const newRoute = {id: 3, name: '新路线', points: []};
function harness() {
    const select = {value: '3', add() {}, set innerHTML(_) {this.value = '';}};
    const calls = [], maps = [];
    const context = vm.createContext({
        Number, String, Boolean, Set, Array,
        Option: function(text, value) {this.text = text; this.value = value;},
        document: {getElementById: () => select},
        route: newRoute, routes: [oldRoute, newRoute], routeSelectionGeneration: 0,
        loadedMapName: 'new_map', mapReady: true,
        mapCanvas: {mapInfo: {}, mapData: []},
        mapOverlay: {classList: {add() {}, remove() {}, toggle() {}}},
        taskState: {active: false, state: 'cancelled', route_id: 2, map_name: 'old_map'},
        inspectionDraft: {mapName: 'new_map', routeId: 3, checkpoints: []},
        template: {checkpoints: []},
        session: {connected: true, async call(name, type, args) {
            calls.push({name, args});
            if (name === '/list_routes') return {success: true, routes: [oldRoute, newRoute]};
            return {success: true, template_json: '{"checkpoints":[]}'};
        }},
        activeSession: {subscribe(topic, type, callback) {context.status = callback;}},
        UI: {TYPES: {}}, selectedRouteId: () => Number(select.value),
        routeFromMessage: value => value, parseJson: JSON.parse,
        invalidateThreeDPreview() {}, renderCheckpointList() {}, renderRouteOnMap() {},
        renderProgressList() {}, renderTaskState() {}, clearGlobalPath() {},
        clearInspectionDraft() {context.inspectionDraft = null;},
        saveInspectionDraft() {context.savedRouteId = context.route?.id || 0;},
        refreshMaps: async name => {maps.push(name);},
        restoreInspectionDraft: async () => {context.draftRestored = true;},
        applyInspectionDraftForm() {}, isXingjiTerrain3D: () => false,
        log() {}, draftStore: {save: value => value},
    });
    vm.runInContext(functions + subscription, context);
    return {context, select, calls, maps};
}
async function settle() {await new Promise(resolve => setImmediate(resolve));}
(async () => {
    for (const mode of ['map_2d', 'terrain_3d']) {
        for (const state of ['cancelled', 'completed', 'failed']) {
            const {context: c, select, calls} = harness();
            for (let i = 0; i < 3; i++) {
                c.status({data: JSON.stringify({active: false, state, route_id: 2,
                    map_name: 'old_map', navigation_mode: mode})});
                await settle();
            }
            assert.equal(select.value, '3', `${state}/${mode}: terminal pushes must not switch routes`);
            assert.equal(c.route.id, 3);
            assert.equal(calls.length, 0, 'terminal tasks must not reload route lists');
            c.loadedMapName = '';
            c.status({data: JSON.stringify(c.taskState)});
            assert.equal(c.loadedMapName, '', 'terminal tasks must not supply a stale map');
            await vm.runInContext('(async () => {' + reentry + '})()', c);
            assert.equal(c.draftRestored, true, 'reentry must restore idle draft after termination');
            assert.equal(c.loadedMapName, '');
        }
    }
    for (const state of ['navigating', 'paused']) {
        const {context: c, select} = harness();
        c.status({data: JSON.stringify({active: true, state, route_id: 2, map_name: 'new_map'})});
        await settle();
        assert.equal(select.value, '2', 'active/paused task still owns its route');
        assert.equal(c.inspectionDraft, null);
    }
    {
        const {context: c, select} = harness();
        select.value = ''; c.route = null;
        await c.refreshRoutes();
        assert.equal(c.route, null, 'manual refresh must not fall back to a terminated route');
    }
    {
        const {context: c, select} = harness();
        let resolveList;
        const original = c.session.call;
        c.session.call = (name, ...args) => name === '/list_routes'
            ? new Promise(resolve => {resolveList = resolve;}) : original(name, ...args);
        const refresh = c.refreshRoutes(2);
        select.value = '3';
        await c.selectRoute(3);
        resolveList({success: true, routes: [oldRoute, newRoute]});
        await refresh;
        assert.equal(select.value, '3', 'late list reply must not undo a newer user selection');
        assert.equal(c.route.id, 3);
        assert.equal(c.savedRouteId, 3);
    }
    console.log('inspection terminal task route selection: ok');
})().catch(error => {console.error(error); process.exitCode = 1;});
