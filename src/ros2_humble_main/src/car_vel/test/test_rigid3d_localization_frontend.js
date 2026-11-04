const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const page = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/navigation.html'), 'utf8');
function namedFunction(name) {
    const start = page.indexOf(`    function ${name}(`);
    assert(start >= 0, name);
    let depth = 0, brace = page.indexOf('{', start);
    // These health adapters contain no brace characters in string literals.
    for (let i = brace; i < page.length; i++) {
        if (page[i] === '{') depth++;
        if (page[i] === '}' && --depth === 0) return page.slice(start, i + 1);
    }
    throw new Error(name);
}
const now = Date.now();
let good = 0, degraded = 0;
const context = {
    Date, localizationReady: true, localizationHealth: 'DEGRADED',
    LOCALIZATION_HEALTH: {GOOD: 'GOOD', DEGRADED: 'DEGRADED', LOST: 'LOST'},
    threeDNavigationMode: 'terrain_3d', threeDRobotBodyStatus: {config: {selected: 'xingji'}},
    threeDRobotBodyGeneration: 1, loadedMapName: 'map', currentMapLoadId: 'load',
    unified3DStatus: {scope: 'xingji_3d', ready: true, transform_model: 'se3_map_odom_v1',
        map_name: 'map', map_load_id: 'load', body_generation: 1},
    unified3DStatusAtMs: now, amclStaleSinceMs: null,
    THREE_D_STATUS_MAX_AGE_MS: 3000,
    latestMotionState() { throw new Error('3D entered duplicate legacy AMCL health gate'); },
    markLocalizationGood() { good++; context.localizationHealth = 'GOOD'; },
    markLocalizationDegraded() { degraded++; context.localizationHealth = 'DEGRADED'; }
};
vm.createContext(context);
for (const name of ['usesUnifiedThreeDLocalization', 'applyUnifiedThreeDHealth']) {
    if (page.includes(`function ${name}(`)) vm.runInContext(namedFunction(name), context);
}
vm.runInContext(namedFunction('monitorLocalizationSample'), context);
assert.equal(context.monitorLocalizationSample({position: 10, yaw: 10}, {stamp: 100}, now), true);
assert.equal(good, 1);
context.unified3DStatus.ready = false;
assert.equal(context.monitorLocalizationSample({}, {stamp: 101}, now), false);
assert.equal(degraded, 1);
context.unified3DStatus.ready = true;
context.unified3DStatus.map_load_id = 'old';
assert.equal(context.monitorLocalizationSample({}, {stamp: 102}, now), false);
context.threeDNavigationMode = 'map_2d';
assert.throws(() => context.monitorLocalizationSample({}, {stamp: 103}, now), /legacy AMCL/);
context.threeDNavigationMode = 'terrain_3d';
context.threeDRobotBodyStatus.config.selected = 'gangben_zsl1';
assert.throws(() => context.monitorLocalizationSample({}, {stamp: 104}, now), /legacy AMCL/);
context.threeDRobotBodyStatus.config.selected = 'xingji';
context.unified3DStatus.map_load_id = 'load';
context.localizationHealth = 'LOST';
assert.equal(context.monitorLocalizationSample({}, {stamp: 105}, now), false,
    'a later ready message cannot clear a previously latched user/localization loss');
console.log('PASS: Xingji3D uses scoped authority; legacy health gate remains isolated');
