'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const frontend = path.join(__dirname, '..', 'car_vel', 'web_frontend');

function page(name) {
  return fs.readFileSync(path.join(frontend, name), 'utf8');
}

function compileInlineScripts(source, name) {
  const scripts = [...source.matchAll(/<script(?![^>]*\bsrc=)[^>]*>([\s\S]*?)<\/script>/gi)];
  assert(scripts.length > 0, `${name} must contain an inline controller`);
  scripts.forEach((match) => new Function(match[1])); // eslint-disable-line no-new-func
}

const settings = page('settings.html');
const inspection = page('inspection.html');
compileInlineScripts(settings, 'settings.html');
compileInlineScripts(inspection, 'inspection.html');

assert(settings.includes("'/navigation/set_mode'"));
assert(settings.includes('UI.TYPES.setNavigationMode'));
assert(settings.includes('navigationModeStatus.locked'));
assert(settings.includes("'/navigation/set_inflation_radius'"));
assert(settings.includes('UI.TYPES.setInflationRadius'));
assert(settings.includes('inflationRadiusStatus.locked'));
assert(inspection.includes("'/navigation/global_path'"));
assert(inspection.includes('mapCanvas.setGlobalPath('));
assert(inspection.includes('mapCanvas.clearGlobalPath()'));

console.log('navigation mode settings and inspection path frontend: ok');
