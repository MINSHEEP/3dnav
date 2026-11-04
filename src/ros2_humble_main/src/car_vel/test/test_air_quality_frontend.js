#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const webRoot = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const widgetPath = path.join(webRoot, 'air_quality_widget.js');
const source = fs.readFileSync(widgetPath, 'utf8');

const context = { window: {}, console, setInterval, clearInterval };
vm.createContext(context);
vm.runInContext(source, context, { filename: widgetPath });
const AirQuality = context.window.RobotAirQuality;

if (!AirQuality) throw new Error('air quality widget API is missing');

const parsed = AirQuality.parsePayload(JSON.stringify({
  sensor: 'M701', online: true, valid: true, warming_up: false,
  values: {
    co2_ppm: 482, formaldehyde_ug_m3: 5, tvoc_ug_m3: 36,
    pm25_ug_m3: 45, pm10_ug_m3: 56,
    temperature_c: 30.5, humidity_rh: 64.6,
  },
}));
if (parsed.values.temperature_c !== 30.5 || parsed.values.humidity_rh !== 64.6) {
  throw new Error('air quality payload decoder changed documented values');
}
if (AirQuality.parsePayload('{bad json') !== null) {
  throw new Error('malformed air quality JSON must be rejected');
}

for (const pageName of ['inspection.html', 'remote_control.html']) {
  const html = fs.readFileSync(path.join(webRoot, pageName), 'utf8');
  const scriptStart = html.lastIndexOf('<script>');
  const scriptEnd = html.indexOf('</script>', scriptStart);
  if (scriptStart < 0 || scriptEnd < 0) {
    throw new Error(`${pageName} is missing its inline application script`);
  }
  new Function(html.slice(scriptStart + '<script>'.length, scriptEnd));
  if (!html.includes('air_quality_widget.js')) {
    throw new Error(`${pageName} does not load the shared air quality widget`);
  }
  if (!html.includes('data-air-quality-widget')) {
    throw new Error(`${pageName} has no air quality display container`);
  }
  if (!/RobotAirQuality\.bind\(\s*activeSession/.test(html)) {
    throw new Error(`${pageName} does not subscribe through the ROS session`);
  }
  if (!html.includes('RobotAirQuality.setDisconnected')) {
    throw new Error(`${pageName} does not mark stale values on ROS disconnect`);
  }
}

console.log('M701 shared frontend widget and both page bindings: ok');
