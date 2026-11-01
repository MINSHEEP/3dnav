#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const html = fs.readFileSync(path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'mapping.html'), 'utf8');

const required = [
  'mapping_cloud_health.js?v=',
  'new window.MappingCloudHealth.GlobalCloudHealthMonitor',
  'initialTimeoutMs: 45000',
  'staleTimeoutMs: 15000',
  "globalCloudHealth.setConnected(connected)",
  'resetGlobalCloudSession()',
  "globalCloudHealth.start()",
  "globalCloudHealth.stop()",
  "if (renderCloud(message, '全局三维地图', 'global'))",
  ': pointCloudViewer.validatePointCloud(message)',
  'globalCloudHealth.markValidFrame()',
  "event.state === 'missing'",
  "event.state === 'stale'",
  '网页已请求订阅',
  '/home/a/3dnav',
  'Global mapping cloud preview enabled',
];

for (const contract of required) {
  if (!html.includes(contract)) {
    throw new Error(`missing global-cloud health contract: ${contract}`);
  }
}

const handler = html.indexOf('function handleGlobalMappingCloud(message)');
const validation = html.indexOf(
  "if (renderCloud(message, '全局三维地图', 'global'))", handler);
const confirm = html.indexOf('globalCloudHealth.markValidFrame();', validation);
const callback = html.indexOf(
  "activeSession.subscribe('/global_mapping_cloud'");
const callbackUsesHandler = html.indexOf('handleGlobalMappingCloud', callback);
if (handler < 0 || validation < handler || confirm < validation
    || callback < 0 || callbackUsesHandler < callback) {
  throw new Error('only a successfully decoded global cloud may confirm health');
}

if (html.includes('latestPointCloudLabel')) {
  throw new Error('global and obstacle point-count labels must remain independent');
}

console.log('Mapping global-cloud health integration contract: ok');
