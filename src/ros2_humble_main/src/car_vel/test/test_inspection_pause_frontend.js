#!/usr/bin/env node
'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const html = fs.readFileSync(path.resolve(
  __dirname, '..', 'car_vel', 'web_frontend', 'inspection.html'), 'utf8');
const script = html.slice(html.lastIndexOf('<script>') + '<script>'.length,
  html.indexOf('</script>', html.lastIndexOf('<script>')));
new Function(script);

const match = script.match(/function pauseControlState\(state\) \{[\s\S]*?\n    \}/);
assert(match, 'pause control state helper is missing');
const pauseControlState = new Function(`return (${match[0]});`)();

assert.deepStrictEqual(pauseControlState({state: 'navigating', active: true, paused: false}),
  {paused: false, canPause: true, canResume: false});
assert.deepStrictEqual(pauseControlState({state: 'pausing', active: true, paused: true}),
  {paused: true, canPause: false, canResume: false});
assert.deepStrictEqual(pauseControlState({state: 'paused', active: true, paused: true}),
  {paused: true, canPause: false, canResume: true});
assert.deepStrictEqual(pauseControlState({state: 'cancelling', active: true, paused: true}),
  {paused: true, canPause: false, canResume: false});
assert.deepStrictEqual(pauseControlState({state: 'paused', active: false, paused: true}),
  {paused: true, canPause: false, canResume: false});

assert(/'btn-resume-task'\)\.disabled[\s\S]*?!pauseControls\.canResume/.test(script),
  'resume button must wait for acknowledged paused state');
assert(/command === 'resume' && !pauseControls\.canResume/.test(script),
  'direct resume command must wait for acknowledged paused state');
assert(/pausing: '正在暂停'/.test(script),
  'intermediate pausing state needs a distinct visible label');

console.log('inspection pause acknowledgement frontend: ok');
