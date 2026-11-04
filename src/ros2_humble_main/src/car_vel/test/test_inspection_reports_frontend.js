#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const reportPath = path.join(frontend, 'inspection_reports.html');
const reportHtml = fs.readFileSync(reportPath, 'utf8');
const commonJs = fs.readFileSync(
  path.join(frontend, 'robot_ui_common.js'), 'utf8');

const scriptStart = reportHtml.lastIndexOf('<script>');
const scriptEnd = reportHtml.indexOf('</script>', scriptStart);
if (scriptStart < 0 || scriptEnd < 0) {
  throw new Error('inspection_reports.html is missing its inline script');
}
new Function(reportHtml.slice(scriptStart + '<script>'.length, scriptEnd));

const ids = [...reportHtml.matchAll(/\bid="([^"]+)"/g)]
  .map(match => match[1]);
const duplicates = ids.filter((id, index) => ids.indexOf(id) !== index);
if (duplicates.length) {
  throw new Error(`duplicate report page ids: ${[...new Set(duplicates)].join(', ')}`);
}

for (const contract of [
  ['queryInspectionReports', '/inspection/query_reports'],
  ['getInspectionReport', '/inspection/get_report'],
  ['reviewInspectionReport', '/inspection/review_report'],
  ['acknowledgeInspectionAlert', '/inspection/acknowledge_alert'],
]) {
  if (!new RegExp(`\\b${contract[0]}\\s*:`).test(commonJs)) {
    throw new Error(`missing RobotUi type ${contract[0]}`);
  }
  if (!reportHtml.includes(contract[1])) {
    throw new Error(`report page is missing ${contract[1]}`);
  }
}

if (/邮件|email/i.test(reportHtml)) {
  throw new Error('email functionality must not appear in the current report page');
}

for (const pageName of [
  'mapping.html', 'map_editor.html', 'navigation.html', 'inspection.html',
  'inspection_reports.html', 'remote_control.html', 'camera.html',
]) {
  const page = fs.readFileSync(path.join(frontend, pageName), 'utf8');
  if (!page.includes('href="inspection_reports.html"')) {
    throw new Error(`${pageName} is missing the inspection reports navigation link`);
  }
  if (!page.includes('<div class="brand">四足机器人巡检</div>')) {
    throw new Error(`${pageName} is missing the unified brand`);
  }
}

const inspectionHtml = fs.readFileSync(
  path.join(frontend, 'inspection.html'), 'utf8');
if (!inspectionHtml.includes(
  'inspection_reports.html?task_id=${encodeURIComponent(item.task_id)}')) {
  throw new Error('inspection history does not link to the full report');
}

const interfaceCmake = fs.readFileSync(path.resolve(
  __dirname, '..', '..', 'nav_service', 'lzp_interface', 'CMakeLists.txt'), 'utf8');
for (const service of [
  'QueryInspectionReports.srv',
  'GetInspectionReport.srv',
  'ReviewInspectionReport.srv',
]) {
  if (!interfaceCmake.includes(`"srv/${service}"`)) {
    throw new Error(`${service} is missing from rosidl_generate_interfaces`);
  }
}

console.log('inspection report frontend syntax, contracts and navigation: ok');

// Execute the actual renderer helpers, including rule evidence with null score.
const measurementStart = reportHtml.indexOf('function alertMeasurementLabel(');
const measurementEnd = reportHtml.indexOf('function renderPoints(', measurementStart);
const helpers = new Function(`${reportHtml.slice(measurementStart, measurementEnd)}; return {alertMeasurementLabel, alertCategoryLabel};`)();
const fallAlert = {category:'fall', measurement_type:'rule_evidence', confidence:null};
if (helpers.alertMeasurementLabel(fallAlert).includes('%') ||
    !helpers.alertCategoryLabel(fallAlert).includes('疑似')) {
  throw new Error('fall rule evidence must not become a made-up probability');
}
if (!helpers.alertCategoryLabel({category:'fight'}).includes('场景级')) {
  throw new Error('fight evidence must retain its scene scope');
}
console.log('behavior report typed evidence: ok');
