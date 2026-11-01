#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');

const frontend = path.resolve(__dirname, '..', 'car_vel', 'web_frontend');
const html = fs.readFileSync(path.join(frontend, 'mapping.html'), 'utf8');
const common = fs.readFileSync(path.join(frontend, 'robot_ui_common.js'), 'utf8');
const viewerPath = path.join(frontend, 'pointcloud_viewer.js');
const healthPath = path.join(frontend, 'mapping_cloud_health.js');

if (!fs.existsSync(viewerPath)) {
  throw new Error('mapping page has no reusable 3D point-cloud viewer');
}
if (!fs.existsSync(healthPath)) {
  throw new Error('mapping page has no session-aware global-cloud health monitor');
}
const viewer = fs.readFileSync(viewerPath, 'utf8');

for (const required of [
  'id="pointcloud-canvas"', 'id="btn-view-3d"', 'id="btn-view-obstacles"',
  'id="btn-view-2d"', "subscribe('/global_mapping_cloud'",
  "subscribe('/octomap_point_cloud_centers'", 'UI.TYPES.pointCloud2',
]) {
  if (!html.includes(required)) {
    throw new Error(`mapping page is missing 3D mapping contract: ${required}`);
  }
}

if (!/pointCloud2\s*:\s*'sensor_msgs\/msg\/PointCloud2'/.test(common)) {
  throw new Error('RobotUi does not expose sensor_msgs/PointCloud2');
}
if (!viewer.includes('DataView') || !viewer.includes('WebGL')
    || !viewer.includes('validatePointCloud(message)')) {
  throw new Error('point-cloud viewer must decode PointCloud2 and render with WebGL');
}
if (!html.includes('全局三维地图') || !html.includes('障碍占据图')
    || !html.includes('二维投影')) {
  throw new Error('mapping page must label 3D point cloud and 2D projection separately');
}
if (!html.includes('GlobalCloudHealthMonitor')
    || !html.includes('未收到有效点云')
    || !html.includes('/home/a/3dnav')) {
  throw new Error('mapping page must diagnose a missing global-cloud backend');
}

console.log('3D mapping point-cloud view and 2D fallback contract: ok');
