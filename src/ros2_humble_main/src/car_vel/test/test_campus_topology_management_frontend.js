const fs = require('fs');
const path = require('path');

const frontend = path.join(__dirname, '..', 'car_vel', 'web_frontend');
const html = fs.readFileSync(path.join(frontend, 'campus_navigation.html'), 'utf8');
const common = fs.readFileSync(path.join(frontend, 'robot_ui_common.js'), 'utf8');

for (const token of [
  'id="btn-edit-topology"', 'id="topology-editor"',
  'id="area-editor"', 'id="transition-editor"',
  'id="capability-profile-editor"', 'id="btn-save-topology"',
  'id="btn-capture-source-pose"', 'id="btn-capture-target-pose"',
  "call('/manage_campus_topology'", 'expected_revision',
  "call('/get_map_list'", 'id="map-name-options"',
  "call('/switch_map'", "subscribe('/current_map_info'",
  "call('/control_launch'", "launch_type: 'car_vel'",
]) {
  if (!html.includes(token)) throw new Error(`missing campus management contract: ${token}`);
}

for (const token of [
  "subscribe('/terrain_observation'", "subscribe('/terrain_costmap/status'",
  "subscribe('/Odometry'", "subscribe('/localization/navigation_pose'",
  '实时地形', '任务执行中禁止修改',
]) {
  if (!html.includes(token)) throw new Error(`missing live safety contract: ${token}`);
}

if (!common.includes("manageCampusTopology: 'lzp_interface/srv/ManageCampusTopology'")) {
  throw new Error('ManageCampusTopology type must be exported to the frontend');
}

console.log('campus topology CRUD, pose capture and live safety frontend contract: ok');
