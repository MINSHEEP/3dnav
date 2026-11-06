"""Keep the shared Nav2 configuration; omit reverse recovery for Xuanji only."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from lzp_service.robot_body import BodyConfigStore


def selected_nav2_params(baseline):
    config = BodyConfigStore().load()
    if config['selected'] != 'xuanji':
        return baseline
    parameters = yaml.safe_load(Path(baseline).read_text())
    controller = parameters['controller_server']['ros__parameters']['FollowPath']
    controller.update(motion_model='DiffDrive', vx_min=0.0, vy_max=0.0, vy_std=0.0)
    smoother = parameters['velocity_smoother']['ros__parameters']
    smoother['min_velocity'][0:2] = [0.0, 0.0]
    smoother['max_velocity'][1] = 0.0
    # Suppress tiny MPPI angular corrections so straight motion is reachable
    # with a body that executes forward and in-place rotation separately.
    smoother['deadband_velocity'][2] = 0.05
    directory = Path.home() / '.ros/robot_navigation/xuanji'
    directory.mkdir(parents=True, exist_ok=True)
    navigator = parameters['bt_navigator']['ros__parameters']
    defaults = Path(get_package_share_directory('nav2_bt_navigator')) / 'behavior_trees'
    for kind in ('to_pose', 'through_poses'):
        key = f'default_nav_{kind}_bt_xml'
        source = navigator.get(key) or defaults / f'navigate_{kind}_w_replanning_and_recovery.xml'
        tree = ET.parse(source)
        for parent in tree.iter():
            for child in list(parent):
                if child.tag == 'BackUp':
                    parent.remove(child)
        target = directory / f'navigate_{kind}.xml'
        tree.write(target, encoding='unicode')
        navigator[key] = str(target)
    target = directory / 'nav2_params.yaml'
    target.write_text(yaml.safe_dump(parameters, sort_keys=False))
    return str(target)
