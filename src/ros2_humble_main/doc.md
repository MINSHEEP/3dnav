




**##2、编译##**
1.先编译雷达驱动
colcon build --symlink-install --cmake-args   -DROS_EDITION=ROS2   -DHUMBLE_ROS=humble --packages-select livox_ros_driver2
2.然后编译interface
colcon build --packages-select interface lzp_interface
3.然后再编译其他包
 colcon build --symlink-install 


**##3、运行##**
3.1 -- 建图

命令：
ros2 launch car_vel mapping.launch


3.2 -- 保存地图
命令:
ros2 service call /map_save std_srvs/srv/Trigger
保存pcd文件到目录 /home/a/nav/ros2_humble_main/src/lio/small_point_lio/pcd，主要给icp重定位使用。
需要手动拷贝到/home/a/nav/ros2_humble_main/src/navigation/robot_navigation2/pcdmaps，或者改icp的配置文件
icp.yaml在registration下面的、config中

命令:
ros2 run nav2_map_server map_saver_cli -t /projected_map -f /home/a/3dnav/src/ros2_humble_main/src/navigation/robot_navigation2/maps/map
保存pgm文件，map是地图名称，



3.3 -- 导航
命令：

ros2 launch car_vel car_vel.launch.py




ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '/home/a/3dnav/src/ros2_humble_main/src/navigation/robot_navigation2/maps/map.yaml'}"


colcon build --symlink-install --packages-skip gtsam sophus livox_sdk2



ros2 launch car_vel car_vel.launch.py
ros2 launch car_vel launch_manager.launch.py

cd /home/a/3dnav/src/ros2_humble_main/src/car_vel/car_vel/web_frontend
python3 -m http.server 8080 --bind 0.0.0.0

下一步操作步骤：
free -h
cat /sys/class/thermal/thermal_zone0/temp

source /home/a/3dnav/install/setup.bash && \
RCUTILS_COLORIZED_OUTPUT=0 \
ros2 launch car_vel launch_manager.launch.py 2>&1 | \
tee /home/a/nav_debug_full.log



ros2 bag record -o ceshi \
  /livox/lidar \
  /livox/imu \
  /Odometry \
  /tf \
  /tf_static \
  /cmd_vel \
  /lio_health \
  /mapping_cloud_filter/status \
  /terrain_grid/status

ros2 bag record -o ceshi \
  /livox/lidar \
  /livox/imu \
  /tf \
  /tf_static \
  /Odometry \
  /cloud_registered \
  /cloud_mapping_filtered \
  /cloud_mapping_dynamic \
  /global_mapping_cloud \
  /mapping_cloud_filter/status \
  /lio_health \
  /emergency_stop \
  /clear_emergency_stop \
  /rosout \
  /diagnostics
  /scan \
  /map \
  /map_metadata \
  /amcl_pose \
  /particle_cloud \
  /goal_pose \
  /plan \
  /local_plan \
  /cmd_vel_nav \
  /cmd_vel_teleop \
  /cmd_vel_campus \
  /cmd_vel \
  /terrain_grid \
  /terrain_grid/status \
  /terrain_costmap \
  /terrain_observatio

cd voicecontrol/
source .venv/bin/activate
python3 -u main.py


~/cyber1_vc/.venv/bin/python src/ros2_humble_main/src/car_vel/scripts/start_voice_debug.py --voice-project ~/cyber1_vc