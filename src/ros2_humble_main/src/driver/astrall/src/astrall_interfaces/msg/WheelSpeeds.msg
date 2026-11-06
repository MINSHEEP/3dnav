std_msgs/Header header
# SDK 时钟单位尚未定义，不用于 ROS 时间同步。
int64 sdk_timestamp
# 保留 SDK 轮速数值与顺序：左前、右前、左后、右后；物理单位待确认。
float32[4] wheel_speed
