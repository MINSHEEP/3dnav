// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__DRIVER_PARAMETERS_HPP_
#define ASTRALL_DRIVER__DRIVER_PARAMETERS_HPP_

#include <cstdint>
#include <string>

namespace astrall_driver {

inline constexpr int64_t kHeartbeatPeriodMs = 100;
inline constexpr int64_t kMaxSdkTimeoutMs = 20;

/**
 * @brief 驱动只读配置；时间为 ms，网络字段只校验 SDK 固定网络约定。
 * 构造后在 configure 中校验，运行时不修改；默认 Fake，不接触真机。
 */
struct DriverParameters {
  bool use_fake_sdk_ = true;
  std::string robot_ip_ = "10.18.10.100";
  std::string local_ip_ = "10.18.10.200";
  int64_t sdk_port_ = 3600;
  int64_t sdk_client_port_ = 3601;
  int64_t init_timeout_ms_ = 1000;
  int64_t sdk_timeout_ms_ = 5;
  int64_t heartbeat_timeout_ms_ = 5;
  int64_t heartbeat_period_ms_ = kHeartbeatPeriodMs;
  int64_t system_poll_period_ms_ = 200;
  int64_t state_publish_period_ms_ = 100;
  int64_t telemetry_publish_period_ms_ = 4;
  int64_t telemetry_query_period_ms_ = 200;
  int64_t telemetry_max_age_ms_ = 300;
  int64_t imu_frequency_hz_ = 250;
  int64_t wheel_speed_frequency_hz_ = 50;
  int64_t joystick_frequency_hz_ = 50;
  bool enable_rgb_ = false;
  bool enable_lidar_ = false;
  std::string imu_topic_ = "imu/data";
  std::string wheel_speed_topic_ = "wheel_speeds";
  std::string joystick_topic_ = "joystick";
  std::string system_status_topic_ = "system_status";
  std::string power_status_topic_ = "power_status";
  std::string sport_status_topic_ = "sport_status";
  std::string device_info_topic_ = "device_info";
  std::string imu_frame_id_ = "base_link";
  std::string base_frame_id_ = "base_link";
  int64_t control_period_ms_ = 20;
  int64_t control_sdk_timeout_ms_ = 2;
  int64_t cmd_vel_timeout_ms_ = 250;
  double max_forward_velocity_ = 1;
  double max_lateral_velocity_ = 1;
  double max_yaw_velocity_ = 1;
  int64_t mode_switch_timeout_ms_ = 3000;
  int64_t mode_state_freshness_ms_ = 250;
  int64_t mode_poll_period_ms_ = 50;
  bool allow_reboot_on_dock_stand_ = false;
  double text_rate_hz_ = 4;
  std::string cmd_vel_topic_ = "cmd_vel";
  std::string lidar_bind_address_ = "0.0.0.0";
  int64_t point_cloud_port_ = 6100;
  int64_t lidar_odometry_port_ = 6101;
  std::string lidar_wire_format_ = "native_le";
  int64_t lidar_max_points_ = 100000;
  int64_t lidar_frame_timeout_ms_ = 100;
  int64_t lidar_max_age_ms_ = 300;
  std::string point_cloud_topic_ = "lidar/points";
  std::string lidar_odometry_topic_ = "lidar/odom";
  std::string odom_frame_id_ = "odom";
  std::string lidar_frame_id_ = "lidar";
  std::string lidar_pose_reference_ = "base";
  double lidar_quaternion_scale_ = 1;
  double lidar_max_position_m_ = 1000000;
  std::string lidar_timestamp_mode_ = "receive";
  int64_t lidar_source_time_tolerance_ms_ = 1000;

  /** @brief 返回空串表示有效；否则解释哪个参数违反边界，不执行网络操作。 */
  std::string Validate() const;
  /** @brief S3 频率、主题与帧配置校验；0Hz 表示取消该传感器订阅。 */
  std::string ValidateTelemetry() const;
  /** @brief 发送/SDK 时间预算、限幅和严格 <5Hz 文本频率校验。 */
  std::string ValidateControl() const;
  /** @brief S5显式线格式、UDP边界、帧与时间假设；即使禁用也校验配置。 */
  std::string ValidateLidar() const;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__DRIVER_PARAMETERS_HPP_
