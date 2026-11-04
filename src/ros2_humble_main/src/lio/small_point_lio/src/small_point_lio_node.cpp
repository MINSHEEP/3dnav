/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "io/pcd_io.h"
#include "lidar_adapter/custom_mid360_driver.h"
#include "lidar_adapter/livox_custom_msg.h"
#include "lidar_adapter/livox_pointcloud2.h"
#include "lidar_adapter/unitree_lidar.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace small_point_lio {

    void SmallPointLioNode::publish_lio_health(const std::string &payload) {
        {
            std::lock_guard<std::mutex> lock(lio_health_mutex);
            lio_health_payload = payload;
        }
        if (!lio_health_publisher) {
            return;
        }
        std_msgs::msg::String message;
        message.data = payload;
        lio_health_publisher->publish(message);
    }

    void SmallPointLioNode::publish_vertical_consistency_warning(
        const double pose_linear_speed,
        const double estimator_linear_speed,
        const double pose_angular_speed,
        const double estimator_angular_speed,
        const double pose_vertical_speed,
        const double estimator_vertical_speed,
        const double position_z,
        const double vertical_consistency_error) {
        const auto json_number = [](const double value) {
            if (!std::isfinite(value)) {
                return std::string("null");
            }
            std::ostringstream number;
            number << std::fixed << std::setprecision(3) << value;
            return number.str();
        };
        std::ostringstream status;
        status << "{\"state\":\"warning\","
               << "\"code\":\"vertical_consistency_warning\","
               << "\"reason\":\"vertical_consistency\","
               << "\"hard_stop\":false,"
               << "\"pose_linear_speed\":" << json_number(pose_linear_speed) << ","
               << "\"estimator_linear_speed\":"
               << json_number(estimator_linear_speed) << ","
               << "\"pose_angular_speed\":" << json_number(pose_angular_speed) << ","
               << "\"estimator_angular_speed\":"
               << json_number(estimator_angular_speed) << ","
               << "\"pose_vertical_speed\":" << json_number(pose_vertical_speed) << ","
               << "\"estimator_vertical_speed\":"
               << json_number(estimator_vertical_speed) << ","
               << "\"position_z\":" << json_number(position_z) << ","
               << "\"vertical_consistency_error\":"
               << json_number(vertical_consistency_error)
               << "}";
        publish_lio_health(status.str());
    }

    void SmallPointLioNode::publish_quarantined_sample_warning(
        const std::string &reason,
        const int64_t consecutive_samples,
        const int64_t required_samples,
        const double pose_linear_speed,
        const double estimator_linear_speed,
        const double pose_angular_speed,
        const double estimator_angular_speed,
        const double pose_vertical_speed,
        const double estimator_vertical_speed,
        const double position_z,
        const double vertical_consistency_error,
        const bool vertical_only,
        const double quarantine_duration,
        const double required_duration,
        const bool has_planar_or_yaw_corroboration,
        const std::size_t point_update_attempts,
        const std::size_t point_update_accepted,
        const double point_update_acceptance_ratio) {
        const auto json_number = [](const double value) {
            if (!std::isfinite(value)) {
                return std::string("null");
            }
            std::ostringstream number;
            number << std::fixed << std::setprecision(3) << value;
            return number.str();
        };
        std::ostringstream status;
        status << "{\"state\":\"warning\","
               << "\"code\":\"odometry_sample_quarantined\","
               << "\"reason\":\"" << reason << "\","
               << "\"hard_stop\":false,"
               << "\"sample_quarantined\":true,"
               << "\"temporary_stop\":"
               << (odometry_guard_auto_recovery_pause_active
                   ? "true" : "false") << ","
               << "\"vertical_only\":"
               << (vertical_only ? "true" : "false") << ","
               << "\"recovery_only\":"
               << (reason == "scan_matching_quality" ? "true" : "false")
               << ","
               << "\"quarantine_duration\":"
               << json_number(quarantine_duration) << ","
               << "\"required_duration\":"
               << json_number(required_duration) << ","
               << "\"has_planar_or_yaw_corroboration\":"
               << (has_planar_or_yaw_corroboration ? "true" : "false")
               << ","
               << "\"consecutive_samples\":" << consecutive_samples << ","
               << "\"required_samples\":" << required_samples << ","
               << "\"pose_linear_speed\":" << json_number(pose_linear_speed) << ","
               << "\"estimator_linear_speed\":"
               << json_number(estimator_linear_speed) << ","
               << "\"pose_angular_speed\":" << json_number(pose_angular_speed) << ","
               << "\"estimator_angular_speed\":"
               << json_number(estimator_angular_speed) << ","
               << "\"pose_vertical_speed\":" << json_number(pose_vertical_speed) << ","
               << "\"estimator_vertical_speed\":"
               << json_number(estimator_vertical_speed) << ","
               << "\"position_z\":" << json_number(position_z) << ","
               << "\"vertical_consistency_error\":"
               << json_number(vertical_consistency_error) << ","
               << "\"point_update_attempts\":"
               << point_update_attempts << ","
               << "\"point_update_accepted\":"
               << point_update_accepted << ","
               << "\"point_update_acceptance_ratio\":"
               << json_number(point_update_acceptance_ratio)
               << "}";
        publish_lio_health(status.str());
    }

    void SmallPointLioNode::set_motion_paused(const bool paused) {
        // Serialize heartbeats and transitions so an old false heartbeat
        // cannot be published after a new pause. This never clears a hard stop.
        std::lock_guard<std::mutex> lock(motion_pause_mutex);
        motion_paused = paused;
        if (motion_pause_publisher) {
            std_msgs::msg::Bool message;
            message.data = motion_paused;
            motion_pause_publisher->publish(message);
        }
    }

    void SmallPointLioNode::repeat_motion_pause_state() {
        std::lock_guard<std::mutex> lock(motion_pause_mutex);
        if (motion_pause_publisher) {
            std_msgs::msg::Bool message;
            message.data = motion_paused;
            motion_pause_publisher->publish(message);
        }
    }

    void SmallPointLioNode::begin_auto_recovery_pause(
        const std::string &reason) {
        if (!odometry_guard_auto_recovery_enabled) {
            return;
        }
        odometry_guard_auto_recovery_good_samples = 0;
        if (odometry_guard_auto_recovery_pause_active) {
            return;
        }
        odometry_guard_auto_recovery_pause_active = true;
        set_motion_paused(true);
        if (recoverable_stop_publisher) {
            recoverable_stop_publisher->publish(std_msgs::msg::Empty{});
        }
        RCLCPP_WARN(
            get_logger(),
            "Temporarily stopped robot motion for automatic LIO recovery "
            "(%s). Suspect lidar corrections will be rolled back; motion "
            "resumes only after %ld consecutive healthy scans.",
            reason.c_str(),
            static_cast<long>(
                odometry_guard_auto_recovery_healthy_samples));
    }

    bool SmallPointLioNode::advance_auto_recovery_validation() {
        if (!odometry_guard_auto_recovery_pause_active) {
            return false;
        }
        ++odometry_guard_auto_recovery_good_samples;
        if (odometry_guard_auto_recovery_good_samples <
            odometry_guard_auto_recovery_healthy_samples) {
            std::ostringstream status;
            status << "{\"state\":\"recovering\","
                   << "\"code\":\"lio_auto_recovery\","
                   << "\"healthy_samples\":"
                   << odometry_guard_auto_recovery_good_samples << ","
                   << "\"required_samples\":"
                   << odometry_guard_auto_recovery_healthy_samples << "}";
            publish_lio_health(status.str());
            return false;
        }
        odometry_guard_auto_recovery_pause_active = false;
        odometry_guard_auto_recovery_good_samples = 0;
        if (lio_health_announced) {
            set_motion_paused(false);
        }
        if (recoverable_resume_publisher) {
            recoverable_resume_publisher->publish(std_msgs::msg::Empty{});
        }
        RCLCPP_INFO(
            get_logger(),
            "Automatic LIO recovery succeeded; %ld consecutive scans were "
            "healthy and the temporary motion stop was cleared.",
            static_cast<long>(
                odometry_guard_auto_recovery_healthy_samples));
        return true;
    }

    void SmallPointLioNode::reset_estimator_for_recovery(
        const std::string &reason) {
        small_point_lio->reset();
        last_odometry = common::Odometry{};
        last_odometry.position.setZero();
        last_odometry.velocity.setZero();
        last_odometry.orientation.setIdentity();
        last_odometry.angular_velocity.setZero();
        odometry_guard_policy.reset();
        odometry_guard_vertical_policy.reset();
        odometry_guard_scan_quality_policy.reset();
        odometry_guard_has_reference = false;
        odometry_guard_quarantine_active = false;
        odometry_guard_sample_warning_active = false;
        odometry_guard_vertical_consistency_warning_active = false;
        odometry_guard_accepted_samples = 0;
        odometry_guard_vertical_consistency_warning_count = 0;
        odometry_guard_vertical_consistency_recovery_count = 0;
        odometry_guard_last_timestamp = 0.0;
        odometry_guard_last_x = 0.0;
        odometry_guard_last_y = 0.0;
        odometry_guard_last_z = 0.0;
        odometry_guard_last_yaw = 0.0;
        odometry_guard_last_vertical_velocity = 0.0;
        odometry_guard_vertical_consistency_error = 0.0;
        lio_health_announced = false;
        lio_fault_latched = false;
        odometry_guard_auto_recovery_pause_active = true;
        odometry_guard_auto_recovery_good_samples = 0;
        set_motion_paused(true);
        if (recoverable_stop_publisher) {
            recoverable_stop_publisher->publish(std_msgs::msg::Empty{});
        }
        std::ostringstream status;
        status << "{\"state\":\"resetting\","
               << "\"code\":\"automatic_estimator_reset\","
               << "\"reason\":\"" << reason << "\","
               << "\"temporary_stop\":true}";
        publish_lio_health(status.str());
        RCLCPP_WARN(
            get_logger(),
            "LIO estimator was reset in-process for bounded automatic "
            "recovery (%s). Raw lidar/IMU subscriptions remain active; "
            "motion stays stopped until fresh initialization and healthy "
            "scan validation complete.",
            reason.c_str());
    }

    void SmallPointLioNode::latch_lio_fault(
        const std::string &reason,
        const double pose_linear_speed,
        const double estimator_linear_speed,
        const double pose_angular_speed,
        const double estimator_angular_speed,
        const double pose_vertical_speed,
        const double estimator_vertical_speed,
        const double position_z,
        const double vertical_consistency_error,
        const EstimatorDiagnostics &diagnostics) {
        if (lio_fault_latched) {
            return;
        }
        lio_fault_latched = true;
        set_motion_paused(true);

        const auto json_number = [](const double value) {
            if (!std::isfinite(value)) {
                return std::string("null");
            }
            std::ostringstream number;
            number << std::fixed << std::setprecision(3) << value;
            return number.str();
        };
        std::ostringstream status;
        status << "{\"state\":\"fault\","
               << "\"hard_stop\":false,"
               << "\"code\":\"odometry_diverged\","
               << "\"reason\":\"" << reason << "\","
               << "\"pose_linear_speed\":" << json_number(pose_linear_speed) << ","
               << "\"estimator_linear_speed\":"
               << json_number(estimator_linear_speed) << ","
               << "\"pose_angular_speed\":" << json_number(pose_angular_speed) << ","
               << "\"estimator_angular_speed\":"
               << json_number(estimator_angular_speed) << ","
               << "\"pose_vertical_speed\":" << json_number(pose_vertical_speed) << ","
               << "\"estimator_vertical_speed\":"
               << json_number(estimator_vertical_speed) << ","
               << "\"position_z\":" << json_number(position_z) << ","
               << "\"vertical_consistency_error\":"
               << json_number(vertical_consistency_error) << ","
               << "\"gravity\":[" << json_number(diagnostics.gravity.x()) << ","
               << json_number(diagnostics.gravity.y()) << ","
               << json_number(diagnostics.gravity.z())
               << "],"
               << "\"acceleration\":["
               << json_number(diagnostics.acceleration.x()) << ","
               << json_number(diagnostics.acceleration.y()) << ","
               << json_number(diagnostics.acceleration.z()) << "],"
               << "\"gyroscope_bias\":["
               << json_number(diagnostics.gyroscope_bias.x()) << ","
               << json_number(diagnostics.gyroscope_bias.y()) << ","
               << json_number(diagnostics.gyroscope_bias.z()) << "],"
               << "\"accelerometer_bias\":["
               << json_number(diagnostics.accelerometer_bias.x()) << ","
               << json_number(diagnostics.accelerometer_bias.y()) << ","
               << json_number(diagnostics.accelerometer_bias.z()) << "]"
               << "}";
        publish_lio_health(status.str());

        RCLCPP_ERROR(
            get_logger(),
            "LIO ODOMETRY FAULT LATCHED (%s): pose speed %.3f m/s, "
            "estimator speed %.3f m/s, pose yaw rate %.3f rad/s, "
            "estimator yaw rate %.3f rad/s, z %.3f m, pose vertical speed "
            "%.3f m/s, estimator vertical speed %.3f m/s, accumulated vertical "
            "consistency error %.3f m, gravity "
            "[%.3f %.3f %.3f], acceleration [%.3f %.3f %.3f], gyro bias "
            "[%.3f %.3f %.3f], accel bias [%.3f %.3f %.3f]. /Odometry, TF and "
            "/cloud_registered are now quarantined and /motion_stop was "
            "published: zero velocity with a latched motion gate, no native "
            "hardware emergency. Estimator reset alone does not release the "
            "motion latch; human confirmation is required. Never use the "
            "quarantined outputs for motion.",
            reason.c_str(), pose_linear_speed, estimator_linear_speed,
            pose_angular_speed, estimator_angular_speed, position_z,
            pose_vertical_speed, estimator_vertical_speed,
            vertical_consistency_error,
            diagnostics.gravity.x(), diagnostics.gravity.y(),
            diagnostics.gravity.z(), diagnostics.acceleration.x(),
            diagnostics.acceleration.y(), diagnostics.acceleration.z(),
            diagnostics.gyroscope_bias.x(), diagnostics.gyroscope_bias.y(),
            diagnostics.gyroscope_bias.z(),
            diagnostics.accelerometer_bias.x(),
            diagnostics.accelerometer_bias.y(),
            diagnostics.accelerometer_bias.z());

        if (motion_stop_publisher) {
            motion_stop_publisher->publish(std_msgs::msg::Empty{});
        }
    }

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string lidar_type = declare_parameter<std::string>("lidar_type");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        const bool legacy_save_pcd =
            declare_parameter<bool>("save_pcd", false);
        bool enable_localization_map =
            declare_parameter<bool>("enable_localization_map", legacy_save_pcd);
        const double localization_voxel_size =
            declare_parameter<double>("localization_pcd_voxel_size", 0.10);
        const auto localization_min_observations =
            declare_parameter<int64_t>("localization_pcd_min_observations", 6);
        const auto localization_observation_stride =
            declare_parameter<int64_t>("localization_pcd_observation_stride", 3);
        const auto localization_max_observation_gap_frames =
            declare_parameter<int64_t>(
                "localization_pcd_max_observation_gap_frames", 6);
        const auto localization_min_stable_span_frames =
            declare_parameter<int64_t>(
                "localization_pcd_min_stable_span_frames", 15);
        const std::string localization_filter_mode_value =
            declare_parameter<std::string>(
                "localization_pcd_filter_mode", "protected");
        bool localization_filter_mode_valid = false;
        const auto localization_filter_mode =
            util::pointcloud_mapping_filter_mode_from_string(
                localization_filter_mode_value,
                &localization_filter_mode_valid);
        if (!localization_filter_mode_valid) {
            RCLCPP_WARN(
                get_logger(),
                "Unknown localization_pcd_filter_mode '%s'; using the safe "
                "default 'protected'. Valid modes: protected, bypass, strict.",
                localization_filter_mode_value.c_str());
        }
        const std::string localization_filter_mode_name =
            util::pointcloud_mapping_filter_mode_name(
                localization_filter_mode);
        if (localization_filter_mode_value !=
            localization_filter_mode_name) {
            set_parameter(rclcpp::Parameter(
                "localization_pcd_filter_mode",
                localization_filter_mode_name));
        }
        const bool localization_ground_preservation_enabled =
            declare_parameter<bool>(
                "localization_pcd_ground_preservation_enabled", true);
        const double localization_ground_cell_size =
            declare_parameter<double>(
                "localization_pcd_ground_cell_size", 0.20);
        const double localization_ground_max_height_above_envelope =
            declare_parameter<double>(
                "localization_pcd_ground_max_height_above_envelope", 0.08);
        const double localization_ground_max_slope_deg =
            declare_parameter<double>(
                "localization_pcd_ground_max_slope_deg", 30.0);
        const auto localization_ground_min_neighbor_cells =
            declare_parameter<int64_t>(
                "localization_pcd_ground_min_neighbor_cells", 3);
        const auto localization_ground_min_observations =
            declare_parameter<int64_t>(
                "localization_pcd_ground_min_observations", 3);
        const auto localization_ground_min_stable_span_frames =
            declare_parameter<int64_t>(
                "localization_pcd_ground_min_stable_span_frames", 6);
        const auto localization_stale_frames =
            declare_parameter<int64_t>("localization_pcd_stale_frames", 150);
        const auto localization_prune_interval_frames =
            declare_parameter<int64_t>("localization_pcd_prune_interval_frames", 300);
        const auto localization_max_voxels =
            declare_parameter<int64_t>("localization_pcd_max_voxels", 750000);
        const auto localization_min_output_points =
            declare_parameter<int64_t>("localization_pcd_min_output_points", 2000);
        const std::string localization_staging_directory =
            declare_parameter<std::string>(
                "localization_pcd_staging_directory",
                "/tmp/small_point_lio");
        double global_mapping_cloud_publish_period =
            declare_parameter<double>(
                "global_mapping_cloud_publish_period", 2.0);
        auto global_mapping_cloud_max_points =
            declare_parameter<int64_t>(
                "global_mapping_cloud_max_points", 120000);
        odometry_guard_enabled =
            declare_parameter<bool>("odometry_guard_enabled", true);
        odometry_guard_max_linear_speed =
            declare_parameter<double>(
                "odometry_guard_max_linear_speed", 0.8);
        odometry_guard_max_vertical_speed =
            declare_parameter<double>(
                "odometry_guard_max_vertical_speed", 0.8);
        odometry_guard_max_angular_speed =
            declare_parameter<double>(
                "odometry_guard_max_angular_speed", 2.0);
        odometry_guard_max_position_step =
            declare_parameter<double>(
                "odometry_guard_max_position_step", 0.15);
        odometry_guard_max_vertical_step =
            declare_parameter<double>(
                "odometry_guard_max_vertical_step", 0.20);
        odometry_guard_max_yaw_step =
            declare_parameter<double>(
                "odometry_guard_max_yaw_step", 0.80);
        odometry_guard_max_vertical_consistency_error =
            declare_parameter<double>(
                "odometry_guard_max_vertical_consistency_error", 0.25);
        odometry_guard_vertical_consistency_time_constant =
            declare_parameter<double>(
                "odometry_guard_vertical_consistency_time_constant", 5.0);
        odometry_guard_vertical_consistency_recovery_ratio =
            declare_parameter<double>(
                "odometry_guard_vertical_consistency_recovery_ratio", 0.60);
        odometry_guard_vertical_fault_duration =
            declare_parameter<double>(
                "odometry_guard_vertical_fault_duration", 2.0);
        odometry_guard_scan_quality_recovery_window =
            declare_parameter<double>(
                "odometry_guard_scan_quality_recovery_window", 5.0);
        odometry_guard_warmup_samples =
            declare_parameter<int64_t>(
                "odometry_guard_warmup_samples", 10);
        odometry_guard_fault_samples =
            declare_parameter<int64_t>(
                "odometry_guard_fault_samples", 8);
        odometry_guard_vertical_consistency_warning_samples =
            declare_parameter<int64_t>(
                "odometry_guard_vertical_consistency_warning_samples", 10);
        odometry_guard_vertical_consistency_recovery_samples =
            declare_parameter<int64_t>(
                "odometry_guard_vertical_consistency_recovery_samples", 20);
        odometry_guard_auto_recovery_enabled =
            declare_parameter<bool>(
                "odometry_guard_auto_recovery_enabled", true);
        odometry_guard_auto_recovery_healthy_samples =
            declare_parameter<int64_t>(
                "odometry_guard_auto_recovery_healthy_samples", 5);

        if (localization_voxel_size < 0.05 ||
            localization_voxel_size > 1.0 ||
            localization_min_observations < 1 ||
            localization_observation_stride < 1 ||
            localization_max_observation_gap_frames <
                localization_observation_stride ||
            localization_min_stable_span_frames < 1 ||
            !std::isfinite(localization_ground_cell_size) ||
            localization_ground_cell_size < localization_voxel_size ||
            localization_ground_cell_size > 1.0 ||
            !std::isfinite(
                localization_ground_max_height_above_envelope) ||
            localization_ground_max_height_above_envelope < 0.01 ||
            localization_ground_max_height_above_envelope > 0.15 ||
            !std::isfinite(localization_ground_max_slope_deg) ||
            localization_ground_max_slope_deg < 1.0 ||
            localization_ground_max_slope_deg > 45.0 ||
            localization_ground_min_neighbor_cells < 1 ||
            localization_ground_min_neighbor_cells > 8 ||
            localization_ground_min_observations < 2 ||
            localization_ground_min_observations >=
                localization_min_observations ||
            localization_ground_min_stable_span_frames <
                localization_observation_stride ||
            localization_ground_min_stable_span_frames >=
                localization_min_stable_span_frames ||
            localization_stale_frames < 1 ||
            localization_prune_interval_frames < 1 ||
            localization_max_voxels < 1000 ||
            localization_min_output_points < 100) {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid localization PCD filter parameters; localization-map "
                "accumulation has been disabled. Check voxel size, observation "
                "counts, frame intervals, point threshold, and voxel capacity.");
            enable_localization_map = false;
        }
        if (!std::isfinite(global_mapping_cloud_publish_period) ||
            global_mapping_cloud_publish_period < 0.5 ||
            global_mapping_cloud_publish_period > 30.0 ||
            global_mapping_cloud_max_points < 1000 ||
            global_mapping_cloud_max_points > 500000) {
            RCLCPP_WARN(
                get_logger(),
                "Invalid global mapping cloud preview limits; using 2.0 s "
                "and 120000 points.");
            global_mapping_cloud_publish_period = 2.0;
            global_mapping_cloud_max_points = 120000;
        }
        if (!std::isfinite(odometry_guard_max_linear_speed) ||
            !std::isfinite(odometry_guard_max_vertical_speed) ||
            !std::isfinite(odometry_guard_max_angular_speed) ||
            !std::isfinite(odometry_guard_max_position_step) ||
            !std::isfinite(odometry_guard_max_vertical_step) ||
            !std::isfinite(odometry_guard_max_yaw_step) ||
            !std::isfinite(odometry_guard_max_vertical_consistency_error) ||
            !std::isfinite(
                odometry_guard_vertical_consistency_time_constant) ||
            !std::isfinite(
                odometry_guard_vertical_consistency_recovery_ratio) ||
            !std::isfinite(odometry_guard_vertical_fault_duration) ||
            !std::isfinite(odometry_guard_scan_quality_recovery_window) ||
            odometry_guard_max_linear_speed <= 0.0 ||
            odometry_guard_max_vertical_speed <= 0.0 ||
            odometry_guard_max_angular_speed <= 0.0 ||
            odometry_guard_max_position_step <= 0.0 ||
            odometry_guard_max_vertical_step <= 0.0 ||
            odometry_guard_max_yaw_step <= 0.0 ||
            odometry_guard_max_vertical_consistency_error <= 0.0 ||
            odometry_guard_vertical_consistency_time_constant <= 0.0 ||
            odometry_guard_vertical_consistency_recovery_ratio <= 0.0 ||
            odometry_guard_vertical_consistency_recovery_ratio >= 1.0 ||
            odometry_guard_vertical_fault_duration < 0.5 ||
            odometry_guard_scan_quality_recovery_window < 2.0 ||
            odometry_guard_warmup_samples < 0 ||
            odometry_guard_fault_samples < 1 ||
            odometry_guard_auto_recovery_healthy_samples < 1 ||
            odometry_guard_vertical_consistency_warning_samples < 1 ||
            odometry_guard_vertical_consistency_recovery_samples < 1) {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid odometry guard parameters; safe defaults will be "
                "used.");
            odometry_guard_max_linear_speed = 0.8;
            odometry_guard_max_vertical_speed = 0.8;
            odometry_guard_max_angular_speed = 2.0;
            odometry_guard_max_position_step = 0.15;
            odometry_guard_max_vertical_step = 0.20;
            odometry_guard_max_yaw_step = 0.80;
            odometry_guard_max_vertical_consistency_error = 0.25;
            odometry_guard_vertical_consistency_time_constant = 5.0;
            odometry_guard_vertical_consistency_recovery_ratio = 0.60;
            odometry_guard_vertical_fault_duration = 2.0;
            odometry_guard_scan_quality_recovery_window = 5.0;
            odometry_guard_warmup_samples = 10;
            odometry_guard_fault_samples = 8;
            odometry_guard_auto_recovery_healthy_samples = 5;
            odometry_guard_vertical_consistency_warning_samples = 10;
            odometry_guard_vertical_consistency_recovery_samples = 20;
        }
        odometry_guard_policy.set_required_samples(
            odometry_guard_fault_samples);
        odometry_guard_vertical_policy.set_required_duration(
            odometry_guard_vertical_fault_duration);
        odometry_guard_scan_quality_policy.set_diagnostic_window(
            odometry_guard_scan_quality_recovery_window);
        RCLCPP_INFO(
            get_logger(),
            "Quadruped odometry guard configured: a kinematic outlier is "
            "quarantined before publication; planar/yaw outliers require %ld "
            "consecutive samples. Scan-quality-only failures remain "
            "recoverable and never latch a fault by themselves (%.1f s "
            "diagnostic window). Vertical-only gait transients never latch a "
            "fault by themselves; vertical motion must persist for %.1f s and "
            "have independent planar/yaw evidence. Numerical and "
            "estimator-health failures stop immediately.",
            static_cast<long>(odometry_guard_fault_samples),
            odometry_guard_scan_quality_recovery_window,
            odometry_guard_vertical_fault_duration);
        if (legacy_save_pcd != enable_localization_map) {
            RCLCPP_INFO(
                get_logger(),
                "Parameter 'save_pcd' is deprecated; "
                "'enable_localization_map' controls PCD accumulation.");
        }
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
        global_mapping_cloud_publisher =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "/global_mapping_cloud",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        lio_health_publisher = create_publisher<std_msgs::msg::String>(
            "/lio_health",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        motion_stop_publisher = create_publisher<std_msgs::msg::Empty>(
            "/motion_stop", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        clear_emergency_stop_publisher =
            create_publisher<std_msgs::msg::Empty>(
                "/clear_emergency_stop",
                rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        recoverable_stop_publisher = create_publisher<std_msgs::msg::Empty>(
            "/lio/recoverable_stop", rclcpp::QoS(10).reliable());
        recoverable_resume_publisher = create_publisher<std_msgs::msg::Empty>(
            "/lio/recoverable_resume", rclcpp::QoS(10).reliable());
        motion_pause_publisher = create_publisher<std_msgs::msg::Bool>(
            "/lio/motion_paused",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        set_motion_paused(true);
        publish_lio_health("{\"state\":\"initializing\"}");
        // rosbridge commonly creates a VOLATILE subscription, so it will not
        // receive a transient-local sample published before the browser
        // connects.  Repeat the tiny status payload at 1 Hz for late clients.
        lio_health_timer = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                repeat_motion_pause_state();
                std::string payload;
                {
                    std::lock_guard<std::mutex> lock(lio_health_mutex);
                    payload = lio_health_payload;
                }
                if (payload.empty() || !lio_health_publisher) {
                    return;
                }
                std_msgs::msg::String message;
                message.data = payload;
                lio_health_publisher->publish(message);
            });
        estimator_reset_trigger = create_service<std_srvs::srv::Trigger>(
            "/lio/reset_estimator",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response) {
                std::lock_guard<std::mutex> lock(lio_processing_mutex);
                reset_estimator_for_recovery("external_bounded_recovery");
                response->success = true;
                response->message =
                    "LIO estimator reset; waiting for fresh healthy scans";
            });
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        if (enable_localization_map) {
            const util::PointcloudMappingGroundPolicy ground_policy{
                localization_ground_preservation_enabled,
                static_cast<float>(localization_ground_cell_size),
                static_cast<float>(
                    localization_ground_max_height_above_envelope),
                static_cast<float>(localization_ground_max_slope_deg),
                static_cast<std::uint32_t>(
                    localization_ground_min_neighbor_cells),
                static_cast<std::uint32_t>(
                    localization_ground_min_observations),
                static_cast<std::uint64_t>(
                    localization_ground_min_stable_span_frames),
                localization_filter_mode};
            pointcloud_mapping = std::make_unique<util::PointcloudMapping>(
                static_cast<float>(localization_voxel_size),
                static_cast<std::uint32_t>(localization_min_observations),
                static_cast<std::uint32_t>(localization_observation_stride),
                static_cast<std::uint64_t>(
                    localization_max_observation_gap_frames),
                static_cast<std::uint64_t>(
                    localization_min_stable_span_frames),
                static_cast<std::uint64_t>(localization_stale_frames),
                static_cast<std::uint64_t>(localization_prune_interval_frames),
                static_cast<std::size_t>(localization_max_voxels),
                ground_policy);
            RCLCPP_INFO(
                get_logger(),
                "Localization PCD accumulation enabled: voxel=%.3f m, "
                "min_observations=%ld, stride=%ld, max_gap=%ld frames, "
                "min_span=%ld frames, max_voxels=%ld; mode=%s, ground=%s, "
                "%ld observations/%ld frames, cell=%.2f m.",
                localization_voxel_size,
                static_cast<long>(localization_min_observations),
                static_cast<long>(localization_observation_stride),
                static_cast<long>(localization_max_observation_gap_frames),
                static_cast<long>(localization_min_stable_span_frames),
                static_cast<long>(localization_max_voxels),
                localization_filter_mode_name.c_str(),
                localization_ground_preservation_enabled ? "enabled" : "disabled",
                static_cast<long>(localization_ground_min_observations),
                static_cast<long>(localization_ground_min_stable_span_frames),
                localization_ground_cell_size);
            global_mapping_cloud_timer = create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(
                        global_mapping_cloud_publish_period)),
                [this, global_mapping_cloud_max_points]() {
                    if (!pointcloud_mapping || !global_mapping_cloud_publisher ||
                        global_mapping_cloud_publisher->get_subscription_count() == 0) {
                        return;
                    }
                    const auto snapshot = pointcloud_mapping->get_snapshot(
                        static_cast<std::size_t>(
                            global_mapping_cloud_max_points));
                    if (snapshot.points.empty()) {
                        return;
                    }
                    sensor_msgs::msg::PointCloud2 message;
                    message.header.stamp = get_clock()->now();
                    message.header.frame_id = "odom";
                    message.height = 1;
                    message.width = static_cast<std::uint32_t>(
                        snapshot.points.size());
                    message.fields.resize(3);
                    const std::array<std::string, 3> names{"x", "y", "z"};
                    for (std::size_t index = 0; index < names.size(); ++index) {
                        message.fields[index].name = names[index];
                        message.fields[index].offset =
                            static_cast<std::uint32_t>(index * sizeof(float));
                        message.fields[index].datatype =
                            sensor_msgs::msg::PointField::FLOAT32;
                        message.fields[index].count = 1;
                    }
                    message.is_bigendian = false;
                    message.point_step = 3 * sizeof(float);
                    message.row_step = message.width * message.point_step;
                    message.is_dense = true;
                    message.data.resize(message.row_step);
                    auto *output = reinterpret_cast<float *>(message.data.data());
                    for (const auto &point : snapshot.points) {
                        *output++ = point.x();
                        *output++ = point.y();
                        *output++ = point.z();
                    }
                    global_mapping_cloud_publisher->publish(message);
                });
            RCLCPP_INFO(
                get_logger(),
                "Global mapping cloud preview enabled on "
                "/global_mapping_cloud: period=%.1f s, max_points=%ld. "
                "The saved PCD remains complete.",
                global_mapping_cloud_publish_period,
                static_cast<long>(global_mapping_cloud_max_points));
        } else {
            RCLCPP_INFO(
                get_logger(),
                "Localization PCD accumulation is disabled (navigation mode).");
        }
        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "/lio/save_localization_pcd",
                [this,
                 enable_localization_map,
                 localization_voxel_size,
                 localization_min_observations,
                 localization_observation_stride,
                 localization_ground_preservation_enabled,
                 localization_ground_cell_size,
                 localization_ground_max_height_above_envelope,
                 localization_ground_max_slope_deg,
                 localization_ground_min_neighbor_cells,
                 localization_ground_min_observations,
                 localization_ground_min_stable_span_frames,
                 localization_min_output_points,
                 localization_staging_directory](
                    const std_srvs::srv::Trigger::Request::SharedPtr req,
                    std_srvs::srv::Trigger::Response::SharedPtr res) {
                    (void) req;
                    std::lock_guard<std::mutex> save_lock(
                        localization_pcd_save_mutex);
                    if (!enable_localization_map || !pointcloud_mapping) {
                        res->success = false;
                        res->message =
                            "Localization PCD save is disabled. Start the LIO "
                            "with enable_localization_map:=true (mapping mode).";
                        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
                        return;
                    }

                    const auto snapshot = pointcloud_mapping->get_snapshot();
                    if (snapshot.capacity_exceeded) {
                        std::ostringstream message;
                        message
                            << "Localization PCD accumulation exceeded its "
                            << "configured voxel capacity and dropped "
                            << snapshot.dropped_voxels
                            << " voxels. Save aborted to avoid an incomplete "
                            << "localization map; increase voxel size or capacity.";
                        res->success = false;
                        res->message = message.str();
                        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
                        return;
                    }
                    if (snapshot.points.size() <
                        static_cast<std::size_t>(localization_min_output_points)) {
                        std::ostringstream message;
                        message
                            << "Only " << snapshot.points.size()
                            << " persistent localization points are available; "
                            << localization_min_output_points
                            << " are required. Continue mapping so static "
                            << "surfaces are observed repeatedly.";
                        res->success = false;
                        res->message = message.str();
                        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
                        return;
                    }

                    try {
                        const std::filesystem::path staging_directory(
                            localization_staging_directory);
                        std::filesystem::create_directories(staging_directory);
                        const auto capture_unix_ns =
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::system_clock::now()
                                    .time_since_epoch())
                                .count();
                        const std::string stem =
                            "localization-map-" +
                            std::to_string(capture_unix_ns);
                        const auto temporary_path =
                            staging_directory / (stem + ".pcd.tmp");
                        const auto final_path =
                            staging_directory / (stem + ".pcd");

                        std::error_code remove_error;
                        std::filesystem::remove(temporary_path, remove_error);
                        std::filesystem::remove(final_path, remove_error);
                        if (!io::pcd::write_pcd(
                                temporary_path.string(), snapshot.points)) {
                            std::filesystem::remove(
                                temporary_path, remove_error);
                            res->success = false;
                            res->message =
                                "Failed to write the filtered localization PCD.";
                            RCLCPP_ERROR(
                                get_logger(), "%s", res->message.c_str());
                            return;
                        }
                        if (!std::filesystem::exists(temporary_path) ||
                            std::filesystem::file_size(temporary_path) == 0) {
                            std::filesystem::remove(
                                temporary_path, remove_error);
                            res->success = false;
                            res->message =
                                "Filtered localization PCD was empty after writing.";
                            RCLCPP_ERROR(
                                get_logger(), "%s", res->message.c_str());
                            return;
                        }
                        std::filesystem::rename(temporary_path, final_path);

                        std::ostringstream result;
                        result
                            << "{"
                            << "\"schema\":\"small_point_lio_localization_snapshot\","
                            << "\"version\":1,"
                            << "\"path\":\"" << final_path.string() << "\","
                            << "\"capture_frame\":\"odom\","
                            << "\"point_count\":" << snapshot.points.size() << ","
                            << "\"voxel_size\":" << localization_voxel_size << ","
                            << "\"min_observations\":"
                            << localization_min_observations << ","
                            << "\"observation_stride\":"
                            << localization_observation_stride << ","
                            << "\"filter_mode\":\""
                            << util::pointcloud_mapping_filter_mode_name(
                                snapshot.filter_mode) << "\","
                            << "\"ground_preservation_enabled\":"
                            << (localization_ground_preservation_enabled
                                    ? "true" : "false") << ","
                            << "\"ground_cell_size\":"
                            << localization_ground_cell_size << ","
                            << "\"ground_max_height_above_envelope\":"
                            << localization_ground_max_height_above_envelope << ","
                            << "\"ground_max_slope_deg\":"
                            << localization_ground_max_slope_deg << ","
                            << "\"ground_min_neighbor_cells\":"
                            << localization_ground_min_neighbor_cells << ","
                            << "\"ground_min_observations\":"
                            << localization_ground_min_observations << ","
                            << "\"ground_min_stable_span_frames\":"
                            << localization_ground_min_stable_span_frames << ","
                            << "\"ground_protected_voxels\":"
                            << snapshot.ground_protected_voxels << ","
                            << "\"received_frames\":"
                            << snapshot.received_frames << ","
                            << "\"sampled_frames\":"
                            << snapshot.sampled_frames << ","
                            << "\"tracked_voxels\":"
                            << snapshot.tracked_voxels << ","
                            << "\"capture_unix_ns\":" << capture_unix_ns
                            << "}";
                        res->success = true;
                        res->message = result.str();
                        RCLCPP_INFO(
                            get_logger(),
                            "Saved filtered localization snapshot: %zu points, "
                            "%zu tracked voxels, path=%s",
                            snapshot.points.size(),
                            snapshot.tracked_voxels,
                            final_path.c_str());
                    } catch (const std::exception &error) {
                        res->success = false;
                        res->message =
                            std::string("Localization PCD save failed: ") +
                            error.what();
                        RCLCPP_ERROR(
                            get_logger(), "%s", res->message.c_str());
                    }
                });
        // small_point_lio->set_odometry_callback([this, lidar_frame](const common::Odometry &odometry) {
        //     last_odometry = odometry;

        //     builtin_interfaces::msg::Time time_msg;
        //     time_msg.sec = std::floor(odometry.timestamp);
        //     time_msg.nanosec = static_cast<uint32_t>((odometry.timestamp - time_msg.sec) * 1e9);

        //     // Estimator 输出的是 odom->IMU (即 odom->base_link)，直接广播 odom->base_link
        //     geometry_msgs::msg::TransformStamped transform_stamped;
        //     transform_stamped.header.stamp = time_msg;
        //     transform_stamped.header.frame_id = "odom";
        //     transform_stamped.child_frame_id = "base_link";
        //     transform_stamped.transform.translation.x = odometry.position.x();
        //     transform_stamped.transform.translation.y = odometry.position.y();
        //     transform_stamped.transform.translation.z = odometry.position.z();
        //     transform_stamped.transform.rotation.x = odometry.orientation.x();
        //     transform_stamped.transform.rotation.y = odometry.orientation.y();
        //     transform_stamped.transform.rotation.z = odometry.orientation.z();
        //     transform_stamped.transform.rotation.w = odometry.orientation.w();
        //     // only publish odom->base_link here; base_link->livox_frame should be provided statically

        //     nav_msgs::msg::Odometry odometry_msg;
        //     odometry_msg.header.stamp = time_msg;
        //     odometry_msg.header.frame_id = "odom";
        //     odometry_msg.child_frame_id = "base_link";
        //     odometry_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
        //     odometry_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
        //     odometry_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
        //     odometry_msg.pose.pose.orientation.x = transform_stamped.transform.rotation.x;
        //     odometry_msg.pose.pose.orientation.y = transform_stamped.transform.rotation.y;
        //     odometry_msg.pose.pose.orientation.z = transform_stamped.transform.rotation.z;
        //     odometry_msg.pose.pose.orientation.w = transform_stamped.transform.rotation.w;

        //     // 填充速度/角速度
        //     // linear velocity: estimator 的速度为 odom-frame（见 eskf.predict_state 中的使用），
        //     // Odometry.twist 应以 child_frame (base_link) 表示，因此将 odom-frame 的速度旋转到 base_link。
        //         // Publish linear velocity as provided by the estimator (in odom frame).
            
        //     Eigen::Vector3d vel_odom = odometry.velocity;
        //     odometry_msg.twist.twist.linear.x = vel_odom.x();
        //     odometry_msg.twist.twist.linear.y = vel_odom.y();
        //     odometry_msg.twist.twist.linear.z = vel_odom.z();


        //     // Publish linear velocity as provided by the estimator (in odom frame).
        //     // Eigen::Vector3d vel_odom = odometry.velocity;

        //     // // 转换到 base_link 坐标系
        //     // Eigen::Quaterniond q_odom_to_base = odometry.orientation.inverse();
        //     // Eigen::Vector3d vel_base = q_odom_to_base * vel_odom;

        //     // odometry_msg.twist.twist.linear.x = vel_base.x();
        //     // odometry_msg.twist.twist.linear.y = vel_base.y();
        //     // odometry_msg.twist.twist.linear.z = vel_base.z();

        //     // angular velocity in the estimator state (`kf.x.omg`) is maintained in the body frame,
        //     // so publish angular velocity directly (expressed in base_link/body frame).
        //     odometry_msg.twist.twist.angular.x = odometry.angular_velocity.x();
        //     odometry_msg.twist.twist.angular.y = odometry.angular_velocity.y();
        //     odometry_msg.twist.twist.angular.z = odometry.angular_velocity.z();

        //     tf_broadcaster->sendTransform(transform_stamped);
        //     odometry_publisher->publish(odometry_msg);
        // });


        small_point_lio->set_odometry_callback([this, lidar_frame](const common::Odometry &odometry) {
            if (lio_fault_latched) {
                return OdometryDecision::Fault;
            }

            builtin_interfaces::msg::Time time_msg;
            time_msg.sec = std::floor(odometry.timestamp);
            time_msg.nanosec = static_cast<uint32_t>((odometry.timestamp - time_msg.sec) * 1e9);

            // robot_state_publisher only owns the fixed base_link -> lidar
            // transform.  Look that transform up without depending on the
            // dynamic base_footprint branch that this callback publishes.
            geometry_msgs::msg::TransformStamped lidar_from_base_link_msg;
            try {
                // lookupTransform(target, source) returns T_target_source.
                lidar_from_base_link_msg =
                    tf_buffer->lookupTransform(lidar_frame, "base_link", time_msg);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("small_point_lio"),
                    "Failed to look up T_%s_base_link: %s", lidar_frame.c_str(), ex.what());
                // The filter has already consumed this batch. Keep its state
                // and discard only the public outputs/map points so a
                // transient TF lookup failure cannot create an IMU time gap.
                return OdometryDecision::Quarantine;
            }

            // tf2 multiplication follows T_a_c = T_a_b * T_b_c.  The LIO
            // state is T_odom_imu and get_lidar_*_wrt_IMU supplies
            // T_imu_lidar, so appending the fixed T_lidar_base_link gives the
            // complete physical six-DoF body pose T_odom_base_link.
            tf2::Transform tf_odom_imu;
            tf_odom_imu.setOrigin(tf2::Vector3(
                odometry.position.x(), odometry.position.y(), odometry.position.z()));
            tf_odom_imu.setRotation(tf2::Quaternion(
                odometry.orientation.x(), odometry.orientation.y(),
                odometry.orientation.z(), odometry.orientation.w()));

            const auto imu_from_lidar_translation =
                small_point_lio->get_lidar_T_wrt_IMU();
            const Eigen::Quaterniond imu_from_lidar_rotation(
                small_point_lio->get_lidar_R_wrt_IMU());
            tf2::Transform tf_imu_lidar;
            tf_imu_lidar.setOrigin(tf2::Vector3(
                imu_from_lidar_translation.x(), imu_from_lidar_translation.y(),
                imu_from_lidar_translation.z()));
            tf_imu_lidar.setRotation(tf2::Quaternion(
                imu_from_lidar_rotation.x(), imu_from_lidar_rotation.y(),
                imu_from_lidar_rotation.z(), imu_from_lidar_rotation.w()));

            tf2::Transform tf_lidar_base_link;
            tf2::fromMsg(lidar_from_base_link_msg.transform, tf_lidar_base_link);
            const tf2::Transform tf_odom_base_link =
                tf_odom_imu * tf_imu_lidar * tf_lidar_base_link;

            // Nav2 and AMCL need a strictly planar robot frame.  Project the
            // complete body pose to x/y/yaw and publish the discarded
            // z/roll/pitch (including the coupled translation) as the exact
            // residual T_base_footprint_base_link.  Therefore:
            //   T_odom_base_footprint * T_base_footprint_base_link
            //     == T_odom_base_link
            const tf2::Matrix3x3 body_basis_odom =
                tf_odom_base_link.getBasis();
            const double body_yaw = std::atan2(
                body_basis_odom[1][0], body_basis_odom[0][0]);

            tf2::Quaternion planar_rotation;
            planar_rotation.setRPY(0.0, 0.0, body_yaw);
            planar_rotation.normalize();

            tf2::Transform tf_odom_base_footprint;
            tf_odom_base_footprint.setOrigin(tf2::Vector3(
                tf_odom_base_link.getOrigin().x(),
                tf_odom_base_link.getOrigin().y(), 0.0));
            tf_odom_base_footprint.setRotation(planar_rotation);

            const tf2::Transform tf_base_footprint_base_link =
                tf_odom_base_footprint.inverse() * tf_odom_base_link;

            geometry_msgs::msg::TransformStamped footprint_transform;
            footprint_transform.header.stamp = time_msg;
            footprint_transform.header.frame_id = "odom";
            footprint_transform.child_frame_id = "base_footprint";
            footprint_transform.transform = tf2::toMsg(tf_odom_base_footprint);

            geometry_msgs::msg::TransformStamped body_residual_transform;
            body_residual_transform.header.stamp = time_msg;
            body_residual_transform.header.frame_id = "base_footprint";
            body_residual_transform.child_frame_id = "base_link";
            body_residual_transform.transform =
                tf2::toMsg(tf_base_footprint_base_link);

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = time_msg;
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = "base_footprint";
            odometry_msg.pose.pose.position.x =
                footprint_transform.transform.translation.x;
            odometry_msg.pose.pose.position.y =
                footprint_transform.transform.translation.y;
            odometry_msg.pose.pose.position.z = 0.0;
            odometry_msg.pose.pose.orientation =
                footprint_transform.transform.rotation;

            // nav_msgs/Odometry requires twist to be expressed in child_frame_id
            // (base_footprint here).  First compensate the IMU-to-base_link
            // lever arm.  The planar origin follows base_link in x/y while its
            // z is fixed at zero, so publish x/y in the yaw-only frame and no
            // vertical velocity.  Angular z is the derivative of the projected
            // yaw, not the body's raw z-axis gyro component.
            const tf2::Transform tf_imu_base_link =
                tf_imu_lidar * tf_lidar_base_link;

            const tf2::Vector3 linear_imu_origin_odom(
                odometry.velocity.x(), odometry.velocity.y(), odometry.velocity.z());
            const tf2::Vector3 angular_imu(
                odometry.angular_velocity.x(), odometry.angular_velocity.y(),
                odometry.angular_velocity.z());

            const tf2::Vector3 angular_odom =
                tf_odom_imu.getBasis() * angular_imu;
            const tf2::Vector3 imu_to_base_odom =
                tf_odom_imu.getBasis() * tf_imu_base_link.getOrigin();
            const tf2::Vector3 linear_base_origin_odom =
                linear_imu_origin_odom + angular_odom.cross(imu_to_base_odom);

            const tf2::Matrix3x3 footprint_from_odom =
                tf_odom_base_footprint.getBasis().transpose();
            const tf2::Vector3 linear_footprint = footprint_from_odom * tf2::Vector3(
                linear_base_origin_odom.x(), linear_base_origin_odom.y(), 0.0);

            // For yaw = atan2(R(1,0), R(0,0)), differentiate the projected
            // body x-axis using R_dot = [omega_odom]x R.  This remains exact
            // with non-zero roll/pitch (away from the Euler singularity).
            const tf2::Vector3 body_x_axis_odom =
                body_basis_odom.getColumn(0);
            const double projected_x_norm_sq =
                body_x_axis_odom.x() * body_x_axis_odom.x() +
                body_x_axis_odom.y() * body_x_axis_odom.y();
            double yaw_rate = angular_odom.z();
            if (projected_x_norm_sq > 1.0e-8) {
                const double body_x_dot =
                    angular_odom.y() * body_x_axis_odom.z() -
                    angular_odom.z() * body_x_axis_odom.y();
                const double body_y_dot =
                    angular_odom.z() * body_x_axis_odom.x() -
                    angular_odom.x() * body_x_axis_odom.z();
                yaw_rate =
                    (body_x_axis_odom.x() * body_y_dot -
                     body_x_axis_odom.y() * body_x_dot) /
                    projected_x_norm_sq;
            }

            const double candidate_x =
                footprint_transform.transform.translation.x;
            const double candidate_y =
                footprint_transform.transform.translation.y;
            const double candidate_z =
                tf_odom_base_link.getOrigin().z();
            const double estimator_linear_speed =
                std::hypot(linear_footprint.x(), linear_footprint.y());
            const double estimator_vertical_velocity =
                linear_base_origin_odom.z();
            const double estimator_vertical_speed =
                std::abs(estimator_vertical_velocity);
            const double estimator_angular_speed = std::abs(yaw_rate);
            double pose_linear_speed = 0.0;
            double pose_vertical_speed = 0.0;
            double pose_angular_speed = 0.0;
            double position_step = 0.0;
            double vertical_step = 0.0;
            double yaw_step = 0.0;
            double candidate_vertical_consistency_error = 0.0;
            double dt = 0.0;
            std::string guard_reason;
            bool guard_requires_immediate_fault = false;
            bool guard_is_vertical_motion = false;
            bool guard_has_planar_or_yaw_corroboration = false;
            const EstimatorDiagnostics diagnostics =
                small_point_lio->get_estimator_diagnostics();

            const char *estimator_health_error =
                small_point_lio->estimator_health_error();
            const bool candidate_finite =
                std::isfinite(odometry.timestamp) &&
                odometry.position.allFinite() &&
                odometry.velocity.allFinite() &&
                odometry.orientation.coeffs().allFinite() &&
                odometry.angular_velocity.allFinite() &&
                std::isfinite(candidate_x) &&
                std::isfinite(candidate_y) &&
                std::isfinite(candidate_z) &&
                std::isfinite(body_yaw) &&
                std::isfinite(estimator_linear_speed) &&
                std::isfinite(estimator_vertical_velocity) &&
                std::isfinite(estimator_vertical_speed) &&
                std::isfinite(estimator_angular_speed);
            if (estimator_health_error != nullptr) {
                guard_reason = estimator_health_error;
                guard_requires_immediate_fault = true;
            } else if (!candidate_finite) {
                guard_reason = "non_finite_state";
                guard_requires_immediate_fault = true;
            } else if (odometry_guard_has_reference) {
                dt = odometry.timestamp - odometry_guard_last_timestamp;
                if (!(dt > 0.0)) {
                    guard_reason = "non_monotonic_timestamp";
                    guard_requires_immediate_fault = true;
                } else {
                    const double vertical_displacement =
                        candidate_z - odometry_guard_last_z;
                    position_step = std::hypot(
                        candidate_x - odometry_guard_last_x,
                        candidate_y - odometry_guard_last_y);
                    vertical_step = std::abs(vertical_displacement);
                    yaw_step = std::abs(std::atan2(
                        std::sin(body_yaw - odometry_guard_last_yaw),
                        std::cos(body_yaw - odometry_guard_last_yaw)));
                    pose_linear_speed = position_step / dt;
                    pose_vertical_speed = vertical_step / dt;
                    pose_angular_speed = yaw_step / dt;
                    if (dt <= 0.5) {
                        const double decay = std::exp(
                            -dt /
                            odometry_guard_vertical_consistency_time_constant);
                        const double estimator_vertical_displacement =
                            0.5 *
                            (odometry_guard_last_vertical_velocity +
                             estimator_vertical_velocity) *
                            dt;
                        candidate_vertical_consistency_error =
                            odometry_guard_vertical_consistency_error * decay +
                            vertical_displacement -
                            estimator_vertical_displacement;
                    }
                }
            }

            const bool guard_is_warm =
                odometry_guard_accepted_samples >=
                odometry_guard_warmup_samples;
            if (guard_reason.empty() && odometry_guard_enabled &&
                guard_is_warm && odometry_guard_has_reference) {
                if (!odometry.scan_matching_healthy) {
                    guard_reason = "scan_matching_quality";
                }
                const bool estimator_vertical_violation =
                    estimator_vertical_speed >
                    odometry_guard_max_vertical_speed;
                const bool pose_vertical_violation =
                    pose_vertical_speed >
                    odometry_guard_max_vertical_speed;
                const bool vertical_step_violation =
                    dt <= 0.5 &&
                    vertical_step > odometry_guard_max_vertical_step;
                const bool estimator_linear_violation =
                    estimator_linear_speed >
                    odometry_guard_max_linear_speed;
                const bool pose_linear_violation =
                    pose_linear_speed >
                    odometry_guard_max_linear_speed;
                const bool estimator_angular_violation =
                    estimator_angular_speed >
                    odometry_guard_max_angular_speed;
                const bool pose_angular_violation =
                    pose_angular_speed >
                    odometry_guard_max_angular_speed;
                const bool position_step_violation =
                    dt <= 0.5 &&
                    position_step > odometry_guard_max_position_step;
                const bool yaw_step_violation =
                    dt <= 0.5 &&
                    yaw_step > odometry_guard_max_yaw_step;
                guard_has_planar_or_yaw_corroboration =
                    estimator_linear_violation ||
                    pose_linear_violation ||
                    estimator_angular_violation ||
                    pose_angular_violation ||
                    position_step_violation ||
                    yaw_step_violation;

                if (!guard_reason.empty()) {
                    // A matching-quality failure is already selected above.
                } else if (estimator_vertical_violation) {
                    guard_reason = "estimator_vertical_speed";
                    guard_is_vertical_motion = true;
                } else if (pose_vertical_violation) {
                    guard_reason = "pose_vertical_speed";
                    guard_is_vertical_motion = true;
                } else if (vertical_step_violation) {
                    guard_reason = "vertical_step";
                    guard_is_vertical_motion = true;
                } else if (estimator_linear_violation) {
                    guard_reason = "estimator_linear_speed";
                } else if (pose_linear_violation) {
                    guard_reason = "pose_linear_speed";
                } else if (estimator_angular_violation) {
                    guard_reason = "estimator_angular_speed";
                } else if (pose_angular_violation) {
                    guard_reason = "pose_angular_speed";
                } else if (position_step_violation) {
                    guard_reason = "position_step";
                } else if (yaw_step_violation) {
                    guard_reason = "yaw_step";
                }
            }

            if (!guard_reason.empty()) {
                odometry_guard_quarantine_active = true;
                OdometryDecision decision = OdometryDecision::Quarantine;
                int64_t consecutive_samples = 0;
                int64_t required_samples = 0;
                double quarantine_duration = 0.0;
                double required_duration = 0.0;
                const bool guard_is_scan_quality =
                    guard_reason == "scan_matching_quality";
                if (guard_is_scan_quality &&
                    !guard_requires_immediate_fault) {
                    odometry_guard_policy.reset();
                    odometry_guard_vertical_policy.reset();
                    decision = odometry_guard_scan_quality_policy.observe(
                        odometry.timestamp);
                    consecutive_samples =
                        odometry_guard_scan_quality_policy.sample_count();
                    quarantine_duration =
                        odometry_guard_scan_quality_policy.duration_seconds();
                    required_duration =
                        odometry_guard_scan_quality_policy
                            .required_duration_seconds();
                    RCLCPP_WARN(
                        get_logger(),
                        "Quarantined recoverable scan-matching sample %ld "
                        "(%.3f/%.3f s, independent planar/yaw evidence: %s): "
                        "%zu/%zu point constraints accepted (ratio %.3f). "
                        "Scan quality alone cannot latch a permanent fault; "
                        "the current navigation task is retained while "
                        "healthy scans are verified.",
                        static_cast<long>(consecutive_samples),
                        quarantine_duration, required_duration,
                        guard_has_planar_or_yaw_corroboration ? "yes" : "no",
                        odometry.point_update_accepted,
                        odometry.point_update_attempts,
                        odometry.point_update_acceptance_ratio);
                } else if (guard_is_vertical_motion &&
                    !guard_requires_immediate_fault) {
                    odometry_guard_policy.reset();
                    odometry_guard_scan_quality_policy.reset();
                    decision = odometry_guard_vertical_policy.observe(
                        odometry.timestamp,
                        guard_has_planar_or_yaw_corroboration);
                    consecutive_samples =
                        odometry_guard_vertical_policy.sample_count();
                    quarantine_duration =
                        odometry_guard_vertical_policy.duration_seconds();
                    required_duration =
                        odometry_guard_vertical_policy
                            .required_duration_seconds();
                    RCLCPP_WARN(
                        get_logger(),
                        "Quarantined quadruped vertical-motion transient "
                        "sample %ld (%.3f/%.3f s, independent planar/yaw "
                        "evidence: %s, %s): pose speed %.3f m/s, estimator "
                        "speed %.3f m/s, pose yaw rate %.3f rad/s, estimator "
                        "yaw rate %.3f rad/s, z %.3f m, pose vertical speed "
                        "%.3f m/s, estimator vertical speed %.3f m/s, "
                        "accumulated vertical consistency error %.3f m, "
                        "dt %.3f s. Vertical-only evidence cannot latch an "
                        "emergency stop.",
                        static_cast<long>(consecutive_samples),
                        quarantine_duration, required_duration,
                        guard_has_planar_or_yaw_corroboration ? "yes" : "no",
                        guard_reason.c_str(), pose_linear_speed,
                        estimator_linear_speed, pose_angular_speed,
                        estimator_angular_speed, candidate_z,
                        pose_vertical_speed, estimator_vertical_speed,
                        candidate_vertical_consistency_error, dt);
                } else {
                    odometry_guard_vertical_policy.reset();
                    odometry_guard_scan_quality_policy.reset();
                    decision = odometry_guard_policy.observe(
                        guard_reason, guard_requires_immediate_fault);
                    consecutive_samples =
                        odometry_guard_policy.consecutive_samples();
                    required_samples =
                        odometry_guard_policy.required_samples();
                    RCLCPP_WARN(
                        get_logger(),
                        "Quarantined suspect LIO odometry sample %ld/%ld "
                        "(%s): pose speed %.3f m/s, estimator speed %.3f m/s, "
                        "pose yaw rate %.3f rad/s, estimator yaw rate %.3f "
                        "rad/s, z %.3f m, pose vertical speed %.3f m/s, "
                        "estimator vertical speed %.3f m/s, accumulated "
                        "vertical consistency error %.3f m, dt %.3f s.",
                        static_cast<long>(consecutive_samples),
                        static_cast<long>(
                            guard_requires_immediate_fault ? 1 :
                            required_samples),
                        guard_reason.c_str(), pose_linear_speed,
                        estimator_linear_speed, pose_angular_speed,
                        estimator_angular_speed, candidate_z,
                        pose_vertical_speed, estimator_vertical_speed,
                        candidate_vertical_consistency_error, dt);
                }
                if (decision == OdometryDecision::Fault) {
                    const std::string fault_reason =
                        guard_is_vertical_motion
                        ? "persistent_vertical_with_planar_corroboration"
                        : guard_reason;
                    latch_lio_fault(
                        fault_reason, pose_linear_speed,
                        estimator_linear_speed, pose_angular_speed,
                        estimator_angular_speed, pose_vertical_speed,
                        estimator_vertical_speed, candidate_z,
                        candidate_vertical_consistency_error, diagnostics);
                    return OdometryDecision::Fault;
                }
                if (!guard_is_vertical_motion &&
                    !guard_requires_immediate_fault) {
                    begin_auto_recovery_pause(guard_reason);
                }
                odometry_guard_sample_warning_active = true;
                publish_quarantined_sample_warning(
                    guard_reason, consecutive_samples, required_samples,
                    pose_linear_speed, estimator_linear_speed,
                    pose_angular_speed, estimator_angular_speed,
                    pose_vertical_speed, estimator_vertical_speed,
                    candidate_z, candidate_vertical_consistency_error,
                    guard_is_vertical_motion, quarantine_duration,
                    required_duration,
                    guard_has_planar_or_yaw_corroboration,
                    odometry.point_update_attempts,
                    odometry.point_update_accepted,
                    odometry.point_update_acceptance_ratio);
                return OdometryDecision::Quarantine;
            }

            const bool recovered_from_sample_quarantine =
                odometry_guard_sample_warning_active ||
                odometry_guard_policy.consecutive_samples() > 0 ||
                odometry_guard_scan_quality_policy.active();
            odometry_guard_policy.reset();
            odometry_guard_vertical_policy.reset();
            odometry_guard_scan_quality_policy.reset();
            odometry_guard_sample_warning_active = false;

            const bool can_track_vertical_consistency =
                odometry_guard_enabled && guard_is_warm &&
                odometry_guard_has_reference &&
                dt > 0.0 && dt <= 0.5;
            if (can_track_vertical_consistency) {
                const double vertical_consistency_magnitude =
                    std::abs(candidate_vertical_consistency_error);
                if (!odometry_guard_vertical_consistency_warning_active) {
                    odometry_guard_vertical_consistency_recovery_count = 0;
                    if (vertical_consistency_magnitude >
                        odometry_guard_max_vertical_consistency_error) {
                        ++odometry_guard_vertical_consistency_warning_count;
                    } else {
                        odometry_guard_vertical_consistency_warning_count = 0;
                    }
                    if (odometry_guard_vertical_consistency_warning_count >=
                        odometry_guard_vertical_consistency_warning_samples) {
                        odometry_guard_vertical_consistency_warning_active = true;
                        odometry_guard_vertical_consistency_recovery_count = 0;
                        publish_vertical_consistency_warning(
                            pose_linear_speed, estimator_linear_speed,
                            pose_angular_speed, estimator_angular_speed,
                            pose_vertical_speed, estimator_vertical_speed,
                            candidate_z,
                            candidate_vertical_consistency_error);
                        RCLCPP_WARN(
                            get_logger(),
                            "LIO vertical-consistency warning: the accumulated "
                            "pose/velocity mismatch remained above %.3f m for "
                            "%ld accepted samples (current %.3f m). This is a "
                            "motion-model diagnostic only; odometry and mapping "
                            "continue. Instantaneous divergence guards remain "
                            "active.",
                            odometry_guard_max_vertical_consistency_error,
                            static_cast<long>(
                                odometry_guard_vertical_consistency_warning_count),
                            candidate_vertical_consistency_error);
                    }
                } else {
                    const double recovery_threshold =
                        odometry_guard_max_vertical_consistency_error *
                        odometry_guard_vertical_consistency_recovery_ratio;
                    if (vertical_consistency_magnitude <= recovery_threshold) {
                        ++odometry_guard_vertical_consistency_recovery_count;
                    } else {
                        odometry_guard_vertical_consistency_recovery_count = 0;
                    }
                    if (odometry_guard_vertical_consistency_recovery_count >=
                        odometry_guard_vertical_consistency_recovery_samples) {
                        odometry_guard_vertical_consistency_warning_active = false;
                        odometry_guard_vertical_consistency_warning_count = 0;
                        odometry_guard_vertical_consistency_recovery_count = 0;
                        publish_lio_health("{\"state\":\"healthy\"}");
                        RCLCPP_INFO(
                            get_logger(),
                            "LIO vertical-consistency warning recovered after "
                            "%ld accepted samples below %.3f m.",
                            static_cast<long>(
                                odometry_guard_vertical_consistency_recovery_samples),
                            recovery_threshold);
                    }
                }
            } else {
                odometry_guard_vertical_consistency_warning_count = 0;
                odometry_guard_vertical_consistency_recovery_count = 0;
            }

            odometry_guard_quarantine_active = false;
            const bool automatic_recovery_completed =
                advance_auto_recovery_validation();
            if (recovered_from_sample_quarantine ||
                automatic_recovery_completed) {
                if (odometry_guard_auto_recovery_pause_active) {
                    RCLCPP_INFO(
                        get_logger(),
                        "LIO produced a healthy recovery scan (%ld/%ld); "
                        "motion remains temporarily stopped.",
                        static_cast<long>(
                            odometry_guard_auto_recovery_good_samples),
                        static_cast<long>(
                            odometry_guard_auto_recovery_healthy_samples));
                } else if (
                    odometry_guard_vertical_consistency_warning_active) {
                    publish_vertical_consistency_warning(
                        pose_linear_speed, estimator_linear_speed,
                        pose_angular_speed, estimator_angular_speed,
                        pose_vertical_speed, estimator_vertical_speed,
                        candidate_z,
                        candidate_vertical_consistency_error);
                } else {
                    publish_lio_health("{\"state\":\"healthy\"}");
                }
                if (!odometry_guard_auto_recovery_pause_active) {
                    RCLCPP_INFO(
                        get_logger(),
                        "LIO odometry recovered; public odometry, TF and "
                        "registered point clouds resumed.");
                }
            }
            odometry_guard_has_reference = true;
            ++odometry_guard_accepted_samples;
            odometry_guard_last_timestamp = odometry.timestamp;
            odometry_guard_last_x = candidate_x;
            odometry_guard_last_y = candidate_y;
            odometry_guard_last_z = candidate_z;
            odometry_guard_last_yaw = body_yaw;
            odometry_guard_last_vertical_velocity =
                estimator_vertical_velocity;
            if (odometry_guard_enabled && guard_is_warm &&
                dt > 0.0 && dt <= 0.5) {
                odometry_guard_vertical_consistency_error =
                    candidate_vertical_consistency_error;
            } else {
                odometry_guard_vertical_consistency_error = 0.0;
            }
            last_odometry = odometry;

            if (!lio_health_announced &&
                odometry_guard_accepted_samples >=
                odometry_guard_warmup_samples) {
                lio_health_announced = true;
                if (!odometry_guard_auto_recovery_pause_active) {
                    set_motion_paused(false);
                }
                publish_lio_health("{\"state\":\"healthy\"}");
                RCLCPP_INFO(
                    get_logger(),
                    "LIO odometry is initialized and the divergence guard is "
                    "active after %ld accepted samples.",
                    static_cast<long>(odometry_guard_accepted_samples));
            }

            odometry_msg.twist.twist.linear.x = linear_footprint.x();
            odometry_msg.twist.twist.linear.y = linear_footprint.y();
            odometry_msg.twist.twist.linear.z = 0.0;
            odometry_msg.twist.twist.angular.x = 0.0;
            odometry_msg.twist.twist.angular.y = 0.0;
            odometry_msg.twist.twist.angular.z = yaw_rate;

            tf_broadcaster->sendTransform(
                std::vector<geometry_msgs::msg::TransformStamped>{
                    footprint_transform, body_residual_transform});
            odometry_publisher->publish(odometry_msg);
            return OdometryDecision::Accept;
        });

        small_point_lio->set_pointcloud_callback(
            [this, enable_localization_map](
                const std::vector<Eigen::Vector3f> &pointcloud) {
            if (lio_fault_latched || odometry_guard_quarantine_active) {
                return;
            }
        //     if (pointcloud_publisher->get_subscription_count() > 0) {
        //         builtin_interfaces::msg::Time time_msg;
        //         time_msg.sec = std::floor(last_odometry.timestamp);
        //         time_msg.nanosec = static_cast<uint32_t>((last_odometry.timestamp - time_msg.sec) * 1e9);

        //         // 点云已由 estimator 转换为 odom（见 small_point_lio::handle_once 中的 pointcloud_odom_frame），直接发布即可
        //         Eigen::Matrix3f lidar_frame_to_odom_R = Eigen::Matrix3f::Identity();
        //         Eigen::Vector3f lidar_frame_to_odom_T = Eigen::Vector3f::Zero();
        //         sensor_msgs::msg::PointCloud2 msg;
        //         msg.header.stamp = time_msg;
        //         msg.header.frame_id = "odom";
        //         msg.width = pointcloud.size();
        //         msg.height = 1;
        //         msg.fields.reserve(4);
        //         sensor_msgs::msg::PointField field;
        //         field.name = "x";
        //         field.offset = 0;
        //         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        //         field.count = 1;
        //         msg.fields.push_back(field);
        //         field.name = "y";
        //         field.offset = 4;
        //         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        //         field.count = 1;
        //         msg.fields.push_back(field);
        //         field.name = "z";
        //         field.offset = 8;
        //         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        //         field.count = 1;
        //         msg.fields.push_back(field);
        //         field.name = "intensity";
        //         field.offset = 12;
        //         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        //         field.count = 1;
        //         msg.fields.push_back(field);
        //         msg.is_bigendian = false;
        //         msg.point_step = 16;
        //         msg.row_step = msg.width * msg.point_step;
        //         msg.data.resize(msg.row_step * msg.height);
        //         Eigen::Vector3f transformed_point;
        //         auto pointer = reinterpret_cast<float *>(msg.data.data());
        //         for (const auto &point: pointcloud) {
        //             // 这里接收到的 pointcloud 应已是 odom-frame 的点（由 SmallPointLio 提供），直接使用
        //             transformed_point = point;
        //             *pointer = transformed_point.x();
        //             ++pointer;
        //             *pointer = transformed_point.y();
        //             ++pointer;
        //             *pointer = transformed_point.z();
        //             ++pointer;
        //             *pointer = 0;
        //             ++pointer;
        //         }
        //         msg.is_dense = false;
        //         pointcloud_publisher->publish(msg);
        //     }

            if (pointcloud_publisher->get_subscription_count() > 0) {
                builtin_interfaces::msg::Time time_msg;
                time_msg.sec = std::floor(last_odometry.timestamp);
                time_msg.nanosec = static_cast<uint32_t>((last_odometry.timestamp - time_msg.sec) * 1e9);

                sensor_msgs::msg::PointCloud2 msg;
                msg.header.stamp = time_msg;
                msg.header.frame_id = "odom";
                msg.width = pointcloud.size();
                msg.height = 1;
                msg.fields.reserve(4);
                sensor_msgs::msg::PointField field;
                field.name = "x";
                field.offset = 0;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "y";
                field.offset = 4;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "z";
                field.offset = 8;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "intensity";
                field.offset = 12;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                msg.is_bigendian = false;
                msg.point_step = 16;
                msg.row_step = msg.width * msg.point_step;
                msg.data.resize(msg.row_step * msg.height);
                auto pointer = reinterpret_cast<float *>(msg.data.data());
                for (const auto &point: pointcloud) {
                    // The core LIO callback already supplies points in the odom frame.
                    // Applying the lidar-to-base extrinsic here would shift them a second time.
                    *pointer = point.x();
                    ++pointer;
                    *pointer = point.y();
                    ++pointer;
                    *pointer = point.z();
                    ++pointer;
                    *pointer = 0;
                    ++pointer;
                }
                msg.is_dense = false;
                pointcloud_publisher->publish(msg);
            }


            if (enable_localization_map && pointcloud_mapping) {
                // The callback supplies one registered scan in the odom frame.
                // The accumulator counts at most one observation per voxel per
                // sampled scan, so dense returns in a single scan cannot make a
                // transient object look persistent.
                pointcloud_mapping->add_pointcloud(pointcloud);
            }
        });
        if (lidar_type == "livox_custom_msg") {
#ifdef HAVE_LIVOX_DRIVER
            lidar_adapter = std::make_unique<LivoxCustomMsgAdapter>();
#else
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "livox_custom_msg requested but not available!");
            rclcpp::shutdown();
            return;
#endif
        } else if (lidar_type == "livox_pointcloud2") {
            lidar_adapter = std::make_unique<LivoxPointCloud2Adapter>();
        } else if (lidar_type == "custom_mid360_driver") {
            lidar_adapter = std::make_unique<CustomMid360DriverAdapter>();
        } else if (lidar_type == "unilidar") {
            lidar_adapter = std::make_unique<UnilidarAdapter>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknwon lidar type");
            rclcpp::shutdown();
            return;
        }
        lidar_adapter->setup_subscription(this, lidar_topic, [this](const std::vector<common::Point> &pointcloud) {
            std::lock_guard<std::mutex> lock(lio_processing_mutex);
            small_point_lio->on_point_cloud_callback(pointcloud);
            small_point_lio->handle_once();
        });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    std::lock_guard<std::mutex> lock(lio_processing_mutex);
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)

