/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "lidar_adapter/base_lidar.h"
#include "small_point_lio/odometry_guard_policy.h"
#include "small_point_lio/small_point_lio.h"
#include "util/pointcloud_mapping.h"
#include <nav_msgs/msg/odometry.hpp>
#include <pch.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <cstdint>
#include <mutex>
#include <string>

namespace small_point_lio {

    class SmallPointLioNode : public rclcpp::Node {
    private:
        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::vector<common::Point> pointcloud;
        std::unique_ptr<LidarAdapterBase> lidar_adapter;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subsciber;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> lio_health_publisher;
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> emergency_stop_publisher;
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>>
            clear_emergency_stop_publisher;
        rclcpp::TimerBase::SharedPtr lio_health_timer;
        std::string lio_health_payload;
        std::mutex lio_health_mutex;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
            estimator_reset_trigger;
        std::mutex lio_processing_mutex;
        common::Odometry last_odometry;
        std::unique_ptr<util::PointcloudMapping> pointcloud_mapping;
        std::mutex localization_pcd_save_mutex;
        bool odometry_guard_enabled{true};
        double odometry_guard_max_linear_speed{0.8};
        double odometry_guard_max_vertical_speed{0.8};
        double odometry_guard_max_angular_speed{2.0};
        double odometry_guard_max_position_step{0.15};
        double odometry_guard_max_vertical_step{0.20};
        double odometry_guard_max_yaw_step{0.80};
        double odometry_guard_max_vertical_consistency_error{0.25};
        double odometry_guard_vertical_consistency_time_constant{5.0};
        double odometry_guard_vertical_consistency_recovery_ratio{0.60};
        double odometry_guard_vertical_fault_duration{2.0};
        double odometry_guard_scan_quality_recovery_window{5.0};
        int64_t odometry_guard_warmup_samples{10};
        int64_t odometry_guard_fault_samples{8};
        int64_t odometry_guard_vertical_consistency_warning_samples{10};
        int64_t odometry_guard_vertical_consistency_recovery_samples{20};
        bool odometry_guard_auto_recovery_enabled{true};
        int64_t odometry_guard_auto_recovery_healthy_samples{5};
        bool odometry_guard_auto_recovery_pause_active{false};
        int64_t odometry_guard_auto_recovery_good_samples{0};
        bool odometry_guard_has_reference{false};
        bool odometry_guard_quarantine_active{false};
        bool odometry_guard_sample_warning_active{false};
        bool odometry_guard_vertical_consistency_warning_active{false};
        bool lio_health_announced{false};
        bool lio_fault_latched{false};
        int64_t odometry_guard_accepted_samples{0};
        int64_t odometry_guard_vertical_consistency_warning_count{0};
        int64_t odometry_guard_vertical_consistency_recovery_count{0};
        double odometry_guard_last_timestamp{0.0};
        double odometry_guard_last_x{0.0};
        double odometry_guard_last_y{0.0};
        double odometry_guard_last_z{0.0};
        double odometry_guard_last_yaw{0.0};
        double odometry_guard_last_vertical_velocity{0.0};
        double odometry_guard_vertical_consistency_error{0.0};
        OdometryGuardPolicy odometry_guard_policy{2};
        VerticalOdometryGuardPolicy odometry_guard_vertical_policy{2.0};
        ScanQualityRecoveryPolicy odometry_guard_scan_quality_policy{5.0};

        void publish_lio_health(const std::string &payload);
        void publish_quarantined_sample_warning(
            const std::string &reason,
            int64_t consecutive_samples,
            int64_t required_samples,
            double pose_linear_speed,
            double estimator_linear_speed,
            double pose_angular_speed,
            double estimator_angular_speed,
            double pose_vertical_speed,
            double estimator_vertical_speed,
            double position_z,
            double vertical_consistency_error,
            bool vertical_only,
            double quarantine_duration,
            double required_duration,
            bool has_planar_or_yaw_corroboration,
            std::size_t point_update_attempts,
            std::size_t point_update_accepted,
            double point_update_acceptance_ratio);
        void publish_vertical_consistency_warning(
            double pose_linear_speed,
            double estimator_linear_speed,
            double pose_angular_speed,
            double estimator_angular_speed,
            double pose_vertical_speed,
            double estimator_vertical_speed,
            double position_z,
            double vertical_consistency_error);
        void latch_lio_fault(
            const std::string &reason,
            double pose_linear_speed,
            double estimator_linear_speed,
            double pose_angular_speed,
            double estimator_angular_speed,
            double pose_vertical_speed,
            double estimator_vertical_speed,
            double position_z,
            double vertical_consistency_error,
            const EstimatorDiagnostics &diagnostics);
        void begin_auto_recovery_pause(const std::string &reason);
        bool advance_auto_recovery_validation();
        void reset_estimator_for_recovery(const std::string &reason);

    public:
        explicit SmallPointLioNode(const rclcpp::NodeOptions &options);
    };

}// namespace small_point_lio
