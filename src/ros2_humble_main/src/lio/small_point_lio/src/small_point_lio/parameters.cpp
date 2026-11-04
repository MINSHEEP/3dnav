/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "parameters.h"

#include <cmath>

namespace small_point_lio {

    void Parameters::read_parameters(rclcpp::Node &node) {
        // 点云过滤
        point_filter_num = static_cast<int>(node.declare_parameter<long>("point_filter_num"));
        auto min_distance = node.declare_parameter<float>("min_distance");
        auto max_distance = node.declare_parameter<float>("max_distance");
        auto registered_cloud_max_distance =
                node.declare_parameter<float>(
                        "registered_cloud_max_distance", max_distance);
        if (!std::isfinite(registered_cloud_max_distance) ||
            registered_cloud_max_distance <= 0.0f) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "registered_cloud_max_distance must be finite and positive; "
                    "using max_distance %.2f m",
                    max_distance);
            registered_cloud_max_distance = max_distance;
        }
        if (registered_cloud_max_distance > max_distance) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "registered_cloud_max_distance %.2f m exceeds max_distance "
                    "%.2f m; clamping it to the LIO input range",
                    registered_cloud_max_distance, max_distance);
            registered_cloud_max_distance = max_distance;
        }
        min_distance_squared = min_distance * min_distance;
        max_distance_squared = max_distance * max_distance;
        registered_cloud_max_distance_squared =
                registered_cloud_max_distance * registered_cloud_max_distance;

        // Remove returns from the robot itself. The box is expressed in the
        // lidar frame because this filter runs before the lidar/IMU extrinsic
        // transform is applied.
        self_filter_enabled = node.declare_parameter<bool>("self_filter_enabled", false);
        const std::vector<double> default_min{-0.75, -0.35, -0.65};
        const std::vector<double> default_max{0.12, 0.35, 0.10};
        auto self_filter_min = node.declare_parameter<std::vector<double>>(
                "self_filter_min_xyz", default_min);
        auto self_filter_max = node.declare_parameter<std::vector<double>>(
                "self_filter_max_xyz", default_max);

        if (self_filter_min.size() != 3) {
            RCLCPP_WARN(node.get_logger(),
                        "self_filter_min_xyz must contain exactly 3 values; using the default box minimum");
            self_filter_min = default_min;
        }
        if (self_filter_max.size() != 3) {
            RCLCPP_WARN(node.get_logger(),
                        "self_filter_max_xyz must contain exactly 3 values; using the default box maximum");
            self_filter_max = default_max;
        }

        self_filter_min_xyz << self_filter_min[0], self_filter_min[1], self_filter_min[2];
        self_filter_max_xyz << self_filter_max[0], self_filter_max[1], self_filter_max[2];
        if ((self_filter_min_xyz.array() >= self_filter_max_xyz.array()).any()) {
            RCLCPP_WARN(node.get_logger(),
                        "Invalid self-filter box: every min_xyz value must be smaller than max_xyz; "
                        "self filtering has been disabled");
            self_filter_enabled = false;
        }
        space_downsample = node.declare_parameter<bool>("space_downsample");
        space_downsample_leaf_size = node.declare_parameter<float>("space_downsample_leaf_size");

        // IMU处理
        std::vector<double> gravity_temp = node.declare_parameter<std::vector<double>>("gravity");
        gravity << gravity_temp[0], gravity_temp[1], gravity_temp[2];
        check_satu = node.declare_parameter<bool>("check_satu");
        fix_gravity_direction = node.declare_parameter<bool>("fix_gravity_direction");
        satu_acc = node.declare_parameter<double>("satu_acc") * 0.99;
        satu_gyro = node.declare_parameter<double>("satu_gyro") * 0.99;
        satu_acc_norm =
                node.declare_parameter<double>("satu_acc_norm", 2.5) * 0.99;
        acc_norm = node.declare_parameter<double>("acc_norm");
        if (!std::isfinite(satu_acc_norm) || satu_acc_norm <= acc_norm) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "satu_acc_norm must be finite and greater than acc_norm; "
                    "using 2.5 g");
            satu_acc_norm = 2.5 * 0.99;
        }

        // 地图
        map_resolution = node.declare_parameter<float>("map_resolution");
        init_map_size = static_cast<size_t>(node.declare_parameter<long>("init_map_size"));

        // 雷达与IMU相对位姿
        extrinsic_est_en = node.declare_parameter<bool>("extrinsic_est_en");
        std::vector<double> extrinsic_T_temp = node.declare_parameter<std::vector<double>>("extrinsic_T");
        extrinsic_T << extrinsic_T_temp[0], extrinsic_T_temp[1], extrinsic_T_temp[2];
        std::vector<double> extrinsic_R_temp = node.declare_parameter<std::vector<double>>("extrinsic_R");
        extrinsic_R << extrinsic_R_temp[0], extrinsic_R_temp[1], extrinsic_R_temp[2],
                extrinsic_R_temp[3], extrinsic_R_temp[4], extrinsic_R_temp[5],
                extrinsic_R_temp[6], extrinsic_R_temp[7], extrinsic_R_temp[8];

        // 滤波器参数
        laser_point_cov = node.declare_parameter<double>("laser_point_cov");
        gravity_aligned_plane_min_alignment =
                node.declare_parameter<double>(
                        "gravity_aligned_plane_min_alignment", 0.90);
        gravity_aligned_plane_covariance_scale =
                node.declare_parameter<double>(
                        "gravity_aligned_plane_covariance_scale", 0.25);
        imu_meas_acc_cov = node.declare_parameter<double>("imu_meas_acc_cov");
        imu_meas_omg_cov = node.declare_parameter<double>("imu_meas_omg_cov");
        velocity_cov = node.declare_parameter<double>("velocity_cov");
        acceleration_cov = node.declare_parameter<double>("acceleration_cov");
        omg_cov = node.declare_parameter<double>("omg_cov");
        ba_cov = node.declare_parameter<double>("ba_cov");
        bg_cov = node.declare_parameter<double>("bg_cov");
        plane_threshold = node.declare_parameter<double>("plane_threshold");
        plane_max_eigenvalue_ratio = node.declare_parameter<double>(
                "plane_max_eigenvalue_ratio", 0.10);
        plane_min_secondary_stddev = node.declare_parameter<double>(
                "plane_min_secondary_stddev", 0.10);
        match_max_neighbor_distance = node.declare_parameter<double>(
                "match_max_neighbor_distance", 1.20);
        point_plane_max_residual = node.declare_parameter<double>(
                "point_plane_max_residual", 0.15);
        match_sqaured = node.declare_parameter<double>("match_sqaured");
        scan_quality_min_attempted_points = static_cast<std::size_t>(
                std::max<long>(
                        1,
                        node.declare_parameter<long>(
                                "scan_quality_min_attempted_points", 200)));
        scan_quality_min_accepted_points = static_cast<std::size_t>(
                std::max<long>(
                        1,
                        node.declare_parameter<long>(
                                "scan_quality_min_accepted_points", 40)));
        scan_quality_strong_accepted_points = static_cast<std::size_t>(
                std::max<long>(
                        static_cast<long>(scan_quality_min_accepted_points),
                        node.declare_parameter<long>(
                                "scan_quality_strong_accepted_points", 120)));
        scan_quality_min_acceptance_ratio =
                node.declare_parameter<double>(
                        "scan_quality_min_acceptance_ratio", 0.04);
        if (!std::isfinite(scan_quality_min_acceptance_ratio) ||
            scan_quality_min_acceptance_ratio <= 0.0 ||
            scan_quality_min_acceptance_ratio > 1.0) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "scan_quality_min_acceptance_ratio must be in (0, 1]; "
                    "using 0.04");
            scan_quality_min_acceptance_ratio = 0.04;
        }
        if (scan_quality_min_accepted_points >
            scan_quality_min_attempted_points) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "scan_quality_min_accepted_points exceeds the attempted "
                    "point threshold; clamping it to %zu",
                    scan_quality_min_attempted_points);
            scan_quality_min_accepted_points =
                    scan_quality_min_attempted_points;
        }
        if (scan_quality_strong_accepted_points <
            scan_quality_min_accepted_points) {
            scan_quality_strong_accepted_points =
                    scan_quality_min_accepted_points;
        }
        if (!std::isfinite(gravity_aligned_plane_min_alignment) ||
            gravity_aligned_plane_min_alignment < 0.0 ||
            gravity_aligned_plane_min_alignment > 1.0) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "gravity_aligned_plane_min_alignment must be in [0, 1]; "
                    "using 0.90");
            gravity_aligned_plane_min_alignment = 0.90;
        }
        if (!std::isfinite(gravity_aligned_plane_covariance_scale) ||
            gravity_aligned_plane_covariance_scale <= 0.0 ||
            gravity_aligned_plane_covariance_scale > 1.0) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "gravity_aligned_plane_covariance_scale must be in (0, 1]; "
                    "using 0.25");
            gravity_aligned_plane_covariance_scale = 0.25;
        }
        if (!std::isfinite(plane_max_eigenvalue_ratio) ||
            plane_max_eigenvalue_ratio <= 0.0 ||
            plane_max_eigenvalue_ratio >= 1.0) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "plane_max_eigenvalue_ratio must be in (0, 1); using 0.10");
            plane_max_eigenvalue_ratio = 0.10;
        }
        if (!std::isfinite(plane_min_secondary_stddev) ||
            plane_min_secondary_stddev <= 0.0) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "plane_min_secondary_stddev must be finite and positive; "
                    "using 0.10 m");
            plane_min_secondary_stddev = 0.10;
        }
        if (!std::isfinite(match_max_neighbor_distance) ||
            match_max_neighbor_distance <= 0.0) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "match_max_neighbor_distance must be finite and positive; "
                    "using 1.20 m");
            match_max_neighbor_distance = 1.20;
        }
        if (!std::isfinite(point_plane_max_residual) ||
            point_plane_max_residual <= 0.0) {
            RCLCPP_WARN(
                    node.get_logger(),
                    "point_plane_max_residual must be finite and positive; "
                    "using 0.15 m");
            point_plane_max_residual = 0.15;
        }

        // 数据发布
        publish_odometry_without_downsample = node.declare_parameter<bool>("publish_odometry_without_downsample");
    }

}// namespace small_point_lio
