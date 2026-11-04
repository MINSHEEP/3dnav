/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace small_point_lio {

    class Parameters {
    public:
        int point_filter_num;
        float min_distance_squared;
        float max_distance_squared;
        float registered_cloud_max_distance_squared;
        bool self_filter_enabled;
        Eigen::Vector3d self_filter_min_xyz;
        Eigen::Vector3d self_filter_max_xyz;
        bool space_downsample;
        float space_downsample_leaf_size;

        Eigen::Vector3d gravity;
        bool check_satu;
        bool fix_gravity_direction;
        double satu_acc;
        double satu_gyro;
        double satu_acc_norm;
        double acc_norm;

        double map_resolution;
        size_t init_map_size;

        bool extrinsic_est_en;
        Eigen::Vector3d extrinsic_T;
        Eigen::Matrix3d extrinsic_R;

        double laser_point_cov;
        double gravity_aligned_plane_min_alignment;
        double gravity_aligned_plane_covariance_scale;
        double imu_meas_acc_cov;
        double imu_meas_omg_cov;
        double velocity_cov;
        double omg_cov;
        double acceleration_cov;
        double bg_cov;
        double ba_cov;
        double plane_threshold;
        double plane_max_eigenvalue_ratio;
        double plane_min_secondary_stddev;
        double match_max_neighbor_distance;
        double point_plane_max_residual;
        double match_sqaured;
        std::size_t scan_quality_min_attempted_points;
        std::size_t scan_quality_min_accepted_points;
        std::size_t scan_quality_strong_accepted_points;
        double scan_quality_min_acceptance_ratio;

        bool publish_odometry_without_downsample = false;

        void read_parameters(rclcpp::Node &node);
    };

}// namespace small_point_lio
