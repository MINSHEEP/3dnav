/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "estimator.h"
#include "odometry_guard_policy.h"
#include "parameters.h"
#include "preprocess.h"
#include <pch.h>

namespace small_point_lio {

    struct EstimatorDiagnostics {
        Eigen::Vector3d gravity;
        Eigen::Vector3d acceleration;
        Eigen::Vector3d gyroscope_bias;
        Eigen::Vector3d accelerometer_bias;
    };

    class SmallPointLio {
    private:
        Parameters parameters;
        Preprocess preprocess;
        Estimator estimator;
        double time_current = 0.0;
        std::vector<Eigen::Vector3f> pointcloud_odom_frame;
        std::vector<Eigen::Vector3f> pending_map_points;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> pointcloud_callback;
        std::function<OdometryDecision(const common::Odometry &odometry)>
            odometry_callback;
        eskf::Checkpoint batch_checkpoint;
        double batch_time_current = 0.0;
        int batch_consecutive_imu_update_failures = 0;
        std::vector<common::ImuMsg> batch_imu_messages;
        std::size_t batch_point_update_attempts = 0;
        std::size_t batch_point_update_accepted = 0;
        bool batch_active = false;
        bool is_init = false;
        int consecutive_imu_update_failures = 0;

    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(rclcpp::Node &node);

        void reset();

        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(
            const std::function<OdometryDecision(
                const common::Odometry &odometry)> &odometry_callback);

        [[nodiscard]] const char *estimator_health_error() const;
        [[nodiscard]] EstimatorDiagnostics get_estimator_diagnostics() const;

    private:
        OdometryDecision publish_odometry(double timestamp);
        void begin_batch();
        void finish_batch(OdometryDecision decision);
        void apply_imu_update(const common::ImuMsg &imu_msg);
    public:
        // Expose the current lidar->IMU extrinsics for external TF composition.
        Eigen::Matrix<state::value_type, 3, 1> get_lidar_T_wrt_IMU() const {
            return parameters.extrinsic_est_en ? estimator.kf.x.offset_T_L_I : estimator.Lidar_T_wrt_IMU;
        }
        Eigen::Matrix<state::value_type, 3, 3> get_lidar_R_wrt_IMU() const {
            return parameters.extrinsic_est_en ? estimator.kf.x.offset_R_L_I : estimator.Lidar_R_wrt_IMU;
        }
    };

}// namespace small_point_lio

