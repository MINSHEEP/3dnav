/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"
#include "measurement_quality_policy.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(rclcpp::Node &node) {
        // init param
        parameters.read_parameters(node);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        if (parameters.publish_odometry_without_downsample) {
            RCLCPP_WARN(
                node.get_logger(),
                "publish_odometry_without_downsample is incompatible with "
                "transactional scan validation and has been disabled");
            parameters.publish_odometry_without_downsample = false;
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        pointcloud_odom_frame.clear();
        pending_map_points.clear();
        time_current = 0.0;
        batch_time_current = 0.0;
        batch_consecutive_imu_update_failures = 0;
        batch_imu_messages.clear();
        batch_point_update_attempts = 0;
        batch_point_update_accepted = 0;
        batch_active = false;
        consecutive_imu_update_failures = 0;
        is_init = false;
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // we need to init small point lio
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // Use the stationary IMU samples to make odom gravity-aligned.
                // Merely rotating the gravity vector into the initially tilted
                // IMU frame leaves the state rotation at identity. With a
                // pitched sensor mount that makes the complete base_link pose
                // inherit the inverse mounting angle.
                if (parameters.fix_gravity_direction) {
                    Eigen::Matrix<state::value_type, 3, 1>
                        mean_specific_force =
                            Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        mean_specific_force +=
                            imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    mean_specific_force /=
                        static_cast<state::value_type>(
                            preprocess.imu_deque.size());
                    mean_specific_force *=
                        static_cast<state::value_type>(
                            estimator.imu_acceleration_scale);

                    const Eigen::Matrix<state::value_type, 3, 1>
                        gravity_odom =
                            parameters.gravity.cast<state::value_type>();
                    const Eigen::Matrix<state::value_type, 3, 1>
                        stationary_specific_force_odom = -gravity_odom;

                    constexpr state::value_type minimum_acceleration_norm =
                        static_cast<state::value_type>(1.0e-6);
                    if (mean_specific_force.allFinite() &&
                        gravity_odom.allFinite() &&
                        mean_specific_force.norm() >
                            minimum_acceleration_norm &&
                        stationary_specific_force_odom.norm() >
                            minimum_acceleration_norm) {
                        using StateQuaternion =
                            Eigen::Quaternion<state::value_type>;
                        const StateQuaternion odom_from_imu =
                            StateQuaternion::FromTwoVectors(
                                mean_specific_force,
                                stationary_specific_force_odom);
                        estimator.kf.x.rotation =
                            odom_from_imu.normalized().toRotationMatrix();
                        estimator.kf.x.gravity = gravity_odom;
                        estimator.kf.x.acceleration = mean_specific_force;
                    } else {
                        // Preserve a finite, stationary fallback if the IMU
                        // delivered invalid data during initialization.
                        estimator.kf.x.rotation =
                            Eigen::Matrix<state::value_type, 3, 3>::Identity();
                        estimator.kf.x.gravity = gravity_odom;
                        estimator.kf.x.acceleration = -gravity_odom;
                    }
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                    estimator.kf.x.acceleration = -estimator.kf.x.gravity;
                }

                // The initial voxel map must use the same gravity-aligned odom
                // frame as all points processed after initialization.
                for (const auto &point: preprocess.point_deque) {
                    Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
                    if (parameters.extrinsic_est_en) {
                        point_imu_frame =
                            estimator.kf.x.offset_R_L_I *
                                point.position.cast<state::value_type>() +
                            estimator.kf.x.offset_T_L_I;
                    } else {
                        point_imu_frame =
                            estimator.Lidar_R_wrt_IMU *
                                point.position.cast<state::value_type>() +
                            estimator.Lidar_T_wrt_IMU;
                    }
                    estimator.ivox->add_point(
                        (estimator.kf.x.rotation * point_imu_frame +
                         estimator.kf.x.position)
                            .cast<float>());
                }
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }

        // judge we should do point update or imu update
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        if (!preprocess.imu_deque.empty() &&
            !preprocess.dense_point_deque.empty() &&
            !preprocess.point_deque.empty()) {
            begin_batch();
        }
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // Use the wider configured range for scan matching, but only
                // publish nearby dense returns to Octomap and the localization
                // PCD accumulator.
                if (dense_point_lidar_frame.position.squaredNorm() <=
                    parameters.registered_cloud_max_distance_squared) {
                    Eigen::Matrix<state::value_type, 3, 1> dense_point_imu_frame;
                    if (parameters.extrinsic_est_en) {
                        dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                    } else {
                        dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                    }
                    pointcloud_odom_frame.emplace_back(
                            (estimator.kf.x.rotation * dense_point_imu_frame +
                             estimator.kf.x.position).cast<float>());
                }

                preprocess.dense_point_deque.pop_front();
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                // predict
                estimator.kf.predict_state(time_current);

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                ++batch_point_update_attempts;
                if (estimator.kf.update_point()) {
                    ++batch_point_update_accepted;
                }

                // Do not expose the current batch to nearest-neighbour
                // matching yet. The batch is committed only after the node's
                // physical plausibility guard accepts its odometry.
                pending_map_points.push_back(estimator.point_odom_frame);

                preprocess.point_deque.pop_front();
            } else {
                // imu update
                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                batch_imu_messages.push_back(imu_msg);
                apply_imu_update(imu_msg);

                preprocess.imu_deque.pop_front();
            }
        }

        if (is_publish_odometry) {
            const OdometryDecision decision = publish_odometry(time_current);
            finish_batch(decision);
            if (decision == OdometryDecision::Accept &&
                !pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
            }
            pointcloud_odom_frame.clear();
        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(
        const std::function<OdometryDecision(
            const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    const char *SmallPointLio::estimator_health_error() const {
        const auto &x = estimator.kf.x;
        const auto &P = estimator.kf.P;
        if (consecutive_imu_update_failures >= 3) {
            return "imu_update_failures";
        }
        if (!x.position.allFinite() ||
            !x.rotation.allFinite() ||
            !x.offset_R_L_I.allFinite() ||
            !x.offset_T_L_I.allFinite() ||
            !x.velocity.allFinite() ||
            !x.omg.allFinite() ||
            !x.acceleration.allFinite() ||
            !x.gravity.allFinite() ||
            !x.bg.allFinite() ||
            !x.ba.allFinite() ||
            !Q.allFinite() ||
            !P.allFinite() ||
            !(P.diagonal().array() >= -1.0e-6).all()) {
            return "filter_numerical_state";
        }
        return nullptr;
    }

    EstimatorDiagnostics SmallPointLio::get_estimator_diagnostics() const {
        return EstimatorDiagnostics{
            estimator.kf.x.gravity.cast<double>(),
            estimator.kf.x.acceleration.cast<double>(),
            estimator.kf.x.bg.cast<double>(),
            estimator.kf.x.ba.cast<double>()};
    }

    OdometryDecision SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            common::Odometry odometry;
            odometry.timestamp = timestamp;
            odometry.position = estimator.kf.x.position.cast<double>();
            odometry.velocity = estimator.kf.x.velocity.cast<double>();
            odometry.orientation = estimator.kf.x.rotation.cast<double>();
            odometry.angular_velocity = estimator.kf.x.omg.cast<double>();
            odometry.point_update_attempts = batch_point_update_attempts;
            odometry.point_update_accepted = batch_point_update_accepted;
            odometry.point_update_acceptance_ratio =
                    batch_point_update_attempts > 0
                    ? static_cast<double>(batch_point_update_accepted) /
                        static_cast<double>(batch_point_update_attempts)
                    : 0.0;
            odometry.scan_matching_healthy =
                    is_scan_matching_healthy(
                            batch_point_update_attempts,
                            batch_point_update_accepted,
                            parameters.scan_quality_min_attempted_points,
                            parameters.scan_quality_min_accepted_points,
                            parameters.scan_quality_strong_accepted_points,
                            parameters.scan_quality_min_acceptance_ratio);
            return odometry_callback(odometry);
        }
        return OdometryDecision::Accept;
    }

    void SmallPointLio::begin_batch() {
        if (batch_active) {
            return;
        }
        batch_checkpoint = estimator.kf.checkpoint();
        batch_time_current = time_current;
        batch_consecutive_imu_update_failures =
            consecutive_imu_update_failures;
        batch_imu_messages.clear();
        batch_point_update_attempts = 0;
        batch_point_update_accepted = 0;
        pending_map_points.clear();
        batch_active = true;
    }

    void SmallPointLio::finish_batch(const OdometryDecision decision) {
        if (!batch_active) {
            return;
        }
        if (decision == OdometryDecision::Accept) {
            for (const auto &point : pending_map_points) {
                estimator.ivox->add_point(point);
            }
        } else {
            estimator.kf.restore(batch_checkpoint);
            time_current = batch_time_current;
            consecutive_imu_update_failures =
                batch_consecutive_imu_update_failures;
            if (decision == OdometryDecision::Quarantine) {
                // Keep only the IMU timeline from a rejected batch.  Replaying
                // it after restoring the checkpoint removes all suspect lidar
                // corrections without creating an IMU time gap.  The next
                // scan can therefore recover from the last accepted state.
                for (const auto &imu_msg : batch_imu_messages) {
                    if (imu_msg.timestamp >= time_current) {
                        apply_imu_update(imu_msg);
                    }
                }
            }
        }
        batch_imu_messages.clear();
        pending_map_points.clear();
        batch_active = false;
    }

    void SmallPointLio::apply_imu_update(const common::ImuMsg &imu_msg) {
        time_current = imu_msg.timestamp;
        estimator.kf.predict_state(time_current);
        estimator.kf.predict_cov(time_current, Q);
        estimator.angular_velocity =
                imu_msg.angular_velocity.cast<state::value_type>();
        estimator.linear_acceleration =
                imu_msg.linear_acceleration.cast<state::value_type>();
        if (estimator.kf.update_imu()) {
            consecutive_imu_update_failures = 0;
        } else {
            ++consecutive_imu_update_failures;
        }
    }

}// namespace small_point_lio
