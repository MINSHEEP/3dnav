/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace common {

    struct Odometry {
        double timestamp;                // Unit: s
        Eigen::Vector3d position;        // Unit: m
        Eigen::Vector3d velocity;        // Unit: m/s
        Eigen::Quaterniond orientation;  // Unit: rad
        Eigen::Vector3d angular_velocity;// Unit: rad/s
        std::size_t point_update_attempts{0};
        std::size_t point_update_accepted{0};
        double point_update_acceptance_ratio{0.0};
        bool scan_matching_healthy{true};
    };

    struct ImuMsg {
        double timestamp;                   // Unit: s
        Eigen::Vector3d linear_acceleration;// Unit: g
        Eigen::Vector3d angular_velocity;   // Unit: rad/s
    };

    struct Point {
        double timestamp;        // Unit: s
        Eigen::Vector3f position;// Unit: m
    };

}// namespace common
