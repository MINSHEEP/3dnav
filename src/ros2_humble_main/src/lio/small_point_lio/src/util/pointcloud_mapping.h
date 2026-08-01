/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>
#include <cstddef>
#include <mutex>

namespace util {

    class PointcloudMappingGrid {
    private:
        std::uint32_t observations = 0;
        Eigen::Vector3f points_sum = Eigen::Vector3f::Zero();
        std::uint64_t first_frame = 0;
        std::uint64_t last_frame = 0;

    public:
        PointcloudMappingGrid() = default;

        void add_observation(
            const Eigen::Vector3f &point, std::uint64_t frame_index);

        [[nodiscard]] Eigen::Vector3f get_point() const;

        [[nodiscard]] std::uint32_t get_observations() const;

        [[nodiscard]] std::uint64_t get_first_frame() const;

        [[nodiscard]] std::uint64_t get_last_frame() const;
    };

    struct PointcloudMappingSnapshot {
        std::vector<Eigen::Vector3f> points;
        std::uint64_t received_frames = 0;
        std::uint64_t sampled_frames = 0;
        std::size_t tracked_voxels = 0;
        std::size_t persistent_voxels = 0;
        std::size_t dropped_voxels = 0;
        bool capacity_exceeded = false;
    };

    class PointcloudMapping {
    private:
        using GridKeyType = Eigen::Vector3i;

        struct GridKeyTypeHasher {
            uint64_t operator()(const GridKeyType &v) const;
        };

        ankerl::unordered_dense::map<GridKeyType, PointcloudMappingGrid, GridKeyTypeHasher> grids_map;
        float inv_resolution;
        std::uint32_t min_observations;
        std::uint32_t observation_stride;
        std::uint64_t stale_after_frames;
        std::uint64_t prune_interval_frames;
        std::size_t max_voxels;
        std::uint64_t received_frames = 0;
        std::uint64_t sampled_frames = 0;
        std::uint64_t last_prune_frame = 0;
        std::size_t dropped_voxels = 0;
        bool capacity_exceeded = false;
        mutable std::mutex mutex;

    public:
        PointcloudMapping(
            float resolution,
            std::uint32_t min_observations,
            std::uint32_t observation_stride,
            std::uint64_t stale_after_frames,
            std::uint64_t prune_interval_frames,
            std::size_t max_voxels);

    private:
        [[nodiscard]] GridKeyType get_position_index(const Eigen::Vector3f &point) const;

        void prune_stale_voxels();

    public:
        void add_pointcloud(const std::vector<Eigen::Vector3f> &point);

        [[nodiscard]] PointcloudMappingSnapshot get_snapshot() const;
    };

}// namespace util
