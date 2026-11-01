/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>
#include <cstddef>
#include <limits>
#include <mutex>
#include <string>

namespace util {

    enum class PointcloudMappingFilterMode {
        Protected,
        Bypass,
        Strict,
    };

    [[nodiscard]] PointcloudMappingFilterMode
    pointcloud_mapping_filter_mode_from_string(
        const std::string &value,
        bool *valid = nullptr);

    [[nodiscard]] const char *pointcloud_mapping_filter_mode_name(
        PointcloudMappingFilterMode mode);

    class PointcloudMappingGrid {
    private:
        std::uint32_t observations = 0;
        Eigen::Vector3f points_sum = Eigen::Vector3f::Zero();
        std::uint64_t first_frame = 0;
        std::uint64_t last_frame = 0;
        std::uint32_t ground_observations = 0;
        std::uint64_t ground_first_frame = 0;
        std::uint64_t ground_last_frame = 0;
        bool regular_confirmation_retained = false;
        bool ground_confirmation_retained = false;

    public:
        PointcloudMappingGrid() = default;

        void reset_observations(
            const Eigen::Vector3f &point,
            std::uint64_t frame_index,
            bool ground_candidate);

        void add_observation(
            const Eigen::Vector3f &point,
            std::uint64_t frame_index,
            std::uint64_t max_observation_gap_frames,
            bool ground_candidate,
            bool retain_regular_confirmation = false,
            bool retain_ground_confirmation = false);

        [[nodiscard]] Eigen::Vector3f get_point() const;

        [[nodiscard]] std::uint32_t get_observations() const;

        [[nodiscard]] std::uint64_t get_first_frame() const;

        [[nodiscard]] std::uint64_t get_last_frame() const;

        [[nodiscard]] bool is_persistent(
            std::uint32_t min_observations,
            std::uint64_t min_stable_span_frames,
            PointcloudMappingFilterMode filter_mode,
            bool ground_preservation_enabled,
            std::uint32_t ground_min_observations,
            std::uint64_t ground_min_stable_span_frames) const;

        [[nodiscard]] bool is_ground_persistent(
            std::uint32_t min_observations,
            std::uint64_t min_stable_span_frames) const;
    };

    struct PointcloudMappingGroundPolicy {
        bool enabled = false;
        float cell_size = 0.20F;
        float max_height_above_envelope = 0.08F;
        float max_slope_deg = 30.0F;
        std::uint32_t min_neighbor_cells = 3;
        std::uint32_t min_observations = 3;
        std::uint64_t min_stable_span_frames = 6;
        PointcloudMappingFilterMode filter_mode =
            PointcloudMappingFilterMode::Protected;
    };

    struct PointcloudMappingSnapshot {
        std::vector<Eigen::Vector3f> points;
        std::uint64_t received_frames = 0;
        std::uint64_t sampled_frames = 0;
        std::size_t tracked_voxels = 0;
        std::size_t persistent_voxels = 0;
        std::size_t ground_protected_voxels = 0;
        std::size_t dropped_voxels = 0;
        bool capacity_exceeded = false;
        PointcloudMappingFilterMode filter_mode =
            PointcloudMappingFilterMode::Protected;
    };

    class PointcloudMapping {
    private:
        using GridKeyType = Eigen::Vector3i;

        struct GroundCellKey {
            int x = 0;
            int y = 0;

            bool operator==(const GroundCellKey &other) const {
                return x == other.x && y == other.y;
            }
        };

        struct GroundCellKeyHasher {
            std::size_t operator()(const GroundCellKey &value) const;
        };

        struct GroundCell {
            float minimum_z = std::numeric_limits<float>::infinity();
        };

        struct GridKeyTypeHasher {
            uint64_t operator()(const GridKeyType &v) const;
        };

        ankerl::unordered_dense::map<GridKeyType, PointcloudMappingGrid, GridKeyTypeHasher> grids_map;
        float inv_resolution;
        std::uint32_t min_observations;
        std::uint32_t observation_stride;
        std::uint64_t max_observation_gap_frames;
        std::uint64_t min_stable_span_frames;
        std::uint64_t stale_after_frames;
        std::uint64_t prune_interval_frames;
        std::size_t max_voxels;
        PointcloudMappingGroundPolicy ground_policy;
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
            std::uint64_t max_observation_gap_frames,
            std::uint64_t min_stable_span_frames,
            std::uint64_t stale_after_frames,
            std::uint64_t prune_interval_frames,
            std::size_t max_voxels,
            PointcloudMappingGroundPolicy ground_policy = {});

    private:
        [[nodiscard]] GridKeyType get_position_index(const Eigen::Vector3f &point) const;

        [[nodiscard]] std::vector<bool> classify_ground_candidates(
            const std::vector<Eigen::Vector3f> &pointcloud) const;

        void prune_stale_voxels();

    public:
        void add_pointcloud(const std::vector<Eigen::Vector3f> &point);

        [[nodiscard]] PointcloudMappingSnapshot get_snapshot(
            std::size_t maximum_points = 0) const;
    };

}// namespace util
