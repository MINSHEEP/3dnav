/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "pointcloud_mapping.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <utility>

namespace util {

    [[nodiscard]] PointcloudMappingFilterMode
    pointcloud_mapping_filter_mode_from_string(
            const std::string &value,
            bool *valid) {
        std::string normalized;
        normalized.reserve(value.size());
        for (const unsigned char character : value) {
            if (!std::isspace(character)) {
                normalized.push_back(static_cast<char>(
                    std::tolower(character)));
            }
        }
        if (normalized == "protected") {
            if (valid != nullptr) {
                *valid = true;
            }
            return PointcloudMappingFilterMode::Protected;
        }
        if (normalized == "bypass") {
            if (valid != nullptr) {
                *valid = true;
            }
            return PointcloudMappingFilterMode::Bypass;
        }
        if (normalized == "strict") {
            if (valid != nullptr) {
                *valid = true;
            }
            return PointcloudMappingFilterMode::Strict;
        }
        if (valid != nullptr) {
            *valid = false;
        }
        return PointcloudMappingFilterMode::Protected;
    }

    [[nodiscard]] const char *pointcloud_mapping_filter_mode_name(
            const PointcloudMappingFilterMode mode) {
        switch (mode) {
            case PointcloudMappingFilterMode::Bypass:
                return "bypass";
            case PointcloudMappingFilterMode::Strict:
                return "strict";
            case PointcloudMappingFilterMode::Protected:
            default:
                return "protected";
        }
    }

    void PointcloudMappingGrid::reset_observations(
            const Eigen::Vector3f &point,
            const std::uint64_t frame_index,
            const bool ground_candidate) {
        observations = 1;
        points_sum = point;
        first_frame = frame_index;
        last_frame = frame_index;
        ground_observations = ground_candidate ? 1U : 0U;
        ground_first_frame = frame_index;
        ground_last_frame = ground_candidate ? frame_index : 0U;
        regular_confirmation_retained = false;
        ground_confirmation_retained = false;
    }

    void PointcloudMappingGrid::add_observation(
            const Eigen::Vector3f &point,
            const std::uint64_t frame_index,
            const std::uint64_t max_observation_gap_frames,
            const bool ground_candidate,
            const bool retain_regular_confirmation,
            const bool retain_ground_confirmation) {
        regular_confirmation_retained |= retain_regular_confirmation;
        ground_confirmation_retained |= retain_ground_confirmation;
        // Absence restarts candidate confirmation, not a committed static
        // surface. Keep its centroid history when the robot revisits it.
        if (observations == 0 ||
            (frame_index > last_frame + max_observation_gap_frames &&
             !regular_confirmation_retained &&
             !ground_confirmation_retained)) {
            reset_observations(point, frame_index, ground_candidate);
            return;
        }
        ++observations;
        points_sum += point;
        last_frame = frame_index;
        if (!ground_candidate) {
            ground_observations = 0;
            ground_first_frame = frame_index;
            ground_last_frame = 0;
        } else if (ground_observations == 0 ||
                   frame_index > ground_last_frame +
                       max_observation_gap_frames) {
            ground_observations = 1;
            ground_first_frame = frame_index;
            ground_last_frame = frame_index;
        } else if (ground_last_frame != frame_index) {
            ++ground_observations;
            ground_last_frame = frame_index;
        }
    }

    [[nodiscard]] Eigen::Vector3f PointcloudMappingGrid::get_point() const {
        if (observations == 0) {
            return Eigen::Vector3f::Zero();
        }
        return points_sum / static_cast<float>(observations);
    }

    [[nodiscard]] std::uint32_t PointcloudMappingGrid::get_observations() const {
        return observations;
    }

    [[nodiscard]] std::uint64_t PointcloudMappingGrid::get_first_frame() const {
        return first_frame;
    }

    [[nodiscard]] std::uint64_t PointcloudMappingGrid::get_last_frame() const {
        return last_frame;
    }

    [[nodiscard]] bool PointcloudMappingGrid::is_persistent(
            const std::uint32_t min_observations,
            const std::uint64_t min_stable_span_frames,
            const PointcloudMappingFilterMode filter_mode,
            const bool ground_preservation_enabled,
            const std::uint32_t ground_min_observations,
            const std::uint64_t ground_min_stable_span_frames) const {
        if (filter_mode == PointcloudMappingFilterMode::Bypass) {
            return observations > 0;
        }
        const bool regular_persistent = regular_confirmation_retained ||
            (observations >= min_observations &&
             last_frame >= first_frame + min_stable_span_frames);
        return regular_persistent ||
            (filter_mode == PointcloudMappingFilterMode::Protected &&
             ground_preservation_enabled && is_ground_persistent(
                ground_min_observations,
                ground_min_stable_span_frames));
    }

    [[nodiscard]] bool PointcloudMappingGrid::is_ground_persistent(
            const std::uint32_t min_observations,
            const std::uint64_t min_stable_span_frames) const {
        return ground_confirmation_retained ||
            (ground_observations >= min_observations &&
             ground_last_frame >= ground_first_frame + min_stable_span_frames);
    }

    uint64_t PointcloudMapping::GridKeyTypeHasher::operator()(const GridKeyType &v) const {
        const auto x = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(v[0]));
        const auto y = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(v[1]));
        const auto z = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(v[2]));
        return (x * 73856093ULL) ^ (y * 19349663ULL) ^ (z * 83492791ULL);
    }

    std::size_t PointcloudMapping::GroundCellKeyHasher::operator()(
            const GroundCellKey &value) const {
        const auto x = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(value.x));
        const auto y = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(value.y));
        return static_cast<std::size_t>(
            (x * 73856093ULL) ^ (y * 19349663ULL));
    }

    PointcloudMapping::PointcloudMapping(
            const float resolution,
            const std::uint32_t min_observations_value,
            const std::uint32_t observation_stride_value,
            const std::uint64_t max_observation_gap_frames_value,
            const std::uint64_t min_stable_span_frames_value,
            const std::uint64_t stale_after_frames_value,
            const std::uint64_t prune_interval_frames_value,
            const std::size_t max_voxels_value,
            PointcloudMappingGroundPolicy ground_policy_value)
        : inv_resolution(1.0F / resolution),
          min_observations(std::max<std::uint32_t>(1, min_observations_value)),
          observation_stride(std::max<std::uint32_t>(1, observation_stride_value)),
          max_observation_gap_frames(std::max<std::uint64_t>(
              observation_stride, max_observation_gap_frames_value)),
          min_stable_span_frames(min_stable_span_frames_value),
          stale_after_frames(std::max<std::uint64_t>(1, stale_after_frames_value)),
          prune_interval_frames(
              std::max<std::uint64_t>(1, prune_interval_frames_value)),
          max_voxels(std::max<std::size_t>(1, max_voxels_value)),
          ground_policy(std::move(ground_policy_value)) {
        ground_policy.cell_size = std::max(0.05F, ground_policy.cell_size);
        ground_policy.max_height_above_envelope = std::max(
            0.01F, ground_policy.max_height_above_envelope);
        ground_policy.max_slope_deg = std::clamp(
            ground_policy.max_slope_deg, 1.0F, 60.0F);
        ground_policy.min_neighbor_cells = std::clamp<std::uint32_t>(
            ground_policy.min_neighbor_cells, 1U, 8U);
        ground_policy.min_observations = std::max<std::uint32_t>(
            2U, ground_policy.min_observations);
        ground_policy.min_stable_span_frames = std::max<std::uint64_t>(
            1U, ground_policy.min_stable_span_frames);
    }

    [[nodiscard]] PointcloudMapping::GridKeyType PointcloudMapping::get_position_index(const Eigen::Vector3f &point) const {
        return GridKeyType{
            static_cast<int>(std::floor(point.x() * inv_resolution)),
            static_cast<int>(std::floor(point.y() * inv_resolution)),
            static_cast<int>(std::floor(point.z() * inv_resolution))};
    }

    [[nodiscard]] std::vector<bool> PointcloudMapping::classify_ground_candidates(
            const std::vector<Eigen::Vector3f> &pointcloud) const {
        std::vector<bool> result(pointcloud.size(), false);
        if (ground_policy.filter_mode !=
                PointcloudMappingFilterMode::Protected ||
            !ground_policy.enabled || pointcloud.empty()) {
            return result;
        }

        const float inverse_cell_size = 1.0F / ground_policy.cell_size;
        ankerl::unordered_dense::map<
            GroundCellKey, GroundCell, GroundCellKeyHasher> cells;
        cells.reserve(pointcloud.size() / 4 + 1);
        for (const auto &point : pointcloud) {
            if (!point.allFinite()) {
                continue;
            }
            const GroundCellKey key{
                static_cast<int>(std::floor(point.x() * inverse_cell_size)),
                static_cast<int>(std::floor(point.y() * inverse_cell_size))};
            auto &cell = cells.try_emplace(key).first->second;
            cell.minimum_z = std::min(cell.minimum_z, point.z());
        }

        const float maximum_slope = std::tan(
            ground_policy.max_slope_deg *
            static_cast<float>(std::acos(-1.0)) / 180.0F);
        for (std::size_t index = 0; index < pointcloud.size(); ++index) {
            const auto &point = pointcloud[index];
            if (!point.allFinite()) {
                continue;
            }
            const GroundCellKey key{
                static_cast<int>(std::floor(point.x() * inverse_cell_size)),
                static_cast<int>(std::floor(point.y() * inverse_cell_size))};
            const auto cell = cells.find(key);
            if (cell == cells.end() ||
                point.z() > cell->second.minimum_z +
                    ground_policy.max_height_above_envelope) {
                continue;
            }
            std::uint32_t consistent_neighbors = 0;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const auto neighbor = cells.find(
                        GroundCellKey{key.x + dx, key.y + dy});
                    if (neighbor == cells.end()) {
                        continue;
                    }
                    const float distance = ground_policy.cell_size *
                        std::hypot(static_cast<float>(dx),
                                   static_cast<float>(dy));
                    const float slope = std::abs(
                        neighbor->second.minimum_z -
                        cell->second.minimum_z) / distance;
                    if (slope <= maximum_slope) {
                        ++consistent_neighbors;
                    }
                }
            }
            result[index] = consistent_neighbors >=
                ground_policy.min_neighbor_cells;
        }
        return result;
    }

    void PointcloudMapping::prune_stale_voxels() {
        if (received_frames <= stale_after_frames) {
            return;
        }

        const auto oldest_allowed_frame = received_frames - stale_after_frames;
        for (auto iterator = grids_map.begin(); iterator != grids_map.end();) {
            const auto &grid = iterator->second;
            if (!grid.is_persistent(
                    min_observations,
                    min_stable_span_frames,
                    ground_policy.filter_mode,
                    ground_policy.enabled,
                    ground_policy.min_observations,
                    ground_policy.min_stable_span_frames) &&
                grid.get_last_frame() < oldest_allowed_frame) {
                iterator = grids_map.erase(iterator);
            } else {
                ++iterator;
            }
        }
        last_prune_frame = received_frames;
    }

    void PointcloudMapping::add_pointcloud(const std::vector<Eigen::Vector3f> &pointcloud) {
        std::lock_guard<std::mutex> lock(mutex);
        ++received_frames;
        if ((received_frames - 1) % observation_stride != 0) {
            return;
        }
        ++sampled_frames;
        const auto ground_candidates = classify_ground_candidates(pointcloud);

        struct FrameGrid {
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            std::uint32_t count = 0;
            std::uint32_t ground_count = 0;
        };
        ankerl::unordered_dense::map<GridKeyType, FrameGrid, GridKeyTypeHasher>
            frame_grids;
        frame_grids.reserve(pointcloud.size() / 4 + 1);

        for (std::size_t index = 0; index < pointcloud.size(); ++index) {
            const auto &point = pointcloud[index];
            if (!point.allFinite()) {
                continue;
            }
            auto &frame_grid =
                frame_grids.try_emplace(get_position_index(point)).first->second;
            frame_grid.sum += point;
            ++frame_grid.count;
            frame_grid.ground_count += ground_candidates[index] ? 1U : 0U;
        }

        if (received_frames - last_prune_frame >= prune_interval_frames ||
            grids_map.size() + frame_grids.size() > max_voxels) {
            prune_stale_voxels();
        }

        for (const auto &[key, frame_grid]: frame_grids) {
            const Eigen::Vector3f centroid =
                frame_grid.sum / static_cast<float>(frame_grid.count);
            const bool ground_candidate =
                frame_grid.ground_count == frame_grid.count;
            auto existing = grids_map.find(key);
            if (existing != grids_map.end()) {
                // Evaluate confirmation before this observation can reset a
                // gap or lose its current-frame ground-neighbour support.
                const bool retain_regular_confirmation =
                    ground_policy.filter_mode != PointcloudMappingFilterMode::Bypass &&
                    existing->second.is_persistent(
                        min_observations, min_stable_span_frames,
                        PointcloudMappingFilterMode::Strict, false,
                        ground_policy.min_observations,
                        ground_policy.min_stable_span_frames);
                const bool retain_ground_confirmation =
                    ground_policy.filter_mode == PointcloudMappingFilterMode::Protected &&
                    ground_policy.enabled &&
                    existing->second.is_ground_persistent(
                        ground_policy.min_observations,
                        ground_policy.min_stable_span_frames);
                existing->second.add_observation(
                    centroid,
                    received_frames,
                    max_observation_gap_frames,
                    ground_candidate,
                    retain_regular_confirmation,
                    retain_ground_confirmation);
                continue;
            }
            if (grids_map.size() >= max_voxels) {
                capacity_exceeded = true;
                ++dropped_voxels;
                continue;
            }
            auto inserted = grids_map.try_emplace(key);
            inserted.first->second.add_observation(
                centroid,
                received_frames,
                max_observation_gap_frames,
                ground_candidate);
        }
    }

    [[nodiscard]] PointcloudMappingSnapshot PointcloudMapping::get_snapshot(
        const std::size_t maximum_points) const {
        std::lock_guard<std::mutex> lock(mutex);
        PointcloudMappingSnapshot snapshot;
        snapshot.received_frames = received_frames;
        snapshot.sampled_frames = sampled_frames;
        snapshot.tracked_voxels = grids_map.size();
        snapshot.dropped_voxels = dropped_voxels;
        snapshot.capacity_exceeded = capacity_exceeded;
        snapshot.filter_mode = ground_policy.filter_mode;
        const std::size_t persistent_count = std::count_if(
            grids_map.begin(), grids_map.end(),
            [this](const auto &item) {
                return item.second.is_persistent(
                    min_observations,
                    min_stable_span_frames,
                    ground_policy.filter_mode,
                    ground_policy.enabled,
                    ground_policy.min_observations,
                    ground_policy.min_stable_span_frames);
            });
        snapshot.ground_protected_voxels = std::count_if(
            grids_map.begin(), grids_map.end(),
            [this](const auto &item) {
                const bool ground = ground_policy.filter_mode ==
                        PointcloudMappingFilterMode::Protected &&
                    ground_policy.enabled &&
                    item.second.is_ground_persistent(
                        ground_policy.min_observations,
                        ground_policy.min_stable_span_frames);
                const bool regular = item.second.is_persistent(
                    min_observations,
                    min_stable_span_frames,
                    PointcloudMappingFilterMode::Strict,
                    false,
                    ground_policy.min_observations,
                    ground_policy.min_stable_span_frames);
                return ground && !regular;
            });
        const std::size_t stride = maximum_points > 0 &&
                persistent_count > maximum_points
            ? (persistent_count + maximum_points - 1) / maximum_points
            : 1;
        snapshot.points.reserve(std::min(
            persistent_count,
            maximum_points > 0 ? maximum_points : persistent_count));
        std::size_t persistent_index = 0;
        for (const auto &item: grids_map) {
            if (item.second.is_persistent(
                    min_observations,
                    min_stable_span_frames,
                    ground_policy.filter_mode,
                    ground_policy.enabled,
                    ground_policy.min_observations,
                    ground_policy.min_stable_span_frames)) {
                if (persistent_index % stride == 0) {
                    snapshot.points.push_back(item.second.get_point());
                }
                ++persistent_index;
            }
        }
        snapshot.persistent_voxels = persistent_count;
        return snapshot;
    }

}// namespace util
