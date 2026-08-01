/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "pointcloud_mapping.h"

namespace util {

    void PointcloudMappingGrid::add_observation(
            const Eigen::Vector3f &point, const std::uint64_t frame_index) {
        if (observations == 0) {
            first_frame = frame_index;
        }
        ++observations;
        points_sum += point;
        last_frame = frame_index;
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

    uint64_t PointcloudMapping::GridKeyTypeHasher::operator()(const GridKeyType &v) const {
        const auto x = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(v[0]));
        const auto y = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(v[1]));
        const auto z = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(v[2]));
        return (x * 73856093ULL) ^ (y * 19349663ULL) ^ (z * 83492791ULL);
    }

    PointcloudMapping::PointcloudMapping(
            const float resolution,
            const std::uint32_t min_observations_value,
            const std::uint32_t observation_stride_value,
            const std::uint64_t stale_after_frames_value,
            const std::uint64_t prune_interval_frames_value,
            const std::size_t max_voxels_value)
        : inv_resolution(1.0F / resolution),
          min_observations(std::max<std::uint32_t>(1, min_observations_value)),
          observation_stride(std::max<std::uint32_t>(1, observation_stride_value)),
          stale_after_frames(std::max<std::uint64_t>(1, stale_after_frames_value)),
          prune_interval_frames(
              std::max<std::uint64_t>(1, prune_interval_frames_value)),
          max_voxels(std::max<std::size_t>(1, max_voxels_value)) {}

    [[nodiscard]] PointcloudMapping::GridKeyType PointcloudMapping::get_position_index(const Eigen::Vector3f &point) const {
        return (point * inv_resolution).array().floor().cast<int>();
    }

    void PointcloudMapping::prune_stale_voxels() {
        if (received_frames <= stale_after_frames) {
            return;
        }

        const auto oldest_allowed_frame = received_frames - stale_after_frames;
        for (auto iterator = grids_map.begin(); iterator != grids_map.end();) {
            const auto &grid = iterator->second;
            if (grid.get_observations() < min_observations &&
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

        struct FrameGrid {
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            std::uint32_t count = 0;
        };
        ankerl::unordered_dense::map<GridKeyType, FrameGrid, GridKeyTypeHasher>
            frame_grids;
        frame_grids.reserve(pointcloud.size() / 4 + 1);

        for (const auto &point: pointcloud) {
            if (!point.allFinite()) {
                continue;
            }
            auto &frame_grid =
                frame_grids.try_emplace(get_position_index(point)).first->second;
            frame_grid.sum += point;
            ++frame_grid.count;
        }

        if (received_frames - last_prune_frame >= prune_interval_frames ||
            grids_map.size() + frame_grids.size() > max_voxels) {
            prune_stale_voxels();
        }

        for (const auto &[key, frame_grid]: frame_grids) {
            const Eigen::Vector3f centroid =
                frame_grid.sum / static_cast<float>(frame_grid.count);
            auto existing = grids_map.find(key);
            if (existing != grids_map.end()) {
                existing->second.add_observation(centroid, received_frames);
                continue;
            }
            if (grids_map.size() >= max_voxels) {
                capacity_exceeded = true;
                ++dropped_voxels;
                continue;
            }
            auto inserted = grids_map.try_emplace(key);
            inserted.first->second.add_observation(centroid, received_frames);
        }
    }

    [[nodiscard]] PointcloudMappingSnapshot PointcloudMapping::get_snapshot() const {
        std::lock_guard<std::mutex> lock(mutex);
        PointcloudMappingSnapshot snapshot;
        snapshot.received_frames = received_frames;
        snapshot.sampled_frames = sampled_frames;
        snapshot.tracked_voxels = grids_map.size();
        snapshot.dropped_voxels = dropped_voxels;
        snapshot.capacity_exceeded = capacity_exceeded;
        snapshot.points.reserve(grids_map.size());
        for (const auto &item: grids_map) {
            if (item.second.get_observations() >= min_observations) {
                snapshot.points.push_back(item.second.get_point());
            }
        }
        snapshot.persistent_voxels = snapshot.points.size();
        return snapshot;
    }

}// namespace util
