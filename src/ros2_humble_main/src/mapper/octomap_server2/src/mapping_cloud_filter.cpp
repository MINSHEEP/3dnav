#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/common/point_tests.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace octomap_server {

enum class FilterMode {
    Protected,
    Bypass,
    Strict,
};

FilterMode parse_filter_mode(const std::string &value, bool &valid) {
    std::string normalized;
    normalized.reserve(value.size());
    for (const unsigned char character : value) {
        if (!std::isspace(character)) {
            normalized.push_back(static_cast<char>(
                std::tolower(character)));
        }
    }
    valid = true;
    if (normalized == "protected") {
        return FilterMode::Protected;
    }
    if (normalized == "bypass") {
        return FilterMode::Bypass;
    }
    if (normalized == "strict") {
        return FilterMode::Strict;
    }
    valid = false;
    return FilterMode::Protected;
}

const char *filter_mode_name(const FilterMode mode) {
    switch (mode) {
        case FilterMode::Bypass:
            return "bypass";
        case FilterMode::Strict:
            return "strict";
        case FilterMode::Protected:
        default:
            return "protected";
    }
}

class MappingCloudFilter final : public rclcpp::Node {
public:
    MappingCloudFilter()
        : Node("mapping_cloud_filter"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {
        input_topic_ = declare_parameter<std::string>(
            "input_topic", "/cloud_registered");
        output_topic_ = declare_parameter<std::string>(
            "output_topic", "/cloud_mapping_filtered");
        dynamic_output_topic_ = declare_parameter<std::string>(
            "dynamic_output_topic", "/cloud_mapping_dynamic");
        const std::string dynamic_filter_mode_value =
            declare_parameter<std::string>(
                "dynamic_filter_mode", "protected");
        bool dynamic_filter_mode_valid = false;
        filter_mode_ = parse_filter_mode(
            dynamic_filter_mode_value, dynamic_filter_mode_valid);
        dynamic_filter_mode_name_ = filter_mode_name(filter_mode_);
        if (!dynamic_filter_mode_valid) {
            RCLCPP_WARN(
                get_logger(),
                "Unknown dynamic_filter_mode '%s'; using the safe default "
                "'protected'. Valid modes: protected, bypass, strict.",
                dynamic_filter_mode_value.c_str());
        }
        if (dynamic_filter_mode_value != dynamic_filter_mode_name_) {
            set_parameter(rclcpp::Parameter(
                "dynamic_filter_mode", dynamic_filter_mode_name_));
        }
        world_frame_ = declare_parameter<std::string>(
            "world_frame", "odom");
        sensor_frame_ = declare_parameter<std::string>(
            "sensor_frame", "livox_frame");
        nominal_sensor_height_ = declare_parameter<double>(
            "nominal_sensor_height", 0.52);
        min_obstacle_height_ = declare_parameter<double>(
            "min_obstacle_height", 0.10);
        max_obstacle_height_ = declare_parameter<double>(
            "max_obstacle_height", 0.85);
        ground_clearance_thickness_ = declare_parameter<double>(
            "ground_clearance_thickness", 0.06);
        floor_search_window_ = declare_parameter<double>(
            "floor_search_window", 0.35);
        floor_bin_size_ = declare_parameter<double>(
            "floor_bin_size", 0.02);
        min_floor_points_ = declare_parameter<int>(
            "min_floor_points", 80);
        min_floor_peak_fraction_ = declare_parameter<double>(
            "min_floor_peak_fraction", 0.04);
        floor_smoothing_alpha_ = declare_parameter<double>(
            "floor_smoothing_alpha", 0.70);
        max_floor_jump_ = declare_parameter<double>(
            "max_floor_jump", 0.10);
        max_floor_rate_ = declare_parameter<double>(
            "max_floor_rate", 0.45);
        recovery_confirmation_frames_ = declare_parameter<int>(
            "recovery_confirmation_frames", 5);
        min_floor_range_ = declare_parameter<double>(
            "min_floor_range", 0.40);
        max_floor_range_ = declare_parameter<double>(
            "max_floor_range", 12.0);
        minimum_output_points_ = declare_parameter<int>(
            "minimum_output_points", 20);
        temporal_voxel_size_ = declare_parameter<double>(
            "temporal_voxel_size", 0.15);
        static_confirmation_observations_ = declare_parameter<int>(
            "static_confirmation_observations", 8);
        static_confirmation_seconds_ = declare_parameter<double>(
            "static_confirmation_seconds", 1.5);
        ground_preservation_enabled_ = declare_parameter<bool>(
            "ground_preservation_enabled", true);
        ground_confirmation_observations_ = declare_parameter<int>(
            "ground_confirmation_observations", 3);
        ground_confirmation_seconds_ = declare_parameter<double>(
            "ground_confirmation_seconds", 0.2);
        max_observation_gap_frames_ = declare_parameter<int>(
            "max_observation_gap_frames", 3);
        candidate_stale_frames_ = declare_parameter<int>(
            "candidate_stale_frames", 30);
        confirmed_stale_frames_ = declare_parameter<int>(
            "confirmed_stale_frames", 1800);
        temporal_prune_interval_frames_ = declare_parameter<int>(
            "temporal_prune_interval_frames", 30);
        max_tracked_voxels_ = declare_parameter<int>(
            "max_tracked_voxels", 500000);

        validate_parameters();

        filtered_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                output_topic_, rclcpp::SensorDataQoS());
        dynamic_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                dynamic_output_topic_, rclcpp::SensorDataQoS());
        status_publisher_ = create_publisher<std_msgs::msg::String>(
            "/mapping_cloud_filter/status",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &MappingCloudFilter::cloud_callback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Temporal mapping filter enabled: %s -> static %s, dynamic %s; "
            "voxel=%.2f m, mode=%s, confirmation=%d observations/%.1f s, "
            "ground=%s "
            "%d observations/%.1f s. Navigation "
            "continues to use the untouched input cloud.",
            input_topic_.c_str(),
            output_topic_.c_str(),
            dynamic_output_topic_.c_str(),
            temporal_voxel_size_,
            dynamic_filter_mode_name_.c_str(),
            static_confirmation_observations_,
            static_confirmation_seconds_,
            ground_preservation_enabled_ ? "protected" : "normal",
            ground_confirmation_observations_,
            ground_confirmation_seconds_);
        if (filter_mode_ == FilterMode::Bypass) {
            RCLCPP_WARN(
                get_logger(),
                "Dynamic temporal confirmation is BYPASSED for controlled "
                "static-scene mapping. Finite/range/floor/height/capacity "
                "guards remain active, but moving objects can be persisted.");
        }
    }

private:
    struct FloorMeasurement {
        bool valid{false};
        double height{0.0};
        std::size_t support{0};
        std::size_t candidates{0};
    };

    struct VoxelKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const VoxelKey &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHasher {
        std::size_t operator()(const VoxelKey &key) const {
            const auto x = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(key.x));
            const auto y = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(key.y));
            const auto z = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(key.z));
            return static_cast<std::size_t>(
                (x * 73856093ULL) ^ (y * 19349663ULL) ^ (z * 83492791ULL));
        }
    };

    struct VoxelState {
        std::uint64_t first_frame{0};
        std::uint64_t last_frame{0};
        std::uint32_t consecutive_observations{0};
        double first_stamp_seconds{0.0};
        std::uint64_t last_ground_frame{0};
        std::uint32_t consecutive_ground_observations{0};
        double first_ground_stamp_seconds{0.0};
        bool confirmed{false};
        bool confirmed_as_ground{false};
    };

    struct FrameVoxel {
        double sum_x{0.0};
        double sum_y{0.0};
        double sum_z{0.0};
        std::size_t count{0};
        std::size_t ground_count{0};

        void add(const pcl::PointXYZ &point, const bool is_ground) {
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            ++count;
            ground_count += is_ground ? 1U : 0U;
        }

        bool is_ground_candidate() const {
            return count > 0 && ground_count == count;
        }

        pcl::PointXYZ centroid() const {
            pcl::PointXYZ point;
            const double divisor = static_cast<double>(std::max<std::size_t>(1, count));
            point.x = static_cast<float>(sum_x / divisor);
            point.y = static_cast<float>(sum_y / divisor);
            point.z = static_cast<float>(sum_z / divisor);
            return point;
        }
    };

    void validate_parameters() {
        if (!std::isfinite(nominal_sensor_height_) ||
            nominal_sensor_height_ <= 0.0) {
            nominal_sensor_height_ = 0.52;
        }
        if (!std::isfinite(min_obstacle_height_) ||
            !std::isfinite(max_obstacle_height_) ||
            min_obstacle_height_ < 0.0 ||
            max_obstacle_height_ <= min_obstacle_height_) {
            min_obstacle_height_ = 0.10;
            max_obstacle_height_ = 0.85;
        }
        if (!std::isfinite(ground_clearance_thickness_) ||
            ground_clearance_thickness_ <= 0.0 ||
            ground_clearance_thickness_ >= min_obstacle_height_) {
            ground_clearance_thickness_ = 0.06;
        }
        if (!std::isfinite(floor_search_window_) ||
            floor_search_window_ < 0.10) {
            floor_search_window_ = 0.35;
        }
        if (!std::isfinite(floor_bin_size_) ||
            floor_bin_size_ < 0.005 ||
            floor_bin_size_ > 0.10) {
            floor_bin_size_ = 0.02;
        }
        min_floor_points_ = std::max(10, min_floor_points_);
        if (!std::isfinite(min_floor_peak_fraction_) ||
            min_floor_peak_fraction_ <= 0.0 ||
            min_floor_peak_fraction_ > 1.0) {
            min_floor_peak_fraction_ = 0.04;
        }
        if (!std::isfinite(floor_smoothing_alpha_) ||
            floor_smoothing_alpha_ <= 0.0 ||
            floor_smoothing_alpha_ > 1.0) {
            floor_smoothing_alpha_ = 0.70;
        }
        if (!std::isfinite(max_floor_jump_) || max_floor_jump_ <= 0.0) {
            max_floor_jump_ = 0.10;
        }
        if (!std::isfinite(max_floor_rate_) || max_floor_rate_ <= 0.0) {
            max_floor_rate_ = 0.45;
        }
        recovery_confirmation_frames_ =
            std::max(2, recovery_confirmation_frames_);
        if (!std::isfinite(min_floor_range_) || min_floor_range_ < 0.0) {
            min_floor_range_ = 0.40;
        }
        if (!std::isfinite(max_floor_range_) ||
            max_floor_range_ <= min_floor_range_) {
            max_floor_range_ = 12.0;
        }
        minimum_output_points_ = std::max(1, minimum_output_points_);
        if (!std::isfinite(temporal_voxel_size_) ||
            temporal_voxel_size_ < 0.05 || temporal_voxel_size_ > 1.0) {
            temporal_voxel_size_ = 0.15;
        }
        static_confirmation_observations_ =
            std::max(2, static_confirmation_observations_);
        if (!std::isfinite(static_confirmation_seconds_) ||
            static_confirmation_seconds_ < 0.0 ||
            static_confirmation_seconds_ > 60.0) {
            static_confirmation_seconds_ = 1.5;
        }
        ground_confirmation_observations_ = std::max(
            2,
            std::min(
                ground_confirmation_observations_,
                std::max(2, static_confirmation_observations_ - 1)));
        if (!std::isfinite(ground_confirmation_seconds_) ||
            ground_confirmation_seconds_ < 0.0 ||
            ground_confirmation_seconds_ >= static_confirmation_seconds_) {
            ground_confirmation_seconds_ = 0.2;
        }
        max_observation_gap_frames_ = std::max(1, max_observation_gap_frames_);
        candidate_stale_frames_ = std::max(
            max_observation_gap_frames_ + 1, candidate_stale_frames_);
        confirmed_stale_frames_ = std::max(
            candidate_stale_frames_ + 1, confirmed_stale_frames_);
        temporal_prune_interval_frames_ =
            std::max(1, temporal_prune_interval_frames_);
        max_tracked_voxels_ = std::max(1000, max_tracked_voxels_);
    }

    VoxelKey voxel_key(const pcl::PointXYZ &point) const {
        const double inverse = 1.0 / temporal_voxel_size_;
        VoxelKey key;
        key.x = static_cast<int>(
            std::floor(static_cast<double>(point.x) * inverse));
        key.y = static_cast<int>(
            std::floor(static_cast<double>(point.y) * inverse));
        key.z = static_cast<int>(
            std::floor(static_cast<double>(point.z) * inverse));
        return key;
    }

    void reset_temporal_state_if_clock_rewound(const rclcpp::Time &stamp) {
        if (last_cloud_stamp_.nanoseconds() > 0 &&
            stamp.nanoseconds() < last_cloud_stamp_.nanoseconds()) {
            temporal_voxels_.clear();
            confirmed_voxels_ = 0;
            ground_confirmed_voxels_ = 0;
            capacity_exceeded_latched_ = false;
            frame_index_ = 0;
            RCLCPP_WARN(
                get_logger(),
                "ROS time moved backwards; temporal mapping state was reset.");
        }
        last_cloud_stamp_ = stamp;
    }

    void prune_temporal_voxels() {
        for (auto iterator = temporal_voxels_.begin();
             iterator != temporal_voxels_.end();) {
            const auto stale_limit = iterator->second.confirmed
                ? static_cast<std::uint64_t>(confirmed_stale_frames_)
                : static_cast<std::uint64_t>(candidate_stale_frames_);
            if (frame_index_ > iterator->second.last_frame + stale_limit) {
                if (iterator->second.confirmed && confirmed_voxels_ > 0) {
                    --confirmed_voxels_;
                }
                if (iterator->second.confirmed_as_ground &&
                    ground_confirmed_voxels_ > 0) {
                    --ground_confirmed_voxels_;
                }
                iterator = temporal_voxels_.erase(iterator);
            } else {
                ++iterator;
            }
        }
        if (temporal_voxels_.size() <
            static_cast<std::size_t>(max_tracked_voxels_)) {
            capacity_exceeded_latched_ = false;
        }
    }

    bool observe_voxel(
        const VoxelKey &key,
        const rclcpp::Time &stamp,
        const bool ground_candidate,
        bool &capacity_exceeded,
        bool &ground_protected) {
        ground_protected = false;
        auto iterator = temporal_voxels_.find(key);
        if (iterator == temporal_voxels_.end()) {
            if (temporal_voxels_.size() >=
                static_cast<std::size_t>(max_tracked_voxels_)) {
                capacity_exceeded = true;
                capacity_exceeded_latched_ = true;
                return false;
            }
            VoxelState state;
            state.first_frame = frame_index_;
            state.last_frame = frame_index_;
            state.consecutive_observations = 1;
            state.first_stamp_seconds = stamp.seconds();
            if (filter_mode_ == FilterMode::Protected &&
                ground_preservation_enabled_ && ground_candidate) {
                state.last_ground_frame = frame_index_;
                state.consecutive_ground_observations = 1;
                state.first_ground_stamp_seconds = stamp.seconds();
            }
            iterator = temporal_voxels_.emplace(key, state).first;
        } else {
            auto &state = iterator->second;
            if (state.confirmed) {
                state.last_frame = frame_index_;
                ground_protected = state.confirmed_as_ground;
                return true;
            }
            if (!state.confirmed &&
                frame_index_ > state.last_frame +
                    static_cast<std::uint64_t>(max_observation_gap_frames_)) {
                state.first_frame = frame_index_;
                state.consecutive_observations = 1;
                state.first_stamp_seconds = stamp.seconds();
            } else if (state.last_frame != frame_index_ && !state.confirmed) {
                state.consecutive_observations = std::min<std::uint32_t>(
                    std::numeric_limits<std::uint32_t>::max(),
                    state.consecutive_observations + 1);
            }
            state.last_frame = frame_index_;
            if (filter_mode_ == FilterMode::Protected &&
                ground_preservation_enabled_ && ground_candidate) {
                if (state.consecutive_ground_observations == 0 ||
                    frame_index_ > state.last_ground_frame +
                        static_cast<std::uint64_t>(
                            max_observation_gap_frames_)) {
                    state.consecutive_ground_observations = 1;
                    state.first_ground_stamp_seconds = stamp.seconds();
                } else if (state.last_ground_frame != frame_index_) {
                    state.consecutive_ground_observations =
                        std::min<std::uint32_t>(
                            std::numeric_limits<std::uint32_t>::max(),
                            state.consecutive_ground_observations + 1);
                }
                state.last_ground_frame = frame_index_;
            } else {
                state.last_ground_frame = 0;
                state.consecutive_ground_observations = 0;
                state.first_ground_stamp_seconds = stamp.seconds();
            }
        }

        auto &state = iterator->second;
        const bool regular_confirmation =
            state.consecutive_observations >=
                static_cast<std::uint32_t>(static_confirmation_observations_) &&
            stamp.seconds() - state.first_stamp_seconds >=
                static_confirmation_seconds_;
        const bool ground_confirmation =
            filter_mode_ == FilterMode::Protected &&
            ground_preservation_enabled_ &&
            state.consecutive_ground_observations >=
                static_cast<std::uint32_t>(
                    ground_confirmation_observations_) &&
            stamp.seconds() - state.first_ground_stamp_seconds >=
                ground_confirmation_seconds_;
        const bool bypass_confirmation =
            filter_mode_ == FilterMode::Bypass;
        if (!state.confirmed &&
            (bypass_confirmation || regular_confirmation ||
             ground_confirmation)) {
            state.confirmed = true;
            state.confirmed_as_ground =
                ground_confirmation && !regular_confirmation;
            ++confirmed_voxels_;
            if (state.confirmed_as_ground) {
                ++ground_confirmed_voxels_;
            }
        }
        ground_protected = state.confirmed_as_ground;
        return state.confirmed;
    }

    void publish_cloud(
        const pcl::PointCloud<pcl::PointXYZ> &cloud,
        const std_msgs::msg::Header &header,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher) {
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header = header;
        publisher->publish(output);
    }

    FloorMeasurement estimate_floor(
        const pcl::PointCloud<pcl::PointXYZ> &cloud,
        const double sensor_x,
        const double sensor_y,
        const double predicted_floor) const {
        const double search_center =
            floor_initialized_ ? filtered_floor_ : predicted_floor;
        double search_min = search_center - floor_search_window_;
        double search_max = search_center + floor_search_window_;
        if (floor_initialized_) {
            // Also examine a bounded area around the sensor-height prediction.
            // This lets a persistent vertical-only LIO reset recover without
            // accepting an unbounded or unrelated horizontal surface.
            const double bounded_prediction = std::max(
                filtered_floor_ - 0.75,
                std::min(filtered_floor_ + 0.75, predicted_floor));
            search_min = std::min(
                search_min,
                bounded_prediction - floor_search_window_);
            search_max = std::max(
                search_max,
                bounded_prediction + floor_search_window_);
        }
        const std::size_t bin_count = std::max<std::size_t>(
            3,
            static_cast<std::size_t>(
                std::ceil((search_max - search_min) / floor_bin_size_)));
        std::vector<std::size_t> histogram(bin_count, 0);
        std::size_t candidate_count = 0;

        for (const auto &point : cloud.points) {
            if (!pcl::isFinite(point)) {
                continue;
            }
            const double dx = static_cast<double>(point.x) - sensor_x;
            const double dy = static_cast<double>(point.y) - sensor_y;
            const double planar_range = std::hypot(dx, dy);
            if (planar_range < min_floor_range_ ||
                planar_range > max_floor_range_ ||
                point.z < search_min ||
                point.z > search_max) {
                continue;
            }
            const auto index = std::min<std::size_t>(
                bin_count - 1,
                static_cast<std::size_t>(
                    (static_cast<double>(point.z) - search_min) /
                    floor_bin_size_));
            ++histogram[index];
            ++candidate_count;
        }

        FloorMeasurement measurement;
        measurement.candidates = candidate_count;
        if (candidate_count == 0) {
            return measurement;
        }

        const auto peak_iterator =
            std::max_element(histogram.begin(), histogram.end());
        const std::size_t peak_index =
            static_cast<std::size_t>(peak_iterator - histogram.begin());
        const std::size_t peak_support = *peak_iterator;
        const double peak_fraction =
            static_cast<double>(peak_support) /
            static_cast<double>(candidate_count);
        if (peak_support < static_cast<std::size_t>(min_floor_points_) ||
            peak_fraction < min_floor_peak_fraction_) {
            return measurement;
        }

        const double peak_center =
            search_min +
            (static_cast<double>(peak_index) + 0.5) * floor_bin_size_;
        const double refine_half_width = floor_bin_size_ * 1.5;
        std::vector<float> peak_heights;
        peak_heights.reserve(peak_support * 2);
        for (const auto &point : cloud.points) {
            if (!pcl::isFinite(point)) {
                continue;
            }
            const double dx = static_cast<double>(point.x) - sensor_x;
            const double dy = static_cast<double>(point.y) - sensor_y;
            const double planar_range = std::hypot(dx, dy);
            if (planar_range >= min_floor_range_ &&
                planar_range <= max_floor_range_ &&
                std::abs(static_cast<double>(point.z) - peak_center) <=
                    refine_half_width) {
                peak_heights.push_back(point.z);
            }
        }
        if (peak_heights.size() <
            static_cast<std::size_t>(min_floor_points_)) {
            return measurement;
        }

        const auto median_iterator =
            peak_heights.begin() + peak_heights.size() / 2;
        std::nth_element(
            peak_heights.begin(), median_iterator, peak_heights.end());
        measurement.valid = true;
        measurement.height = static_cast<double>(*median_iterator);
        measurement.support = peak_heights.size();
        return measurement;
    }

    bool update_floor(
        const FloorMeasurement &measurement,
        const rclcpp::Time &stamp) {
        if (!measurement.valid) {
            pending_floor_count_ = 0;
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Mapping scan skipped: no reliable floor peak was found. "
                "The previous map is kept unchanged.");
            publish_status("waiting_for_floor", measurement, 0);
            return false;
        }

        if (!floor_initialized_) {
            filtered_floor_ = measurement.height;
            last_measurement_stamp_ = stamp;
            floor_initialized_ = true;
            pending_floor_count_ = 0;
            return true;
        }

        const double dt = std::max(
            0.001, (stamp - last_measurement_stamp_).seconds());
        last_measurement_stamp_ = stamp;
        const double allowed_change =
            max_floor_jump_ + max_floor_rate_ * std::min(dt, 1.0);
        if (std::abs(measurement.height - filtered_floor_) >
            allowed_change) {
            if (pending_floor_count_ == 0 ||
                std::abs(measurement.height - pending_floor_) >
                    floor_bin_size_ * 2.0) {
                pending_floor_ = measurement.height;
                pending_floor_count_ = 1;
            } else {
                pending_floor_ =
                    0.5 * pending_floor_ + 0.5 * measurement.height;
                ++pending_floor_count_;
            }
            if (pending_floor_count_ < recovery_confirmation_frames_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Mapping scan skipped: floor estimate jumped from %.3f "
                    "to %.3f m (%d/%d confirmation frames).",
                    filtered_floor_,
                    measurement.height,
                    pending_floor_count_,
                    recovery_confirmation_frames_);
                publish_status("confirming_floor_jump", measurement, 0);
                return false;
            }

            filtered_floor_ = pending_floor_;
            pending_floor_count_ = 0;
            RCLCPP_WARN(
                get_logger(),
                "Accepted a persistent vertical-only floor shift; mapping "
                "continues automatically with floor z=%.3f m.",
                filtered_floor_);
            return true;
        }

        pending_floor_count_ = 0;
        filtered_floor_ =
            floor_smoothing_alpha_ * measurement.height +
            (1.0 - floor_smoothing_alpha_) * filtered_floor_;
        return true;
    }

    void cloud_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr message) {
        if (message->header.frame_id != world_frame_) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Mapping cloud frame is '%s', expected '%s'. Scan skipped "
                "instead of risking a corrupted map.",
                message->header.frame_id.c_str(),
                world_frame_.c_str());
            return;
        }

        geometry_msgs::msg::TransformStamped sensor_transform;
        const rclcpp::Time stamp(message->header.stamp);
        try {
            sensor_transform = tf_buffer_.lookupTransform(
                world_frame_,
                sensor_frame_,
                stamp,
                rclcpp::Duration::from_seconds(0.05));
        } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Mapping scan skipped while waiting for %s -> %s TF: %s",
                world_frame_.c_str(),
                sensor_frame_.c_str(),
                error.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg(*message, input_cloud);
        reset_temporal_state_if_clock_rewound(stamp);
        ++frame_index_;
        if (frame_index_ %
                static_cast<std::uint64_t>(temporal_prune_interval_frames_) == 0 ||
            temporal_voxels_.size() >=
                static_cast<std::size_t>(max_tracked_voxels_)) {
            prune_temporal_voxels();
        }
        const double sensor_x = sensor_transform.transform.translation.x;
        const double sensor_y = sensor_transform.transform.translation.y;
        const double sensor_z = sensor_transform.transform.translation.z;
        const double predicted_floor =
            sensor_z - nominal_sensor_height_;
        const FloorMeasurement measurement = estimate_floor(
            input_cloud, sensor_x, sensor_y, predicted_floor);
        if (!update_floor(measurement, stamp)) {
            return;
        }

        std::unordered_map<VoxelKey, FrameVoxel, VoxelKeyHasher> frame_voxels;
        frame_voxels.reserve(input_cloud.points.size() / 4 + 1);
        std::size_t ground_points = 0;
        std::size_t obstacle_points = 0;
        for (const auto &point : input_cloud.points) {
            if (!pcl::isFinite(point)) {
                continue;
            }
            const double relative_height =
                static_cast<double>(point.z) - filtered_floor_;
            const bool is_ground =
                std::abs(relative_height) <= ground_clearance_thickness_;
            const bool is_obstacle =
                relative_height >= min_obstacle_height_ &&
                relative_height <= max_obstacle_height_;
            if (!is_ground && !is_obstacle) {
                continue;
            }
            frame_voxels[voxel_key(point)].add(point, is_ground);
            ground_points += is_ground ? 1U : 0U;
            obstacle_points += is_obstacle ? 1U : 0U;
        }

        pcl::PointCloud<pcl::PointXYZ> static_cloud;
        pcl::PointCloud<pcl::PointXYZ> dynamic_cloud;
        const bool publish_dynamic_cloud =
            dynamic_cloud_publisher_->get_subscription_count() > 0U;
        std::size_t dynamic_points = 0;
        std::size_t ground_protected_voxels = 0;
        static_cloud.header = input_cloud.header;
        static_cloud.points.reserve(frame_voxels.size());
        if (publish_dynamic_cloud) {
            dynamic_cloud.header = input_cloud.header;
            dynamic_cloud.points.reserve(frame_voxels.size());
        }
        bool capacity_exceeded = false;
        for (const auto &entry : frame_voxels) {
            bool ground_protected = false;
            if (observe_voxel(
                    entry.first,
                    stamp,
                    entry.second.is_ground_candidate(),
                    capacity_exceeded,
                    ground_protected)) {
                auto mapping_point = entry.second.centroid();
                // Normalize only the persistent mapping copy. Dynamic debug
                // points retain odom-frame z so the browser can overlay the
                // live robot pose correctly.
                mapping_point.z = static_cast<float>(
                    static_cast<double>(mapping_point.z) - filtered_floor_);
                static_cloud.points.push_back(mapping_point);
                ground_protected_voxels += ground_protected ? 1U : 0U;
            } else {
                ++dynamic_points;
                if (publish_dynamic_cloud) {
                    dynamic_cloud.points.push_back(
                        entry.second.centroid());
                }
            }
        }

        static_cloud.width =
            static_cast<std::uint32_t>(static_cloud.points.size());
        static_cloud.height = 1;
        static_cloud.is_dense = false;
        if (publish_dynamic_cloud) {
            dynamic_cloud.width =
                static_cast<std::uint32_t>(dynamic_cloud.points.size());
            dynamic_cloud.height = 1;
            dynamic_cloud.is_dense = false;
            // Include empty frames while subscribed so a live viewer clears
            // stale moving obstacles. With no subscriber, skip constructing
            // and serializing this display-only cloud entirely.
            publish_cloud(dynamic_cloud, message->header, dynamic_cloud_publisher_);
        }
        capacity_exceeded = capacity_exceeded || capacity_exceeded_latched_;

        if (static_cloud.points.size() <
            static_cast<std::size_t>(minimum_output_points_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Persistent mapping scan is warming up: %zu static and %zu "
                "dynamic/candidate voxels.",
                static_cloud.points.size(),
                dynamic_points);
            publish_status(
                capacity_exceeded ? "capacity_exceeded" : "confirming_static_voxels",
                measurement,
                static_cloud.points.size(),
                ground_points,
                obstacle_points,
                input_cloud.points.size(),
                dynamic_points,
                frame_voxels.size(),
                capacity_exceeded,
                ground_protected_voxels);
            return;
        }
        publish_cloud(static_cloud, message->header, filtered_cloud_publisher_);
        publish_status(
            capacity_exceeded ? "capacity_exceeded" : "healthy",
            measurement,
            static_cloud.points.size(),
            ground_points,
            obstacle_points,
            input_cloud.points.size(),
            dynamic_points,
            frame_voxels.size(),
            capacity_exceeded,
            ground_protected_voxels);
    }

    void publish_status(
        const std::string &state,
        const FloorMeasurement &measurement,
        const std::size_t output_points,
        const std::size_t ground_points = 0,
        const std::size_t obstacle_points = 0,
        const std::size_t input_points = 0,
        const std::size_t dynamic_points = 0,
        const std::size_t relevant_voxels = 0,
        const bool capacity_exceeded = false,
        const std::size_t ground_protected_voxels = 0) {
        const auto now = get_clock()->now();
        if (last_status_publish_time_.nanoseconds() > 0 &&
            (now - last_status_publish_time_).seconds() < 1.0) {
            return;
        }
        last_status_publish_time_ = now;
        const double filtered_ratio = relevant_voxels > 0
            ? static_cast<double>(dynamic_points) /
                static_cast<double>(relevant_voxels)
            : 0.0;
        const bool dynamic_cloud_subscribed =
            dynamic_cloud_publisher_->get_subscription_count() > 0U;
        std::ostringstream payload;
        payload << "{\"state\":\"" << state << "\","
                << "\"floor_initialized\":"
                << (floor_initialized_ ? "true" : "false") << ","
                << "\"filter_mode\":\""
                << dynamic_filter_mode_name_ << "\","
                << "\"temporal_bypass_active\":"
                << (filter_mode_ == FilterMode::Bypass ? "true" : "false")
                << ","
                << "\"floor_z\":"
                << (floor_initialized_ ? filtered_floor_ : 0.0) << ","
                << "\"floor_support\":" << measurement.support << ","
                << "\"floor_candidates\":" << measurement.candidates << ","
                << "\"input_points\":" << input_points << ","
                << "\"relevant_voxels\":" << relevant_voxels << ","
                << "\"static_points\":" << output_points << ","
                << "\"dynamic_points\":" << dynamic_points << ","
                << "\"filtered_ratio\":" << filtered_ratio << ","
                << "\"tracked_voxels\":" << temporal_voxels_.size() << ","
                << "\"confirmed_voxels\":" << confirmed_voxels_ << ","
                << "\"ground_confirmed_voxels\":"
                << ground_confirmed_voxels_ << ","
                << "\"ground_protected_voxels\":"
                << ground_protected_voxels << ","
                << "\"candidate_voxels\":"
                << (temporal_voxels_.size() - confirmed_voxels_) << ","
                << "\"capacity_exceeded\":"
                << (capacity_exceeded ? "true" : "false") << ","
                << "\"dynamic_cloud_subscribed\":"
                << (dynamic_cloud_subscribed ? "true" : "false") << ","
                << "\"output_points\":" << output_points << ","
                << "\"ground_points\":" << ground_points << ","
                << "\"obstacle_points\":" << obstacle_points << "}";
        std_msgs::msg::String status;
        status.data = payload.str();
        status_publisher_->publish(status);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string dynamic_output_topic_;
    std::string dynamic_filter_mode_name_{"protected"};
    FilterMode filter_mode_{FilterMode::Protected};
    std::string world_frame_;
    std::string sensor_frame_;
    double nominal_sensor_height_{0.52};
    double min_obstacle_height_{0.10};
    double max_obstacle_height_{0.85};
    double ground_clearance_thickness_{0.06};
    double floor_search_window_{0.35};
    double floor_bin_size_{0.02};
    int min_floor_points_{80};
    double min_floor_peak_fraction_{0.04};
    double floor_smoothing_alpha_{0.70};
    double max_floor_jump_{0.10};
    double max_floor_rate_{0.45};
    int recovery_confirmation_frames_{5};
    double min_floor_range_{0.40};
    double max_floor_range_{12.0};
    int minimum_output_points_{20};
    double temporal_voxel_size_{0.15};
    int static_confirmation_observations_{8};
    double static_confirmation_seconds_{1.5};
    bool ground_preservation_enabled_{true};
    int ground_confirmation_observations_{3};
    double ground_confirmation_seconds_{0.2};
    int max_observation_gap_frames_{3};
    int candidate_stale_frames_{30};
    int confirmed_stale_frames_{1800};
    int temporal_prune_interval_frames_{30};
    int max_tracked_voxels_{500000};

    bool floor_initialized_{false};
    double filtered_floor_{0.0};
    rclcpp::Time last_measurement_stamp_{0, 0, RCL_ROS_TIME};
    double pending_floor_{0.0};
    int pending_floor_count_{0};
    rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
    std::uint64_t frame_index_{0};
    std::size_t confirmed_voxels_{0};
    std::size_t ground_confirmed_voxels_{0};
    bool capacity_exceeded_latched_{false};
    std::unordered_map<VoxelKey, VoxelState, VoxelKeyHasher> temporal_voxels_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        filtered_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        dynamic_cloud_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};

}  // namespace octomap_server

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<octomap_server::MappingCloudFilter>());
    rclcpp::shutdown();
    return 0;
}
