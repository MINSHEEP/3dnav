#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/point_tests.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace octomap_server {

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

        validate_parameters();

        filtered_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                output_topic_, rclcpp::SensorDataQoS());
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
            "Mapping-only cloud protection enabled: %s -> %s; dynamic "
            "obstacle band %.2f..%.2f m above the estimated floor. "
            "Navigation topics are unchanged.",
            input_topic_.c_str(),
            output_topic_.c_str(),
            min_obstacle_height_,
            max_obstacle_height_);
    }

private:
    struct FloorMeasurement {
        bool valid{false};
        double height{0.0};
        std::size_t support{0};
        std::size_t candidates{0};
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

        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        filtered_cloud.header = input_cloud.header;
        filtered_cloud.points.reserve(input_cloud.points.size() / 2 + 1);
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
            auto output_point = point;
            // Normalize only the mapping copy of the cloud so its detected
            // floor is always z=0. OctoMap can then treat ground returns as
            // free-space rays and obstacle returns as occupied endpoints.
            output_point.z = static_cast<float>(relative_height);
            filtered_cloud.points.push_back(output_point);
            ground_points += is_ground ? 1U : 0U;
            obstacle_points += is_obstacle ? 1U : 0U;
        }
        if (filtered_cloud.points.size() <
            static_cast<std::size_t>(minimum_output_points_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Mapping scan skipped: only %zu points remain in "
                "the protected ground/obstacle cloud.",
                filtered_cloud.points.size());
            publish_status(
                "insufficient_mapping_points",
                measurement,
                filtered_cloud.points.size());
            return;
        }

        filtered_cloud.width =
            static_cast<std::uint32_t>(filtered_cloud.points.size());
        filtered_cloud.height = 1;
        filtered_cloud.is_dense = false;
        sensor_msgs::msg::PointCloud2 output_message;
        pcl::toROSMsg(filtered_cloud, output_message);
        output_message.header = message->header;
        filtered_cloud_publisher_->publish(output_message);
        publish_status(
            "healthy",
            measurement,
            filtered_cloud.points.size(),
            ground_points,
            obstacle_points);
    }

    void publish_status(
        const std::string &state,
        const FloorMeasurement &measurement,
        const std::size_t output_points,
        const std::size_t ground_points = 0,
        const std::size_t obstacle_points = 0) {
        const auto now = get_clock()->now();
        if (last_status_publish_time_.nanoseconds() > 0 &&
            (now - last_status_publish_time_).seconds() < 1.0) {
            return;
        }
        last_status_publish_time_ = now;
        std::ostringstream payload;
        payload << "{\"state\":\"" << state << "\","
                << "\"floor_initialized\":"
                << (floor_initialized_ ? "true" : "false") << ","
                << "\"floor_z\":"
                << (floor_initialized_ ? filtered_floor_ : 0.0) << ","
                << "\"floor_support\":" << measurement.support << ","
                << "\"floor_candidates\":" << measurement.candidates << ","
                << "\"output_points\":" << output_points << ","
                << "\"ground_points\":" << ground_points << ","
                << "\"obstacle_points\":" << obstacle_points << "}";
        std_msgs::msg::String status;
        status.data = payload.str();
        status_publisher_->publish(status);
    }

    std::string input_topic_;
    std::string output_topic_;
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

    bool floor_initialized_{false};
    double filtered_floor_{0.0};
    rclcpp::Time last_measurement_stamp_{0, 0, RCL_ROS_TIME};
    double pending_floor_{0.0};
    int pending_floor_count_{0};
    rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        filtered_cloud_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};

}  // namespace octomap_server

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<octomap_server::MappingCloudFilter>());
    rclcpp::shutdown();
    return 0;
}
