#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <algorithm> // for std::min, std::max
#include <cmath>     // for std::isnan
#include <limits>    // for std::numeric_limits

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class ScanMerger : public rclcpp::Node {
public:
    ScanMerger() : Node("scan_merger") {
        // NOTE: The old implementation attempted to "merge" multiple 360deg scans by taking
        // the minimum range per index without TF transformation. In Isaac Sim this also
        // resulted in invalid frame_ids (topic names) being forwarded to Nav2/AMCL.
        //
        // Default behavior is now a safe passthrough of a single scan topic.
        this->declare_parameter<std::string>("strategy", "passthrough");
        this->declare_parameter<std::string>("passthrough_topic", "/lidar_front/scan");
        this->declare_parameter<std::string>("output_topic", "/scan");
        this->declare_parameter<std::string>("output_frame_id", "base_link");
        this->declare_parameter<std::vector<std::string>>("merge_topics", std::vector<std::string>{
            "/lidar_front/scan",
            "/lidar_left/scan",
            "/lidar_right/scan",
            "/lidar_rear/scan",
        });

        strategy_ = this->get_parameter("strategy").as_string();
        passthrough_topic_ = this->get_parameter("passthrough_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        topics_ = this->get_parameter("merge_topics").as_string_array();

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

        if (strategy_ == "passthrough") {
            passthrough_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                passthrough_topic_, 10,
                std::bind(&ScanMerger::passthrough_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "scan_merger: passthrough %s -> %s frame_id=%s",
                        passthrough_topic_.c_str(), output_topic_.c_str(), output_frame_id_.c_str());
            return;
        }

        // Legacy strategy: min_merge (kept for compatibility, but requires inputs to be
        // in a common frame and aligned in angle indexing)
        latest_scans_.resize(topics_.size(), nullptr);
        subs_.reserve(topics_.size());

        for (size_t i = 0; i < topics_.size(); ++i) {
            auto callback = [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->scan_callback(msg, i);
            };
            subs_.push_back(
                this->create_subscription<sensor_msgs::msg::LaserScan>(topics_[i], 10, callback));
        }

        timer_ = this->create_timer(15ms, std::bind(&ScanMerger::publish_merged_scan, this));
        RCLCPP_WARN(this->get_logger(), "scan_merger: using legacy strategy '%s'", strategy_.c_str());
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg, size_t idx) {
        // Store the pointer to the message
        latest_scans_[idx] = msg;
    }

    void passthrough_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!pub_) return;
        auto out = *msg;
        if (!output_frame_id_.empty()) {
            out.header.frame_id = output_frame_id_;
        }
        pub_->publish(out);
    }

    void publish_merged_scan() {
        // Check if we have received at least one message for every topic
        for (const auto& scan : latest_scans_) {
            if (!scan) return; // If any pointer is null, return
        }

        // Reference scan (index 0)
        auto ref_scan = latest_scans_[0];

        auto merged = sensor_msgs::msg::LaserScan();

        // Copy Metadata
        merged.header = ref_scan->header;
        if (!output_frame_id_.empty()) {
            merged.header.frame_id = output_frame_id_;
        }
        merged.angle_min = ref_scan->angle_min;
        merged.angle_max = ref_scan->angle_max;
        merged.angle_increment = ref_scan->angle_increment;
        merged.time_increment = ref_scan->time_increment;
        merged.scan_time = ref_scan->scan_time;

        // Calculate global Min/Max Range
        merged.range_min = ref_scan->range_min;
        merged.range_max = ref_scan->range_max;

        for (const auto& scan : latest_scans_) {
            if (scan->range_min < merged.range_min) merged.range_min = scan->range_min;
            if (scan->range_max > merged.range_max) merged.range_max = scan->range_max;
        }

        // Merge Ranges
        size_t num_points = ref_scan->ranges.size();
        merged.ranges.reserve(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            std::vector<float> valid_values;
            valid_values.reserve(latest_scans_.size());

            for (const auto& scan : latest_scans_) {
                // Safety check for array bounds
                if (i >= scan->ranges.size()) continue;

                float v = scan->ranges[i];

                // Logic: Check bounds and NaN
                bool is_valid = (v > scan->range_min) && 
                                (v < scan->range_max) && 
                                (!std::isnan(v));

                if (is_valid) {
                    valid_values.push_back(v);
                }
            }

            if (!valid_values.empty()) {
                // Use minimum distance found (closest obstacle)
                merged.ranges.push_back(*std::min_element(valid_values.begin(), valid_values.end()));
            } else {
                // No valid data = Infinity
                merged.ranges.push_back(std::numeric_limits<float>::infinity());
            }
        }

        // Handle Intensities (optional, creates zeros if ref has intensities)
        if (!ref_scan->intensities.empty()) {
            merged.intensities.resize(num_points, 0.0f);
        }

        pub_->publish(merged);
    }

    // --- Member Variables ---

    std::vector<std::string> topics_;

    std::string strategy_;
    std::string passthrough_topic_;
    std::string output_topic_;
    std::string output_frame_id_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr passthrough_sub_;

    // Holds the latest laser scan messages (pointers can be null)
    std::vector<sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_;

    // Holds the subscriptions to keep them alive
    // FIXED: Added ::SharedPtr to the type definition
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}