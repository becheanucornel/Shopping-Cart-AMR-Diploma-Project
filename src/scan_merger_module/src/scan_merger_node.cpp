#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// TF2 includes
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;

class ScanMerger : public rclcpp::Node {
public:
    ScanMerger() : Node("scan_merger") {
        this->declare_parameter<std::string>("output_topic", "/scan");
        this->declare_parameter<std::string>("output_frame_id", "custom_base_link");
        this->declare_parameter<std::vector<std::string>>("merge_topics", std::vector<std::string>{
            "/lidar_front_left/scan", "/lidar_front_right/scan", 
            "/lidar_back_left/scan", "/lidar_back_right/scan"
        });

        output_topic_ = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        topics_ = this->get_parameter("merge_topics").as_string_array();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

        latest_scans_.resize(topics_.size(), nullptr);
        cached_transforms_.resize(topics_.size());
        transform_is_cached_.resize(topics_.size(), false);
        subs_.reserve(topics_.size());

        for (size_t i = 0; i < topics_.size(); ++i) {
            auto callback = [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->latest_scans_[i] = msg;
            };
            subs_.push_back(this->create_subscription<sensor_msgs::msg::LaserScan>(topics_[i], 10, callback));
        }

        // --- PRE-ALLOCATE THE MERGED MESSAGE ONCE ---
        const double resolution_deg = 0.5; 
        merged_msg_.angle_increment = resolution_deg * M_PI / 180.0;
        merged_msg_.angle_min = -M_PI;
        merged_msg_.angle_max = M_PI;
        merged_msg_.range_min = 0.05; 
        merged_msg_.range_max = 12.0; 
        merged_msg_.header.frame_id = output_frame_id_;
        
        num_points_ = std::ceil((merged_msg_.angle_max - merged_msg_.angle_min) / merged_msg_.angle_increment);
        merged_msg_.ranges.assign(num_points_, std::numeric_limits<float>::infinity());

        timer_ = this->create_wall_timer(50ms, std::bind(&ScanMerger::publish_merged_scan, this));
        RCLCPP_INFO(this->get_logger(), "Optimized Scan Merger initialized!");
    }

private:
    void publish_merged_scan() {
        bool has_data = false;
        for (const auto& scan : latest_scans_) {
            if (scan != nullptr) {
                has_data = true;
                break;
            }
        }
        if (!has_data) return;

        merged_msg_.header.stamp = this->now();
        // Fast memory overwrite instead of reallocation
        std::fill(merged_msg_.ranges.begin(), merged_msg_.ranges.end(), std::numeric_limits<float>::infinity());

        for (size_t i = 0; i < latest_scans_.size(); ++i) {
            const auto& scan = latest_scans_[i];
            if (!scan) continue;

            // --- TF CACHING LOGIC ---
            if (!transform_is_cached_[i]) {
                try {
                    geometry_msgs::msg::TransformStamped transform_msg = tf_buffer_->lookupTransform(
                        output_frame_id_, scan->header.frame_id, tf2::TimePointZero);
                    
                    tf2::Transform transform;
                    transform.setOrigin(tf2::Vector3(
                        transform_msg.transform.translation.x,
                        transform_msg.transform.translation.y,
                        transform_msg.transform.translation.z
                    ));
                    tf2::Quaternion q(
                        transform_msg.transform.rotation.x, transform_msg.transform.rotation.y,
                        transform_msg.transform.rotation.z, transform_msg.transform.rotation.w
                    );
                    transform.setRotation(q);
                    
                    cached_transforms_[i] = transform;
                    transform_is_cached_[i] = true;
                    RCLCPP_INFO(this->get_logger(), "Successfully cached static transform for %s", scan->header.frame_id.c_str());
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Error: %s", ex.what());
                    continue;
                }
            }

            const tf2::Transform& transform = cached_transforms_[i];

            // --- MATH LOOP ---
            for (size_t j = 0; j < scan->ranges.size(); ++j) {
                float r = scan->ranges[j];
                if (r < scan->range_min || r > scan->range_max || std::isnan(r) || std::isinf(r)) continue;

                double angle = scan->angle_min + j * scan->angle_increment;
                double x = r * std::cos(angle);
                double y = r * std::sin(angle);

                tf2::Vector3 point_local(x, y, 0.0);
                tf2::Vector3 point_base = transform * point_local;

                double merged_r = std::hypot(point_base.x(), point_base.y());
                double merged_angle = std::atan2(point_base.y(), point_base.x());

                if (merged_r >= merged_msg_.range_min && merged_r <= merged_msg_.range_max) {
                    int index = std::round((merged_angle - merged_msg_.angle_min) / merged_msg_.angle_increment);
                    if (index >= 0 && index < num_points_) {
                        if (merged_r < merged_msg_.ranges[index]) {
                            merged_msg_.ranges[index] = merged_r;
                        }
                    }
                }
            }
        }
        pub_->publish(merged_msg_);
    }

    std::vector<std::string> topics_;
    std::string output_topic_;
    std::string output_frame_id_;
    int num_points_;

    sensor_msgs::msg::LaserScan merged_msg_; // Persistent memory

    std::vector<sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;
    std::vector<tf2::Transform> cached_transforms_;
    std::vector<bool> transform_is_cached_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}