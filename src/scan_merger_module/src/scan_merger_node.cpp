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
            "/lidar_back_left/scan",  "/lidar_back_right/scan"
        });
        this->declare_parameter<double>("mask_radius", 0.30);
        this->declare_parameter<double>("resolution_deg", 0.5);

        output_topic_    = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        topics_          = this->get_parameter("merge_topics").as_string_array();
        mask_radius_     = static_cast<float>(this->get_parameter("mask_radius").as_double());

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        pub_         = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

        latest_scans_.resize(topics_.size(), nullptr);
        cached_transforms_.resize(topics_.size());
        transform_is_cached_.resize(topics_.size(), false);
        subs_.reserve(topics_.size());

        for (size_t i = 0; i < topics_.size(); ++i) {
            auto callback = [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                latest_scans_[i] = msg;
            };
            subs_.push_back(this->create_subscription<sensor_msgs::msg::LaserScan>(
                topics_[i], 10, callback));
        }

        const double resolution_deg = this->get_parameter("resolution_deg").as_double();
        merged_msg_.angle_increment = resolution_deg * M_PI / 180.0;
        merged_msg_.angle_min       = -M_PI;
        merged_msg_.angle_max       =  M_PI;
        merged_msg_.range_min       = 0.05f;
        merged_msg_.range_max       = 12.0f;
        merged_msg_.header.frame_id = output_frame_id_;

        num_points_ = static_cast<int>(
            std::ceil((merged_msg_.angle_max - merged_msg_.angle_min) / merged_msg_.angle_increment));
        merged_msg_.ranges.assign(num_points_, std::numeric_limits<float>::infinity());

        timer_ = this->create_wall_timer(50ms, std::bind(&ScanMerger::publish_merged_scan, this));
        RCLCPP_INFO(this->get_logger(), "Optimized Scan Merger initialized!");
    }

private:
    void publish_merged_scan()
    {
        // Verifica daca avem cel putin un scan
        bool has_data = false;
        for (const auto & scan : latest_scans_) {
            if (scan != nullptr) { has_data = true; break; }
        }
        if (!has_data) return;

        // Use the timestamp of the most recent received scan so that TF lookups
        // in SLAM/AMCL find a valid odom→base transform at exactly that time.
        rclcpp::Time latest_stamp(0, 0, RCL_ROS_TIME);
        for (const auto & scan : latest_scans_) {
            if (scan) {
                rclcpp::Time t = scan->header.stamp;
                if (t > latest_stamp) latest_stamp = t;
            }
        }
        merged_msg_.header.stamp = latest_stamp;

        // Reset ranges
        std::fill(merged_msg_.ranges.begin(), merged_msg_.ranges.end(),
                  std::numeric_limits<float>::infinity());

        for (size_t i = 0; i < latest_scans_.size(); ++i) {
            const auto & scan = latest_scans_[i];
            if (!scan) continue;

            // ── TF caching — static transforms nu se schimba niciodata ─────
            if (!transform_is_cached_[i]) {
                try {
                    geometry_msgs::msg::TransformStamped tf_msg =
                        tf_buffer_->lookupTransform(
                            output_frame_id_,
                            scan->header.frame_id,
                            tf2::TimePointZero);

                    tf2::Transform transform;
                    transform.setOrigin(tf2::Vector3(
                        tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                        tf_msg.transform.translation.z));
                    tf2::Quaternion q(
                        tf_msg.transform.rotation.x,
                        tf_msg.transform.rotation.y,
                        tf_msg.transform.rotation.z,
                        tf_msg.transform.rotation.w);
                    transform.setRotation(q);

                    cached_transforms_[i]   = transform;
                    transform_is_cached_[i] = true;
                    RCLCPP_INFO(this->get_logger(),
                        "Cached transform for %s", scan->header.frame_id.c_str());
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_SKIPFIRST_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "TF Error pentru %s: %s", scan->header.frame_id.c_str(), ex.what());
                    continue;
                }
            }

            const tf2::Transform & transform = cached_transforms_[i];

            for (size_t j = 0; j < scan->ranges.size(); ++j) {
                float r = scan->ranges[j];

                // Filtreaza valori invalide
                if (!std::isfinite(r)) continue;
                if (r < scan->range_min || r > scan->range_max) continue;

                double angle = scan->angle_min + j * scan->angle_increment;
                double lx = r * std::cos(angle);
                double ly = r * std::sin(angle);

                // ── FIX: mask_radius parametrizabil, ignora corpul robotului
                if (std::hypot(lx, ly) < mask_radius_) continue;

                tf2::Vector3 point_local(lx, ly, 0.0);
                tf2::Vector3 point_base = transform * point_local;

                double merged_r     = std::hypot(point_base.x(), point_base.y());
                double merged_angle = std::atan2(point_base.y(), point_base.x());

                if (merged_r < merged_msg_.range_min || merged_r > merged_msg_.range_max) continue;

                int index = static_cast<int>(std::round(
                    (merged_angle - merged_msg_.angle_min) / merged_msg_.angle_increment));

                if (index < 0 || index >= num_points_) continue;

                // Pastreaza cel mai apropiat punct per unghi (min range)
                if (static_cast<float>(merged_r) < merged_msg_.ranges[index]) {
                    merged_msg_.ranges[index] = static_cast<float>(merged_r);
                }
            }
        }

        pub_->publish(merged_msg_);
    }

    // ── Members ─────────────────────────────────────────────────────────────
    std::vector<std::string> topics_;
    std::string output_topic_;
    std::string output_frame_id_;
    float mask_radius_;
    int num_points_;

    sensor_msgs::msg::LaserScan merged_msg_;

    std::vector<sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;
    std::vector<tf2::Transform> cached_transforms_;
    std::vector<bool> transform_is_cached_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}