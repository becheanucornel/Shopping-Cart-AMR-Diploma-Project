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
        this->declare_parameter<std::string>("output_frame_id", "base_link");
        this->declare_parameter<std::vector<std::string>>("merge_topics", std::vector<std::string>{
            "/lidar_front_left/scan",
            "/lidar_front_right/scan",
            "/lidar_rear_left/scan",
            "/lidar_rear_right/scan"
        });

        output_topic_ = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        topics_ = this->get_parameter("merge_topics").as_string_array();

        // Inițializare TF Buffer și Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

        latest_scans_.resize(topics_.size(), nullptr);
        subs_.reserve(topics_.size());

        for (size_t i = 0; i < topics_.size(); ++i) {
            auto callback = [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->latest_scans_[i] = msg;
            };
            subs_.push_back(
                this->create_subscription<sensor_msgs::msg::LaserScan>(topics_[i], 10, callback));
        }

        // Timer-ul rulează la 20Hz (50ms). Se poate ajusta în funcție de rotația RPLIDAR-ului (ex: 10Hz)
        timer_ = this->create_wall_timer(50ms, std::bind(&ScanMerger::publish_merged_scan, this));
        
        RCLCPP_INFO(this->get_logger(), "Scan Merger inițializat! Aștept date...");
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

        if (!has_data) return; // Nu avem niciun scan încă

        // Configurăm scanul final (360 grade)
        auto merged = sensor_msgs::msg::LaserScan();
        merged.header.stamp = this->now();
        merged.header.frame_id = output_frame_id_;
        
        // Setup rezoluție: RPLIDAR C1 are o rezoluție tipică destul de bună.
        // Aici definim rezoluția scanului centralizat (ex. 0.5 grade -> ~0.0087 rad)
        const double resolution_deg = 0.5; 
        merged.angle_increment = resolution_deg * M_PI / 180.0;
        merged.angle_min = -M_PI;
        merged.angle_max = M_PI;
        merged.range_min = 0.05; // 5 cm
        merged.range_max = 12.0; // 12 m (specificație aprox C1)
        
        size_t num_points = std::ceil((merged.angle_max - merged.angle_min) / merged.angle_increment);
        merged.ranges.assign(num_points, std::numeric_limits<float>::infinity());

        // Parcurgem fiecare scan primit
        for (const auto& scan : latest_scans_) {
            if (!scan) continue;

            // Obținem transformarea de la cadrul senzorului curent la base_link
            geometry_msgs::msg::TransformStamped transform_msg;
            try {
                transform_msg = tf_buffer_->lookupTransform(
                    output_frame_id_, scan->header.frame_id, 
                    tf2::TimePointZero); // Luăm cel mai recent TF
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "Eroare TF: %s", ex.what());
                continue;
            }

            // Construim transformarea manual pentru a evita erorile de linker tf2_geometry_msgs
            tf2::Transform transform;
                transform.setOrigin(tf2::Vector3(
                transform_msg.transform.translation.x,
                transform_msg.transform.translation.y,
                transform_msg.transform.translation.z
            ));
            tf2::Quaternion q(
                transform_msg.transform.rotation.x,
                transform_msg.transform.rotation.y,
                transform_msg.transform.rotation.z,
                transform_msg.transform.rotation.w
            );
            transform.setRotation(q);

            // Parcurgem toate punctele din scanul curent
            for (size_t i = 0; i < scan->ranges.size(); ++i) {
                float r = scan->ranges[i];
                if (r < scan->range_min || r > scan->range_max || std::isnan(r) || std::isinf(r)) {
                    continue; // Ignorăm punctele invalide
                }

                // 1. Polar -> Cartezian local
                double angle = scan->angle_min + i * scan->angle_increment;
                double x = r * std::cos(angle);
                double y = r * std::sin(angle);

                // 2. Transformare în base_link
                tf2::Vector3 point_local(x, y, 0.0);
                tf2::Vector3 point_base = transform * point_local;

                // 3. Cartezian base_link -> Polar base_link
                double merged_r = std::hypot(point_base.x(), point_base.y());
                double merged_angle = std::atan2(point_base.y(), point_base.x());

                // 4. Inserare în array-ul final (păstrăm cel mai apropiat obstacol)
                if (merged_r >= merged.range_min && merged_r <= merged.range_max) {
                    int index = std::round((merged_angle - merged.angle_min) / merged.angle_increment);
                    
                    // Verificare de siguranță pentru limitele array-ului
                    if (index >= 0 && index < static_cast<int>(num_points)) {
                        if (merged_r < merged.ranges[index]) {
                            merged.ranges[index] = merged_r;
                        }
                    }
                }
            }
        }

        pub_->publish(merged);
    }

    std::vector<std::string> topics_;
    std::string output_topic_;
    std::string output_frame_id_;

    std::vector<sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;

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