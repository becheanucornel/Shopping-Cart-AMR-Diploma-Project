#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <cstdlib>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ModeManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ModeManager() : Node("mode_manager")
    {
        // Declarare parametrii
        this->declare_parameter<double>("nav_cruise_linear_scale", 1.0);
        this->declare_parameter<double>("nav_cruise_angular_scale", 1.0);
        this->declare_parameter<double>("nav_linear_scale", 1.0);
        this->declare_parameter<double>("nav_angular_scale", 1.0);
        this->declare_parameter<double>("nav_min_linear_x", 0.0);
        this->declare_parameter<double>("nav_min_angular_z", 0.0);
        this->declare_parameter<double>("nav_stuck_linear_x", 0.02);
        this->declare_parameter<double>("nav_stuck_angular_z", 0.02);
        this->declare_parameter<double>("cmd_vel_publish_rate_hz", 20.0);
        this->declare_parameter<double>("cmd_vel_timeout_sec", 0.5);
        this->declare_parameter<std::string>("map_save_path", "/tmp/saved_map");
        this->declare_parameter<std::string>("map_yaml_path", "/tmp/map.yaml");

        // Citire parametrii
        nav_cruise_linear_scale_ = this->get_parameter("nav_cruise_linear_scale").as_double();
        nav_cruise_angular_scale_ = this->get_parameter("nav_cruise_angular_scale").as_double();
        nav_linear_scale_ = this->get_parameter("nav_linear_scale").as_double();
        nav_angular_scale_ = this->get_parameter("nav_angular_scale").as_double();
        nav_min_linear_x_ = this->get_parameter("nav_min_linear_x").as_double();
        nav_min_angular_z_ = this->get_parameter("nav_min_angular_z").as_double();
        nav_stuck_linear_x_ = this->get_parameter("nav_stuck_linear_x").as_double();
        nav_stuck_angular_z_ = this->get_parameter("nav_stuck_angular_z").as_double();
        cmd_vel_publish_rate_hz_ = this->get_parameter("cmd_vel_publish_rate_hz").as_double();
        cmd_vel_timeout_sec_ = this->get_parameter("cmd_vel_timeout_sec").as_double();

        modes_ = {"IDLE", "MANUAL", "MAPPING", "NAVIGATION", "FOLLOWING"};
        current_mode_ = "IDLE";

        mode_publisher_ = this->create_publisher<std_msgs::msg::String>("/mode", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriberi
        teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_teleop", 10, std::bind(&ModeManager::teleop_callback, this, _1));

        nav_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav2", 10, std::bind(&ModeManager::nav_cmd_callback, this, _1));

        follow_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_follow", 10, std::bind(&ModeManager::follow_cmd_callback, this, _1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ModeManager::odom_callback, this, _1));

        ui_mode_subscriber_abs_ = this->create_subscription<std_msgs::msg::String>(
            "/ui/mode/absolute", 10, std::bind(&ModeManager::ui_mode_callback, this, _1));

        ui_goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ui/nav_goal", 10, std::bind(&ModeManager::ui_goal_callback, this, _1));

        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_vel_publish_rate_hz_));
        cmd_vel_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ModeManager::cmd_vel_timer_tick, this));

        R_INFO("Mode Manager pornit (Clean Version - Fără Watchdog RPi). Gata de lucru.");
    }

private:
    void R_INFO(const std::string &msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); }
    void R_ERROR(const std::string &msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); }

    // Membri
    std::string current_mode_;
    std::vector<std::string> modes_;
    double nav_cruise_linear_scale_, nav_cruise_angular_scale_, nav_linear_scale_, nav_angular_scale_;
    double nav_min_linear_x_, nav_min_angular_z_, nav_stuck_linear_x_, nav_stuck_angular_z_;
    double cmd_vel_publish_rate_hz_, cmd_vel_timeout_sec_;
    double last_odom_linear_x_{0.0}, last_odom_angular_z_{0.0};
    geometry_msgs::msg::Twist last_teleop_cmd_{}, last_nav_cmd_{}, last_follow_cmd_{};
    rclcpp::Time last_teleop_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_nav_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_follow_cmd_time_{0, 0, RCL_ROS_TIME};

    // Pub/Sub/Action
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscriber_, nav_cmd_subscriber_, follow_cmd_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ui_mode_subscriber_abs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ui_goal_subscriber_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

    // Callbacks
    void cancel_active_navigation() {
        if (current_goal_handle_) {
            nav_to_pose_client_->async_cancel_goal(current_goal_handle_);
            current_goal_handle_.reset();
        }
    }

    void publish_current_mode() {
        auto msg = std_msgs::msg::String();
        msg.data = current_mode_;
        mode_publisher_->publish(msg);
    }

    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_teleop_cmd_ = *msg;
        last_teleop_cmd_time_ = this->now();
    }

    void nav_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_nav_cmd_ = *msg;
        last_nav_cmd_time_ = this->now();
    }

    void follow_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_follow_cmd_ = *msg;
        last_follow_cmd_time_ = this->now();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_linear_x_ = msg->twist.twist.linear.x;
        last_odom_angular_z_ = msg->twist.twist.angular.z;
    }

    void ui_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        R_INFO("Nav goal primit în Mode Manager.");
    }

    void ui_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string target = msg->data;
        R_INFO("COMANDA PRIMITA: " + target + " (Stare actuala: " + current_mode_ + ")");

        // Iesirea din MAPPING: Oprim SLAM, logam noua harta, repornim AMCL
        if (current_mode_ == "MAPPING" && target != "MAPPING") {
            R_INFO("Iesire din modul MAPPING. Salvez harta si opresc SLAM...");
            
            // 1. Salvăm harta
            std::string save_path = this->get_parameter("map_save_path").as_string();
            std::string yaml_path = this->get_parameter("map_yaml_path").as_string();
    
            // Use dynamic string construction
            std::string save_cmd = "ros2 run nav2_map_server map_saver_cli -f " + save_path + " --ros-args -p save_map_timeout:=10.0";
            std::system(save_cmd.c_str());

            // 2. Oprim curat SLAM
            std::system("pkill -2 -f slam_toolbox");
            std::system("pkill -2 -f async_slam_toolbox_node");
            std::system("sleep 2");

            // 3. Reactivăm localizarea (AMCL și Map Server) folosind Lifecycle Manager
            R_INFO("Re-activez sistemul de localizare AMCL...");
            std::system("ros2 lifecycle set /map_server activate &");
            std::system("ros2 lifecycle set /amcl activate &");

            // 4. Încărcăm noua hartă direct în map_server
            std::string load_cmd = "ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \"{map_url: '" + yaml_path + "'}\" &";
            std::system(load_cmd.c_str());
        }

        if (target != current_mode_) {
            if (current_mode_ == "NAVIGATION") cancel_active_navigation();
            current_mode_ = target;
            publish_current_mode();

            // Intrarea in MAPPING: Adormim AMCL si Map Server ca sa lasam SLAM sa controleze TF-ul
            if (target == "MAPPING") {
                R_INFO("Dezactivez AMCL temporar si pornesc SLAM...");
                std::system("ros2 lifecycle set /amcl deactivate");
                std::system("ros2 lifecycle set /map_server deactivate");
                std::system("sleep 1");
                
                std::system("ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &");
            }
            cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
        } else {
            publish_current_mode();
        }
    }

    void cmd_vel_timer_tick() {
        auto now = this->now();
        auto timeout = rclcpp::Duration::from_seconds(cmd_vel_timeout_sec_);
        geometry_msgs::msg::Twist out;

        if (current_mode_ == "MANUAL" || current_mode_ == "MAPPING") {
            // Teleop passes through directly for instant response
            if ((now - last_teleop_cmd_time_) < timeout) {
                out = last_teleop_cmd_;
            }
        } else if (current_mode_ == "NAVIGATION") {
            if ((now - last_nav_cmd_time_) < timeout) {
                out = last_nav_cmd_;
                
                // 1. Apply Cruise Scaling to Nav2 commands
                out.linear.x *= nav_cruise_linear_scale_;
                out.angular.z *= nav_cruise_angular_scale_;
                
                // 2. Anti-Stuck Logic
                // If the robot is commanded to move but the odometry says it's physically stationary
                if (std::abs(out.linear.x) > 0.0 && std::abs(last_odom_linear_x_) < nav_stuck_linear_x_) {
                    out.linear.x *= nav_linear_scale_; // Temporarily boost power to overcome friction
                    // Ensure it meets minimum required speed to break static friction
                    if (out.linear.x > 0) out.linear.x = std::max(out.linear.x, nav_min_linear_x_);
                    if (out.linear.x < 0) out.linear.x = std::min(out.linear.x, -nav_min_linear_x_);
                }
            }
        } else if (current_mode_ == "FOLLOWING") {
            if ((now - last_follow_cmd_time_) < timeout) {
                out = last_follow_cmd_;
                // YOLO visual servoing can be scaled here in the future if needed
            }
        }
        
        cmd_vel_publisher_->publish(out);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeManager>());
    rclcpp::shutdown();
    return 0;
}