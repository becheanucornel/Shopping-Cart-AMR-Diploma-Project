#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <cstdlib>
#include <chrono>
#include <thread>

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
        this->declare_parameter<std::string>("slam_params_file", "");
        this->declare_parameter<double>("initial_pose_x", 0.0);
        this->declare_parameter<double>("initial_pose_y", 0.0);
        this->declare_parameter<double>("initial_pose_yaw", 0.0);

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

        current_mode_ = "IDLE";

        // Publishers
        mode_publisher_    = this->create_publisher<std_msgs::msg::String>("/mode", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribers
        teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_teleop", 10, std::bind(&ModeManager::teleop_callback, this, _1));

        nav_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav2", 10, std::bind(&ModeManager::nav_cmd_callback, this, _1));

        follow_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_follow", 10, std::bind(&ModeManager::follow_cmd_callback, this, _1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/custom_odom_topic", 10, std::bind(&ModeManager::odom_callback, this, _1));

        ui_mode_subscriber_abs_ = this->create_subscription<std_msgs::msg::String>(
            "/ui/mode/absolute", 10, std::bind(&ModeManager::ui_mode_callback, this, _1));

        ui_goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ui/nav_goal", 10, std::bind(&ModeManager::ui_goal_callback, this, _1));

        // Nav2 action client
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // cmd_vel timer
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_vel_publish_rate_hz_));
        cmd_vel_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ModeManager::cmd_vel_timer_tick, this));

        RCLCPP_INFO(this->get_logger(), "Mode Manager pornit. Gata de lucru.");
    }

private:
    void R_INFO(const std::string & msg)  { RCLCPP_INFO(this->get_logger(),  "%s", msg.c_str()); }
    void R_ERROR(const std::string & msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); }

    std::string current_mode_;
    double nav_cruise_linear_scale_, nav_cruise_angular_scale_;
    double nav_linear_scale_, nav_angular_scale_;
    double nav_min_linear_x_, nav_min_angular_z_;
    double nav_stuck_linear_x_, nav_stuck_angular_z_;
    double cmd_vel_publish_rate_hz_, cmd_vel_timeout_sec_;
    double last_odom_linear_x_{0.0}, last_odom_angular_z_{0.0};

    geometry_msgs::msg::Twist last_teleop_cmd_{};
    geometry_msgs::msg::Twist last_nav_cmd_{};
    geometry_msgs::msg::Twist last_follow_cmd_{};

    rclcpp::Time last_teleop_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_nav_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_follow_cmd_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      mode_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      teleop_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      nav_cmd_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      follow_cmd_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          ui_mode_subscriber_abs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ui_goal_subscriber_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

    void publish_current_mode()
    {
        auto msg = std_msgs::msg::String();
        msg.data = current_mode_;
        mode_publisher_->publish(msg);
    }

    void stop_robot()
    {
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
    }

    void cancel_active_navigation()
    {
        if (current_goal_handle_) {
            nav_to_pose_client_->async_cancel_goal(current_goal_handle_);
            current_goal_handle_.reset();
            R_INFO("Navigatie anulata.");
        }
    }

    void publish_initial_pose(double x, double y, double yaw)
    {
        double qz = std::sin(yaw / 2.0);
        double qw = std::cos(yaw / 2.0);

        std::string cmd =
            "ros2 topic pub --once /initialpose "
            "geometry_msgs/msg/PoseWithCovarianceStamped "
            "\"{"
            "header: {frame_id: 'map'}, "
            "pose: {pose: {"
            "position: {x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", z: 0.0}, "
            "orientation: {x: 0.0, y: 0.0, z: " + std::to_string(qz) + ", w: " + std::to_string(qw) + "}"
            "}, "
            "covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, "
            "             0,0,0,0,0,0,   0,0,0,0,0,0,   0,0,0,0,0,0.068]"
            "}}\" 2>/dev/null || true";

        std::system(cmd.c_str());
        R_INFO("Initial pose publicat catre AMCL (" +
               std::to_string(x) + ", " + std::to_string(y) + ", yaw=" + std::to_string(yaw) + ")");
    }

    void save_map_and_switch_to_localization(const std::string & save_path,
                                              const std::string & yaml_path,
                                              double init_x, double init_y, double init_yaw)
    {
        std::thread([save_path, yaml_path, init_x, init_y, init_yaw, this]() {

            fprintf(stderr, "[mode_manager] Incep salvarea hartii la: %s\n", save_path.c_str());
            std::string save_cmd = "ros2 run nav2_map_server map_saver_cli -f "
                + save_path + " --ros-args -p save_map_timeout:=10.0 2>&1";
            int ret = std::system(save_cmd.c_str());
            if (ret != 0) {
                fprintf(stderr, "[mode_manager] map_saver_cli a esuat (cod %d)\n", ret);
            } else {
                fprintf(stderr, "[mode_manager] Harta salvata cu succes.\n");
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::system("pkill -SIGINT -f async_slam_toolbox_node 2>/dev/null || true");
            fprintf(stderr, "[mode_manager] SLAM Toolbox oprit.\n");
            std::this_thread::sleep_for(std::chrono::seconds(3));

            fprintf(stderr, "[mode_manager] Incarc harta noua in map_server...\n");
            std::string load_cmd =
                "ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "
                "\"{map_url: '" + yaml_path + "'}\"" + " 2>/dev/null || true";
            std::system(load_cmd.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(3));

            std::system("ros2 lifecycle set /map_server activate 2>/dev/null || true");
            std::this_thread::sleep_for(std::chrono::seconds(1));

            fprintf(stderr, "[mode_manager] Activez AMCL...\n");
            std::system("ros2 lifecycle set /amcl activate 2>/dev/null || true");
            std::this_thread::sleep_for(std::chrono::seconds(3));

            fprintf(stderr, "[mode_manager] Publicand initial pose la (%.2f, %.2f, %.2f)...\n",
                    init_x, init_y, init_yaw);
            this->publish_initial_pose(init_x, init_y, init_yaw);

            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::system(
                "ros2 run human_detection_module detection_node "
                "--ros-args -r __node:=yolo_tracker "
                "-p confidence_threshold:=0.50 "
                "-p nms_threshold:=0.45 "
                "-p focal_length_px:=500.0 "
                "-p person_real_height_m:=1.70 "
                "-p ball_real_height_m:=0.04 "
                "-p min_box_height_px:=20 &"
            );
            fprintf(stderr, "[mode_manager] YOLO repornit.\n");

            fprintf(stderr, "[mode_manager] Secventa MAPPING -> LOCALIZARE completa.\n");

        }).detach();
    }

    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_teleop_cmd_ = *msg;
        last_teleop_cmd_time_ = this->now();
    }

    void nav_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_nav_cmd_ = *msg;
        last_nav_cmd_time_ = this->now();
    }

    void follow_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_follow_cmd_ = *msg;
        last_follow_cmd_time_ = this->now();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_odom_linear_x_  = msg->twist.twist.linear.x;
        last_odom_angular_z_ = msg->twist.twist.angular.z;
    }

    void ui_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (current_mode_ != "NAVIGATION") {
            R_INFO("Goal ignorat — nu suntem in modul NAVIGATION.");
            return;
        }

        if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
            R_ERROR("NavigateToPose action server nu e disponibil!");
            return;
        }

        cancel_active_navigation();

        auto goal = NavigateToPose::Goal();
        goal.pose = *msg;
        goal.pose.header.stamp = this->now();
        goal.pose.header.frame_id = "map";

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleNavigateToPose::SharedPtr & handle) {
                if (!handle) {
                    R_ERROR("Goal respins de Nav2!");
                } else {
                    current_goal_handle_ = handle;
                    R_INFO("Goal acceptat de Nav2. Navigatie in desfasurare...");
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleNavigateToPose::SharedPtr,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                (void)feedback;
            };

        send_goal_options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        R_INFO("Navigatie completata cu succes!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        R_ERROR("Navigatia a fost anulata de Nav2 (ABORTED).");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        R_INFO("Navigatia a fost anulata de utilizator.");
                        break;
                    default:
                        R_ERROR("Navigatia a esuat cu cod necunoscut.");
                        break;
                }
                current_goal_handle_.reset();
            };

        nav_to_pose_client_->async_send_goal(goal, send_goal_options);
        R_INFO("Goal trimis la Nav2: x=" + std::to_string(msg->pose.position.x) +
               " y=" + std::to_string(msg->pose.position.y));
    }

    void ui_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string target = msg->data;
        R_INFO("COMANDA PRIMITA: " + target + " (Stare actuala: " + current_mode_ + ")");

        if (current_mode_ == "MAPPING" && target != "MAPPING") {
            R_INFO("Iesire din modul MAPPING. Salvez harta...");
            std::string save_path = this->get_parameter("map_save_path").as_string();
            std::string yaml_path = this->get_parameter("map_yaml_path").as_string();
            double init_x   = this->get_parameter("initial_pose_x").as_double();
            double init_y   = this->get_parameter("initial_pose_y").as_double();
            double init_yaw = this->get_parameter("initial_pose_yaw").as_double();
            save_map_and_switch_to_localization(save_path, yaml_path, init_x, init_y, init_yaw);
        }

        if (target != current_mode_) {
            if (current_mode_ == "NAVIGATION" || current_mode_ == "FOLLOWING") {
                cancel_active_navigation();
            }

            current_mode_ = target;
            publish_current_mode();
            stop_robot();

            if (target == "MAPPING") {
                R_INFO("Dezactivez AMCL si pornesc SLAM Toolbox...");
                std::system("pkill -SIGINT -f lifecycle_manager_localization 2>/dev/null || true");
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                std::system("ros2 lifecycle set /amcl deactivate 2>/dev/null || true");
                std::system("ros2 lifecycle set /map_server deactivate 2>/dev/null || true");
                std::system("pkill -SIGINT -f detection_node 2>/dev/null || true");
                R_INFO("YOLO oprit pentru mapping (elibereaza CPU pentru SLAM).");
                std::this_thread::sleep_for(std::chrono::seconds(2));

                std::string slam_params = this->get_parameter("slam_params_file").as_string();
                std::string slam_cmd = "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false";
                if (!slam_params.empty()) slam_cmd += " slam_params_file:=" + slam_params;
                slam_cmd += " &";
                R_INFO("Lansez SLAM Toolbox...");
                std::system(slam_cmd.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(3));
                R_INFO("SLAM Toolbox pornit.");
            }

            if (target == "NAVIGATION") {
                R_INFO("Mod NAVIGATION activ. AMCL ar trebui sa fie localizat.");
            }

        } else {
            publish_current_mode();
        }
    }

    void cmd_vel_timer_tick()
    {
        auto now = this->now();
        auto timeout = rclcpp::Duration::from_seconds(cmd_vel_timeout_sec_);
        geometry_msgs::msg::Twist out;

        if (current_mode_ == "MANUAL" || current_mode_ == "MAPPING") {
            if ((now - last_teleop_cmd_time_) < timeout) {
                out = last_teleop_cmd_;
            }
        } else if (current_mode_ == "NAVIGATION") {
            if ((now - last_nav_cmd_time_) < timeout) {
                out = last_nav_cmd_;
                out.linear.x  *= nav_cruise_linear_scale_;
                out.angular.z *= nav_cruise_angular_scale_;
                if (std::abs(out.linear.x) > 0.0 &&
                    std::abs(last_odom_linear_x_) < nav_stuck_linear_x_)
                {
                    out.linear.x *= nav_linear_scale_;
                    if (out.linear.x > 0) out.linear.x = std::max(out.linear.x, nav_min_linear_x_);
                    if (out.linear.x < 0) out.linear.x = std::min(out.linear.x, -nav_min_linear_x_);
                }
            }
        } else if (current_mode_ == "FOLLOWING") {
            if ((now - last_nav_cmd_time_) < timeout) {
                out = last_nav_cmd_;
            }
        }

        cmd_vel_publisher_->publish(out);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeManager>());
    rclcpp::shutdown();
    return 0;
}