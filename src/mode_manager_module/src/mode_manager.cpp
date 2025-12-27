#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mode_manager_interfaces/srv/set_mode.hpp"
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
        // Initialize State
        current_mode_ = "IDLE";
        modes_ = {"IDLE", "MANUAL", "FOLLOWING", "NAVIGATION"};

        // Publishers
        mode_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_mode", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Service
        mode_service_ = this->create_service<mode_manager_interfaces::srv::SetMode>(
            "set_mode",
            std::bind(&ModeManager::handle_set_mode, this, _1, _2));

        // Subscribers
        teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "teleop/cmd_vel",
            10,
            std::bind(&ModeManager::teleop_callback, this, _1));

        // UI Mode Subscribers (Absolute and Relative)
        ui_mode_subscriber_abs_ = this->create_subscription<std_msgs::msg::String>(
            "/ui_mode_command",
            10,
            std::bind(&ModeManager::ui_mode_callback, this, _1));

        ui_mode_subscriber_rel_ = this->create_subscription<std_msgs::msg::String>(
            "ui_mode_command",
            10,
            std::bind(&ModeManager::ui_mode_callback, this, _1));

        // Action Client
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        // Initial Publish
        publish_current_mode();
        RCLCPP_INFO(this->get_logger(), "Mode Manager started, current mode: %s", current_mode_.c_str());
    }

private:
    // Member Variables
    std::string current_mode_;
    std::vector<std::string> modes_;
    
    // ROS Handles
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Service<mode_manager_interfaces::srv::SetMode>::SharedPtr mode_service_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ui_mode_subscriber_abs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ui_mode_subscriber_rel_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    
    // Store the current goal handle to allow cancellation
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;

    // ----------------------------------------------------------------------
    // Callbacks
    // ----------------------------------------------------------------------

    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (current_mode_ == "MANUAL") {
            cmd_vel_publisher_->publish(*msg);
        }
    }

    void handle_set_mode(
        const std::shared_ptr<mode_manager_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<mode_manager_interfaces::srv::SetMode::Response> response)
    {
        // Check if requested mode is in the allowed list
        bool valid_mode = false;
        for (const auto & m : modes_) {
            if (m == request->mode) {
                valid_mode = true;
                break;
            }
        }

        if (valid_mode) {
            if (current_mode_ == "NAVIGATION" && request->mode != "NAVIGATION") {
                cancel_navigation();
            }
            
            current_mode_ = request->mode;
            publish_current_mode();
            RCLCPP_INFO(this->get_logger(), "Switching to %s mode", current_mode_.c_str());
            response->success = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid mode requested: %s", request->mode.c_str());
            response->success = false;
        }
    }

    void ui_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string raw = msg->data;
        // Strip whitespace (simple version) and convert to upper case
        raw.erase(remove(raw.begin(), raw.end(), ' '), raw.end()); 
        std::transform(raw.begin(), raw.end(), raw.begin(), ::toupper);

        std::map<std::string, std::string> alias_map = {
            {"IDLE", "IDLE"}, {"MANUAL", "MANUAL"}, 
            {"FOLLOW", "FOLLOWING"}, {"FOLLOWING", "FOLLOWING"},
            {"NAV", "NAVIGATION"}, {"NAVIGATION", "NAVIGATION"}
        };

        if (alias_map.find(raw) == alias_map.end()) {
            RCLCPP_WARN(this->get_logger(), "UI mode command invalid: %s", raw.c_str());
            return;
        }

        std::string target = alias_map[raw];

        // Reuse logic
        if (current_mode_ == "NAVIGATION" && target != "NAVIGATION") {
            cancel_navigation();
        }

        if (target != current_mode_) {
            current_mode_ = target;
            publish_current_mode();
            RCLCPP_INFO(this->get_logger(), "UI requested mode change -> %s", current_mode_.c_str());
        }
    }

    void publish_current_mode()
    {
        auto msg = std_msgs::msg::String();
        msg.data = current_mode_;
        mode_publisher_->publish(msg);
    }

    // ----------------------------------------------------------------------
    // Action Client Logic
    // ----------------------------------------------------------------------

public: 
    // Made public to be callable, though usually triggered by internal logic
    void send_navigation_goal(const geometry_msgs::msg::Pose & pose)
    {
        if (current_mode_ != "NAVIGATION") {
            RCLCPP_WARN(this->get_logger(), "Cannot send navigation goal, not in NAVIGATION mode");
            return;
        }

        if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose = pose; 
        // Note: NavigateToPose.Goal expects a PoseStamped usually, 
        // but depending on nav2 version it might vary. 
        // Assuming goal_msg.pose is geometry_msgs/PoseStamped:
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&ModeManager::goal_response_callback, this, _1);
            
        send_goal_options.result_callback =
            std::bind(&ModeManager::get_result_callback, this, _1);

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal rejected :(");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted :)");
        current_goal_handle_ = goal_handle;
    }

    void get_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Result: Succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Result: Aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Result: Canceled");
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Result: Unknown code");
                break;
        }
        // Optionally switch back to IDLE
        // current_mode_ = "IDLE";
        
        // Reset handle
        current_goal_handle_.reset();
    }

    void cancel_navigation()
    {
        if (current_goal_handle_) {
            // Check if active not directly available in all ros2 versions on pointer, 
            // but non-null implies we tracked it.
            RCLCPP_INFO(this->get_logger(), "Canceling current navigation goal");
            
            auto future_cancel = nav_to_pose_client_->async_cancel_goal(current_goal_handle_);
            
            // In C++, we usually define the callback via lambda or bind for the future
            // However, async_cancel_goal returns a future, processing it requires spinning 
            // or attaching a continuation if using advanced executors. 
            // For simplicity in standard node spin, we rely on the Action Server 
            // triggering the result_callback with status CANCELED.
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeManager>());
    rclcpp::shutdown();
    return 0;
}