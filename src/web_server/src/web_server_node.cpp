#include "web_server/web_server_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <iostream>

namespace web_server
{

WebServerNode::WebServerNode(const rclcpp::NodeOptions & options)
: Node("web_server_node", options), running_(true), current_pose_({})
{
  this->declare_parameter("port", 8080);
  port_ = this->get_parameter("port").as_int();

  this->declare_parameter("publish_odom_tf", true);
  this->declare_parameter("odom_frame_id", "custom_odom");
  this->declare_parameter("base_frame_id", "custom_base_link");
  publish_odom_tf_ = this->get_parameter("publish_odom_tf").as_bool();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();

  this->declare_parameter("auto_initialpose_from_odom", false);
  this->declare_parameter("initialpose_frame_id", "map");
  this->declare_parameter("initialpose_cov_xy", 0.25);
  this->declare_parameter("initialpose_cov_yaw", 0.25);
  auto_initialpose_from_odom_ = this->get_parameter("auto_initialpose_from_odom").as_bool();
  initialpose_frame_id_ = this->get_parameter("initialpose_frame_id").as_string();
  initialpose_cov_xy_ = this->get_parameter("initialpose_cov_xy").as_double();
  initialpose_cov_yaw_ = this->get_parameter("initialpose_cov_yaw").as_double();

  try {
    resource_path_ = ament_index_cpp::get_package_share_directory("web_server") + "/resource";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get package directory: %s", e.what());
    resource_path_ = "./resource";
  }

  // --- Configurare Cale XML pentru Follow Me ---
  try {
    follow_me_xml_path_ = ament_index_cpp::get_package_share_directory("system_bringup") + "/behavior_trees/follow_me.xml";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Atenție: Nu am găsit pachetul system_bringup pentru XML-ul Follow Me: %s", e.what());
    follow_me_xml_path_ = "";
  }

  // Create ROS 2 publishers and subscribers
  odom_nav2_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_nav2", 10);
  initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

  if (publish_odom_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  // Bridge web UI goals -> Nav2 NavigateToPose action
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ui/nav_goal", 10, std::bind(&WebServerNode::goal_pose_callback, this, std::placeholders::_1));
  goal_cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/goal_cancel", 10, std::bind(&WebServerNode::goal_cancel_callback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/custom_odom_topic", 10, std::bind(&WebServerNode::odom_callback, this, std::placeholders::_1));
  
  amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10, std::bind(&WebServerNode::amcl_callback, this, std::placeholders::_1));

  preempt_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&WebServerNode::preempt_timer_cb, this));

  // --- Mode Management Setup ---
  mode_pub_ = this->create_publisher<std_msgs::msg::String>("/mode", 10);
  ui_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/ui/mode/absolute", 10, std::bind(&WebServerNode::ui_mode_callback, this, std::placeholders::_1));

  mode_publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), [this](){
      std_msgs::msg::String mode_msg;
      mode_msg.data = current_mode_;
      if(mode_pub_) mode_pub_->publish(mode_msg);
    });

  // --- YOLO Target Subscriber (Follow Me) ---
  yolo_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/yolo/target_pose", 10, std::bind(&WebServerNode::yolo_target_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Starting web server on port %d", port_);
  RCLCPP_INFO(this->get_logger(), "Serving files from: %s", resource_path_.c_str());
  std::cout << "Web server starting on port " << port_ << std::endl;

  ioc_ = std::make_shared<net::io_context>(1);
  server_thread_ = std::make_unique<std::thread>(&WebServerNode::run_server, this);
}

void WebServerNode::ui_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Mod schimbat din Web UI: %s", msg->data.c_str());
  current_mode_ = msg->data;

  if (current_mode_ != "FOLLOWING") {
      is_following_active_ = false;
  }

  // Oprim robotul din a mai naviga singur când se schimbă modul forțat
  if ((current_mode_ == "MANUAL" || current_mode_ == "IDLE") && current_goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "Anulez navigația autonomă curentă (trecere în %s).", current_mode_.c_str());
    (void)nav_client_->async_cancel_goal(current_goal_handle_);
  }

  if (current_mode_ == "FOLLOWING") {
      is_following_active_ = true;
      RCLCPP_INFO(this->get_logger(), "Mod Follow Me Activat! Aștept target pe /yolo/target_pose");
  }

  std_msgs::msg::String mode_msg;
  mode_msg.data = current_mode_;
  mode_pub_->publish(mode_msg);
}

void WebServerNode::yolo_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Ignoră comenzile de la cameră dacă modul Follow Me nu a fost apăsat în UI
  if (!is_following_active_) {
      return;
  }

  if (!nav_client_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Nav2 action client not initialized");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = *msg;
  
  // Forțează XML-ul de Follow Me (cel la 5Hz)
  if (!follow_me_xml_path_.empty()) {
      goal.behavior_tree = follow_me_xml_path_;
  } else {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Nu se găsește follow_me.xml!");
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "Trimit update la Nav2 către om: x=%.2f, y=%.2f", goal.pose.pose.position.x, goal.pose.pose.position.y);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  nav_client_->async_send_goal(goal, send_goal_options);
}

void WebServerNode::send_pending_goal()
{
  if (!pending_goal_) {
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = pending_goal_pose_;
  goal.behavior_tree = ""; // Pentru Navigația normală (puncte pe hartă), folosește XML-ul default din YAML
  pending_goal_ = false;

  RCLCPP_INFO(this->get_logger(), "Forwarding goal to Nav2: frame=%s x=%.3f y=%.3f",
              goal.pose.header.frame_id.c_str(), goal.pose.pose.position.x, goal.pose.pose.position.y);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](GoalHandleNavigateToPose::SharedPtr goal_handle) {
      if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Nav2 goal rejected");
        return;
      }
      current_goal_handle_ = goal_handle;
      RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted");
    };

  options.result_callback =
    [this](const GoalHandleNavigateToPose::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Nav2 goal succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(this->get_logger(), "Nav2 goal aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Nav2 goal canceled");
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Nav2 goal finished with unknown result code");
          break;
      }
      current_goal_handle_.reset();
    };

  (void)nav_client_->async_send_goal(goal, options);
}

void WebServerNode::preempt_timer_cb()
{
  if (!cancel_in_progress_) {
    return;
  }

  if (current_goal_handle_) {
    return;
  }

  cancel_in_progress_ = false;
  send_pending_goal();
}

void WebServerNode::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!nav_client_) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action client not initialized");
    return;
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "NavigateToPose action server not available");
    return;
  }

  pending_goal_pose_ = *msg;
  pending_goal_ = true;

  if (current_goal_handle_) {
    if (!cancel_in_progress_) {
      cancel_in_progress_ = true;
      RCLCPP_INFO(this->get_logger(), "Canceling active goal before sending new goal");
      (void)nav_client_->async_cancel_goal(current_goal_handle_);
    }
    return;
  }

  send_pending_goal();
}

void WebServerNode::goal_cancel_callback(const std_msgs::msg::Empty::SharedPtr)
{
  if (!nav_client_) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action client not initialized");
    return;
  }

  if (!current_goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Cancel requested");
  (void)nav_client_->async_cancel_goal(current_goal_handle_);
}

WebServerNode::~WebServerNode()
{
  running_ = false;
  if (ioc_) {
    ioc_->stop();
  }
  if (server_thread_ && server_thread_->joinable()) {
    server_thread_->join();
  }
}

void WebServerNode::run_server()
{
  try {
    auto const address = net::ip::make_address("0.0.0.0");
    tcp::acceptor acceptor{*ioc_, {address, static_cast<unsigned short>(port_)}};
    std::cout << "Server listening on port " << port_ << std::endl;

    while (running_) {
      tcp::socket socket{*ioc_};
      try {
        acceptor.accept(socket);
        std::thread(&WebServerNode::handle_session, this, std::move(socket)).detach();
      } catch (const std::exception & e) {
        if (running_) {
          RCLCPP_DEBUG(this->get_logger(), "Accept error: %s", e.what());
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Server error: %s", e.what());
    std::cerr << "Server error: " << e.what() << std::endl;
  }
}

void WebServerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_odom_ = *msg;

  if (auto_initialpose_from_odom_ && !initialpose_sent_ && initialpose_pub_) {
    geometry_msgs::msg::PoseWithCovarianceStamped init;
    init.header.stamp = msg->header.stamp;
    init.header.frame_id = initialpose_frame_id_;
    init.pose.pose = msg->pose.pose;

    init.pose.covariance.fill(0.0);
    init.pose.covariance[0] = initialpose_cov_xy_;
    init.pose.covariance[7] = initialpose_cov_xy_;
    init.pose.covariance[35] = initialpose_cov_yaw_;

    initialpose_pub_->publish(init);
    initialpose_sent_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Published /initialpose from /custom_odom_topic: frame=%s x=%.3f y=%.3f",
                init.header.frame_id.c_str(), init.pose.pose.position.x, init.pose.pose.position.y);
  }

  if (odom_nav2_pub_) {
    nav_msgs::msg::Odometry filtered = *msg;
    filtered.header.frame_id = odom_frame_id_;
    filtered.child_frame_id = base_frame_id_;

    filtered.twist.twist.linear.y = 0.0;
    filtered.twist.twist.linear.z = 0.0;
    filtered.twist.twist.angular.x = 0.0;
    filtered.twist.twist.angular.y = 0.0;

    odom_nav2_pub_->publish(filtered);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = filtered.header.stamp;
      tf.header.frame_id = filtered.header.frame_id;
      tf.child_frame_id = filtered.child_frame_id;
      tf.transform.translation.x = filtered.pose.pose.position.x;
      tf.transform.translation.y = filtered.pose.pose.position.y;
      tf.transform.translation.z = filtered.pose.pose.position.z;
      tf.transform.rotation = filtered.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }
}

void WebServerNode::amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_pose_ = msg->pose.pose;
}

void WebServerNode::handle_session(tcp::socket socket)
{
  try {
    beast::flat_buffer buffer;
    http::request<http::string_body> req;
    http::read(socket, buffer, req);

    std::string path = std::string(req.target());
    
    if (path.find("/api/") == 0) {
      handle_api_request(path, socket);
      return;
    }
    
    if (path == "/") {
      path = "/index.html";
    }

    http::response<http::string_body> res;
    std::string file_content = load_file(resource_path_ + path);
    
    if (!file_content.empty()) {
      res.result(http::status::ok);
      res.set(http::field::server, "ROS2 Web Server");
      res.set(http::field::content_type, get_mime_type(path));
      res.set(http::field::access_control_allow_origin, "*");
      res.body() = file_content;
    } else {
      res.result(http::status::not_found);
      res.set(http::field::content_type, "text/plain");
      res.set(http::field::access_control_allow_origin, "*");
      res.body() = "File not found: " + path;
    }

    res.prepare_payload();
    http::write(socket, res);
    socket.shutdown(tcp::socket::shutdown_send);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Session error: %s", e.what());
  }
}

void WebServerNode::handle_api_request(const std::string & path, tcp::socket & socket)
{
  http::response<http::string_body> res;
  res.set(http::field::server, "ROS2 Web Server");
  res.set(http::field::content_type, "application/json");
  res.set(http::field::access_control_allow_origin, "*");
  
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (path == "/api/robot/state") {
    res.result(http::status::ok);
    std::string json = "{\"connected\": true, \"position\": {\"x\": " + 
                       std::to_string(current_pose_.position.x) + ", \"y\": " + 
                       std::to_string(current_pose_.position.y) + "}, \"orientation\": " + 
                       std::to_string(current_pose_.orientation.z) + "}";
    res.body() = json;
  } else {
    res.result(http::status::not_found);
    res.body() = "{\"error\": \"API endpoint not found\"}";
  }
  
  res.prepare_payload();
  http::write(socket, res);
  socket.shutdown(tcp::socket::shutdown_send);
}

std::string WebServerNode::get_mime_type(const std::string & path)
{
  auto has_extension = [](const std::string & str, const std::string & ext) {
    return str.size() >= ext.size() && 
           str.compare(str.size() - ext.size(), ext.size(), ext) == 0;
  };

  if (has_extension(path, ".html")) return "text/html";
  if (has_extension(path, ".js")) return "application/javascript";
  if (has_extension(path, ".css")) return "text/css";
  if (has_extension(path, ".json")) return "application/json";
  if (has_extension(path, ".png")) return "image/png";
  if (has_extension(path, ".jpg") || has_extension(path, ".jpeg")) return "image/jpeg";
  return "text/plain";
}

std::string WebServerNode::load_file(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    RCLCPP_WARN(this->get_logger(), "Could not open file: %s", path.c_str());
    return "";
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

}  // namespace web_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(web_server::WebServerNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<web_server::WebServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}