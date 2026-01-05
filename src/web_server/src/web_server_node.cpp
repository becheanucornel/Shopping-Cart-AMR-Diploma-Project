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

  try {
    resource_path_ = ament_index_cpp::get_package_share_directory("web_server") + "/resource";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get package directory: %s", e.what());
    resource_path_ = "./resource";
  }

  // Create ROS 2 publishers and subscribers
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&WebServerNode::odom_callback, this, std::placeholders::_1));
  
  amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10, std::bind(&WebServerNode::amcl_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Starting web server on port %d", port_);
  RCLCPP_INFO(this->get_logger(), "Serving files from: %s", resource_path_.c_str());
  std::cout << "Web server starting on port " << port_ << std::endl;

  ioc_ = std::make_shared<net::io_context>(1);
  server_thread_ = std::make_unique<std::thread>(&WebServerNode::run_server, this);
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
    
    // Handle API endpoints
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