#ifndef WEB_SERVER__WEB_SERVER_NODE_HPP_
#define WEB_SERVER__WEB_SERVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <thread>
#include <memory>
#include <mutex>

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = boost::beast::http;
using tcp = net::ip::tcp;

namespace web_server
{

class WebServerNode : public rclcpp::Node
{
public:
  explicit WebServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~WebServerNode();

private:
  void run_server();
  void handle_session(tcp::socket socket);
  void handle_api_request(const std::string & path, tcp::socket & socket);
  std::string get_mime_type(const std::string & path);
  std::string load_file(const std::string & path);
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  int port_;
  std::string resource_path_;
  bool running_;
  std::shared_ptr<net::io_context> ioc_;
  std::unique_ptr<std::thread> server_thread_;
  
  // ROS 2 pub/sub
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  
  // Robot state
  std::mutex data_mutex_;
  geometry_msgs::msg::Pose current_pose_;
  nav_msgs::msg::Odometry current_odom_;
};

}  // namespace web_server

#endif  // WEB_SERVER__WEB_SERVER_NODE_HPP_