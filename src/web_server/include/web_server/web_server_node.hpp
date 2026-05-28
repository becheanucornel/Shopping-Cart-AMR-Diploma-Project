#ifndef WEB_SERVER__WEB_SERVER_NODE_HPP_
#define WEB_SERVER__WEB_SERVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <thread>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <atomic>

namespace net   = boost::asio;
namespace beast = boost::beast;
namespace http  = boost::beast::http;
using tcp       = net::ip::tcp;

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
  void handle_mjpeg_stream(const std::string & path, tcp::socket socket);
  std::string get_mime_type(const std::string & path);
  std::string load_file(const std::string & path);

  void send_pending_goal();
  void preempt_timer_cb();
  void follow_goal_timer_cb();

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void goal_cancel_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void ui_mode_callback(const std_msgs::msg::String::SharedPtr msg);
  void yolo_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Camera stream callbacks
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void debug_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  using NavigateToPose           = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  int         port_;
  std::string resource_path_;
  bool        running_;
  std::shared_ptr<net::io_context>   ioc_;
  std::unique_ptr<std::thread>       server_thread_;

  // ── ROS pubs / subs ───────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr            goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr                       goal_cancel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                       odom_nav2_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

  // ── Mode management ───────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ui_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    mode_pub_;
  std::string current_mode_{"IDLE"};
  rclcpp::TimerBase::SharedPtr mode_publish_timer_;

  // ── Follow Me / YOLO ──────────────────────────────────────────────────────
  bool        is_following_active_{false};
  std::string follow_me_xml_path_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr yolo_target_sub_;
  rclcpp::TimerBase::SharedPtr              follow_goal_timer_;
  geometry_msgs::msg::PoseStamped           pending_follow_pose_;
  bool                                      has_pending_follow_{false};

  // ── Nav2 action client ────────────────────────────────────────────────────
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNavigateToPose::SharedPtr              current_goal_handle_;
  bool                                             cancel_in_progress_{false};
  geometry_msgs::msg::PoseStamped                  pending_goal_pose_;
  bool                                             pending_goal_{false};
  rclcpp::TimerBase::SharedPtr                     preempt_timer_;

  // ── Robot state ───────────────────────────────────────────────────────────
  std::mutex               data_mutex_;
  geometry_msgs::msg::Pose current_pose_;
  nav_msgs::msg::Odometry  current_odom_;

  bool        publish_odom_tf_{true};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool        auto_initialpose_from_odom_{false};
  bool        initialpose_sent_{false};
  std::string initialpose_frame_id_{"map"};
  double      initialpose_cov_xy_{0.25};
  double      initialpose_cov_yaw_{0.25};

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── MJPEG stream state ────────────────────────────────────────────────────
  // Latest JPEG frames, protected by their own mutexes for lock-free reads
  std::mutex         camera_frame_mutex_;
  std::vector<uchar> latest_camera_jpeg_;

  std::mutex         debug_frame_mutex_;
  std::vector<uchar> latest_debug_jpeg_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_image_sub_;

  // MJPEG stream quality (0-100)
  int jpeg_quality_{75};
};

}  // namespace web_server

#endif  // WEB_SERVER__WEB_SERVER_NODE_HPP_
