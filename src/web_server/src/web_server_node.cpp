
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
  odom_frame_id_   = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_   = this->get_parameter("base_frame_id").as_string();

  this->declare_parameter("auto_initialpose_from_odom", false);
  this->declare_parameter("initialpose_frame_id", "map");
  this->declare_parameter("initialpose_cov_xy",  0.25);
  this->declare_parameter("initialpose_cov_yaw", 0.25);
  this->declare_parameter("jpeg_quality", 75);
  auto_initialpose_from_odom_ = this->get_parameter("auto_initialpose_from_odom").as_bool();
  initialpose_frame_id_       = this->get_parameter("initialpose_frame_id").as_string();
  initialpose_cov_xy_         = this->get_parameter("initialpose_cov_xy").as_double();
  initialpose_cov_yaw_        = this->get_parameter("initialpose_cov_yaw").as_double();
  jpeg_quality_               = this->get_parameter("jpeg_quality").as_int();

  try {
    resource_path_ = ament_index_cpp::get_package_share_directory("web_server") + "/resource";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get package directory: %s", e.what());
    resource_path_ = "./resource";
  }

  try {
    follow_me_xml_path_ =
      ament_index_cpp::get_package_share_directory("system_bringup")
      + "/behavior_trees/follow_me.xml";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "follow_me.xml not found: %s", e.what());
    follow_me_xml_path_ = "";
  }

  // TF2
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  odom_nav2_pub_   = this->create_publisher<nav_msgs::msg::Odometry>("/odom_nav2", 10);
  initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  mode_pub_        = this->create_publisher<std_msgs::msg::String>("/mode", 10);

  if (publish_odom_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Subscribers
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ui/nav_goal", 10, std::bind(&WebServerNode::goal_pose_callback, this, std::placeholders::_1));

  goal_cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/goal_cancel", 10, std::bind(&WebServerNode::goal_cancel_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/custom_odom_topic", 10, std::bind(&WebServerNode::odom_callback, this, std::placeholders::_1));

  amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10, std::bind(&WebServerNode::amcl_callback, this, std::placeholders::_1));

  ui_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/ui/mode/absolute", 10, std::bind(&WebServerNode::ui_mode_callback, this, std::placeholders::_1));

  yolo_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/yolo/target_pose", 10, std::bind(&WebServerNode::yolo_target_callback, this, std::placeholders::_1));

  // Camera stream subscribers — use SensorDataQoS to match camera publisher
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera", rclcpp::SensorDataQoS(),
    std::bind(&WebServerNode::camera_callback, this, std::placeholders::_1));

  debug_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/yolo/debug_image", rclcpp::SensorDataQoS(),
    std::bind(&WebServerNode::debug_image_callback, this, std::placeholders::_1));

  // Timers
  preempt_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&WebServerNode::preempt_timer_cb, this));

  mode_publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), [this]() {
      std_msgs::msg::String m; m.data = current_mode_;
      if (mode_pub_) mode_pub_->publish(m);
    });

  this->declare_parameter("follow_distance",       0.7);
  this->declare_parameter("min_goal_displacement", 0.3);
  follow_distance_       = this->get_parameter("follow_distance").as_double();
  min_goal_displacement_ = this->get_parameter("min_goal_displacement").as_double();

  // 1 Hz — gives Nav2 enough time to plan and start moving before the goal updates
  follow_goal_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), std::bind(&WebServerNode::follow_goal_timer_cb, this));

  RCLCPP_INFO(this->get_logger(), "Web server starting on port %d (MJPEG streams enabled)", port_);

  ioc_          = std::make_shared<net::io_context>(1);
  server_thread_ = std::make_unique<std::thread>(&WebServerNode::run_server, this);
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera frame callbacks — JPEG encode and store latest frame
// ─────────────────────────────────────────────────────────────────────────────
void WebServerNode::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std::vector<uchar> buf;
    cv::imencode(".jpg", frame, buf,
                 {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    std::lock_guard<std::mutex> lock(camera_frame_mutex_);
    latest_camera_jpeg_ = std::move(buf);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "camera_callback encode error: %s", e.what());
  }
}

void WebServerNode::debug_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std::vector<uchar> buf;
    cv::imencode(".jpg", frame, buf,
                 {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    std::lock_guard<std::mutex> lock(debug_frame_mutex_);
    latest_debug_jpeg_ = std::move(buf);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "debug_image_callback encode error: %s", e.what());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Mode callback
// ─────────────────────────────────────────────────────────────────────────────
void WebServerNode::ui_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Mode change: %s", msg->data.c_str());
  current_mode_ = msg->data;

  if (current_mode_ != "FOLLOWING") {
    is_following_active_  = false;
    has_pending_follow_   = false;
    has_sent_follow_goal_ = false;
  }

  if (current_mode_ == "MANUAL" || current_mode_ == "IDLE") {
    // Cancel all active Nav2 goals — async_cancel_all_goals covers the case
    // where current_goal_handle_ is null (follow goals cycle; the handle is
    // reset after each result, so there may be an in-flight goal with no handle).
    if (nav_client_) {
      RCLCPP_WARN(this->get_logger(), "Cancelling all Nav2 goals (switching to %s)", current_mode_.c_str());
      (void)nav_client_->async_cancel_all_goals();
      current_goal_handle_.reset();
    }
  }

  if (current_mode_ == "FOLLOWING") {
    is_following_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Follow-Me active. Waiting on /yolo/target_pose");
  }

  std_msgs::msg::String m; m.data = current_mode_;
  mode_pub_->publish(m);
}

// ─────────────────────────────────────────────────────────────────────────────
// YOLO target → transform to map → store for throttled send
// ─────────────────────────────────────────────────────────────────────────────
void WebServerNode::yolo_target_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!is_following_active_) return;

  geometry_msgs::msg::PoseStamped pose_in_map;
  try {
    pose_in_map = tf_buffer_->transform(*msg, "map", tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "TF camera_link→map failed: %s", ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_follow_pose_ = pose_in_map;
  has_pending_follow_  = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Follow goal timer — sends at ≤5 Hz to avoid flooding Nav2
// ─────────────────────────────────────────────────────────────────────────────
void WebServerNode::follow_goal_timer_cb()
{
  if (!is_following_active_) return;

  geometry_msgs::msg::PoseStamped target;
  double rx, ry;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_pending_follow_) return;
    target              = pending_follow_pose_;
    has_pending_follow_ = false;
    rx = current_pose_.position.x;
    ry = current_pose_.position.y;
  }

  if (!nav_client_) return;

  double px   = target.pose.position.x;
  double py   = target.pose.position.y;
  double dist = std::hypot(px - rx, py - ry);

  // Already within follow distance — let Nav2 stop naturally, don't send another goal
  if (dist < follow_distance_) return;

  // Target a point follow_distance behind the person (toward the robot)
  // so we never ask Nav2 to plan into the person's body
  double ux = (rx - px) / dist;
  double uy = (ry - py) / dist;
  double gx = px + ux * follow_distance_;
  double gy = py + uy * follow_distance_;

  // Suppress update if the new goal hasn't moved enough from the last sent goal
  if (has_sent_follow_goal_) {
    double delta = std::hypot(gx - last_follow_goal_x_, gy - last_follow_goal_y_);
    if (delta < min_goal_displacement_) return;
  }

  // Face the person from the approach point
  double angle = std::atan2(py - gy, px - gx);
  target.pose.position.x    = gx;
  target.pose.position.y    = gy;
  target.pose.orientation.x = 0.0;
  target.pose.orientation.y = 0.0;
  target.pose.orientation.z = std::sin(angle / 2.0);
  target.pose.orientation.w = std::cos(angle / 2.0);

  last_follow_goal_x_  = gx;
  last_follow_goal_y_  = gy;
  has_sent_follow_goal_ = true;

  RCLCPP_INFO(this->get_logger(),
    "Follow→Nav2: person(%.2f,%.2f) dist=%.2fm goal(%.2f,%.2f)",
    px, py, dist, gx, gy);

  NavigateToPose::Goal goal;
  goal.pose = target;
  if (!follow_me_xml_path_.empty()) goal.behavior_tree = follow_me_xml_path_;

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr gh) {
    if (gh) current_goal_handle_ = gh;
  };
  opts.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
    current_goal_handle_.reset();
    // Reset so the next significant detection immediately triggers a fresh goal
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      has_sent_follow_goal_ = false;
    }
  };
  nav_client_->async_send_goal(goal, opts);
}

// ─────────────────────────────────────────────────────────────────────────────
// Nav goal / cancel helpers
// ─────────────────────────────────────────────────────────────────────────────
void WebServerNode::send_pending_goal()
{
  if (!pending_goal_) return;

  NavigateToPose::Goal goal;
  goal.pose          = pending_goal_pose_;
  goal.behavior_tree = "";
  pending_goal_      = false;

  RCLCPP_INFO(this->get_logger(), "UI goal → Nav2: frame=%s x=%.3f y=%.3f",
    goal.pose.header.frame_id.c_str(),
    goal.pose.pose.position.x, goal.pose.pose.position.y);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr gh) {
    if (!gh) { RCLCPP_WARN(this->get_logger(), "Nav2 goal rejected"); return; }
    current_goal_handle_ = gh;
    RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted");
  };
  opts.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & r) {
    const char * s = "unknown";
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED) s = "succeeded";
    else if (r.code == rclcpp_action::ResultCode::ABORTED)  s = "aborted";
    else if (r.code == rclcpp_action::ResultCode::CANCELED)  s = "canceled";
    RCLCPP_INFO(this->get_logger(), "Nav2 goal %s", s);
    current_goal_handle_.reset();
  };
  (void)nav_client_->async_send_goal(goal, opts);
}

void WebServerNode::preempt_timer_cb()
{
  if (!cancel_in_progress_ || current_goal_handle_) return;
  cancel_in_progress_ = false;
  send_pending_goal();
}

void WebServerNode::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!nav_client_ || !nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Nav2 action server not available");
    return;
  }
  pending_goal_pose_ = *msg;
  pending_goal_      = true;

  if (current_goal_handle_) {
    if (!cancel_in_progress_) {
      cancel_in_progress_ = true;
      (void)nav_client_->async_cancel_goal(current_goal_handle_);
    }
    return;
  }
  send_pending_goal();
}

void WebServerNode::goal_cancel_callback(const std_msgs::msg::Empty::SharedPtr)
{
  if (!nav_client_ || !current_goal_handle_) return;
  RCLCPP_INFO(this->get_logger(), "Cancel requested");
  (void)nav_client_->async_cancel_goal(current_goal_handle_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Odometry / AMCL callbacks
// ─────────────────────────────────────────────────────────────────────────────
void WebServerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_odom_ = *msg;

  if (auto_initialpose_from_odom_ && !initialpose_sent_ && initialpose_pub_) {
    geometry_msgs::msg::PoseWithCovarianceStamped init;
    init.header.stamp    = msg->header.stamp;
    init.header.frame_id = initialpose_frame_id_;
    init.pose.pose       = msg->pose.pose;
    init.pose.covariance.fill(0.0);
    init.pose.covariance[0]  = initialpose_cov_xy_;
    init.pose.covariance[7]  = initialpose_cov_xy_;
    init.pose.covariance[35] = initialpose_cov_yaw_;
    initialpose_pub_->publish(init);
    initialpose_sent_ = true;
  }

  if (odom_nav2_pub_) {
    nav_msgs::msg::Odometry filtered = *msg;
    filtered.header.frame_id       = odom_frame_id_;
    filtered.child_frame_id        = base_frame_id_;
    filtered.twist.twist.linear.y  = 0.0;
    filtered.twist.twist.linear.z  = 0.0;
    filtered.twist.twist.angular.x = 0.0;
    filtered.twist.twist.angular.y = 0.0;
    odom_nav2_pub_->publish(filtered);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp       = filtered.header.stamp;
      tf.header.frame_id    = filtered.header.frame_id;
      tf.child_frame_id     = filtered.child_frame_id;
      tf.transform.translation.x = filtered.pose.pose.position.x;
      tf.transform.translation.y = filtered.pose.pose.position.y;
      tf.transform.translation.z = filtered.pose.pose.position.z;
      tf.transform.rotation      = filtered.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }
}

void WebServerNode::amcl_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_pose_ = msg->pose.pose;
}

// ─────────────────────────────────────────────────────────────────────────────
// HTTP server
// ─────────────────────────────────────────────────────────────────────────────
WebServerNode::~WebServerNode()
{
  running_ = false;
  if (ioc_) ioc_->stop();
  if (server_thread_ && server_thread_->joinable()) server_thread_->join();
}

void WebServerNode::run_server()
{
  try {
    auto const address = net::ip::make_address("0.0.0.0");
    tcp::acceptor acceptor{*ioc_, {address, static_cast<unsigned short>(port_)}};
    RCLCPP_INFO(this->get_logger(), "HTTP server listening on port %d", port_);

    while (running_) {
      tcp::socket socket{*ioc_};
      try {
        acceptor.accept(socket);
        // Peek at the path to decide if it's an MJPEG stream
        // (long-lived connection, needs its own thread)
        std::thread([this, s = std::move(socket)]() mutable {
          handle_session(std::move(s));
        }).detach();
      } catch (const std::exception & e) {
        if (running_) RCLCPP_DEBUG(this->get_logger(), "Accept error: %s", e.what());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Server error: %s", e.what());
  }
}

void WebServerNode::handle_session(tcp::socket socket)
{
  try {
    beast::flat_buffer               buffer;
    http::request<http::string_body> req;
    http::read(socket, buffer, req);

    std::string path = std::string(req.target());

    // FIX: Strip query string (?timestamp cache-bust etc.) before routing.
    // Without this, /stream/debug?1779982987818 falls through to the static
    // file handler and logs "File not found: .../resource/stream/debug?..."
    auto qs_pos = path.find('?');
    if (qs_pos != std::string::npos) {
      path = path.substr(0, qs_pos);
    }

    // MJPEG streams — long-lived, handled separately
    if (path == "/stream/camera" || path == "/stream/debug") {
      handle_mjpeg_stream(path, std::move(socket));
      return;
    }

    if (path.find("/api/") == 0) {
      handle_api_request(path, socket);
      return;
    }

    if (path == "/") path = "/index.html";

    http::response<http::string_body> res;
    std::string content = load_file(resource_path_ + path);

    if (!content.empty()) {
      res.result(http::status::ok);
      res.set(http::field::server, "ROS2 Web Server");
      res.set(http::field::content_type, get_mime_type(path));
      res.set(http::field::access_control_allow_origin, "*");
      res.body() = content;
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

// ─────────────────────────────────────────────────────────────────────────────
// MJPEG streaming — push frames as multipart/x-mixed-replace
// Each frame is sent as a JPEG boundary block.
// Runs in its own thread (detached from accept loop above).
// ─────────────────────────────────────────────────────────────────────────────
void WebServerNode::handle_mjpeg_stream(
  const std::string & path, tcp::socket socket)
{
  try {
    // Send HTTP header
    const std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace;boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Connection: close\r\n"
      "\r\n";
    boost::asio::write(socket, boost::asio::buffer(header));

    const bool is_debug = (path == "/stream/debug");

    while (running_) {
      std::vector<uchar> jpeg;

      if (is_debug) {
        std::lock_guard<std::mutex> lock(debug_frame_mutex_);
        jpeg = latest_debug_jpeg_;
      } else {
        std::lock_guard<std::mutex> lock(camera_frame_mutex_);
        jpeg = latest_camera_jpeg_;
      }

      if (jpeg.empty()) {
        // No frame yet from ROS — show a minimal placeholder
        cv::Mat placeholder(240, 320, CV_8UC3, cv::Scalar(15, 15, 15));
        cv::putText(placeholder, "Connecting...",
          cv::Point(70, 125), cv::FONT_HERSHEY_SIMPLEX,
          0.7, cv::Scalar(0, 230, 118), 1);
        cv::imencode(".jpg", placeholder, jpeg,
                     {cv::IMWRITE_JPEG_QUALITY, 60});
      }

      const std::string part_header =
        "--frame\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: " + std::to_string(jpeg.size()) + "\r\n"
        "\r\n";

      boost::system::error_code ec;
      boost::asio::write(socket, boost::asio::buffer(part_header), ec);
      if (ec) break;
      boost::asio::write(socket, boost::asio::buffer(jpeg.data(), jpeg.size()), ec);
      if (ec) break;
      boost::asio::write(socket, boost::asio::buffer(std::string("\r\n")), ec);
      if (ec) break;

      // ~20 FPS cap — avoids hammering the network on a Jetson
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  } catch (const std::exception & e) {
    // Client disconnected — normal, not an error
    (void)e;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// API endpoints
// ─────────────────────────────────────────────────────────────────────────────
void WebServerNode::handle_api_request(
  const std::string & path, tcp::socket & socket)
{
  http::response<http::string_body> res;
  res.set(http::field::server, "ROS2 Web Server");
  res.set(http::field::content_type, "application/json");
  res.set(http::field::access_control_allow_origin, "*");

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (path == "/api/robot/state") {
    res.result(http::status::ok);
    res.body() =
      "{\"connected\":true,\"position\":{\"x\":" +
      std::to_string(current_pose_.position.x) + ",\"y\":" +
      std::to_string(current_pose_.position.y) + "},\"orientation\":" +
      std::to_string(current_pose_.orientation.z) + "}";
  } else if (path == "/api/robot/mode") {
    res.result(http::status::ok);
    res.body() = "{\"mode\":\"" + current_mode_ + "\"}";
  } else {
    res.result(http::status::not_found);
    res.body() = "{\"error\":\"endpoint not found\"}";
  }

  res.prepare_payload();
  http::write(socket, res);
  socket.shutdown(tcp::socket::shutdown_send);
}

std::string WebServerNode::get_mime_type(const std::string & path)
{
  auto ends = [](const std::string & s, const std::string & ext) {
    return s.size() >= ext.size() &&
           s.compare(s.size() - ext.size(), ext.size(), ext) == 0;
  };
  if (ends(path, ".html")) return "text/html";
  if (ends(path, ".js"))   return "application/javascript";
  if (ends(path, ".css"))  return "text/css";
  if (ends(path, ".json")) return "application/json";
  if (ends(path, ".png"))  return "image/png";
  if (ends(path, ".jpg") || ends(path, ".jpeg")) return "image/jpeg";
  return "text/plain";
}

std::string WebServerNode::load_file(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    RCLCPP_WARN(this->get_logger(), "File not found: %s", path.c_str());
    return "";
  }
  std::stringstream buf;
  buf << file.rdbuf();
  return buf.str();
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