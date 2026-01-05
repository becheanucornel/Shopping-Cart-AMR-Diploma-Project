#ifndef WEB_SERVER__WEB_SERVER_NODE_HPP_
#define WEB_SERVER__WEB_SERVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <memory>
#include <string>
#include <thread>

namespace web_server
{

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

class WebServerNode : public rclcpp::Node
{
public:
  explicit WebServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~WebServerNode();

private:
  void run_server();
  void handle_session(tcp::socket socket);
  std::string get_mime_type(const std::string & path);
  std::string load_file(const std::string & path);

  std::unique_ptr<std::thread> server_thread_;
  std::shared_ptr<net::io_context> ioc_;
  int port_;
  std::string resource_path_;
  bool running_;
};

}  // namespace web_server

#endif  // WEB_SERVER__WEB_SERVER_NODE_HPP_