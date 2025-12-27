#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

class TestCameraStreamPublisherNode : public rclcpp::Node {
 public:
    TestCameraStreamPublisherNode() : Node("test_camera_stream_publisher_node") {
        int camera_index = this->declare_parameter<int>("camera_index", 0);
        int frame_width = this->declare_parameter<int>("frame_width", 640);
        int frame_height = this->declare_parameter<int>("frame_height", 480);
        int fps = this->declare_parameter<int>("fps", 30);
        std::string fourcc = this->declare_parameter<std::string>("fourcc", "MJPG");

        cap_.open(camera_index, cv::CAP_V4L2);
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        } else {
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
            cap_.set(cv::CAP_PROP_FPS, fps);
            if (fourcc.size() == 4) {
                cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]));
            }
            RCLCPP_INFO(this->get_logger(), "Camera opened successfully (index=%d, %dx%d@%dfps, %s)", camera_index, frame_width, frame_height, fps, fourcc.c_str());
        }

        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&TestCameraStreamPublisherNode::timer_callback, this));
    }

 private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_frame";

        cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, frame);
        sensor_msgs::msg::Image out_msg;
        img_bridge.toImageMsg(out_msg);
        publisher_->publish(out_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestCameraStreamPublisherNode>());
    rclcpp::shutdown();
    return 0;
}