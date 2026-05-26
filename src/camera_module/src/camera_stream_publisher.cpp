#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraStreamPublisherNode : public rclcpp::Node {
public:
    CameraStreamPublisherNode() : Node("camera_stream_publisher_node") {
        // Declare parameters
        int camera_index = this->declare_parameter<int>("camera_index", 0);
        int frame_width = this->declare_parameter<int>("frame_width", 640);
        int frame_height = this->declare_parameter<int>("frame_height", 480);
        int fps = this->declare_parameter<int>("fps", 30);

        // Initialize Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

        // Build the GStreamer pipeline dynamically using parameters
        std::string pipeline = 
            "nvarguscamerasrc sensor-id=" + std::to_string(camera_index) + " ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=" + std::to_string(fps) + "/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=" + std::to_string(frame_width) + ", height=" + std::to_string(frame_height) + ", format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink";

        // Open camera
        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream with GStreamer!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Camera opened: %dx%d @ %dfps", frame_width, frame_height, fps);
        }

        // CORRECTED: Bind now points to the actual class name
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000/fps),
                std::bind(&CameraStreamPublisherNode::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            return; // Silent return to avoid flooding logs
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
    rclcpp::spin(std::make_shared<CameraStreamPublisherNode>());
    rclcpp::shutdown();
    return 0;
}