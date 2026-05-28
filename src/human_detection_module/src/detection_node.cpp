#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_srvs/srv/set_bool.hpp> // ADĂUGAT

using std::placeholders::_1;

class DetectionNodeCpp : public rclcpp::Node {
public:
    DetectionNodeCpp() : Node("detection_node_cpp"), target_class_id_(0) {
        std::string model_path = ament_index_cpp::get_package_share_directory("human_detection_module") + "/model/yolov8n.onnx";
        
        try {
            net_ = cv::dnn::readNetFromONNX(model_path);
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            RCLCPP_INFO(this->get_logger(), "YOLOv8 ONNX încărcat nativ cu suport CUDA.");
        } catch(const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Eroare la încărcarea modelului: %s", e.what());
        }
        
        srv_ = this->create_service<std_srvs::srv::SetBool>("/detector/set_class",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr req, 
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
                this->target_class_id_ = req->data ? 32 : 0;
                res->success = true;
                res->message = req->data ? "Urmăresc mingea" : "Urmăresc omul";
            });

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera", 10, std::bind(&DetectionNodeCpp::image_callback, this, _1));
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/yolo/target_pose", 10);
    }

private:
    cv::dnn::Net net_;
    int target_class_id_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat blob = cv::dnn::blobFromImage(frame, 1/255.0, cv::Size(640, 640), cv::Scalar(0,0,0), true, false);
            net_.setInput(blob);
            std::vector<cv::Mat> outputs;
            net_.forward(outputs, net_.getUnconnectedOutLayersNames());
            
            cv::Mat output = outputs[0].reshape(1, outputs[0].size[2]);
            std::vector<float> confidences;
            std::vector<cv::Rect> boxes;

            for (int i = 0; i < output.rows; ++i) {
                float* data = output.ptr<float>(i);
                float confidence = data[4 + target_class_id_]; 
                if (confidence > 0.5) {
                    float cx = data[0], cy = data[1], w = data[2], h = data[3];
                    boxes.push_back(cv::Rect(cx - w/2, cy - h/2, w, h));
                    confidences.push_back(confidence);
                }
            }

            std::vector<int> indices;
            cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

            if (!indices.empty()) {
                cv::Rect box = boxes[indices[0]];
                float real_height = (target_class_id_ == 32) ? 0.04f : 1.7f;
                float focal_length = 500.0;
                
                float estimated_distance_x = (real_height * focal_length) / std::max((float)box.height, 1.0f);
                float estimated_lateral_y = -(((box.x + box.width/2.0) - frame.cols/2.0) * estimated_distance_x) / focal_length;
                
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.frame_id = "custom_base_link";
                pose_msg.header.stamp = this->now();
                pose_msg.pose.position.x = estimated_distance_x;
                pose_msg.pose.position.y = estimated_lateral_y;
                
                float yaw = std::atan2(estimated_lateral_y, estimated_distance_x);
                pose_msg.pose.orientation.z = std::sin(yaw / 2.0);
                pose_msg.pose.orientation.w = std::cos(yaw / 2.0);
                
                target_pub_->publish(pose_msg);
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Eroare CV_Bridge: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionNodeCpp>());
    rclcpp::shutdown();
    return 0;
}