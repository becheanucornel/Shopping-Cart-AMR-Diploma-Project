#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

#define CUDA_CHECK(call)                                                        \
  do {                                                                          \
    cudaError_t err = (call);                                                   \
    if (err != cudaSuccess) {                                                   \
      throw std::runtime_error(std::string("CUDA error at " __FILE__ ":") +    \
                               std::to_string(__LINE__) + " — " +              \
                               cudaGetErrorString(err));                        \
    }                                                                           \
  } while (0)

class TRTLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char * msg) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "[TRT] %s\n", msg);
    }
  }
};

struct TRTDestroy {
  template<class T>
  void operator()(T * obj) const { if (obj) obj->destroy(); }
};

static std::vector<char> load_file(const std::string & path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) throw std::runtime_error("Cannot open file: " + path);
  auto size = f.tellg();
  f.seekg(0);
  std::vector<char> buf(size);
  f.read(buf.data(), size);
  return buf;
}

struct Detection {
  float x, y, w, h;
  float conf;
  int   class_id;
};

static std::vector<Detection> nms(
  std::vector<Detection> & dets, float iou_thresh)
{
  std::sort(dets.begin(), dets.end(),
    [](const Detection & a, const Detection & b){ return a.conf > b.conf; });

  std::vector<Detection> out;
  std::vector<bool> suppressed(dets.size(), false);

  for (size_t i = 0; i < dets.size(); ++i) {
    if (suppressed[i]) continue;
    out.push_back(dets[i]);
    float x1i = dets[i].x - dets[i].w / 2;
    float y1i = dets[i].y - dets[i].h / 2;
    float x2i = dets[i].x + dets[i].w / 2;
    float y2i = dets[i].y + dets[i].h / 2;
    float ai  = dets[i].w * dets[i].h;

    for (size_t j = i + 1; j < dets.size(); ++j) {
      if (suppressed[j]) continue;
      float x1j = dets[j].x - dets[j].w / 2;
      float y1j = dets[j].y - dets[j].h / 2;
      float x2j = dets[j].x + dets[j].w / 2;
      float y2j = dets[j].y + dets[j].h / 2;

      float ix1 = std::max(x1i, x1j);
      float iy1 = std::max(y1i, y1j);
      float ix2 = std::min(x2i, x2j);
      float iy2 = std::min(y2i, y2j);

      float inter = std::max(0.0f, ix2 - ix1) * std::max(0.0f, iy2 - iy1);
      float aj    = dets[j].w * dets[j].h;
      float iou   = inter / (ai + aj - inter + 1e-6f);
      if (iou > iou_thresh) suppressed[j] = true;
    }
  }
  return out;
}

class DetectionNodeCpp : public rclcpp::Node
{
public:
  DetectionNodeCpp() : Node("detection_node_cpp"), target_class_id_(0)
  {
    // ── Parameters ─────────────────────────────────────────────────────────
    declare_parameter<double>("confidence_threshold", 0.50);
    declare_parameter<double>("nms_threshold",        0.45);
    declare_parameter<double>("focal_length_px",      500.0);
    declare_parameter<double>("person_real_height_m", 1.70);
    declare_parameter<double>("ball_real_height_m",   0.04);
    declare_parameter<int>   ("input_width",          640);
    declare_parameter<int>   ("input_height",         640);
    declare_parameter<int>   ("min_box_height_px",    20);

    conf_thresh_    = get_parameter("confidence_threshold").as_double();
    nms_thresh_     = get_parameter("nms_threshold").as_double();
    focal_length_   = get_parameter("focal_length_px").as_double();
    person_height_  = get_parameter("person_real_height_m").as_double();
    ball_height_    = get_parameter("ball_real_height_m").as_double();
    input_w_        = get_parameter("input_width").as_int();
    input_h_        = get_parameter("input_height").as_int();
    min_box_h_      = get_parameter("min_box_height_px").as_int();

    std::string share =
      ament_index_cpp::get_package_share_directory("human_detection_module");
    onnx_path_ = share + "/model/yolov8n.onnx";

    const char* home = std::getenv("HOME");
    std::string engine_dir = std::string(home ? home : "/tmp") + "/.ros/human_detection";
    // Create directory if it doesn't exist
    std::filesystem::create_directories(engine_dir);
    engine_path_ = engine_dir + "/yolov8n.engine";

    trt_ready_ = false;
    try {
      init_tensorrt();
      allocate_buffers();
      trt_ready_ = true;
      RCLCPP_INFO(get_logger(), "TensorRT ready. Inference enabled.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "TRT init failed: %s — node will publish placeholder frames only.",
        e.what());
    }

    srv_ = create_service<std_srvs::srv::SetBool>(
      "/detector/set_class",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
        target_class_id_ = req->data ? 32 : 0;
        res->success = true;
        res->message = req->data ? "Tracking: sports ball (32)"
                                 : "Tracking: person (0)";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    // ── Pub / Sub ───────────────────────────────────────────────────────────
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera", rclcpp::SensorDataQoS(),
      std::bind(&DetectionNodeCpp::image_callback, this, _1));

    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/yolo/target_pose", 10);

    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/yolo/debug_image", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
      "Detection node ready (TensorRT %.1f FP16). "
      "conf=%.2f  nms=%.2f  focal=%.0fpx",
      static_cast<float>(NV_TENSORRT_MAJOR) +
      static_cast<float>(NV_TENSORRT_MINOR) / 10.0f,
      conf_thresh_, nms_thresh_, focal_length_);
  }

  ~DetectionNodeCpp()
  {
    if (d_input_)  cudaFree(d_input_);
    if (d_output_) cudaFree(d_output_);
    if (stream_)   cudaStreamDestroy(stream_);
  }

private:
  TRTLogger                                          trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime>                runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine>             engine_;
  std::unique_ptr<nvinfer1::IExecutionContext>        context_;

  cudaStream_t stream_{nullptr};
  void *       d_input_{nullptr};
  void *       d_output_{nullptr};
  size_t       input_bytes_{0};
  size_t       output_bytes_{0};

  std::string onnx_path_;
  std::string engine_path_;
  bool        trt_ready_{false};
  int         target_class_id_;
  double      conf_thresh_;
  double      nms_thresh_;
  double      focal_length_;
  double      person_height_;
  double      ball_height_;
  int         input_w_;
  int         input_h_;
  int         min_box_h_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          debug_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr             srv_;

  std::vector<float> h_input_;

  void init_tensorrt()
  {
    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) throw std::runtime_error("Failed to create TRT IRuntime");

    std::ifstream engine_file(engine_path_, std::ios::binary);
    if (engine_file.good()) {
      RCLCPP_INFO(get_logger(), "Loading cached TRT engine: %s", engine_path_.c_str());
      auto buf = load_file(engine_path_);
      engine_.reset(
        runtime_->deserializeCudaEngine(buf.data(), buf.size()));
    } else {
      RCLCPP_WARN(get_logger(),
        "No cached engine found. Building from ONNX: %s", onnx_path_.c_str());
      RCLCPP_WARN(get_logger(),
        "This takes ~90 seconds on the Orin Nano Super. Please wait...");
      build_engine_from_onnx();
    }

    if (!engine_) throw std::runtime_error("Failed to create/load TRT engine");

    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("Failed to create TRT execution context");

    RCLCPP_INFO(get_logger(), "TensorRT engine ready.");
  }

  void build_engine_from_onnx()
  {
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(
      nvinfer1::createInferBuilder(trt_logger_));
    if (!builder) throw std::runtime_error("createInferBuilder failed");

    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
      builder->createNetworkV2(0U));
    if (!network) throw std::runtime_error("createNetworkV2 failed");

    auto parser = std::unique_ptr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, trt_logger_));
    if (!parser) throw std::runtime_error("createParser failed");

    if (!parser->parseFromFile(onnx_path_.c_str(),
          static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
      throw std::runtime_error("ONNX parse failed: " + onnx_path_);
    }

    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(
      builder->createBuilderConfig());
    if (!config) throw std::runtime_error("createBuilderConfig failed");

    if (builder->platformHasFastFp16()) {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
      RCLCPP_INFO(get_logger(), "FP16 enabled.");
    }

    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                               1ULL << 30);

    auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
      builder->buildSerializedNetwork(*network, *config));
    if (!serialized) throw std::runtime_error("buildSerializedNetwork failed");

    std::ofstream out(engine_path_, std::ios::binary);
    if (out) {
      out.write(static_cast<const char*>(serialized->data()), serialized->size());
      RCLCPP_INFO(get_logger(), "Engine saved to: %s", engine_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Could not save engine to disk (non-fatal).");
    }

    engine_.reset(
      runtime_->deserializeCudaEngine(serialized->data(), serialized->size()));
  }

  void allocate_buffers()
  {
    CUDA_CHECK(cudaStreamCreate(&stream_));

    input_bytes_  = 1 * 3 * input_h_ * input_w_ * sizeof(float);
    output_bytes_ = 1 * 84 * 8400    * sizeof(float);

    CUDA_CHECK(cudaMalloc(&d_input_,  input_bytes_));
    CUDA_CHECK(cudaMalloc(&d_output_, output_bytes_));

    h_input_.resize(3 * input_h_ * input_w_);

    const char * input_name  = engine_->getIOTensorName(0);
    const char * output_name = engine_->getIOTensorName(1);

    context_->setTensorAddress(input_name,  d_input_);
    context_->setTensorAddress(output_name, d_output_);

    RCLCPP_INFO(get_logger(),
      "CUDA buffers allocated. Input='%s' Output='%s'",
      input_name, output_name);
  }

  cv::Mat letterbox(const cv::Mat & src,
                    float & scale, float & pad_x, float & pad_y)
  {
    float r = std::min(
      static_cast<float>(input_w_) / src.cols,
      static_cast<float>(input_h_) / src.rows);
    scale = r;

    int new_w = static_cast<int>(std::round(src.cols * r));
    int new_h = static_cast<int>(std::round(src.rows * r));

    pad_x = (input_w_ - new_w) / 2.0f;
    pad_y = (input_h_ - new_h) / 2.0f;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    cv::Mat out(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(
      static_cast<int>(pad_x), static_cast<int>(pad_y), new_w, new_h)));
    return out;
  }

  void preprocess(const cv::Mat & img_bgr)
  {
    cv::Mat rgb;
    cv::cvtColor(img_bgr, rgb, cv::COLOR_BGR2RGB);

    const int planeSize = input_h_ * input_w_;
    for (int c = 0; c < 3; ++c) {
      for (int i = 0; i < input_h_; ++i) {
        const uchar * row = rgb.ptr<uchar>(i);
        for (int j = 0; j < input_w_; ++j) {
          h_input_[c * planeSize + i * input_w_ + j] =
            row[j * 3 + c] / 255.0f;
        }
      }
    }

    CUDA_CHECK(cudaMemcpyAsync(
      d_input_, h_input_.data(), input_bytes_,
      cudaMemcpyHostToDevice, stream_));
  }

  void infer()
  {
    // TRT10: enqueueV3 (replaces enqueueV2 from TRT8)
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TRT enqueueV3 failed");
    }
    CUDA_CHECK(cudaStreamSynchronize(stream_));
  }

  std::vector<Detection> postprocess(float scale, float pad_x, float pad_y,
                                     int orig_w, int orig_h)
  {
    const int   num_anchors = 8400;
    const int   num_cols    = 84;
    std::vector<float> h_output(num_anchors * num_cols);

    std::vector<float> raw(1 * num_cols * num_anchors);
    CUDA_CHECK(cudaMemcpy(raw.data(), d_output_, output_bytes_,
                          cudaMemcpyDeviceToHost));

    for (int a = 0; a < num_anchors; ++a) {
      for (int c = 0; c < num_cols; ++c) {
        h_output[a * num_cols + c] = raw[c * num_anchors + a];
      }
    }

    std::vector<Detection> candidates;
    candidates.reserve(256);

    for (int a = 0; a < num_anchors; ++a) {
      const float * row  = h_output.data() + a * num_cols;
      float         conf = row[4 + target_class_id_];
      if (conf < static_cast<float>(conf_thresh_)) continue;

      float cx = row[0], cy = row[1], w = row[2], h = row[3];

      cx = (cx - pad_x) / scale;
      cy = (cy - pad_y) / scale;
      w  = w  / scale;
      h  = h  / scale;

      cx = std::max(0.0f, std::min(cx, static_cast<float>(orig_w)));
      cy = std::max(0.0f, std::min(cy, static_cast<float>(orig_h)));
      w  = std::min(w, static_cast<float>(orig_w));
      h  = std::min(h, static_cast<float>(orig_h));

      if (h < static_cast<float>(min_box_h_)) continue;

      candidates.push_back({cx, cy, w, h, conf, target_class_id_});
    }

    return nms(candidates, static_cast<float>(nms_thresh_));
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    if (!trt_ready_) {
      cv::Mat overlay = frame.clone();
      const std::string msg_text = "Building TRT engine, please wait...";
      int baseline = 0;
      cv::Size ts = cv::getTextSize(msg_text, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
      cv::Point origin((overlay.cols - ts.width) / 2, overlay.rows / 2);
      cv::rectangle(overlay,
        cv::Point(origin.x - 8, origin.y - ts.height - 8),
        cv::Point(origin.x + ts.width + 8, origin.y + baseline + 8),
        cv::Scalar(0, 0, 0), cv::FILLED);
      cv::putText(overlay, msg_text, origin,
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 230, 118), 2);

      cv_bridge::CvImage cv_img;
      cv_img.header.stamp    = msg->header.stamp;
      cv_img.header.frame_id = "camera_link";
      cv_img.encoding        = sensor_msgs::image_encodings::BGR8;
      cv_img.image           = overlay;
      debug_pub_->publish(*cv_img.toImageMsg());
      return;
    }

    const int orig_w = frame.cols;
    const int orig_h = frame.rows;

    try {
      float scale, pad_x, pad_y;
      cv::Mat lb = letterbox(frame, scale, pad_x, pad_y);

      preprocess(lb);

      infer();

      auto dets = postprocess(scale, pad_x, pad_y, orig_w, orig_h);

      cv::Mat debug_frame = frame.clone();
      for (const auto & d : dets) {
        int x1 = static_cast<int>(d.x - d.w / 2);
        int y1 = static_cast<int>(d.y - d.h / 2);
        int x2 = static_cast<int>(d.x + d.w / 2);
        int y2 = static_cast<int>(d.y + d.h / 2);
        x1 = std::max(0, x1); y1 = std::max(0, y1);
        x2 = std::min(orig_w - 1, x2); y2 = std::min(orig_h - 1, y2);

        cv::Scalar colour = (&d == &dets[0])
          ? cv::Scalar(0, 255, 0)
          : cv::Scalar(0, 120, 255);

        cv::rectangle(debug_frame, cv::Point(x1, y1), cv::Point(x2, y2), colour, 2);

        const char * label = (d.class_id == 32) ? "ball" : "person";
        std::string text = std::string(label) + " " +
                           std::to_string(static_cast<int>(d.conf * 100)) + "%";
        int baseline = 0;
        cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
        cv::rectangle(debug_frame,
          cv::Point(x1, y1 - ts.height - 6),
          cv::Point(x1 + ts.width, y1), colour, cv::FILLED);
        cv::putText(debug_frame, text, cv::Point(x1, y1 - 4),
          cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1);
      }

      {
        cv_bridge::CvImage cv_img;
        cv_img.header.stamp    = msg->header.stamp;
        cv_img.header.frame_id = "camera_link";
        cv_img.encoding        = sensor_msgs::image_encodings::BGR8;
        cv_img.image           = debug_frame;
        debug_pub_->publish(*cv_img.toImageMsg());
      }

      if (dets.empty()) return;

      const Detection & best = dets[0];

      float real_h  = (target_class_id_ == 32)
                      ? static_cast<float>(ball_height_)
                      : static_cast<float>(person_height_);

      float dist    = (real_h * static_cast<float>(focal_length_)) /
                      std::max(best.h, 1.0f);

      float cx_px   = best.x;
      float offset  = cx_px - orig_w / 2.0f;
      float lateral = -(offset * dist) / static_cast<float>(focal_length_);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp    = msg->header.stamp;
      pose.header.frame_id = "camera_link";

      pose.pose.position.x = static_cast<double>(lateral);
      pose.pose.position.y = 0.0;
      pose.pose.position.z = static_cast<double>(dist);

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      target_pub_->publish(pose);

      RCLCPP_DEBUG(get_logger(),
        "Detection: class=%d conf=%.2f dist=%.2fm lat=%.2fm",
        target_class_id_, best.conf, dist, lateral);

    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Inference error: %s", e.what());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionNodeCpp>());
  rclcpp::shutdown();
  return 0;
}