import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped # Modificat pentru integrarea cu Web Serverul
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import os
import sys
import math

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        from ament_index_python.packages import get_package_share_directory
        self.model_dir = os.path.join(get_package_share_directory('human_detection_module'), 'model')
        
        # 1. VERIFICARE SMART CUDA
        if torch.cuda.is_available():
            self.device = 'cuda'
            self.get_logger().info(f"CUDA activat cu succes! GPU: {torch.cuda.get_device_name(0)}")
        else:
            self.device = 'cpu'
            self.get_logger().warn("ATENȚIE: PyTorch nu recunoaște CUDA 12.6! Rulez pe CPU temporar.")

        self.get_logger().info("Checking for TensorRT engine...")
        self.model_path = self.ensure_model_compiled('yolov8n.pt')
        
        self.get_logger().info(f"Loading YOLOv8 model: {self.model_path}")
        self.model = YOLO(self.model_path, task='detect')
        
        self.target_class_id = 0 
        
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)
        
        # 2. RUTEAZĂ DATELE CĂTRE NODUL C++ (PoseStamped pe /yolo/target_pose)
        self.target_pub = self.create_publisher(PoseStamped, '/yolo/target_pose', 10)
        self.srv = self.create_service(SetBool, '/detector/set_class', self.switch_mode_callback)
        
        self.get_logger().info("DetectionNode ready. Default: Human Tracking (Class 0).")

    def ensure_model_compiled(self, source_pt):
        base_name = source_pt.replace('.pt', '')
        engine_path = os.path.join(self.model_dir, f"{base_name}.engine")
        pt_path = os.path.join(self.model_dir, source_pt)
        
        # Dacă PyTorch e pe CPU, TensorRT va eșua. Facem fallback direct la .pt
        if self.device == 'cpu':
            self.get_logger().warn("Sari peste exportul TensorRT. Folosesc fișierul .pt standard.")
            return pt_path

        if not os.path.exists(engine_path):
            self.get_logger().warn(f"Engine not found. Compiling {source_pt} to TensorRT...")
            try:
                model = YOLO(pt_path)
                # Exportă folosind device-ul detectat, nu hardcodat
                model.export(format='engine', device=0, half=True)
                self.get_logger().info("Model compiled to TensorRT successfully.")
                return engine_path
            except Exception as e:
                self.get_logger().error(f"Failed to compile TensorRT: {e}. Fallback la .pt")
                return pt_path
        
        return engine_path

    def switch_mode_callback(self, request, response):
        if request.data:
            self.target_class_id = 32
            response.message = "Switched to Ball Detection (Class 32)"
        else:
            self.target_class_id = 0
            response.message = "Switched to Human Detection (Class 0)"
        
        self.get_logger().info(response.message)
        response.success = True
        return response

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Predict folosind device-ul detectat corect
            results = self.model.predict(
                cv_image, classes=[self.target_class_id], 
                device=self.device, half=(self.device == 'cuda'), verbose=False, conf=0.5
            )
            
            if results and results[0].boxes:
                box = results[0].boxes[0].xyxy.cpu().numpy()[0]
                
                cx = float(box[0] + (box[2] - box[0]) / 2)
                cy = float(box[1] + (box[3] - box[1]) / 2)
                bbox_height = float(box[3] - box[1])
                
                image_center_x = 320.0 
                focal_length = 500.0 
                real_height = 1.7 if self.target_class_id == 0 else 0.22 
                
                estimated_distance_x = (real_height * focal_length) / max(bbox_height, 1.0)
                estimated_lateral_y = -((cx - image_center_x) * estimated_distance_x) / focal_length
                
                # 3. CONSTRUIRE POSE STAMPED PENTRU NAV2 PURE PURSUIT
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = "custom_base_link" 
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                
                # Setăm poziția (X înainte, Y stânga/dreapta)
                pose_msg.pose.position.x = float(estimated_distance_x)
                pose_msg.pose.position.y = float(estimated_lateral_y)
                pose_msg.pose.position.z = 0.0
                
                # Calculăm unghiul (yaw) spre target ca robotul să se și rotească spre tine
                yaw = math.atan2(estimated_lateral_y, estimated_distance_x)
                pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
                pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
                
                self.target_pub.publish(pose_msg)
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()