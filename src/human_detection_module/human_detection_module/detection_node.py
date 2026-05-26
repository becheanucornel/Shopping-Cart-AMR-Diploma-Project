import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import sys

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        # 1. Path Setup: Points to the 'model' folder in your package
        from ament_index_python.packages import get_package_share_directory
        self.model_dir = os.path.join(get_package_share_directory('human_detection_module'), 'model')
        
        # 2. Check and Compile the Unified Model
        self.get_logger().info("Checking for TensorRT engine...")
        self.ensure_model_compiled('yolov8n.pt')
        
        # 3. Load the single Engine
        self.get_logger().info("Loading YOLOv8 Engine into GPU memory...")
        engine_path = os.path.join(self.model_dir, 'yolov8n.engine')
        self.model = YOLO(engine_path, task='detect')
        
        # Default to Human (Class 0). Sports Ball is Class 32.
        self.target_class_id = 0 
        
        # 4. ROS2 Infrastructure
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.target_pub = self.create_publisher(PointStamped, '/follow_point', 10)
        self.srv = self.create_service(SetBool, '/detector/set_class', self.switch_mode_callback)
        
        self.get_logger().info("DetectionNode ready. Default: Human Tracking (Class 0).")

    def ensure_model_compiled(self, source_pt):
        """Checks if the .engine exists. If not, compiles it from the .pt file."""
        base_name = source_pt.replace('.pt', '')
        engine_path = os.path.join(self.model_dir, f"{base_name}.engine")
        pt_path = os.path.join(self.model_dir, source_pt)
        
        if not os.path.exists(engine_path):
            self.get_logger().warn(f"Engine not found. Compiling {source_pt} to TensorRT... (This takes 5-15 mins)")
            try:
                # Load the PyTorch model and export it to TensorRT
                model = YOLO(pt_path)
                model.export(format='engine', device=0, half=True)
                self.get_logger().info("Model compiled to TensorRT successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to compile model: {e}")
                sys.exit(1)

    def switch_mode_callback(self, request, response):
        """Switches the YOLO target class dynamically via Web UI."""
        if request.data: # True -> Ball
            self.target_class_id = 32
            response.message = "Switched to Ball Detection (Class 32)"
        else: # False -> Human
            self.target_class_id = 0
            response.message = "Switched to Human Detection (Class 0)"
        
        self.get_logger().info(response.message)
        response.success = True
        return response

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Predict only the currently selected class
            results = self.model.predict(
                cv_image, classes=[self.target_class_id], 
                half=True, verbose=False, conf=0.5
            )
            
            if results and results[0].boxes:
                # Target acquired: Get the highest confidence detection
                box = results[0].boxes[0].xyxy.cpu().numpy()[0]
                
                # Calculate Bounding Box Center and Height
                cx = float(box[0] + (box[2] - box[0]) / 2)
                cy = float(box[1] + (box[3] - box[1]) / 2)
                bbox_height = float(box[3] - box[1])
                
                # --- SPATIAL ESTIMATION FOR NAV2 DPF ---
                # Nav2 Dynamic Point Follower expects meters (X forward, Y left), NOT pixels.
                # We must fake a 3D coordinate using the bounding box height and center.
                
                # Constants (You can tune these later for your specific camera)
                image_center_x = 320.0 # Assuming 640x480 resolution
                focal_length = 500.0   # Approximate focal length in pixels
                real_height = 1.7 if self.target_class_id == 0 else 0.22 # Human ~1.7m, Ball ~0.22m
                
                # 1. Estimate distance (X in base_link) using similar triangles
                estimated_distance_x = (real_height * focal_length) / max(bbox_height, 1.0)
                
                # 2. Estimate lateral offset (Y in base_link)
                # If target is left of center (cx < 320), Y is positive in ROS
                estimated_lateral_y = -((cx - image_center_x) * estimated_distance_x) / focal_length
                
                # Publish the PointStamped
                pt = PointStamped()
                pt.header.frame_id = "custom_base_link" # Tell Nav2 this point is relative to the robot
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x = float(estimated_distance_x)
                pt.point.y = float(estimated_lateral_y)
                pt.point.z = 0.0
                
                self.target_pub.publish(pt)
                
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