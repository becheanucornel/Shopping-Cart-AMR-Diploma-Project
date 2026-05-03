import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class YoloHumanTracker(Node):
    def __init__(self):
        super().__init__('yolo_human_tracker')

        self.get_logger().info("Loading YOLOv8 model for Body Tracking...")
        # NOTA: Pentru performanta MAXIMA pe Jetson, ar trebui sa folosesti un .engine (TensorRT) in loc de .pt
        self.model = YOLO('/home/apollo/MobileRobot/src/human_detection_module/model/yolov8n.pt') 
        self.get_logger().info("YOLOv8 Model Loaded.")

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(Point, '/human/target_center', 10)
        self.srv = self.create_service(SetBool, '/human_detector/enable', self.enable_callback)
        
        self.bridge = CvBridge()
        self.show_debug_window = True

        self.target_id = None 
        self.tracking_enabled = False 
        self.get_logger().info("Node ready. Waiting for START command...")

    def enable_callback(self, request, response):
        self.tracking_enabled = request.data
        if self.tracking_enabled:
            self.target_id = None
            response.message = "Tracking STARTED."
            self.get_logger().info("COMMAND RECEIVED: Start Tracking.")
        else:
            response.message = "Tracking STOPPED."
            self.get_logger().info("COMMAND RECEIVED: Stop Tracking.")
        response.success = True
        return response

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # Daca nu suntem in modul FOLLOW, doar afisam (daca e activat debug-ul) si dam return
        if not self.tracking_enabled:
            if self.show_debug_window:
                cv2.putText(cv_image, "Waiting for command...", (30, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                cv2.imshow("YOLOv8 Human Tracker", cv_image)
                cv2.waitKey(1)
            return

        # --- TRACKING ---
        # Folosim half=True pentru a rula modelul in FP16 (mult mai rapid pe GPU-urile Jetson)
        results = self.model.track(source=cv_image, classes=0, conf=0.5, persist=True, verbose=False, half=True, tracker="bytetrack.yaml")

        if results and results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            track_ids = results[0].boxes.id.cpu().numpy().astype(int)
            
            target_box = None
            
            # Daca nu avem un target, il alegem pe cel mai mare (cel mai apropiat)
            if self.target_id is None:
                max_area = 0
                selected_id = None
                selected_box = None
                for box, track_id in zip(boxes, track_ids):
                    x1, y1, x2, y2 = box
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        selected_id = track_id
                        selected_box = box
                
                if selected_id is not None:
                    self.target_id = selected_id
                    target_box = selected_box
                    self.get_logger().info(f"Target LOCKED on ID: {self.target_id}")
            else:
                # Daca avem target, il cautam in frame-ul curent
                if self.target_id in track_ids:
                    matches = np.where(track_ids == self.target_id)[0]
                    if len(matches) > 0:
                        index = matches[0]
                        target_box = boxes[index]
                else:
                    self.get_logger().warn(f"Target ID {self.target_id} lost.")
                    # Poti decomenta linia de mai jos daca vrei ca robotul sa aleaga alta persoana automat cand o pierde pe prima
                    # self.target_id = None 

            if target_box is not None:
                x1, y1, x2, y2 = target_box
                
                # Centrul bounding box-ului
                cx = x1 + (x2 - x1) / 2
                cy = y1 + (y2 - y1) / 2
                
                # Inaltimea bounding box-ului (pentru estimarea distantei)
                bbox_height = y2 - y1

                point_msg = Point()
                point_msg.x = float(cx)
                point_msg.y = float(cy)
                point_msg.z = float(bbox_height) # Am salvat inaltimea aici!
                
                self.publisher.publish(point_msg)

                if self.show_debug_window:
                    cv2.circle(cv_image, (int(cx), int(cy)), 15, (0, 255, 0), -1)
                    cv2.putText(cv_image, f"TRACKING ID: {self.target_id}", (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if self.show_debug_window:
            cv2.imshow("YOLOv8 Human Tracker", cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloHumanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()