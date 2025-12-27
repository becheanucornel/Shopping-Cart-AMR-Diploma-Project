import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import mediapipe as mp # Importam MediaPipe

class YoloHumanTracker(Node):
    def __init__(self):
        super().__init__('yolo_human_tracker')

        self.get_logger().info("Loading YOLOv8 model for Body Tracking...")
        self.model = YOLO('/home/apollo/MobileRobot/src/human_detection_module/model/yolov8n.pt') 
        self.get_logger().info("YOLOv8 Model Loaded.")

        # --- MODIFICARE: Initializare MediaPipe Face Detection ---
        # model_selection=0 este optimizat pentru distante scurte (camere robot/webcam)
        # model_selection=1 este pentru distante mari (peste 5 metri)
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detector = self.mp_face_detection.FaceDetection(
            model_selection=0, 
            min_detection_confidence=0.5
        )
        self.get_logger().info("MediaPipe Face Detector Loaded (High Performance).")

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
        self.get_logger().info("Node ready. GDPR Active. Waiting for START command...")

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

    def apply_face_blur(self, image):
        """
        Detectie fete folosind MediaPipe (mult mai robust la profil si rotatii).
        """
        h, w, _ = image.shape
        
        # MediaPipe are nevoie de RGB, OpenCV foloseste BGR
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Procesarea imaginii pentru a gasi fete
        results = self.face_detector.process(rgb_image)

        if results.detections:
            for detection in results.detections:
                # Extragem cutia (bounding box) relativa (0.0 - 1.0)
                bboxC = detection.location_data.relative_bounding_box
                
                # Convertim la pixeli
                x = int(bboxC.xmin * w)
                y = int(bboxC.ymin * h)
                width = int(bboxC.width * w)
                height = int(bboxC.height * h)

                # Corectii pentru a nu iesi din imagine (safety check)
                x = max(0, x)
                y = max(0, y)
                width = min(w - x, width)
                height = min(h - y, height)

                # Extragem zona fetei
                roi = image[y:y+height, x:x+width]
                
                if roi.size > 0:
                    # Aplicam Blur
                    # Kernel size (51, 51) este destul de puternic
                    blurred_roi = cv2.GaussianBlur(roi, (51, 51), 30)
                    image[y:y+height, x:x+width] = blurred_roi

        return image

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # --- PASUL 1: GDPR BLUR (Acum cu MediaPipe) ---
        # Se executa instantaneu
        cv_image = self.apply_face_blur(cv_image)

        # --- PASUL 2: Verificam daca suntem activati ---
        if not self.tracking_enabled:
            if self.show_debug_window:
                cv2.putText(cv_image, "Wating for command...", (30, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                cv2.imshow("YOLOv8 Human Tracker", cv_image)
                cv2.waitKey(1)
            return

        # --- PASUL 3: TRACKING ---
        results = self.model.track(source=cv_image, classes=0, conf=0.5, persist=True, verbose=False, tracker="bytetrack.yaml")

        if results and results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            track_ids = results[0].boxes.id.cpu().numpy().astype(int)
            
            target_box = None
            
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
                if self.target_id in track_ids:
                    matches = np.where(track_ids == self.target_id)[0]
                    if len(matches) > 0:
                        index = matches[0]
                        target_box = boxes[index]
                else:
                    self.get_logger().warn(f"Target ID {self.target_id} lost.")

            if target_box is not None:
                x1, y1, x2, y2 = target_box
                cx = x1 + (x2 - x1) / 2
                cy = y1 + (y2 - y1) / 2

                point_msg = Point()
                point_msg.x = float(cx)
                point_msg.y = float(cy)
                point_msg.z = 0.0 
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
        # Eliberam resursele MediaPipe corect
        if hasattr(node, 'face_detector'):
            node.face_detector.close()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()