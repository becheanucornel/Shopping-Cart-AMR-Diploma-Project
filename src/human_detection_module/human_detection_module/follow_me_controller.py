import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import math

class FollowMeController(Node):
    def __init__(self):
        super().__init__('follow_me_controller')

        # Parametrii Camerei si Controlului
        self.declare_parameter('camera_width', 640.0)
        self.declare_parameter('target_height', 280.0) # Inaltimea ideala a bounding box-ului in pixeli (distanta optima)
        
        # Gain-uri Proportionale (Kp) - Cat de agresiv sa fie controlul
        self.declare_parameter('kp_linear', 0.003)
        self.declare_parameter('kp_angular', 0.004)
        
        # Siguranta LiDAR
        self.declare_parameter('safety_distance', 0.6) # Metri

        self.camera_center_x = self.get_parameter('camera_width').value / 2.0
        self.target_height = self.get_parameter('target_height').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.safety_distance = self.get_parameter('safety_distance').value

        # Stare interna
        self.is_following = False
        self.front_clearance = 999.0 # Distanta pana la cel mai apropiat obstacol frontal

        # Publisher catre ModeManager
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_follow', 10)

        # Service Client pentru a porni/opri YOLO
        self.yolo_client = self.create_client(SetBool, '/human_detector/enable')

        # Subscribers
        self.mode_sub = self.create_subscription(String, '/mode', self.mode_callback, 10)
        self.target_sub = self.create_subscription(Point, '/human/target_center', self.target_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar_front/scan', self.lidar_callback, 10)

        self.get_logger().info("Follow-Me Controller (Visual Servoing) a pornit.")

    def set_yolo_state(self, state: bool):
        """Activeaza sau dezactiveaza detectia YOLO pentru a salva resurse."""
        if not self.yolo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Serviciul YOLO nu este valabil!")
            return

        req = SetBool.Request()
        req.data = state
        
        future = self.yolo_client.call_async(req)
        future.add_done_callback(self.yolo_response_callback)

    def yolo_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"YOLO State Update: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Eroare la apelarea serviciului YOLO: {e}")

    def mode_callback(self, msg: String):
        """Monitorizeaza starea robotului (din UI/ModeManager)."""
        if msg.data == "FOLLOWING":
            if not self.is_following:
                self.get_logger().info("Intrat in modul FOLLOW. Activez YOLO.")
                self.is_following = True
                self.set_yolo_state(True)
        else:
            if self.is_following:
                self.get_logger().info("Iesit din modul FOLLOW. Opresc YOLO.")
                self.is_following = False
                self.set_yolo_state(False)
                
                # Trimitem un semnal de oprire din siguranta
                stop_msg = Twist()
                self.cmd_pub.publish(stop_msg)

    def lidar_callback(self, msg: LaserScan):
        """Citeste lidarul frontal pentru a preveni coliziunile."""
        if not self.is_following:
            return

        # Vrem sa ne uitam doar in fata robotului (ex: un con de la -30 grade la +30 grade)
        # In ROS, 0 radian este in fata, deci verificam inceputul si sfarsitul array-ului de scan
        
        angle_range = math.radians(30)
        idx_range = int(angle_range / msg.angle_increment)
        
        # Preluam razele din stanga-fata (primele elemente) si dreapta-fata (ultimele elemente)
        front_ranges = msg.ranges[:idx_range] + msg.ranges[-idx_range:]
        
        # Filtram valorile invalide (infinit sau NaN) si distantele 0
        valid_ranges = [r for r in front_ranges if r > msg.range_min and r < msg.range_max and not math.isinf(r) and not math.isnan(r)]
        
        if valid_ranges:
            self.front_clearance = min(valid_ranges)
        else:
            self.front_clearance = 999.0

    def target_callback(self, msg: Point):
        """Calculeaza vitezele pe baza datelor de la camera (Visual Servoing)."""
        if not self.is_following:
            return

        target_x = msg.x
        target_height = msg.z

        twist = Twist()

        # 1. ROTAȚIA (Axa Z)
        # Eroarea este diferenta dintre centrul camerei si centrul omului
        error_x = self.camera_center_x - target_x
        # Aplicam P-Controller: Viteza = Constanta * Eroarea
        angular_speed = self.kp_angular * error_x
        
        # Limitam viteza de rotatie la un maxim de siguranta (ex: 1.0 rad/s)
        twist.angular.z = max(min(angular_speed, 1.0), -1.0)


        # 2. ÎNAINTAREA (Axa X)
        # Eroarea este diferenta dintre inaltimea ideala si inaltimea curenta
        error_height = self.target_height - target_height
        linear_speed = self.kp_linear * error_height
        
        # Limitam viteza liniara la un maxim de siguranta (ex: 0.5 m/s)
        # Limităm și mersul înapoi (-0.2 m/s) ca să nu dăm cu spatele prea tare dacă omul vine spre robot
        twist.linear.x = max(min(linear_speed, 0.5), -0.2)

        # --- SAFETY OVERRIDE (Senzorul de Parcare) ---
        if twist.linear.x > 0.0 and self.front_clearance < self.safety_distance:
            self.get_logger().warn(f"OBSTACOL LA {self.front_clearance:.2f}m! Frana automata (YOLO Override).", throttle_duration_sec=1.0)
            twist.linear.x = 0.0 # Oprim inaintarea
            
            # (Optional) Dacă vrem, putem lăsa rotatia activa ca robotul sa ramana intors spre om 
            # chiar daca nu poate inainta.

        # Publicam comanda catre ModeManager
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowMeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()