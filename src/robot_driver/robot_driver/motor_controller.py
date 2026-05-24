import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import Jetson.GPIO as GPIO
import time
import threading
import math
import os

# Suppress the pinmux warning globally
os.environ["JETSON_GPIO_PINMUX_CHECK"] = "0"

# --- Kept Exactly from your Test Script ---
class SoftwarePWM:
    """A custom class to simulate PWM on standard digital I/O pins."""
    def __init__(self, pin, frequency):
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle = 0.0
        self.is_running = False
        self.thread = None
        
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)

    def start(self, duty_cycle):
        self.duty_cycle = duty_cycle
        self.is_running = True
        self.thread = threading.Thread(target=self._run_pwm, daemon=True)
        self.thread.start()

    def ChangeDutyCycle(self, duty_cycle):
        self.duty_cycle = max(0.0, min(100.0, duty_cycle))

    def stop(self):
        self.is_running = False
        if self.thread is not None:
            self.thread.join()
        GPIO.output(self.pin, GPIO.LOW)

    def _run_pwm(self):
        while self.is_running:
            if self.duty_cycle == 0.0:
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(0.05)
            elif self.duty_cycle == 100.0:
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(0.05)
            else:
                period = 1.0 / self.frequency
                time_high = period * (self.duty_cycle / 100.0)
                time_low = period - time_high
                
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(time_high)
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(time_low)


# --- ROS 2 Node Implementation ---
class AmrMotorNode(Node):
    def __init__(self):
        super().__init__('amr_motor_controller')

        # Configuration
        self.PIN_M1_FWD = 32   # Motor 1 (Left) - Hardware PWM
        self.PIN_M1_REV = 7    # Motor 1 (Left) - Software PWM
        self.PIN_M2_FWD = 33   # Motor 2 (Right) - Hardware PWM
        self.PIN_M2_REV = 15   # Motor 2 (Right) - Software PWM
        self.PWM_FREQ = 1000

        # --- Odometry & Encoder Configuration ---
        self.PIN_ENC_L = 11      # Left Channel A
        self.PIN_ENC_L_B = 12    # Left Channel B (Change to your actual pin)
        self.PIN_ENC_R = 16      # Right Channel A
        self.PIN_ENC_R_B = 18    # Right Channel B (Change to your actual pin)
        
        self.declare_parameter('wheel_radius', 0.0835)      
        self.declare_parameter('wheelbase', 0.40)           
        self.declare_parameter('ticks_per_rev', 1440.0)      
        self.declare_parameter('odom_frame', 'custom_odom')        
        self.declare_parameter('base_frame', 'custom_base_link')   

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.N = self.get_parameter('ticks_per_rev').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.left_motor_dir = 1
        self.right_motor_dir = 1
        
        # --- Added Debounce Variables ---
        # 500,000 nanoseconds = 0.5 ms. This allows up to 2000 ticks/sec.
        # This safely filters noise while easily clearing your ~1050 ticks/sec peak.
        self.debounce_ns = 100_000 
        self.last_left_tick_time = time.perf_counter_ns()
        self.last_right_tick_time = time.perf_counter_ns()
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.first_reading = True
        self.last_time = self.get_clock().now()

        # Publishers & Broadcasters
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        #self.tf_broadcaster = TransformBroadcaster(self)
        # ---------------------------------------------

        self.get_logger().info("Initializing GPIO and PWM...")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        # Setup Motor 1 (Left)
        self.pwm_m1_rev = SoftwarePWM(self.PIN_M1_REV, self.PWM_FREQ)
        self.pwm_m1_rev.start(0)
        GPIO.setup(self.PIN_M1_FWD, GPIO.OUT)
        self.pwm_m1_fwd = GPIO.PWM(self.PIN_M1_FWD, self.PWM_FREQ)
        self.pwm_m1_fwd.start(0)

        # Setup Motor 2 (Right)
        self.pwm_m2_rev = SoftwarePWM(self.PIN_M2_REV, self.PWM_FREQ)
        self.pwm_m2_rev.start(0)
        GPIO.setup(self.PIN_M2_FWD, GPIO.OUT)
        self.pwm_m2_fwd = GPIO.PWM(self.PIN_M2_FWD, self.PWM_FREQ)
        self.pwm_m2_fwd.start(0)

        # --- Setup Encoders (Channel A & B - Active Low Logic) ---
        GPIO.setup(self.PIN_ENC_L, GPIO.IN)
        GPIO.setup(self.PIN_ENC_L_B, GPIO.IN)
        
        GPIO.setup(self.PIN_ENC_R, GPIO.IN)
        GPIO.setup(self.PIN_ENC_R_B, GPIO.IN)
        
        # Using FALLING because the dummy block holds it HIGH (3.3V) and a tick pulls it LOW (0V)
        GPIO.add_event_detect(self.PIN_ENC_L, GPIO.RISING, callback=self.left_tick_cb)
        GPIO.add_event_detect(self.PIN_ENC_R, GPIO.RISING, callback=self.right_tick_cb)
        # ---------------------------------------------------

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # --- Timers ---
        self.timer = self.create_timer(0.05, self.update_odometry) # 20Hz Odometry
        self.debug_timer = self.create_timer(1.0, self.print_debug_info) # 1Hz Tick Logger

        self.get_logger().info("AMR Motor Node Ready. Listening to /cmd_vel and Encoders...")

    # --- Debug Logger ---
    def print_debug_info(self):
        """Prints the raw hardware tick counts once per second."""
        self.get_logger().info(f"TICKS | Left: {self.left_ticks} | Right: {self.right_ticks}")
        
    # --- TRUE Quadrature Encoder Callbacks (Filtered) ---
    # --- Command-Directed Encoder Callbacks (Filtered) ---
    def left_tick_cb(self, channel):
        current_time = time.perf_counter_ns()
        if (current_time - self.last_left_tick_time) > self.debounce_ns:
            # Add or subtract based on commanded motor direction
            self.left_ticks += self.left_motor_dir
            self.last_left_tick_time = current_time

    def right_tick_cb(self, channel):
        current_time = time.perf_counter_ns()
        if (current_time - self.last_right_tick_time) > self.debounce_ns:
            # Add or subtract based on commanded motor direction
            self.right_ticks += self.right_motor_dir
            self.last_right_tick_time = current_time

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        # Differential Drive Kinematics
        linear = msg.linear.x   # Forward/Back
        angular = msg.angular.z # Left/Right rotation

        # Mix linear and angular to get wheel speeds
        left_speed = linear - angular
        right_speed = linear + angular

        # Normalize speeds to ensure we don't exceed 100% PWM while keeping the turning ratio
        max_speed = max(abs(left_speed), abs(right_speed), 1.0)
        left_pwm = (left_speed / max_speed) * 100.0
        right_pwm = (right_speed / max_speed) * 100.0

        # Apply to Motor 1 (Left)
        if left_pwm > 0:
            self.left_motor_dir = 1
            self.pwm_m1_rev.ChangeDutyCycle(0)
            self.pwm_m1_fwd.ChangeDutyCycle(left_pwm)
        elif left_pwm < 0:
            self.left_motor_dir = -1
            self.pwm_m1_fwd.ChangeDutyCycle(0)
            self.pwm_m1_rev.ChangeDutyCycle(abs(left_pwm))
        else:
            self.left_motor_dir = 1
            self.pwm_m1_fwd.ChangeDutyCycle(0)
            self.pwm_m1_rev.ChangeDutyCycle(0)

        # Apply to Motor 2 (Right)
        if right_pwm > 0:
            self.right_motor_dir = 1
            self.pwm_m2_rev.ChangeDutyCycle(0)
            self.pwm_m2_fwd.ChangeDutyCycle(right_pwm)
        elif right_pwm < 0:
            self.right_motor_dir = -1
            self.pwm_m2_fwd.ChangeDutyCycle(0)
            self.pwm_m2_rev.ChangeDutyCycle(abs(right_pwm))
        else:
            self.right_motor_dir = 1
            self.pwm_m2_fwd.ChangeDutyCycle(0)
            self.pwm_m2_rev.ChangeDutyCycle(0)

    # --- Odometry Calculation ---
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        if self.first_reading:
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            self.first_reading = False
            self.last_time = current_time
            return

        delta_left = self.left_ticks - self.prev_left_ticks
        delta_right = self.right_ticks - self.prev_right_ticks
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        dist_left = 2 * math.pi * self.R * (delta_left / self.N)
        dist_right = 2 * math.pi * self.R * (delta_right / self.N)

        delta_s = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.L

        self.x += delta_s * math.cos(self.theta + (delta_theta / 2.0))
        self.y += delta_s * math.sin(self.theta + (delta_theta / 2.0))
        self.theta += delta_theta

        self.last_time = current_time

        #t = TransformStamped()
        #t.header.stamp = current_time.to_msg()
        #t.header.frame_id = self.odom_frame
        #t.child_frame_id = self.base_frame
        #t.transform.translation.x = self.x
        #t.transform.translation.y = self.y
        #t.transform.rotation.z = q_z
        #t.transform.rotation.w = q_w
        #self.tf_broadcaster.sendTransform(t)

        # --- Add these two lines back in ---
        q_z = math.sin(self.theta / 2.0)
        q_w = math.cos(self.theta / 2.0)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = delta_s / dt
        odom.twist.twist.angular.z = delta_theta / dt
        self.odom_pub.publish(odom)

    def stop_motors(self):
        self.pwm_m1_fwd.ChangeDutyCycle(0)
        self.pwm_m1_rev.ChangeDutyCycle(0)
        self.pwm_m2_fwd.ChangeDutyCycle(0)
        self.pwm_m2_rev.ChangeDutyCycle(0)

    def cleanup(self):
        self.get_logger().info("Shutting down motors and cleaning up GPIO...")
        self.stop_motors()
        self.pwm_m1_rev.stop()
        self.pwm_m1_fwd.stop()
        self.pwm_m2_rev.stop()
        self.pwm_m2_fwd.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = AmrMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt. Shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()