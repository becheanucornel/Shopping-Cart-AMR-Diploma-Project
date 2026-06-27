import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import Jetson.GPIO as GPIO
import time
import threading
import math
import os

os.environ["JETSON_GPIO_PINMUX_CHECK"] = "0"

class SoftwarePWM:
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
        if self.thread: self.thread.join()
        GPIO.output(self.pin, GPIO.LOW)

    def _run_pwm(self):
        period = 1.0 / self.frequency
        while self.is_running:
            if self.duty_cycle <= 0.0:
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(period)
            elif self.duty_cycle >= 100.0:
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(period)
            else:
                time_high = period * (self.duty_cycle / 100.0)
                time_low = period - time_high
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(time_high)
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(time_low)

class AmrMotorNode(Node):
    def __init__(self):
        super().__init__('amr_motor_controller')

        self.PIN_M1_FWD, self.PIN_M1_REV = 32, 7
        self.PIN_M2_FWD, self.PIN_M2_REV = 33, 15
        self.PWM_FREQ = 1000

        self.PIN_ENC_L, self.PIN_ENC_L_B = 11, 12
        self.PIN_ENC_R, self.PIN_ENC_R_B = 16, 18
        
        self.declare_parameter('wheel_radius', 0.067)      
        self.declare_parameter('wheelbase', 0.39)          
        self.declare_parameter('ticks_per_rev', 1440.0)     
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.N = self.get_parameter('ticks_per_rev').value

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'custom_odom_topic', 10)
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        self.pwm_m1_rev = SoftwarePWM(self.PIN_M1_REV, self.PWM_FREQ)
        self.pwm_m1_rev.start(0)
        GPIO.setup(self.PIN_M1_FWD, GPIO.OUT)
        self.pwm_m1_fwd = GPIO.PWM(self.PIN_M1_FWD, self.PWM_FREQ)
        self.pwm_m1_fwd.start(0)

        self.pwm_m2_rev = SoftwarePWM(self.PIN_M2_REV, self.PWM_FREQ)
        self.pwm_m2_rev.start(0)
        GPIO.setup(self.PIN_M2_FWD, GPIO.OUT)
        self.pwm_m2_fwd = GPIO.PWM(self.PIN_M2_FWD, self.PWM_FREQ)
        self.pwm_m2_fwd.start(0)

        GPIO.setup([self.PIN_ENC_L, self.PIN_ENC_L_B, self.PIN_ENC_R, self.PIN_ENC_R_B], GPIO.IN)
        GPIO.add_event_detect(self.PIN_ENC_L, GPIO.BOTH, callback=self.left_tick_cb)
        GPIO.add_event_detect(self.PIN_ENC_R, GPIO.BOTH, callback=self.right_tick_cb)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.05, self.update_odometry)

    def left_tick_cb(self, channel):
        # Detecție direcție prin starea pinului B
        if GPIO.input(self.PIN_ENC_L) != GPIO.input(self.PIN_ENC_L_B):
            self.left_ticks += 1
        else:
            self.left_ticks -= 1

    def right_tick_cb(self, channel):
        if GPIO.input(self.PIN_ENC_R) == GPIO.input(self.PIN_ENC_R_B):
            self.right_ticks += 1
        else:
            self.right_ticks -= 1

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        
        v_l = v - (w * self.L / 2.0)
        v_r = v + (w * self.L / 2.0)
        
        # Mapping simplificat pentru demo
        self.apply_pwm(self.pwm_m1_fwd, self.pwm_m1_rev, v_l * 100)
        self.apply_pwm(self.pwm_m2_fwd, self.pwm_m2_rev, v_r * 100)

    def apply_pwm(self, fwd, rev, speed):
        pwm = max(-100, min(100, speed))
        if pwm > 0:
            rev.ChangeDutyCycle(0)
            fwd.ChangeDutyCycle(pwm)
        else:
            fwd.ChangeDutyCycle(0)
            rev.ChangeDutyCycle(abs(pwm))

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        dl = self.left_ticks - self.prev_left_ticks
        dr = self.right_ticks - self.prev_right_ticks
        self.prev_left_ticks, self.prev_right_ticks = self.left_ticks, self.right_ticks
        
        d_left = 2 * math.pi * self.R * (dl / self.N)
        d_right = 2 * math.pi * self.R * (dr / self.N)
        ds = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.L
        
        self.theta += d_theta
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)
        self.last_time = current_time

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'custom_odom'
        odom.child_frame_id = 'custom_base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom)

    def cleanup(self):
        self.pwm_m1_rev.stop(); self.pwm_m1_fwd.stop()
        self.pwm_m2_rev.stop(); self.pwm_m2_fwd.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = AmrMotorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__': main()