import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

        self.declare_parameter('wheel_radius',       0.067)
        self.declare_parameter('wheelbase',          0.39)
        self.declare_parameter('ticks_per_rev',      1440.0)
        # Speed limits applied to incoming cmd_vel
        self.declare_parameter('max_linear_speed',   0.2)   # m/s
        self.declare_parameter('max_angular_speed',  0.6)   # rad/s
        # Acceleration/deceleration limits (units per second)
        self.declare_parameter('linear_accel_limit',  0.15)  # m/s²
        self.declare_parameter('linear_decel_limit',  1.5)   # m/s² — faster stop
        self.declare_parameter('angular_accel_limit', 0.5)   # rad/s²
        self.declare_parameter('angular_decel_limit', 3.0)   # rad/s²
        # How many m/s maps to 100% PWM (tune to match motor characteristics)
        self.declare_parameter('pwm_scale',          1.0)   # 1.0 m/s = 100% PWM
        # Seconds without a cmd_vel before ramping to stop
        self.declare_parameter('cmd_vel_timeout',    0.5)

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.N = self.get_parameter('ticks_per_rev').value
        self.max_v   = self.get_parameter('max_linear_speed').value
        self.max_w   = self.get_parameter('max_angular_speed').value
        self.acc_v   = self.get_parameter('linear_accel_limit').value
        self.dec_v   = self.get_parameter('linear_decel_limit').value
        self.acc_w   = self.get_parameter('angular_accel_limit').value
        self.dec_w   = self.get_parameter('angular_decel_limit').value
        self.pwm_scale = self.get_parameter('pwm_scale').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value

        # Velocity ramp state
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_cmd_time = self.get_clock().now()

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

        self._enc_diag_timer = self.create_timer(5.0, self._check_encoder_health)
        self._enc_l_count_last = 0
        self._enc_r_count_last = 0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.05, self.update_odometry)

    def _check_encoder_health(self):
        dl = abs(self.left_ticks - self._enc_l_count_last)
        dr = abs(self.right_ticks - self._enc_r_count_last)
        self._enc_l_count_last = self.left_ticks
        self._enc_r_count_last = self.right_ticks
        if dl > 10 and dr == 0:
            self.get_logger().error(
                f'RIGHT ENCODER DEAD! left_ticks={dl} right_ticks={dr} in last 5s. '
                f'Check wiring on BOARD pin {self.PIN_ENC_R}. Odometry is WRONG!')
        elif dr > 10 and dl == 0:
            self.get_logger().error(
                f'LEFT ENCODER DEAD! left_ticks={dl} right_ticks={dr} in last 5s. '
                f'Check wiring on BOARD pin {self.PIN_ENC_L}. Odometry is WRONG!')
        elif dl > 5 or dr > 5:
            self.get_logger().info(f'Encoder health OK: L={dl} R={dr} ticks/5s')

    def left_tick_cb(self, channel):
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
        # Clamp incoming velocity to configured limits
        self.target_v = max(-self.max_v, min(self.max_v, msg.linear.x))
        self.target_w = max(-self.max_w, min(self.max_w, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def _ramp(self, current, target, accel, decel, dt):
        delta = target - current
        # Deceleration: moving toward zero or reversing direction
        slowing = (current > 0 and target < current) or (current < 0 and target > current)
        limit = decel if slowing else accel
        max_step = limit * dt
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)

    def _apply_velocities(self, v, w):
        v_l = v - (w * self.L / 2.0)
        v_r = v + (w * self.L / 2.0)
        self.apply_pwm(self.pwm_m1_fwd, self.pwm_m1_rev, v_l * 100.0 / self.pwm_scale)
        self.apply_pwm(self.pwm_m2_fwd, self.pwm_m2_rev, v_r * 100.0 / self.pwm_scale)

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

        if dt <= 0.0:
            return

        # Stop if cmd_vel has gone stale
        elapsed_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if elapsed_since_cmd > self.cmd_timeout:
            self.target_v = 0.0
            self.target_w = 0.0

        # Ramp current velocities toward targets
        self.current_v = self._ramp(self.current_v, self.target_v, self.acc_v, self.dec_v, dt)
        self.current_w = self._ramp(self.current_w, self.target_w, self.acc_w, self.dec_w, dt)

        # Drive motors at ramped velocity
        self._apply_velocities(self.current_v, self.current_w)

        dl = self.left_ticks - self.prev_left_ticks
        dr = self.right_ticks - self.prev_right_ticks
        self.prev_left_ticks, self.prev_right_ticks = self.left_ticks, self.right_ticks

        d_left  = 2 * math.pi * self.R * (dl / self.N)
        d_right = 2 * math.pi * self.R * (dr / self.N)
        ds      = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.L

        v     = ds / dt
        omega = d_theta / dt

        if abs(dl) > 0 or abs(dr) > 0:
            self.get_logger().info(f'dt={dt:.4f} dl={dl} dr={dr} ds={ds:.4f} v={v:.4f} omega={omega:.4f}')

        self.theta += d_theta
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)
        self.last_time = current_time

        odom = Odometry()
        odom.header.stamp    = current_time.to_msg()
        odom.header.frame_id = 'custom_odom'
        odom.child_frame_id  = 'custom_base_footprint'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega

        odom.pose.covariance[0]  = 0.1
        odom.pose.covariance[7]  = 0.1
        odom.pose.covariance[14] = 1e9
        odom.pose.covariance[21] = 1e9
        odom.pose.covariance[28] = 1e9
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0]  = 0.05
        odom.twist.covariance[7]  = 1e9
        odom.twist.covariance[14] = 1e9
        odom.twist.covariance[21] = 1e9
        odom.twist.covariance[28] = 1e9
        odom.twist.covariance[35] = 0.2

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
