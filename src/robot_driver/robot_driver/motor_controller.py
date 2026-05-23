import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
import time
import threading

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

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("AMR Motor Node Ready. Listening to /cmd_vel...")

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
            self.pwm_m1_rev.ChangeDutyCycle(0)
            self.pwm_m1_fwd.ChangeDutyCycle(left_pwm)
        elif left_pwm < 0:
            self.pwm_m1_fwd.ChangeDutyCycle(0)
            self.pwm_m1_rev.ChangeDutyCycle(abs(left_pwm))
        else:
            self.pwm_m1_fwd.ChangeDutyCycle(0)
            self.pwm_m1_rev.ChangeDutyCycle(0)

        # Apply to Motor 2 (Right)
        if right_pwm > 0:
            self.pwm_m2_rev.ChangeDutyCycle(0)
            self.pwm_m2_fwd.ChangeDutyCycle(right_pwm)
        elif right_pwm < 0:
            self.pwm_m2_fwd.ChangeDutyCycle(0)
            self.pwm_m2_rev.ChangeDutyCycle(abs(right_pwm))
        else:
            self.pwm_m2_fwd.ChangeDutyCycle(0)
            self.pwm_m2_rev.ChangeDutyCycle(0)

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