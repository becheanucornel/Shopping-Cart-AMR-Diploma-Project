#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.pwm_pin_fwd = 32
        self.pwm_pin_rev = 33
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pwm_pin_fwd, GPIO.OUT)
        GPIO.setup(self.pwm_pin_rev, GPIO.OUT)
        
        self.pwm_fwd = GPIO.PWM(self.pwm_pin_fwd, 1000)
        self.pwm_rev = GPIO.PWM(self.pwm_pin_rev, 1000)
        
        self.pwm_fwd.start(0)
        self.pwm_rev.start(0)
        
        self.wheel_base = 0.30  
        self.max_speed = 1.5    
        
        # --- MODIFICAREA DE SIGURANTA ---
        # Aici setam limita maxima de putere. 30.0 inseamna 30% din puterea motorului.
        # Daca motorul abia se misca sau baraie dar nu invarte, poti creste la 40.0 sau 50.0.
        self.max_allowed_pwm = 30.0 
        
        self.get_logger().info(f'Nodul de testare a pornit. LIMITA DE SIGURANTA PWM: {self.max_allowed_pwm}%')

    def cmd_vel_callback(self, msg):
        v_linear = msg.linear.x
        v_angular = msg.angular.z
        
        wheel_speed = v_linear + (v_angular * self.wheel_base / 2.0)
        
        # Calculam procentul teoretic necesar (0-100)
        duty_cycle = (abs(wheel_speed) / self.max_speed) * 100.0
        
        # --- APLICAREA LIMITEI DE SIGURANTA ---
        # Daca duty_cycle-ul teoretic depaseste limita noastra (ex: 30%), il fortam sa ramana la 30%.
        duty_cycle = max(0.0, min(self.max_allowed_pwm, duty_cycle))
        
        if wheel_speed > 0.01:
            self.pwm_rev.ChangeDutyCycle(0)
            self.pwm_fwd.ChangeDutyCycle(duty_cycle)
            self.get_logger().info(f'Inainte: PWM={duty_cycle:.2f}% (Limitat la {self.max_allowed_pwm}%)')
            
        elif wheel_speed < -0.01:
            self.pwm_fwd.ChangeDutyCycle(0)
            self.pwm_rev.ChangeDutyCycle(duty_cycle)
            self.get_logger().info(f'Inapoi: PWM={duty_cycle:.2f}% (Limitat la {self.max_allowed_pwm}%)')
            
        else:
            self.pwm_fwd.ChangeDutyCycle(0)
            self.pwm_rev.ChangeDutyCycle(0)
            self.get_logger().info('Stop')

    def destroy_node(self):
        self.pwm_fwd.stop()
        self.pwm_rev.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nod oprit de la tastatura.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()