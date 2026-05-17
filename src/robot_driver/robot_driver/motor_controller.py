#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO

class MirrorDualMotorController(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # --- PARAMETRI ---
        self.declare_parameter('max_linear_speed', 1.5)    
        self.declare_parameter('robot_wheel_base', 0.45)   
        self.speed_limit = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('robot_wheel_base').get_parameter_value().double_value
        
        self.get_logger().info(f"Sistem Dual Motor - Șasiu în Oglindă. Limită: {self.speed_limit:.2f} m/s")
        
        # --- CONFIGURARE PINI HARDWARE ---
        self.PIN_L_SPEED = 32  
        self.PIN_L_DIR   = 33  
        
        self.PIN_R_SPEED = 15  
        self.PIN_R_DIR   = 7   
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        
        GPIO.setup(self.PIN_L_SPEED, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.PIN_L_DIR,   GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.PIN_R_SPEED, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.PIN_R_DIR,   GPIO.OUT, initial=GPIO.LOW)
        
        # Stări interne
        self.duty_L = 0.0
        self.duty_R = 0.0
        self.dir_L = "STOP" 
        self.dir_R = "STOP"
        self.pulse_counter = 0
        
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.005, self.timer_hardware_callback)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Prioritizează mișcarea liniară pentru stabilitate dacă se comandă și rotație simultan
        if abs(linear_x) > 0.01 and abs(angular_z) > 0.01:
            angular_z = angular_z * 0.5
        
        # Cinematică diferențială standard (v_L și v_R matematice)
        req_v_L = linear_x - (angular_z * self.wheel_base / 2.0)
        req_v_R = linear_x - (angular_z * self.wheel_base / 2.0)
        
        # Plafonare automată (Saturare la limita de 1.5 m/s)
        v_L = max(min(req_v_L, self.speed_limit), -self.speed_limit)
        v_R = max(min(req_v_R, self.speed_limit), -self.speed_limit)
        
        # Calcul procente Duty Cycle
        self.duty_L = (abs(v_L) / self.speed_limit) * 100.0 if self.speed_limit > 0 else 0.0
        self.duty_R = (abs(v_R) / self.speed_limit) * 100.0 if self.speed_limit > 0 else 0.0
        
        # Deadband logic
        if self.duty_L < 5.0: self.duty_L = 0.0
        if self.duty_R < 5.0: self.duty_R = 0.0

        # --- LOGICĂ DIRECȚIE MOTOR STÂNGA (Standard) ---
        if v_L > 0.01:      self.dir_L = "FORWARD"
        elif v_L < -0.01:    self.dir_L = "REVERSE"
        else:                self.dir_L = "STOP"
            
        # --- LOGICĂ DIRECȚIE MOTOR DREAPTA (INVERSATĂ DIN CAUZA MONTAJULUI ÎN OGLINDĂ) ---
        if v_R > 0.01:      self.dir_R = "FORWARD"  # Când matematica cere înainte, fizic mergem invers
        elif v_R < -0.01:    self.dir_R = "REVERSE"  # Când matematica cere înapoi, fizic mergem în față
        else:                self.dir_R = "STOP"

    def timer_hardware_callback(self):
        self.pulse_counter = (self.pulse_counter + 1) % 20
        thresh_L = (self.duty_L / 100.0) * 20
        thresh_R = (self.duty_R / 100.0) * 20
        
        # --- CONTROL MOTOR STÂNGA (Logică Inversată pe Reverse) ---
        if self.dir_L == "FORWARD":
            GPIO.output(self.PIN_L_DIR, GPIO.LOW)
            GPIO.output(self.PIN_L_SPEED, GPIO.HIGH if self.pulse_counter < thresh_L else GPIO.LOW)
        elif self.dir_L == "REVERSE":
            GPIO.output(self.PIN_L_DIR, GPIO.HIGH)
            GPIO.output(self.PIN_L_SPEED, GPIO.LOW if self.pulse_counter < thresh_L else GPIO.HIGH)
        else:
            GPIO.output(self.PIN_L_SPEED, GPIO.LOW)
            GPIO.output(self.PIN_L_DIR, GPIO.LOW)
            
        # --- CONTROL MOTOR DREAPTA (Logică Simetrică pe Reverse) ---
        if self.dir_R == "FORWARD":
            GPIO.output(self.PIN_R_DIR, GPIO.LOW)
            GPIO.output(self.PIN_R_SPEED, GPIO.HIGH if self.pulse_counter < thresh_R else GPIO.LOW)
        elif self.dir_R == "REVERSE":
            GPIO.output(self.PIN_R_DIR, GPIO.HIGH)
            GPIO.output(self.PIN_R_SPEED, GPIO.HIGH if self.pulse_counter < thresh_R else GPIO.LOW)
        else:
            GPIO.output(self.PIN_R_SPEED, GPIO.LOW)
            GPIO.output(self.PIN_R_DIR, GPIO.LOW)

    def stop_all_motors(self):
        GPIO.output(self.PIN_L_SPEED, GPIO.LOW)
        GPIO.output(self.PIN_L_DIR, GPIO.LOW)
        GPIO.output(self.PIN_R_SPEED, GPIO.LOW)
        GPIO.output(self.PIN_R_DIR, GPIO.LOW)

    def destroy_node(self):
        self.stop_all_motors()
        try: GPIO.cleanup()
        except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MirrorDualMotorController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()