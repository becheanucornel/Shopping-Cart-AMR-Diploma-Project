import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Adafruit CircuitPython imports
import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')
        
        # Declare parameters for the voltage divider resistors (in Ohms)
        self.declare_parameter('r1', 100000.0)
        self.declare_parameter('r2', 10000.0)
        
        self.r1 = self.get_parameter('r1').value
        self.r2 = self.get_parameter('r2').value
        self.divider_ratio = (self.r1 + self.r2) / self.r2
        
        self.publisher_ = self.create_publisher(Float32, 'battery_voltage', 10)
        
        try:
            # Initialize the I2C bus using Jetson's default SCL/SDA
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize the ADC
            self.ads = ADS1115(self.i2c)
            
            # Gain of 1 sets the read range to +/- 4.096V
            self.ads.gain = 1 
            
            # Setup single-ended reading on A0. 
            # Using '0' directly bypasses any missing library constants.
            self.chan = AnalogIn(self.ads, 0)
            
            self.get_logger().info('Python Battery Monitor Started. Listening on A0...')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C or ADC: {e}')
            return

        # Timer to read and publish the voltage at 10Hz
        self.timer = self.create_timer(0.1, self.publish_voltage)

    def publish_voltage(self):
        try:
            # Read the raw voltage from the ADC
            adc_voltage = self.chan.voltage
            
            # Apply the voltage divider math to get the battery value
            battery_voltage = adc_voltage * self.divider_ratio
            
            # Force the terminal to print the math it is actually using!
            self.get_logger().info(f'Raw A0: {adc_voltage:.3f}V | Multiplier: {self.divider_ratio:.1f} | Output: {battery_voltage:.2f}V')
            
            # Create and publish the Float32 message
            msg = Float32()
            msg.data = float(battery_voltage)
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().warning(f'Failed to read ADC: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    
    # Only spin if the ADC initialized successfully
    if hasattr(node, 'ads'):
        rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()