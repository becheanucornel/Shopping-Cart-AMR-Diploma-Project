from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    # Define the 4 LiDAR nodes
    lidar_fl = Node(
        package='sllidar_ros2', executable='sllidar_node', name='lidar_fl',
        parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'lidar_fl_link', 'serial_baudrate': 460800, 'angle_compensate': True}],
        remappings=[('/scan', '/lidar_front_left/scan')]
    )

    lidar_fr = Node(
        package='sllidar_ros2', executable='sllidar_node', name='lidar_fr',
        parameters=[{'serial_port': '/dev/ttyUSB1', 'frame_id': 'lidar_fr_link', 'serial_baudrate': 460800, 'angle_compensate': True}],
        remappings=[('/scan', '/lidar_front_right/scan')]
    )

    lidar_bl = Node(
        package='sllidar_ros2', executable='sllidar_node', name='lidar_bl',
        parameters=[{'serial_port': '/dev/ttyUSB2', 'frame_id': 'lidar_bl_link', 'serial_baudrate': 460800, 'angle_compensate': True}],
        remappings=[('/scan', '/lidar_back_left/scan')]
    )

    lidar_br = Node(
        package='sllidar_ros2', executable='sllidar_node', name='lidar_br',
        parameters=[{'serial_port': '/dev/ttyUSB3', 'frame_id': 'lidar_br_link', 'serial_baudrate': 460800, 'angle_compensate': True}],
        remappings=[('/scan', '/lidar_back_right/scan')]
    )

    scan_merger = Node(
        package='ros2_laser_scan_merger',
        executable='ros2_laser_scan_merger',
        name='scan_merger',
        output='screen',
        parameters=[{
            'laserscan_topics': ['/lidar_front_left/scan', '/lidar_front_right/scan', '/lidar_back_left/scan', '/lidar_back_right/scan'],
            'destination_topic': '/scan',
            'frame_id': 'custom_base_link',
        }]
    )

    return LaunchDescription([
        # Start LiDARs one by one to avoid USB bus power spikes
        TimerAction(period=0.0, actions=[lidar_fl]),
        TimerAction(period=1.0, actions=[lidar_fr]),
        TimerAction(period=2.0, actions=[lidar_bl]),
        TimerAction(period=3.0, actions=[lidar_br]),
        
        # Start merger after sensors are initialized
        TimerAction(period=5.0, actions=[scan_merger])
    ])