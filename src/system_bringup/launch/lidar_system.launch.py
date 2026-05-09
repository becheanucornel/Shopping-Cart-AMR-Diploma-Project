import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Opțional: Dacă ai deja un fișier de configurare salvat în pachetul tău.
    # Dacă nu, poți comenta linia "arguments" de la nodul RViz de mai jos,
    # îl deschizi gol, îl configurezi manual și apoi îl salvezi.
    rviz_config_path = os.path.join(
         get_package_share_directory('system_bringup'),
         'rviz',
         'lidar_visualizer.rviz'
     )

    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. NODURILE SLLIDAR (Citirea senzorilor)
        # ---------------------------------------------------------
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_front_left',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 460800, 'frame_id': 'lidar_fr_link', 'angle_compensate': True}],
            remappings=[('/scan', '/lidar_front_right/scan')]
        ),
        
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_front_right',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB1', 'serial_baudrate': 460800, 'frame_id': 'lidar_fl_link', 'angle_compensate': True}],
            remappings=[('/scan', '/lidar_front_left/scan')]
        ),
        
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_back_right', # Corectat numele
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB2', 'serial_baudrate': 460800, 'frame_id': 'lidar_br_link', 'angle_compensate': True}], # Corectat frame_id
            remappings=[('/scan', '/lidar_back_right/scan')]
        ),
        
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_back_left', # Corectat numele
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB3', 'serial_baudrate': 460800, 'frame_id': 'lidar_bl_link', 'angle_compensate': True}], # Corectat frame_id
            remappings=[('/scan', '/lidar_back_left/scan')]
        ),

        # ---------------------------------------------------------
        # 2. TRANSFORMARI STATICE (TF2) - FOARTE IMPORTANT!
        # Parametrii sunt: x y z yaw pitch roll frame_id child_frame_id
        # Acestea definesc unde sunt montate LiDAR-urile fata de base_link.
        # TREBUIE SA LE MODIFICI CU MASURATORILE REALE ALE ROBOTULUI TAU (in metri si radiani).
        # ---------------------------------------------------------
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='tf_fl',
            arguments=['0.2', '0.2', '0.1', '0.0', '0.0', '0.0', 'base_link', 'lidar_fl_link']
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='tf_fr',
            arguments=['0.2', '-0.2', '0.1', '0.0', '0.0', '0.0', 'base_link', 'lidar_fr_link']
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='tf_br',
            arguments=['-0.2', '-0.2', '0.1', '3.14159', '0.0', '0.0', 'base_link', 'lidar_br_link']
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='tf_bl',
            arguments=['-0.2', '0.2', '0.1', '3.14159', '0.0', '0.0', 'base_link', 'lidar_bl_link']
        ),

        # ---------------------------------------------------------
        # 3. NODUL C++ PENTRU MERGE (Scriptul tau)
        # ---------------------------------------------------------
        Node(
            package='scan_merger_module',
            executable='scan_merger_node',
            name='scan_merger',
            output='screen',
            parameters=[{
                'output_topic': '/scan',
                'output_frame_id': 'base_link',
                'merge_topics': [
                    '/lidar_front_left/scan',
                    '/lidar_front_right/scan',
                    '/lidar_back_left/scan',
                    '/lidar_back_right/scan'
                ]
            }]
            # Nu e nevoie de remapping deoarece am dat lista de topicuri direct ca parametru
        ),

        # ---------------------------------------------------------
        # 4. RVIZ2 PENTRU VIZUALIZARE
        # ---------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
             arguments=['-d', rviz_config_path] 
        )
    ])