import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    GroupAction,
    TimerAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def _has_pkg(pkg_name: str) -> bool:
    """Check if a ROS2 package is installed."""
    try:
        get_package_prefix(pkg_name)
        return True
    except Exception:
        return False

def generate_launch_description():
    system_bringup_dir = get_package_share_directory('system_bringup')
 
    urdf_path = os.path.join(system_bringup_dir, 'urdf', 'robot_base.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    
    nav2_params_file = os.path.join(system_bringup_dir, 'config', 'nav2_config.yaml')
    slam_params_file = os.path.join(system_bringup_dir, 'config', 'slam_toolbox.yaml')
    robot_rviz_config = os.path.join(system_bringup_dir, 'rviz', 'shopping_cart_amr.rviz')
    
    # --- Launch Arguments (Kept exactly as provided) ---
    declare_mode_arg = DeclareLaunchArgument('mode', default_value='idle')
    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_rviz_arg = DeclareLaunchArgument('rviz', default_value='false')
    declare_autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    declare_nav2_params_arg = DeclareLaunchArgument('nav2_params_file', default_value=nav2_params_file)
    declare_slam_params_arg = DeclareLaunchArgument('slam_params_file', default_value=slam_params_file)
    declare_map_file_arg = DeclareLaunchArgument('map_file', default_value=os.path.join(system_bringup_dir, 'map', 'map.yaml'))
    declare_map_save_dir_arg = DeclareLaunchArgument('map_save_dir', default_value=os.path.join(system_bringup_dir, 'map'))
    declare_map_save_name_arg = DeclareLaunchArgument('map_save_name', default_value='saved_map')
    declare_slam_mode_arg = DeclareLaunchArgument('slam_mode', default_value='localization')
    declare_gui_arg = DeclareLaunchArgument('gui', default_value='false')
    declare_merge_scans_arg = DeclareLaunchArgument('merge_scans', default_value='true')
    declare_use_startup_delays_arg = DeclareLaunchArgument('use_startup_delays', default_value='true')
    declare_publish_odom_tf_arg = DeclareLaunchArgument('publish_odom_tf', default_value='true')
    declare_nav_cruise_linear_scale_arg = DeclareLaunchArgument('nav_cruise_linear_scale', default_value='1.0')
    declare_nav_cruise_angular_scale_arg = DeclareLaunchArgument('nav_cruise_angular_scale', default_value='1.0')
    declare_nav_stuck_linear_scale_arg = DeclareLaunchArgument('nav_stuck_linear_scale', default_value='8.0')
    declare_nav_stuck_angular_scale_arg = DeclareLaunchArgument('nav_stuck_angular_scale', default_value='4.0')
    declare_nav_min_linear_x_arg = DeclareLaunchArgument('nav_min_linear_x', default_value='0.1')
    declare_nav_min_angular_z_arg = DeclareLaunchArgument('nav_min_angular_z', default_value='0.3')
    declare_nav_stuck_linear_x_arg = DeclareLaunchArgument('nav_stuck_linear_x', default_value='0.05')
    declare_nav_stuck_angular_z_arg = DeclareLaunchArgument('nav_stuck_angular_z', default_value='0.05')
    declare_enable_follow_arg = DeclareLaunchArgument('enable_follow', default_value='true')
    declare_camera_width_arg = DeclareLaunchArgument('camera_width', default_value='640.0')
    declare_target_height_arg = DeclareLaunchArgument('target_height', default_value='280.0')
    
    # --- Core Nodes ---
    launch_robot_description = PythonExpression(["'", LaunchConfiguration('mode'), "' != 'view' and '", LaunchConfiguration('mode'), "' != 'ui_only'"])
    
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_description_content, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(launch_robot_description)
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher', output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' in ['description', 'manual']"]))
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui', name='joint_state_publisher_gui', output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'description' and '", LaunchConfiguration('gui'), "' == 'true'"]))
    )
    
    mode_manager = Node(
        package='mode_manager_module', executable='mode_manager_node', name='mode_manager', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'nav_cruise_linear_scale': ParameterValue(LaunchConfiguration('nav_cruise_linear_scale'), value_type=float),
            'nav_cruise_angular_scale': ParameterValue(LaunchConfiguration('nav_cruise_angular_scale'), value_type=float),
            'nav_linear_scale': ParameterValue(LaunchConfiguration('nav_stuck_linear_scale'), value_type=float),
            'nav_angular_scale': ParameterValue(LaunchConfiguration('nav_stuck_angular_scale'), value_type=float),
            'nav_min_linear_x': ParameterValue(LaunchConfiguration('nav_min_linear_x'), value_type=float),
            'nav_min_angular_z': ParameterValue(LaunchConfiguration('nav_min_angular_z'), value_type=float),
            'nav_stuck_linear_x': ParameterValue(LaunchConfiguration('nav_stuck_linear_x'), value_type=float),
            'nav_stuck_angular_z': ParameterValue(LaunchConfiguration('nav_stuck_angular_z'), value_type=float),
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' not in ['view', 'description']"]))
    )
    
    # --- FIXED: Sensor Nodes Assigned to Variables ---
    lidar_fl = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_front_left', output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 460800, 'frame_id': 'lidar_fr_link', 'angle_compensate': True}],
        remappings=[('/scan', '/lidar_front_right/scan')]
    )
    
    lidar_fr = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_front_right', output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB1', 'serial_baudrate': 460800, 'frame_id': 'lidar_fl_link', 'angle_compensate': True}],
        remappings=[('/scan', '/lidar_front_left/scan')]
    )
    
    lidar_br = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_back_right', output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB2', 'serial_baudrate': 460800, 'frame_id': 'lidar_br_link', 'angle_compensate': True}], 
        remappings=[('/scan', '/lidar_back_right/scan')]
    )
    
    lidar_bl = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_back_left', output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB3', 'serial_baudrate': 460800, 'frame_id': 'lidar_bl_link', 'angle_compensate': True}], 
        remappings=[('/scan', '/lidar_back_left/scan')]
    )

    # --- FIXED: Static Transforms enforcing custom_base_link ---
    tf_fl = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_fl',
        arguments=['0.2', '0.2', '0.1', '0.0', '0.0', '0.0', 'custom_base_link', 'lidar_fl_link'])
    tf_fr = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_fr',
        arguments=['0.2', '-0.2', '0.1', '0.0', '0.0', '0.0', 'custom_base_link', 'lidar_fr_link'])
    tf_br = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_br',
        arguments=['-0.2', '-0.2', '0.1', '3.14159', '0.0', '0.0', 'custom_base_link', 'lidar_br_link'])
    tf_bl = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_bl',
        arguments=['-0.2', '0.2', '0.1', '3.14159', '0.0', '0.0', 'custom_base_link', 'lidar_bl_link'])

    # --- FIXED: Scan Merger enforcing custom_base_link ---
    scan_merger = Node(
        package='scan_merger_module', executable='scan_merger_node', name='scan_merger', output='screen',
        parameters=[{
            'output_topic': '/scan',
            'output_frame_id': 'custom_base_link',
            'merge_topics': ['/lidar_front_left/scan', '/lidar_front_right/scan', '/lidar_back_left/scan', '/lidar_back_right/scan']
        }]
    )
        
    motor_controller = Node(
        package='robot_driver', executable='motor_controller_node', name='motor_controller', output='screen'
    )

    # --- NEW: LiDAR Odometry Node ---
    rf2o_node = Node(
        package='rf2o_laser_odometry', 
        executable='rf2o_laser_odometry_node', 
        name='rf2o_laser_odometry', 
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'], # <--- ADD THIS LINE
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False, 
            'base_frame_id': 'custom_base_link',
            'odom_frame_id': 'custom_odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }]
    )

    # --- NEW: Sensor Fusion (EKF) Node ---
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[os.path.join(system_bringup_dir, 'config', 'ekf.yaml')] 
    )

    # --- Web, Nav2, & Vision Nodes ---
    web_server = Node(
        package='web_server', executable='web_server_node', name='web_server', output='screen',
        parameters=[{
            'port': 8080,
            'auto_initialpose_from_odom': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'publish_odom_tf': ParameterValue(LaunchConfiguration('publish_odom_tf'), value_type=bool),
        }],
    )

    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', robot_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    
    nav2_enabled = PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'localization' and '", LaunchConfiguration('mode'), "' not in ['view', 'description', 'ui_only']"])

    nav2_container = Node(
        package='rclcpp_components', executable='component_container_isolated', name='nav2_container', output='screen',
        parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(nav2_enabled)
    )
    
    load_nav2_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(package='nav2_controller', plugin='nav2_controller::ControllerServer', name='controller_server', parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}], remappings=[('/cmd_vel', '/cmd_vel_nav2')]),
            ComposableNode(package='nav2_planner', plugin='nav2_planner::PlannerServer', name='planner_server', parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}], remappings=[('/cmd_vel', '/cmd_vel_nav2')]),
            ComposableNode(package='nav2_behaviors', plugin='behavior_server::BehaviorServer', name='behavior_server', parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}], remappings=[('/cmd_vel', '/cmd_vel_nav2')]),
            ComposableNode(package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator', name='bt_navigator', parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}], remappings=[('/cmd_vel', '/cmd_vel_nav2')]),
            ComposableNode(
                package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager', name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                    'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
                }]
            ),
        ],
        condition=IfCondition(nav2_enabled)
    )
    
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file'), 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'localization' and '", LaunchConfiguration('mode'), "' in ['navigation','idle','manual','slam']"]))
    )
    
    amcl_node = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'localization' and '", LaunchConfiguration('mode'), "' in ['navigation','idle','manual','slam']"]))
    )
    
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool), 'node_names': ['map_server', 'amcl']}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'localization' and '", LaunchConfiguration('mode'), "' in ['navigation','idle','manual','slam']"]))
    )

    slam_toolbox_mapping = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
        parameters=[LaunchConfiguration('slam_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time'), 'map_start_at_dock': False}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"]))
    )

    map_saver_server = Node(
        package='nav2_map_server', executable='map_saver_server', name='map_saver', output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'save_map_timeout': 2000.0, 'free_thresh_default': 0.25, 'occupied_thresh_default': 0.65, 'map_subscribe_transient_local': True}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"]))
    )

    map_saver_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map_saver', output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool), 'node_names': ['map_saver']}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"]))
    )

    rosbridge_websocket = Node(
        package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket', output='screen',
        parameters=[{'port': 9090, 'delay_between_messages': 0.0}]
    )
    
    yolo_tracker_node = Node(
        package='human_detection_module', executable='yolo_human_tracker', name='yolo_human_tracker', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_follow'))
    )

    follow_controller_node = Node(
        package='human_detection_module', executable='follow_me_controller', name='follow_me_controller', output='screen',
        parameters=[{'camera_width': ParameterValue(LaunchConfiguration('camera_width'), value_type=float), 'target_height': ParameterValue(LaunchConfiguration('target_height'), value_type=float)}],
        condition=IfCondition(LaunchConfiguration('enable_follow'))
    )
    
    # --- Launch Description Assembly ---
    ld = LaunchDescription()
    
    for arg in [declare_mode_arg, declare_use_sim_time_arg, declare_rviz_arg, declare_autostart_arg, declare_nav2_params_arg, declare_slam_params_arg, declare_map_file_arg, declare_map_save_dir_arg, declare_map_save_name_arg, declare_slam_mode_arg, declare_gui_arg, declare_merge_scans_arg, declare_use_startup_delays_arg, declare_publish_odom_tf_arg, declare_nav_cruise_linear_scale_arg, declare_nav_cruise_angular_scale_arg, declare_nav_stuck_linear_scale_arg, declare_nav_stuck_angular_scale_arg, declare_nav_min_linear_x_arg, declare_nav_min_angular_z_arg, declare_nav_stuck_linear_x_arg, declare_nav_stuck_angular_z_arg, declare_enable_follow_arg, declare_camera_width_arg, declare_target_height_arg]:
        ld.add_action(arg)
    
    staged_startup = IfCondition(LaunchConfiguration('use_startup_delays'))
    immediate_startup = UnlessCondition(LaunchConfiguration('use_startup_delays'))

    # All nodes to be launched
    all_nodes = [
        robot_state_publisher, joint_state_publisher, joint_state_publisher_gui,
        tf_fl, tf_fr, tf_br, tf_bl,  # Fixed Static TFs
        lidar_fl, lidar_fr, lidar_br, lidar_bl, # Fixed LiDARs
        scan_merger, motor_controller, rf2o_node, ekf_node, # Integration nodes
        mode_manager, map_server_node, amcl_node, map_server_lifecycle,
        slam_toolbox_mapping, map_saver_server, map_saver_lifecycle,
        nav2_container, load_nav2_nodes, web_server, rosbridge_websocket,
        yolo_tracker_node, follow_controller_node, rviz2
    ]

    immediate_group = GroupAction(
        condition=immediate_startup,
        actions=[LogInfo(msg="[Startup] Launching immediately")] + all_nodes
    )

    staged_group = GroupAction(
        condition=staged_startup,
        actions=[
            LogInfo(msg="[Stage 1/6] Starting robot state publisher & static TFs..."),
            robot_state_publisher, joint_state_publisher, joint_state_publisher_gui,
            tf_fl, tf_fr, tf_br, tf_bl,
            TimerAction(
                period=1.0,
                actions=[
                    LogInfo(msg="[Stage 2/6] Starting Sensors & Motor Control..."),
                    lidar_fl, lidar_fr, lidar_br, lidar_bl,
                    scan_merger, motor_controller
                ]
            ),
            TimerAction(
                period=2.0,
                actions=[
                    LogInfo(msg="[Stage 2.5/6] Starting Odometry & Sensor Fusion..."),
                    rf2o_node, ekf_node
                ]
            ),
            TimerAction(
                period=3.0,
                actions=[
                    LogInfo(msg="[Stage 3/6] Starting localization / mapping..."),
                    mode_manager, map_server_node, amcl_node, map_server_lifecycle,
                    slam_toolbox_mapping, map_saver_server, map_saver_lifecycle,
                ]
            ),
            TimerAction(
                period=4.0,
                actions=[
                    LogInfo(msg="[Stage 4/6] Starting Nav2 container..."),
                    nav2_container,
                ]
            ),
            TimerAction(
                period=5.0,
                actions=[
                    LogInfo(msg="[Stage 5/6] Loading Nav2 composable nodes..."),
                    load_nav2_nodes,
                ]
            ),
            TimerAction(
                period=5.5,
                actions=[
                    LogInfo(msg="[Stage 6/6] Starting Web, ROSBridge & Follow-Me modules..."),
                    web_server, rosbridge_websocket, yolo_tracker_node, follow_controller_node, rviz2,
                ]
            ),
        ]
    )

    ld.add_action(immediate_group)
    ld.add_action(staged_group)
    
    return ld