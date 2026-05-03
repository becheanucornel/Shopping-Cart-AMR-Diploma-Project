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
    
    # Launch Arguments
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='idle',
        description='Operating mode: navigation, slam, manual, view, description, ui_only'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true in simulation)'
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz visualization'
    )
    
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle-managed nodes (Nav2, map_server, AMCL)'
    )
    
    declare_nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=nav2_params_file,
        description='Full path to Nav2 parameters file'
    )
    
    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to SLAM Toolbox parameters file'
    )
    
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(system_bringup_dir, 'map', 'map.yaml'), 
        description='Full path to map YAML file'
    )

    declare_map_save_dir_arg = DeclareLaunchArgument(
        'map_save_dir',
        default_value=os.path.join(system_bringup_dir, 'map'),
        description='Directory where map_saver_server will save maps in SLAM mapping mode'
    )

    declare_map_save_name_arg = DeclareLaunchArgument(
        'map_save_name',
        default_value='saved_map',
        description='Base filename for saved map (no extension) in SLAM mapping mode'
    )
    
    declare_slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='localization',
        description='SLAM mode: localization or mapping'
    )

    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch joint state publisher GUI (only for description mode)'
    )
    
    declare_merge_scans_arg = DeclareLaunchArgument(
        'merge_scans',
        default_value='true',
        description='Enable scan merger for multiple lidars'
    )
    
    declare_use_startup_delays_arg = DeclareLaunchArgument(
        'use_startup_delays',
        default_value='true',
        description='Use staged startup with delays for better node initialization'
    )

    declare_publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='true',
        description='WebServer: publish odom->base_link TF from /odom (disable if your driver already publishes it)'
    )

    declare_nav_cruise_linear_scale_arg = DeclareLaunchArgument(
        'nav_cruise_linear_scale',
        default_value='1.0',
        description='ModeManager: always-on multiplier for Nav2 linear.x'
    )

    declare_nav_cruise_angular_scale_arg = DeclareLaunchArgument(
        'nav_cruise_angular_scale',
        default_value='1.0',
        description='ModeManager: always-on multiplier for Nav2 angular.z'
    )

    declare_nav_stuck_linear_scale_arg = DeclareLaunchArgument(
        'nav_stuck_linear_scale',
        default_value='8.0',
        description='ModeManager: extra multiplier for Nav2 linear.x when stuck'
    )

    declare_nav_stuck_angular_scale_arg = DeclareLaunchArgument(
        'nav_stuck_angular_scale',
        default_value='4.0',
        description='ModeManager: extra multiplier for Nav2 angular.z when stuck'
    )

    declare_nav_min_linear_x_arg = DeclareLaunchArgument(
        'nav_min_linear_x',
        default_value='0.4',
        description='ModeManager: minimum |linear.x| when stuck and command is nonzero'
    )

    declare_nav_min_angular_z_arg = DeclareLaunchArgument(
        'nav_min_angular_z',
        default_value='0.6',
        description='ModeManager: minimum |angular.z| when stuck and command is nonzero'
    )

    declare_nav_stuck_linear_x_arg = DeclareLaunchArgument(
        'nav_stuck_linear_x',
        default_value='0.03',
        description='ModeManager: consider robot stuck if |odom_nav2.twist.linear.x| below this'
    )

    declare_nav_stuck_angular_z_arg = DeclareLaunchArgument(
        'nav_stuck_angular_z',
        default_value='0.03',
        description='ModeManager: consider robot stuck if |odom_nav2.twist.angular.z| below this'
    )

    # NOU: Launch Arguments for YOLO Follow-Me Module
    declare_enable_follow_arg = DeclareLaunchArgument(
        'enable_follow',
        default_value='true',
        description='Launch YOLOv8 tracking and Follow-Me visual servoing nodes'
    )

    declare_camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640.0',
        description='Follow-Me: Width of the camera image in pixels'
    )

    declare_target_height_arg = DeclareLaunchArgument(
        'target_height',
        default_value='280.0',
        description='Follow-Me: Ideal height of bounding box (dictates following distance)'
    )
    
    # Core Robot Nodes (Common to most modes)

    launch_robot_description = PythonExpression([
        "'", LaunchConfiguration('mode'), "' != 'view' and '",
        LaunchConfiguration('mode'), "' != 'ui_only'"
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(launch_robot_description)
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' in ['description', 'manual']"
        ]))
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' == 'description' and '",
            LaunchConfiguration('gui'), "' == 'true'"
        ]))
    )
    
    # Mode manager
    mode_manager = Node(
        package='mode_manager_module',
        executable='mode_manager_node',
        name='mode_manager',
        output='screen',
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
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' not in ['view', 'description']"
        ]))
    )
    
    # Sensor Nodes
    scan_merger = Node(
        package='scan_merger_module',
        executable='scan_merger_node',
        name='advanced_scan_merger',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'output_topic': '/scan',
            'robot_base_frame': 'base_link',
        }],
        condition=IfCondition(LaunchConfiguration('merge_scans')),
    )
    
    web_server = Node(
        package='web_server',
        executable='web_server_node',
        name='web_server',
        parameters=[{
            'port': 8080,
            'auto_initialpose_from_odom': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'publish_odom_tf': ParameterValue(LaunchConfiguration('publish_odom_tf'), value_type=bool),
        }],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', robot_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    
    nav2_enabled = PythonExpression([
        "'", LaunchConfiguration('slam_mode'), "' == 'localization' and '",
        LaunchConfiguration('mode'), "' not in ['view', 'description', 'ui_only']"
    ])

    # Nav2 container
    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(nav2_enabled)
    )
    
    # Load Nav2 composable nodes
    load_nav2_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('/cmd_vel', '/cmd_vel_nav2')]
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('/cmd_vel', '/cmd_vel_nav2')]
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('/cmd_vel', '/cmd_vel_nav2')]
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('/cmd_vel', '/cmd_vel_nav2')]
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                    ]
                }]
            ),
        ],
        condition=IfCondition(nav2_enabled)
    )
    
    # Map Server Node - Publishes the map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': LaunchConfiguration('map_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'localization' and '",
            LaunchConfiguration('mode'), "' in ['navigation','idle','manual','slam']"
        ]))
    )
    
    # AMCL Node for localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'localization' and '",
            LaunchConfiguration('mode'), "' in ['navigation','idle','manual','slam']"
        ]))
    )
    
    # Lifecycle manager for map_server and AMCL
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                'node_names': ['map_server', 'amcl']
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'localization' and '",
            LaunchConfiguration('mode'), "' in ['navigation','idle','manual','slam']"
        ]))
    )

    # SLAM Toolbox - Async mapping mode
    slam_toolbox_mapping = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_start_at_dock': False,
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'mapping'"
        ]))
    )

    # Map saver server (lifecycle)
    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'save_map_timeout': 2000.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True,
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'mapping'"
        ]))
    )

    map_saver_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
            'node_names': ['map_saver'],
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'mapping'"
        ]))
    )

    # ROSBridge WebSocket server
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090, 'delay_between_messages': 0.0}]
    )
    

    # NOU: YOLOv8 Human Tracker Node
    yolo_tracker_node = Node(
        package='human_detection_module',  # Schimba cu numele real al pachetului tau daca difera
        executable='yolo_human_tracker',   # Schimba cu numele executabilului tau din setup.py
        name='yolo_human_tracker',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_follow'))
    )

    # NOU: Follow-Me Controller Node (Visual Servoing)
    follow_controller_node = Node(
        package='human_detection_module',  # Schimba cu numele real al pachetului tau daca difera
        executable='follow_me_controller', # Schimba cu numele executabilului tau din setup.py
        name='follow_me_controller',
        output='screen',
        parameters=[{
            'camera_width': ParameterValue(LaunchConfiguration('camera_width'), value_type=float),
            'target_height': ParameterValue(LaunchConfiguration('target_height'), value_type=float)
        }],
        condition=IfCondition(LaunchConfiguration('enable_follow'))
    )
    
    # Build Launch Description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_mode_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_autostart_arg)
    ld.add_action(declare_nav2_params_arg)
    ld.add_action(declare_slam_params_arg)
    ld.add_action(declare_map_file_arg)
    ld.add_action(declare_map_save_dir_arg)
    ld.add_action(declare_map_save_name_arg)
    ld.add_action(declare_slam_mode_arg)
    ld.add_action(declare_gui_arg)
    ld.add_action(declare_merge_scans_arg)
    ld.add_action(declare_use_startup_delays_arg)
    ld.add_action(declare_publish_odom_tf_arg)
    ld.add_action(declare_nav_cruise_linear_scale_arg)
    ld.add_action(declare_nav_cruise_angular_scale_arg)
    ld.add_action(declare_nav_stuck_linear_scale_arg)
    ld.add_action(declare_nav_stuck_angular_scale_arg)
    ld.add_action(declare_nav_min_linear_x_arg)
    ld.add_action(declare_nav_min_angular_z_arg)
    ld.add_action(declare_nav_stuck_linear_x_arg)
    ld.add_action(declare_nav_stuck_angular_z_arg)
    
    # Add new Follow-Me arguments
    ld.add_action(declare_enable_follow_arg)
    ld.add_action(declare_camera_width_arg)
    ld.add_action(declare_target_height_arg)
    
    staged_startup = IfCondition(LaunchConfiguration('use_startup_delays'))
    immediate_startup = UnlessCondition(LaunchConfiguration('use_startup_delays'))

    immediate_group = GroupAction(
        condition=immediate_startup,
        actions=[
            LogInfo(msg="[Startup] use_startup_delays:=false -> launching all enabled nodes immediately"),
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            scan_merger,
            mode_manager,
            map_server_node,
            amcl_node,
            map_server_lifecycle,
            slam_toolbox_mapping,
            map_saver_server,
            map_saver_lifecycle,
            nav2_container,
            load_nav2_nodes,
            web_server,
            rosbridge_websocket,
            yolo_tracker_node,
            follow_controller_node,
            rviz2,
        ],
    )

    staged_group = GroupAction(
        condition=staged_startup,
        actions=[
            LogInfo(msg="[Stage 1/6] Starting robot state publisher..."),
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            TimerAction(
                period=1.0,
                actions=[
                    LogInfo(msg="[Stage 2/6] Starting sensors (scan_merger)..."),
                    scan_merger,
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    LogInfo(msg="[Stage 3/6] Starting localization / mapping..."),
                    mode_manager,
                    map_server_node,
                    amcl_node,
                    map_server_lifecycle,
                    slam_toolbox_mapping,
                    map_saver_server,
                    map_saver_lifecycle,
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    LogInfo(msg="[Stage 4/6] Starting Nav2 container..."),
                    nav2_container,
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    LogInfo(msg="[Stage 5/6] Loading Nav2 composable nodes..."),
                    load_nav2_nodes,
                ],
            ),
            TimerAction(
                period=4.5,
                actions=[
                    LogInfo(msg="[Stage 6/6] Starting web server, rosbridge and Follow-Me modules..."),
                    web_server,
                    rosbridge_websocket,
                    yolo_tracker_node,
                    follow_controller_node,
                    rviz2,
                ],
            ),
        ],
    )

    ld.add_action(immediate_group)
    ld.add_action(staged_group)
    
    return ld