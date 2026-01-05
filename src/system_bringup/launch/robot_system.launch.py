import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction, 
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    LogInfo
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
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
        default_value='true',
        description='Use simulation time'
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz visualization'
    )
    
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
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
        default_value=os.path.join(system_bringup_dir, 'map', 'sim_map.yaml'),
        description='Full path to map YAML file for localization'
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' not in ['view', 'description']"
        ]))
    )
    
    # Sensor Nodes
    scan_merger = Node(
        package='scan_merger_module',
        executable='scan_merger_node',
        name='scan_merger',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    web_server = Node(
        package='web_server',
        executable='web_server_node',
        name='web_server',
        parameters=[{'port': 8080}],
        output='screen',
    )
    
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
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' in ['navigation','manual','idle','slam']"
        ]))
    )
    
    # Load Nav2 composable nodes
    load_nav2_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[nav2_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[nav2_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[nav2_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[nav2_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': True,
                    'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
                }]
            ),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' in ['navigation','manual','idle','slam']"
        ]))
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
            "'", LaunchConfiguration('slam_mode'), "' == 'localization'"
        ]))
    )
    
    # AMCL Node for localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                'set_initial_pose': True,
                'initial_pose.x': 0.0,
                'initial_pose.y': 0.0,
                'initial_pose.yaw': 0.0,
                'always_reset_initial_pose': True,
                'tf_broadcast': True,
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'localization'"
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
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'localization'"
        ]))
    )

    # SLAM Toolbox - Localization mode
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_name': LaunchConfiguration('map_file'),
                'map_start_at_dock': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
            }
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_mode'), "' == 'localization'"
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

    # ROSBridge WebSocket server
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090, 'delay_between_messages': 0.0}]
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
    ld.add_action(declare_slam_mode_arg)
    ld.add_action(declare_gui_arg)
    ld.add_action(declare_merge_scans_arg)
    ld.add_action(declare_use_startup_delays_arg)
    
    # Stage 1: Core robot description and state publisher
    ld.add_action(LogInfo(msg="[Stage 1/6] Starting robot state publisher..."))
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    
    # Stage 3: Sensors after motion control (1.5s delay)
    ld.add_action(TimerAction(
        period=1.5,
        actions=[
            LogInfo(msg="[Stage 3/6] Starting sensors (scan_merger)..."),
            scan_merger
        ]
    ))
    
    # Stage 4: Navigation and localization
    ld.add_action(TimerAction(
        period=2.5,
        actions=[
            LogInfo(msg="[Stage 4/6] Starting navigation, map server, and localization..."),
            nav2_container,
            mode_manager,
            map_server_node,
            amcl_node,
            map_server_lifecycle,
            slam_toolbox_mapping,
        ]
    ))
    
    # Stage 5: Nav2 components after container is ready (3.5s delay)
    ld.add_action(TimerAction(
        period=3.5,
        actions=[
            LogInfo(msg="[Stage 5/6] Loading Nav2 composable nodes..."),
            load_nav2_nodes
        ]
    ))
    
    # Stage 6: Web server and rosbridge
    ld.add_action(TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg="[Stage 6/6] Starting web server..."),
            web_server,
            rosbridge_websocket
        ]
    ))
    
    return ld
