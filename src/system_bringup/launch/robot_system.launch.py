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
    
    # TODO Differential Drive Node
    
    
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
                parameters=[LaunchConfiguration('nav2_params_file')]
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[LaunchConfiguration('nav2_params_file')]
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[LaunchConfiguration('nav2_params_file')]
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'autostart': LaunchConfiguration('autostart'),
                    'node_names': ['controller_server', 'planner_server', 'bt_navigator']
                }]
            ),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' in ['navigation','manual','idle','slam']"
        ]))
    )
    
    # SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=['/home/apollo/licenta/src/robot_bringup/config/slam_toolbox.yaml'],
        output='screen'
    )
    
    # ROSBridge WebSocket server
    rosbridge_nodes = []
    if _has_pkg('rosbridge_server'):
        rosbridge_websocket = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090, 'delay_between_messages': 0.0}],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('web_ui'), "' == 'true' or '",
                LaunchConfiguration('mode'), "' == 'ui_only'"
            ]))
        )
        rosbridge_nodes.append(rosbridge_websocket)
    
    # Build Launch Description
    ld = LaunchDescription()
    # Add launch arguments
    ld.add_action(declare_mode_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_autostart_arg)
    ld.add_action(declare_nav2_params_arg)
    ld.add_action(declare_slam_params_arg)
    ld.add_action(declare_gui_arg)
    ld.add_action(declare_merge_scans_arg)
    ld.add_action(declare_use_startup_delays_arg)
    
    # Stage 1: Core robot description and state publisher
    ld.add_action(LogInfo(msg="[Stage 1/6] Starting robot state publisher..."))
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    
    # TODO Stage 2: Differential Drive Module
    
    # Stage 3: Sensors after motion control (1.5s delay)
    ld.add_action(TimerAction(
        period=1.5,
        actions=[
            LogInfo(msg="[Stage 3/6] Starting sensors (scan_merger)..."),
            scan_merger
        ]
    ))
    
    # Stage 4: Navigation stack and mode manager (2.5s delay)
    ld.add_action(TimerAction(
        period=2.5,
        actions=[
            LogInfo(msg="[Stage 4/6] Starting navigation container, mode manager, and SLAM..."),
            nav2_container,
            mode_manager,
            slam_toolbox_node,
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
    
    # TODO WEB UI
    ld.add_action(TimerAction(
        period=1.0,
        actions=[LogInfo(msg="[Stage 6/6] Starting web server..."),
        web_server
        ]
    ))
    
    return ld
