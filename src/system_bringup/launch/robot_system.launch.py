import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def _has_pkg(pkg_name: str) -> bool:
    try:
        get_package_prefix(pkg_name)
        return True
    except Exception:
        return False


def generate_launch_description():
    system_bringup_dir = get_package_share_directory('system_bringup')

    urdf_path = os.path.join(system_bringup_dir, 'urdf', 'robot_base.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    nav2_params_file  = os.path.join(system_bringup_dir, 'config', 'nav2_config.yaml')
    slam_params_file  = os.path.join(system_bringup_dir, 'config', 'slam_toolbox.yaml')
    robot_rviz_config = os.path.join(system_bringup_dir, 'rviz',   'shopping_cart_amr.rviz')

    # ── Launch arguments ────────────────────────────────────────────────────
    declare_mode_arg                    = DeclareLaunchArgument('mode',                    default_value='idle')
    declare_use_sim_time_arg            = DeclareLaunchArgument('use_sim_time',            default_value='false')
    declare_rviz_arg                    = DeclareLaunchArgument('rviz',                    default_value='false')
    declare_autostart_arg               = DeclareLaunchArgument('autostart',               default_value='true')
    declare_nav2_params_arg             = DeclareLaunchArgument('nav2_params_file',        default_value=nav2_params_file)
    declare_slam_params_arg             = DeclareLaunchArgument('slam_params_file',        default_value=slam_params_file)
    declare_map_file_arg                = DeclareLaunchArgument('map_file',                default_value=os.path.join(system_bringup_dir, 'map', 'map.yaml'))
    declare_map_save_dir_arg            = DeclareLaunchArgument('map_save_dir',            default_value=os.path.join(system_bringup_dir, 'map'))
    declare_map_save_name_arg           = DeclareLaunchArgument('map_save_name',           default_value='saved_map')
    declare_slam_mode_arg               = DeclareLaunchArgument('slam_mode',               default_value='localization')
    declare_gui_arg                     = DeclareLaunchArgument('gui',                     default_value='false')
    declare_merge_scans_arg             = DeclareLaunchArgument('merge_scans',             default_value='true')
    declare_use_startup_delays_arg      = DeclareLaunchArgument('use_startup_delays',      default_value='true')
    declare_publish_odom_tf_arg         = DeclareLaunchArgument('publish_odom_tf',         default_value='true')
    declare_nav_cruise_linear_scale_arg = DeclareLaunchArgument('nav_cruise_linear_scale', default_value='1.0')
    declare_nav_cruise_angular_scale_arg= DeclareLaunchArgument('nav_cruise_angular_scale',default_value='1.0')
    declare_nav_stuck_linear_scale_arg  = DeclareLaunchArgument('nav_stuck_linear_scale',  default_value='8.0')
    declare_nav_stuck_angular_scale_arg = DeclareLaunchArgument('nav_stuck_angular_scale', default_value='4.0')
    declare_nav_min_linear_x_arg        = DeclareLaunchArgument('nav_min_linear_x',        default_value='0.1')
    declare_nav_min_angular_z_arg       = DeclareLaunchArgument('nav_min_angular_z',       default_value='0.3')
    declare_nav_stuck_linear_x_arg      = DeclareLaunchArgument('nav_stuck_linear_x',      default_value='0.05')
    declare_nav_stuck_angular_z_arg     = DeclareLaunchArgument('nav_stuck_angular_z',     default_value='0.05')
    declare_enable_follow_arg           = DeclareLaunchArgument('enable_follow',           default_value='true')
    # FIX 1: camera_width / target_height were declared but never used by any
    # node — the detection node now reads focal_length_px and real heights as
    # ROS parameters directly. Keeping the args for backwards-compat but they
    # are no longer forwarded anywhere.
    declare_camera_width_arg            = DeclareLaunchArgument('camera_width',            default_value='640.0')
    declare_target_height_arg           = DeclareLaunchArgument('target_height',           default_value='280.0')

    # ── Conditions ──────────────────────────────────────────────────────────
    not_view_or_ui = PythonExpression([
        "'", LaunchConfiguration('mode'), "' != 'view' and '",
        LaunchConfiguration('mode'), "' != 'ui_only'"
    ])
    localization_mode = PythonExpression([
        "'", LaunchConfiguration('slam_mode'), "' == 'localization' and '",
        LaunchConfiguration('mode'), "' not in ['view', 'description', 'ui_only']"
    ])
    localization_with_map = PythonExpression([
        "'", LaunchConfiguration('slam_mode'), "' == 'localization' and '",
        LaunchConfiguration('mode'), "' in ['navigation','idle','manual','slam']"
    ])
    mapping_mode = PythonExpression([
        "'", LaunchConfiguration('slam_mode'), "' == 'mapping'"
    ])

    # ── Robot description ───────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(not_view_or_ui)
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' in ['description', 'manual']"
        ]))
    )

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

    # ── Mode Manager ────────────────────────────────────────────────────────
    mode_manager = Node(
        package='mode_manager_module',
        executable='mode_manager_node',
        name='mode_manager',
        output='screen',
        parameters=[{
            'use_sim_time':            LaunchConfiguration('use_sim_time'),
            'nav_cruise_linear_scale': ParameterValue(LaunchConfiguration('nav_cruise_linear_scale'),  value_type=float),
            'nav_cruise_angular_scale':ParameterValue(LaunchConfiguration('nav_cruise_angular_scale'), value_type=float),
            'nav_linear_scale':        ParameterValue(LaunchConfiguration('nav_stuck_linear_scale'),   value_type=float),
            'nav_angular_scale':       ParameterValue(LaunchConfiguration('nav_stuck_angular_scale'),  value_type=float),
            'nav_min_linear_x':        ParameterValue(LaunchConfiguration('nav_min_linear_x'),         value_type=float),
            'nav_min_angular_z':       ParameterValue(LaunchConfiguration('nav_min_angular_z'),        value_type=float),
            'nav_stuck_linear_x':      ParameterValue(LaunchConfiguration('nav_stuck_linear_x'),       value_type=float),
            'nav_stuck_angular_z':     ParameterValue(LaunchConfiguration('nav_stuck_angular_z'),      value_type=float),
            'map_save_path':           os.path.join(system_bringup_dir, 'map', 'map'),
            'map_yaml_path':           os.path.join(system_bringup_dir, 'map', 'map.yaml'),
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mode'), "' not in ['view', 'description']"
        ]))
    )

    # ── LiDARs ──────────────────────────────────────────────────────────────
    lidar_fl = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_front_left', output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 460800,
                     'frame_id': 'lidar_front_left', 'angle_compensate': True, 'inverted': True}],
        remappings=[('/scan', '/lidar_front_left/scan')]
    )
    lidar_fr = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_front_right', output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB1', 'serial_baudrate': 460800,
                     'frame_id': 'lidar_front_right', 'angle_compensate': True, 'inverted': True}],
        remappings=[('/scan', '/lidar_front_right/scan')]
    )
    lidar_br = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_back_right', output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB2', 'serial_baudrate': 460800,
                     'frame_id': 'lidar_back_right', 'angle_compensate': True, 'inverted': False}],
        remappings=[('/scan', '/lidar_back_right/scan')]
    )
    lidar_bl = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_back_left', output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB3', 'serial_baudrate': 460800,
                     'frame_id': 'lidar_back_left', 'angle_compensate': True, 'inverted': False}],
        remappings=[('/scan', '/lidar_back_left/scan')]
    )

    # ── Scan merger ─────────────────────────────────────────────────────────
    scan_merger = Node(
        package='scan_merger_module',
        executable='scan_merger_node',
        name='scan_merger',
        output='screen',
        parameters=[{
            'output_topic':    '/scan',
            'output_frame_id': 'custom_base_link',
            'merge_topics': [
                '/lidar_front_left/scan', '/lidar_front_right/scan',
                '/lidar_back_left/scan',  '/lidar_back_right/scan'
            ]
        }]
    )

    # ── Motor controller ────────────────────────────────────────────────────
    motor_controller = Node(
        package='robot_driver',
        executable='motor_controller_node',
        name='motor_controller',
        output='screen'
    )

    # ── Camera publisher ─────────────────────────────────────────────────────
    # FIX 2: camera_module was completely missing from the launch file.
    # The web UI MANUAL and FOLLOW panels both need /camera.
    # The detection node also subscribes to /camera.
    # executable name matches CMakeLists: video_stream_publisher
    camera_publisher = Node(
        package='camera_module',
        executable='video_stream_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_index':  0,
            'frame_width':   640,
            'frame_height':  480,
            'fps':           30,
        }]
    )

    # ── LiDAR odometry (rf2o) ────────────────────────────────────────────────
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic':       '/odom_rf2o',
            'publish_tf':       False,
            'base_frame_id':    'custom_base_link',
            'odom_frame_id':    'custom_odom',
            'init_pose_from_topic': '',
            'freq':             20.0
        }]
    )

    # ── EKF sensor fusion ────────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(system_bringup_dir, 'config', 'ekf.yaml')]
    )

    # ── Web server ───────────────────────────────────────────────────────────
    # FIX 3: jpeg_quality parameter exposed so it can be tuned without rebuild.
    web_server = Node(
        package='web_server',
        executable='web_server_node',
        name='web_server',
        output='screen',
        parameters=[{
            'port':                       8080,
            'jpeg_quality':               75,
            'auto_initialpose_from_odom': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'publish_odom_tf':            ParameterValue(LaunchConfiguration('publish_odom_tf'), value_type=bool),
        }]
    )

    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', robot_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ── Nav2 ─────────────────────────────────────────────────────────────────
    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[LaunchConfiguration('nav2_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(localization_mode)
    )

    load_nav2_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller', plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('/cmd_vel', '/cmd_vel_nav2')]
            ),
            ComposableNode(
                package='nav2_planner', plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('/cmd_vel', '/cmd_vel_nav2')]
            ),
            ComposableNode(
                package='nav2_behaviors', plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[LaunchConfiguration('nav2_params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[('/cmd_vel', '/cmd_vel_nav2')]
            ),
            ComposableNode(
                package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator',
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
                    'autostart':    ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                    'node_names':   ['controller_server', 'planner_server',
                                     'behavior_server',   'bt_navigator']
                }]
            ),
        ],
        condition=IfCondition(localization_mode)
    )

    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file'),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(localization_with_map)
    )

    amcl_node = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[LaunchConfiguration('nav2_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(localization_with_map)
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':    ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
            'node_names':   ['map_server', 'amcl']
        }],
        condition=IfCondition(localization_with_map)
    )

    slam_toolbox_mapping = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        # FIX 4: slam_toolbox.yaml has use_sim_time: true hardcoded inside it.
        # We override it here so it respects the launch argument.
        parameters=[LaunchConfiguration('slam_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'map_start_at_dock': False}],
        condition=IfCondition(mapping_mode)
    )

    map_saver_server = Node(
        package='nav2_map_server', executable='map_saver_server', name='map_saver', output='screen',
        parameters=[{
            'use_sim_time':               LaunchConfiguration('use_sim_time'),
            'save_map_timeout':           2000.0,
            'free_thresh_default':        0.25,
            'occupied_thresh_default':    0.65,
            'map_subscribe_transient_local': True
        }],
        condition=IfCondition(mapping_mode)
    )

    map_saver_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':    ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
            'node_names':   ['map_saver']
        }],
        condition=IfCondition(mapping_mode)
    )

    # ── ROSBridge ─────────────────────────────────────────────────────────────
    # FIX 5: unsubscribe_timeout added. Without it, dead web clients leave
    # subscriptions alive and the Jetson wastes bandwidth republishing
    # map + scan + odom to nobody. 10 seconds is a reasonable grace period.
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port':                  9090,
            'delay_between_messages': 0.0,
            'unsubscribe_timeout':   10.0,
        }]
    )

    # ── YOLO / detection node ─────────────────────────────────────────────────
    yolo_tracker_node = Node(
        package='human_detection_module',
        executable='detection_node',
        name='yolo_tracker',
        output='screen',
        # FIX 6: Pass calibration parameters from launch so they can be
        # tuned without recompiling. These match the parameter names declared
        # in detection_node.cpp.
        parameters=[{
            'confidence_threshold':  0.50,
            'nms_threshold':         0.45,
            'focal_length_px':       500.0,   # <-- calibrate this for your lens
            'person_real_height_m':  1.70,
            'ball_real_height_m':    0.04,
            'min_box_height_px':     20,
        }],
        condition=IfCondition(LaunchConfiguration('enable_follow'))
    )

    # ── Launch description assembly ───────────────────────────────────────────
    ld = LaunchDescription()

    for arg in [
        declare_mode_arg, declare_use_sim_time_arg, declare_rviz_arg,
        declare_autostart_arg, declare_nav2_params_arg, declare_slam_params_arg,
        declare_map_file_arg, declare_map_save_dir_arg, declare_map_save_name_arg,
        declare_slam_mode_arg, declare_gui_arg, declare_merge_scans_arg,
        declare_use_startup_delays_arg, declare_publish_odom_tf_arg,
        declare_nav_cruise_linear_scale_arg, declare_nav_cruise_angular_scale_arg,
        declare_nav_stuck_linear_scale_arg, declare_nav_stuck_angular_scale_arg,
        declare_nav_min_linear_x_arg, declare_nav_min_angular_z_arg,
        declare_nav_stuck_linear_x_arg, declare_nav_stuck_angular_z_arg,
        declare_enable_follow_arg, declare_camera_width_arg, declare_target_height_arg,
    ]:
        ld.add_action(arg)

    staged_startup   = IfCondition(LaunchConfiguration('use_startup_delays'))
    immediate_startup = UnlessCondition(LaunchConfiguration('use_startup_delays'))

    all_nodes = [
        robot_state_publisher, joint_state_publisher, joint_state_publisher_gui,
        lidar_fl, lidar_fr, lidar_br, lidar_bl,
        scan_merger, motor_controller, camera_publisher,
        rf2o_node, ekf_node,
        mode_manager, map_server_node, amcl_node, map_server_lifecycle,
        slam_toolbox_mapping, map_saver_server, map_saver_lifecycle,
        nav2_container, load_nav2_nodes,
        web_server, rosbridge_websocket, yolo_tracker_node, rviz2
    ]

    immediate_group = GroupAction(
        condition=immediate_startup,
        actions=[LogInfo(msg="[Startup] Launching all nodes immediately.")] + all_nodes
    )

    staged_group = GroupAction(
        condition=staged_startup,
        actions=[
            LogInfo(msg="[Stage 1/6] Robot state publisher..."),
            robot_state_publisher, joint_state_publisher, joint_state_publisher_gui,

            TimerAction(period=1.0,  actions=[LogInfo(msg="[Stage 2/6] Lidar FL..."),  lidar_fl]),
            TimerAction(period=2.0,  actions=[LogInfo(msg="[Stage 2/6] Lidar FR..."),  lidar_fr]),
            TimerAction(period=3.0,  actions=[LogInfo(msg="[Stage 2/6] Lidar BL..."),  lidar_bl]),
            TimerAction(period=4.0,  actions=[LogInfo(msg="[Stage 2/6] Lidar BR..."),  lidar_br]),

            TimerAction(period=5.0, actions=[
                LogInfo(msg="[Stage 2.5/6] Scan merger, motor control & camera..."),
                scan_merger, motor_controller, camera_publisher
            ]),
            TimerAction(period=5.5, actions=[
                LogInfo(msg="[Stage 2.75/6] Odometry & sensor fusion..."),
                rf2o_node, ekf_node
            ]),
            TimerAction(period=6.0, actions=[
                LogInfo(msg="[Stage 3/6] Localization / mapping..."),
                mode_manager, map_server_node, amcl_node, map_server_lifecycle,
                slam_toolbox_mapping, map_saver_server, map_saver_lifecycle,
            ]),
            TimerAction(period=7.0, actions=[
                LogInfo(msg="[Stage 4/6] Nav2 container..."),
                nav2_container,
            ]),
            TimerAction(period=8.0, actions=[
                LogInfo(msg="[Stage 5/6] Nav2 composable nodes..."),
                load_nav2_nodes,
            ]),
            TimerAction(period=9.0, actions=[
                LogInfo(msg="[Stage 6/6] Web, ROSBridge, YOLO & RViz..."),
                web_server, rosbridge_websocket, yolo_tracker_node, rviz2,
            ]),
        ]
    )

    ld.add_action(immediate_group)
    ld.add_action(staged_group)

    return ld