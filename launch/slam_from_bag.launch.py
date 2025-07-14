#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    SetEnvironmentVariable, IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    # ────────── CLI arguments ──────────
    DeclareLaunchArgument('bag',   default_value='')
    DeclareLaunchArgument('rate',  default_value='1.0')
    DeclareLaunchArgument('clock', default_value='true')

    bag  = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')

    # ────────── config paths ──────────
    pkg     = FindPackageShare('mapping_docker')
    cfg_dir = PathJoinSubstitution([pkg, 'config'])

    slam_yaml = PathJoinSubstitution([cfg_dir, 'slam_toolbox_params.yaml'])
    ekf_yaml  = PathJoinSubstitution([cfg_dir, 'ekf.yaml'])
    rviz_cfg  = PathJoinSubstitution([cfg_dir, 'slam_setup.rviz'])
    qos_yaml = PathJoinSubstitution([cfg_dir, 'tf_qos.yaml']) 
    
    
    # ────────── 1.  rosbag (starts paused) ──────────
    bag_play = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2','bag','play', bag,
                '--clock',
                '--rate', rate,
                '--qos-profile-overrides-path', qos_yaml                 
                ],
            respawn=False,
            output='screen')]
    )

    tf_debug = ExecuteProcess(
    cmd=['ros2', 'topic', 'echo', '/tf_static'],
    output='screen'
    )
   
    delayed_bag = TimerAction(period=3.0, actions=[bag_play])


    # ────────── 2. transforms ──────────

    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll','0', '--pitch','0', '--yaw','0',
            '--frame-id',       'map',
            '--child-frame-id', 'odom'
            ],
        parameters=[{'use_sim_time': True}],
        name='tf_map_odom_boot'
    )
  
    # 1) odom_lidar → odom  (identity)
    tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x',   '0', '--y', '0', '--z', '0',
            '--roll','0', '--pitch','0', '--yaw','0',
            '--frame-id',        'odom',
            '--child-frame-id',  'base_link'
        ],
        parameters=[{'use_sim_time': True}],
        name='tf_odom_identity'
    )

    # 2) base_link → lidar  (LiDAR 15 cm above base, –25° roll)
    tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.15",
            "--roll", "-0.436332", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar"
        ],
        parameters=[{"use_sim_time": True}],
        name="tf_base_lidar"
    )


    # 3) lidar → laser  (needed by velodyne_laserscan)
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x',   '0', '--y', '0', '--z', '0',
            '--roll','0', '--pitch','0', '--yaw','0',
            '--frame-id',        'lidar',
            '--child-frame-id',  'laser'
        ],
        parameters=[{'use_sim_time': True}],
        name='tf_laser'
    )

    # ────────── 3.  Velodyne → LaserScan (after TFs ready) ──────────
    velodyne_scan = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        parameters=[{
            'queue_size': 50,
            'min_height': -0.05,
            'max_height':  0.05,
            'scan_phase': 0.0,
            'use_sim_time': True        
        }],
        remappings=[('velodyne_points','/velodyne_points'),
                    ('scan','/scan')],
        output='screen')

    delayed_scan = TimerAction(period=5.0,actions=[velodyne_scan])

    
    
        # ────────── 4.  EKF + SLAM ──────────
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_yaml,
                    {'buffer_duration_sec':20.0, 'use_sim_time':True}],
        output='screen')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('slam_toolbox'),
             '/launch/online_async_launch.py']),
        
        launch_arguments={
            'slam_params_file':slam_yaml,
            'use_sim_time':'true',
            'start_new_map':'true'
        }.items())

    delayed_slam = TimerAction(period=6.0,actions=[slam])



    # Forward the odom data:
    odom_relay = ExecuteProcess(
        cmd=['ros2','topic','relay',
            '/kiss/odometry','/odom',
            '--qos-reliability','reliable'],
        output='screen')


    # ────────── 5.  RViz ──────────
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        additional_env={'OGRE_GL_STATE_CACHE_SUPPORT': '0'},
        output='screen'
    )

    return LaunchDescription([

        tf_map_odom,            
        tf_odom_base,      
        tf_base_lidar,         
        tf_laser,   

    #     odom_relay,          
     
     
        delayed_bag,
        delayed_scan,          # TimerAction to start velodyne_laserscan_node after statics are latched
        delayed_slam,
       
    #     ekf,
        rviz
    ])