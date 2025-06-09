
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare   

from launch.substitutions import PathJoinSubstitution



def generate_launch_description():

    pkg_share = FindPackageShare('mapping_docker')
    cfg_dir   = PathJoinSubstitution([pkg_share, 'config'])

    ekf_yaml  = PathJoinSubstitution([cfg_dir, 'ekf.yaml'])
    slam_yaml = PathJoinSubstitution([cfg_dir, 'slam_toolbox_params.yaml'])
    rviz_cfg  = PathJoinSubstitution([cfg_dir, 'slam_setup.rviz'])

    # ─────────────  RealSense T265  ─────────────
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='',                  
        name='t265',
        output='screen',
        parameters=[{
            # enable pose stream
            'enable_pose':        True,

            # ask the driver to publish odom→pose TF
            'publish_odom_tf':    False,

            'base_frame_id':      'base_link',   # physical IMU frame
            'pose_frame_id':      't265_pose',   
            'odom_frame_id':      'odom'
        }]
    )


    # ─────────────  Hokuyo lidar  ─────────────
    urg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('urg_node2'), 'launch', 'urg_node2.launch.py']
            )
        ),
        launch_arguments={'serial_port': '/dev/ttyACM0'}.items()
    )

    # ─────────────  Static TFs  ─────────────
	
    tf_base_to_t265 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--static',              
            '0', '0', '0',           # xyz
            '0', '0', '0', '1',      # qx qy qz qw
            'base_link',             # parent  (source)
            't265_pose'              # child   (target)
        ],
        output='screen'
        )
        
    tf_base_laser = Node(
	    package='tf2_ros',
	    executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--static',
            '0', '0', '0',  
            '0', '0', '0', '1',
            'base_link', 'laser'
        ])

    # ─────────────  EKF  ─────────────
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml]
    )

    # ─────────────  SLAM Toolbox  ─────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'),
                 'launch', 'online_async_launch.py']
            )
        ),
        launch_arguments={'slam_params_file': slam_yaml}.items()
    )

    # ─────────────  RViz  ─────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return LaunchDescription([
        realsense,
        urg,
        tf_pose_base,
        tf_base_laser,
        ekf,
        slam,
        rviz
    ])

