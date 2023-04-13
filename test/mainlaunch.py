import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import SetParameter
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    realsense_prefix = os.getcwd()
    slam_cfg = os.path.join(realsense_prefix, 'config', 'params.yaml')
    #realsense_prefix = get_package_share_directory('realsense_examples')
    cartographer_config_dir = LaunchConfiguration('file_path', default=os.path.join(realsense_prefix, 'config', 'params.yaml'))
    print(slam_cfg)
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(realsense_prefix, 'config', 'rs_cartographer.rviz')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[os.path.join(realsense_prefix, 'config', 'ukf2.yaml')],
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            namespace='camera',
            parameters=[{'enable_gyro': True, 'enable_pose': True, 'enable_accel': True, 'unite_imu_method': 2, 'pointcloud.enable': True, 'pointcloud.stream_filter': 2, 'align_depth.enable': True, 'linear_accel_cov': 1.0,
                'rgb_camera.enable_auto_exposure': False,
                'rgb_camera.exposure': 800,
                'rgb_camera.gain': 450,
                'depth_module.min_distance': 230,
                'depth_module.laser_power': 100,
                'depth_module.confidence_threshold': 3,
                'depth_module.profile': "320x240x30",
                'rgb_camera.profile': "640x360x6",
                #'initial_reset': True,
                }],
        ),
        #Node(
         #       package='drivetrain',
        #        executable='drivetrain',
        #        name='drivetrain',
        #        output='screen',
        #),
        Node(
            package='mynode',
            executable='mynode',
            name='mynode',
            output='screen',
        ),
        Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=["0.025", "0.04", "0", "0", "0", "0", "camorg", "drivetrain_link"]
        ),
        Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                name='map2odom',
                arguments=["0", "0", "0", "0", "0", "0", "/map", "/odom"]
        ),
        Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                name='camorg',
                arguments=["0", "0", "0", "0", "0.31", "-0.01", "/camorg", "/camera_link"]
        ),
        Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                name='camorg',
                arguments=["-0.1", "0", "0", "0", "0", "0", "drivetrain_link", "pedestal_stacker"]
        ),
    ])
