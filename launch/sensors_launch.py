# ragnarhorn_autonomy/launch/all_sensors_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Microstrain IMU launch
    microstrain_share = FindPackageShare('microstrain_inertial_driver').find('microstrain_inertial_driver')
    microstrain_launch = os.path.join(microstrain_share, 'launch', 'microstrain_launch.py')

    # Livox LiDAR launch
    livox_share = FindPackageShare('livox_ros_driver2').find('livox_ros_driver2')
    livox_launch = os.path.join(livox_share, 'launch', 'rviz_MID360_launch.py')

    # Static transform publisher node for lidar
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
        output='screen'
    )

    # DLIO SLAM launch
    dlio_share = FindPackageShare('direct_lidar_inertial_odometry').find('direct_lidar_inertial_odometry')
    dlio_launch = os.path.join(dlio_share, 'launch', 'dlio.launch.py')

    # Telemtry node
    telem_node = Node(
        package='ragnarhorn_autonomy',
        executable='telem_pub',
        name='telemetry'
    )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(microstrain_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch)
        ),

        static_tf_node,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dlio_launch),
            launch_arguments={
                'rviz': 'false',
                'pointcloud_topic': '/livox/points',
                'imu_topic': '/livox/imu'
            }.items()
        ),

        telem_node,
        
    ])
