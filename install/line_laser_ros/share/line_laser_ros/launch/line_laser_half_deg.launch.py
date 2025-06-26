from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    frame_id = LaunchConfiguration('frame_id')
    version_num = LaunchConfiguration('version_num')

    # Define the package share directory
    pkg_share = get_package_share_directory('line_laser_ros')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the LiDAR device'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for the LiDAR serial communication'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_laser_link',
            description='Frame ID for the LaserScan messages'
        ),
        DeclareLaunchArgument(
            'version_num',
            default_value='1',
            description='Version number: 0 for n301 TOF, 1 for line laser 1 deg, 2 for line laser 0.5 deg'
        ),

        # Node for the LiDAR driver
        Node(
            package='line_laser_ros',
            executable='n301n_serial_publisher',
            name='line_laser',
            output='screen',
            parameters=[
                {'port': port},
                {'baud_rate': baud_rate},
                {'frame_id': frame_id},
                {'version_num': version_num}
            ]
        ),

        # Node for RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'launch', 'laser.rviz')]
        )
    ])