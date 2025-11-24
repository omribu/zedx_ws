from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    zed_x = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            ])
        ]),
                launch_arguments={'camera_model': 'zedx'}.items()
    )

    zed_odom_recording = Node(
        package="zed_odometry",
        executable="record_zed_odom.py",
        name="record_zed_odometry",
        output="screen",
    )

    # keyboard_controller_node = Node(
    #     package="zed_odometry",
    #     executable="keyboard_controller.py",
    #     name="keyboard_controller",
    #     output="screen",
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("zed_odometry"), "rviz", "display.rviz")]
    )

    return LaunchDescription([
        zed_x,
        zed_odom_recording,
        # keyboard_controller_node,
        rviz_node
    ])