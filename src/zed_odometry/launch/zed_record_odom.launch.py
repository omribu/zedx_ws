from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


#############  Change here to set path  #############
PATH = "/home/volcani/zedx_ws/zed_odom_recordings/15_01_2026/1.0m_s/"




def generate_launch_description():

    # ---------------- ZED ----------------
    zed_x = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            ])
        ),
        launch_arguments={
            'camera_model': 'zedx',
        }.items()
    )

    # ---------------- RealSense ----------------
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'enable_depth': 'false',
            'enable_color': 'true',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_sync': 'false',
        }.items()
    )

    # Delay RealSense startup (CRITICAL)
    realsense_delayed = TimerAction(
        period=5.0,
        actions=[realsense]
    )

    # ---------------- Recording Nodes ----------------
    record_odom = Node(
        package="zed_odometry",
        executable="record_odometry.py",
        name="record_odometry",
        output="screen",
        parameters=[{'path_to_save': PATH}]
    )

    capture_images = Node(
        package="zed_odometry",
        executable="capture_images.py",
        name="capture_images",
        output="screen",
        parameters=[{'path_to_save': PATH}]
    )

    # ---------------- Log Info ----------------
    cameras_ready_message = TimerAction(
        period=13.0,  
        actions=[LogInfo(msg="✅ Both ZED and RealSense cameras are ready. You can start the experiment.")]
    )


    # ---------------- Launch ----------------
    return LaunchDescription([
        zed_x,               # MUST start first
        realsense_delayed,   # delayed start
        record_odom,
        capture_images,
        cameras_ready_message,
    ])
























# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import PathJoinSubstitution
# import os
# from ament_index_python.packages import get_package_share_directory

# PATH = "/home/volcani/zedx_ws/zed_odom_recordings/15_01_2026/1.0m_s/"



# def generate_launch_description():

#     realsense_delayed = TimerAction(
#         period=3.0,  # even 3s is often enough
#         actions=[realsense]
#     )

#     # zed_x = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([
#     #         PathJoinSubstitution([
#     #             FindPackageShare('zed_wrapper'),
#     #             'launch',
#     #             'zed_camera.launch.py',
#     #         ])
#     #     ]),
#     #             launch_arguments={'camera_model': 'zedx'}.items()
#     # )

#     zed_x = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         PathJoinSubstitution([
#             FindPackageShare('zed_wrapper'),
#             'launch',
#             'zed_camera.launch.py',
#         ])
#     ),
#     launch_arguments={
#         'camera_model': 'zedx',
#     }.items()
# )


#     realsense = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             PathJoinSubstitution([
#                 FindPackageShare('realsense2_camera'),
#                 'launch',
#                 'rs_launch.py',
#             ])
#         ),
#         launch_arguments={
#             'enable_depth': 'false',
#             'enable_color': 'true',
#             'enable_gyro': 'false',
#             'enable_accel': 'false',

#             'enable_sync': 'false'
#         }.items()
#     )

#     realsense_delayed = TimerAction(
#         period=3.0,  # even 3s is often enough
#         actions=[realsense]
#     )



#     zed_odom_recording = Node(
#         package="zed_odometry",
#         executable="record_zed_odom.py",
#         name="record_zed_odometry",
#         output="screen",
#     )

#     record_odom = Node(
#         package="zed_odometry",
#         executable="record_odometry.py",
#         name="record_odometry",
#         output="screen",
#         parameters=[{'path_to_save': PATH}]
#     )

#     capture_images = Node(
#         package="zed_odometry",
#         executable="capture_images.py",
#         name="capture_images",
#         output="screen",
#         parameters=[{'path_to_save': PATH}]
#     )

#     # keyboard_controller_node = Node(
#     #     package="zed_odometry",
#     #     executable="keyboard_controller.py",
#     #     name="keyboard_controller",
#     #     output="screen",
#     # )

#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="screen",
#         arguments=["-d", os.path.join(get_package_share_directory("zed_odometry"), "rviz", "display.rviz")]
#     )

#     return LaunchDescription([
#         zed_x, 
#         realsense_delayed,
#         # cleanup_zed                             # FOR expirement
#         # zed_odom_recording,    ### when conecting a screen directly to jetson 
#         record_odom,         ### when conecting remotely via ssh
#         capture_images,
#         # keyboard_controller_node,
#         # rviz_node
#     ])