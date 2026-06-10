from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


# ================================================== #
#############  Change here to set path  #############
# ================================================== #
EXP_TYPE = "" #"1.0m_s/with_features/"  
DATE = "3_6_26"
PATH = "/home/volcani/workspaces/zedx_ws/zed_odom_recordings/" + DATE + "/" + EXP_TYPE + "/"





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
            'publish_imu_tf': 'true',
            'ros_params_override_path': os.path.join(
                get_package_share_directory('zed_odometry'),
                'config',
                'zedx_outdoor_vio.yaml'
            ),
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


    # Delay recording nodes until ZED has had time to start publishing odom.
    # Without this, on experiment 3+ the nodes spin up before odom is live.
    
    record_odom_delayed = TimerAction(
        period=15.0,
        actions=[record_odom, capture_images]
    )

    # ---------------- Log Info ----------------
    cameras_ready_message = TimerAction(
        period=18.0,  
        actions=[LogInfo(msg="✅ Both ZED and RealSense cameras are ready. You can start the experiment.")]
    )


    # ---------------- Launch ----------------
    return LaunchDescription([
        zed_x,                  # t=0s
        realsense_delayed,      # t=5s
        record_odom_delayed,    # t=15s  ← this replaces the two individual entries
        cameras_ready_message,  # t=18s
    ])


















