#!/usr/bin/env python3

import os
import subprocess
from datetime import datetime
import rclpy
from rclpy.node import Node
import std_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ZedOdom(Node):
    def __init__(self):
        super().__init__('record_odom')

        self.bridge = CvBridge()

        self.odom_zed_sub_ = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odometry_callback, 10)

        self.image_sub = self.create_subscription(
            Image, '/zed/zed_node/rgb/color/rect/image', self.image_callback, 10)


        self.path_pub_ = self.create_publisher(Path, "zed_path", 10)

        self.path = Path()
        self.path.header.frame_id = "odom" 

        # Services to start/stop recording
        self.start_recording_srv = self.create_service(
            Trigger, 
            'start_recording', 
            self.start_recording_callback
        )
        self.stop_recording_srv = self.create_service(
            Trigger, 
            'stop_recording', 
            self.stop_recording_callback
        )

        # Recording state
        self.recording_process = None
        self.is_recording = False
        self.bag_directory = os.path.expanduser("/home/volcani/zedx_ws/zed_odom_recordings")
        
        # Create directory if it doesn't exist
        os.makedirs(self.bag_directory, exist_ok=True)

        self.get_logger().info('ZED Odometry recorder initialized')
        self.get_logger().info(f'Recordings will be saved to: {self.bag_directory}')
        self.get_logger().info("Press 'r' to start/stop recording, 'q' to quit")


    def odometry_callback(self, msg):
        
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(pose_stamped)

        self.path_pub_.publish(self.path)


    def image_callback(self, msg):
        """Display camera feed and handle keyboard input"""
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Add recording indicator if recording
            if self.is_recording:
                cv2.circle(frame, (30, 30), 15, (0, 0, 255), -1)  # Red circle
                cv2.putText(frame, "REC", (60, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Display frame
            cv2.imshow("ZED Camera - Odometry Recording", frame)
            
            # Handle key press
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('r'):
                if self.is_recording:
                    self.stop_recording()
                else:
                    self.start_recording()
            elif key == ord('q'):
                self.get_logger().info("Quitting...")
                if self.is_recording:
                    self.stop_recording()
                rclpy.shutdown()
                
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def start_recording(self):
        """Start rosbag recording"""
        if self.is_recording:
            self.get_logger().warn("Recording is already in progress")
            return

        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f"zed_odom_{timestamp}"
            bag_path = os.path.join(self.bag_directory, bag_name)

            self.recording_process = subprocess.Popen([
                'ros2', 'bag', 'record',
                '-o', bag_path,
                '/zed/zed_node/odom',
                '/zed_path',
            ])

            self.is_recording = True
            self.get_logger().info(f"🔴 RECORDING STARTED: {bag_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {str(e)}")

    def stop_recording(self):
        """Stop rosbag recording"""
        if not self.is_recording:
            self.get_logger().warn("No recording in progress")
            return

        try:
            if self.recording_process:
                self.recording_process.terminate()
                self.recording_process.wait(timeout=5)
                self.recording_process = None

            self.is_recording = False
            self.get_logger().info("⏹️  RECORDING STOPPED")

        except Exception as e:
            self.get_logger().error(f"Failed to stop recording: {str(e)}")

    def start_recording_callback(self, request, response):
        self.start_recording()
        response.success = True
        response.message = "Recording started"
        return response

    def stop_recording_callback(self, request, response):
        self.stop_recording()
        response.success = True
        response.message = "Recording stopped"
        return response

    def destroy_node(self):
        if self.is_recording and self.recording_process:
            self.get_logger().info("Stopping recording before shutdown...")
            self.recording_process.terminate()
            self.recording_process.wait(timeout=5)
        cv2.destroyAllWindows()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = ZedOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()




