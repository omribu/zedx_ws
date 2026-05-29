#!/usr/bin/env python3

import os
import time
from builtin_interfaces.msg import Time
import subprocess
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


class ZedOdomRecorder(Node):
    def __init__(self):
        super().__init__('record_odometry')

        
        self.declare_parameter('path_to_save', '/home/volcani/workspaces/zedx_ws/zed_odom_recordings/5_5_2026/') 
        self.save_recording = self.get_parameter('path_to_save').get_parameter_value().string_value


       
        # Subscribe to zed odometry
        self.odom_sub = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)
        

        # Services to start/stop recording
        self.start_recording_srv = self.create_service(Trigger, 'start_recording', self.start_recording_callback)
        self.stop_recording_srv = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)

        self.bag_path_pub = self.create_publisher(String, '/current_bag_path', 10)
 

        self.image = None
        self.last_odom_time = None

        # Recording state
        self.recording_process = None
        self.is_recording = False
        self.bag_directory = os.path.expanduser(
            self.save_recording                      #"/home/volcani/zedx_ws/zed_odom_recordings/15_01_2026/1.5m_s/"
        )
        os.makedirs(self.bag_directory, exist_ok=True)

        self.get_logger().info('ZED Odometry recorder initialized')
        self.get_logger().info(f'Recordings will be saved to: {self.bag_directory}')
        self.get_logger().info("Use the services 'start_recording' and 'stop_recording'")

    def odom_callback(self, msg):
        # You can add processing here if needed, for now just pass
        self.last_odom_time = time.time()

    def image_callback(self, msg):
        # You can add processing here if needed, for now just pass
        self.image = msg

    def start_recording(self):
        if self.is_recording:
            self.get_logger().warn("Recording already in progress")
            return
        
        if self.last_odom_time is None:
            self.get_logger().error("❌ /zed/zed_node/odom has NEVER been received. Is ZED running?")
            return

        age = time.time() - self.last_odom_time
        if age > 2.0:  # 2 second threshold
            self.get_logger().error(
                f"❌ Last odom message was {age:.1f}s ago. "
                "ZED may not be ready yet. Aborting recording."
            )
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"zed_odom_{timestamp}"
        bag_path = os.path.join(self.bag_directory, bag_name)

        try:
            self.recording_process = subprocess.Popen([
                'ros2', 'bag', 'record',
                '-o', bag_path,
                '/zed/zed_node/odom'
            ])
            self.is_recording = True
            msg = String()
            msg.data = bag_path
            self.bag_path_pub.publish(msg)
            self.get_logger().info(f"🔴 RECORDING STARTED: {bag_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {str(e)}")

    def stop_recording(self):
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
        if self.last_odom_time is None or (time.time() - self.last_odom_time) > 2.0:
            response.success = False
            response.message = "Odom topic not live — ZED not ready"
            self.get_logger().error(response.message)
            return response

        self.start_recording()
        response.success = self.is_recording  # reflects actual state
        response.message = "Recording started" if self.is_recording else "Failed to start"
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
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedOdomRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_recording and node.recording_process:
            node.get_logger().info("Ctrl+C caught — stopping recording...")
            node.recording_process.terminate()
            node.recording_process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()























































































































































# #!/usr/bin/env python3

# import os
# import subprocess
# from datetime import datetime
# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Trigger
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry


# class ZedOdomRecorder(Node):
#     def __init__(self):
#         super().__init__('record_odometry')

        
#         self.declare_parameter('path_to_save', '/home/volcani/workspaces/zedx_ws/zed_odom_recordings/30_03_26_fk/1.0m_s/') 
#         self.save_recording = self.get_parameter('path_to_save').get_parameter_value().string_value


       
#         # Subscribe to zed odometry
#         self.odom_sub = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)
        

#         # Services to start/stop recording
#         self.start_recording_srv = self.create_service(Trigger, 'start_recording', self.start_recording_callback)
#         self.stop_recording_srv = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)
 

#         self.image = None

#         # Recording state
#         self.recording_process = None
#         self.is_recording = False
#         self.bag_directory = os.path.expanduser(
#             self.save_recording                      #"/home/volcani/zedx_ws/zed_odom_recordings/15_01_2026/1.5m_s/"
#         )
#         os.makedirs(self.bag_directory, exist_ok=True)

#         self.get_logger().info('ZED Odometry recorder initialized')
#         self.get_logger().info(f'Recordings will be saved to: {self.bag_directory}')
#         self.get_logger().info("Use the services 'start_recording' and 'stop_recording'")

#     def odom_callback(self, msg):
#         # You can add processing here if needed, for now just pass
#         pass
#     def image_callback(self, msg):
#         # You can add processing here if needed, for now just pass
#         self.image = msg

#     def start_recording(self):
#         if self.is_recording:
#             self.get_logger().warn("Recording already in progress")
#             return

#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         bag_name = f"zed_odom_{timestamp}"
#         bag_path = os.path.join(self.bag_directory, bag_name)

#         try:
#             self.recording_process = subprocess.Popen([
#                 'ros2', 'bag', 'record',
#                 '-o', bag_path,
#                 '/zed/zed_node/odom'
#             ])
#             self.is_recording = True
#             self.get_logger().info(f"🔴 RECORDING STARTED: {bag_path}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to start recording: {str(e)}")

#     def stop_recording(self):
#         if not self.is_recording:
#             self.get_logger().warn("No recording in progress")
#             return
        
#         try:
#             if self.recording_process:
#                 self.recording_process.terminate()
#                 self.recording_process.wait(timeout=5)
#                 self.recording_process = None

#             self.is_recording = False
#             self.get_logger().info("⏹️  RECORDING STOPPED")
#         except Exception as e:
#             self.get_logger().error(f"Failed to stop recording: {str(e)}")

#     def start_recording_callback(self, request, response):
        
#         self.start_recording()
#         response.success = True
#         response.message = "Recording started"
#         return response

#     def stop_recording_callback(self, request, response):
#         self.stop_recording()
#         response.success = True
#         response.message = "Recording stopped"
#         return response

#     def destroy_node(self):
#         if self.is_recording and self.recording_process:
#             self.get_logger().info("Stopping recording before shutdown...")
#             self.recording_process.terminate()
#             self.recording_process.wait(timeout=5)
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = ZedOdomRecorder()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






























































