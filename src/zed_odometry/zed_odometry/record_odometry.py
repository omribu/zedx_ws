#!/usr/bin/env python3

import os
import subprocess
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge

class ZedOdomRecorder(Node):
    def __init__(self):
        super().__init__('record_odometry')

        
        self.declare_parameter('path_to_save', '/home/volcani/zedx_ws/zed_odom_recordings/15_01_2026/1.5m_s/')
        self.save_recording = self.get_parameter('path_to_save').get_parameter_value().string_value

        # self.save_recording = "/home/volcani/zedx_ws/zed_odom_recordings/15_01_2026/1.5m_s/"
        self.bridge = CvBridge()
        
       
        # Subscribe to zed odometry
        self.odom_sub = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)


        # Services to start/stop recording
        self.start_recording_srv = self.create_service(Trigger, 'start_recording', self.start_recording_callback)
        self.stop_recording_srv = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)
 

        self.image = None

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
        pass
    def image_callback(self, msg):
        # You can add processing here if needed, for now just pass
        self.image = msg

    def start_recording(self):
        if self.is_recording:
            self.get_logger().warn("Recording already in progress")
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
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedOdomRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






















# #!/usr/bin/env python3

# import os
# import subprocess
# import signal
# import time
# from datetime import datetime
# import rclpy
# from rclpy.node import Node
# from rclpy.callback_groups import ReentrantCallbackGroup
# from std_srvs.srv import Trigger
# from nav_msgs.msg import Odometry

# class ZedOdomRecorder(Node):
#     def __init__(self):
#         super().__init__('zed_odom_recorder')
        
#         # Use reentrant callback group for non-blocking service calls
#         self.callback_group = ReentrantCallbackGroup()

#         self.odom_sub = self.create_subscription(
#             Odometry,
#             '/zed/zed_node/odom',
#             self.odom_callback,
#             10,
#             callback_group=self.callback_group
#         )

#         self.start_recording_srv = self.create_service(
#             Trigger,
#             'start_recording',
#             self.start_recording_callback,
#             callback_group=self.callback_group
#         )
#         self.stop_recording_srv = self.create_service(
#             Trigger,
#             'stop_recording',
#             self.stop_recording_callback,
#             callback_group=self.callback_group
#         )

#         self.recording_process = None
#         self.is_recording = False
#         self.current_bag_path = None
#         self.bag_directory = os.path.expanduser(
#             "/home/volcani/zedx_ws/zed_odom_recordings/15_01_2026/0.5m_s"
#         )
#         os.makedirs(self.bag_directory, exist_ok=True)

#         self.odom_received = False
#         self.odom_count = 0

#         self.get_logger().info('ZED Odometry recorder initialized')
#         self.get_logger().info(f'Recordings will be saved to: {self.bag_directory}')

#     def odom_callback(self, msg):
#         if not self.odom_received:
#             self.odom_received = True
#             self.get_logger().info("✓ Odometry topic is active")
#         self.odom_count += 1

#     def start_recording(self):
#         if self.is_recording:
#             self.get_logger().warn("Recording already in progress")
#             return False, "Recording already in progress"

#         if not self.odom_received:
#             self.get_logger().error("No odometry data received yet!")
#             return False, "No odometry data received yet. Is the ZED camera running?"

#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         bag_name = f"zed_odom_{timestamp}"
#         self.current_bag_path = os.path.join(self.bag_directory, bag_name)

#         try:
#             self.get_logger().info(f"Starting recording to: {self.current_bag_path}")
            
#             # Open devnull for stdout/stderr
#             devnull = open(os.devnull, 'w')
            
#             # Start the process with explicit environment
#             env = os.environ.copy()
            
#             self.recording_process = subprocess.Popen(
#                 ['ros2', 'bag', 'record', 
#                  '-o', self.current_bag_path, 
#                  '/zed/zed_node/odom'],
#                 stdout=devnull,
#                 stderr=devnull,
#                 env=env,
#                 start_new_session=True,
#                 preexec_fn=os.setpgrp  # Create new process group
#             )
            
#             # Give it a moment to initialize
#             time.sleep(0.3)
            
#             # Check if process started successfully
#             poll_result = self.recording_process.poll()
#             if poll_result is not None:
#                 self.get_logger().error(f"Recording process failed immediately with code: {poll_result}")
#                 return False, f"Failed to start recording (exit code: {poll_result})"
            
#             self.is_recording = True
#             self.get_logger().info(f"🔴 RECORDING STARTED: {self.current_bag_path}")
#             return True, f"Recording started: {bag_name}"
            
#         except Exception as e:
#             error_msg = f"Failed to start recording: {str(e)}"
#             self.get_logger().error(error_msg)
#             return False, error_msg

#     def stop_recording(self):
#         if not self.is_recording:
#             self.get_logger().warn("No recording in progress")
#             return False, "No recording in progress"

#         try:
#             if self.recording_process:
#                 self.get_logger().info("Stopping recording...")
                
#                 # Try graceful shutdown first
#                 self.recording_process.send_signal(signal.SIGINT)
                
#                 try:
#                     self.recording_process.wait(timeout=5)
#                     self.get_logger().info("Recording stopped gracefully")
#                 except subprocess.TimeoutExpired:
#                     self.get_logger().warn("Process didn't stop gracefully, forcing...")
#                     self.recording_process.kill()
#                     self.recording_process.wait()
                
#                 self.recording_process = None

#             self.is_recording = False
#             msg = f"⏹️  RECORDING STOPPED: {self.current_bag_path}"
#             self.get_logger().info(msg)
#             self.get_logger().info(f"Total odometry messages received: {self.odom_count}")
#             return True, f"Recording stopped: {self.current_bag_path}"
            
#         except Exception as e:
#             error_msg = f"Failed to stop: {str(e)}"
#             self.get_logger().error(error_msg)
#             return False, error_msg

#     def start_recording_callback(self, request, response):
#         self.get_logger().info("=== SERVICE CALL RECEIVED ===")
#         self.get_logger().info(f"Odometry received: {self.odom_received}")
#         self.get_logger().info(f"Currently recording: {self.is_recording}")
        
#         if self.is_recording:
#             response.success = False
#             response.message = "Already recording"
#             self.get_logger().info("=== RETURNING: Already recording ===")
#             return response
        
#         self.get_logger().info("=== ABOUT TO START RECORDING ===")
#         success, message = self.start_recording()
        
#         response.success = success
#         response.message = message
#         self.get_logger().info(f"=== RETURNING: {message} ===")
#         return response

#     def stop_recording_callback(self, request, response):
#         self.get_logger().info("Received stop_recording service call")
#         success, message = self.stop_recording()
#         response.success = success
#         response.message = message
#         return response

#     def destroy_node(self):
#         if self.is_recording:
#             self.get_logger().info("Node shutting down, stopping recording...")
#             self.stop_recording()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
    
#     # Use MultiThreadedExecutor for non-blocking service calls
#     from rclpy.executors import MultiThreadedExecutor
#     executor = MultiThreadedExecutor()
    
#     node = ZedOdomRecorder()
#     executor.add_node(node)
    
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

























# #!/usr/bin/env python3

# import os
# import subprocess
# import signal
# import threading
# from datetime import datetime
# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Trigger
# from nav_msgs.msg import Odometry

# class ZedOdomRecorder(Node):
#     def __init__(self):
#         super().__init__('zed_odom_recorder')

#         self.odom_sub = self.create_subscription(
#             Odometry,
#             '/zed/zed_node/odom',
#             self.odom_callback,
#             10
#         )

#         self.start_recording_srv = self.create_service(
#             Trigger,
#             'start_recording',
#             self.start_recording_callback
#         )
#         self.stop_recording_srv = self.create_service(
#             Trigger,
#             'stop_recording',
#             self.stop_recording_callback
#         )

#         self.recording_process = None
#         self.is_recording = False
#         self.current_bag_path = None
#         self.bag_directory = os.path.expanduser(
#             "/home/volcani/zedx_ws/zed_odom_recordings/15_01_2026/0.5m_s"
#         )
#         os.makedirs(self.bag_directory, exist_ok=True)

#         self.odom_received = False

#         self.get_logger().info('ZED Odometry recorder initialized')
#         self.get_logger().info(f'Recordings will be saved to: {self.bag_directory}')

#     def odom_callback(self, msg):
#         if not self.odom_received:
#             self.odom_received = True
#             self.get_logger().info("✓ Odometry topic is active")

#     def start_recording_thread(self):
#         """Run the recording process in a separate thread"""
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         bag_name = f"zed_odom_{timestamp}"
#         self.current_bag_path = os.path.join(self.bag_directory, bag_name)

#         try:
#             self.recording_process = subprocess.Popen(
#                 ['ros2', 'bag', 'record', '-o', self.current_bag_path, '/zed/zed_node/odom'],
#                 stdout=subprocess.DEVNULL,
#                 stderr=subprocess.DEVNULL,
#                 start_new_session=True
#             )
#             self.get_logger().info(f"🔴 RECORDING STARTED: {self.current_bag_path}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to start recording: {str(e)}")
#             self.is_recording = False

#     def start_recording(self):
#         if self.is_recording:
#             return False, "Recording already in progress"

#         if not self.odom_received:
#             return False, "No odometry data received yet. Is the ZED camera running?"

#         self.is_recording = True
        
#         # Start recording in a separate thread to avoid blocking
#         thread = threading.Thread(target=self.start_recording_thread, daemon=True)
#         thread.start()
        
#         return True, "Recording started"

#     def stop_recording(self):
#         if not self.is_recording:
#             return False, "No recording in progress"

#         try:
#             if self.recording_process:
#                 self.recording_process.send_signal(signal.SIGINT)
#                 try:
#                     self.recording_process.wait(timeout=5)
#                 except subprocess.TimeoutExpired:
#                     self.recording_process.kill()
#                     self.recording_process.wait()
                
#                 self.recording_process = None

#             self.is_recording = False
#             self.get_logger().info(f"⏹️  RECORDING STOPPED: {self.current_bag_path}")
#             return True, "Recording stopped"
#         except Exception as e:
#             return False, f"Failed to stop: {str(e)}"

#     def start_recording_callback(self, request, response):
#         success, message = self.start_recording()
#         response.success = success
#         response.message = message
#         return response

#     def stop_recording_callback(self, request, response):
#         success, message = self.stop_recording()
#         response.success = success
#         response.message = message
#         return response

#     def destroy_node(self):
#         if self.is_recording:
#             self.stop_recording()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = ZedOdomRecorder()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()







































