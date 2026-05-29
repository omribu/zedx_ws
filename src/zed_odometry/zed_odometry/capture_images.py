#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np


class CaptureImg(Node):
    def __init__(self):
        super().__init__('capture_images')

        self.declare_parameter('path_to_save', '/home/volcani/zedx_ws/zed_odom_recordings/5_5_2026/')
        self.save_images = self.get_parameter('path_to_save').get_parameter_value().string_value

        self.bag_path_sub = self.create_subscription(String, '/current_bag_path', self.bag_path_callback, 10)

        # Subscribe to realsense camera
        self.image_sub_ = self.create_subscription(
            Image, 
            '/camera/zed/color/image_raw', 
            self.image_callback, 
            10
        )
        
        # Services to start/stop recording
        self.start_recording_srv = self.create_service(
            Trigger, 
            'capture_start_image', 
            self.start_capture_img_callback
        )
        self.stop_recording_srv = self.create_service(
            Trigger, 
            'capture_stop_image', 
            self.stop_capture_img_callback
        )

        self.image = None
        
        self.get_logger().info('Image capture node initialized')
        self.get_logger().info(f'Images will be saved to: {self.save_images}')

    def _imgmsg_to_cv2(self, msg):
        channels = len(msg.data) // (msg.height * msg.width)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
        if msg.encoding in ('rgb8', 'RGB8'):
            img = img[:, :, ::-1].copy()
        return img

    def bag_path_callback(self, msg):
        if msg.data:
            self.save_images = msg.data
            self.get_logger().info(f'Image save path updated to: {self.save_images}')

    def image_callback(self, msg):
        # Store the latest image
        self.image = msg

        cv_image = self._imgmsg_to_cv2(self.image)

        # cv2.imshow('RealSense Camera', cv_image)
        cv2.waitKey(1)


    def start_capture_img_callback(self, request, response):
        if self.image is None:
            self.get_logger().warn('No image available yet')
            response.success = False
            response.message = "No image data available"
            return response
            
        try:
            cv_image = self._imgmsg_to_cv2(self.image)
            image_path = os.path.join(self.save_images, 'start.png')
            os.makedirs(self.save_images, exist_ok=True)
            cv2.imwrite(image_path, cv_image)
            self.get_logger().info(f"Start image saved to: {image_path}")
            response.success = True
            response.message = f"Image saved to {image_path}"
        except Exception as e:
            self.get_logger().error(f"Failed to save start image: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response

    def stop_capture_img_callback(self, request, response):
        if self.image is None:
            self.get_logger().warn('No image available yet')
            response.success = False
            response.message = "No image data available"
            return response
            
        try:
            cv_image = self._imgmsg_to_cv2(self.image)
            image_path = os.path.join(self.save_images, 'stop.png')
            os.makedirs(self.save_images, exist_ok=True)
            cv2.imwrite(image_path, cv_image)
            self.get_logger().info(f"Stop image saved to: {image_path}")
            response.success = True
            response.message = f"Image saved to {image_path}"
        except Exception as e:
            self.get_logger().error(f"Failed to save stop image: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CaptureImg()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()









































